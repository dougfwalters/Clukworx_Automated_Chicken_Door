#include <WiFi.h>
#include <time.h>
#include <Preferences.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <AccelStepper.h>
#include "secrets.h"
#include <ArduinoOTA.h>
#include <ESPmDNS.h>

// ============================================================
// ===================== USER CONFIG ===========================
// ============================================================

// ---------------- WiFi / Time ----------------
static const char* WIFI_SSID = SECRET_SSID;
static const char* WIFI_PASS = SECRET_PASS;
static const char* TZ_INFO   = "CST6CDT,M3.2.0/2,M11.1.0/2";
static const char* NTP1      = "pool.ntp.org";
static const char* NTP2      = "time.nist.gov";
static const uint32_t WEEKLY_SYNC_SEC = 7UL * 24UL * 3600UL;

// ---------------- OTA / Telnet ----------------
// --- NEW --- Keep these easy to disable while debugging performance.
static const bool ENABLE_OTA_FEATURE   = false;
static const bool ENABLE_TELNET_LOGGER = false;
static const bool ENABLE_DEBUG_LOGS    = true;

static const char* OTA_HOSTNAME = "coop-door";
static const char* OTA_PASSWORD = "000";
static const uint16_t TELNET_PORT = 23;

// ---------------- Schedule ----------------
static const int OPEN_HOUR    = 5;
static const int OPEN_MINUTE  = 0;
static const int CLOSE_HOUR   = 22;
static const int CLOSE_MINUTE = 0;
static const uint32_t SCREEN_ACTIVE_WINDOW_MIN = 5;

// ---------------- Display ----------------
static const int PIN_TFT_BL = 27;
static const int BL_PWM_FREQ = 5000;
static const int BL_PWM_BITS = 8;
static const uint8_t BL_DIMMED = 26;
static const uint8_t BL_FULL   = 255;

// ---------------- Stepper / DRV8825 ----------------
static const int PIN_STEP = 18;
static const int PIN_DIR  = 19;
static const int PIN_EN   = 23;   // active LOW

// Motion
// --- NEW --- Use realistic values, not crazy-high until system is proven stable.
static const float STEPPER_MAX_SPEED = 500.0f;
static const float STEPPER_ACCEL     = 900.0f;
static const uint16_t STEP_PULSE_US  = 3;

// Hard programmed travel limits
static const long CLOSED_POSITION_STEPS_DEFAULT = 0;
static const long OPEN_POSITION_STEPS_DEFAULT   = 3050;
static const long MANUAL_JOG_STEPS              = 100;

// ---------------- HC-SR04 ----------------
static const int PIN_TRIG = 25;
static const int PIN_ECHO = 32; // must be level shifted to 3.3V
static const uint32_t SONAR_TIMEOUT_US = 25000;
static const float OBSTACLE_THRESHOLD_CM_DEFAULT = 20.0f;

// ---------------- Retry behavior ----------------
static const uint32_t RETRY_DELAY_MS = 5000;
static const uint8_t  OBSTACLE_HIT_CONFIRM_COUNT = 2;

// ---------------- UI timing ----------------
static const uint32_t UI_REFRESH_MS        = 100;
static const uint32_t SONAR_POLL_MS = 60;   // same sonar rate always
static const uint32_t TOUCH_POLL_MS        = 50;   // --- NEW ---
static const uint32_t TOUCH_WAKE_MS        = 2UL * 60UL * 1000UL;

// ============================================================
// ====================== GLOBALS ==============================
// ============================================================

TFT_eSPI tft = TFT_eSPI();
Preferences prefs;
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);

WiFiServer telnetServer(TELNET_PORT);
WiFiClient telnetClient;
bool telnetWasConnected = false;

// --- NEW --- Cross-core protection for shared state / stepper access.
portMUX_TYPE stepperMux = portMUX_INITIALIZER_UNLOCKED;

// --- NEW --- Dedicated motor task on Core 0.
TaskHandle_t motorTaskHandle = nullptr;

// --- NEW --- Loop-rate counters.
volatile uint32_t motorLoopCounter = 0;
uint32_t uiLoopCounter = 0;
uint32_t lastLoopReportMs = 0;

//--Counter for door obstruction validation
uint32_t obstacleStartMs = 0;

// ---------------- States ----------------
enum DoorState {
  DOOR_BOOTING,
  DOOR_STARTUP_SYNC_TIME,
  DOOR_IDLE_OPEN,
  DOOR_IDLE_CLOSED,
  DOOR_OPENING,
  DOOR_CLOSING,
  DOOR_REOPENING_AFTER_OBSTACLE,
  DOOR_WAITING_RETRY,
  DOOR_MANUAL,
  DOOR_ERROR
};

enum MotionReason {
  REASON_NONE,
  REASON_SCHEDULE_OPEN,
  REASON_SCHEDULE_CLOSE,
  REASON_MANUAL_OPEN,
  REASON_MANUAL_CLOSE,
  REASON_MANUAL_ZERO_CLOSED,
  REASON_OBSTACLE_REOPEN
};

enum SyncState {
  SYNC_IDLE,
  SYNC_CONNECTING,
  SYNC_WAITING_NTP,
  SYNC_DISCONNECTING
};

volatile DoorState doorState = DOOR_BOOTING;
volatile MotionReason motionReason = REASON_NONE;
volatile SyncState syncState = SYNC_IDLE;

// ---------------- Runtime / Settings ----------------
bool autoMode = true;
bool touchWasDown = false;

long openPositionSteps   = OPEN_POSITION_STEPS_DEFAULT;
long closedPositionSteps = CLOSED_POSITION_STEPS_DEFAULT;
float obstacleThresholdCm = OBSTACLE_THRESHOLD_CM_DEFAULT;

volatile float lastDistanceCm = -1.0f;
volatile uint8_t obstacleHitCounter = 0;

time_t lastSuccessfulSyncEpoch = 0;
uint32_t retryAtMs = 0;
uint32_t touchWakeUntilMs = 0;
uint32_t motorActiveUntilMs = 0;

int lastHandledMinute = -1;
int lastHandledHour   = -1;
int lastHandledYDay   = -1;

uint32_t openCycleCount = 0;
uint32_t closeCycleCount = 0;
uint32_t zeroClosedCount = 0;

// --- NEW --- Motion completion flag raised by motor task, handled by UI task.
volatile bool motionCompleteFlag = false;

// Sonar state
enum SonarState {
  SONAR_IDLE,
  SONAR_TRIG_LOW,
  SONAR_TRIG_HIGH,
  SONAR_WAIT_RISE,
  SONAR_WAIT_FALL
};

SonarState sonarState = SONAR_IDLE;
uint32_t sonarStateAtUs = 0;
uint32_t sonarEchoRiseUs = 0;
uint32_t lastSonarKickMs = 0;

// UI timing
uint32_t lastUiDrawMs = 0;
uint32_t syncStartedMs = 0;

// Backlight
uint8_t currentBacklightLevel = 255;

// ---------------- Colors ----------------
uint16_t C_BG, C_PANEL, C_GOLD, C_GOLD_DARK, C_TEXT, C_MUTED, C_RED, C_GREEN, C_BLUE;

// ---------------- Buttons ----------------
struct Button {
  int x, y, w, h;
  const char* label;
};

Button btnMode, btnOpen, btnClose, btnStop, btnHome, btnSync, btnJogOpen, btnJogClose;

// ---------------- Dynamic UI cache ----------------
String prevValTime = "";
String prevValDate = "";
String prevValDoorState = "";
String prevValMotion = "";
String prevValSync = "";
String prevValLastSync = "";
String prevValOpenTime = "";
String prevValCloseTime = "";
String prevValSonar = "";
String prevValThresh = "";
String prevValPos = "";
String prevValLimits = "";
String prevValCycles = "";
String prevValZeroOps = "";
String prevValAngle = "";
bool prevAutoModeDrawn = false;

String prevValSonarLive1 = "";
String prevValSonarLive2 = "";
String prevValSonarLive3 = "";

volatile uint32_t lastEchoPulseUs = 0;
volatile uint32_t sonarSuccessCount = 0;
volatile uint32_t sonarTimeoutRiseCount = 0;
volatile uint32_t sonarTimeoutFallCount = 0;
volatile uint32_t sonarCycleCount = 0;
volatile uint32_t sonarLastGoodMs = 0;
volatile uint8_t  sonarEchoPinLive = 0;
volatile SonarState sonarDebugState = SONAR_IDLE;
// ============================================================
// ====================== LOW LEVEL HELPERS ====================
// ============================================================

void logPrint(const String& s) {
  if (!ENABLE_DEBUG_LOGS) return;
  Serial.print(s);
  if (ENABLE_TELNET_LOGGER && telnetClient && telnetClient.connected()) telnetClient.print(s);
}

void logPrintln(const String& s = "") {
  if (!ENABLE_DEBUG_LOGS) return;
  Serial.println(s);
  if (ENABLE_TELNET_LOGGER && telnetClient && telnetClient.connected()) telnetClient.println(s);
}

void logPrintf(const char* fmt, ...) {
  if (!ENABLE_DEBUG_LOGS) return;
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(buf);
  if (ENABLE_TELNET_LOGGER && telnetClient && telnetClient.connected()) telnetClient.print(buf);
}

// --- NEW --- Safe wrappers around stepper object because both cores use it.
long getStepperCurrentPosition() {
  portENTER_CRITICAL(&stepperMux);
  long v = stepper.currentPosition();
  portEXIT_CRITICAL(&stepperMux);
  return v;
}

long getStepperTargetPosition() {
  portENTER_CRITICAL(&stepperMux);
  long v = stepper.targetPosition();
  portEXIT_CRITICAL(&stepperMux);
  return v;
}

long getStepperDistanceToGo() {
  portENTER_CRITICAL(&stepperMux);
  long v = stepper.distanceToGo();
  portEXIT_CRITICAL(&stepperMux);
  return v;
}

bool stepperIsMoving() {
  return getStepperDistanceToGo() != 0;
}

void stepperMoveTo(long target) {
  portENTER_CRITICAL(&stepperMux);
  stepper.moveTo(target);
  portEXIT_CRITICAL(&stepperMux);
}

void stepperStop() {
  portENTER_CRITICAL(&stepperMux);
  stepper.stop();
  portEXIT_CRITICAL(&stepperMux);
}

void stepperSetCurrentPosition(long pos) {
  portENTER_CRITICAL(&stepperMux);
  stepper.setCurrentPosition(pos);
  portEXIT_CRITICAL(&stepperMux);
}

void stepperEnableOutputs() {
  portENTER_CRITICAL(&stepperMux);
  stepper.enableOutputs();
  portEXIT_CRITICAL(&stepperMux);
}

void stepperDisableOutputs() {
  portENTER_CRITICAL(&stepperMux);
  stepper.disableOutputs();
  portEXIT_CRITICAL(&stepperMux);
}

void stepperRunFast() {
  portENTER_CRITICAL(&stepperMux);
  stepper.run();
  portEXIT_CRITICAL(&stepperMux);
}

bool getLocalTimeSafe(tm &t) {
  time_t now = time(nullptr);
  if (now < 100000) return false;
  localtime_r(&now, &t);
  return true;
}

bool isOpenHours(const tm& t) {
  int nowMin   = t.tm_hour * 60 + t.tm_min;
  int openMin  = OPEN_HOUR * 60 + OPEN_MINUTE;
  int closeMin = CLOSE_HOUR * 60 + CLOSE_MINUTE;
  return (nowMin >= openMin && nowMin < closeMin);
}

String nowTimeString() {
  tm t;
  if (!getLocalTimeSafe(t)) return "--:--:--";
  char buf[20];
  strftime(buf, sizeof(buf), "%I:%M:%S %p", &t);
  return String(buf);
}

String nowDateString() {
  tm t;
  if (!getLocalTimeSafe(t)) return "--/--/----";
  char buf[20];
  strftime(buf, sizeof(buf), "%m/%d/%Y", &t);
  return String(buf);
}

String lastSyncString() {
  if (lastSuccessfulSyncEpoch <= 0) return "Never";
  tm t;
  localtime_r(&lastSuccessfulSyncEpoch, &t);
  char buf[24];
  strftime(buf, sizeof(buf), "%m/%d %I:%M %p", &t);
  return String(buf);
}

String wifiStatusToString(wl_status_t s) {
  switch (s) {
    case WL_IDLE_STATUS:      return "WL_IDLE_STATUS";
    case WL_NO_SSID_AVAIL:    return "WL_NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED:   return "WL_SCAN_COMPLETED";
    case WL_CONNECTED:        return "WL_CONNECTED";
    case WL_CONNECT_FAILED:   return "WL_CONNECT_FAILED";
    case WL_CONNECTION_LOST:  return "WL_CONNECTION_LOST";
    case WL_DISCONNECTED:     return "WL_DISCONNECTED";
    default:                  return "WL_UNKNOWN";
  }
}

String doorStateString() {
  switch (doorState) {
    case DOOR_BOOTING: return "BOOTING";
    case DOOR_STARTUP_SYNC_TIME: return "SYNC TIME";
    case DOOR_IDLE_OPEN: return "OPEN";
    case DOOR_IDLE_CLOSED: return "CLOSED";
    case DOOR_OPENING: return "OPENING";
    case DOOR_CLOSING: return "CLOSING";
    case DOOR_REOPENING_AFTER_OBSTACLE: return "REOPENING";
    case DOOR_WAITING_RETRY: return "WAIT RETRY";
    case DOOR_MANUAL: return "MANUAL";
    case DOOR_ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

String syncStateString() {
  switch (syncState) {
    case SYNC_IDLE: return "IDLE";
    case SYNC_CONNECTING: return "CONNECT";
    case SYNC_WAITING_NTP: return "NTP WAIT";
    case SYNC_DISCONNECTING: return "DONE";
    default: return "UNKNOWN";
  }
}

String motionReasonString() {
  switch (motionReason) {
    case REASON_NONE: return "NONE";
    case REASON_SCHEDULE_OPEN: return "AUTO OPEN";
    case REASON_SCHEDULE_CLOSE: return "AUTO CLOSE";
    case REASON_MANUAL_OPEN: return "MAN OPEN";
    case REASON_MANUAL_CLOSE: return "MAN CLOSE";
    case REASON_MANUAL_ZERO_CLOSED: return "ZERO CLOSED";
    case REASON_OBSTACLE_REOPEN: return "OBSTACLE";
    default: return "UNKNOWN";
  }
}

String sonarStateString(SonarState s) {
  switch (s) {
    case SONAR_IDLE:      return "IDLE";
    case SONAR_TRIG_LOW:  return "TRIG_LOW";
    case SONAR_TRIG_HIGH: return "TRIG_HIGH";
    case SONAR_WAIT_RISE: return "WAIT_RISE";
    case SONAR_WAIT_FALL: return "WAIT_FALL";
    default:              return "UNKNOWN";
  }
}

float getDoorAngleDegrees(long currentPos) {
  long span = openPositionSteps - closedPositionSteps;
  if (span <= 0) return 0.0f;

  float ratio = float(currentPos - closedPositionSteps) / float(span);
  if (ratio < 0.0f) ratio = 0.0f;
  if (ratio > 1.0f) ratio = 1.0f;

  return ratio * 90.0f;
}

void logCurrentTime(const char* prefix) {
  time_t now = time(nullptr);
  logPrintf("%s epoch=%lld\n", prefix, (long long)now);
  if (now > 100000) {
    tm t;
    localtime_r(&now, &t);
    char buf[40];
    strftime(buf, sizeof(buf), "%Y-%m-%d %I:%M:%S %p", &t);
    logPrintf("%s local=%s\n", prefix, buf);
  }
}

void logStepperConfig(const char* prefix) {
  logPrintf("%s STEPPER_MAX_SPEED const = %.1f\n", prefix, STEPPER_MAX_SPEED);
  logPrintf("%s STEPPER_ACCEL const     = %.1f\n", prefix, STEPPER_ACCEL);

  portENTER_CRITICAL(&stepperMux);
  float maxSpeed = stepper.maxSpeed();
  float accel = stepper.acceleration();
  portEXIT_CRITICAL(&stepperMux);

  logPrintf("%s stepper.maxSpeed()      = %.1f\n", prefix, maxSpeed);
  logPrintf("%s stepper.acceleration()  = %.1f\n", prefix, accel);
  logPrintf("%s openPositionSteps       = %ld\n", prefix, openPositionSteps);
  logPrintf("%s closedPositionSteps     = %ld\n", prefix, closedPositionSteps);
}

// ============================================================
// ====================== PREFS ================================
// ============================================================

void saveSettings() {
  prefs.putLong("openPos", openPositionSteps);
  prefs.putLong("closedPos", closedPositionSteps);
  prefs.putBool("autoMode", autoMode);
  prefs.putULong("openCount", openCycleCount);
  prefs.putULong("closeCount", closeCycleCount);
  prefs.putULong("zeroCount", zeroClosedCount);
  prefs.putULong64("lastsync", (uint64_t)lastSuccessfulSyncEpoch);
}

void loadSettings() {
  // Force calibration constants from source code
  openPositionSteps       = OPEN_POSITION_STEPS_DEFAULT;
  closedPositionSteps     = CLOSED_POSITION_STEPS_DEFAULT;
  obstacleThresholdCm     = OBSTACLE_THRESHOLD_CM_DEFAULT;

  // Keep only the values you actually want persisted
  autoMode                = prefs.getBool("autoMode", true);
  openCycleCount          = prefs.getULong("openCount", 0);
  closeCycleCount         = prefs.getULong("closeCount", 0);
  zeroClosedCount         = prefs.getULong("zeroCount", 0);
  lastSuccessfulSyncEpoch = (time_t)prefs.getULong64("lastsync", 0);
}

bool shouldWeeklyResync() {
  if (lastSuccessfulSyncEpoch <= 0) return true;
  time_t now = time(nullptr);
  if (now <= 0) return false;
  return (uint32_t)(now - lastSuccessfulSyncEpoch) >= WEEKLY_SYNC_SEC;
}

// ============================================================
// ====================== BACKLIGHT ============================
// ============================================================

void setBacklight(uint8_t level) {
  if (level == currentBacklightLevel) return;
  currentBacklightLevel = level;
  ledcWrite(PIN_TFT_BL, level);
}

bool isWithinActiveScheduleWindow() {
  tm t;
  if (!getLocalTimeSafe(t)) return false;

  int nowMin = t.tm_hour * 60 + t.tm_min;
  int openMin = OPEN_HOUR * 60 + OPEN_MINUTE;
  int closeMin = CLOSE_HOUR * 60 + CLOSE_MINUTE;
  int window = (int)SCREEN_ACTIVE_WINDOW_MIN;

  bool nearOpen  = (nowMin >= (openMin - window)  && nowMin <= (openMin + window));
  bool nearClose = (nowMin >= (closeMin - window) && nowMin <= (closeMin + window));

  return nearOpen || nearClose;
}

void updateBacklightPolicy() {
  bool full =
    isWithinActiveScheduleWindow() ||
    stepperIsMoving() ||
    (millis() < motorActiveUntilMs) ||
    (millis() < touchWakeUntilMs);

  setBacklight(full ? BL_FULL : BL_DIMMED);
}

// ============================================================
// ====================== SONAR ================================
// ============================================================

void startSonarCycle() {
  sonarState = SONAR_TRIG_LOW;
  sonarStateAtUs = micros();
  digitalWrite(PIN_TRIG, LOW);
}

void serviceSonar() {
  uint32_t nowUs = micros();

  sonarEchoPinLive = digitalRead(PIN_ECHO);
  sonarDebugState = sonarState;

  switch (sonarState) {
    case SONAR_IDLE:
      if (millis() - lastSonarKickMs >= SONAR_POLL_MS) {
        lastSonarKickMs = millis();
        sonarCycleCount++;
        startSonarCycle();
      }
      break;

    case SONAR_TRIG_LOW:
      if (nowUs - sonarStateAtUs >= 3) {
        digitalWrite(PIN_TRIG, HIGH);
        sonarState = SONAR_TRIG_HIGH;
        sonarStateAtUs = nowUs;
      }
      break;

    case SONAR_TRIG_HIGH:
      if (nowUs - sonarStateAtUs >= 10) {
        digitalWrite(PIN_TRIG, LOW);
        sonarState = SONAR_WAIT_RISE;
        sonarStateAtUs = nowUs;
      }
      break;

    case SONAR_WAIT_RISE:
      if (digitalRead(PIN_ECHO) == HIGH) {
        sonarEchoRiseUs = micros();
        sonarState = SONAR_WAIT_FALL;
      } else if ((uint32_t)(micros() - sonarStateAtUs) > SONAR_TIMEOUT_US) {
        lastDistanceCm = -1.0f;
        lastEchoPulseUs = 0;
        sonarTimeoutRiseCount++;
        sonarState = SONAR_IDLE;
      }
      break;

    case SONAR_WAIT_FALL:
      if (digitalRead(PIN_ECHO) == LOW) {
        uint32_t dur = micros() - sonarEchoRiseUs;
        lastEchoPulseUs = dur;
        lastDistanceCm = dur * 0.0343f / 2.0f;
        sonarSuccessCount++;
        sonarLastGoodMs = millis();
        sonarState = SONAR_IDLE;
      } else if ((uint32_t)(micros() - sonarEchoRiseUs) > SONAR_TIMEOUT_US) {
        lastDistanceCm = -1.0f;
        lastEchoPulseUs = 0;
        sonarTimeoutFallCount++;
        sonarState = SONAR_IDLE;
      }
      break;
  }

  sonarDebugState = sonarState;
  sonarEchoPinLive = digitalRead(PIN_ECHO);
}

// ============================================================
// ====================== WIFI / NTP ===========================
// ============================================================

void startTelnetServer() {
  if (!ENABLE_TELNET_LOGGER) return;
  telnetServer.begin();
  telnetServer.setNoDelay(true);
  logPrintf("[TELNET] Server started on port %u\n", TELNET_PORT);
}

void serviceTelnet() {
  if (!ENABLE_TELNET_LOGGER) return;

  if (telnetServer.hasClient()) {
    WiFiClient newClient = telnetServer.available();
    if (!telnetClient || !telnetClient.connected()) {
      telnetClient = newClient;
      telnetClient.setNoDelay(true);
      telnetWasConnected = true;
      telnetClient.println();
      telnetClient.println("======================================");
      telnetClient.println("Chicken Door Telnet Logger Connected");
      telnetClient.println("======================================");
      telnetClient.flush();
      Serial.println("[TELNET] Client connected.");
    } else {
      newClient.println("Another Telnet client is already connected.");
      newClient.stop();
      Serial.println("[TELNET] Rejected extra client.");
    }
  }

  if (telnetWasConnected && (!telnetClient || !telnetClient.connected())) {
    telnetWasConnected = false;
    Serial.println("[TELNET] Client disconnected.");
  }
}

void setupArduinoOTA() {
  if (!ENABLE_OTA_FEATURE) return;

  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA
    .onStart([]() { logPrintln("[OTA] Start updating"); })
    .onEnd([]() { logPrintln("[OTA] End"); })
    .onProgress([](unsigned int progress, unsigned int total) {
      static uint32_t lastOtaLogMs = 0;
      if (millis() - lastOtaLogMs >= 500) {
        lastOtaLogMs = millis();
        logPrintf("[OTA] Progress: %u%%\n", (progress / (total / 100)));
      }
    })
    .onError([](ota_error_t error) {
      logPrintf("[OTA] Error: %u\n", error);
    });

  ArduinoOTA.begin();
  logPrintf("[OTA] Ready. Hostname: %s.local\n", OTA_HOSTNAME);
}

void serviceArduinoOTA() {
  if (!ENABLE_OTA_FEATURE) return;
  ArduinoOTA.handle();
}

void beginTimeSync() {
  if (syncState != SYNC_IDLE) return;

  syncState = SYNC_CONNECTING;
  syncStartedMs = millis();
  doorState = DOOR_STARTUP_SYNC_TIME;

  logPrintln("[TIME] beginTimeSync() starting...");
  logPrintf("[TIME] SSID: %s\n", WIFI_SSID);
  logPrintf("[TIME] Current WiFi.status(): %s\n", wifiStatusToString(WiFi.status()).c_str());

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  logPrintln("[TIME] WiFi.begin() called.");
}

void serviceTimeSync() {
  if (syncState == SYNC_IDLE) return;

  static wl_status_t lastLoggedWifi = WL_IDLE_STATUS;
  wl_status_t nowStatus = WiFi.status();
  if (nowStatus != lastLoggedWifi) {
    lastLoggedWifi = nowStatus;
    logPrintf("[TIME] WiFi status changed: %s\n", wifiStatusToString(nowStatus).c_str());
  }

  switch (syncState) {
    case SYNC_CONNECTING:
      if (WiFi.status() == WL_CONNECTED) {
        logPrintln("[TIME] WiFi connected.");
        logPrintf("[TIME] Local IP: %s\n", WiFi.localIP().toString().c_str());

        if (ENABLE_OTA_FEATURE) {
          if (MDNS.begin(OTA_HOSTNAME)) {
            logPrintf("[MDNS] Started: http://%s.local/\n", OTA_HOSTNAME);
          }
          setupArduinoOTA();
        }

        if (ENABLE_TELNET_LOGGER) startTelnetServer();

        configTzTime(TZ_INFO, NTP1, NTP2);
        syncState = SYNC_WAITING_NTP;
        syncStartedMs = millis();
      } else if (millis() - syncStartedMs > 15000) {
        logPrintln("[TIME] WiFi connect timed out.");
        syncState = SYNC_DISCONNECTING;
      }
      break;

    case SYNC_WAITING_NTP: {
      static uint32_t lastNtpPollMs = 0;
      if (millis() - lastNtpPollMs >= 250) {
        lastNtpPollMs = millis();
        time_t now = time(nullptr);
        if (now > 100000) {
          lastSuccessfulSyncEpoch = now;
          prefs.putULong64("lastsync", (uint64_t)lastSuccessfulSyncEpoch);
          logPrintln("[TIME] NTP sync successful.");
          logCurrentTime("[TIME]");
          syncState = SYNC_DISCONNECTING;
        }
      }

      if (millis() - syncStartedMs > 12000) {
        logPrintln("[TIME] NTP wait timed out.");
        syncState = SYNC_DISCONNECTING;
      }
      break;
    }

    case SYNC_DISCONNECTING:
      // --- NEW --- Leave WiFi connected only if OTA/Telnet are enabled.
      if (!ENABLE_OTA_FEATURE && !ENABLE_TELNET_LOGGER) {
        WiFi.disconnect(true, false);
        WiFi.mode(WIFI_OFF);
        logPrintln("[TIME] WiFi turned off after sync.");
      } else {
        logPrintln("[TIME] Leaving WiFi connected after sync.");
      }

      syncState = SYNC_IDLE;

      if (!stepperIsMoving()) {
        tm t;
        if (getLocalTimeSafe(t)) {
          if (isOpenHours(t)) {
            stepperSetCurrentPosition(openPositionSteps);
            stepperMoveTo(openPositionSteps);
            doorState = DOOR_IDLE_OPEN;
            logPrintln("[TIME] Startup assumption set to OPEN based on time of day.");
          } else {
            stepperSetCurrentPosition(closedPositionSteps);
            stepperMoveTo(closedPositionSteps);
            doorState = DOOR_IDLE_CLOSED;
            logPrintln("[TIME] Startup assumption set to CLOSED based on time of day.");
          }
        } else {
          stepperSetCurrentPosition(closedPositionSteps);
          stepperMoveTo(closedPositionSteps);
          doorState = DOOR_IDLE_CLOSED;
          logPrintln("[TIME] No valid time. Defaulting startup assumption to CLOSED.");
        }
      }
      break;

    default:
      break;
  }
}

// ============================================================
// ====================== MOTION CONTROL =======================
// ============================================================

void markMotorActiveWindow() {
  motorActiveUntilMs = millis() + (2UL * 60UL * 1000UL);
}

bool sonarValidForMotion() {
  return true;
}

// bool sonarValidForMotion() {
//   // valid if we have had at least one good reading recently
//   if (sonarSuccessCount == 0) return false;
//   if (millis() - sonarLastGoodMs > 2000) return false;
//   return true;
// }

void startMoveTo(long target, MotionReason reason, DoorState newState) {
  // --- NEW --- Hard safety interlock before ANY motion.
  if (!sonarValidForMotion()) {
    logPrintln("[SAFETY] Sonar not responding. Movement blocked.");
    doorState = DOOR_ERROR;
    motionReason = REASON_NONE;
    return;
  }

  long currentPos = getStepperCurrentPosition();
  if (target == currentPos) {
    if (target <= closedPositionSteps) doorState = DOOR_IDLE_CLOSED;
    else if (target >= openPositionSteps) doorState = DOOR_IDLE_OPEN;
    else doorState = DOOR_MANUAL;
    motionReason = REASON_NONE;
    return;
  }

  stepperEnableOutputs();
  motionReason = reason;
  doorState = newState;
  motionCompleteFlag = false;
  stepperMoveTo(target);
  markMotorActiveWindow();
}

void stopMotionNow(DoorState newState = DOOR_MANUAL) {
  stepperStop();
  motionReason = REASON_NONE;
  doorState = newState;
  markMotorActiveWindow();
}

void beginScheduledOpen()  { startMoveTo(openPositionSteps,   REASON_SCHEDULE_OPEN, DOOR_OPENING); }
void beginScheduledClose() { obstacleHitCounter = 0; startMoveTo(closedPositionSteps, REASON_SCHEDULE_CLOSE, DOOR_CLOSING); }
void beginManualOpen()     { startMoveTo(openPositionSteps,   REASON_MANUAL_OPEN, DOOR_OPENING); }
void beginManualClose()    { obstacleHitCounter = 0; startMoveTo(closedPositionSteps, REASON_MANUAL_CLOSE, DOOR_CLOSING); }
void beginObstacleReopen() { startMoveTo(openPositionSteps,   REASON_OBSTACLE_REOPEN, DOOR_REOPENING_AFTER_OBSTACLE); }

void jogRelativeClamped(long delta, MotionReason reason) {
  long current = getStepperCurrentPosition();
  long target = current + delta;

  if (target > openPositionSteps)   target = openPositionSteps;
  if (target < closedPositionSteps) target = closedPositionSteps;
  if (target == current) return;

  startMoveTo(target, reason, DOOR_MANUAL);
}

void zeroCurrentAsClosed() {
  stepperSetCurrentPosition(closedPositionSteps);
  stepperMoveTo(closedPositionSteps);
  doorState = DOOR_IDLE_CLOSED;
  motionReason = REASON_MANUAL_ZERO_CLOSED;
  zeroClosedCount++;
  saveSettings();
  markMotorActiveWindow();
}

void handleClosingSafety() {
  if (doorState != DOOR_CLOSING) return;

  bool obstacleNow = false;

  if (lastDistanceCm > 0.0f && lastDistanceCm <= obstacleThresholdCm) {
    obstacleNow = true;
  }

  uint32_t now = millis();

  if (obstacleNow) {
    if (obstacleStartMs == 0) {
      obstacleStartMs = now;
    }

    if (now - obstacleStartMs >= 250) {
      logPrintf("[SAFETY] Obstacle confirmed %.1f cm for %lu ms. Reopening.\n",
                lastDistanceCm, (unsigned long)(now - obstacleStartMs));

      stepperStop();
      obstacleStartMs = 0;
      beginObstacleReopen();
    }
  }
  else {
    obstacleStartMs = 0;
  }
}

// --- NEW --- Only state-machine completion here. The motor task raises the flag.
void handleMotionCompletion() {
  if (!motionCompleteFlag) return;
  motionCompleteFlag = false;

  stepperDisableOutputs();

  switch (doorState) {
    case DOOR_OPENING:
      doorState = DOOR_IDLE_OPEN;
      motionReason = REASON_NONE;
      openCycleCount++;
      saveSettings();
      break;

    case DOOR_CLOSING:
      doorState = DOOR_IDLE_CLOSED;
      motionReason = REASON_NONE;
      closeCycleCount++;
      saveSettings();
      break;

    case DOOR_REOPENING_AFTER_OBSTACLE:
      doorState = DOOR_WAITING_RETRY;
      motionReason = REASON_NONE;
      retryAtMs = millis() + RETRY_DELAY_MS;
      break;

    case DOOR_MANUAL: {
      motionReason = REASON_NONE;
      long pos = getStepperCurrentPosition();
      if (pos <= closedPositionSteps) doorState = DOOR_IDLE_CLOSED;
      else if (pos >= openPositionSteps) doorState = DOOR_IDLE_OPEN;
      else doorState = DOOR_MANUAL;
      break;
    }

    default:
      break;
  }
}

void serviceRetryClose() {
  if (doorState != DOOR_WAITING_RETRY) return;
  if (millis() < retryAtMs) return;
  if (stepperIsMoving()) return;

  logPrintln("[RETRY] Retry timer expired. Attempting close again.");
  beginScheduledClose();
}

// ============================================================
// ====================== SCHEDULE =============================
// ============================================================

void serviceScheduledEvents() {
  if (syncState != SYNC_IDLE || stepperIsMoving() || doorState == DOOR_WAITING_RETRY) return;

  tm t;
  if (!getLocalTimeSafe(t)) return;

  if (t.tm_yday == lastHandledYDay && t.tm_hour == lastHandledHour && t.tm_min == lastHandledMinute) return;

  lastHandledYDay   = t.tm_yday;
  lastHandledHour   = t.tm_hour;
  lastHandledMinute = t.tm_min;

  if (t.tm_hour == OPEN_HOUR && t.tm_min == OPEN_MINUTE) {
    logPrintln("[SCHEDULE] 5:00 AM scheduled OPEN triggered.");
    beginScheduledOpen();
    return;
  }

  if (t.tm_hour == CLOSE_HOUR && t.tm_min == CLOSE_MINUTE) {
    logPrintln("[SCHEDULE] 10:00 PM scheduled CLOSE triggered.");
    beginScheduledClose();
    return;
  }
}

void serviceWeeklySync() {
  if (syncState != SYNC_IDLE || !shouldWeeklyResync() || stepperIsMoving() || doorState == DOOR_WAITING_RETRY) return;
  beginTimeSync();
}

// ============================================================
// ====================== UI HELPERS ===========================
// ============================================================

bool pointInButton(int x, int y, const Button& b) {
  return (x >= b.x && x < (b.x + b.w) && y >= b.y && y < (b.y + b.h));
}

void clearValueField(int x, int y, int w, int h, uint16_t bg) {
  tft.fillRect(x, y, w, h, bg);
}

void drawValueField(int x, int y, int w, int h, const String& value, uint16_t fg, uint16_t bg, int font = 2) {
  clearValueField(x, y, w, h, bg);
  tft.setTextColor(fg, bg);
  tft.drawString(value, x, y, font);
}

void buildButtons() {
  btnMode     = {200, 180, 50, 34, autoMode ? "AUTO" : "MANU"};
  btnOpen     = {255, 180, 50, 34, "OPEN"};
  btnClose    = {310, 180, 50, 34, "CLOSE"};
  btnStop     = {365, 180, 50, 34, "STOP"};
  btnHome     = {420, 180, 50, 34, "HOME"};
  btnSync     = {200, 224, 65, 34, "SYNC"};
  btnJogOpen  = {280, 224, 65, 30, "S OPEN"};
  btnJogClose = {360, 224, 65, 30, "S CLOSE"};
}

void drawButton(const Button& b, bool active = false) {
  uint16_t fill = active ? C_GOLD : C_PANEL;
  uint16_t text = active ? C_BG : C_GOLD;

  tft.fillRoundRect(b.x, b.y, b.w, b.h, 9, fill);
  tft.drawRoundRect(b.x, b.y, b.w, b.h, 9, C_GOLD);
  tft.drawRoundRect(b.x + 2, b.y + 2, b.w - 4, b.h - 4, 7, C_GOLD);
  tft.setTextColor(text, fill);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(b.label, b.x + b.w / 2, b.y + b.h / 2, 2);
  tft.setTextDatum(TL_DATUM);
}

void drawDoorMotifLeft() {
  int x = 10, y = 10, w = 180, h = 300;
  tft.fillRoundRect(x, y, w, h, 10, C_PANEL);
  tft.drawRoundRect(x, y, w, h, 10, C_GOLD);
  tft.fillRoundRect(x + 20, y + 20, 140, 110, 6, C_GOLD);
  tft.fillRoundRect(x + 20, y + 165, 140, 110, 6, C_GOLD);
  tft.fillRect(x + 30, y + 32, 120, 86, C_BG);
  tft.fillRect(x + 30, y + 177, 120, 86, C_BG);

  uint16_t c = C_GOLD;
  tft.drawLine(x+34, y+52,  x+65, y+42, c);
  tft.drawLine(x+65, y+42,  x+94, y+67, c);
  tft.drawLine(x+94, y+67,  x+128, y+45, c);
  tft.drawLine(x+42, y+92,  x+78, y+74, c);
  tft.drawLine(x+78, y+74,  x+112, y+94, c);
  tft.drawLine(x+112,y+94,  x+145, y+77, c);
  tft.drawLine(x+50, y+112, x+90, y+99, c);
  tft.drawLine(x+90, y+99,  x+136, y+109, c);
  tft.drawLine(x+34, y+194, x+69, y+186, c);
  tft.drawLine(x+69, y+186, x+98, y+207, c);
  tft.drawLine(x+98, y+207, x+132, y+188, c);
  tft.drawLine(x+44, y+233, x+82, y+216, c);
  tft.drawLine(x+82, y+216, x+112, y+239, c);
  tft.drawLine(x+112,y+239, x+145, y+221, c);
  tft.fillCircle(x + 18, y + 150, 12, C_GOLD);
  tft.drawCircle(x + 18, y + 150, 12, C_GOLD_DARK);
}

void drawStaticUI() {
  buildButtons();

  tft.fillScreen(C_BG);
  drawDoorMotifLeft();

  int x = 205;
  int y = 10;
  int w = tft.width() - x - 10;
  int h = 150;

  tft.fillRoundRect(x, y, w, h, 10, C_PANEL);
  tft.drawRoundRect(x, y, w, h, 10, C_GOLD);

  tft.setTextColor(C_GOLD, C_PANEL);
  tft.drawString("ClukWorx", x + 12, y + 10, 4);

  tft.setFreeFont(&FreeSans9pt7b);
  tft.drawString("Vernoi Door Controls", x + 12, y + 35);
  tft.setFreeFont(NULL);   // return to built-in fonts

  // Left-side info only
  tft.drawString("Time/Date:",  x + 12,  y + 60,  2);
  tft.drawString("Door State:", x + 12,  y + 78,  2);
  tft.drawString("Motion:",     x + 12,  y + 96,  2);
  tft.drawString("Sync:",       x + 12,  y + 114, 2);
  tft.drawString("Last Sync:",  x + 12,  y + 132, 2);

  drawButton(btnMode, autoMode);
  drawButton(btnOpen);
  drawButton(btnClose);
  drawButton(btnStop);
  drawButton(btnHome);
  drawButton(btnSync);
  drawButton(btnJogOpen);
  drawButton(btnJogClose);

  // Bottom two lines only
  tft.setTextColor(C_MUTED, C_BG);
  tft.drawString("SONAR:", 210, 268, 2);
  tft.drawString("DIST:",  210, 288, 2);

  prevValTime = "";
  prevValDate = "";
  prevValDoorState = "";
  prevValMotion = "";
  prevValSync = "";
  prevValLastSync = "";
  prevValOpenTime = "";
  prevValCloseTime = "";
  prevValSonar = "";
  prevValThresh = "";
  prevValPos = "";
  prevValLimits = "";
  prevValCycles = "";
  prevValZeroOps = "";
  prevValAngle = "";
  prevValSonarLive1 = "";
  prevValSonarLive2 = "";
  prevValSonarLive3 = "";
  prevAutoModeDrawn = !autoMode;
}

void updateDynamicUI(bool force = false) {
  int x = 205;
  int y = 10;

  long currentPos = getStepperCurrentPosition();
  long targetPos  = getStepperTargetPosition();

  float sonarCm   = lastDistanceCm;
  float sonarIn   = (sonarCm < 0.0f) ? -1.0f : (sonarCm / 2.54f);
  float doorAngle = getDoorAngleDegrees(currentPos);

  String valTime      = nowTimeString();
  String valDate      = nowDateString();
  String valTimeDate  = valTime + " " + valDate;
  String valDoorState = doorStateString();
  String valMotion    = motionReasonString();
  String valSync      = syncStateString();
  String valLastSync  = lastSyncString();

  String valSonarLive1 = (sonarCm < 0.0f) ? "OBSTACLE" : "LIVE";
  String valSonarLive2 = (sonarCm < 0.0f)
    ? "OBSTRUCTION/NO ECHO"
    : (String(sonarCm, 1) + " cm  " + String(sonarIn, 1) + " in");

  // Position + angle on same SONAR row, shifted right
  String valPosAngle = String(currentPos) + "/" + String(targetPos) + "  " + String((int)doorAngle) + "d";

  // Keep spacer under title untouched so it does not clip the subtitle
  if (force || valTime != prevValTime) {
    prevValTime = valTime;
  }

  if (force || valTimeDate != prevValDate) {
    drawValueField(x + 90, y + 60, 145, 16, valTimeDate, C_TEXT, C_PANEL, 2);
    prevValDate = valTimeDate;
  }

  if (force || valDoorState != prevValDoorState) {
    drawValueField(x + 110, y + 78, 120, 16, valDoorState, C_TEXT, C_PANEL, 2);
    prevValDoorState = valDoorState;
  }

  if (force || valMotion != prevValMotion) {
    drawValueField(x + 80, y + 96, 140, 16, valMotion, C_TEXT, C_PANEL, 2);
    prevValMotion = valMotion;
  }

  if (force || valSync != prevValSync) {
    drawValueField(x + 62, y + 114, 110, 16, valSync, C_TEXT, C_PANEL, 2);
    prevValSync = valSync;
  }

  if (force || valLastSync != prevValLastSync) {
    drawValueField(x + 95, y + 132, 120, 16, valLastSync, C_TEXT, C_PANEL, 2);
    prevValLastSync = valLastSync;
  }

  // SONAR line value
  if (force || valSonarLive1 != prevValSonarLive1) {
    drawValueField(275, 268, 70, 16, valSonarLive1, C_MUTED, C_BG, 2);
    prevValSonarLive1 = valSonarLive1;
  }

  // Position + angle on same line, shifted right enough to show
  if (force || valPosAngle != prevValPos) {
    drawValueField(375, 268, 105, 16, valPosAngle, C_MUTED, C_BG, 2);
    prevValPos = valPosAngle;
  }

  // DIST line
  if (force || valSonarLive2 != prevValSonarLive2) {
    drawValueField(255, 288, 225, 16, valSonarLive2, C_MUTED, C_BG, 2);
    prevValSonarLive2 = valSonarLive2;
  }

  if (force || autoMode != prevAutoModeDrawn) {
    btnMode.label = autoMode ? "AUTO" : "MANU";
    drawButton(btnMode, autoMode);
    prevAutoModeDrawn = autoMode;
  }
}

// uint32_t rawPulseUs = lastEchoPulseUs;
// uint32_t okCount = sonarSuccessCount;
// uint32_t toRise = sonarTimeoutRiseCount;
// uint32_t toFall = sonarTimeoutFallCount;
// uint32_t ageMs = (sonarLastGoodMs == 0) ? 0 : (millis() - sonarLastGoodMs);
// uint8_t echoLive = sonarEchoPinLive;
// SonarState sState = sonarDebugState;

// String valSonarLive1 = (sonarCm < 0.0f)
//   ? ("DIST: NO ECHO   RAW: 0 us")
//   : ("DIST: " + String(sonarCm, 1) + " cm   RAW: " + String(rawPulseUs) + " us");

// String valSonarLive2 =
//   "STATE: " + sonarStateString(sState) +
//   "   ECHO: " + String(echoLive ? "HIGH" : "LOW");

// String valSonarLive3 =
//   "GOOD: " + String(okCount) +
//   "   TO(R/F): " + String(toRise) + "/" + String(toFall) +
//   "   AGE: " + String(ageMs) + "ms";

//   String valTime      = nowTimeString();
//   String valDate      = nowDateString();
//   String valDoorState = doorStateString();
//   String valMotion    = motionReasonString();
//   String valSync      = syncStateString();
//   String valLastSync  = lastSyncString();

//   String valOpenTime  = "05:00 AM";
//   String valCloseTime = "10:00 PM";
//   String valSonar     = (sonarCm < 0.0f) ? "NO SENSOR" : (String(sonarCm, 1) + " cm");
//   String valThresh    = String(obstacleThresholdCm, 1) + " cm";
//   String valPos       = String(currentPos) + "  T:" + String(targetPos);
//   String valLimits    = "C:" + String(closedPositionSteps) + " O:" + String(openPositionSteps);
//   String valCycles    = "Cycles O/C: " + String(openCycleCount) + "/" + String(closeCycleCount);
//   String valZeroOps   = "Zero Closed Ops: " + String(zeroClosedCount);

//   if (force || valTime != prevValTime) { drawValueField(x + 70, y + 42, 150, 16, valTime, C_TEXT, C_PANEL, 2); prevValTime = valTime; }
//   if (force || valDate != prevValDate) { drawValueField(x + 70, y + 60, 150, 16, valDate, C_TEXT, C_PANEL, 2); prevValDate = valDate; }
//   if (force || valDoorState != prevValDoorState) { drawValueField(x + 110, y + 78, 140, 16, valDoorState, C_TEXT, C_PANEL, 2); prevValDoorState = valDoorState; }
//   if (force || valMotion != prevValMotion) { drawValueField(x + 80, y + 96, 150, 16, valMotion, C_TEXT, C_PANEL, 2); prevValMotion = valMotion; }
//   if (force || valSync != prevValSync) { drawValueField(x + 62, y + 114, 120, 16, valSync, C_TEXT, C_PANEL, 2); prevValSync = valSync; }
//   if (force || valLastSync != prevValLastSync) { drawValueField(x + 95, y + 132, 150, 16, valLastSync, C_TEXT, C_PANEL, 2); prevValLastSync = valLastSync; }
//   if (force || valOpenTime != prevValOpenTime) { drawValueField(x + 370, y + 42, 90, 16, valOpenTime, C_TEXT, C_PANEL, 2); prevValOpenTime = valOpenTime; }
//   if (force || valCloseTime != prevValCloseTime) { drawValueField(x + 375, y + 58, 90, 16, valCloseTime, C_TEXT, C_PANEL, 2); prevValCloseTime = valCloseTime; }
//   if (force || valSonar != prevValSonar) { drawValueField(x + 335, y + 74, 130, 16, valSonar, C_TEXT, C_PANEL, 2); prevValSonar = valSonar; }
//   if (force || valThresh != prevValThresh) { drawValueField(x + 343, y + 90, 100, 16, valThresh, C_TEXT, C_PANEL, 2); prevValThresh = valThresh; }
//   if (force || valPos != prevValPos) { drawValueField(x + 315, y + 106, 170, 16, valPos, C_TEXT, C_PANEL, 2); prevValPos = valPos; }
//   if (force || valLimits != prevValLimits) { drawValueField(x + 325, y + 122, 150, 16, valLimits, C_TEXT, C_PANEL, 2); prevValLimits = valLimits; }
//   if (force || valSonarLive1 != prevValSonarLive1) {
//     drawValueField(300, 266, 180, 16, valSonarLive1, C_MUTED, C_BG, 2);
//     prevValSonarLive1 = valSonarLive1;
//   }

//   if (force || valSonarLive2 != prevValSonarLive2) {
//     drawValueField(300, 284, 180, 16, valSonarLive2, C_MUTED, C_BG, 2);
//     prevValSonarLive2 = valSonarLive2;
//   }

//   if (force || valSonarLive3 != prevValSonarLive3) {
//     drawValueField(300, 302, 180, 16, valSonarLive3, C_MUTED, C_BG, 2);
//     prevValSonarLive3 = valSonarLive3;
//   }

//   if (force || autoMode != prevAutoModeDrawn) {
//     btnMode.label = autoMode ? "AUTO" : "MANU";
//     drawButton(btnMode, autoMode);
//     prevAutoModeDrawn = autoMode;
//   }
// }

void handleTouch() {
  // --- NEW --- Poll touch only every 50 ms.
  static uint32_t lastTouchPollMs = 0;
  if (millis() - lastTouchPollMs < TOUCH_POLL_MS) return;
  lastTouchPollMs = millis();

  uint16_t touchX, touchY;
  bool down = tft.getTouch(&touchX, &touchY);

  if (down) {
    touchY = tft.height() - touchY;   // flip Y touch only
    touchWakeUntilMs = millis() + TOUCH_WAKE_MS;
  }

  if (down && !touchWasDown) {
    // If your panel is inverted on Y, uncomment the next line:
    // touchY = tft.height() - touchY;

    if (pointInButton(touchX, touchY, btnMode)) {
      autoMode = !autoMode;
      saveSettings();
    }
    else if (pointInButton(touchX, touchY, btnSync)) {
      if (syncState == SYNC_IDLE) beginTimeSync();
    }
    else if (pointInButton(touchX, touchY, btnOpen)) {
      autoMode = false;
      if (!stepperIsMoving()) beginManualOpen();
    }
    else if (pointInButton(touchX, touchY, btnClose)) {
      autoMode = false;
      if (!stepperIsMoving()) beginManualClose();
    }
    else if (pointInButton(touchX, touchY, btnStop)) {
      stopMotionNow(DOOR_MANUAL);
    }
    else if (pointInButton(touchX, touchY, btnHome)) {
      autoMode = false;
      if (!stepperIsMoving()) zeroCurrentAsClosed();
    }
    else if (pointInButton(touchX, touchY, btnJogOpen)) {
      autoMode = false;
      if (!stepperIsMoving()) jogRelativeClamped(MANUAL_JOG_STEPS, REASON_MANUAL_OPEN);
    }
    else if (pointInButton(touchX, touchY, btnJogClose)) {
      autoMode = false;
      if (!stepperIsMoving()) jogRelativeClamped(-MANUAL_JOG_STEPS, REASON_MANUAL_CLOSE);
    }
  }

  touchWasDown = down;
}

// ============================================================
// ====================== CORE 0 MOTOR TASK ====================
// ============================================================

// --- NEW --- This task isolates time-critical stepping, sonar, and safety.
void Core0MotorTask(void *pvParameters) {
  uint32_t wdKickCounter = 0;

  for (;;) {
    motorLoopCounter++;

    // 1) Run motor as fast as possible when needed.
    if (stepperIsMoving()) {
      stepperRunFast();
    }

    // 2) Sonar and close-obstacle safety live here too.
    serviceSonar();
    handleClosingSafety();

    // 3) Raise completion flag once motion actually reaches target.
    if (!motionCompleteFlag && !stepperIsMoving()) {
      if (doorState == DOOR_OPENING ||
          doorState == DOOR_CLOSING ||
          doorState == DOOR_REOPENING_AFTER_OBSTACLE ||
          doorState == DOOR_MANUAL) {
        motionCompleteFlag = true;
      }
    }

// --- NEW --- Watchdog-friendly behavior:
    // Fast loop always to maintain sonar polling accuracy, occasionally delay for watchdog.
    wdKickCounter++;
    if (wdKickCounter >= 200) {
      wdKickCounter = 0;
      vTaskDelay(1);
    } else {
      taskYIELD();
    }
  }
}

// ============================================================
// ====================== SETUP ================================
// ============================================================

void setupPins() {
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  digitalWrite(PIN_STEP, LOW);
  digitalWrite(PIN_TRIG, LOW);
}

void setupBacklight() {
  ledcAttach(PIN_TFT_BL, BL_PWM_FREQ, BL_PWM_BITS);
  setBacklight(BL_FULL);
}

void setupStepper() {
  // --- NEW --- Let AccelStepper manage enable pin and pulse width.
  stepper.setEnablePin(PIN_EN);
  stepper.setPinsInverted(false, false, true); // enable is active LOW
  stepper.setMinPulseWidth(STEP_PULSE_US);
  stepper.setMaxSpeed(STEPPER_MAX_SPEED);
  stepper.setAcceleration(STEPPER_ACCEL);

  stepperSetCurrentPosition(closedPositionSteps);
  stepperMoveTo(closedPositionSteps);
  stepperDisableOutputs();

  logStepperConfig("[STEPPER]");
}

void setupDisplayAndTouch() {
  tft.init();
  tft.setRotation(1);

  C_BG        = tft.color565(8, 8, 8);
  C_PANEL     = tft.color565(18, 18, 20);
  C_GOLD      = tft.color565(225, 188, 90);
  C_GOLD_DARK = tft.color565(150, 115, 35);
  C_TEXT      = tft.color565(235, 235, 230);
  C_MUTED     = tft.color565(140, 140, 135);
  C_RED       = tft.color565(220, 60, 50);
  C_GREEN     = tft.color565(60, 200, 90);
  C_BLUE      = tft.color565(70, 130, 220);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  prefs.begin("coopdoor", false);
  prefs.remove("obstThresh");
  loadSettings();
  obstacleThresholdCm = 30.0f;

  logPrintln();
  logPrintln("========== BOOT ==========");
  logPrintf("[PREFS] openPositionSteps = %ld\n", openPositionSteps);
  logPrintf("[PREFS] closedPositionSteps = %ld\n", closedPositionSteps);
  logPrintf("[PREFS] obstacleThresholdCm = %.1f\n", obstacleThresholdCm);
  logPrintf("[PREFS] autoMode = %s\n", autoMode ? "true" : "false");
  logPrintf("[PREFS] lastSuccessfulSyncEpoch = %lld\n", (long long)lastSuccessfulSyncEpoch);

  setupPins();
  setupBacklight();
  setupStepper();
  setupDisplayAndTouch();

  drawStaticUI();
  updateDynamicUI(true);


  // --- NEW --- Start dedicated motor task on Core 0.
  xTaskCreatePinnedToCore(
    Core0MotorTask,
    "MotorTask",
    6144,
    nullptr,
    3,
    &motorTaskHandle,
    0
  );

  logPrintln("[SETUP] Calling beginTimeSync()...");
  beginTimeSync();
}

// ============================================================
// ====================== LOOP (CORE 1) ========================
// ============================================================

void loop() {
  uiLoopCounter++;

  serviceTimeSync();

  if (ENABLE_OTA_FEATURE) {
    serviceArduinoOTA();
  }

  if (ENABLE_TELNET_LOGGER) {
    serviceTelnet();
  }

  handleMotionCompletion();
  serviceRetryClose();
  serviceScheduledEvents();
  serviceWeeklySync();

  handleTouch();
  updateBacklightPolicy();

  if (millis() - lastUiDrawMs >= UI_REFRESH_MS) {
    lastUiDrawMs = millis();
    updateDynamicUI(false);
  }

  if (ENABLE_DEBUG_LOGS) {
    static uint32_t lastHeartbeatMs = 0;
    if (millis() - lastHeartbeatMs >= 5000) {
      lastHeartbeatMs = millis();

      logPrintln("---------- HEARTBEAT ----------");
      logPrintf("doorState = %s\n", doorStateString().c_str());
      logPrintf("syncState = %s\n", syncStateString().c_str());
      logPrintf("WiFi.status = %s\n", wifiStatusToString(WiFi.status()).c_str());
      logPrintf("stepper current = %ld\n", getStepperCurrentPosition());
      logPrintf("stepper target = %ld\n", getStepperTargetPosition());
      logPrintf("distance cm = %.1f\n", lastDistanceCm);
      logPrintf("sonar state = %s\n", sonarStateString(sonarDebugState).c_str());
      logPrintf("sonar echo pin = %s\n", sonarEchoPinLive ? "HIGH" : "LOW");
      logPrintf("sonar raw pulse us = %lu\n", (unsigned long)lastEchoPulseUs);
      logPrintf("sonar good count = %lu\n", (unsigned long)sonarSuccessCount);
      logPrintf("sonar timeout rise = %lu\n", (unsigned long)sonarTimeoutRiseCount);
      logPrintf("sonar timeout fall = %lu\n", (unsigned long)sonarTimeoutFallCount);
      logPrintf("sonar last good age ms = %lu\n",
                (unsigned long)((sonarLastGoodMs == 0) ? 0 : (millis() - sonarLastGoodMs)));
      logCurrentTime("[HB]");
    }
  }

  if (millis() - lastLoopReportMs >= 2000) {
    lastLoopReportMs = millis();

    uint32_t motorLoops = motorLoopCounter;
    motorLoopCounter = 0;

    Serial.print("[LOOP RATE] Core1 loops/sec = ");
    Serial.println(uiLoopCounter / 2);

    Serial.print("[LOOP RATE] Core0 loops/sec = ");
    Serial.println(motorLoops / 2);

    uiLoopCounter = 0;
  }

  delay(1);
}