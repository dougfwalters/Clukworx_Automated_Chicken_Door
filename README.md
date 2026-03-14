The provided firmware for the automated chicken coop door is designed to handle multi-tasking on the 4" CYD, separating high-precision motor control and sensor polling from the user interface and network logic. Dual core processing allows us to split high priority tasks on the first core, and dedicate the 2nd core to the UI + main loop.   I've had multiple engineering projects now with stepper motors and CYDs due to their cost efficiency, but stepper motors require PWM pulses at a high rate which many ESP32's find difficult to manage while running heavy UI or background features. Below is a summary of the code and 

1. Configuration and Hardware Definitions
Lines 11–131: This block defines all user-adjustable constants and pin assignments.

Networking & Time (11–32): Configures WiFi credentials, Time Zone (CST/CDT), and NTP servers for periodic time synchronization.

Hardware Pins (35–61): Defines pins for the TFT Display (ILI9341), the DRV8825 Stepper Driver (Step, Dir, Enable), and the HC-SR04 Sonar Sensor (Trigger and Echo).

Motion Settings (64–82): Sets realistic speed and acceleration for the coop door and hard-programs the travel limits (e.g., 3050 steps between open and closed positions).

Safety & UI (85–118): Defines the obstacle detection threshold (20 cm) and the refresh rates for the UI (100ms) and sonar polling (60ms).

2. State Management and Global Variables
Lines 135–271: 

State Enums (155–191): The door operates on three primary state machines:

DoorState: Tracks the physical status (e.g., DOOR_OPENING, DOOR_WAITING_RETRY, DOOR_ERROR).

SyncState: Manages the WiFi/NTP connection process.

SonarState: Handles the non-blocking microsecond timing required to trigger the sonar and measure the echo.


Persistence (137, 212–219): Uses the Preferences library to save settings like the current door position and cycle counts so they survive power outages.

3. High-Priority Motor and Sonar Task (Core 0)
Lines 816–854 (Core0MotorTask): 


Parallel Processing: The ESP32’s dual-core capability is used to pin this task to Core 0, ensuring that motor steps and sonar measurements are never interrupted by the UI or WiFi lag.


Non-Blocking Sonar (serviceSonar): Instead of using the standard pulseIn() (which pauses the whole program), the code manually cycles through the trigger and echo pins using micros().


Fast Loop Logic: The task runs at maximum speed to ensure high accuracy for the sonar and smooth steps for the motor, yielding the processor only once every 200 cycles to prevent the watchdog timer from resetting the chip.

4. Logic and Helper Functions
Lines 275–812: 


Safe Stepper Wrappers (374–382): Because two different cores access the stepper motor object, these functions use portENTER_CRITICAL to prevent "race conditions" (data corruption) when updating the motor's position.


WiFi & Time Sync (serviceTimeSync): Manages connecting to the internet, starting the OTA (Over-the-Air) update server, and syncing the internal clock with atomic time.

Motion Control (startMoveTo & handleClosingSafety): Initiates door movement and constantly checks the lastDistanceCm during a close cycle. If an obstacle is detected within the threshold for more than 250ms, the door immediately stops and reopens.

5. User Interface and Main Loop (Core 1)
Lines 857–1600 (setup and loop)

Setup (857–926): Initializes the display, sets up the dual-core task, and triggers the initial time sync.

Main Loop (928–964): Runs on Core 1 and handles the less time-sensitive tasks:


serviceScheduledEvents: Checks if it is currently 5:00 AM (Open) or 10:00 PM (Close).

serviceUI: Polls for touch inputs on the screen and draws the graphical dashboard, including the door motif and live sonar data.


updateBacklightPolicy: Dims the screen after inactivity to save power, but wakes it up if the door moves or the screen is touched. 
