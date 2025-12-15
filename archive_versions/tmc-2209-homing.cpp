// TMC2209 Test Script - adapted to use existing wiring

#include <Arduino.h>
#include <TMCStepper.h>
#include <SoftwareSerial.h>

// Commenting out for test
// #include <Wire.h>
// #include <AccelStepper.h>
// #include "EncoderReader.h"

// AS5600 Encoder I2C Pins (for reference - Wire library uses these by default)
// For Arduino Uno/Nano: SDA = A4, SCL = A5
// For Arduino Mega: SDA = 20, SCL = 21
// Note: Most AS5600 modules have built-in pull-up resistors
#define SDA_PIN A4 // I2C Data
#define SCL_PIN A5 // I2C Clock

// TMC2209 Stepper Driver Pins
#define STEP_PIN 5
#define DIR_PIN 4
#define ENABLE_PIN 6
#define SW_RX 7      // SoftwareSerial RX (connect to TMC2209 PDN_UART via 1k resistor to SW_TX)
#define SW_TX 8      // SoftwareSerial TX (connect to TMC2209 PDN_UART)
#define STALLGUARD 3 // DIAG pin from TMC2209

// TMC2209 Configuration
#define R_SENSE 0.11f       // SilentStepStick series use 0.11 Ohm
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define RUN_CURRENT 700     // Motor current in mA (adjust based on your motor specs)
#define MICROSTEPS 256      // Microsteps setting (1, 2, 4, 8, 16, 32, 64, 128, 256)

// StallGuard Configuration
#define STALL_VALUE 10 // [0..255] - Higher = less sensitive, Lower = more sensitive
#define TOFF_VALUE 5   // [1..15] Off time setting

// Other I/O Pins
#define LED_PIN 13
#define JOYSTICK_VRX A0
#define JOYSTICK_SW 2
#define STEPS_PER_REV 200
#define TOTAL_STEPS (STEPS_PER_REV * MICROSTEPS)

// Startup sequence configuration
#define STARTUP_SPEED 160000 // Speed for startup movement (VACTUAL units)

// Startup sequence states
enum StartupState
{
  STARTUP_MOVING_FIRST_SIDE,
  STARTUP_MOVING_SECOND_SIDE,
  STARTUP_MOVING_BACK,
  STARTUP_COMPLETE
};

// Oscillation state
enum OscillationState
{
  OSC_FORWARD,
  OSC_BACKWARD
};

// Create SoftwareSerial for TMC2209 UART communication
SoftwareSerial SoftSerial(SW_RX, SW_TX);

// Create TMC2209 driver object
TMC2209Stepper TMC_Driver(&SoftSerial, R_SENSE, DRIVER_ADDRESS);

// Global state variables
StartupState startupState = STARTUP_MOVING_FIRST_SIDE;
OscillationState oscState = OSC_FORWARD;
int32_t currentPosition = 0;         // Track motor position in steps
int32_t oscillationSteps = 0;        // Counter for oscillation steps
volatile bool stallDetected = false; // Flag set by interrupt
volatile uint32_t lastStallTime = 0; // Debounce stall detection

//== Interrupt Service Routine ==================================================================

// ISR for DIAG pin - triggered when stall is detected
void stallInterruptHandler()
{
  stallDetected = true;

  // Serial.print("Status: SG=");
  // Serial.print(TMC_Driver.SG_RESULT(), DEC);
  // Serial.print(" STALL=");
  // Serial.print(TMC_Driver.SG_RESULT() < STALL_VALUE ? 1 : 0);
  // Serial.print(" I=");
  // Serial.print(TMC_Driver.cs2rms(TMC_Driver.cs_actual()), DEC);
  // Serial.println("mA");
}

//== Setup ======================================================================================

void setup()
{

  Serial.begin(115200); // initialize hardware serial for debugging
  while (!Serial)
    ;                       // Wait for serial port to connect
  SoftSerial.begin(115200); // initialize software serial for UART motor control

  // IMPORTANT: Enable driver in hardware BEFORE UART communication
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // Enable driver in hardware (active LOW)
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  // Attach interrupt to DIAG pin (pin 3 supports interrupts on most Arduino boards)
  // RISING edge: when DIAG goes HIGH, a stall is detected
  attachInterrupt(digitalPinToInterrupt(STALLGUARD), stallInterruptHandler, RISING);

  Serial.println("TMC2209 StallGuard Test with Joystick Control");
  Serial.println("Wiring: EN=6, DIR=4, STEP=5, SW_RX=7, SW_TX=8");
  Serial.println("IMPORTANT: VIO to 5V, GND to GND, 1k resistor between pins 7 and 8 to PDN_UART");
  delay(500);

  TMC_Driver.beginSerial(115200); // Initialize UART

  TMC_Driver.begin();          // UART: Init SW UART (if selected) with default 115200 baudrate
  TMC_Driver.toff(5);          // Enables driver in software
  TMC_Driver.rms_current(770); // Set motor RMS current (1770mA)
  TMC_Driver.microsteps(256);  // Set microsteps to 256

  // StallGuard configuration
  TMC_Driver.TCOOLTHRS(0xFFFFF);  // 20bit max - Enable StallGuard (velocity threshold)
  TMC_Driver.semin(5);            // Minimum StallGuard value for smart current control
  TMC_Driver.semax(2);            // Maximum StallGuard value for smart current control
  TMC_Driver.sedn(0b00);          // Current down step speed
  TMC_Driver.SGTHRS(STALL_VALUE); // StallGuard threshold [0..255]

  TMC_Driver.en_spreadCycle(false); // Toggle spreadCycle (false = use StealthChop)
  TMC_Driver.pwm_autoscale(true);   // Needed for stealthChop

  Serial.println("TMC2209 configured with StallGuard!");
  Serial.println("");
  Serial.println("=== STARTUP SEQUENCE ===");
  Serial.println("Moving to first side until stall detected...");

  // Start moving in positive direction
  TMC_Driver.VACTUAL(STARTUP_SPEED);

  delay(500);
}

//== Loop ========================================================================================

void loop()
{
  static uint32_t last_time = 0;
  static int32_t speed = 0; // Motor speed in VACTUAL units
  uint32_t ms = millis();

  // Read DIAG pin for stall detection
  bool diagPin = digitalRead(STALLGUARD);

  //== STARTUP SEQUENCE HANDLING ==================================================================
  if (startupState != STARTUP_COMPLETE)
  {
    static uint32_t last_diag_check = 0;

    // Check for stall detection via interrupt flag
    if (stallDetected)
    {
      stallDetected = false; // Clear the flag

      Serial.print("SG_RESULT: ");
      Serial.println(TMC_Driver.SG_RESULT());

      // Stall detected via interrupt!
      if (startupState == STARTUP_MOVING_FIRST_SIDE)
      {
        Serial.println(">>> STALL DETECTED on first side!");
        Serial.println(">>> Clearing stall condition...");

        // Clear StallGuard fault by toggling the enable pin
        // A high DIAG pin indicates a stall - must be cleared by disabling
        // and re-enabling the driver via the ENN (enable) pin
        // digitalWrite(ENABLE_PIN, HIGH); // Disable driver (active LOW)
        // delay(10);                      // Brief pause to clear the fault
        // digitalWrite(ENABLE_PIN, LOW);  // Re-enable driver

        Serial.println(">>> Reversing direction...");

        // Move to second side (negative direction)
        TMC_Driver.VACTUAL(-STARTUP_SPEED);
        startupState = STARTUP_MOVING_SECOND_SIDE;
      }
      else if (startupState == STARTUP_MOVING_SECOND_SIDE)
      {
        Serial.println(">>> STALL DETECTED on second side!");
        Serial.println(">>> Clearing stall condition...");

        // Clear StallGuard fault by toggling the enable pin
        digitalWrite(ENABLE_PIN, HIGH); // Disable driver (active LOW)
        delay(10);                      // Brief pause to clear the fault
        digitalWrite(ENABLE_PIN, LOW);  // Re-enable driver

        Serial.println(">>> Moving back 1000 steps...");

        // Move back 1000 steps (positive direction)
        TMC_Driver.VACTUAL(STARTUP_SPEED);
        currentPosition = 0; // Reset position counter
        startupState = STARTUP_MOVING_BACK;
      }
    }

    // Handle MOVING_BACK state (time-based movement)
    if (startupState == STARTUP_MOVING_BACK)
    {
      // Check if we've moved 1000 steps (approximate based on time/speed)
      // This is a simple approach - for precise stepping, you'd need step counting
      static uint32_t moveStartTime = 0;
      if (moveStartTime == 0)
      {
        moveStartTime = ms;
      }

      // After sufficient time for ~1000 steps, start oscillation
      if ((ms - moveStartTime) > 2000) // Adjust timing as needed
      {
        TMC_Driver.VACTUAL(0);
        speed = 0;
        moveStartTime = 0;

        Serial.println(">>> STARTUP SEQUENCE COMPLETE!");
        Serial.println(">>> Starting oscillation: 500 steps back and forth");

        // Start oscillation
        TMC_Driver.VACTUAL(STARTUP_SPEED);
        oscillationSteps = 0;
        oscState = OSC_FORWARD;
        startupState = STARTUP_COMPLETE;
      }
    }

    // Print status during startup every 50ms
    if ((ms - last_diag_check) > 50)
    {
      last_diag_check = ms;

      Serial.print("Startup: State=");
      if (startupState == STARTUP_MOVING_FIRST_SIDE)
        Serial.print("FIRST_SIDE");
      else if (startupState == STARTUP_MOVING_SECOND_SIDE)
        Serial.print("SECOND_SIDE");
      else if (startupState == STARTUP_MOVING_BACK)
        Serial.print("MOVING_BACK");
      Serial.print(" DIAG=");
      Serial.println(diagPin);

      Serial.print("Status: SG=");
      Serial.print(TMC_Driver.SG_RESULT(), DEC);
      Serial.print(" STALL=");
      Serial.print(TMC_Driver.SG_RESULT() < STALL_VALUE ? 1 : 0);
      Serial.print(" DIAG=");
      Serial.print(diagPin, DEC);
      Serial.print(" I=");
      Serial.print(TMC_Driver.cs2rms(TMC_Driver.cs_actual()), DEC);
      Serial.println("mA");
    }

    // During startup, skip normal joystick control
    return;
  }

  //== NORMAL OPERATION - OSCILLATION ==============================================================

  // Oscillation logic: move 500 steps back and forth
  static uint32_t last_osc_time = 0;
  static uint32_t osc_step_count = 0;

  // Estimate steps based on time and speed (rough approximation)
  // For more precise control, you would need to count actual step pulses
  if ((ms - last_osc_time) > 10) // Check every 10ms
  {
    last_osc_time = ms;
    osc_step_count++;

    // Roughly estimate 500 steps completed (adjust based on your speed/timing)
    if (osc_step_count >= 100) // ~1 second at 10ms intervals
    {
      osc_step_count = 0;

      // Toggle direction
      if (oscState == OSC_FORWARD)
      {
        Serial.println(">>> Oscillation: Reversing to BACKWARD");
        TMC_Driver.VACTUAL(-STARTUP_SPEED);
        oscState = OSC_BACKWARD;
      }
      else
      {
        Serial.println(">>> Oscillation: Reversing to FORWARD");
        TMC_Driver.VACTUAL(STARTUP_SPEED);
        oscState = OSC_FORWARD;
      }
    }
  }

  // // Handle serial commands (COMMENTED OUT)
  // while (Serial.available() > 0)
  // {
  //   int8_t read_byte = Serial.read();

  //   if (read_byte == '0') // Disable motor
  //   {
  //     Serial.print("Motor ");
  //     if (TMC_Driver.toff() == 0)
  //     {
  //       Serial.print("already ");
  //     }
  //     Serial.println("disabled.");
  //     TMC_Driver.toff(0);
  //     speed = 0;
  //   }
  //   else if (read_byte == '1') // Enable motor
  //   {
  //     Serial.print("Motor ");
  //     if (TMC_Driver.toff() != 0)
  //     {
  //       Serial.print("already ");
  //     }
  //     Serial.println("enabled.");
  //     TMC_Driver.toff(TOFF_VALUE);
  //   }
  //   else if (read_byte == '+') // Increase speed
  //   {
  //     speed += 1000;
  //     if (speed == 0)
  //     {
  //       Serial.println("Hold motor.");
  //     }
  //     else
  //     {
  //       Serial.println("Increase speed.");
  //     }
  //     TMC_Driver.VACTUAL(speed);
  //   }
  //   else if (read_byte == '-') // Decrease speed
  //   {
  //     speed -= 1000;
  //     if (speed == 0)
  //     {
  //       Serial.println("Hold motor.");
  //     }
  //     else
  //     {
  //       Serial.println("Decrease speed.");
  //     }
  //     TMC_Driver.VACTUAL(speed);
  //   }
  // }

  // // Read joystick and control motor speed (COMMENTED OUT)
  // int potVal = analogRead(A0); // Read potentiometer (0-1023)

  // if (potVal <= 500) // In lower half of range turn counter clockwise
  // {                  // (direction depends on motor wiring?)
  //   speed = map(potVal, 0, 500, -200000, 0);
  // }
  // else if (potVal >= 520) // In high half of range turn clockwise
  // {
  //   speed = map(potVal, 520, 1023, 0, 200000);
  // }
  // else // Create a "dead zone" between CW and CCW
  // {    // if 500 < potVal < 520
  //   speed = 0;
  // }

  // // Apply speed to motor (COMMENTED OUT)
  // TMC_Driver.VACTUAL(speed);

  // Print StallGuard results every 100ms
  // if ((ms - last_time) > 100)
  // {
  //   last_time = ms;

  //   // Only print full status when motor is enabled and moving
  //   // if (TMC_Driver.toff() != 0 && speed != 0)
  //   // {
  //   Serial.print("Status: SG=");
  //   Serial.print(TMC_Driver.SG_RESULT(), DEC);
  //   Serial.print(" STALL=");
  //   Serial.print(TMC_Driver.SG_RESULT() < STALL_VALUE ? 1 : 0);
  //   Serial.print(" DIAG=");
  //   Serial.print(diagPin, DEC);
  //   Serial.print(" I=");
  //   Serial.print(TMC_Driver.cs2rms(TMC_Driver.cs_actual()), DEC);
  //   Serial.println("mA");
  //   // }
  // } // end print every 100ms
} // end loop
// void setup()
// {
//   Serial.begin(9600);
//   delay(500);

//   // Initialize LED
//   pinMode(LED_PIN, OUTPUT);
//   digitalWrite(LED_PIN, LOW);

//   // Initialize joystick pins
//   pinMode(JOYSTICK_VRX, INPUT);
//   pinMode(JOYSTICK_SW, INPUT_PULLUP); // Use internal pull-up for button

//   // Initialize motor pins
//   pinMode(ENABLE_PIN, OUTPUT);
//   digitalWrite(ENABLE_PIN, LOW); // Enable driver BEFORE UART config (active LOW)

//   Serial.println("IMPORTANT: Ensure TMC2209 VIO pin is connected to Arduino 5V!");
//   delay(500);

//   // Initialize TMC2209 UART communication
//   softSerial.begin(115200);
//   driver.begin();
//   delay(50); // Allow driver to initialize

//   // Test UART connection
//   Serial.print("TMC2209 UART test: ");
//   uint8_t result = driver.test_connection();
//   if (result == 0) {
//     Serial.println("FAILED - Check wiring!");
//     Serial.println("  - RX (pin 7) via 1k resistor to PDN_UART");
//     Serial.println("  - TX (pin 8) to PDN_UART");
//     Serial.println("  - VIO to 5V, GND to GND");
//   } else {
//     Serial.println("OK");
//   }
//   delay(50);

//   // Configure TMC2209
//   Serial.println("Configuring TMC2209...");
//   driver.toff(5);                    // Enable driver in software (TOFF > 0)
//   delay(10);
//   driver.rms_current(RUN_CURRENT);   // Set motor RMS current in mA
//   delay(10);
//   driver.microsteps(MICROSTEPS);     // Set microsteps
//   delay(10);
//   driver.en_spreadCycle(false);      // Disable spreadCycle (use StealthChop for quiet operation)
//   delay(10);
//   driver.pwm_autoscale(true);        // Enable automatic current scaling
//   delay(10);
//   driver.TCOOLTHRS(0xFFFFF);         // Enable CoolStep (optional, for efficiency)
//   delay(10);

//   // Read back and verify configuration
//   Serial.print("  GCONF: 0x");
//   Serial.println(driver.GCONF(), HEX);
//   Serial.print("  Microsteps: ");
//   Serial.println(driver.microsteps());
//   Serial.print("  RMS Current: ");
//   Serial.print(driver.rms_current());
//   Serial.println(" mA");
//   Serial.print("  DRV_STATUS: 0x");
//   Serial.println(driver.DRV_STATUS(), HEX);

//   Serial.println("TMC2209 initialized");
//   delay(100);

//   stepper.setMaxSpeed(MAX_SPEED);
//   stepper.setAcceleration(ACCELERATION);
//   stepper.setCurrentPosition(0);

//   // Debug mode: Move 16 steps right, then 16 steps left
//   Serial.println("DEBUG: Moving 16 steps right...");
//   stepper.moveTo(16);
//   while (stepper.distanceToGo() != 0)
//   {
//     stepper.run();
//   }
//   delay(500);

//   Serial.println("DEBUG: Moving 16 steps left...");
//   stepper.moveTo(-16);
//   while (stepper.distanceToGo() != 0)
//   {
//     stepper.run();
//   }
//   delay(500);

//   Serial.println("DEBUG: Returning to zero...");
//   stepper.moveTo(0);
//   while (stepper.distanceToGo() != 0)
//   {
//     stepper.run();
//   }
//   delay(500);

//   Serial.println("DEBUG: Complete!");
//   stepper.setCurrentPosition(0); // Reset position to zero after debug

//   Serial.println("HOMING: 1.Move to 1st limit, click. 2.Move to 2nd limit, click.");
//   Serial.println("Controls: Small deflection=half speed, Large=full speed");

//   // Initialize I2C (Wire library)
//   Wire.begin();

//   // Initialize AS5600 encoder
//   encoder.begin(4); // 4 = default I2C direction mode

//   // Check if AS5600 is connected
//   if (encoder.isConnected())
//   {
//     Serial.println("AS5600 OK");
//     // Flash LED 3 times
//     for (int i = 0; i < 3; i++)
//     {
//       digitalWrite(LED_PIN, HIGH);
//       delay(100);
//       digitalWrite(LED_PIN, LOW);
//       delay(100);
//     }
//   }
//   else
//   {
//     Serial.println("AS5600 ERROR - check wiring");
//     // Flash LED rapidly
//     while (1)
//     {
//       digitalWrite(LED_PIN, HIGH);
//       delay(50);
//       digitalWrite(LED_PIN, LOW);
//       delay(50);
//     }
//   }

//   if (encoder.magnetTooStrong())
//     Serial.println("Mag: too close");
//   else if (encoder.magnetTooWeak())
//     Serial.println("Mag: too far");
//   else if (encoder.detectMagnet())
//     Serial.println("Mag: OK");
//   else
//     Serial.println("Mag: none");

//   Serial.println("Ready");
//   delay(500);
// }

// void loop()
// {
//   // IMPORTANT: Must call stepper.run() regularly for AccelStepper to work!
//   // This handles acceleration, deceleration, and stepping
//   // stepper.run();

//   // Update encoder state (detects rotation crossings)
//   encoder.update();

//   // Read joystick VRX (X-axis)
//   int vrxValue = analogRead(JOYSTICK_VRX);

//   // Read joystick button (active LOW)
//   bool buttonPressed = (digitalRead(JOYSTICK_SW) == LOW);

//   // Handle button press for homing sequence
//   static bool lastButtonState = false;
//   if (buttonPressed && !lastButtonState)
//   {
//     long currentPosition = stepper.currentPosition();

//     switch (homingState)
//     {
//     case UNHOMED:
//       firstLimitPosition = currentPosition;
//       homingState = HOMING_FIRST_LIMIT;
//       Serial.print("L1:");
//       Serial.println(firstLimitPosition);
//       break;

//     case HOMING_FIRST_LIMIT:
//       secondLimitPosition = currentPosition;
//       if (firstLimitPosition < secondLimitPosition)
//       {
//         minLimit = firstLimitPosition;
//         maxLimit = secondLimitPosition;
//       }
//       else
//       {
//         minLimit = secondLimitPosition;
//         maxLimit = firstLimitPosition;
//       }
//       homingState = HOMED;
//       Serial.print("L2:");
//       Serial.print(secondLimitPosition);
//       Serial.print(" R:");
//       Serial.println(maxLimit - minLimit);
//       break;

//     case HOMED:
//       homingState = UNHOMED;
//       Serial.println("Rehome");
//       break;
//     }
//   }
//   lastButtonState = buttonPressed;

//   // Motor control - different modes for homing vs homed
//   if (homingState == HOMED)
//   {
//     // HOMED: Use moveTo/run for acceleration control
//     long currentPos = stepper.currentPosition();
//     long targetPosition;
//     float maxSpeedSetting;

//     if (abs(vrxValue - JOYSTICK_CENTER) > JOYSTICK_DEADZONE)
//     {
//       // Joystick is deflected - determine speed and direction
//       int deflection = abs(vrxValue - JOYSTICK_CENTER);

//       // Set max speed based on deflection
//       if (deflection > JOYSTICK_HALF_THRESHOLD)
//         maxSpeedSetting = MAX_SPEED;
//       else
//         maxSpeedSetting = HALF_SPEED;

//       // Calculate target position far in the direction of movement
//       if (vrxValue > JOYSTICK_CENTER + JOYSTICK_DEADZONE)
//       {
//         // Forward - set target to max limit
//         targetPosition = maxLimit;
//       }
//       else
//       {
//         // Backward - set target to min limit
//         targetPosition = minLimit;
//       }
//     }
//     else
//     {
//       // Joystick centered - stop at current position with deceleration
//       targetPosition = currentPos;
//       maxSpeedSetting = MAX_SPEED;
//     }

//     // Apply settings and move
//     stepper.setMaxSpeed(maxSpeedSetting);
//     stepper.moveTo(targetPosition);
//     stepper.run();
//   }
//   else
//   {
//     // NOT HOMED: Use setSpeed/runSpeed for continuous motion during homing
//     float targetSpeed = 0.0;

//     if (abs(vrxValue - JOYSTICK_CENTER) > JOYSTICK_DEADZONE)
//     {
//       // Joystick is deflected - determine speed and direction
//       int deflection = abs(vrxValue - JOYSTICK_CENTER);

//       // Set speed based on deflection
//       float speedStep;
//       if (deflection > JOYSTICK_HALF_THRESHOLD)
//         speedStep = MAX_SPEED;
//       else
//         speedStep = HALF_SPEED;

//       // Apply direction
//       if (vrxValue > JOYSTICK_CENTER + JOYSTICK_DEADZONE)
//         targetSpeed = speedStep; // Forward
//       else
//         targetSpeed = -speedStep; // Backward
//     }

//     // Apply speed
//     stepper.setSpeed(targetSpeed);
//     stepper.runSpeed();
//   }

//   // Print encoder and motor data every 100ms (non-blocking)
//   static unsigned long lastPrint = 0;
//   if (ENABLE_SERIAL_PRINT && (millis() - lastPrint >= 100))
//   {
//     lastPrint = millis();

//     // Get encoder readings
//     uint16_t rawAngle = encoder.getRawAngle();
//     float degrees = encoder.getDegrees();
//     long rotationCount = encoder.getRotationCount();

//     // Get motor status
//     long motorPosition = stepper.currentPosition();

//     // Print formatted output
//     Serial.print("State: ");
//     switch (homingState)
//     {
//     case UNHOMED:
//       Serial.print("UNHOMED");
//       break;
//     case HOMING_FIRST_LIMIT:
//       Serial.print("HOMING");
//       break;
//     case HOMED:
//       Serial.print("HOMED");
//       break;
//     }
//     Serial.print(" | VRX:");
//     Serial.print(vrxValue);
//     Serial.print(" | Pos:");
//     Serial.print(motorPosition);
//     Serial.print(" | Spd:");
//     Serial.print(stepper.speed(), 0);

//     if (homingState == HOMED)
//     {
//       Serial.print(" | Lim:[");
//       Serial.print(minLimit);
//       Serial.print(",");
//       Serial.print(maxLimit);
//       Serial.print("]");
//     }

//     Serial.print(" | Enc:");
//     Serial.print(rawAngle);
//     Serial.println();
//   }

//   // Blink LED based on encoder position (always active)
//   uint16_t rawAngle = encoder.getRawAngle();
//   if (rawAngle < 100) // Near zero position
//   {
//     digitalWrite(LED_PIN, HIGH);
//   }
//   else
//   {
//     digitalWrite(LED_PIN, LOW);
//   }
// }
