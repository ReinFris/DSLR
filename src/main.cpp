/*
 * TMC2209 Stepper Motor with AS5600 Encoder for Arduino Uno
 *
 * ============================================================================
 * PIN DEFINITIONS - ARDUINO UNO
 * ============================================================================
 *
 * TMC2209 Stepper Driver -> Arduino Uno
 * ----------------------------------------
 * STEP     -> Digital Pin 5
 * DIR      -> Digital Pin 4
 * EN       -> Digital Pin 6 (active LOW - pull LOW to enable driver)
 * DIAG     -> Digital Pin 3 (StallGuard detection, INT1, interrupt-capable)
 * PDN_UART -> Digital Pins 7 & 8 (see UART wiring below)
 * VCC_IO   -> 5V (Arduino logic level)
 * GND      -> GND
 * VM       -> Motor power supply (4.75V - 29V, separate from Arduino)
 * A1, A2, B1, B2 -> Stepper motor coils
 *
 * TMC2209 UART Communication (for sensorless homing)
 * ---------------------------------------------------
 * SW_RX    -> Digital Pin 7 (connect via 1kΩ resistor to PDN_UART)
 * SW_TX    -> Digital Pin 8 (connect directly to PDN_UART)
 * NOTE: A 1kΩ resistor MUST be placed between SW_RX (pin 7) and PDN_UART
 *       SW_TX (pin 8) connects directly to PDN_UART
 *       Both pins connect to the same PDN_UART pin on TMC2209
 *
 * AS5600 Magnetic Encoder -> Arduino Uno
 * ----------------------------------------
 * VCC    -> 5V (or 3.3V if available on your Uno)
 * GND    -> GND
 * SDA    -> A4 (I2C SDA - FIXED on Arduino Uno)
 * SCL    -> A5 (I2C SCL - FIXED on Arduino Uno)
 *
 * Built-in LED
 * ----------------------------------------
 * LED    -> Digital Pin 13 (onboard LED, used for status indication)
 *
 * ============================================================================
 * IMPORTANT NOTES
 * ============================================================================
 *
 * 1. Arduino Uno I2C pins are FIXED:
 *    - A4 = SDA (cannot be changed)
 *    - A5 = SCL (cannot be changed)
 *
 * 2. AS5600 Hardware Setup:
 *    - Place a diametric magnet above the AS5600 sensor (2-3mm distance)
 *    - The magnet should be centered over the sensor
 *    - Magnet polarity: one pole facing the sensor
 *    - Recommended magnet: 6mm diameter x 2-3mm thickness
 *
 * 3. Sensorless Homing (StallGuard):
 *    - DIAG pin must be connected to pin 3 (interrupt-capable)
 *    - StallGuard detects motor stall when hitting physical limits
 *    - Startup sequence: Move until stall, reverse, stall, then back off
 *    - STALL_VALUE (10) controls sensitivity: higher = less sensitive
 *
 * 4. Power Supply:
 *    - TMC2209 VCC_IO: 5V from Arduino
 *    - TMC2209 VM: Separate motor power supply (typically 12V or 24V)
 *    - AS5600 VCC: 5V from Arduino (AS5600 supports 3.3V-5V)
 *    - Always connect grounds together (Arduino GND, motor PSU GND)
 *
 * 5. Serial Monitor:
 *    - Baud rate: 115200
 *    - Displays: Encoder angle, Degrees, Rotation count, Magnet status
 */

#include <Arduino.h>
#include <Wire.h>
#include <TMCStepper.h>
#include <SoftwareSerial.h>
#include <AccelStepper.h>
#include "EncoderReader.h"

// ============================================================================
// PIN ASSIGNMENTS FOR ARDUINO UNO
// ============================================================================

// Stepper motor control pins
#define STEP_PIN 5   // Digital pin 5 - Step pulse for TMC2209
#define DIR_PIN 4    // Digital pin 4 - Direction control for TMC2209
#define ENABLE_PIN 6 // Digital pin 6 - Enable pin (LOW = enabled, HIGH = disabled)
#define DIAG_PIN 3   // Digital pin 3 - StallGuard DIAG (INT1 - interrupt-capable)

// TMC2209 UART pins (SoftwareSerial)
#define SW_RX 7 // Digital pin 7 - RX via 1kΩ resistor to PDN_UART
#define SW_TX 8 // Digital pin 8 - TX directly to PDN_UART

// Status LED
#define LED_PIN 13 // Digital pin 13 - Built-in LED on Arduino Uno

// I2C pins for AS5600 encoder
// NOTE: These are FIXED on Arduino Uno and cannot be changed
// Do NOT attempt to use other pins for I2C on Uno - it will not work
#define I2C_SDA_PIN A4 // Analog pin A4 - I2C SDA (FIXED on Uno)
#define I2C_SCL_PIN A5 // Analog pin A5 - I2C SCL (FIXED on Uno)

// Joystick pins
#define JOYSTICK_VRX A0 // Analog pin A0 - Joystick X-axis (VRx)
#define JOYSTICK_SW 2   // Digital pin 2 - Joystick button (INT0, interrupt-capable)

// ============================================================================
// MOTOR & TMC2209 CONFIGURATION
// ============================================================================

// Motor parameters
#define STEPS_PER_REV 200 // Standard stepper motor (1.8° per step)
#define MICROSTEPS 2      // Microsteps setting for TMC2209
#define STEP_DELAY 100    // Microseconds between steps (for homing) 700?

// TMC2209 Configuration
#define R_SENSE 0.11f       // SilentStepStick series use 0.11 Ohm
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address (MS1 and MS2 to GND)
#define RUN_CURRENT 500     // Motor current in mA (adjust for your motor)
#define STALL_VALUE 24      // StallGuard threshold [0..255] (lower = more sensitive)
#define TOFF_VALUE 4        // Off time setting [1..15]

// AccelStepper Configuration
#define MAX_SPEED 9000.0                         // Maximum speed in steps/second
#define ACCELERATION 10000.0                     // Acceleration in steps/second^2
#define DEFAULT_SPEED 1000.0                     // Default speed for moves
#define TOTAL_STEPS (STEPS_PER_REV * MICROSTEPS) // Total steps per revolution

// Movement parameters
#define HOMING_BACKOFF_STEPS 500 // Steps to back off after hitting stall

// Joystick parameters
#define JOYSTICK_CENTER 512         // Center position (ADC value 0-1023)
#define JOYSTICK_DEADZONE 50        // Deadzone around center (prevents drift)
#define JOYSTICK_HALF_THRESHOLD 150 // Threshold for half vs full speed
#define JOYSTICK_MIN 0              // Minimum ADC value
#define JOYSTICK_MAX 800            // Maximum ADC value

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

// Create SoftwareSerial for TMC2209 UART communication
SoftwareSerial SoftSerial(SW_RX, SW_TX);

// Create TMC2209 driver object
TMC2209Stepper TMC_Driver(&SoftSerial, R_SENSE, DRIVER_ADDRESS);

// Create AccelStepper object using DRIVER mode (step/direction interface)
// AccelStepper stepper(interface, stepPin, directionPin)
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Create encoder instance
EncoderReader encoder;

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

volatile bool stallDetected = false; // Flag set by interrupt
bool shaftVal = false;               // Direction: false = forward, true = reverse
bool startup = true;                 // Run homing on startup

// Homing limit positions
long firstLimitPosition = 0;
long secondLimitPosition = 0;
long minLimit = 0;
long maxLimit = 0;
long centerPosition = 0;
bool isHomed = false;

// Update interval for serial output (milliseconds)
const unsigned long PRINT_INTERVAL = 100; // 10Hz update rate
unsigned long lastPrintTime = 0;

// ============================================================================
// INTERRUPT SERVICE ROUTINE - StallGuard Detection
// ============================================================================

// ISR for DIAG pin - triggered when stall is detected
void stallInterruptHandler()
{
  stallDetected = true;
  // Note: Keep ISR minimal - detailed debugging happens in main loop
}

// ============================================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================================

// Move motor by relative steps (positive = forward, negative = backward)
void moveRelative(long steps)
{
  stepper.move(steps);
  Serial.print(F("Moving by "));
  Serial.print(steps);
  Serial.println(F(" steps"));
}

// Rotate motor by degrees (based on TOTAL_STEPS configuration)
void rotateDegrees(float degrees)
{
  long steps = (long)((degrees / 360.0) * TOTAL_STEPS);
  stepper.move(steps);
  Serial.print(F("Rotating "));
  Serial.print(degrees);
  Serial.print(F("° ("));
  Serial.print(steps);
  Serial.println(F(" steps)"));
}

// Generate step pulses for motor movement (OLD - kept for homing compatibility)
// Used during sensorless homing where blocking movement is needed
void motor(int steps, int stepDelay)
{
  digitalWrite(ENABLE_PIN, LOW);   // Ensure driver is enabled
  digitalWrite(DIR_PIN, shaftVal); // Set direction via hardware pin

  for (int i = 0; i < steps; i++)
  {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelay);

    // Check for stall during movement
    if (stallDetected)
    {
      break;
    }
  }
}

// Helper function: Move until stall is detected (used for finding limits)
// Returns the position where stall was detected
// ignoreSteps: number of steps to move before checking for stall (to clear "hot" stall signal)
long moveUntilStall(float speed, const char *stepLabel, long ignoreSteps = 0)
{
  Serial.print(F("\n["));
  Serial.print(stepLabel);
  Serial.println(F("] Finding limit..."));

  stepper.setSpeed(speed);
  long startPosition = stepper.currentPosition();

  // Clear any pending stall flag
  stallDetected = false;

  while (!stallDetected)
  {
    stepper.runSpeed();
    encoder.update();

    // Ignore stall detection for the first ignoreSteps
    if (abs(stepper.currentPosition() - startPosition) < ignoreSteps)
    {
      stallDetected = false; // Keep clearing the flag during ignore period
    }

    // Serial printing disabled during movement to prevent stuttering
    // Printing blocks execution and interrupts smooth stepper pulses
  }

  long stallPosition = stepper.currentPosition();
  Serial.print(F("["));
  Serial.print(stepLabel);
  Serial.print(F("] Limit found at position: "));
  Serial.println(stallPosition);

  stepper.setSpeed(0);
  delay(100);

  return stallPosition;
}

// Helper function: Move to target position and print progress
void moveToPosition(long targetPosition, float speed, unsigned long printInterval)
{
  long stepsToMove = targetPosition - stepper.currentPosition();
  float moveSpeed = (stepsToMove > 0) ? speed : -speed;
  stepper.setSpeed(moveSpeed);

  while ((stepsToMove > 0) ? (stepper.currentPosition() < targetPosition) : (stepper.currentPosition() > targetPosition))
  {
    stepper.runSpeed();
    encoder.update();

    // Serial printing disabled during movement to prevent stuttering
  }

  stepper.setSpeed(0);
}

// Homing routine - uses StallGuard to find limits on both ends, then returns to center
// Uses runSpeed() for constant speed movement with no acceleration (5000 microsteps/s)
void homeX()
{
  const float HOMING_SPEED = 800.0; // works OK with #define MICROSTEPS 2

  Serial.println(F("\n=== HOMING SEQUENCE START ==="));
  Serial.print(F("Homing speed: "));
  Serial.print(HOMING_SPEED);
  Serial.println(F(" microsteps/s (~312 full steps/s)"));

  stallDetected = false;
  delay(100);

  Serial.println(F("[Home] Attaching interrupt..."));
  attachInterrupt(digitalPinToInterrupt(DIAG_PIN), stallInterruptHandler, RISING);
  delay(10);
  stallDetected = false;

  // STEP 1: Find first limit (negative direction)
  moveUntilStall(-HOMING_SPEED, "Step 1", 50);

  firstLimitPosition = 0;
  stepper.setCurrentPosition(0);
  Serial.print(F("[Step 1] First limit set at position: "));
  Serial.println(firstLimitPosition);

  // STEP 2: Find second limit (positive direction)
  // Ignore first 100 steps to clear "hot" stall signal from Step 1
  moveUntilStall(HOMING_SPEED, "Step 2", 100);

  secondLimitPosition = stepper.currentPosition();

  // Calculate limits and center
  minLimit = (firstLimitPosition < secondLimitPosition) ? firstLimitPosition : secondLimitPosition;
  maxLimit = (firstLimitPosition < secondLimitPosition) ? secondLimitPosition : firstLimitPosition;
  centerPosition = (minLimit + maxLimit) / 2;

  Serial.println(F("\n=== HOMING RESULTS ==="));
  Serial.print(F("First limit:  "));
  Serial.println(firstLimitPosition);
  Serial.print(F("Second limit: "));
  Serial.println(secondLimitPosition);
  Serial.print(F("Min limit:    "));
  Serial.println(minLimit);
  Serial.print(F("Max limit:    "));
  Serial.println(maxLimit);
  Serial.print(F("Range:        "));
  Serial.print(maxLimit - minLimit);
  Serial.println(F(" steps"));
  Serial.print(F("Center:       "));
  Serial.println(centerPosition);

  detachInterrupt(digitalPinToInterrupt(DIAG_PIN));
  Serial.println(F("[Home] Interrupt detached"));

  // STEP 3: Move to center position using full AccelStepper capabilities
  Serial.println(F("\n[Step 3] Moving to center with acceleration..."));
  Serial.print(F("Current position: "));
  Serial.println(stepper.currentPosition());
  Serial.print(F("Target position:  "));
  Serial.println(centerPosition);
  Serial.print(F("Steps to center:  "));
  Serial.println(centerPosition - stepper.currentPosition());
  Serial.print(F("Max speed:        "));
  Serial.print(stepper.maxSpeed());
  Serial.println(F(" steps/s"));
  Serial.print(F("Acceleration:     "));
  Serial.print(stepper.acceleration());
  Serial.println(F(" steps/s²"));

  // Use moveTo() for accelerated movement to center
  stepper.moveTo(centerPosition);

  // Run motor with acceleration until target is reached
  // NO Serial.print() here - blocking I/O causes stuttering!
  while (stepper.distanceToGo() != 0)
  {
    stepper.run();
    encoder.update();
  }

  Serial.print(F("[Step 3] Arrived at center! Position="));
  Serial.println(stepper.currentPosition());
  Serial.print(F("Final speed: "));
  Serial.print(stepper.speed());
  Serial.println(F(" steps/s"));

  isHomed = true;
  Serial.println(F("\n=== HOMING COMPLETE ===\n"));
}

// ============================================================================
// SETUP
// ============================================================================

void setup()
{
  // Initialize serial communication FIRST
  Serial.begin(115200);
  while (!Serial && millis() < 3000)
  {
    // Wait for serial port to connect (max 3 seconds)
    delay(10);
  }

  Serial.println(F("\n\nTMC2209 + AS5600"));
  Serial.println(F("Uno v2.0 - Homing"));

  // Initialize stepper motor pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(DIAG_PIN, INPUT); // StallGuard DIAG pin (open-drain from TMC2209)

  // Initialize joystick pins
  pinMode(JOYSTICK_VRX, INPUT);
  pinMode(JOYSTICK_SW, INPUT_PULLUP); // Use internal pull-up for button
  Serial.println(F("[HW] Joystick initialized on A0 and D2"));

  // IMPORTANT: Enable driver in hardware BEFORE UART communication
  digitalWrite(ENABLE_PIN, LOW); // Enable driver (active LOW)
  Serial.println(F("[HW] Driver ON"));

  // Initialize AccelStepper
  // Using safe, reliable speeds per AccelStepper manual (max 1000 steps/s)
  stepper.setMaxSpeed(1000.0);     // Safe reliable speed (per manual)
  stepper.setAcceleration(2000.0); // Conservative acceleration (2x speed)
  stepper.setCurrentPosition(0);   // Set current position as zero
  Serial.print(F("[ACCEL] MaxSpd="));
  Serial.print(stepper.maxSpeed());
  Serial.print(F(" Accel="));
  Serial.print(stepper.acceleration());
  Serial.print(F(" Steps/rev="));
  Serial.println(TOTAL_STEPS);

  // Check initial DIAG pin state
  Serial.print(F("[HW] DIAG initial="));
  Serial.println(digitalRead(DIAG_PIN));

  // DON'T attach interrupt yet - wait until homing starts
  // attachInterrupt(digitalPinToInterrupt(DIAG_PIN), stallInterruptHandler, RISING);
  Serial.println(F("[HW] INT will attach in homeX"));

  // Blink LED 3 times on startup
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }

  // Initialize TMC2209 UART communication
  Serial.println(F("\n[TMC] Init UART"));
  SoftSerial.begin(115200);
  TMC_Driver.beginSerial(115200);
  delay(100);

  // Configure TMC2209
  TMC_Driver.begin();
  TMC_Driver.toff(TOFF_VALUE);
  TMC_Driver.blank_time(24);
  TMC_Driver.rms_current(RUN_CURRENT);
  TMC_Driver.microsteps(MICROSTEPS);

  // IMPORTANT: Enable SpreadCycle for StallGuard to work
  // StallGuard does NOT work in StealthChop mode!
  // TMC_Driver.en_spreadCycle(true); // Enable SpreadCycle (required for StallGuard)
  TMC_Driver.pwm_autoscale(true);

  // Test UART communication
  Serial.print(F("[TMC] UART: "));
  uint32_t drv_status = TMC_Driver.DRV_STATUS();
  if (drv_status == 0 || drv_status == 0xFFFFFFFF)
  {
    Serial.println(F("FAIL"));
    Serial.println(F("  Chk pins 7,8->PDN"));
  }
  else
  {
    Serial.print(F("OK 0x"));
    Serial.println(drv_status, HEX);
  }

  // Configure StallGuard for sensorless homing
  TMC_Driver.TCOOLTHRS(0xFFFFF);
  TMC_Driver.semin(0);
  TMC_Driver.semax(2);
  TMC_Driver.sedn(0b01);
  TMC_Driver.SGTHRS(STALL_VALUE);

  Serial.print(F("[TMC] I="));
  Serial.print(RUN_CURRENT);
  Serial.print(F("mA uStep="));
  Serial.print(MICROSTEPS);
  Serial.print(F(" SG="));
  Serial.println(STALL_VALUE);

  // Verify StallGuard configuration
  Serial.print(F("[TMC] SGTHRS read="));
  Serial.print(TMC_Driver.SGTHRS());
  Serial.print(F(" TCOOLTHRS=0x"));
  Serial.print(TMC_Driver.TCOOLTHRS(), HEX);
  Serial.print(F(" SpreadCycle="));
  Serial.println(TMC_Driver.en_spreadCycle() ? F("ON") : F("OFF"));

  // Initialize I2C
  Serial.println(F("[I2C] Init A4/A5"));
  Wire.begin();
  Wire.setClock(100000);

  // Recover I2C bus
  for (int i = 0; i < 10; i++)
  {
    Wire.beginTransmission(0x36);
    Wire.endTransmission();
    delay(10);
  }

  // Test AS5600
  Wire.beginTransmission(0x36);
  if (Wire.endTransmission() == 0)
  {
    Serial.print(F("[AS5600] "));
    encoder.begin();
    delay(100);

    if (encoder.detectMagnet())
    {
      if (encoder.magnetTooStrong())
        Serial.println(F("Strong!"));
      else if (encoder.magnetTooWeak())
        Serial.println(F("Weak!"));
      else
        Serial.println(F("OK"));
    }
    else
      Serial.println(F("No mag!"));
  }
  else
    Serial.println(F("[AS5600] Not found"));

  Serial.println(F("\n--- Ready ---"));
  Serial.println(F("Homing on startup..."));
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop()
{
  // Run homing sequence on startup
  if (startup)
  {
    startup = false;
    homeX();
  }

  // Update encoder state (detects zero crossings)
  encoder.update();

  // ============================================================================
  // JOYSTICK CONTROL (after homing)
  // ============================================================================

  // if (isHomed)
  // {
  //   // Read joystick VRX (X-axis)
  //   int vrxValue = analogRead(JOYSTICK_VRX);

  //   // Motor control - use moveTo/run for acceleration control
  //   long currentPos = stepper.currentPosition();
  //   long targetPosition;
  //   float maxSpeedSetting;

  //   if (abs(vrxValue - JOYSTICK_CENTER) > JOYSTICK_DEADZONE)
  //   {
  //     // Joystick is deflected - determine speed and direction
  //     int deflection = abs(vrxValue - JOYSTICK_CENTER);

  //     // Set max speed based on deflection
  //     if (deflection > JOYSTICK_HALF_THRESHOLD)
  //       maxSpeedSetting = MAX_SPEED;
  //     else
  //       maxSpeedSetting = MAX_SPEED / 2.0; // Half speed

  //     // Calculate target position far in the direction of movement
  //     if (vrxValue > JOYSTICK_CENTER + JOYSTICK_DEADZONE)
  //     {
  //       // Forward - set target to max limit
  //       targetPosition = maxLimit;
  //     }
  //     else
  //     {
  //       // Backward - set target to min limit
  //       targetPosition = minLimit;
  //     }
  //   }
  //   else
  //   {
  //     // Joystick centered - stop at current position with deceleration
  //     targetPosition = currentPos;
  //     maxSpeedSetting = MAX_SPEED;
  //   }

  //   // Apply settings and move
  //   stepper.setMaxSpeed(maxSpeedSetting);
  //   stepper.moveTo(targetPosition);
  // }

  // IMPORTANT: Must call stepper.run() regularly for AccelStepper to work!
  // This handles acceleration, deceleration, and stepping
  stepper.run();

  // ============================================================================
  // SERIAL PRINTING (every PRINT_INTERVAL ms)
  // ============================================================================

  // unsigned long currentTime = millis();
  // if (isHomed && (currentTime - lastPrintTime >= PRINT_INTERVAL))
  // {
  //   lastPrintTime = currentTime;

  //   // Read joystick value for display
  //   int vrxValue = analogRead(JOYSTICK_VRX);

  //   // Print formatted output
  //   Serial.print(F("VRX:"));
  //   Serial.print(vrxValue);
  //   Serial.print(F(" | Pos:"));
  //   Serial.print(stepper.currentPosition());
  //   Serial.print(F(" | Spd:"));
  //   Serial.print(stepper.speed(), 0);
  //   Serial.print(F(" | Lim:["));
  //   Serial.print(minLimit);
  //   Serial.print(F(","));
  //   Serial.print(maxLimit);
  //   Serial.print(F("] | Enc:"));
  //   Serial.print(encoder.getRawAngle());
  //   Serial.println();
  // }
}
