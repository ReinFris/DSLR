/*
 * TMC2209 Stepper Motor with AS5600 Encoder for ESP32 DevKit V1
 *
 * ============================================================================
 * PIN DEFINITIONS - ESP32 DevKit V1
 * ============================================================================
 *
 * TMC2209 Stepper Driver -> ESP32 DevKit V1
 * ------------------------------------------
 * STEP     -> GPIO 26 (PWM capable)
 * DIR      -> GPIO 25 (PWM capable)
 * EN       -> GPIO 33 (active LOW - pull LOW to enable driver)
 * DIAG     -> GPIO 35 (StallGuard detection, input only, interrupt-capable)
 * PDN_UART -> GPIO 16 & 17 (Hardware Serial2, see UART wiring below)
 * VCC_IO   -> 3.3V (ESP32 logic level - IMPORTANT!)
 * GND      -> GND
 * VM       -> Motor power supply (4.75V - 29V, separate from ESP32)
 * A1, A2, B1, B2 -> Stepper motor coils
 *
 * TMC2209 UART Communication (Hardware Serial2)
 * ----------------------------------------------
 * RX (GPIO16) -> Connect via 1kΩ resistor to PDN_UART
 * TX (GPIO17) -> Connect directly to PDN_UART
 * NOTE: A 1kΩ resistor MUST be placed between RX (GPIO16) and PDN_UART
 *       TX (GPIO17) connects directly to PDN_UART
 *       Both pins connect to the same PDN_UART pin on TMC2209
 *       ESP32 uses Hardware Serial2 (no SoftwareSerial needed)
 *
 * AS5600 Magnetic Encoder -> ESP32 DevKit V1
 * -------------------------------------------
 * VCC    -> 3.3V (ESP32 logic level)
 * GND    -> GND
 * SDA    -> GPIO 21 (I2C SDA - default on ESP32, configurable)
 * SCL    -> GPIO 22 (I2C SCL - default on ESP32, configurable)
 *
 * Joystick -> ESP32 DevKit V1
 * -------------------------------------------
 * VRX    -> GPIO 34 (ADC1_CH6, input only, 12-bit ADC)
 * SW     -> GPIO 32 (button with internal pullup)
 * VCC    -> 3.3V
 * GND    -> GND
 *
 * Built-in LED
 * -------------------------------------------
 * LED    -> GPIO 2 (onboard LED on ESP32 DevKit V1)
 *
 * ============================================================================
 * IMPORTANT NOTES
 * ============================================================================
 *
 * 1. VOLTAGE LEVEL - CRITICAL:
 *    - ESP32 operates at 3.3V logic (NOT 5V like Arduino!)
 *    - TMC2209 VCC_IO MUST be connected to 3.3V
 *    - AS5600 supports both 3.3V and 5V (use 3.3V)
 *    - All modules must be 3.3V compatible
 *
 * 2. ESP32 I2C pins are configurable:
 *    - Default GPIO21 = SDA, GPIO22 = SCL
 *    - Can be changed by calling Wire.begin(sda, scl)
 *
 * 3. AS5600 Hardware Setup:
 *    - Place a diametric magnet above the AS5600 sensor (2-3mm distance)
 *    - The magnet should be centered over the sensor
 *    - Magnet polarity: one pole facing the sensor
 *    - Recommended magnet: 6mm diameter x 2-3mm thickness
 *
 * 4. Sensorless Homing (StallGuard):
 *    - DIAG pin connected to GPIO35 (interrupt-capable, input only)
 *    - ESP32 supports interrupts on all GPIO pins
 *    - StallGuard detects motor stall when hitting physical limits
 *    - Startup sequence: Move until stall, reverse, stall, then back off
 *    - STALL_VALUE controls sensitivity: higher = less sensitive
 *
 * 5. Power Supply:
 *    - TMC2209 VCC_IO: 3.3V from ESP32 (CRITICAL!)
 *    - TMC2209 VM: Separate motor power supply (typically 12V or 24V)
 *    - AS5600 VCC: 3.3V from ESP32
 *    - Always connect grounds together (ESP32 GND, motor PSU GND)
 *
 * 6. ESP32 ADC:
 *    - 12-bit resolution (0-4095) vs Arduino's 10-bit (0-1023)
 *    - Joystick calibration values scaled 4x from Arduino version
 *    - Using ADC1 to avoid conflicts with WiFi (if needed in future)
 *
 * 7. Serial Monitor:
 *    - Baud rate: 115200
 *    - Displays: Encoder angle, Degrees, Rotation count, Magnet status
 */

#include <Arduino.h>
#include <Wire.h>
#include <TMCStepper.h>
#include <AccelStepper.h>
#include "EncoderReader.h"

// ============================================================================
// PIN ASSIGNMENTS FOR ESP32 DevKit V1
// ============================================================================

// Stepper motor control pins
// TMC2209 Stepper Driver
#define STEP_PIN 26      // GPIO26 - Step pulse for TMC2209 (PWM capable)
#define DIR_PIN 25       // GPIO25 - Direction control for TMC2209 (PWM capable)
#define ENABLE_PIN 33    // GPIO33 - Enable pin (LOW = enabled, HIGH = disabled)
#define DIAG_PIN 35      // GPIO35 - StallGuard DIAG (input only, interrupt capable)

// TMC2209 UART pins (Hardware Serial2)
#define TMC_UART_RX 16   // GPIO16 - Serial2 RX via 1kΩ resistor to PDN_UART
#define TMC_UART_TX 17   // GPIO17 - Serial2 TX directly to PDN_UART

// Status LED
#define LED_PIN 2        // GPIO2 - Built-in LED on ESP32 DevKit V1

// I2C pins for AS5600 encoder
// NOTE: ESP32 I2C pins are configurable, using default pins
#define I2C_SDA_PIN 21   // GPIO21 - I2C SDA (default on ESP32)
#define I2C_SCL_PIN 22   // GPIO22 - I2C SCL (default on ESP32)

// Joystick pins
#define JOYSTICK_VRX 34  // GPIO34 - Joystick X-axis (ADC1_CH6, input only, 12-bit)
#define JOYSTICK_SW 32   // GPIO32 - Joystick button (with internal pullup)

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
#define SAFETY_BUFFER_STEPS 50   // Safety buffer from hard limits (microsteps)

// Joystick parameters (ESP32 12-bit ADC: 0-4095)
#define JOYSTICK_CENTER 2048        // Center position (12-bit ADC midpoint)
#define JOYSTICK_DEADZONE 200       // Deadzone around center (scaled 4x for 12-bit)
#define JOYSTICK_HALF_THRESHOLD 600 // Threshold for half vs full speed (scaled 4x)
#define JOYSTICK_MIN 0              // Minimum ADC value
#define JOYSTICK_MAX 3200           // Maximum ADC value (scaled 4x, may need calibration)

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

// TMC2209 UART - Forward declare Serial stream (will use Serial2 on ESP32)
// Serial2 will be initialized in setup() with begin(baud, config, rx, tx)
Stream* TMCSerial_ptr = nullptr;

// Create TMC2209 driver object (pointer will be set to Serial2 in setup)
TMC2209Stepper TMC_Driver(TMCSerial_ptr, R_SENSE, DRIVER_ADDRESS);

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

// Homing limit positions
long firstLimitPosition = 0;
long secondLimitPosition = 0;
long minLimit = 0;
long maxLimit = 0;
long safeMinLimit = 0;  // Min limit + safety buffer
long safeMaxLimit = 0;  // Max limit - safety buffer
long centerPosition = 0;
bool isHomed = false;

// Update interval for serial output (milliseconds)
const unsigned long PRINT_INTERVAL = 100; // 10Hz update rate
unsigned long lastPrintTime = 0;

// ============================================================================
// POSITION MARKER SYSTEM - DATA STRUCTURES
// ============================================================================

// System states
enum SystemState
{
  STATE_STARTUP,        // Before homing
  STATE_READY,          // Normal operation
  STATE_DISABLED,       // Motor disabled for manual positioning
  STATE_PLAYBACK_DELAY, // 5-second countdown
  STATE_PLAYBACK        // Executing playback sequence
};

// Encoder calibration (captured during homing)
struct EncoderCalibration
{
  long minLimitRotations;
  uint16_t minLimitRawAngle;
  long minLimitStepperPos;
  long maxLimitRotations;
  uint16_t maxLimitRawAngle;
  long maxLimitStepperPos;
  float stepsPerEncoderUnit;
  float encoderRotationsPerStepperRotation;  // Gear ratio for rotation-based conversion
  bool isCalibrated;
};

// Position markers (encoder readings)
struct PositionMarker
{
  long rotationCount;
  uint16_t rawAngle;
  bool isSet;
};

// Button state tracking
struct ButtonState
{
  bool currentState;
  bool lastState;
  unsigned long pressStartTime;
  unsigned long releaseTime;
  bool isPressed;
  bool longPressTriggered;
};

// Playback sequence state
struct PlaybackState
{
  uint8_t currentMarkerIndex;
  unsigned long delayStartTime;
  unsigned long pauseStartTime;
  bool movingToStart;
  bool isComplete;
};

// Constants
const uint8_t MAX_MARKERS = 3;
const unsigned long DEBOUNCE_DELAY = 50;
const unsigned long LONG_PRESS_TIME = 2000;
const unsigned long SHORT_PRESS_MAX = 500;
const unsigned long PLAYBACK_DELAY_MS = 5000;
const unsigned long MARKER_PAUSE_MS = 500;

// Globals
SystemState currentState = STATE_STARTUP;
EncoderCalibration encoderCal;
PositionMarker markers[MAX_MARKERS];
uint8_t markerCount = 0;
ButtonState button;
PlaybackState playback;

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

long encoderToStepperPosition(long rotationCount, uint16_t rawAngle);

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
  attachInterrupt(DIAG_PIN, stallInterruptHandler, RISING);
  delay(10);
  stallDetected = false;

  // STEP 1: Find first limit (negative direction)
  moveUntilStall(-HOMING_SPEED, "Step 1", 100);

  firstLimitPosition = 0;
  stepper.setCurrentPosition(0);

  // CRITICAL: Reset encoder to create consistent reference frame
  encoder.setRotationCount(0);
  Serial.println(F("[Step 1] Encoder rotation count reset to 0"));

  Serial.print(F("[Step 1] First limit set at position: "));
  Serial.println(firstLimitPosition);

  // Capture encoder position at first limit (min limit)
  encoderCal.minLimitRotations = encoder.getRotationCount();
  encoderCal.minLimitRawAngle = encoder.getRawAngle();
  encoderCal.minLimitStepperPos = 0;
  Serial.print(F("[Calibration] Min limit encoder: rot="));
  Serial.print(encoderCal.minLimitRotations);
  Serial.print(F(" raw="));
  Serial.println(encoderCal.minLimitRawAngle);

  // STEP 2: Find second limit (positive direction)
  // Ignore first 100 steps to clear "hot" stall signal from Step 1
  moveUntilStall(HOMING_SPEED, "Step 2", 100);

  secondLimitPosition = stepper.currentPosition();

  // Capture encoder position at second limit (max limit)
  encoderCal.maxLimitRotations = encoder.getRotationCount();
  encoderCal.maxLimitRawAngle = encoder.getRawAngle();
  encoderCal.maxLimitStepperPos = stepper.currentPosition();
  Serial.print(F("[Calibration] Max limit encoder: rot="));
  Serial.print(encoderCal.maxLimitRotations);
  Serial.print(F(" raw="));
  Serial.println(encoderCal.maxLimitRawAngle);

  // Calculate limits and center
  minLimit = (firstLimitPosition < secondLimitPosition) ? firstLimitPosition : secondLimitPosition;
  maxLimit = (firstLimitPosition < secondLimitPosition) ? secondLimitPosition : firstLimitPosition;
  centerPosition = (minLimit + maxLimit) / 2;

  // Calculate safe limits with buffer
  safeMinLimit = minLimit + SAFETY_BUFFER_STEPS;
  safeMaxLimit = maxLimit - SAFETY_BUFFER_STEPS;

  Serial.println(F("\n=== HOMING RESULTS ==="));
  Serial.print(F("First limit:  "));
  Serial.println(firstLimitPosition);
  Serial.print(F("Second limit: "));
  Serial.println(secondLimitPosition);
  Serial.print(F("Min limit:    "));
  Serial.println(minLimit);
  Serial.print(F("Max limit:    "));
  Serial.println(maxLimit);
  Serial.print(F("Safe min:     "));
  Serial.println(safeMinLimit);
  Serial.print(F("Safe max:     "));
  Serial.println(safeMaxLimit);
  Serial.print(F("Range:        "));
  Serial.print(maxLimit - minLimit);
  Serial.println(F(" steps"));
  Serial.print(F("Center:       "));
  Serial.println(centerPosition);

  // Calculate encoder calibration using rotation-based math
  float encoderMinRot = (float)encoderCal.minLimitRotations +
                        (encoderCal.minLimitRawAngle / 4096.0);
  float encoderMaxRot = (float)encoderCal.maxLimitRotations +
                        (encoderCal.maxLimitRawAngle / 4096.0);
  float encoderRotationRange = encoderMaxRot - encoderMinRot;

  float stepperRotationRange = (maxLimit - minLimit) / (float)(STEPS_PER_REV * MICROSTEPS);

  encoderCal.encoderRotationsPerStepperRotation = encoderRotationRange / stepperRotationRange;

  // Keep legacy calculation for backward compatibility
  long encoderMin = (encoderCal.minLimitRotations * 4096L) + encoderCal.minLimitRawAngle;
  long encoderMax = (encoderCal.maxLimitRotations * 4096L) + encoderCal.maxLimitRawAngle;
  long encoderRange = abs(encoderMax - encoderMin);
  long stepperRange = maxLimit - minLimit;
  encoderCal.stepsPerEncoderUnit = (float)stepperRange / (float)encoderRange;
  encoderCal.isCalibrated = true;

  Serial.println(F("\n=== ENCODER CALIBRATION ==="));
  Serial.print(F("Encoder rotations: "));
  Serial.println(encoderRotationRange, 6);
  Serial.print(F("Stepper rotations: "));
  Serial.println(stepperRotationRange, 6);
  Serial.print(F("Encoder/Stepper ratio: "));
  Serial.println(encoderCal.encoderRotationsPerStepperRotation, 6);
  Serial.print(F("Steps/unit (legacy): "));
  Serial.println(encoderCal.stepsPerEncoderUnit, 6);

  // Verify calibration by converting limits back
  long verifyMin = encoderToStepperPosition(
      encoderCal.minLimitRotations,
      encoderCal.minLimitRawAngle);
  long verifyMax = encoderToStepperPosition(
      encoderCal.maxLimitRotations,
      encoderCal.maxLimitRawAngle);

  Serial.println(F("\n=== CALIBRATION VERIFICATION ==="));
  Serial.print(F("Min limit: "));
  Serial.print(verifyMin);
  Serial.print(F(" (expect ~"));
  Serial.print(safeMinLimit);
  Serial.println(F(")"));
  Serial.print(F("Max limit: "));
  Serial.print(verifyMax);
  Serial.print(F(" (expect ~"));
  Serial.print(safeMaxLimit);
  Serial.println(F(")"));

  detachInterrupt(DIAG_PIN);
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
  currentState = STATE_READY;
  Serial.println(F("\n=== HOMING COMPLETE ===\n"));
}

// ============================================================================
// POSITION MARKER SYSTEM - HELPER FUNCTIONS
// ============================================================================

// Button handler (non-blocking)
// Returns: 0 = no event, 1 = short press, 2 = long press
uint8_t updateButton()
{
  uint8_t event = 0;
  button.currentState = (digitalRead(JOYSTICK_SW) == LOW);
  unsigned long currentTime = millis();

  // Detect press with debouncing
  if (button.currentState && !button.lastState)
  {
    if (currentTime - button.releaseTime > DEBOUNCE_DELAY)
    {
      button.pressStartTime = currentTime;
      button.isPressed = true;
      button.longPressTriggered = false;
    }
  }

  // Detect long press while held
  if (button.isPressed && !button.longPressTriggered)
  {
    if (currentTime - button.pressStartTime >= LONG_PRESS_TIME)
    {
      button.longPressTriggered = true;
      event = 2; // Long press
    }
  }

  // Detect release
  if (!button.currentState && button.lastState)
  {
    if (currentTime - button.pressStartTime > DEBOUNCE_DELAY)
    {
      button.releaseTime = currentTime;
      if (button.isPressed && !button.longPressTriggered)
      {
        unsigned long duration = currentTime - button.pressStartTime;
        if (duration < SHORT_PRESS_MAX)
        {
          event = 1; // Short press
        }
      }
      button.isPressed = false;
    }
  }

  button.lastState = button.currentState;
  return event;
}

// Convert encoder position to stepper position (rotation-based math)
long encoderToStepperPosition(long rotationCount, uint16_t rawAngle)
{
  if (!encoderCal.isCalibrated)
    return 0;

  // Calculate encoder position in rotations (floating point precision)
  float encoderRotations = (float)rotationCount + (rawAngle / 4096.0);
  float encoderMinRotations = (float)encoderCal.minLimitRotations +
                               (encoderCal.minLimitRawAngle / 4096.0);
  float encoderRelativeRotations = encoderRotations - encoderMinRotations;

  // Convert to stepper rotations, then to steps
  float stepperRelativeRotations = encoderRelativeRotations /
                                    encoderCal.encoderRotationsPerStepperRotation;
  long stepperRelativeSteps = (long)(stepperRelativeRotations * STEPS_PER_REV * MICROSTEPS);
  long stepperPos = encoderCal.minLimitStepperPos + stepperRelativeSteps;

  // Clamp to safe range (with buffer)
  if (stepperPos < safeMinLimit)
    stepperPos = safeMinLimit;
  if (stepperPos > safeMaxLimit)
    stepperPos = safeMaxLimit;

  return stepperPos;
}

// Save current encoder position as marker
bool saveMarker()
{
  if (markerCount >= MAX_MARKERS)
  {
    Serial.println(F("[Marker] Already at max (3) markers!"));
    return false;
  }

  long currentRotations = encoder.getRotationCount();
  uint16_t currentRawAngle = encoder.getRawAngle();
  long stepperPos = encoderToStepperPosition(currentRotations, currentRawAngle);

  markers[markerCount].rotationCount = currentRotations;
  markers[markerCount].rawAngle = currentRawAngle;
  markers[markerCount].isSet = true;

  Serial.print(F("[Marker] Saved #"));
  Serial.print(markerCount + 1);
  Serial.print(F(" at stepper pos: "));
  Serial.println(stepperPos);

  markerCount++;
  return true;
}

// Clear all markers
void clearMarkers()
{
  for (uint8_t i = 0; i < MAX_MARKERS; i++)
  {
    markers[i].isSet = false;
  }
  markerCount = 0;
}

// Determine which end position is closest to first marker
long determineStartPosition()
{
  if (markerCount == 0)
    return minLimit;

  long firstMarkerPos = encoderToStepperPosition(
      markers[0].rotationCount, markers[0].rawAngle);

  long distToMin = abs(firstMarkerPos - minLimit);
  long distToMax = abs(firstMarkerPos - maxLimit);

  return (distToMin < distToMax) ? minLimit : maxLimit;
}

// Sort markers based on start position
void sortMarkers(long startPosition)
{
  if (markerCount <= 1)
    return;

  long markerPositions[MAX_MARKERS];
  for (uint8_t i = 0; i < markerCount; i++)
  {
    markerPositions[i] = encoderToStepperPosition(
        markers[i].rotationCount, markers[i].rawAngle);
  }

  bool ascending = (startPosition == minLimit);
  for (uint8_t i = 0; i < markerCount - 1; i++)
  {
    for (uint8_t j = 0; j < markerCount - i - 1; j++)
    {
      bool swap = ascending ? (markerPositions[j] > markerPositions[j + 1]) : (markerPositions[j] < markerPositions[j + 1]);

      if (swap)
      {
        long tempPos = markerPositions[j];
        markerPositions[j] = markerPositions[j + 1];
        markerPositions[j + 1] = tempPos;

        PositionMarker tempMarker = markers[j];
        markers[j] = markers[j + 1];
        markers[j + 1] = tempMarker;
      }
    }
  }
}

// Sort markers by stepper position (ascending order)
void sortMarkersByPosition()
{
  if (markerCount <= 1)
    return;

  // Bubble sort markers by stepper position (ascending)
  for (uint8_t i = 0; i < markerCount - 1; i++)
  {
    for (uint8_t j = 0; j < markerCount - i - 1; j++)
    {
      long posJ = encoderToStepperPosition(markers[j].rotationCount, markers[j].rawAngle);
      long posJplus1 = encoderToStepperPosition(markers[j + 1].rotationCount, markers[j + 1].rawAngle);

      if (posJ > posJplus1)
      {
        PositionMarker temp = markers[j];
        markers[j] = markers[j + 1];
        markers[j + 1] = temp;
      }
    }
  }

  Serial.println(F("[Playback] Markers sorted by position:"));
  for (uint8_t i = 0; i < markerCount; i++)
  {
    long pos = encoderToStepperPosition(markers[i].rotationCount, markers[i].rawAngle);
    Serial.print(F("  Marker "));
    Serial.print(i + 1);
    Serial.print(F(": "));
    Serial.println(pos);
  }
}

// Initialize playback sequence
void initializePlayback()
{
  // Sort markers by position (ascending order)
  sortMarkersByPosition();

  playback.currentMarkerIndex = 0;
  playback.movingToStart = true; // Now means "moving to first marker"
  playback.isComplete = false;

  // Calculate first marker's stepper position
  long firstMarkerPos = encoderToStepperPosition(
      markers[0].rotationCount,
      markers[0].rawAngle);

  Serial.print(F("[Playback] Moving to first marker at stepper pos: "));
  Serial.println(firstMarkerPos);
  stepper.moveTo(firstMarkerPos);
}

// ============================================================================
// POSITION MARKER SYSTEM - STATE HANDLERS
// ============================================================================

void handleStartupState()
{
  static bool homingStarted = false;
  if (!homingStarted)
  {
    homingStarted = true;
    homeX(); // Sets currentState = STATE_READY at end
  }
}

void handleReadyState(uint8_t buttonEvent)
{
  if (buttonEvent == 2)
  { // Long press
    Serial.println(F("\n=== MOTOR DISABLED ==="));
    Serial.println(F("Move motor manually to desired positions"));
    Serial.println(F("SHORT press to save position (max 3)"));
    Serial.println(F("LONG press to start playback sequence\n"));
    digitalWrite(ENABLE_PIN, HIGH);
    clearMarkers();
    currentState = STATE_DISABLED;
  }
}

void handleDisabledState(uint8_t buttonEvent)
{
  if (buttonEvent == 1)
  { // Short press
    saveMarker();
  }
  else if (buttonEvent == 2)
  { // Long press
    if (markerCount == 0)
    {
      Serial.println(F("[Error] No markers saved!"));
      return;
    }
    Serial.println(F("\n=== STARTING PLAYBACK ==="));
    playback.delayStartTime = millis();
    currentState = STATE_PLAYBACK_DELAY;
  }

  // Display position periodically
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200)
  {
    Serial.print(F("DISABLED - Enc: "));
    Serial.print(encoder.getRotationCount());
    Serial.print(F(":"));
    Serial.print(encoder.getRawAngle());
    Serial.print(F(" | Markers: "));
    Serial.print(markerCount);
    Serial.print(F("/"));
    Serial.println(MAX_MARKERS);
    lastPrint = millis();
  }
}

void handlePlaybackDelayState()
{
  unsigned long elapsed = millis() - playback.delayStartTime;

  static unsigned long lastCountdown = 0;
  if (millis() - lastCountdown > 1000)
  {
    Serial.print(F("[Playback] Starting in "));
    Serial.print((PLAYBACK_DELAY_MS - elapsed) / 1000);
    Serial.println(F("s..."));
    lastCountdown = millis();
  }

  if (elapsed >= PLAYBACK_DELAY_MS)
  {
    Serial.println(F("[Playback] BEGIN!"));
    digitalWrite(ENABLE_PIN, LOW); // Enable motor
    delay(100);
    initializePlayback();
    currentState = STATE_PLAYBACK;
  }
}

void handlePlaybackState()
{
  if (stepper.distanceToGo() != 0)
    return; // Still moving

  if (playback.movingToStart)
  {
    // Arrived at first marker, start the sequence
    Serial.println(F("[Playback] At first marker, starting sequence"));
    playback.movingToStart = false;
    playback.pauseStartTime = millis();
    playback.currentMarkerIndex = 1; // Skip first marker (already there)
    return;
  }

  if (playback.pauseStartTime > 0)
  {
    if (millis() - playback.pauseStartTime < MARKER_PAUSE_MS)
    {
      return; // Still pausing
    }
    playback.pauseStartTime = 0;
  }

  if (playback.currentMarkerIndex < markerCount)
  {
    long targetPos = encoderToStepperPosition(
        markers[playback.currentMarkerIndex].rotationCount,
        markers[playback.currentMarkerIndex].rawAngle);

    Serial.print(F("[Playback] Marker #"));
    Serial.println(playback.currentMarkerIndex + 1);
    stepper.moveTo(targetPos);
    playback.pauseStartTime = millis();
    playback.currentMarkerIndex++;
  }
  else if (!playback.isComplete)
  {
    // Determine which end is furthest from last marker
    long lastMarkerPos = encoderToStepperPosition(
        markers[markerCount - 1].rotationCount,
        markers[markerCount - 1].rawAngle);

    long distToMin = abs(lastMarkerPos - safeMinLimit);
    long distToMax = abs(lastMarkerPos - safeMaxLimit);
    long endPos = (distToMin > distToMax) ? safeMinLimit : safeMaxLimit;

    Serial.print(F("[Playback] Moving to opposite end: "));
    Serial.println(endPos);
    stepper.moveTo(endPos);
    playback.isComplete = true;
  }
  else
  {
    Serial.println(F("\n=== PLAYBACK COMPLETE ==="));
    currentState = STATE_READY;
    clearMarkers();
  }
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

  // Initialize TMC2209 UART communication using ESP32 Serial2
  Serial.println(F("\n[TMC] Init UART"));
  Serial2.begin(115200, SERIAL_8N1, TMC_UART_RX, TMC_UART_TX);
  TMCSerial_ptr = &Serial2;  // Point to Serial2 for TMC driver
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

  // Initialize I2C with ESP32 pins (GPIO21=SDA, GPIO22=SCL)
  Serial.println(F("[I2C] Init GPIO21/GPIO22"));
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
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

  // Initialize button state
  button.currentState = false;
  button.lastState = false;
  button.pressStartTime = 0;
  button.releaseTime = 0;
  button.isPressed = false;
  button.longPressTriggered = false;

  // Initialize encoder calibration
  encoderCal.isCalibrated = false;
  encoderCal.stepsPerEncoderUnit = 0.0;

  // Initialize markers
  clearMarkers();

  Serial.println(F("\n--- Ready ---"));
  Serial.println(F("Homing on startup..."));
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop()
{
  // Update encoder state (detects zero crossings)
  encoder.update();

  // Update button (check for short/long press events)
  uint8_t buttonEvent = updateButton();

  // State machine
  switch (currentState)
  {
  case STATE_STARTUP:
    handleStartupState();
    break;

  case STATE_READY:
    handleReadyState(buttonEvent);
    break;

  case STATE_DISABLED:
    handleDisabledState(buttonEvent);
    break;

  case STATE_PLAYBACK_DELAY:
    handlePlaybackDelayState();
    break;

  case STATE_PLAYBACK:
    handlePlaybackState();
    break;
  }

  // Run stepper (except when disabled or waiting for playback)
  if (currentState != STATE_DISABLED &&
      currentState != STATE_PLAYBACK_DELAY)
  {
    stepper.run();
  }
}
