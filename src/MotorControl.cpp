/*
 * Motor Control Module Implementation
 */

#include "MotorControl.h"

// ============================================================================
// STATIC MEMBERS
// ============================================================================

MotorControl* MotorControl::instance = nullptr;

// ============================================================================
// CONSTRUCTOR
// ============================================================================

MotorControl::MotorControl(TMC2209Stepper &driver, AccelStepper &stepper, EncoderReader &encoder)
    : _driver(driver), _stepper(stepper), _encoder(encoder),
      _stallDetected(false), _shaftVal(false), _isHomed(false),
      _firstLimitPosition(0), _secondLimitPosition(0),
      _minLimit(0), _maxLimit(0), _safeMinLimit(0), _safeMaxLimit(0),
      _centerPosition(0)
{
  _encoderCal.isCalibrated = false;
  _encoderCal.stepsPerEncoderUnit = 0.0;
  instance = this; // Set static instance pointer
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void MotorControl::begin()
{
  // Nothing to initialize here - hardware setup happens in main setup()
}

// ============================================================================
// INTERRUPT HANDLERS
// ============================================================================

// Static ISR - called by hardware interrupt
void IRAM_ATTR MotorControl::stallInterruptISR()
{
  if (instance != nullptr)
  {
    instance->handleStallInterrupt();
  }
}

// Instance method - handles the actual interrupt logic
void MotorControl::handleStallInterrupt()
{
  _stallDetected = true;
}

// ============================================================================
// BASIC MOVEMENT FUNCTIONS
// ============================================================================

void MotorControl::moveRelative(long steps)
{
  _stepper.move(steps);
  Serial.print(F("Moving by "));
  Serial.print(steps);
  Serial.println(F(" steps"));
}

void MotorControl::rotateDegrees(float degrees)
{
  long steps = (long)((degrees / 360.0) * TOTAL_STEPS);
  _stepper.move(steps);
  Serial.print(F("Rotating "));
  Serial.print(degrees);
  Serial.print(F("° ("));
  Serial.print(steps);
  Serial.println(F(" steps)"));
}

// ============================================================================
// LOW-LEVEL MOTOR CONTROL (for homing)
// ============================================================================

void MotorControl::motor(int steps, int stepDelay)
{
  digitalWrite(ENABLE_PIN, LOW);   // Ensure driver is enabled
  digitalWrite(DIR_PIN, _shaftVal); // Set direction via hardware pin

  for (int i = 0; i < steps; i++)
  {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelay);

    // Check for stall during movement
    if (_stallDetected)
    {
      break;
    }
  }
}

// ============================================================================
// HOMING HELPER FUNCTIONS
// ============================================================================

long MotorControl::moveUntilStall(float speed, const char *stepLabel, long ignoreSteps)
{
  Serial.print(F("\n["));
  Serial.print(stepLabel);
  Serial.println(F("] Finding limit..."));

  _stepper.setSpeed(speed);
  long startPosition = _stepper.currentPosition();

  // Clear any pending stall flag
  _stallDetected = false;

  while (!_stallDetected)
  {
    _stepper.runSpeed();
    _encoder.update();

    // Ignore stall detection for the first ignoreSteps
    if (abs(_stepper.currentPosition() - startPosition) < ignoreSteps)
    {
      _stallDetected = false; // Keep clearing the flag during ignore period
    }
  }

  long stallPosition = _stepper.currentPosition();
  Serial.print(F("["));
  Serial.print(stepLabel);
  Serial.print(F("] Limit found at position: "));
  Serial.println(stallPosition);

  _stepper.setSpeed(0);
  delay(100);

  return stallPosition;
}

void MotorControl::moveToPosition(long targetPosition, float speed, unsigned long printInterval)
{
  long stepsToMove = targetPosition - _stepper.currentPosition();
  float moveSpeed = (stepsToMove > 0) ? speed : -speed;
  _stepper.setSpeed(moveSpeed);

  while ((stepsToMove > 0) ? (_stepper.currentPosition() < targetPosition) : (_stepper.currentPosition() > targetPosition))
  {
    _stepper.runSpeed();
    _encoder.update();
  }

  _stepper.setSpeed(0);
}

// ============================================================================
// HOMING ROUTINE
// ============================================================================

void MotorControl::homeX()
{
  const float HOMING_SPEED = 800.0; // works OK with MICROSTEPS = 2

  Serial.println(F("\n=== HOMING SEQUENCE START ==="));
  Serial.print(F("Homing speed: "));
  Serial.print(HOMING_SPEED);
  Serial.println(F(" microsteps/s (~312 full steps/s)"));

  _stallDetected = false;
  delay(100);

  Serial.println(F("[Home] Attaching interrupt..."));
  attachInterrupt(DIAG_PIN, stallInterruptISR, RISING);
  delay(10);
  _stallDetected = false;

  // STEP 1: Find first limit (negative direction)
  moveUntilStall(-HOMING_SPEED, "Step 1", 100);

  _firstLimitPosition = 0;
  _stepper.setCurrentPosition(0);

  // CRITICAL: Reset encoder to create consistent reference frame
  _encoder.setRotationCount(0);
  Serial.println(F("[Step 1] Encoder rotation count reset to 0"));

  Serial.print(F("[Step 1] First limit set at position: "));
  Serial.println(_firstLimitPosition);

  // Capture encoder position at first limit (min limit)
  _encoderCal.minLimitRotations = _encoder.getRotationCount();
  _encoderCal.minLimitRawAngle = _encoder.getRawAngle();
  _encoderCal.minLimitStepperPos = 0;
  Serial.print(F("[Calibration] Min limit encoder: rot="));
  Serial.print(_encoderCal.minLimitRotations);
  Serial.print(F(" raw="));
  Serial.println(_encoderCal.minLimitRawAngle);

  // STEP 2: Find second limit (positive direction)
  moveUntilStall(HOMING_SPEED, "Step 2", 100);

  _secondLimitPosition = _stepper.currentPosition();

  // Capture encoder position at second limit (max limit)
  _encoderCal.maxLimitRotations = _encoder.getRotationCount();
  _encoderCal.maxLimitRawAngle = _encoder.getRawAngle();
  _encoderCal.maxLimitStepperPos = _stepper.currentPosition();
  Serial.print(F("[Calibration] Max limit encoder: rot="));
  Serial.print(_encoderCal.maxLimitRotations);
  Serial.print(F(" raw="));
  Serial.println(_encoderCal.maxLimitRawAngle);

  // Calculate limits and center
  _minLimit = (_firstLimitPosition < _secondLimitPosition) ? _firstLimitPosition : _secondLimitPosition;
  _maxLimit = (_firstLimitPosition < _secondLimitPosition) ? _secondLimitPosition : _firstLimitPosition;
  _centerPosition = (_minLimit + _maxLimit) / 2;

  // Calculate safe limits with buffer
  _safeMinLimit = _minLimit + SAFETY_BUFFER_STEPS;
  _safeMaxLimit = _maxLimit - SAFETY_BUFFER_STEPS;

  Serial.println(F("\n=== HOMING RESULTS ==="));
  Serial.print(F("First limit:  "));
  Serial.println(_firstLimitPosition);
  Serial.print(F("Second limit: "));
  Serial.println(_secondLimitPosition);
  Serial.print(F("Min limit:    "));
  Serial.println(_minLimit);
  Serial.print(F("Max limit:    "));
  Serial.println(_maxLimit);
  Serial.print(F("Safe min:     "));
  Serial.println(_safeMinLimit);
  Serial.print(F("Safe max:     "));
  Serial.println(_safeMaxLimit);
  Serial.print(F("Range:        "));
  Serial.print(_maxLimit - _minLimit);
  Serial.println(F(" steps"));
  Serial.print(F("Center:       "));
  Serial.println(_centerPosition);

  // Calculate encoder calibration using rotation-based math
  float encoderMinRot = (float)_encoderCal.minLimitRotations +
                        (_encoderCal.minLimitRawAngle / 4096.0);
  float encoderMaxRot = (float)_encoderCal.maxLimitRotations +
                        (_encoderCal.maxLimitRawAngle / 4096.0);
  float encoderRotationRange = encoderMaxRot - encoderMinRot;

  float stepperRotationRange = (_maxLimit - _minLimit) / (float)(STEPS_PER_REV * MICROSTEPS);

  _encoderCal.encoderRotationsPerStepperRotation = encoderRotationRange / stepperRotationRange;

  // Keep legacy calculation for backward compatibility
  long encoderMin = (_encoderCal.minLimitRotations * 4096L) + _encoderCal.minLimitRawAngle;
  long encoderMax = (_encoderCal.maxLimitRotations * 4096L) + _encoderCal.maxLimitRawAngle;
  long encoderRange = abs(encoderMax - encoderMin);
  long stepperRange = _maxLimit - _minLimit;
  _encoderCal.stepsPerEncoderUnit = (float)stepperRange / (float)encoderRange;
  _encoderCal.isCalibrated = true;

  Serial.println(F("\n=== ENCODER CALIBRATION ==="));
  Serial.print(F("Encoder rotations: "));
  Serial.println(encoderRotationRange, 6);
  Serial.print(F("Stepper rotations: "));
  Serial.println(stepperRotationRange, 6);
  Serial.print(F("Encoder/Stepper ratio: "));
  Serial.println(_encoderCal.encoderRotationsPerStepperRotation, 6);
  Serial.print(F("Steps/unit (legacy): "));
  Serial.println(_encoderCal.stepsPerEncoderUnit, 6);

  // Verify calibration by converting limits back
  long verifyMin = encoderToStepperPosition(
      _encoderCal.minLimitRotations,
      _encoderCal.minLimitRawAngle);
  long verifyMax = encoderToStepperPosition(
      _encoderCal.maxLimitRotations,
      _encoderCal.maxLimitRawAngle);

  Serial.println(F("\n=== CALIBRATION VERIFICATION ==="));
  Serial.print(F("Min limit: "));
  Serial.print(verifyMin);
  Serial.print(F(" (expect ~"));
  Serial.print(_safeMinLimit);
  Serial.println(F(")"));
  Serial.print(F("Max limit: "));
  Serial.print(verifyMax);
  Serial.print(F(" (expect ~"));
  Serial.print(_safeMaxLimit);
  Serial.println(F(")"));

  detachInterrupt(DIAG_PIN);
  Serial.println(F("[Home] Interrupt detached"));

  // STEP 3: Move to center position using full AccelStepper capabilities
  Serial.println(F("\n[Step 3] Moving to center with acceleration..."));
  Serial.print(F("Current position: "));
  Serial.println(_stepper.currentPosition());
  Serial.print(F("Target position:  "));
  Serial.println(_centerPosition);
  Serial.print(F("Steps to center:  "));
  Serial.println(_centerPosition - _stepper.currentPosition());
  Serial.print(F("Max speed:        "));
  Serial.print(_stepper.maxSpeed());
  Serial.println(F(" steps/s"));
  Serial.print(F("Acceleration:     "));
  Serial.print(_stepper.acceleration());
  Serial.println(F(" steps/s²"));

  // Use moveTo() for accelerated movement to center
  _stepper.moveTo(_centerPosition);

  // Run motor with acceleration until target is reached
  while (_stepper.distanceToGo() != 0)
  {
    _stepper.run();
    _encoder.update();
  }

  Serial.print(F("[Step 3] Arrived at center! Position="));
  Serial.println(_stepper.currentPosition());
  Serial.print(F("Final speed: "));
  Serial.print(_stepper.speed());
  Serial.println(F(" steps/s"));

  _isHomed = true;
  Serial.println(F("\n=== HOMING COMPLETE ===\n"));
}

// ============================================================================
// ENCODER CONVERSION
// ============================================================================

long MotorControl::encoderToStepperPosition(long rotationCount, uint16_t rawAngle)
{
  if (!_encoderCal.isCalibrated)
    return 0;

  // Calculate encoder position in rotations (floating point precision)
  float encoderRotations = (float)rotationCount + (rawAngle / 4096.0);
  float encoderMinRotations = (float)_encoderCal.minLimitRotations +
                               (_encoderCal.minLimitRawAngle / 4096.0);
  float encoderRelativeRotations = encoderRotations - encoderMinRotations;

  // Convert to stepper rotations, then to steps
  float stepperRelativeRotations = encoderRelativeRotations /
                                    _encoderCal.encoderRotationsPerStepperRotation;
  long stepperRelativeSteps = (long)(stepperRelativeRotations * STEPS_PER_REV * MICROSTEPS);
  long stepperPos = _encoderCal.minLimitStepperPos + stepperRelativeSteps;

  // Clamp to safe range (with buffer)
  if (stepperPos < _safeMinLimit)
    stepperPos = _safeMinLimit;
  if (stepperPos > _safeMaxLimit)
    stepperPos = _safeMaxLimit;

  return stepperPos;
}
