/*
 * Motor Control Module
 *
 * Handles all stepper motor operations including:
 * - Basic movement functions
 * - Sensorless homing using StallGuard
 * - Limit detection and management
 * - Encoder calibration
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include <TMCStepper.h>
#include <AccelStepper.h>
#include "EncoderReader.h"
#include "HardwareConfig.h"

// ============================================================================
// ENCODER CALIBRATION STRUCTURE
// ============================================================================

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

// ============================================================================
// MOTOR CONTROL CLASS
// ============================================================================

class MotorControl
{
public:
  // Constructor
  MotorControl(TMC2209Stepper &driver, AccelStepper &stepper, EncoderReader &encoder);

  // Initialization
  void begin();

  // Basic movement functions
  void moveRelative(long steps);
  void rotateDegrees(float degrees);

  // Homing and calibration
  void homeX();
  bool isHomed() const { return _isHomed; }

  // Limit accessors
  long getMinLimit() const { return _minLimit; }
  long getMaxLimit() const { return _maxLimit; }
  long getSafeMinLimit() const { return _safeMinLimit; }
  long getSafeMaxLimit() const { return _safeMaxLimit; }
  long getCenterPosition() const { return _centerPosition; }

  // Encoder conversion
  long encoderToStepperPosition(long rotationCount, uint16_t rawAngle);

  // Position synchronization
  bool syncStepperToEncoder();

  // Calibration accessor
  const EncoderCalibration &getCalibration() const { return _encoderCal; }

  // Interrupt handler (must be public to attach)
  void handleStallInterrupt();

  // Static instance pointer for ISR callback
  static MotorControl* instance;
  static void IRAM_ATTR stallInterruptISR();

private:
  // Hardware references
  TMC2209Stepper &_driver;
  AccelStepper &_stepper;
  EncoderReader &_encoder;

  // State variables
  volatile bool _stallDetected;
  bool _shaftVal;  // Direction: false = forward, true = reverse
  bool _isHomed;

  // Limit positions
  long _firstLimitPosition;
  long _secondLimitPosition;
  long _minLimit;
  long _maxLimit;
  long _safeMinLimit;
  long _safeMaxLimit;
  long _centerPosition;

  // Encoder calibration
  EncoderCalibration _encoderCal;

  // Helper functions
  void motor(int steps, int stepDelay);
  long moveUntilStall(float speed, const char *stepLabel, long ignoreSteps = 0);
  void moveToPosition(long targetPosition, float speed, unsigned long printInterval);
};

#endif // MOTOR_CONTROL_H
