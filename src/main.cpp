// AS5600 Encoder + AccelStepper + Joystick with homing

#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>
#include "EncoderReader.h"

#define STEP_PIN 5
#define DIR_PIN 4
#define ENABLE_PIN 6
#define LED_PIN 13
#define JOYSTICK_VRX A0
#define JOYSTICK_SW 2
#define STEPS_PER_REV 200
#define MICROSTEPS 4
#define TOTAL_STEPS (STEPS_PER_REV * MICROSTEPS)

// Create EncoderReader object
EncoderReader encoder;

// Create AccelStepper object using DRIVER mode (step/direction interface)
// AccelStepper stepper(interface, stepPin, directionPin)
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Motor control parameters
#define MAX_SPEED 1000.0  // Maximum speed in steps/second
#define HALF_SPEED 500.0  // Half speed in steps/second
#define ACCELERATION 20.0 // Acceleration in steps/second^2

// Joystick parameters
#define JOYSTICK_CENTER 512         // Center position (ADC value 0-1023)
#define JOYSTICK_DEADZONE 50        // Deadzone around center (prevents drift)
#define JOYSTICK_HALF_THRESHOLD 150 // Threshold for half vs full speed
#define JOYSTICK_MIN 0              // Minimum ADC value
#define JOYSTICK_MAX 800            // Maximum ADC value

// Debug settings
#define ENABLE_SERIAL_PRINT false // Set to false to disable loop serial output for better performance

// Homing state machine
enum HomingState
{
  UNHOMED,            // Waiting for first limit
  HOMING_FIRST_LIMIT, // First limit set, waiting for second limit
  HOMED               // Both limits set, normal operation
};

// Homing variables
HomingState homingState = UNHOMED;
long firstLimitPosition = 0;
long secondLimitPosition = 0;
long minLimit = 0;
long maxLimit = 0;

void setup()
{
  Serial.begin(9600);
  delay(500);

  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize joystick pins
  pinMode(JOYSTICK_VRX, INPUT);
  pinMode(JOYSTICK_SW, INPUT_PULLUP); // Use internal pull-up for button

  // Initialize motor pins
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // Enable the motor driver (active LOW)

  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(ACCELERATION);
  stepper.setCurrentPosition(0);

  Serial.println("HOMING: 1.Move to 1st limit, click. 2.Move to 2nd limit, click.");
  Serial.println("Controls: Small deflection=half speed, Large=full speed");

  // Initialize I2C (Wire library)
  Wire.begin();

  // Initialize AS5600 encoder
  encoder.begin(4); // 4 = default I2C direction mode

  // Check if AS5600 is connected
  if (encoder.isConnected())
  {
    Serial.println("AS5600 OK");
    // Flash LED 3 times
    for (int i = 0; i < 3; i++)
    {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }
  else
  {
    Serial.println("AS5600 ERROR - check wiring");
    // Flash LED rapidly
    while (1)
    {
      digitalWrite(LED_PIN, HIGH);
      delay(50);
      digitalWrite(LED_PIN, LOW);
      delay(50);
    }
  }

  if (encoder.magnetTooStrong())
    Serial.println("Mag: too close");
  else if (encoder.magnetTooWeak())
    Serial.println("Mag: too far");
  else if (encoder.detectMagnet())
    Serial.println("Mag: OK");
  else
    Serial.println("Mag: none");

  Serial.println("Ready");
  delay(500);
}

void loop()
{
  // IMPORTANT: Must call stepper.run() regularly for AccelStepper to work!
  // This handles acceleration, deceleration, and stepping
  // stepper.run();

  // Update encoder state (detects rotation crossings)
  encoder.update();

  // Read joystick VRX (X-axis)
  int vrxValue = analogRead(JOYSTICK_VRX);

  // Read joystick button (active LOW)
  bool buttonPressed = (digitalRead(JOYSTICK_SW) == LOW);

  // Handle button press for homing sequence
  static bool lastButtonState = false;
  if (buttonPressed && !lastButtonState)
  {
    long currentPosition = stepper.currentPosition();

    switch (homingState)
    {
    case UNHOMED:
      firstLimitPosition = currentPosition;
      homingState = HOMING_FIRST_LIMIT;
      Serial.print("L1:");
      Serial.println(firstLimitPosition);
      break;

    case HOMING_FIRST_LIMIT:
      secondLimitPosition = currentPosition;
      if (firstLimitPosition < secondLimitPosition)
      {
        minLimit = firstLimitPosition;
        maxLimit = secondLimitPosition;
      }
      else
      {
        minLimit = secondLimitPosition;
        maxLimit = firstLimitPosition;
      }
      homingState = HOMED;
      Serial.print("L2:");
      Serial.print(secondLimitPosition);
      Serial.print(" R:");
      Serial.println(maxLimit - minLimit);
      break;

    case HOMED:
      homingState = UNHOMED;
      Serial.println("Rehome");
      break;
    }
  }
  lastButtonState = buttonPressed;

  // Motor control - different modes for homing vs homed
  if (homingState == HOMED)
  {
    // HOMED: Use moveTo/run for acceleration control
    long currentPos = stepper.currentPosition();
    long targetPosition;
    float maxSpeedSetting;

    if (abs(vrxValue - JOYSTICK_CENTER) > JOYSTICK_DEADZONE)
    {
      // Joystick is deflected - determine speed and direction
      int deflection = abs(vrxValue - JOYSTICK_CENTER);

      // Set max speed based on deflection
      if (deflection > JOYSTICK_HALF_THRESHOLD)
        maxSpeedSetting = MAX_SPEED;
      else
        maxSpeedSetting = HALF_SPEED;

      // Calculate target position far in the direction of movement
      if (vrxValue > JOYSTICK_CENTER + JOYSTICK_DEADZONE)
      {
        // Forward - set target to max limit
        targetPosition = maxLimit;
      }
      else
      {
        // Backward - set target to min limit
        targetPosition = minLimit;
      }
    }
    else
    {
      // Joystick centered - stop at current position with deceleration
      targetPosition = currentPos;
      maxSpeedSetting = MAX_SPEED;
    }

    // Apply settings and move
    stepper.setMaxSpeed(maxSpeedSetting);
    stepper.moveTo(targetPosition);
    stepper.run();
  }
  else
  {
    // NOT HOMED: Use setSpeed/runSpeed for continuous motion during homing
    float targetSpeed = 0.0;

    if (abs(vrxValue - JOYSTICK_CENTER) > JOYSTICK_DEADZONE)
    {
      // Joystick is deflected - determine speed and direction
      int deflection = abs(vrxValue - JOYSTICK_CENTER);

      // Set speed based on deflection
      float speedStep;
      if (deflection > JOYSTICK_HALF_THRESHOLD)
        speedStep = MAX_SPEED;
      else
        speedStep = HALF_SPEED;

      // Apply direction
      if (vrxValue > JOYSTICK_CENTER + JOYSTICK_DEADZONE)
        targetSpeed = speedStep;  // Forward
      else
        targetSpeed = -speedStep; // Backward
    }

    // Apply speed
    stepper.setSpeed(targetSpeed);
    stepper.runSpeed();
  }

  // Print encoder and motor data every 100ms (non-blocking)
  static unsigned long lastPrint = 0;
  if (ENABLE_SERIAL_PRINT && (millis() - lastPrint >= 100))
  {
    lastPrint = millis();

    // Get encoder readings
    uint16_t rawAngle = encoder.getRawAngle();
    float degrees = encoder.getDegrees();
    long rotationCount = encoder.getRotationCount();

    // Get motor status
    long motorPosition = stepper.currentPosition();

    // Print formatted output
    Serial.print("State: ");
    switch (homingState)
    {
    case UNHOMED:
      Serial.print("UNHOMED");
      break;
    case HOMING_FIRST_LIMIT:
      Serial.print("HOMING");
      break;
    case HOMED:
      Serial.print("HOMED");
      break;
    }
    Serial.print(" | VRX:");
    Serial.print(vrxValue);
    Serial.print(" | Pos:");
    Serial.print(motorPosition);
    Serial.print(" | Spd:");
    Serial.print(stepper.speed(), 0);

    if (homingState == HOMED)
    {
      Serial.print(" | Lim:[");
      Serial.print(minLimit);
      Serial.print(",");
      Serial.print(maxLimit);
      Serial.print("]");
    }

    Serial.print(" | Enc:");
    Serial.print(rawAngle);
    Serial.println();
  }

  // Blink LED based on encoder position (always active)
  uint16_t rawAngle = encoder.getRawAngle();
  if (rawAngle < 100) // Near zero position
  {
    digitalWrite(LED_PIN, HIGH);
  }
  else
  {
    digitalWrite(LED_PIN, LOW);
  }
}
