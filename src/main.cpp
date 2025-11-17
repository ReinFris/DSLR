/*
 * AS5600 Encoder + AccelStepper Motor + Joystick Control
 *
 * WIRING INSTRUCTIONS - AS5600 to Arduino Uno:
 * ============================================
 * AS5600 VCC   -> Arduino 5V (or 3.3V - AS5600 supports both)
 * AS5600 GND   -> Arduino GND
 * AS5600 SDA   -> Arduino A4 (I2C Data)
 * AS5600 SCL   -> Arduino A5 (I2C Clock)
 * AS5600 DIR   -> Arduino GND (sets rotation direction - can also connect to 5V)
 * AS5600 GPO   -> Not connected (optional output pin)
 * AS5600 PGO   -> Not connected (programming pin)
 *
 * WIRING INSTRUCTIONS - Stepper Driver to Arduino Uno:
 * ====================================================
 * Driver STEP  -> Arduino Pin 5
 * Driver DIR   -> Arduino Pin 4
 * Driver EN    -> Arduino Pin 6 (Enable - active LOW)
 * Driver VCC   -> External power supply (check driver voltage rating)
 * Driver GND   -> Common ground with Arduino
 *
 * WIRING INSTRUCTIONS - Analog Joystick to Arduino Uno:
 * ======================================================
 * Joystick VCC -> Arduino 5V
 * Joystick GND -> Arduino GND
 * Joystick VRX -> Arduino A0 (X-axis analog input, 0-1023)
 * Joystick VRY -> Not connected (not used yet)
 * Joystick SW  -> Arduino Pin 2 (switch/button, active LOW)
 *
 * JOYSTICK CONTROL:
 * ================
 * - Center position (VRX ~512): Motor stopped
 * - Push right (VRX > 512): Motor moves forward, speed increases with deflection
 * - Push left (VRX < 512): Motor moves backward, speed increases with deflection
 * - Press button (SW): Emergency stop / reset position to zero
 *
 * MAGNET PLACEMENT:
 * ================
 * - Use a diametrically magnetized magnet (included with AS5600 module)
 * - Place magnet centered above the AS5600 chip
 * - Distance: 0.5mm to 3mm from chip surface
 * - Magnet should be free to rotate (you'll rotate it by hand for testing)
 *
 * NOTES:
 * - The AS5600 will show angle from 0-4095 (12-bit resolution)
 * - 0° = 0, 90° = 1024, 180° = 2048, 270° = 3072, 360° = 4095
 */

#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>
#include "EncoderReader.h"

// Motor pins
#define STEP_PIN 5   // Step pin
#define DIR_PIN 4    // Direction pin
#define ENABLE_PIN 6 // Enable pin (active LOW)
#define LED_PIN 13   // Built-in LED for debugging

// Joystick pins
#define JOYSTICK_VRX A0 // X-axis analog input
#define JOYSTICK_SW 2   // Switch button (active LOW)

// Motor configuration
#define STEPS_PER_REV 200                        // Steps per revolution (1.8° motor = 200 steps)
#define MICROSTEPS 4                             // Microstepping setting on driver
#define TOTAL_STEPS (STEPS_PER_REV * MICROSTEPS) // Total steps per revolution

// Create EncoderReader object
EncoderReader encoder;

// Create AccelStepper object using DRIVER mode (step/direction interface)
// AccelStepper stepper(interface, stepPin, directionPin)
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Motor control parameters
#define MAX_SPEED 3000.0 // Maximum speed in steps/second
#define ACCELERATION 1.0 // Acceleration in steps/second^2

// Joystick parameters
#define JOYSTICK_CENTER 512  // Center position (ADC value 0-1023)
#define JOYSTICK_DEADZONE 50 // Deadzone around center (prevents drift)
#define JOYSTICK_MIN 0       // Minimum ADC value
#define JOYSTICK_MAX 800     // Maximum ADC value

// Debug settings
#define ENABLE_SERIAL_PRINT false // Set to false to disable loop serial output for better performance

void setup()
{
  // Initialize serial for debugging
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== AS5600 Magnetic Encoder Test ===");
  Serial.println();

  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize joystick pins
  pinMode(JOYSTICK_VRX, INPUT);
  pinMode(JOYSTICK_SW, INPUT_PULLUP); // Use internal pull-up for button

  // Initialize motor pins
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // Enable the motor driver (active LOW)

  // Initialize AccelStepper
  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(ACCELERATION);
  stepper.setCurrentPosition(0); // Set current position as zero

  Serial.println("=== AccelStepper Motor + Joystick Control ===");
  Serial.print("Max Speed: ");
  Serial.print(MAX_SPEED);
  Serial.println(" steps/sec");
  Serial.print("Acceleration: ");
  Serial.print(ACCELERATION);
  Serial.println(" steps/sec^2");
  Serial.print("Steps per revolution: ");
  Serial.println(TOTAL_STEPS);
  Serial.println();
  Serial.println("Joystick Controls:");
  Serial.println("  - Center: Stop");
  Serial.println("  - Right: Forward (speed increases with deflection)");
  Serial.println("  - Left: Backward (speed increases with deflection)");
  Serial.println("  - Button: Reset motor position to zero");
  Serial.println();

  // Initialize I2C (Wire library)
  Wire.begin();

  // Initialize AS5600 encoder
  encoder.begin(4); // 4 = default I2C direction mode

  // Check if AS5600 is connected
  if (encoder.isConnected())
  {
    Serial.println("✓ AS5600 detected!");

    // Check AS5600 configuration registers for angle limiting
    Serial.println("\n--- AS5600 Configuration ---");

    // Read ZPOS (Zero Position) register 0x01-0x02
    Wire.beginTransmission(0x36);
    Wire.write(0x01);
    Wire.endTransmission();
    Wire.requestFrom(0x36, 2);
    uint16_t zpos = (Wire.read() << 8) | Wire.read();

    // Read MPOS (Maximum Position) register 0x03-0x04
    Wire.beginTransmission(0x36);
    Wire.write(0x03);
    Wire.endTransmission();
    Wire.requestFrom(0x36, 2);
    uint16_t mpos = (Wire.read() << 8) | Wire.read();

    // Read MANG (Maximum Angle) register 0x05-0x06
    Wire.beginTransmission(0x36);
    Wire.write(0x05);
    Wire.endTransmission();
    Wire.requestFrom(0x36, 2);
    uint16_t mang = (Wire.read() << 8) | Wire.read();

    Serial.print("ZPOS (Zero Position): ");
    Serial.println(zpos);
    Serial.print("MPOS (Max Position): ");
    Serial.println(mpos);
    Serial.print("MANG (Max Angle): ");
    Serial.print(mang);
    Serial.print(" (");
    Serial.print((mang * 360.0) / 4096.0, 1);
    Serial.println("°)");

    if (mang > 0 && mang < 4095)
    {
      Serial.println("⚠ WARNING: MANG is set to limit output range!");
      Serial.println("  This will restrict angle to less than 360°");
    }
    Serial.println("----------------------------\n");

    // Flash LED 3 times to indicate successful connection
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
    Serial.println("✗ AS5600 NOT detected!");
    Serial.println("Check wiring:");
    Serial.println("  - SDA to A4");
    Serial.println("  - SCL to A5");
    Serial.println("  - VCC to 5V");
    Serial.println("  - GND to GND");

    // Flash LED rapidly to indicate error
    while (1)
    {
      digitalWrite(LED_PIN, HIGH);
      delay(50);
      digitalWrite(LED_PIN, LOW);
      delay(50);
    }
  }

  // Check if magnet is detected
  Serial.print("Magnet status: ");
  if (encoder.magnetTooStrong())
  {
    Serial.println("⚠ Magnet too strong or too close to sensor - move it further away");
  }
  else if (encoder.magnetTooWeak())
  {
    Serial.println("⚠ Magnet too weak or too far from sensor - move it closer");
  }
  else if (encoder.detectMagnet())
  {
    Serial.println("✓ Magnet detected and positioned correctly");
  }
  else
  {
    Serial.println("✗ No magnet detected");
  }

  Serial.println();
  Serial.println("=== Ready for operation ===");
  Serial.println();

  // Optional: Set current position as home point
  // Uncomment the line below to set the current position as zero
  // encoder.setHome();

  // Or set a specific offset in degrees (e.g., 90 degrees)
  // encoder.setDegreeOffset(90.0);

  // Or set a specific offset in raw angle units (e.g., 1024 = 90 degrees)
  // encoder.setAngleOffset(1024);

  delay(1000);
}

// ============================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================

// Move motor to absolute position (in steps)
void moveToPosition(long position)
{
  stepper.moveTo(position);
  Serial.print("Moving to position: ");
  Serial.println(position);
}

// Move motor by relative steps (positive = forward, negative = backward)
void moveRelative(long steps)
{
  stepper.move(steps);
  Serial.print("Moving by ");
  Serial.print(steps);
  Serial.println(" steps");
}

// Rotate motor by degrees (based on TOTAL_STEPS configuration)
void rotateDegrees(float degrees)
{
  long steps = (long)((degrees / 360.0) * TOTAL_STEPS);
  stepper.move(steps);
  Serial.print("Rotating ");
  Serial.print(degrees);
  Serial.print("° (");
  Serial.print(steps);
  Serial.println(" steps)");
}

// Rotate one full revolution forward
void rotateOneRevolution()
{
  rotateDegrees(360.0);
}

// Example: Run a simple test sequence
void runTestSequence()
{
  Serial.println("\n=== Starting Test Sequence ===");

  // Move forward one revolution
  Serial.println("Test 1: Rotating 360° forward");
  rotateDegrees(360.0);
  while (stepper.distanceToGo() != 0)
  {
    stepper.run();
  }
  delay(1000);

  // Move backward one revolution
  Serial.println("Test 2: Rotating 360° backward");
  rotateDegrees(-360.0);
  while (stepper.distanceToGo() != 0)
  {
    stepper.run();
  }
  delay(1000);

  Serial.println("=== Test Sequence Complete ===\n");
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

  // Handle button press - reset motor position to zero
  static bool lastButtonState = false;
  if (buttonPressed && !lastButtonState)
  {
    stepper.setCurrentPosition(0);
    Serial.println(">>> Button pressed! Motor position reset to 0");
  }
  lastButtonState = buttonPressed;

  // Calculate motor speed based on joystick position
  float targetSpeed = 0.0;

  if (abs(vrxValue - JOYSTICK_CENTER) > JOYSTICK_DEADZONE)
  {
    // Joystick is outside deadzone - calculate speed
    if (vrxValue > JOYSTICK_CENTER + JOYSTICK_DEADZONE)
    {
      // Right - Forward motion
      float normalizedInput = (float)(vrxValue - JOYSTICK_CENTER - JOYSTICK_DEADZONE) /
                              (float)(JOYSTICK_MAX - JOYSTICK_CENTER - JOYSTICK_DEADZONE);
      targetSpeed = normalizedInput * MAX_SPEED;
    }
    else if (vrxValue < JOYSTICK_CENTER - JOYSTICK_DEADZONE)
    {
      // Left - Backward motion
      float normalizedInput = (float)(JOYSTICK_CENTER - JOYSTICK_DEADZONE - vrxValue) /
                              (float)(JOYSTICK_CENTER - JOYSTICK_DEADZONE - JOYSTICK_MIN);
      targetSpeed = -normalizedInput * MAX_SPEED;
    }
  }

  // Set motor speed using runSpeed mode
  stepper.setSpeed(targetSpeed);
  stepper.run();

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
    Serial.print("VRX: ");
    Serial.print(vrxValue);
    Serial.print(" | Speed: ");
    Serial.print(targetSpeed, 0);
    Serial.print(" | Pos: ");
    Serial.print(motorPosition);
    Serial.print(" | Enc: ");
    Serial.print(rawAngle);
    Serial.print(" (");
    Serial.print(degrees, 1);
    Serial.print("°) | Rot: ");
    Serial.print(rotationCount);
    Serial.print(" | Btn: ");
    Serial.println(buttonPressed ? "PRESSED" : "---");
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
