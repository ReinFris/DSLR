/*
 * TMC2209 Stepper Motor with AS5600 Encoder for Arduino Uno
 *
 * ============================================================================
 * PIN DEFINITIONS - ARDUINO UNO
 * ============================================================================
 *
 * TMC2209 Stepper Driver -> Arduino Uno
 * ----------------------------------------
 * STEP   -> Digital Pin 2
 * DIR    -> Digital Pin 3
 * EN     -> Digital Pin 4 (active LOW - pull LOW to enable driver)
 * VCC_IO -> 5V (Arduino logic level)
 * GND    -> GND
 * VM     -> Motor power supply (4.75V - 29V, separate from Arduino)
 * A1, A2, B1, B2 -> Stepper motor coils
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
 * 3. Power Supply:
 *    - TMC2209 VCC_IO: 5V from Arduino
 *    - TMC2209 VM: Separate motor power supply (typically 12V or 24V)
 *    - AS5600 VCC: 5V from Arduino (AS5600 supports 3.3V-5V)
 *    - Always connect grounds together (Arduino GND, motor PSU GND)
 *
 * 4. Serial Monitor:
 *    - Baud rate: 115200
 *    - Displays: Encoder angle, Degrees, Rotation count, Magnet status
 */

#include <Arduino.h>
#include <Wire.h>
#include "EncoderReader.h"

// ============================================================================
// PIN ASSIGNMENTS FOR ARDUINO UNO
// ============================================================================

// Stepper motor control pins
#define STEP_PIN 2       // Digital pin 2 - Step pulse for TMC2209
#define DIR_PIN 3        // Digital pin 3 - Direction control for TMC2209
#define ENABLE_PIN 4     // Digital pin 4 - Enable pin (LOW = enabled, HIGH = disabled)

// Status LED
#define LED_PIN 13       // Digital pin 13 - Built-in LED on Arduino Uno

// I2C pins for AS5600 encoder
// NOTE: These are FIXED on Arduino Uno and cannot be changed
// Do NOT attempt to use other pins for I2C on Uno - it will not work
#define I2C_SDA_PIN A4   // Analog pin A4 - I2C SDA (FIXED on Uno)
#define I2C_SCL_PIN A5   // Analog pin A5 - I2C SCL (FIXED on Uno)

// Motor parameters
#define STEPS_PER_REV 200 // Standard stepper motor (1.8° per step)
#define MICROSTEPS 8      // TMC2209 default microstepping in standalone
#define STEP_DELAY 1000   // Microseconds between steps

// Create encoder instance
EncoderReader encoder;

// Update interval for serial output (milliseconds)
const unsigned long PRINT_INTERVAL = 100; // 10Hz update rate
unsigned long lastPrintTime = 0;

// Motor control variables
bool motorRunning = false;
int motorDirection = 0; // 0 = stopped, 1 = clockwise, -1 = counter-clockwise

void setup()
{
  // Initialize stepper motor pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // Enable the driver (LOW = enabled)
  digitalWrite(ENABLE_PIN, LOW);

  // Set initial direction (LOW = clockwise for TMC2209)
  digitalWrite(DIR_PIN, LOW);

  // Blink LED 3 times on startup
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }

  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial && millis() < 3000)
  {
    // Wait for serial port to connect (max 3 seconds)
    delay(10);
  }

  Serial.println("\n\n=== TMC2209 Stepper + AS5600 Encoder ===");
  Serial.println("Arduino Uno - Firmware v1.0");
  Serial.println("Initializing...\n");

  // Initialize I2C with default pins (A4=SDA, A5=SCL on Arduino Uno)
  Serial.println("Starting I2C bus (A4=SDA, A5=SCL)...");
  Wire.begin();
  Wire.setClock(100000); // Set I2C to 100kHz standard mode

  // Recover I2C bus state by performing multiple dummy transactions
  // This clears any stuck states from previous incomplete I2C operations
  Serial.println("Recovering I2C bus...");
  for (int i = 0; i < 10; i++) {
    Wire.beginTransmission(0x36);
    Wire.endTransmission();
    delay(10);
  }
  Serial.println("I2C bus recovery complete");

  Serial.println("Testing I2C communication...");

  // Test if AS5600 is present by checking I2C address 0x36
  Wire.beginTransmission(0x36);
  int i2cError = Wire.endTransmission();

  if (i2cError == 0)
  {
    Serial.println("✓ AS5600 detected on I2C bus");

    // Initialize encoder (with I2C bus recovery done above, this should work now)
    Serial.println("Initializing encoder...");
    encoder.begin();
    Serial.println("✓ Encoder initialized");

    delay(100); // Give encoder time to stabilize

    // Check magnet status
    if (encoder.detectMagnet())
    {
      Serial.println("✓ Magnet detected");

      if (encoder.magnetTooStrong())
      {
        Serial.println("⚠ WARNING: Magnet too strong - move it further away");
      }
      else if (encoder.magnetTooWeak())
      {
        Serial.println("⚠ WARNING: Magnet too weak - move it closer");
      }
      else
      {
        Serial.println("✓ Magnet strength is good");
      }
    }
    else
    {
      Serial.println("✗ WARNING: No magnet detected!");
    }
  }
  else
  {
    Serial.println("✗ WARNING: AS5600 not found on I2C bus!");
    Serial.println("  I2C Error code: " + String(i2cError));
    Serial.println("  Check I2C wiring (SDA=A4, SCL=A5)");
    Serial.println("  Verify AS5600 has 5V power and GND connected");
    Serial.println("  Motor will still run, but encoder readings will be zero");
  }

  Serial.println("\n✓ Motor driver initialized");
  Serial.println("\nStarting motor rotation and encoder readings...");
  Serial.println("------------------------------------------------------------");
  Serial.println("Direction | Raw Angle | Degrees  | Rotations | Magnet Status");
  Serial.println("------------------------------------------------------------");
}

void loop()
{
  // Update encoder state (detects zero crossings)
  encoder.update();

  // Run motor continuously - one revolution clockwise, pause, one revolution counter-clockwise, pause
  static unsigned long motorStateChangeTime = 0;
  static int motorState = 0; // 0 = CW, 1 = pause after CW, 2 = CCW, 3 = pause after CCW
  static int stepCount = 0;
  unsigned long currentTime = millis();

  switch (motorState)
  {
  case 0:                        // Rotate clockwise
    digitalWrite(DIR_PIN, LOW);  // LOW = clockwise for TMC2209
    digitalWrite(LED_PIN, HIGH); // LED ON during clockwise rotation
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY);
    stepCount++;

    if (stepCount >= (STEPS_PER_REV * MICROSTEPS))
    {
      stepCount = 0;
      motorState = 1;
      motorStateChangeTime = currentTime;
    }
    break;

  case 1:                       // Pause after clockwise
    digitalWrite(LED_PIN, LOW); // LED OFF during pause
    if (currentTime - motorStateChangeTime >= 1000)
    {
      motorState = 2;
    }
    break;

  case 2:                        // Rotate counter-clockwise
    digitalWrite(DIR_PIN, HIGH); // HIGH = counter-clockwise for TMC2209
    digitalWrite(LED_PIN, LOW);  // LED OFF during counter-clockwise rotation
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY);
    stepCount++;

    if (stepCount >= (STEPS_PER_REV * MICROSTEPS))
    {
      stepCount = 0;
      motorState = 3;
      motorStateChangeTime = currentTime;
    }
    break;

  case 3:                       // Pause after counter-clockwise
    digitalWrite(LED_PIN, LOW); // LED OFF during pause
    if (currentTime - motorStateChangeTime >= 1000)
    {
      motorState = 0;
    }
    break;
  }

  // Print readings at specified interval
  if (currentTime - lastPrintTime >= PRINT_INTERVAL)
  {
    lastPrintTime = currentTime;

    // Get encoder readings
    uint16_t rawAngle = encoder.getRawAngle();
    float degrees = encoder.getDegrees();
    long rotations = encoder.getRotationCount();

    // Determine direction for display
    const char *dirStr;
    if (motorState == 0)
    {
      dirStr = "CW      ";
    }
    else if (motorState == 1)
    {
      dirStr = "PAUSE   ";
    }
    else if (motorState == 2)
    {
      dirStr = "CCW     ";
    }
    else
    {
      dirStr = "PAUSE   ";
    }

    // Format and print data
    // Note: Arduino's snprintf doesn't support %f, so we format degrees separately
    char buffer[100];

    // Convert degrees to integer and decimal parts for display
    int deg_int = (int)degrees;
    int deg_frac = (int)((degrees - deg_int) * 100);

    snprintf(buffer, sizeof(buffer),
             "%s  | %4d      | %3d.%02d°  | %5ld     | ",
             dirStr, rawAngle, deg_int, deg_frac, rotations);
    Serial.print(buffer);

    // Print magnet status
    if (!encoder.detectMagnet())
    {
      Serial.println("NO MAGNET");
    }
    else if (encoder.magnetTooStrong())
    {
      Serial.println("TOO STRONG");
    }
    else if (encoder.magnetTooWeak())
    {
      Serial.println("TOO WEAK");
    }
    else
    {
      Serial.println("OK");
    }
  }
}
