/*
 * AS5600 Magnetic Encoder Test
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
#include "EncoderReader.h"

// Motor pins (commented out for encoder testing)
// #define enablePin 6              // Enable
// #define dirPin 4                 // Direction
// #define stepPin 5                // Steps
#define LED_PIN 13 // Built-in LED for debugging
// #define stepsPerRevolution 15000 // Full revolution with microstepping

// Create EncoderReader object
EncoderReader encoder;

// Smooth motion parameters (commented out for encoder testing)
// #define MAX_SPEED 50    // Minimum delay in microseconds (faster)
// #define MIN_SPEED 2500  // Maximum delay in microseconds (slower)
// #define ACCEL_STEPS 800 // Steps to accelerate/decelerate
// int currentDelay = MIN_SPEED;

// ============================================================
// SMOOTHSTEP FUNCTIONS (commented out for encoder testing)
// ============================================================
/*
// Original smoothstep (3rd order polynomial)
// Zero 1st derivative at edges
float smoothstep(float t)
{
  if (t <= 0.0f)
    return 0.0f;
  if (t >= 1.0f)
    return 1.0f;
  return t * t * (3.0f - 2.0f * t); // 3t² - 2t³
}

// Ken Perlin's smootherstep (5th order polynomial)
// Zero 1st and 2nd derivatives at edges - RECOMMENDED for motor control
float smootherstep(float t)
{
  if (t <= 0.0f)
    return 0.0f;
  if (t >= 1.0f)
    return 1.0f;
  return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f); // 6t⁵ - 15t⁴ + 10t³
}

// 7th order smoothstep - Ultra smooth but computationally expensive
// Zero 1st, 2nd, and 3rd derivatives at edges
float smootheststep(float t)
{
  if (t <= 0.0f)
    return 0.0f;
  if (t >= 1.0f)
    return 1.0f;
  return t * t * t * t * (35.0f - t * (84.0f - t * (70.0f - 20.0f * t))); // -20t⁷ + 70t⁶ - 84t⁵ + 35t⁴
}

// Map a value using smootherstep interpolation (better than linear map)
float smoothMap(float x, float in_min, float in_max, float out_min, float out_max)
{
  // Normalize input to 0-1 range
  float t = (x - in_min) / (in_max - in_min);
  // Apply smootherstep
  float smooth_t = smootherstep(t);
  // Map to output range
  return out_min + smooth_t * (out_max - out_min);
}
*/

void setup()
{
  // Initialize serial for debugging
  Serial.begin(9600);
  delay(1000);
  Serial.println("=== AS5600 Magnetic Encoder Test ===");
  Serial.println();

  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

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

  // Diagnostic: Rotate magnet slowly and observe the raw angle range
  Serial.println("=== Diagnostic: Checking raw angle range ===");
  Serial.println("Please rotate the magnet SLOWLY through one full rotation...");
  delay(2000);

  uint16_t minRaw = 4095;
  uint16_t maxRaw = 0;

  // Sample for 10 seconds while user rotates
  for (int i = 0; i < 100; i++)
  {
    encoder.update();
    uint16_t raw = encoder.getRawAngle();
    if (raw < minRaw) minRaw = raw;
    if (raw > maxRaw) maxRaw = raw;

    Serial.print("Raw: ");
    Serial.print(raw);
    Serial.print(" | Min: ");
    Serial.print(minRaw);
    Serial.print(" | Max: ");
    Serial.println(maxRaw);
    delay(100);
  }

  Serial.println();
  Serial.println("=== Range Test Complete ===");
  Serial.print("Min raw angle seen: ");
  Serial.println(minRaw);
  Serial.print("Max raw angle seen: ");
  Serial.println(maxRaw);
  Serial.print("Total range: ");
  Serial.print(maxRaw - minRaw);
  Serial.println(" (should be ~4095 for full rotation)");
  Serial.println();

  Serial.println("=== Now starting normal operation ===");
  Serial.println("Format: Raw Angle | Degrees | Direction");
  Serial.println("=========================================");
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
// MOTOR CONTROL FUNCTION (commented out for encoder testing)
// ============================================================
/*
void smoothStep(int totalSteps, bool forward)
{
  digitalWrite(dirPin, forward ? HIGH : LOW);
  delay(5); // Direction setup time

  for (int i = 0; i < totalSteps; i++)
  {
    float t;           // Normalized position (0.0 to 1.0)
    float speedFactor; // 0.0 (stopped) to 1.0 (full speed)

    // Calculate speed with acceleration/deceleration using smootherstep
    if (i < ACCEL_STEPS)
    {
      // Accelerate: 0.0 to 1.0 speed
      t = (float)i / (float)ACCEL_STEPS;
      speedFactor = smootherstep(t);
    }
    else if (i >= (totalSteps - ACCEL_STEPS))
    {
      // Decelerate: 1.0 to 0.0 speed
      t = (float)(i - (totalSteps - ACCEL_STEPS)) / (float)ACCEL_STEPS;
      speedFactor = 1.0f - smootherstep(t);
    }
    else
    {
      // Constant max speed
      speedFactor = 1.0f;
    }

    // Convert speed factor to delay (inverse relationship)
    // Use reciprocal interpolation for proper speed curve
    float minFreq = 1.0f / (float)MIN_SPEED;
    float maxFreq = 1.0f / (float)MAX_SPEED;
    float currentFreq = minFreq + speedFactor * (maxFreq - minFreq);
    currentDelay = (int)(1.0f / currentFreq);

    // Generate step pulse
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(currentDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(currentDelay);
  }
}
*/

void loop()
{
  // Update encoder state (detects rotation crossings)
  encoder.update();

  // Get encoder readings using class methods
  uint16_t rawAngle = encoder.getRawAngle();
  float degrees = encoder.getDegrees();
  long rotationCount = encoder.getRotationCount();

  // Print formatted output
  Serial.print("Raw: ");
  Serial.print(rawAngle);
  Serial.print("\t| Deg: ");
  Serial.print(degrees, 2);
  Serial.print("°\t| Rotations: ");
  Serial.print(rotationCount);

  // Visual indicator - print a simple bar graph
  Serial.print("\t| ");
  int barLength = (int)((degrees / 360.0) * 40);
  for (int i = 0; i < 40; i++)
  {
    if (i < barLength)
      Serial.print("█");
    else
      Serial.print("·");
  }

  Serial.println();

  // Blink LED based on position
  if (rawAngle < 100) // Near zero position
  {
    digitalWrite(LED_PIN, HIGH);
  }
  else
  {
    digitalWrite(LED_PIN, LOW);
  }

  delay(1); // Update rate: 100Hz (faster sampling for rotation detection)
}
