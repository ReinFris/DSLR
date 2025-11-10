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
#include <AS5600.h>
#include <Wire.h>

// Motor pins (commented out for encoder testing)
// #define enablePin 6              // Enable
// #define dirPin 4                 // Direction
// #define stepPin 5                // Steps
#define LED_PIN 13               // Built-in LED for debugging
// #define stepsPerRevolution 15000 // Full revolution with microstepping

// Create AS5600 encoder object
AS5600 encoder;

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
  encoder.begin(4);  // 4 = default I2C direction mode

  // Check if AS5600 is connected
  if (encoder.isConnected())
  {
    Serial.println("✓ AS5600 detected!");

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
  Serial.println("=== Now rotate the magnet by hand ===");
  Serial.println("Format: Raw Angle | Degrees | Direction");
  Serial.println("=========================================");
  Serial.println();

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
  // Read raw angle from encoder (0-4095, 12-bit resolution)
  uint16_t rawAngle = encoder.rawAngle();

  // Convert to degrees (0-360)
  float degrees = (rawAngle * 360.0) / 4096.0;

  // Determine rotation direction (clockwise or counter-clockwise)
  static uint16_t lastAngle = 0;
  static long rotationCount = 0;

  // Detect zero crossing
  if (lastAngle > 3800 && rawAngle < 300)  // Crossed 0 going forward
  {
    rotationCount++;
  }
  else if (lastAngle < 300 && rawAngle > 3800)  // Crossed 0 going backward
  {
    rotationCount--;
  }

  lastAngle = rawAngle;

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
  if (rawAngle < 100)  // Near zero position
  {
    digitalWrite(LED_PIN, HIGH);
  }
  else
  {
    digitalWrite(LED_PIN, LOW);
  }

  delay(100);  // Update rate: 10Hz
}
