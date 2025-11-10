#include <Arduino.h>

#define enablePin 6              // Enable
#define dirPin 4                 // Direction
#define stepPin 5                // Steps
#define LED_PIN 13               // Built-in LED for debugging
#define stepsPerRevolution 15000 // Full revolution with microstepping

// Smooth motion parameters
#define MAX_SPEED 50    // Minimum delay in microseconds (faster)
#define MIN_SPEED 2500  // Maximum delay in microseconds (slower)
#define ACCEL_STEPS 800 // Steps to accelerate/decelerate

int currentDelay = MIN_SPEED;

// Smoothstep interpolation functions (return value 0.0 to 1.0 for input 0.0 to 1.0)

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

void setup()
{
  // Initialize serial for debugging
  Serial.begin(9600);
  delay(1000);
  Serial.println("=== Stepper Motor - Smooth Motion ===");

  // Declare pins as outputs
  pinMode(enablePin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // Initialize pins
  digitalWrite(stepPin, LOW);
  digitalWrite(dirPin, LOW);
  digitalWrite(enablePin, LOW); // Enable driver

  Serial.print("Steps per revolution: ");
  Serial.println(stepsPerRevolution);
  Serial.print("Acceleration steps: ");
  Serial.println(ACCEL_STEPS);

  // Flash LED 3 times on startup
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }

  Serial.println("Ready. Starting smooth motion...\n");
  delay(1000);
}

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

void loop()
{
  // // Forward rotation
  // digitalWrite(LED_PIN, HIGH);
  // Serial.println("=== FORWARD ROTATION ===");
  // smoothStep(stepsPerRevolution, true);

  // Serial.println("Forward complete.\n");
  // digitalWrite(LED_PIN, LOW);
  // delay(1500);

  // // Reverse rotation
  // digitalWrite(LED_PIN, HIGH);
  // Serial.println("=== REVERSE ROTATION ===");
  // smoothStep(stepsPerRevolution, false);

  // Serial.println("Reverse complete.\n");
  // digitalWrite(LED_PIN, LOW);
  // delay(1500);

  // Serial.println("--- Cycle Complete ---\n");
}
