/*
 * STEPPER MOTOR TIMELAPSE CONTROLLER WITH MINIMUM SNAP TRAJECTORY
 *
 * This program simulates a camera slider timelapse system using a stepper motor.
 * The motor moves smoothly along a path, stopping at regular intervals to simulate
 * photo capture, then continues to the next position.
 *
 * TIMELAPSE MODE (ACTIVE):
 * - Divides the full motion into segments with stops in between
 * - Each stop includes a dwell period (simulating camera shutter)
 * - LED flashes during each stop to indicate photo capture
 * - Uses minimum snap trajectory for ultra-smooth motion between stops
 *
 * TRAJECTORY: MINIMUM SNAP (7th order polynomial)
 * - Minimizes the 4th derivative of position (snap/jounce)
 * - Optimal for reducing vibration and mechanical stress
 * - Industry standard for drones, CNC machines, and precision robotics
 *
 * OTHER AVAILABLE TRAJECTORIES:
 * - Raised Cosine           - Trigonometric, extremely smooth
 * - Sine Curve              - S-curve based on sine function
 * - Tanh Sigmoid            - Hyperbolic tangent, adjustable steepness
 * - Logistic Sigmoid        - Neural network-style S-curve
 * - Smootherstep (5th order)- Perlin noise function
 *
 * CONFIGURATION:
 * - Change number of stops: Edit TIMELAPSE_STOPS constant
 * - Change dwell time: Edit DWELL_TIME_MS constant
 * - Change trajectory: Edit accelerationCurve() function
 * - Disable timelapse: Uncomment simple mode in loop()
 */

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

// Timelapse parameters
#define TIMELAPSE_STOPS 3  // Number of stops for photo capture
#define DWELL_TIME_MS 2000 // Time to pause at each stop (milliseconds)

int currentDelay = MIN_SPEED;

// ========================================
// ACCELERATION CURVE FUNCTIONS
// All functions map input t (0.0 to 1.0) to output (0.0 to 1.0)
// ========================================

// --- POLYNOMIAL-BASED CURVES ---

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
// Zero 1st and 2nd derivatives at edges
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

// --- TRIGONOMETRIC CURVES (Excellent for robotics) ---

// Raised Cosine (Hann window function)
// Extremely smooth, zero jerk at endpoints
// Widely used in signal processing and motion control
// RECOMMENDED for robotics - smoother than polynomial curves
float raisedCosine(float t)
{
  if (t <= 0.0f)
    return 0.0f;
  if (t >= 1.0f)
    return 1.0f;
  // 0.5 * (1 - cos(π*t))
  return 0.5f * (1.0f - cos(PI * t));
}

// Sine-based S-curve
// Very smooth acceleration profile
// Good for reducing mechanical stress
float sineCurve(float t)
{
  if (t <= 0.0f)
    return 0.0f;
  if (t >= 1.0f)
    return 1.0f;
  // sin((t - 0.5) * π) mapped from [-1,1] to [0,1]
  return (sin((t - 0.5f) * PI) + 1.0f) * 0.5f;
}

// --- MINIMUM SNAP TRAJECTORY (Optimal for robotics) ---

// Minimum Snap 7th Order Polynomial Trajectory
//
// BACKGROUND:
// In robotics and motion control, "snap" (also called jounce) is the 4th derivative
// of position with respect to time: snap = d⁴x/dt⁴
//
// Derivative hierarchy:
//   Position:     x(t)
//   Velocity:     dx/dt    (1st derivative)
//   Acceleration: d²x/dt²  (2nd derivative)
//   Jerk:         d³x/dt³  (3rd derivative)
//   Snap:         d⁴x/dt⁴  (4th derivative)
//
// WHY MINIMIZE SNAP?
// - For quadrotors/drones: Snap is directly related to the rate of change of rotor thrust
// - For stepper motors: Minimizing snap reduces vibration and mechanical stress
// - Lower snap = smoother control inputs = less energy waste = longer component life
//
// MATHEMATICAL FORMULATION:
// This curve solves the optimization problem:
//   minimize: ∫₀¹ (d⁴x/dt⁴)² dt
//   subject to boundary conditions:
//     x(0) = 0,  x'(0) = 0,  x''(0) = 0,  x'''(0) = 0
//     x(1) = 1,  x'(1) = 0,  x''(1) = 0,  x'''(1) = 0
//
// The solution is the 7th order polynomial:
//   x(t) = 35t⁴ - 84t⁵ + 70t⁶ - 20t⁷
//
// BENEFITS FOR STEPPER MOTORS:
//   - Minimizes control effort and energy consumption
//   - Extremely smooth motion with continuous derivatives up to 3rd order
//   - Reduces mechanical wear, vibration, and audible noise
//   - Industry standard for UAVs, drones, and precision CNC machines
//   - Optimal for applications requiring precise positioning with minimal overshoot
float minimumSnap(float t)
{
  if (t <= 0.0f)
    return 0.0f;
  if (t >= 1.0f)
    return 1.0f;

  // Optimized Horner's method evaluation of: 35t⁴ - 84t⁵ + 70t⁶ - 20t⁷
  float t2 = t * t;   // t²
  float t4 = t2 * t2; // t⁴

  // 35t⁴ - 84t⁵ + 70t⁶ - 20t⁷ = t⁴(35 - t(84 - t(70 - 20t)))
  return t4 * (35.0f - t * (84.0f - t * (70.0f - 20.0f * t)));
}

// --- SIGMOID-BASED CURVES (True S-curves) ---

// Tanh-based sigmoid (hyperbolic tangent S-curve)
// Produces a true S-curve with excellent smoothness
// Excellent jerk limiting - ideal for precision robotics
// Steepness parameter controls how aggressive the transition is
float tanhSigmoid(float t, float steepness = 6.0f)
{
  if (t <= 0.0f)
    return 0.0f;
  if (t >= 1.0f)
    return 1.0f;

  // Map t from [0,1] to [-steepness, steepness] then apply tanh
  // tanh output is [-1,1], map to [0,1]
  float x = (t - 0.5f) * 2.0f * steepness; // Map to [-steepness, steepness]
  float tanhValue = tanh(x);
  return (tanhValue + 1.0f) * 0.5f;
}

// Logistic sigmoid S-curve
// Another true S-curve, slightly different characteristics than tanh
// Used in neural networks and control systems
float logisticSigmoid(float t, float steepness = 12.0f)
{
  if (t <= 0.0f)
    return 0.0f;
  if (t >= 1.0f)
    return 1.0f;

  // Map t to [-6, 6] range (adjustable with steepness)
  float x = (t - 0.5f) * steepness;
  // Logistic function: 1 / (1 + e^(-x))
  return 1.0f / (1.0f + exp(-x));
}

// --- CURRENT ACTIVE CURVE ---
// Change this function to switch between different acceleration profiles
float accelerationCurve(float t)
{
  // Try these options:
  // return smootherstep(t);      // Original (5th order polynomial)
  // return smootheststep(t);     // 7th order polynomial (same as minimumSnap)
  // return minimumSnap(t);       // Minimum snap trajectory (optimal for robotics)
  // return raisedCosine(t);      // Cosine-based (very smooth, no abrupt transitions)
  // return sineCurve(t);         // Sine-based S-curve
  // return tanhSigmoid(t, 3.0f); // Tanh sigmoid (lower steepness = gentler)
  // return tanhSigmoid(t, 5.0f); // Tanh sigmoid (moderate steepness)
  // return logisticSigmoid(t);   // Logistic sigmoid

  return minimumSnap(t); // ACTIVE: Minimum snap - optimal for stepper motors
}

// Map a value using acceleration curve interpolation (better than linear map)
float smoothMap(float x, float in_min, float in_max, float out_min, float out_max)
{
  // Normalize input to 0-1 range
  float t = (x - in_min) / (in_max - in_min);
  // Apply acceleration curve
  float smooth_t = accelerationCurve(t);
  // Map to output range
  return out_min + smooth_t * (out_max - out_min);
}

void setup()
{
  // Initialize serial for debugging
  Serial.begin(9600);
  delay(1000);
  Serial.println("========================================");
  Serial.println("  STEPPER MOTOR TIMELAPSE CONTROLLER");
  Serial.println("  Minimum Snap Trajectory");
  Serial.println("========================================");

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
  Serial.print("Timelapse stops: ");
  Serial.println(TIMELAPSE_STOPS);
  Serial.print("Dwell time per stop: ");
  Serial.print(DWELL_TIME_MS);
  Serial.println(" ms");

  // Flash LED 3 times on startup
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }

  Serial.println("\nReady. Starting timelapse motion...\n");
  delay(1000);
}

// Execute a single smooth motion segment
void smoothStep(int totalSteps, bool forward)
{
  digitalWrite(dirPin, forward ? HIGH : LOW);
  delay(5); // Direction setup time

  for (int i = 0; i < totalSteps; i++)
  {
    float t;           // Normalized position (0.0 to 1.0)
    float speedFactor; // 0.0 (stopped) to 1.0 (full speed)

    // Calculate speed with acceleration/deceleration using selected curve
    if (i < ACCEL_STEPS)
    {
      // Accelerate: 0.0 to 1.0 speed
      t = (float)i / (float)ACCEL_STEPS;
      speedFactor = accelerationCurve(t);
    }
    else if (i >= (totalSteps - ACCEL_STEPS))
    {
      // Decelerate: 1.0 to 0.0 speed
      t = (float)(i - (totalSteps - ACCEL_STEPS)) / (float)ACCEL_STEPS;
      speedFactor = 1.0f - accelerationCurve(t);
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

// Timelapse motion: Move through totalSteps with multiple stops for photo capture
// This simulates a camera slider moving, stopping for a photo, then continuing
void timelapseMotion(int totalSteps, bool forward, int numStops)
{
  Serial.print("Starting timelapse motion with ");
  Serial.print(numStops);
  Serial.println(" stops...");

  // Calculate steps per segment (divide total motion into segments)
  // numStops = 3 means 4 segments of motion
  int numSegments = numStops + 1;
  int stepsPerSegment = totalSteps / numSegments;

  for (int segment = 0; segment < numSegments; segment++)
  {
    // Print segment info
    Serial.print("  Segment ");
    Serial.print(segment + 1);
    Serial.print("/");
    Serial.print(numSegments);
    Serial.print(" - Moving ");
    Serial.print(stepsPerSegment);
    Serial.println(" steps...");

    // Move this segment with smooth acceleration/deceleration
    smoothStep(stepsPerSegment, forward);

    // Stop and dwell (except after the last segment)
    if (segment < numStops)
    {
      Serial.print("  >> Stop ");
      Serial.print(segment + 1);
      Serial.print(" - Dwelling for ");
      Serial.print(DWELL_TIME_MS);
      Serial.println("ms (simulating photo capture)");

      // Flash LED rapidly during dwell (simulating camera shutter)
      for (int flash = 0; flash < 3; flash++)
      {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
      }

      // Remaining dwell time
      delay(DWELL_TIME_MS - 600);
    }
  }

  Serial.println("Timelapse motion complete!\n");
}

void loop()
{
  // ========================================
  // TIMELAPSE MODE: Move with periodic stops
  // ========================================

  // // Forward timelapse motion
  // digitalWrite(LED_PIN, HIGH);
  // Serial.println("=== FORWARD TIMELAPSE (Minimum Snap) ===");
  // timelapseMotion(stepsPerRevolution, true, TIMELAPSE_STOPS);

  // digitalWrite(LED_PIN, LOW);
  // Serial.println("Pausing before return journey...\n");
  // delay(2000);

  // // Reverse timelapse motion (return to start)
  // digitalWrite(LED_PIN, HIGH);
  // Serial.println("=== REVERSE TIMELAPSE (Minimum Snap) ===");
  // timelapseMotion(stepsPerRevolution, false, 0);

  // digitalWrite(LED_PIN, LOW);
  // Serial.println("=================================");
  // Serial.println("Timelapse cycle complete!");
  // Serial.println("=================================\n");
  // delay(3000);

  // ========================================
  // SIMPLE MODE (commented out)
  // Uncomment below for continuous motion without stops
  // ========================================
  /*
  digitalWrite(LED_PIN, HIGH);
  Serial.println("=== FORWARD ROTATION (Minimum Snap Trajectory) ===");
  smoothStep(stepsPerRevolution, true);

  Serial.println("Forward complete.\n");
  digitalWrite(LED_PIN, LOW);
  delay(1500);

  digitalWrite(LED_PIN, HIGH);
  Serial.println("=== REVERSE ROTATION (Minimum Snap Trajectory) ===");
  smoothStep(stepsPerRevolution, false);

  Serial.println("Reverse complete.\n");
  digitalWrite(LED_PIN, LOW);
  delay(1500);

  Serial.println("--- Cycle Complete ---\n");
  */
}
