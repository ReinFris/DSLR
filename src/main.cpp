/*
 * TMC2209 Stepper Motor with AS5600 Encoder for Arduino Uno
 *
 * ============================================================================
 * PIN DEFINITIONS - ARDUINO UNO
 * ============================================================================
 *
 * TMC2209 Stepper Driver -> Arduino Uno
 * ----------------------------------------
 * STEP     -> Digital Pin 5
 * DIR      -> Digital Pin 4
 * EN       -> Digital Pin 6 (active LOW - pull LOW to enable driver)
 * DIAG     -> Digital Pin 3 (StallGuard detection, INT1, interrupt-capable)
 * PDN_UART -> Digital Pins 7 & 8 (see UART wiring below)
 * VCC_IO   -> 5V (Arduino logic level)
 * GND      -> GND
 * VM       -> Motor power supply (4.75V - 29V, separate from Arduino)
 * A1, A2, B1, B2 -> Stepper motor coils
 *
 * TMC2209 UART Communication (for sensorless homing)
 * ---------------------------------------------------
 * SW_RX    -> Digital Pin 7 (connect via 1kΩ resistor to PDN_UART)
 * SW_TX    -> Digital Pin 8 (connect directly to PDN_UART)
 * NOTE: A 1kΩ resistor MUST be placed between SW_RX (pin 7) and PDN_UART
 *       SW_TX (pin 8) connects directly to PDN_UART
 *       Both pins connect to the same PDN_UART pin on TMC2209
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
 * 3. Sensorless Homing (StallGuard):
 *    - DIAG pin must be connected to pin 3 (interrupt-capable)
 *    - StallGuard detects motor stall when hitting physical limits
 *    - Startup sequence: Move until stall, reverse, stall, then back off
 *    - STALL_VALUE (10) controls sensitivity: higher = less sensitive
 *
 * 4. Power Supply:
 *    - TMC2209 VCC_IO: 5V from Arduino
 *    - TMC2209 VM: Separate motor power supply (typically 12V or 24V)
 *    - AS5600 VCC: 5V from Arduino (AS5600 supports 3.3V-5V)
 *    - Always connect grounds together (Arduino GND, motor PSU GND)
 *
 * 5. Serial Monitor:
 *    - Baud rate: 115200
 *    - Displays: Encoder angle, Degrees, Rotation count, Magnet status
 */

#include <Arduino.h>
#include <Wire.h>
#include <TMCStepper.h>
#include <SoftwareSerial.h>
#include "EncoderReader.h"

// ============================================================================
// PIN ASSIGNMENTS FOR ARDUINO UNO
// ============================================================================

// Stepper motor control pins
#define STEP_PIN 5   // Digital pin 5 - Step pulse for TMC2209
#define DIR_PIN 4    // Digital pin 4 - Direction control for TMC2209
#define ENABLE_PIN 6 // Digital pin 6 - Enable pin (LOW = enabled, HIGH = disabled)
#define DIAG_PIN 3   // Digital pin 3 - StallGuard DIAG (INT1 - interrupt-capable)

// TMC2209 UART pins (SoftwareSerial)
#define SW_RX 7 // Digital pin 7 - RX via 1kΩ resistor to PDN_UART
#define SW_TX 8 // Digital pin 8 - TX directly to PDN_UART

// Status LED
#define LED_PIN 13 // Digital pin 13 - Built-in LED on Arduino Uno

// I2C pins for AS5600 encoder
// NOTE: These are FIXED on Arduino Uno and cannot be changed
// Do NOT attempt to use other pins for I2C on Uno - it will not work
#define I2C_SDA_PIN A4 // Analog pin A4 - I2C SDA (FIXED on Uno)
#define I2C_SCL_PIN A5 // Analog pin A5 - I2C SCL (FIXED on Uno)

// ============================================================================
// MOTOR & TMC2209 CONFIGURATION
// ============================================================================

// Motor parameters
#define STEPS_PER_REV 200 // Standard stepper motor (1.8° per step)
#define MICROSTEPS 16     // Microsteps setting for TMC2209
#define STEP_DELAY 100    // Microseconds between steps (for homing)

// TMC2209 Configuration
#define R_SENSE 0.11f       // SilentStepStick series use 0.11 Ohm
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address (MS1 and MS2 to GND)
#define RUN_CURRENT 500     // Motor current in mA (adjust for your motor)
#define STALL_VALUE 2       // StallGuard threshold [0..255] (lower = more sensitive)
#define TOFF_VALUE 4        // Off time setting [1..15]

// Movement parameters
#define HOMING_BACKOFF_STEPS 5000 // Steps to back off after hitting stall

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

// Create SoftwareSerial for TMC2209 UART communication
SoftwareSerial SoftSerial(SW_RX, SW_TX);

// Create TMC2209 driver object
TMC2209Stepper TMC_Driver(&SoftSerial, R_SENSE, DRIVER_ADDRESS);

// Create encoder instance
EncoderReader encoder;

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

volatile bool stallDetected = false; // Flag set by interrupt
bool shaftVal = false;               // Direction: false = forward, true = reverse
bool startup = true;                 // Run homing on startup

// Update interval for serial output (milliseconds)
const unsigned long PRINT_INTERVAL = 100; // 10Hz update rate
unsigned long lastPrintTime = 0;

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

// Generate step pulses for motor movement
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

// Homing routine - uses StallGuard to find limits
void homeX()
{
  const int homeDelay = STEP_DELAY;

  Serial.println(F("Homing X"));
  shaftVal = true;

  // Clear any initial stall flag (motor not moving yet)
  stallDetected = false;
  delay(100);

  // Check DIAG before starting movement
  Serial.print(F("[Home] DIAG before move="));
  Serial.println(digitalRead(DIAG_PIN));

  // Start motor movement to initialize StallGuard
  motor(2000, homeDelay); // Move a bit first
  delay(50);

  // NOW attach the interrupt after motor is moving
  Serial.print(F("[Home] DIAG after startup move="));
  Serial.println(digitalRead(DIAG_PIN));

  // Clear flag before attaching interrupt
  stallDetected = false;
  attachInterrupt(digitalPinToInterrupt(DIAG_PIN), stallInterruptHandler, RISING);
  delay(10);             // Small delay for interrupt to stabilize
  stallDetected = false; // Clear again in case of spurious trigger

  Serial.println(F("[Home] Interrupt attached"));

  // Fast approach to first side
  int loopCount = 0;
  while (!stallDetected)
  {
    motor(1000, homeDelay);
    loopCount++;

    // Debug StallGuard values during homing
    uint16_t sg_result = TMC_Driver.SG_RESULT();
    uint8_t diag_state = digitalRead(DIAG_PIN);
    Serial.print(F("[Home] Loop="));
    Serial.print(loopCount);
    Serial.print(F(" SG="));
    Serial.print(sg_result);
    Serial.print(F(" DIAG="));
    Serial.println(diag_state);
  }

  Serial.print(F("[Home] Loops before stall: "));
  Serial.println(loopCount);

  // Stall detected - print final values
  uint16_t sg_result = TMC_Driver.SG_RESULT();
  Serial.print(F("[Home] STALL! SG="));
  Serial.print(sg_result);
  Serial.print(F(" DIAG="));
  Serial.println(digitalRead(DIAG_PIN));

  stallDetected = false;
  delay(1000);

  Serial.println(F("[Home] DONE!"));
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

  // IMPORTANT: Enable driver in hardware BEFORE UART communication
  digitalWrite(ENABLE_PIN, LOW); // Enable driver (active LOW)
  Serial.println(F("[HW] Driver ON"));

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

  // Initialize TMC2209 UART communication
  Serial.println(F("\n[TMC] Init UART"));
  SoftSerial.begin(115200);
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

  // Initialize I2C
  Serial.println(F("[I2C] Init A4/A5"));
  Wire.begin();
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

  // ============================================================================
  // STARTUP HOMING
  // ============================================================================

  if (startup)
  {
    startup = false;
    homeX();
  }

  // ============================================================================
  // STALL DETECTION DURING MOVEMENT
  // ============================================================================

  if (stallDetected)
  {
    const int backSteps = 1000;

    // Debug: Print StallGuard values when stall is detected
    uint16_t sg_result = TMC_Driver.SG_RESULT();
    uint8_t diag_state = digitalRead(DIAG_PIN);
    uint32_t drv_status = TMC_Driver.DRV_STATUS();

    Serial.println(F("Stalled"));
    Serial.print(F("  SG_RESULT="));
    Serial.println(sg_result);
    Serial.print(F("  DIAG_PIN="));
    Serial.println(diag_state);
    Serial.print(F("  DRV_STATUS=0x"));
    Serial.println(drv_status, HEX);

    stallDetected = false;
    shaftVal = !shaftVal; // Reverse direction
    motor(backSteps, STEP_DELAY);
  }

  // ============================================================================
  // SERIAL COMMANDS
  // ============================================================================

  if (Serial.available() > 0)
  {
    char readVal = Serial.read();

    // Move X steps: "x1000" or "x-1000"
    if (readVal == 'x')
    {
      int steps = Serial.parseInt();

      if (steps < 0)
      {
        shaftVal = true;
        steps *= -1;
      }
      else
      {
        shaftVal = false;
      }

      motor(steps, STEP_DELAY);
    }

    // Home command: "h"
    if (readVal == 'h')
    {
      homeX();
    }
  }

  // ============================================================================
  // ENCODER STATUS OUTPUT
  // ============================================================================

  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= PRINT_INTERVAL)
  {
    lastPrintTime = currentTime;

    // Get encoder readings
    uint16_t rawAngle = encoder.getRawAngle();
    float degrees = encoder.getDegrees();
    long rotations = encoder.getRotationCount();

    // Format and print data
    int deg_int = (int)degrees;
    int deg_frac = (int)((degrees - deg_int) * 100);

    Serial.print(rawAngle);
    Serial.print(F("|"));
    Serial.print(deg_int);
    Serial.print(F("."));
    Serial.print(deg_frac);
    Serial.print(F("|"));
    Serial.print(rotations);
    Serial.print(F("|"));

    // Magnet status
    if (!encoder.detectMagnet())
      Serial.print(F("N"));
    else if (encoder.magnetTooStrong())
      Serial.print(F("S"));
    else if (encoder.magnetTooWeak())
      Serial.print(F("W"));
    else
      Serial.print(F("O"));

    // StallGuard debug info
    uint16_t sg_result = TMC_Driver.SG_RESULT();
    uint8_t diag_state = digitalRead(DIAG_PIN);

    Serial.print(F("|SG="));
    Serial.print(sg_result);
    Serial.print(F("|DIAG="));
    Serial.println(diag_state);
  }

  // Blink LED to show we're running
  static unsigned long lastBlink = 0;
  if (currentTime - lastBlink >= 1000)
  {
    lastBlink = currentTime;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}
