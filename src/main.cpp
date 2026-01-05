/*
 * TMC2209 Stepper Motor with AS5600 Encoder for ESP32 DevKit V1
 *
 * ============================================================================
 * PIN DEFINITIONS - ESP32 DevKit V1
 * ============================================================================
 *
 * TMC2209 Stepper Driver -> ESP32 DevKit V1
 * ------------------------------------------
 * STEP     -> GPIO 26 (PWM capable)
 * DIR      -> GPIO 25 (PWM capable)
 * EN       -> GPIO 33 (active LOW - pull LOW to enable driver)
 * DIAG     -> GPIO 35 (StallGuard detection, input only, interrupt-capable)
 * PDN_UART -> GPIO 16 & 17 (Hardware Serial2, see UART wiring below)
 * VCC_IO   -> 3.3V (ESP32 logic level - IMPORTANT!)
 * GND      -> GND
 * VM       -> Motor power supply (4.75V - 29V, separate from ESP32)
 * A1, A2, B1, B2 -> Stepper motor coils
 *
 * TMC2209 UART Communication (Hardware Serial2)
 * ----------------------------------------------
 * RX (GPIO16) -> Connect via 1kΩ resistor to PDN_UART
 * TX (GPIO17) -> Connect directly to PDN_UART
 * NOTE: A 1kΩ resistor MUST be placed between RX (GPIO16) and PDN_UART
 *       TX (GPIO17) connects directly to PDN_UART
 *       Both pins connect to the same PDN_UART pin on TMC2209
 *       ESP32 uses Hardware Serial2 (no SoftwareSerial needed)
 *
 * AS5600 Magnetic Encoder -> ESP32 DevKit V1
 * -------------------------------------------
 * VCC    -> 3.3V (ESP32 logic level)
 * GND    -> GND
 * SDA    -> GPIO 21 (I2C SDA - default on ESP32, configurable)
 * SCL    -> GPIO 22 (I2C SCL - default on ESP32, configurable)
 *
 * Joystick -> ESP32 DevKit V1
 * -------------------------------------------
 * VRX    -> GPIO 34 (ADC1_CH6, input only, 12-bit ADC)
 * SW     -> GPIO 32 (button with internal pullup)
 * VCC    -> 3.3V
 * GND    -> GND
 *
 * Continuous Rotation Servo (360°) -> ESP32 DevKit V1
 * -------------------------------------------
 * Signal -> GPIO 27 (PWM capable)
 * VCC    -> 5V (external power recommended for higher current)
 * GND    -> GND (common ground with ESP32)
 *
 * Built-in LED
 * -------------------------------------------
 * LED    -> GPIO 2 (onboard LED on ESP32 DevKit V1)
 *
 * ============================================================================
 * IMPORTANT NOTES
 * ============================================================================
 *
 * 1. VOLTAGE LEVEL - CRITICAL:
 *    - ESP32 operates at 3.3V logic (NOT 5V like Arduino!)
 *    - TMC2209 VCC_IO MUST be connected to 3.3V
 *    - AS5600 supports both 3.3V and 5V (use 3.3V)
 *    - All modules must be 3.3V compatible
 *
 * 2. ESP32 I2C pins are configurable:
 *    - Default GPIO21 = SDA, GPIO22 = SCL
 *    - Can be changed by calling Wire.begin(sda, scl)
 *
 * 3. AS5600 Hardware Setup:
 *    - Place a diametric magnet above the AS5600 sensor (2-3mm distance)
 *    - The magnet should be centered over the sensor
 *    - Magnet polarity: one pole facing the sensor
 *    - Recommended magnet: 6mm diameter x 2-3mm thickness
 *
 * 4. Sensorless Homing (StallGuard):
 *    - DIAG pin connected to GPIO35 (interrupt-capable, input only)
 *    - ESP32 supports interrupts on all GPIO pins
 *    - StallGuard detects motor stall when hitting physical limits
 *    - Startup sequence: Move until stall, reverse, stall, then back off
 *    - STALL_VALUE controls sensitivity: higher = less sensitive
 *
 * 5. Power Supply:
 *    - TMC2209 VCC_IO: 3.3V from ESP32 (CRITICAL!)
 *    - TMC2209 VM: Separate motor power supply (typically 12V or 24V)
 *    - AS5600 VCC: 3.3V from ESP32
 *    - Always connect grounds together (ESP32 GND, motor PSU GND)
 *
 * 6. ESP32 ADC:
 *    - 12-bit resolution (0-4095) vs Arduino's 10-bit (0-1023)
 *    - Joystick calibration values scaled 4x from Arduino version
 *    - Using ADC1 to avoid conflicts with WiFi (if needed in future)
 *
 * 7. Serial Monitor:
 *    - Baud rate: 115200
 *    - Displays: Encoder angle, Degrees, Rotation count, Magnet status
 */

#include <Arduino.h>
#include <Wire.h>
#include <TMCStepper.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>
#include "EncoderReader.h"
#include "HardwareConfig.h"
#include "MotorControl.h"
#include "MarkerSystem.h"
#include "StateMachine.h"

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

// TMC2209 driver object using ESP32's Serial2 (Hardware UART)
TMC2209Stepper TMC_Driver(&Serial2, R_SENSE, DRIVER_ADDRESS);

// Create AccelStepper object using DRIVER mode (step/direction interface)
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Create encoder instance
EncoderReader encoder;

// Create subsystem objects
MotorControl motorControl(TMC_Driver, stepper, encoder);
MarkerSystem markerSystem(encoder, motorControl, stepper);
StateMachine stateMachine(encoder, motorControl, markerSystem, stepper);

// Continuous rotation servo
Servo servo360;

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

  // Initialize joystick pins
  pinMode(JOYSTICK_VRX, INPUT);
  pinMode(JOYSTICK_SW, INPUT_PULLUP); // Use internal pull-up for button
  Serial.println(F("[HW] Joystick initialized on A0 and D2"));

  // IMPORTANT: Enable driver in hardware BEFORE UART communication
  digitalWrite(ENABLE_PIN, LOW); // Enable driver (active LOW)
  Serial.println(F("[HW] Driver ON"));

  // Initialize AccelStepper
  stepper.setMaxSpeed(1000.0);     // Safe reliable speed (per manual)
  stepper.setAcceleration(2000.0); // Conservative acceleration (2x speed)
  stepper.setCurrentPosition(0);   // Set current position as zero
  Serial.print(F("[ACCEL] MaxSpd="));
  Serial.print(stepper.maxSpeed());
  Serial.print(F(" Accel="));
  Serial.print(stepper.acceleration());
  Serial.print(F(" Steps/rev="));
  Serial.println(TOTAL_STEPS);

  // Check initial DIAG pin state
  Serial.print(F("[HW] DIAG initial="));
  Serial.println(digitalRead(DIAG_PIN));
  Serial.println(F("[HW] INT will attach in homeX"));

  // Blink LED 3 times on startup
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }

  // Initialize TMC2209 UART communication using ESP32 Serial2
  Serial.println(F("\n[TMC] Init UART"));
  Serial2.begin(115200, SERIAL_8N1, TMC_UART_RX, TMC_UART_TX);
  delay(100);

  // Configure TMC2209
  TMC_Driver.begin();
  TMC_Driver.toff(TOFF_VALUE);
  TMC_Driver.blank_time(24);
  TMC_Driver.rms_current(RUN_CURRENT);
  TMC_Driver.microsteps(MICROSTEPS);
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

  // Initialize I2C with ESP32 pins (GPIO21=SDA, GPIO22=SCL)
  Serial.println(F("[I2C] Init GPIO21/GPIO22"));
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
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

  // Initialize subsystems
  motorControl.begin();
  markerSystem.begin();
  stateMachine.begin();

  // Initialize continuous rotation servo
  Serial.println(F("[SERVO] Initializing 360° servo"));
  servo360.attach(SERVO_360_PIN);
  servo360.write(SERVO_STOP); // Start in stopped position
  Serial.print(F("[SERVO] Attached to GPIO"));
  Serial.print(SERVO_360_PIN);
  Serial.println(F(", controlled by joystick"));

  Serial.println(F("\n--- Ready ---"));
  Serial.println(F("Homing on startup..."));
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop()
{
  // All logic handled by state machine
  stateMachine.update();

  // Handle continuous rotation servo control with joystick
  // Read joystick potentiometer (0-4095 on ESP32 12-bit ADC)
  int vrxValue = analogRead(JOYSTICK_VRX);

  // Map joystick position to servo speed
  // Center (2048) = stop (90), Left (0) = full CCW (180), Right (4095) = full CW (0)
  int servoSpeed;

  if (abs(vrxValue - JOYSTICK_CENTER) < JOYSTICK_DEADZONE)
  {
    // Within deadzone - stop the servo
    servoSpeed = SERVO_STOP;
  }
  else if (vrxValue < JOYSTICK_CENTER)
  {
    // Left side - counter-clockwise rotation
    // Map from (0 to CENTER-DEADZONE) to (CCW_FULL to STOP)
    servoSpeed = map(vrxValue, 0, JOYSTICK_CENTER - JOYSTICK_DEADZONE, SERVO_CCW_FULL, SERVO_STOP);
  }
  else
  {
    // Right side - clockwise rotation
    // Map from (CENTER+DEADZONE to 4095) to (STOP to CW_FULL)
    servoSpeed = map(vrxValue, JOYSTICK_CENTER + JOYSTICK_DEADZONE, 4095, SERVO_STOP, SERVO_CW_FULL);
  }

  servo360.write(servoSpeed);

  // Debug output every 500ms
  static unsigned long lastServoDebug = 0;
  if (millis() - lastServoDebug > 500)
  {
    Serial.print(F("[SERVO] Joy: "));
    Serial.print(vrxValue);
    Serial.print(F(" → Speed: "));
    Serial.print(servoSpeed);
    if (servoSpeed == SERVO_STOP)
      Serial.println(F(" (STOP)"));
    else if (servoSpeed < SERVO_STOP)
      Serial.println(F(" (CW)"));
    else
      Serial.println(F(" (CCW)"));
    lastServoDebug = millis();
  }
}
