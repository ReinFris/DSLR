// TMC2209 Test Script - adapted to use existing wiring

#include <Arduino.h>
#include <TMCStepper.h>
#include <SoftwareSerial.h>

// Commenting out for test
// #include <Wire.h>
// #include <AccelStepper.h>
// #include "EncoderReader.h"

// AS5600 Encoder I2C Pins (for reference - Wire library uses these by default)
// For Arduino Uno/Nano: SDA = A4, SCL = A5
// For Arduino Mega: SDA = 20, SCL = 21
// Note: Most AS5600 modules have built-in pull-up resistors
#define SDA_PIN A4 // I2C Data
#define SCL_PIN A5 // I2C Clock

// TMC2209 Stepper Driver Pins
#define STEP_PIN 5
#define DIR_PIN 4
#define ENABLE_PIN 6
#define SW_RX 7      // SoftwareSerial RX (connect to TMC2209 PDN_UART via 1k resistor to SW_TX)
#define SW_TX 8      // SoftwareSerial TX (connect to TMC2209 PDN_UART)
#define STALLGUARD 3 // DIAG pin from TMC2209

// TMC2209 Configuration
#define R_SENSE 0.11f       // SilentStepStick series use 0.11 Ohm
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define RUN_CURRENT 700     // Motor current in mA (adjust based on your motor specs)
#define MICROSTEPS 256      // Microsteps setting (1, 2, 4, 8, 16, 32, 64, 128, 256)

// StallGuard Configuration
#define STALL_VALUE 20 // 10 works ok false positive [0..255] - Higher = less sensitive, Lower = more sensitive
#define TOFF_VALUE 4   // [1..15] Off time setting

// Other I/O Pins
#define LED_PIN 13
#define JOYSTICK_VRX A0
#define JOYSTICK_SW 2
#define STEPS_PER_REV 200
#define TOTAL_STEPS (STEPS_PER_REV * MICROSTEPS)

// Homing sequence configuration
#define HOMING_SPEED 60000      // 60000 works Slower speed for homing (VACTUAL units)
#define BACKOFF_TIME 200        // Time in ms to back off after second stall (small fraction)
#define PAUSE_BETWEEN_HOMES 500 // Time in ms to pause between first and second homing

// Homing sequence states
enum StartupState
{
  STARTUP_MOVING_FIRST_SIDE,
  STARTUP_PAUSING,
  STARTUP_MOVING_SECOND_SIDE,
  STARTUP_BACKING_OFF,
  STARTUP_COMPLETE
};

// Create SoftwareSerial for TMC2209 UART communication
SoftwareSerial SoftSerial(SW_RX, SW_TX);

// Create TMC2209 driver object
TMC2209Stepper TMC_Driver(&SoftSerial, R_SENSE, DRIVER_ADDRESS);

// Global state variables
StartupState startupState = STARTUP_MOVING_FIRST_SIDE;
volatile bool stallDetected = false; // Flag set by interrupt

//== Interrupt Service Routine ==================================================================

// ISR for DIAG pin - triggered when stall is detected
void stallInterruptHandler()
{
  stallDetected = true;
}

//== Setup ======================================================================================

void setup()
{

  Serial.begin(115200); // initialize hardware serial for debugging
  while (!Serial)
    ;                       // Wait for serial port to connect
  SoftSerial.begin(115200); // initialize software serial for UART motor control

  // IMPORTANT: Enable driver in hardware BEFORE UART communication
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // Enable driver in hardware (active LOW)
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  // Attach interrupt to DIAG pin (pin 3 supports interrupts on most Arduino boards)
  // RISING edge: when DIAG goes HIGH, a stall is detected
  attachInterrupt(digitalPinToInterrupt(STALLGUARD), stallInterruptHandler, RISING);

  Serial.println("TMC2209 Sensorless Homing");
  Serial.println("Wiring: EN=6, DIR=4, STEP=5, SW_RX=7, SW_TX=8, DIAG=3");
  Serial.println("IMPORTANT: VIO to 5V, GND to GND, 1k resistor between pins 7 and 8 to PDN_UART");
  delay(500);

  TMC_Driver.beginSerial(115200); // Initialize UART

  TMC_Driver.begin();          // UART: Init SW UART (if selected) with default 115200 baudrate
  TMC_Driver.toff(5);          // Enables driver in software
  TMC_Driver.rms_current(770); // Set motor RMS current (770mA)
  TMC_Driver.microsteps(256);  // Set microsteps to 256

  // StallGuard configuration
  TMC_Driver.TCOOLTHRS(0xFFFFF);  // 20bit max - Enable StallGuard (velocity threshold)
  TMC_Driver.semin(5);            // Minimum StallGuard value for smart current control
  TMC_Driver.semax(2);            // Maximum StallGuard value for smart current control
  TMC_Driver.sedn(0b00);          // Current down step speed
  TMC_Driver.SGTHRS(STALL_VALUE); // StallGuard threshold [0..255]

  TMC_Driver.en_spreadCycle(false); // Toggle spreadCycle (false = use StealthChop)
  TMC_Driver.pwm_autoscale(true);   // Needed for stealthChop

  Serial.println("TMC2209 configured with StallGuard!");
  Serial.println("");
  Serial.println("=== HOMING SEQUENCE ===");
  Serial.println("Moving to first side until stall detected (slower speed)...");

  // Start moving in positive direction at slower homing speed
  TMC_Driver.VACTUAL(HOMING_SPEED);

  delay(500);
}

//== Loop ========================================================================================

void loop()
{
  uint32_t ms = millis();

  // Read DIAG pin for stall detection
  bool diagPin = digitalRead(STALLGUARD);

  //== HOMING SEQUENCE HANDLING ===================================================================
  if (startupState != STARTUP_COMPLETE)
  {
    static uint32_t last_diag_check = 0;

    // Print status during homing every 100ms (BEFORE checking for stall)
    if ((ms - last_diag_check) > 10)
    {
      last_diag_check = ms;

      Serial.print("Homing: State=");
      if (startupState == STARTUP_MOVING_FIRST_SIDE)
        Serial.print("FIRST_SIDE");
      else if (startupState == STARTUP_PAUSING)
        Serial.print("PAUSING");
      else if (startupState == STARTUP_MOVING_SECOND_SIDE)
        Serial.print("SECOND_SIDE");
      else if (startupState == STARTUP_BACKING_OFF)
        Serial.print("BACKING_OFF");
      Serial.print(" DIAG=");
      Serial.print(diagPin);
      Serial.print(" SG=");
      Serial.print(TMC_Driver.SG_RESULT(), DEC);
      Serial.print(" MSCNT=");
      Serial.print(TMC_Driver.MSCNT());
      Serial.print(" I=");
      Serial.print(TMC_Driver.cs2rms(TMC_Driver.cs_actual()), DEC);
      Serial.println("mA");
    }

    // Check for stall detection via interrupt flag
    if (stallDetected)
    {
      stallDetected = false; // Clear the flag

      Serial.print("SG_RESULT: ");
      Serial.println(TMC_Driver.SG_RESULT());
      Serial.print("MSCNT: ");
      Serial.println(TMC_Driver.MSCNT());

      // Stall detected via interrupt!
      if (startupState == STARTUP_MOVING_FIRST_SIDE)
      {
        Serial.println(">>> STALL DETECTED on first side!");
        Serial.println(">>> Pausing for 500ms...");

        // Stop motor during pause
        TMC_Driver.VACTUAL(0);
        startupState = STARTUP_PAUSING;
      }
      else if (startupState == STARTUP_MOVING_SECOND_SIDE)
      {
        Serial.println(">>> STALL DETECTED on second side!");
        Serial.println(">>> Backing off a small fraction...");

        // Back off a small amount in positive direction
        TMC_Driver.VACTUAL(HOMING_SPEED);
        startupState = STARTUP_BACKING_OFF;
      }
    }

    // Handle PAUSING state (pause after first stall before moving to second side)
    if (startupState == STARTUP_PAUSING)
    {
      static uint32_t pauseStartTime = 0;
      if (pauseStartTime == 0)
      {
        pauseStartTime = ms;
      }

      // After pause duration, start moving to second side
      if ((ms - pauseStartTime) > PAUSE_BETWEEN_HOMES)
      {
        pauseStartTime = 0;

        Serial.println(">>> Moving to second side...");

        // Move to second side (negative direction) at slow homing speed
        TMC_Driver.VACTUAL(-HOMING_SPEED);
        startupState = STARTUP_MOVING_SECOND_SIDE;
      }
    }

    // Handle BACKING_OFF state (time-based short movement)
    if (startupState == STARTUP_BACKING_OFF)
    {
      static uint32_t backoffStartTime = 0;
      if (backoffStartTime == 0)
      {
        backoffStartTime = ms;
      }

      // After brief backoff time, stop completely
      if ((ms - backoffStartTime) > BACKOFF_TIME)
      {
        TMC_Driver.VACTUAL(0); // Stop motor
        backoffStartTime = 0;

        Serial.println(">>> HOMING COMPLETE!");
        Serial.println(">>> Motor stopped.");

        startupState = STARTUP_COMPLETE;
      }
    }

    // During homing, skip everything else
    return;
  }

  //== NORMAL OPERATION - MOTOR STOPPED ============================================================

  // Motor is stopped after homing - do nothing
  // You can add code here later for manual control via serial commands if needed
}
