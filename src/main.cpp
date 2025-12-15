// TMC2209 Library Test - adapted for Arduino Uno with SoftwareSerial
// Using existing wiring: EN=6, DIR=4, STEP=5, SW_RX=7, SW_TX=8

#include <Arduino.h>
#include <TMC2209.h>
#include <SoftwareSerial.h>

// TMC2209 Stepper Driver Pins (using your existing wiring)
const uint8_t STEP_PIN = 5;
const uint8_t DIRECTION_PIN = 4;
const uint8_t ENABLE_PIN = 6;
const uint8_t SW_RX = 7;      // SoftwareSerial RX (connect to TMC2209 PDN_UART via 1k resistor to SW_TX)
const uint8_t SW_TX = 8;      // SoftwareSerial TX (connect to TMC2209 PDN_UART)

// Motion parameters
const uint32_t STEP_COUNT = 51200;
const uint16_t HALF_STEP_DURATION_MICROSECONDS = 10;
const uint16_t STOP_DURATION = 1000;

// Current values may need to be reduced to prevent overheating depending on
// specific motor and power supply voltage
const uint8_t RUN_CURRENT_PERCENT = 100;

// Create SoftwareSerial for TMC2209 UART communication
// Note: Arduino Uno doesn't have HardwareSerial ports like Serial3,
// so we use SoftwareSerial instead
SoftwareSerial serial_stream(SW_RX, SW_TX);

// Instantiate TMC2209
TMC2209 stepper_driver;

void setup()
{
  // Initialize hardware serial for debugging
  Serial.begin(115200);
  while (!Serial);
  Serial.println("TMC2209 Library Test");
  Serial.println("Wiring: EN=6, DIR=4, STEP=5, SW_RX=7, SW_TX=8");
  Serial.println("IMPORTANT: VIO to 5V, GND to GND, 1k resistor between pins 7 and 8 to PDN_UART");
  Serial.println();

  // Initialize SoftwareSerial for TMC2209
  serial_stream.begin(115200);

  // Setup TMC2209
  stepper_driver.setup(serial_stream);

  // Setup step and direction pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIRECTION_PIN, OUTPUT);

  // Setup enable pin (optional, but good practice)
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);  // Enable driver (active LOW)

  // Configure driver
  stepper_driver.setRunCurrent(RUN_CURRENT_PERCENT);
  stepper_driver.enableCoolStep();
  stepper_driver.enable();

  Serial.println("TMC2209 configured successfully!");
  Serial.println("Starting movement...");
  Serial.println();
}

void loop()
{
  // One step takes two iterations through the for loop
  for (uint32_t i=0; i<STEP_COUNT*2; ++i)
  {
    digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));
    delayMicroseconds(HALF_STEP_DURATION_MICROSECONDS);
  }

  // Reverse direction
  digitalWrite(DIRECTION_PIN, !digitalRead(DIRECTION_PIN));

  Serial.print("Direction changed. New direction: ");
  Serial.println(digitalRead(DIRECTION_PIN) ? "HIGH" : "LOW");

  // Pause before next movement
  delay(STOP_DURATION);
}
