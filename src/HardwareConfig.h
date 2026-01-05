/*
 * Hardware Configuration for ESP32 DevKit V1 + TMC2209 + AS5600
 *
 * Contains all pin definitions, motor parameters, and hardware constants.
 * See main.cpp header comment for detailed wiring instructions.
 */

#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include <Arduino.h>

// ============================================================================
// PIN ASSIGNMENTS FOR ESP32 DevKit V1
// ============================================================================

// Stepper motor control pins
#define STEP_PIN 26      // GPIO26 - Step pulse for TMC2209 (PWM capable)
#define DIR_PIN 25       // GPIO25 - Direction control for TMC2209 (PWM capable)
#define ENABLE_PIN 33    // GPIO33 - Enable pin (LOW = enabled, HIGH = disabled)
#define DIAG_PIN 35      // GPIO35 - StallGuard DIAG (input only, interrupt capable)

// TMC2209 UART pins (Hardware Serial2)
#define TMC_UART_RX 16   // GPIO16 - Serial2 RX via 1kΩ resistor to PDN_UART
#define TMC_UART_TX 17   // GPIO17 - Serial2 TX directly to PDN_UART

// Status LED
#define LED_PIN 2        // GPIO2 - Built-in LED on ESP32 DevKit V1

// I2C pins for AS5600 encoder
#define I2C_SDA_PIN 21   // GPIO21 - I2C SDA (default on ESP32)
#define I2C_SCL_PIN 22   // GPIO22 - I2C SCL (default on ESP32)

// Joystick pins
#define JOYSTICK_VRX 34  // GPIO34 - Joystick X-axis (ADC1_CH6, input only, 12-bit)
#define JOYSTICK_SW 32   // GPIO32 - Joystick button (with internal pullup)

// ============================================================================
// MOTOR & TMC2209 CONFIGURATION
// ============================================================================

// Motor parameters
#define STEPS_PER_REV 200 // Standard stepper motor (1.8° per step)
#define MICROSTEPS 2      // Microsteps setting for TMC2209
#define STEP_DELAY 100    // Microseconds between steps (for homing)

// TMC2209 Configuration
#define R_SENSE 0.11f       // SilentStepStick series use 0.11 Ohm
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address (MS1 and MS2 to GND)
#define RUN_CURRENT 500     // Motor current in mA (adjust for your motor)
#define STALL_VALUE 24      // StallGuard threshold [0..255] (lower = more sensitive)
#define TOFF_VALUE 4        // Off time setting [1..15]

// AccelStepper Configuration
#define MAX_SPEED 9000.0                         // Maximum speed in steps/second
#define ACCELERATION 10000.0                     // Acceleration in steps/second^2
#define DEFAULT_SPEED 1000.0                     // Default speed for moves
#define TOTAL_STEPS (STEPS_PER_REV * MICROSTEPS) // Total steps per revolution

// Movement parameters
#define HOMING_BACKOFF_STEPS 500 // Steps to back off after hitting stall
#define SAFETY_BUFFER_STEPS 50   // Safety buffer from hard limits (microsteps)

// Joystick parameters (ESP32 12-bit ADC: 0-4095)
#define JOYSTICK_CENTER 2048        // Center position (12-bit ADC midpoint)
#define JOYSTICK_DEADZONE 200       // Deadzone around center (scaled 4x for 12-bit)
#define JOYSTICK_HALF_THRESHOLD 600 // Threshold for half vs full speed (scaled 4x)
#define JOYSTICK_MIN 0              // Minimum ADC value
#define JOYSTICK_MAX 3200           // Maximum ADC value (scaled 4x, may need calibration)

#endif // HARDWARE_CONFIG_H
