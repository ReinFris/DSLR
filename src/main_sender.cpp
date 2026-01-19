/*
 * DSLR Motor Controller - Wireless Remote (Sender)
 *
 * ESP32 Remote Control Unit for DSLR Motor System
 *
 * Hardware:
 * - ESP32 DevKit V1 (Sender MAC: 00:70:07:1d:06:f4)
 * - Joystick on GPIO 34 (VRX) and GPIO 32 (button)
 * - LED on GPIO 2 for status indication
 *
 * Target:
 * - Receiver ESP32 MAC: 48:E7:29:B6:70:9C
 */

#include <Arduino.h>
#include "ESPNowComm.h"

// ============================================================================
// PIN DEFINITIONS
// ============================================================================

#define JOYSTICK_VRX 34 // Joystick X-axis (ADC1_CH6)
#define JOYSTICK_SW 32  // Joystick button
#define LED_PIN 2       // Built-in LED

// ============================================================================
// CONFIGURATION
// ============================================================================

// Receiver MAC Address: 48:E7:29:B6:70:9C
uint8_t receiverAddress[] = {0x48, 0xE7, 0x29, 0xB6, 0x70, 0x9C};

// Joystick calibration (ESP32 uses 12-bit ADC: 0-4095)
const int JOYSTICK_CENTER = 2048;
const int JOYSTICK_DEADZONE = 200;
const int JOYSTICK_MAX_OFFSET = 512; // Maximum value to send: -512 to +512

// Button timing
const unsigned long DEBOUNCE_DELAY = 50;
const unsigned long LONG_PRESS_DURATION = 1000;
const unsigned long DOUBLE_PRESS_INTERVAL = 400;

// Send interval for joystick updates
const unsigned long JOYSTICK_SEND_INTERVAL = 100; // 10Hz update rate

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

ESPNowSender espNowSender;

// ============================================================================
// BUTTON STATE
// ============================================================================

struct ButtonState
{
  bool currentState;
  bool lastState;
  unsigned long pressStartTime;
  unsigned long lastReleaseTime;
  bool longPressDetected;
  int pressCount;
} buttonState = {HIGH, HIGH, 0, 0, false, 0};

// ============================================================================
// JOYSTICK STATE
// ============================================================================

unsigned long lastJoystickSendTime = 0;
int16_t lastJoystickValue = 0;

// ============================================================================
// BUTTON HANDLING
// ============================================================================

ButtonEvent updateButton()
{
  ButtonEvent event = BTN_NONE;
  bool reading = digitalRead(JOYSTICK_SW);
  unsigned long currentTime = millis();

  // Detect button press (transition from HIGH to LOW)
  if (reading == LOW && buttonState.lastState == HIGH)
  {
    buttonState.pressStartTime = currentTime;
    buttonState.longPressDetected = false;
  }

  // Detect button release (transition from LOW to HIGH)
  if (reading == HIGH && buttonState.lastState == LOW)
  {
    unsigned long pressDuration = currentTime - buttonState.pressStartTime;

    if (!buttonState.longPressDetected)
    {
      if (pressDuration >= LONG_PRESS_DURATION)
      {
        // Long press detected
        event = BTN_LONG_PRESS;
        Serial.println(F("[BTN] Long press"));
      }
      else if (pressDuration >= DEBOUNCE_DELAY)
      {
        // Valid short press
        if ((currentTime - buttonState.lastReleaseTime) < DOUBLE_PRESS_INTERVAL)
        {
          // Double press detected
          event = BTN_DOUBLE_PRESS;
          buttonState.pressCount = 0; // Reset counter
          Serial.println(F("[BTN] Double press"));
        }
        else
        {
          // First press, wait to see if there's a second
          buttonState.pressCount = 1;
          buttonState.lastReleaseTime = currentTime;
        }
      }
    }
    buttonState.longPressDetected = false;
  }

  // Check if holding for long press
  if (reading == LOW && !buttonState.longPressDetected)
  {
    unsigned long pressDuration = currentTime - buttonState.pressStartTime;
    if (pressDuration >= LONG_PRESS_DURATION)
    {
      event = BTN_LONG_PRESS;
      buttonState.longPressDetected = true;
      Serial.println(F("[BTN] Long press (hold)"));
    }
  }

  // Check for single press timeout
  if (buttonState.pressCount == 1 &&
      (currentTime - buttonState.lastReleaseTime) >= DOUBLE_PRESS_INTERVAL)
  {
    event = BTN_SHORT_PRESS;
    buttonState.pressCount = 0;
    Serial.println(F("[BTN] Short press"));
  }

  buttonState.lastState = reading;
  return event;
}

// ============================================================================
// JOYSTICK HANDLING
// ============================================================================

int16_t readJoystick()
{
  int rawValue = analogRead(JOYSTICK_VRX);

  // Center and apply deadzone
  int offset = rawValue - JOYSTICK_CENTER;

  if (abs(offset) < JOYSTICK_DEADZONE)
  {
    return 0; // Within deadzone
  }

  // Scale to -512 to +512 range
  int16_t scaledValue = constrain(offset, -JOYSTICK_MAX_OFFSET, JOYSTICK_MAX_OFFSET);

  return scaledValue;
}

// ============================================================================
// SETUP
// ============================================================================

void setup()
{
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000);

  Serial.println(F("\n\n================================="));
  Serial.println(F("DSLR Motor Remote Control"));
  Serial.println(F("ESP-NOW Sender"));
  Serial.println(F("=================================\n"));

  // Initialize pins
  pinMode(JOYSTICK_VRX, INPUT);
  pinMode(JOYSTICK_SW, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  Serial.println(F("[HW] Pins configured"));

  // Blink LED 3 times on startup
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }

  // Initialize ESP-NOW sender
  if (espNowSender.begin(receiverAddress))
  {
    Serial.println(F("[ESP-NOW] Sender initialized successfully"));
    Serial.print(F("[ESP-NOW] This device MAC: "));
    Serial.println(espNowSender.getMacAddress());

    // LED on to indicate ready
    digitalWrite(LED_PIN, HIGH);
  }
  else
  {
    Serial.println(F("[ESP-NOW] Failed to initialize sender!"));
    // Fast blink to indicate error
    while (true)
    {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }

  Serial.println(F("\n--- Ready ---"));
  Serial.println(F("Controls:"));
  Serial.println(F("  Joystick: Move motor"));
  Serial.println(F("  Short press: Set marker (when disabled)"));
  Serial.println(F("  Long press: Toggle motor disable/playback"));
  Serial.println(F("  Double press: Emergency stop"));
  Serial.println();
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop()
{
  unsigned long currentTime = millis();

  // Update button and check for events
  ButtonEvent buttonEvent = updateButton();

  // Send button events
  if (buttonEvent != BTN_NONE)
  {
    if (espNowSender.sendButtonEvent(buttonEvent))
    {
      // Brief LED blink on successful send
      digitalWrite(LED_PIN, LOW);
      delay(50);
      digitalWrite(LED_PIN, HIGH);
    }
  }

  // Read and send joystick position at regular intervals
  if (currentTime - lastJoystickSendTime >= JOYSTICK_SEND_INTERVAL)
  {
    int16_t joystickValue = readJoystick();

    // Only send if value changed or is non-zero
    if (joystickValue != lastJoystickValue || joystickValue != 0)
    {
      espNowSender.sendJoystickMove(joystickValue);
      lastJoystickValue = joystickValue;

      // Debug output for significant movements
      if (abs(joystickValue) > 100)
      {
        Serial.print(F("[JOY] "));
        Serial.println(joystickValue);
      }
    }

    lastJoystickSendTime = currentTime;
  }

  // Small delay to prevent excessive loop rate
  delay(10);
}
