/*
 * ESP-NOW Communication Module
 *
 * Handles wireless communication between two ESP32s:
 * - Sender: Remote control (joystick, buttons)
 * - Receiver: Motor control system (this device)
 *
 * MAC Addresses:
 * - Receiver (Motor Control): 48:E7:29:B6:70:9C
 * - Sender (Remote): 00:70:07:1d:06:f4
 */

#ifndef ESPNOW_COMM_H
#define ESPNOW_COMM_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// ============================================================================
// COMMAND STRUCTURE
// ============================================================================

// Command types for motor control
enum CommandType
{
  CMD_NONE = 0,
  CMD_JOYSTICK_MOVE,      // Joystick movement command
  CMD_BUTTON_PRESS,       // Button press event
  CMD_SET_MARKER,         // Set marker at current position
  CMD_START_PLAYBACK,     // Start playback sequence
  CMD_STOP,               // Emergency stop
  CMD_ENABLE_MOTOR,       // Enable motor
  CMD_DISABLE_MOTOR,      // Disable motor for manual positioning
  CMD_HOME,               // Trigger homing sequence
  CMD_ENABLE_LOGGING,     // Enable serial logging
  CMD_DISABLE_LOGGING     // Disable serial logging
};

// Button event types
enum ButtonEvent
{
  BTN_NONE = 0,
  BTN_SHORT_PRESS,
  BTN_LONG_PRESS,
  BTN_DOUBLE_PRESS
};

// Data structure for commands - must match sender structure exactly
typedef struct struct_command
{
  CommandType commandType;
  ButtonEvent buttonEvent;
  int16_t joystickValue;  // -512 to +512 range (centered at 0)
  uint32_t timestamp;      // For debugging/latency tracking
} struct_command;

// ============================================================================
// CALLBACK FUNCTION TYPES
// ============================================================================

// Callback for when a command is received
typedef void (*CommandReceivedCallback)(const struct_command &cmd);

// ============================================================================
// ESP-NOW RECEIVER CLASS
// ============================================================================

class ESPNowReceiver
{
public:
  // Constructor
  ESPNowReceiver();

  // Initialization
  bool begin();

  // Register callback for received commands
  void onCommandReceived(CommandReceivedCallback callback);

  // Get last received command
  const struct_command &getLastCommand() const { return _lastCommand; }

  // Check if new command is available
  bool hasNewCommand() const { return _newCommandAvailable; }

  // Clear the new command flag (after processing)
  void clearNewCommand() { _newCommandAvailable = false; }

  // Get MAC address of this device
  String getMacAddress() const;

  // Static callback wrapper (ESP-NOW requires static function)
  static void onDataRecvStatic(const uint8_t *mac, const uint8_t *incomingData, int len);

  // Static instance pointer for callback
  static ESPNowReceiver *instance;

private:
  // Last received command
  struct_command _lastCommand;

  // New command flag
  volatile bool _newCommandAvailable;

  // User callback
  CommandReceivedCallback _userCallback;

  // Instance callback (non-static)
  void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
};

// ============================================================================
// ESP-NOW SENDER CLASS
// ============================================================================

class ESPNowSender
{
public:
  // Constructor
  ESPNowSender();

  // Initialization
  bool begin(const uint8_t *receiverMacAddress);

  // Send command to receiver
  bool sendCommand(const struct_command &cmd);

  // Convenience functions for sending specific commands
  bool sendJoystickMove(int16_t joystickValue);
  bool sendButtonEvent(ButtonEvent event);
  bool sendMarkerCommand();
  bool sendPlaybackCommand();
  bool sendStopCommand();
  bool sendEnableMotor();
  bool sendDisableMotor();
  bool sendHomeCommand();

  // Get MAC address of this device
  String getMacAddress() const;

  // Static callback for send status
  static void onDataSentStatic(const uint8_t *mac_addr, esp_now_send_status_t status);

  // Static instance pointer for callback
  static ESPNowSender *instance;

private:
  // Receiver MAC address
  uint8_t _receiverMac[6];

  // Send statistics
  unsigned long _lastSendTime;
  uint32_t _sendSuccessCount;
  uint32_t _sendFailCount;

  // Instance callback (non-static)
  void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
};

#endif // ESPNOW_COMM_H
