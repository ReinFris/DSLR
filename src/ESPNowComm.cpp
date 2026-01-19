/*
 * ESP-NOW Communication Module Implementation
 */

#include "ESPNowComm.h"

// ============================================================================
// STATIC INSTANCE POINTERS
// ============================================================================

ESPNowReceiver *ESPNowReceiver::instance = nullptr;
ESPNowSender *ESPNowSender::instance = nullptr;

// ============================================================================
// ESP-NOW RECEIVER IMPLEMENTATION
// ============================================================================

ESPNowReceiver::ESPNowReceiver()
    : _newCommandAvailable(false), _userCallback(nullptr)
{
  // Clear last command
  memset(&_lastCommand, 0, sizeof(_lastCommand));

  // Set static instance pointer
  instance = this;
}

bool ESPNowReceiver::begin()
{
  Serial.println(F("[ESP-NOW] Initializing Receiver..."));

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Print MAC address
  Serial.print(F("[ESP-NOW] Receiver MAC: "));
  Serial.println(WiFi.macAddress());

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println(F("[ESP-NOW] Error initializing ESP-NOW"));
    return false;
  }

  // Register receive callback
  esp_now_register_recv_cb(esp_now_recv_cb_t(onDataRecvStatic));

  Serial.println(F("[ESP-NOW] Receiver ready"));
  return true;
}

void ESPNowReceiver::onCommandReceived(CommandReceivedCallback callback)
{
  _userCallback = callback;
}

String ESPNowReceiver::getMacAddress() const
{
  return WiFi.macAddress();
}

void ESPNowReceiver::onDataRecvStatic(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  if (instance != nullptr)
  {
    instance->onDataRecv(mac, incomingData, len);
  }
}

void ESPNowReceiver::onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  // Verify data length
  if (len != sizeof(struct_command))
  {
    Serial.print(F("[ESP-NOW] Invalid data length: "));
    Serial.println(len);
    return;
  }

  // Copy received data
  memcpy(&_lastCommand, incomingData, sizeof(_lastCommand));
  _newCommandAvailable = true;

  // Debug output
  Serial.print(F("[ESP-NOW] Cmd: "));
  Serial.print(_lastCommand.commandType);
  Serial.print(F(" Joy: "));
  Serial.print(_lastCommand.joystickValue);
  Serial.print(F(" Btn: "));
  Serial.println(_lastCommand.buttonEvent);

  // Call user callback if registered
  if (_userCallback != nullptr)
  {
    _userCallback(_lastCommand);
  }
}

// ============================================================================
// ESP-NOW SENDER IMPLEMENTATION
// ============================================================================

ESPNowSender::ESPNowSender()
    : _lastSendTime(0), _sendSuccessCount(0), _sendFailCount(0)
{
  // Clear receiver MAC
  memset(_receiverMac, 0, 6);

  // Set static instance pointer
  instance = this;
}

bool ESPNowSender::begin(const uint8_t *receiverMacAddress)
{
  Serial.println(F("[ESP-NOW] Initializing Sender..."));

  // Copy receiver MAC address
  memcpy(_receiverMac, receiverMacAddress, 6);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Print MAC address
  Serial.print(F("[ESP-NOW] Sender MAC: "));
  Serial.println(WiFi.macAddress());

  Serial.print(F("[ESP-NOW] Target MAC: "));
  for (int i = 0; i < 6; i++)
  {
    Serial.printf("%02X", _receiverMac[i]);
    if (i < 5)
      Serial.print(":");
  }
  Serial.println();

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println(F("[ESP-NOW] Error initializing ESP-NOW"));
    return false;
  }

  // Register send callback
  esp_now_register_send_cb(onDataSentStatic);

  // Register peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, _receiverMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println(F("[ESP-NOW] Failed to add peer"));
    return false;
  }

  Serial.println(F("[ESP-NOW] Sender ready"));
  return true;
}

bool ESPNowSender::sendCommand(const struct_command &cmd)
{
  esp_err_t result = esp_now_send(_receiverMac, (uint8_t *)&cmd, sizeof(cmd));

  if (result == ESP_OK)
  {
    _lastSendTime = millis();
    return true;
  }
  else
  {
    Serial.println(F("[ESP-NOW] Send error"));
    return false;
  }
}

bool ESPNowSender::sendJoystickMove(int16_t joystickValue)
{
  struct_command cmd;
  cmd.commandType = CMD_JOYSTICK_MOVE;
  cmd.buttonEvent = BTN_NONE;
  cmd.joystickValue = joystickValue;
  cmd.timestamp = millis();

  return sendCommand(cmd);
}

bool ESPNowSender::sendButtonEvent(ButtonEvent event)
{
  struct_command cmd;
  cmd.commandType = CMD_BUTTON_PRESS;
  cmd.buttonEvent = event;
  cmd.joystickValue = 0;
  cmd.timestamp = millis();

  return sendCommand(cmd);
}

bool ESPNowSender::sendMarkerCommand()
{
  struct_command cmd;
  cmd.commandType = CMD_SET_MARKER;
  cmd.buttonEvent = BTN_NONE;
  cmd.joystickValue = 0;
  cmd.timestamp = millis();

  return sendCommand(cmd);
}

bool ESPNowSender::sendPlaybackCommand()
{
  struct_command cmd;
  cmd.commandType = CMD_START_PLAYBACK;
  cmd.buttonEvent = BTN_NONE;
  cmd.joystickValue = 0;
  cmd.timestamp = millis();

  return sendCommand(cmd);
}

bool ESPNowSender::sendStopCommand()
{
  struct_command cmd;
  cmd.commandType = CMD_STOP;
  cmd.buttonEvent = BTN_NONE;
  cmd.joystickValue = 0;
  cmd.timestamp = millis();

  return sendCommand(cmd);
}

bool ESPNowSender::sendEnableMotor()
{
  struct_command cmd;
  cmd.commandType = CMD_ENABLE_MOTOR;
  cmd.buttonEvent = BTN_NONE;
  cmd.joystickValue = 0;
  cmd.timestamp = millis();

  return sendCommand(cmd);
}

bool ESPNowSender::sendDisableMotor()
{
  struct_command cmd;
  cmd.commandType = CMD_DISABLE_MOTOR;
  cmd.buttonEvent = BTN_NONE;
  cmd.joystickValue = 0;
  cmd.timestamp = millis();

  return sendCommand(cmd);
}

bool ESPNowSender::sendHomeCommand()
{
  struct_command cmd;
  cmd.commandType = CMD_HOME;
  cmd.buttonEvent = BTN_NONE;
  cmd.joystickValue = 0;
  cmd.timestamp = millis();

  return sendCommand(cmd);
}

String ESPNowSender::getMacAddress() const
{
  return WiFi.macAddress();
}

void ESPNowSender::onDataSentStatic(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  if (instance != nullptr)
  {
    instance->onDataSent(mac_addr, status);
  }
}

void ESPNowSender::onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  if (status == ESP_NOW_SEND_SUCCESS)
  {
    _sendSuccessCount++;
  }
  else
  {
    _sendFailCount++;
    Serial.println(F("[ESP-NOW] Send failed"));
  }
}
