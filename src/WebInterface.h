/*
 * Web Interface Module
 *
 * Provides a web-based GUI for controlling the DSLR motor system
 * from a laptop browser over WiFi.
 *
 * Features:
 * - Virtual joystick for motor control
 * - Position marker management
 * - Playback controls
 * - Real-time status display
 * - WebSocket for low-latency updates
 */

#ifndef WEB_INTERFACE_H
#define WEB_INTERFACE_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

// Forward declaration
class StateMachine;

// ============================================================================
// WEB INTERFACE CLASS
// ============================================================================

class WebInterface
{
public:
  // Constructor
  WebInterface(StateMachine &stateMachine);

  // Initialization
  bool begin(const char *ssid, const char *password, bool accessPointMode = true);

  // Update function (call in loop())
  void update();

  // Get IP address
  String getIPAddress() const;

  // WebSocket message handler
  void handleWebSocketMessage(uint8_t clientNum, uint8_t *payload, size_t length);

  // Send status update to all connected clients
  void broadcastStatus(const String &status);

private:
  // Hardware reference
  StateMachine &_stateMachine;

  // Web server
  WebServer _server;

  // WebSocket server
  WebSocketsServer _webSocket;

  // HTTP request handlers
  void handleRoot();
  void handleNotFound();

  // WebSocket event handler (static wrapper)
  static void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);

  // Static instance pointer for callbacks
  static WebInterface *instance;
};

#endif // WEB_INTERFACE_H
