/*
 * Web Interface Implementation
 */

#include "WebInterface.h"
#include "StateMachine.h"
#include "ESPNowComm.h"
#include "HardwareConfig.h"

// Static instance pointer
WebInterface *WebInterface::instance = nullptr;

// HTML page with embedded CSS and JavaScript
const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>DSLR Motor Control</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: #7C98A0;
            color: #4A3329;
            padding: 20px;
            min-height: 100vh;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
        }
        h1 {
            text-align: center;
            margin-bottom: 30px;
            font-size: 2.5em;
            color: #4A3329;
            font-weight: 300;
            letter-spacing: 2px;
        }
        .status {
            background: #B2C7C4;
            border-radius: 8px;
            padding: 20px;
            margin-bottom: 20px;
            border: 2px solid #8C9C7B;
        }
        .status h2 {
            margin-bottom: 10px;
            font-size: 1.2em;
            color: #4A3329;
            font-weight: 400;
        }
        .status-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 15px;
            margin-top: 15px;
        }
        .status-item {
            background: #7C98A0;
            padding: 15px;
            border-radius: 6px;
        }
        .status-label {
            font-size: 0.9em;
            color: #B2C7C4;
            margin-bottom: 5px;
            text-transform: uppercase;
            letter-spacing: 1px;
        }
        .status-value {
            font-size: 1.5em;
            font-weight: 600;
            color: #ffffff;
        }
        .controls {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 20px;
            margin-bottom: 20px;
        }
        @media (max-width: 768px) {
            .controls {
                grid-template-columns: 1fr;
            }
        }
        .control-panel {
            background: #B2C7C4;
            border-radius: 8px;
            padding: 25px;
            border: 2px solid #8C9C7B;
        }
        .control-panel h2 {
            margin-bottom: 20px;
            font-size: 1.3em;
            color: #4A3329;
            font-weight: 400;
        }
        .direction-btn {
            background: #D4876B;
            user-select: none;
            -webkit-user-select: none;
            touch-action: manipulation;
        }
        .direction-btn:active {
            background: #b36c53;
            transform: scale(0.95);
        }
        .button-grid {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 15px;
        }
        button {
            background: #8C9C7B;
            color: #ffffff;
            border: none;
            padding: 18px 25px;
            border-radius: 6px;
            font-size: 1.1em;
            cursor: pointer;
            transition: all 0.2s ease;
            font-weight: 500;
        }
        button:hover {
            background: #6f7d61;
            transform: translateY(-1px);
        }
        button:active {
            transform: translateY(0);
        }
        button.emergency {
            background: #D4876B;
            grid-column: span 2;
        }
        button.emergency:hover {
            background: #b36c53;
        }
        button.primary {
            background: #7C98A0;
        }
        button.primary:hover {
            background: #617780;
        }
        .marker-list {
            background: #7C98A0;
            border-radius: 6px;
            padding: 15px;
            max-height: 200px;
            overflow-y: auto;
        }
        .marker-item {
            background: #8C9C7B;
            padding: 10px;
            margin-bottom: 8px;
            border-radius: 4px;
            display: flex;
            justify-content: space-between;
            align-items: center;
            color: #ffffff;
        }
        .connection-status {
            position: fixed;
            top: 20px;
            right: 20px;
            padding: 10px 20px;
            border-radius: 6px;
            font-size: 0.9em;
            font-weight: 500;
        }
        .connection-status.connected {
            background: #8C9C7B;
            color: #ffffff;
        }
        .connection-status.disconnected {
            background: #D4876B;
            color: #ffffff;
        }
    </style>
</head>
<body>
    <div class="connection-status" id="connectionStatus">Connecting...</div>

    <div class="container">
        <h1>DSLR MOTOR CONTROL</h1>

        <div class="status">
            <h2>System Status</h2>
            <div class="status-grid">
                <div class="status-item">
                    <div class="status-label">State</div>
                    <div class="status-value" id="systemState">--</div>
                </div>
                <div class="status-item">
                    <div class="status-label">Position</div>
                    <div class="status-value" id="position">--</div>
                </div>
                <div class="status-item">
                    <div class="status-label">Markers</div>
                    <div class="status-value" id="markerCount">0</div>
                </div>
                <div class="status-item">
                    <div class="status-label">Motor</div>
                    <div class="status-value" id="motorStatus">--</div>
                </div>
            </div>
        </div>

        <div class="controls">
            <div class="control-panel">
                <h2>Movement Control</h2>
                <div style="display: flex; gap: 20px; justify-content: center; margin: 20px 0;">
                    <button id="leftBtn" class="direction-btn" style="flex: 1; max-width: 200px; min-height: 120px; font-size: 48px;">
                        ◄
                    </button>
                    <button id="rightBtn" class="direction-btn" style="flex: 1; max-width: 200px; min-height: 120px; font-size: 48px;">
                        ►
                    </button>
                </div>
                <div style="text-align: center; opacity: 0.8; margin-top: 10px;">
                    Hold button to move motor
                </div>
            </div>

            <div class="control-panel">
                <h2>Motor Controls</h2>
                <div class="button-grid">
                    <button onclick="sendCommand('home')">Home</button>
                    <button onclick="sendCommand('enable')">Enable</button>
                    <button onclick="sendCommand('disable')">Disable</button>
                    <button onclick="sendCommand('marker')" class="primary">Add Marker</button>
                    <button onclick="sendCommand('playback')" class="primary">Playback</button>
                    <button onclick="sendCommand('clearMarkers')">Clear Markers</button>
                    <button onclick="sendCommand('stop')" class="emergency">EMERGENCY STOP</button>
                </div>
                <div style="margin-top: 20px; padding-top: 20px; border-top: 1px solid rgba(74, 51, 41, 0.2);">
                    <label style="display: flex; align-items: center; justify-content: space-between; cursor: pointer;">
                        <span style="font-size: 0.95em;">Enable Logging</span>
                        <input type="checkbox" id="loggingToggle" onchange="toggleLogging(this.checked)" checked style="width: 20px; height: 20px; cursor: pointer;">
                    </label>
                    <div style="font-size: 0.85em; opacity: 0.7; margin-top: 8px;">
                        Disable to improve motion smoothness
                    </div>
                </div>
            </div>
        </div>

        <div class="control-panel">
            <h2>Position Markers</h2>
            <div class="marker-list" id="markerList">
                <div style="text-align: center; opacity: 0.6;">No markers set</div>
            </div>
        </div>
    </div>

    <script>
        let ws;
        let joystickActive = false;
        let joystickInterval;

        // WebSocket connection
        function connectWebSocket() {
            ws = new WebSocket('ws://' + window.location.hostname + ':81');

            ws.onopen = function() {
                console.log('WebSocket connected');
                document.getElementById('connectionStatus').textContent = 'Connected';
                document.getElementById('connectionStatus').className = 'connection-status connected';
            };

            ws.onclose = function() {
                console.log('WebSocket disconnected');
                document.getElementById('connectionStatus').textContent = 'Disconnected';
                document.getElementById('connectionStatus').className = 'connection-status disconnected';
                setTimeout(connectWebSocket, 3000);
            };

            ws.onmessage = function(event) {
                try {
                    const data = JSON.parse(event.data);
                    updateStatus(data);
                } catch(e) {
                    console.log('Status:', event.data);
                }
            };

            ws.onerror = function(error) {
                console.error('WebSocket error:', error);
            };
        }

        // Update status display
        function updateStatus(data) {
            if(data.state) document.getElementById('systemState').textContent = data.state;
            if(data.position !== undefined) document.getElementById('position').textContent = data.position;
            if(data.markerCount !== undefined) document.getElementById('markerCount').textContent = data.markerCount;
            if(data.motorEnabled !== undefined) {
                document.getElementById('motorStatus').textContent = data.motorEnabled ? 'Enabled' : 'Disabled';
            }
        }

        // Send command via WebSocket
        function sendCommand(cmd) {
            if(ws && ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({cmd: cmd}));
            }
        }

        // Toggle logging on/off
        function toggleLogging(enabled) {
            const cmd = enabled ? 'enableLogging' : 'disableLogging';
            sendCommand(cmd);
        }

        // Direction button control
        const leftBtn = document.getElementById('leftBtn');
        const rightBtn = document.getElementById('rightBtn');
        let moveInterval = null;
        let currentDirection = 0;

        function startMove(direction) {
            currentDirection = direction;
            sendMove();
            // Send continuous updates while button is held
            moveInterval = setInterval(sendMove, 100);
        }

        function stopMove() {
            currentDirection = 0;
            clearInterval(moveInterval);
            moveInterval = null;
            sendMove();
        }

        function sendMove() {
            if(ws && ws.readyState === WebSocket.OPEN) {
                // Send full speed in direction: -512 for left, +512 for right
                const value = currentDirection * 512;
                ws.send(JSON.stringify({cmd: 'joystick', value: value}));
            }
        }

        // Mouse events for left button
        leftBtn.addEventListener('mousedown', () => startMove(-1));
        leftBtn.addEventListener('mouseup', stopMove);
        leftBtn.addEventListener('mouseleave', stopMove);

        // Touch events for left button
        leftBtn.addEventListener('touchstart', (e) => { e.preventDefault(); startMove(-1); });
        leftBtn.addEventListener('touchend', stopMove);
        leftBtn.addEventListener('touchcancel', stopMove);

        // Mouse events for right button
        rightBtn.addEventListener('mousedown', () => startMove(1));
        rightBtn.addEventListener('mouseup', stopMove);
        rightBtn.addEventListener('mouseleave', stopMove);

        // Touch events for right button
        rightBtn.addEventListener('touchstart', (e) => { e.preventDefault(); startMove(1); });
        rightBtn.addEventListener('touchend', stopMove);
        rightBtn.addEventListener('touchcancel', stopMove);

        // Initialize WebSocket
        connectWebSocket();

        // Request status update every second
        setInterval(() => {
            if(ws && ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({cmd: 'getStatus'}));
            }
        }, 1000);
    </script>
</body>
</html>
)rawliteral";

// ============================================================================
// CONSTRUCTOR
// ============================================================================

WebInterface::WebInterface(StateMachine &stateMachine)
    : _stateMachine(stateMachine), _server(80), _webSocket(81)
{
  instance = this;
}

// ============================================================================
// INITIALIZATION
// ============================================================================

bool WebInterface::begin(const char *ssid, const char *password, bool accessPointMode)
{
  Serial.println(F("\n[WEB] Initializing WiFi..."));

  if (accessPointMode)
  {
    // Create Access Point
    WiFi.softAP(ssid, password);
    Serial.println(F("[WEB] Access Point created"));
    Serial.print(F("[WEB] SSID: "));
    Serial.println(ssid);
    Serial.print(F("[WEB] Password: "));
    Serial.println(password);
    Serial.print(F("[WEB] IP Address: "));
    Serial.println(WiFi.softAPIP());
  }
  else
  {
    // Connect to existing WiFi
    WiFi.begin(ssid, password);
    Serial.print(F("[WEB] Connecting to WiFi"));

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20)
    {
      delay(500);
      Serial.print(".");
      attempts++;
    }

    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.println(F("\n[WEB] Failed to connect to WiFi"));
      return false;
    }

    Serial.println(F("\n[WEB] Connected to WiFi"));
    Serial.print(F("[WEB] IP Address: "));
    Serial.println(WiFi.localIP());
  }

  // Setup web server routes
  _server.on("/", [this]()
             { handleRoot(); });
  _server.onNotFound([this]()
                     { handleNotFound(); });

  // Start web server
  _server.begin();
  Serial.println(F("[WEB] HTTP server started on port 80"));

  // Start WebSocket server
  _webSocket.begin();
  _webSocket.onEvent(webSocketEvent);
  Serial.println(F("[WEB] WebSocket server started on port 81"));

  return true;
}

// ============================================================================
// UPDATE
// ============================================================================

void WebInterface::update()
{
  _server.handleClient();
  _webSocket.loop();
}

// ============================================================================
// HTTP HANDLERS
// ============================================================================

void WebInterface::handleRoot()
{
  _server.send_P(200, "text/html", HTML_PAGE);
}

void WebInterface::handleNotFound()
{
  _server.send(404, "text/plain", "404: Not Found");
}

// ============================================================================
// WEBSOCKET HANDLERS
// ============================================================================

void WebInterface::webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
  if (instance != nullptr)
  {
    switch (type)
    {
    case WStype_DISCONNECTED:
      Serial.printf("[WS] Client #%u disconnected\n", num);
      break;

    case WStype_CONNECTED:
    {
      IPAddress ip = instance->_webSocket.remoteIP(num);
      Serial.printf("[WS] Client #%u connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
    }
    break;

    case WStype_TEXT:
      instance->handleWebSocketMessage(num, payload, length);
      break;

    default:
      break;
    }
  }
}

void WebInterface::handleWebSocketMessage(uint8_t clientNum, uint8_t *payload, size_t length)
{
  // Parse JSON command
  String message = String((char *)payload);
  if (enableLogging)
  {
    Serial.print(F("[WS] Message: "));
    Serial.println(message);
  }

  // Simple JSON parsing (looking for "cmd" field)
  int cmdStart = message.indexOf("\"cmd\":\"") + 7;
  int cmdEnd = message.indexOf("\"", cmdStart);
  String cmd = message.substring(cmdStart, cmdEnd);

  struct_command espCmd;
  espCmd.commandType = CMD_NONE;
  espCmd.buttonEvent = BTN_NONE;
  espCmd.joystickValue = 0;
  espCmd.timestamp = millis();

  // Process command
  if (cmd == "home")
  {
    espCmd.commandType = CMD_HOME;
  }
  else if (cmd == "enable")
  {
    espCmd.commandType = CMD_ENABLE_MOTOR;
  }
  else if (cmd == "disable")
  {
    espCmd.commandType = CMD_DISABLE_MOTOR;
  }
  else if (cmd == "marker")
  {
    espCmd.commandType = CMD_SET_MARKER;
  }
  else if (cmd == "playback")
  {
    espCmd.commandType = CMD_START_PLAYBACK;
  }
  else if (cmd == "stop")
  {
    espCmd.commandType = CMD_STOP;
  }
  else if (cmd == "joystick")
  {
    // Extract value
    int valueStart = message.indexOf("\"value\":") + 8;
    int valueEnd = message.indexOf(",", valueStart);
    if (valueEnd == -1)
      valueEnd = message.indexOf("}", valueStart);

    int value = message.substring(valueStart, valueEnd).toInt();
    espCmd.commandType = CMD_JOYSTICK_MOVE;
    espCmd.joystickValue = constrain(value, -512, 512);
  }
  else if (cmd == "getStatus")
  {
    sendStatusUpdate(clientNum);
    return;
  }
  else if (cmd == "enableLogging")
  {
    espCmd.commandType = CMD_ENABLE_LOGGING;
  }
  else if (cmd == "disableLogging")
  {
    espCmd.commandType = CMD_DISABLE_LOGGING;
  }

  // Send command to state machine
  if (espCmd.commandType != CMD_NONE)
  {
    _stateMachine.processWirelessCommand(espCmd);
  }
}

// ============================================================================
// UTILITY
// ============================================================================

String WebInterface::getIPAddress() const
{
  if (WiFi.getMode() == WIFI_AP)
  {
    return WiFi.softAPIP().toString();
  }
  else
  {
    return WiFi.localIP().toString();
  }
}

void WebInterface::broadcastStatus(const String &status)
{
  String msg = status; // Create non-const copy
  _webSocket.broadcastTXT(msg);
}

void WebInterface::sendStatusUpdate(uint8_t clientNum)
{
  // Get current state
  SystemState state = _stateMachine.getCurrentState();

  // Convert state enum to string
  String stateStr;
  switch(state) {
    case STATE_STARTUP: stateStr = "STARTUP"; break;
    case STATE_READY: stateStr = "READY"; break;
    case STATE_DISABLED: stateStr = "DISABLED"; break;
    case STATE_PLAYBACK_DELAY: stateStr = "PLAYBACK_DELAY"; break;
    case STATE_PLAYBACK: stateStr = "PLAYBACK"; break;
    default: stateStr = "UNKNOWN"; break;
  }

  // Get other status info
  long position = _stateMachine.getCurrentPosition();
  uint8_t markerCount = _stateMachine.getMarkerCount();
  bool motorEnabled = (digitalRead(ENABLE_PIN) == LOW);

  // Build JSON response
  String json = "{";
  json += "\"state\":\"" + stateStr + "\",";
  json += "\"position\":" + String(position) + ",";
  json += "\"markerCount\":" + String(markerCount) + ",";
  json += "\"motorEnabled\":" + String(motorEnabled ? "true" : "false");
  json += "}";

  // Send to client
  _webSocket.sendTXT(clientNum, json);
}
