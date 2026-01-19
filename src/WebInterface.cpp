/*
 * Web Interface Implementation
 */

#include "WebInterface.h"
#include "StateMachine.h"
#include "ESPNowComm.h"

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
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: #fff;
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
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
        }
        .status {
            background: rgba(255,255,255,0.1);
            border-radius: 15px;
            padding: 20px;
            margin-bottom: 20px;
            backdrop-filter: blur(10px);
        }
        .status h2 {
            margin-bottom: 10px;
            font-size: 1.2em;
        }
        .status-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 15px;
            margin-top: 15px;
        }
        .status-item {
            background: rgba(255,255,255,0.1);
            padding: 15px;
            border-radius: 10px;
        }
        .status-label {
            font-size: 0.9em;
            opacity: 0.8;
            margin-bottom: 5px;
        }
        .status-value {
            font-size: 1.5em;
            font-weight: bold;
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
            background: rgba(255,255,255,0.1);
            border-radius: 15px;
            padding: 25px;
            backdrop-filter: blur(10px);
        }
        .control-panel h2 {
            margin-bottom: 20px;
            font-size: 1.3em;
        }
        .joystick-container {
            position: relative;
            width: 250px;
            height: 250px;
            margin: 20px auto;
            background: rgba(0,0,0,0.2);
            border-radius: 50%;
            border: 3px solid rgba(255,255,255,0.3);
        }
        .joystick {
            position: absolute;
            width: 80px;
            height: 80px;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            border-radius: 50%;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            cursor: grab;
            box-shadow: 0 5px 15px rgba(0,0,0,0.3);
            border: 3px solid rgba(255,255,255,0.5);
        }
        .joystick:active {
            cursor: grabbing;
        }
        .button-grid {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 15px;
        }
        button {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            border: none;
            padding: 18px 25px;
            border-radius: 12px;
            font-size: 1.1em;
            cursor: pointer;
            transition: all 0.3s ease;
            box-shadow: 0 4px 6px rgba(0,0,0,0.2);
            font-weight: 600;
        }
        button:hover {
            transform: translateY(-2px);
            box-shadow: 0 6px 12px rgba(0,0,0,0.3);
        }
        button:active {
            transform: translateY(0);
        }
        button.emergency {
            background: linear-gradient(135deg, #f093fb 0%, #f5576c 100%);
            grid-column: span 2;
        }
        button.primary {
            background: linear-gradient(135deg, #4facfe 0%, #00f2fe 100%);
        }
        .marker-list {
            background: rgba(0,0,0,0.2);
            border-radius: 10px;
            padding: 15px;
            max-height: 200px;
            overflow-y: auto;
        }
        .marker-item {
            background: rgba(255,255,255,0.1);
            padding: 10px;
            margin-bottom: 8px;
            border-radius: 8px;
            display: flex;
            justify-content: space-between;
            align-items: center;
        }
        .connection-status {
            position: fixed;
            top: 20px;
            right: 20px;
            padding: 10px 20px;
            border-radius: 20px;
            font-size: 0.9em;
            font-weight: 600;
        }
        .connection-status.connected {
            background: #48bb78;
        }
        .connection-status.disconnected {
            background: #f56565;
        }
    </style>
</head>
<body>
    <div class="connection-status" id="connectionStatus">Connecting...</div>

    <div class="container">
        <h1>🎬 DSLR Motor Control</h1>

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
                <div class="joystick-container" id="joystickContainer">
                    <div class="joystick" id="joystick"></div>
                </div>
                <div style="text-align: center; opacity: 0.8; margin-top: 10px;">
                    Drag to move motor
                </div>
            </div>

            <div class="control-panel">
                <h2>Motor Controls</h2>
                <div class="button-grid">
                    <button onclick="sendCommand('home')">🏠 Home</button>
                    <button onclick="sendCommand('enable')">✓ Enable</button>
                    <button onclick="sendCommand('disable')">✗ Disable</button>
                    <button onclick="sendCommand('marker')" class="primary">📍 Add Marker</button>
                    <button onclick="sendCommand('playback')" class="primary">▶ Playback</button>
                    <button onclick="sendCommand('clearMarkers')">🗑 Clear Markers</button>
                    <button onclick="sendCommand('stop')" class="emergency">🛑 EMERGENCY STOP</button>
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

        // Joystick control
        const joystick = document.getElementById('joystick');
        const container = document.getElementById('joystickContainer');

        let isDragging = false;
        let startX, startY;

        function handleStart(e) {
            isDragging = true;
            joystickActive = true;
            const touch = e.touches ? e.touches[0] : e;
            startX = touch.clientX;
            startY = touch.clientY;

            // Start sending joystick updates
            joystickInterval = setInterval(sendJoystickPosition, 100);
        }

        function handleMove(e) {
            if(!isDragging) return;
            e.preventDefault();

            const touch = e.touches ? e.touches[0] : e;
            const rect = container.getBoundingClientRect();
            const centerX = rect.left + rect.width / 2;
            const centerY = rect.top + rect.height / 2;

            let deltaX = touch.clientX - centerX;
            let deltaY = touch.clientY - centerY;

            // Limit to circle radius
            const maxRadius = (rect.width / 2) - 40;
            const distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
            if(distance > maxRadius) {
                const angle = Math.atan2(deltaY, deltaX);
                deltaX = maxRadius * Math.cos(angle);
                deltaY = maxRadius * Math.sin(angle);
            }

            joystick.style.transform = `translate(calc(-50% + ${deltaX}px), calc(-50% + ${deltaY}px))`;
        }

        function handleEnd() {
            isDragging = false;
            joystickActive = false;
            joystick.style.transform = 'translate(-50%, -50%)';
            clearInterval(joystickInterval);

            // Send zero position
            if(ws && ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({cmd: 'joystick', value: 0}));
            }
        }

        function sendJoystickPosition() {
            if(!joystickActive) return;

            const transform = joystick.style.transform;
            const match = transform.match(/translate\(calc\(-50% \+ (-?\d+)px\)/);
            if(match) {
                const deltaX = parseInt(match[1]);
                // Convert to -512 to 512 range
                const value = Math.round((deltaX / 85) * 512);

                if(ws && ws.readyState === WebSocket.OPEN) {
                    ws.send(JSON.stringify({cmd: 'joystick', value: value}));
                }
            }
        }

        // Mouse events
        joystick.addEventListener('mousedown', handleStart);
        document.addEventListener('mousemove', handleMove);
        document.addEventListener('mouseup', handleEnd);

        // Touch events
        joystick.addEventListener('touchstart', handleStart);
        document.addEventListener('touchmove', handleMove, {passive: false});
        document.addEventListener('touchend', handleEnd);

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
  Serial.print(F("[WS] Message: "));
  Serial.println(message);

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
    // Send status update
    // TODO: Implement status retrieval from state machine
    return;
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
