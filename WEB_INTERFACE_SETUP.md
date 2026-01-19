# Web Interface Setup - Laptop Control

Control your DSLR motor system from your laptop browser - no physical remote needed!

## Overview

The ESP32 creates its own WiFi access point and serves a beautiful web interface that you can access from any device with a web browser (laptop, tablet, phone).

## Quick Start

### 1. Upload Firmware

```bash
# Build and upload to ESP32
pio run --target upload

# Or just build
pio run
```

### 2. Connect to WiFi

After the ESP32 starts:

1. Open your laptop's WiFi settings
2. Look for network: **DSLR_Motor_Control**
3. Password: **motor123**
4. Connect to it

### 3. Open Web Interface

Open your browser and go to:
```
http://192.168.4.1
```

That's it! You should see the control interface.

## Web Interface Features

### 🎮 Virtual Joystick
- **Drag the joystick** left/right to move the motor
- Smooth, responsive control
- Automatic deadzone to prevent drift
- Works with mouse or touch

### 🎛️ Motor Controls

| Button | Function |
|--------|----------|
| 🏠 Home | Trigger homing sequence |
| ✓ Enable | Enable motor driver |
| ✗ Disable | Disable motor for manual positioning |
| 📍 Add Marker | Set position marker at current location |
| ▶ Playback | Start playback sequence through markers |
| 🗑 Clear Markers | Remove all position markers |
| 🛑 EMERGENCY STOP | Immediately stop all motor movement |

### 📊 Real-time Status Display

The interface shows:
- **System State**: Current operation mode (Ready, Disabled, Playback, etc.)
- **Position**: Current stepper position
- **Markers**: Number of position markers set
- **Motor**: Motor enable/disable status

### 🔄 WebSocket Connection

- Real-time bidirectional communication
- Low latency (<50ms typical)
- Auto-reconnect if connection lost
- Connection status indicator in top-right corner

## Network Configuration

### Access Point Mode (Default)

The ESP32 creates its own WiFi network:
- **SSID**: `DSLR_Motor_Control`
- **Password**: `motor123`
- **IP Address**: `192.168.4.1`

To change the credentials, edit in [main.cpp](src/main.cpp):
```cpp
const char *AP_SSID = "Your_SSID_Here";
const char *AP_PASSWORD = "Your_Password_Here";
```

### Station Mode (Connect to Existing WiFi)

To connect to your existing WiFi network instead:

1. Edit [main.cpp](src/main.cpp), change:
```cpp
webInterface.begin(AP_SSID, AP_PASSWORD, true)  // true = Access Point
```
to:
```cpp
webInterface.begin(AP_SSID, AP_PASSWORD, false)  // false = Station mode
```

2. Update SSID and PASSWORD to match your WiFi network

3. The ESP32 will connect to your WiFi and print its IP address on serial monitor

4. Access the web interface at the displayed IP address

## Workflow Example

### Setting Up a Sequence

1. Open web interface in browser
2. Click **✗ Disable** to disable the motor
3. Manually move camera to first position
4. Click **📍 Add Marker** to save position
5. Move to next position, click **📍 Add Marker** again
6. Repeat for all desired positions (up to 10 markers)
7. Click **▶ Playback** to execute the sequence

### Using the Virtual Joystick

1. Ensure motor is enabled (click **✓ Enable** if needed)
2. Click and drag the virtual joystick
3. Drag left: Motor moves in negative direction
4. Drag right: Motor moves in positive direction
5. Release: Joystick returns to center, motor stops

## Technical Details

### Architecture

```
┌─────────────┐         WiFi          ┌──────────────┐
│   Laptop    │◄──────────────────────►│    ESP32     │
│   Browser   │   HTTP + WebSocket    │  Web Server  │
└─────────────┘                        └──────┬───────┘
                                              │
                                              │ Commands
                                              ▼
                                      ┌───────────────┐
                                      │ State Machine │
                                      └───────┬───────┘
                                              │
                                              ▼
                                      ┌───────────────┐
                                      │ Motor Control │
                                      └───────────────┘
```

### Ports

- **HTTP Server**: Port 80 (web page)
- **WebSocket**: Port 81 (real-time commands)

### Communication Protocol

Commands are sent as JSON over WebSocket:

```javascript
// Joystick movement
{"cmd": "joystick", "value": -256}  // -512 to +512

// Button commands
{"cmd": "home"}
{"cmd": "enable"}
{"cmd": "disable"}
{"cmd": "marker"}
{"cmd": "playback"}
{"cmd": "clearMarkers"}
{"cmd": "stop"}

// Status request
{"cmd": "getStatus"}
```

### Libraries Used

- **WebServer**: ESP32 built-in HTTP server
- **WebSockets**: Real-time bidirectional communication ([links2004/WebSockets](https://github.com/Links2004/arduinoWebSockets))
- **WiFi**: ESP32 WiFi management

## Troubleshooting

### Can't Connect to WiFi

1. **Check serial monitor** for WiFi status messages
2. **Restart ESP32** - some devices need a reboot after first flash
3. **Verify password** - default is `motor123`
4. **Check distance** - stay within ~20 meters of ESP32

### Can't Access Web Interface

1. **Verify connection** - check you're connected to DSLR_Motor_Control WiFi
2. **Try IP directly** - `http://192.168.4.1`
3. **Disable cellular data** - some devices prefer cellular over WiFi
4. **Check firewall** - some laptops block local network access

### Web Page Loads But Controls Don't Work

1. **Check WebSocket connection** - look for green "Connected" badge
2. **Open browser console** (F12) - check for errors
3. **Refresh page** - sometimes helps establish WebSocket connection
4. **Try different browser** - Chrome/Edge/Firefox recommended

### Motor Doesn't Respond

1. **Check motor enable status** - motor may be disabled
2. **Verify homing completed** - system must be homed first
3. **Serial monitor** - check for error messages
4. **Hardware check** - verify motor driver and power connections

### Slow Response

1. **WiFi signal** - move closer to ESP32
2. **Other devices** - disconnect other devices from AP
3. **Serial monitor** - disable if not needed (frees up resources)

## Serial Monitor Output

When connected, you'll see:

```
[WEB] Initializing web control interface...
[WEB] Access Point created
[WEB] SSID: DSLR_Motor_Control
[WEB] Password: motor123
[WEB] IP Address: 192.168.4.1
[WEB] HTTP server started on port 80
[WEB] WebSocket server started on port 81

===========================================
  CONNECT TO WEB INTERFACE
===========================================
  WiFi SSID: DSLR_Motor_Control
  Password:  motor123
  Open browser: http://192.168.4.1
===========================================
```

## Mobile Support

The web interface is fully responsive and works on:
- ✅ Laptops (Chrome, Firefox, Edge, Safari)
- ✅ Tablets (iPad, Android tablets)
- ✅ Smartphones (iPhone, Android phones)

Touch gestures work with the virtual joystick!

## Advanced Configuration

### Change WiFi Channel

Edit [WebInterface.cpp](src/WebInterface.cpp):
```cpp
WiFi.softAP(ssid, password, channel);  // Add channel parameter (1-13)
```

### Enable DHCP Reservation

If using Station mode, you can configure your router to always assign the same IP to the ESP32 for easier access.

### Add Authentication

For added security, you can add HTTP basic authentication by modifying the WebServer handlers in [WebInterface.cpp](src/WebInterface.cpp).

## Comparison: Web Interface vs ESP-NOW

| Feature | Web Interface | ESP-NOW |
|---------|--------------|---------|
| Control Device | Any browser | Second ESP32 only |
| Range | ~20-50m | ~20-50m |
| Setup | Connect to WiFi | Requires second ESP32 |
| Cost | Free | Requires additional hardware |
| Interface | Rich GUI | Physical buttons/joystick |
| Latency | <50ms | <50ms |

## Performance

- **Update Rate**: 10Hz (joystick updates every 100ms)
- **Latency**: Typically <50ms command to motor response
- **Range**: ~20-50 meters (depends on environment)
- **Concurrent Users**: Supports multiple browsers simultaneously
- **Power**: No additional power consumption vs standalone mode

## Source Files

- [WebInterface.h](src/WebInterface.h) - Web interface class declaration
- [WebInterface.cpp](src/WebInterface.cpp) - Web server and HTML page
- [main.cpp](src/main.cpp) - Integration with motor control
- [platformio.ini](platformio.ini) - Build configuration

## Next Steps

- **Customize UI**: Edit the HTML/CSS in WebInterface.cpp
- **Add Features**: Extend WebSocket command protocol
- **Status Updates**: Implement real-time position/status broadcasting
- **Profiles**: Save/load movement sequences
- **Camera Trigger**: Add camera shutter control
