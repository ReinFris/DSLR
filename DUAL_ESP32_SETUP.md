# Dual ESP32 Wireless Control Setup

This project uses two ESP32 DevKit V1 boards communicating via ESP-NOW:

## System Architecture

### Receiver (Motor Control Unit)
- **MAC Address**: `48:E7:29:B6:70:9C`
- **Role**: Controls stepper motor, reads encoder, executes sequences
- **Hardware**:
  - TMC2209 stepper driver
  - AS5600 magnetic encoder
  - Stepper motor
  - (Optional) Local joystick and button for direct control

### Sender (Wireless Remote)
- **MAC Address**: `00:70:07:1d:06:f4`
- **Role**: Wireless remote control via joystick and button
- **Hardware**:
  - Joystick module (VRX on GPIO 34, SW on GPIO 32)
  - Built-in LED for status indication

## Building and Uploading

### Receiver (Motor Control)
```bash
# Build receiver firmware
pio run -e receiver

# Upload to motor control ESP32
pio run -e receiver --target upload

# Monitor serial output
pio device monitor -e receiver
```

### Sender (Remote Control)
```bash
# Build sender firmware
pio run -e sender

# Upload to remote control ESP32
pio run -e sender --target upload

# Monitor serial output
pio device monitor -e sender
```

## Pin Configuration

### Receiver ESP32 (Motor Control)
See [main.cpp](src/main.cpp) header comments for detailed pin assignments:
- **TMC2209**: STEP (GPIO 26), DIR (GPIO 25), EN (GPIO 33), DIAG (GPIO 35)
- **AS5600**: SDA (GPIO 21), SCL (GPIO 22)
- **Joystick** (optional): VRX (GPIO 34), SW (GPIO 32)
- **LED**: GPIO 2

### Sender ESP32 (Remote Control)
- **Joystick VRX**: GPIO 34 (analog input)
- **Joystick SW**: GPIO 32 (button with internal pullup)
- **LED**: GPIO 2 (status indicator)
- **VCC**: 3.3V
- **GND**: GND

## Communication Protocol

### ESP-NOW Commands

The sender can send the following command types:

1. **CMD_JOYSTICK_MOVE**: Continuous joystick movement (-512 to +512)
2. **CMD_BUTTON_PRESS**: Button events (short press, long press, double press)
3. **CMD_SET_MARKER**: Set position marker
4. **CMD_START_PLAYBACK**: Begin playback sequence
5. **CMD_STOP**: Emergency stop
6. **CMD_ENABLE_MOTOR**: Enable motor driver
7. **CMD_DISABLE_MOTOR**: Disable motor for manual positioning
8. **CMD_HOME**: Trigger homing sequence

### Button Controls (Remote)

- **Short Press**: Set marker at current position (when motor disabled)
- **Long Press**: Toggle motor disable/start playback
- **Double Press**: Emergency stop
- **Joystick**: Move motor (proportional to deflection)

## ESP-NOW Configuration

### Setting MAC Addresses

The MAC addresses are hardcoded in the source files:

**Receiver** ([ESPNowComm.cpp](src/ESPNowComm.cpp)):
- Automatically uses the device's native MAC address
- Expected: `48:E7:29:B6:70:9C`

**Sender** ([main_sender.cpp](src/main_sender.cpp)):
```cpp
uint8_t receiverAddress[] = {0x48, 0xE7, 0x29, 0xB6, 0x70, 0x9C};
```

### Verifying MAC Addresses

1. Upload the receiver firmware and check serial monitor for MAC address
2. Update `receiverAddress[]` in [main_sender.cpp](src/main_sender.cpp) if needed
3. Rebuild and upload sender firmware

## Troubleshooting

### Communication Issues

1. **Verify MAC addresses**: Check serial output on both devices
2. **Check power**: Both ESP32s must be powered and running
3. **Distance**: Keep devices within ~20-50 meters for reliable communication
4. **Interference**: Avoid areas with heavy WiFi traffic

### LED Indicators

**Sender LED**:
- Solid ON: Ready and connected
- Fast blink: Initialization error
- Brief blink: Command sent successfully

**Receiver LED**:
- 3 blinks on startup: Normal initialization
- See main.cpp for additional LED patterns

### Serial Debugging

Enable debug output by monitoring both devices:
```bash
# Terminal 1 - Receiver
pio device monitor -e receiver

# Terminal 2 - Sender
pio device monitor -e sender
```

## Development Notes

### Adding New Commands

1. Add command type to `CommandType` enum in [ESPNowComm.h](src/ESPNowComm.h)
2. Add convenience method to `ESPNowSender` class
3. Add handler in `StateMachine::processWirelessCommand()` in [StateMachine.cpp](src/StateMachine.cpp)
4. Add sender control in [main_sender.cpp](src/main_sender.cpp) loop

### Hybrid Operation

The receiver can operate with:
- **Wireless only**: Remote control via ESP-NOW
- **Local only**: Direct joystick/button (if connected)
- **Hybrid**: Both wireless and local controls simultaneously

The state machine processes both input sources in the same loop.

## Performance

- **Update Rate**: Joystick sends updates at 10Hz (100ms interval)
- **Latency**: Typically <50ms for command execution
- **Range**: ~20-50 meters line-of-sight (depends on environment)
- **Reliability**: ESP-NOW provides built-in acknowledgment and retry

## Power Requirements

- **Receiver**: Powered via USB or external 5V supply (motor power separate)
- **Sender**: Can be powered via:
  - USB cable (tethered remote)
  - USB power bank (portable remote)
  - Battery pack with 5V regulator

## Safety Features

- **Emergency Stop**: Double-press button on remote
- **Timeout**: Consider adding timeout if no commands received (future enhancement)
- **Limit Protection**: Hardware and software limits still enforced on receiver
- **StallGuard**: Motor stall detection still active on receiver
