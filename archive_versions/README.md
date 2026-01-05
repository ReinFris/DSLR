# Archive of Development Iterations

This directory contains archived versions of `main.cpp` from various experimental branches during the DSLR motor control project development. Each file represents a different approach or feature test.

## Directory Contents

All files are snapshots of `src/main.cpp` from their respective branches, renamed for clarity.

---

## Branch Overview

### 1. `main_branch.cpp` (main)
**Status:** Reference/Production branch
**Focus:** Current stable version
**Key Features:**
- Most recent stable implementation
- Combined functionality from successful experiments

**Commit:** b292777 - "claude md"

---

### 2. `tmcAccelStepper.cpp` (tmcAccelStepper) ⭐ CURRENT
**Status:** Active development
**Focus:** TMC2209 driver with AccelStepper library integration
**Key Features:**
- TMC2209 stepper driver with UART control
- SoftwareSerial communication (RX=7, TX=8)
- StallGuard functionality for sensorless homing
- Interrupt-driven stall detection (DIAG pin 3)
- Microsteps: 256

**Libraries:**
- TMCStepper
- SoftwareSerial

**Commit:** 37d6ffd - "working"

---

### 3. `tmc-2209-homing.cpp` (tmc-2209-homing)
**Status:** Successful experiment
**Focus:** Automated sensorless homing sequence with TMC2209
**Key Features:**
- Two-stage homing sequence (find both limits)
- StallGuard-based limit detection
- Automatic startup routine:
  1. Move to first limit (detect stall)
  2. Pause 500ms
  3. Move to second limit (detect stall)
  4. Back off slightly
  5. Complete - motor stopped
- Homing speed: 60,000 VACTUAL units
- Backoff time: 200ms
- StallGuard threshold: 20 (tuned to reduce false positives)

**State Machine:**
- STARTUP_MOVING_FIRST_SIDE
- STARTUP_PAUSING
- STARTUP_MOVING_SECOND_SIDE
- STARTUP_BACKING_OFF
- STARTUP_COMPLETE

**Libraries:**
- TMCStepper
- SoftwareSerial

**Commit:** b431ef7 - "test homing"

---

### 4. `tryout-tmc2209.cpp` (tryout-tmc2209)
**Status:** Initial TMC2209 exploration
**Focus:** Basic TMC2209 driver setup and StallGuard testing
**Key Features:**
- First TMC2209 implementation
- Joystick control (VRX on A0)
- StallGuard configuration testing
- Higher current: 1770mA
- Higher StallGuard threshold: 150
- Serial command interface (0=Disable, 1=Enable, +=Faster, -=Slower)

**Libraries:**
- TMCStepper
- SoftwareSerial

**Commit:** 65da253 - "speed"

---

### 5. `test-joystick.cpp` (test-joystick)
**Status:** Successful experiment
**Focus:** Joystick control with manual homing
**Key Features:**
- AS5600 encoder integration (I2C: SDA=A4, SCL=A5)
- Joystick VRX analog input (A0)
- Joystick button for limit setting (pin 2, active LOW)
- Manual two-point homing:
  1. Move to first limit, click button
  2. Move to second limit, click button
  3. System calculates range and enforces soft limits
- Variable speed control:
  - Small deflection: HALF_SPEED (500 steps/sec)
  - Large deflection: MAX_SPEED (1000 steps/sec)
  - Deadzone: 50 units around center (512)
- AccelStepper with acceleration/deceleration
- LED blinks when encoder near zero

**Libraries:**
- AccelStepper
- Wire (I2C)
- EncoderReader (custom)

**Commit:** 0c78084 - "works fine"

---

### 6. `test-accelstepper.cpp` (test-accelstepper)
**Status:** Successful library integration
**Focus:** AccelStepper library implementation with AS5600 encoder
**Key Features:**
- AccelStepper library integration
- AS5600 magnetic encoder (12-bit resolution: 0-4095)
- EncoderReader wrapper class
- Motor configuration:
  - Steps per rev: 200 (1.8° motor)
  - Microsteps: 8
  - Total steps: 1600
  - Max speed: 9000 steps/sec
  - Acceleration: 10000 steps/sec²
- Automated test sequence:
  - 360° forward rotation
  - 1 second pause
  - 360° backward rotation
  - Continuous alternating motion every 3 seconds (720°)
- AS5600 configuration diagnostics (ZPOS, MPOS, MANG registers)
- Magnet strength detection
- LED indicator for encoder zero position

**Libraries:**
- AccelStepper
- Wire (I2C)
- EncoderReader (custom)

**Wiring documented in comments**

**Commit:** 562debe - "Implement AccelStepper library for motor control"

---

### 7. `test-as5600-encoder.cpp` (test-as5600-encoder)
**Status:** Successful sensor integration
**Focus:** AS5600 magnetic encoder testing and diagnostics
**Key Features:**
- First successful AS5600 integration
- EncoderReader wrapper class implementation
- Diagnostic features:
  - Connection check
  - Magnet detection (too strong/weak/correct/missing)
  - Register reading (ZPOS, MPOS, MANG)
  - Rotation counting (cumulative turns)
  - Angle limiting detection
- Serial output of encoder data
- Comprehensive setup documentation in comments

**Libraries:**
- Wire (I2C)
- EncoderReader (custom)

**Commit:** 25f7103 - "Add EncoderReader wrapper class and diagnostic features"

---

### 8. `minimum-snap-trajectory.cpp` (minimum-snap-trajectory)
**Status:** Advanced motion algorithm exploration
**Focus:** Camera slider timelapse with smooth trajectory planning
**Key Features:**
- Timelapse mode with configurable stops
- Minimum snap trajectory (7th order polynomial)
  - Minimizes 4th derivative (snap/jounce)
  - Industry standard for drones/CNC/precision robotics
- Multiple trajectory options implemented:
  - Raised Cosine (Hann window) - RECOMMENDED
  - Sine curve
  - Tanh sigmoid
  - Logistic sigmoid
  - Smootherstep (5th order polynomial)
  - Smootheststep (7th order polynomial)
- Timelapse parameters:
  - Configurable stops (default: 3)
  - Dwell time: 2000ms per stop
  - LED flash during photo capture simulation
- Smooth acceleration/deceleration
- Steps: 15000 per revolution (high microstepping)

**No external libraries** - pure motion control algorithms

**Commit:** 19811f5 - "comment out"

---

### 9. `smoother-motor-acceleration.cpp` (smoother-motor-acceleration)
**Status:** Motion smoothing experiment
**Focus:** Sinusoidal acceleration profile testing
**Key Features:**
- Sinusoidal acceleration curve
- Basic motion control without external libraries
- Smooth start/stop implementation
- LED indicator
- Steps: 15000 per revolution

**No external libraries**

**Commit:** 1f685ab - "sinusoid"

---

## Hardware Configuration

### Common Pin Definitions

**Stepper Motor:**
- STEP_PIN: 5
- DIR_PIN: 4
- ENABLE_PIN: 6 (active LOW)

**AS5600 Encoder (when used):**
- SDA: A4 (I2C Data)
- SCL: A5 (I2C Clock)
- VCC: 5V or 3.3V
- GND: GND

**TMC2209 UART (when used):**
- SW_RX: 7 (Arduino RX from TMC)
- SW_TX: 8 (Arduino TX to TMC)
- DIAG/StallGuard: 3 (interrupt-capable pin)
- **Important:** 1k resistor between pins 7 and 8 to PDN_UART

**Joystick (when used):**
- VRX: A0 (analog input)
- SW: 2 (button, active LOW with pull-up)

**Other:**
- LED_PIN: 13 (built-in LED)

---

## Library Dependencies by Branch

| Branch | AccelStepper | TMCStepper | Wire (I2C) | EncoderReader | SoftwareSerial |
|--------|-------------|------------|-----------|---------------|----------------|
| main_branch | ✓ | ✓ | ✓ | ✓ | ✓ |
| tmcAccelStepper | - | ✓ | - | - | ✓ |
| tmc-2209-homing | - | ✓ | - | - | ✓ |
| tryout-tmc2209 | - | ✓ | - | - | ✓ |
| test-joystick | ✓ | - | ✓ | ✓ | - |
| test-accelstepper | ✓ | - | ✓ | ✓ | - |
| test-as5600-encoder | - | - | ✓ | ✓ | - |
| minimum-snap-trajectory | - | - | - | - | - |
| smoother-motor-acceleration | - | - | - | - | - |

---

## Development Timeline

1. **First working version** (4d2f324) - Basic motor control
2. **test-as5600-encoder** (25f7103) - Added encoder support
3. **test-accelstepper** (562debe) - Integrated AccelStepper library
4. **smoother-motor-acceleration** (1f685ab) - Sinusoidal motion
5. **minimum-snap-trajectory** (19811f5) - Advanced trajectory planning
6. **test-joystick** (0c78084) - Manual control with homing
7. **tryout-tmc2209** (65da253) - First TMC2209 driver test
8. **tmc-2209-homing** (b431ef7) - Automated sensorless homing
9. **tmcAccelStepper** (37d6ffd) - Current development (ACTIVE)

---

## Key Learnings

### AccelStepper Library
- Excellent for smooth acceleration/deceleration
- Two modes:
  - `moveTo()` + `run()`: position-based with acceleration
  - `setSpeed()` + `runSpeed()`: constant velocity (useful for unhomed systems)
- Must call `run()` or `runSpeed()` frequently in loop
- Works well with joystick control by setting far targets

### TMC2209 Driver
- UART communication requires SoftwareSerial
- **Critical:** 1k resistor between RX and TX pins to PDN_UART
- **Critical:** Enable driver in hardware BEFORE UART communication
- StallGuard tuning is load-dependent:
  - Lower threshold = more sensitive (more false positives)
  - Higher threshold = less sensitive (may miss stalls)
  - Values tested: 20 (good), 150 (too high)
- VACTUAL mode bypasses step/dir pins for velocity control
- StealthChop mode for quiet operation
- Interrupt-driven stall detection works well

### AS5600 Encoder
- 12-bit resolution (0-4095 = 360°)
- I2C interface with pull-ups usually on module
- Magnet placement critical: 0.5-3mm from chip
- Can limit output range via MANG register
- Good for position feedback and rotation counting

### Joystick Control
- Center value: ~512 (10-bit ADC)
- Deadzone prevents drift
- Two-speed control improves precision
- Pull-up resistor needed for button

### Smooth Motion Algorithms
- Raised Cosine: Best for robotics (smooth, efficient)
- 7th order polynomial: Smoothest but computationally expensive
- Sinusoidal: Good balance of smoothness and simplicity

---

## Future Integration Ideas

Based on these experiments, a combined system could include:

1. **TMC2209 driver** for quiet operation and sensorless homing
2. **AS5600 encoder** for position feedback and verification
3. **AccelStepper** for smooth motion control
4. **Joystick** for manual control and limit teaching
5. **Automated startup** sequence:
   - Sensorless homing to find limits
   - Encoder calibration
   - Ready for operation
6. **Motion modes:**
   - Manual joystick control (with soft limits)
   - Timelapse with smooth trajectories
   - Continuous motion with position feedback

---

## Usage Notes

To test any archived version:

1. Copy the desired `.cpp` file to `src/main.cpp`
2. Update `platformio.ini` with required libraries
3. Check pin definitions match your hardware
4. Build and upload: `pio run --target upload`

**Important:** Some versions require additional files:
- `EncoderReader.h/.cpp` for encoder-based versions
- Specific library versions may affect behavior

---

## File Sizes

```
main_branch.cpp                 20K
tmc-2209-homing.cpp             22K
tryout-tmc2209.cpp              16K
test-accelstepper.cpp            8.7K
test-as5600-encoder.cpp          9.9K
test-joystick.cpp                8.3K
minimum-snap-trajectory.cpp     14K
smoother-motor-acceleration.cpp  7.1K
tmcAccelStepper.cpp              8.4K
```

---

## Contact / Notes

This archive preserves the iterative development process and documents lessons learned. Each approach has merit for different use cases. The current active development (tmcAccelStepper branch) aims to combine the best aspects of these experiments.

Generated: 2025-12-15
