/*
 * State Machine Implementation
 */

#include "StateMachine.h"

// ============================================================================
// CONSTRUCTOR
// ============================================================================

StateMachine::StateMachine(EncoderReader &encoder, MotorControl &motorControl,
                           MarkerSystem &markerSystem, AccelStepper &stepper)
    : _encoder(encoder), _motorControl(motorControl),
      _markerSystem(markerSystem), _stepper(stepper),
      _currentState(STATE_STARTUP)
{
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void StateMachine::begin()
{
  // Set playback mode to selection order
  _markerSystem.setPlaybackMode(PLAYBACK_BY_SELECTION_ORDER);
}

// ============================================================================
// WIRELESS COMMAND PROCESSING
// ============================================================================

void StateMachine::processWirelessCommand(const struct_command &cmd)
{
  // Handle different command types
  switch (cmd.commandType)
  {
  case CMD_JOYSTICK_MOVE:
    // Handle joystick movement (in appropriate states)
    if (_currentState == STATE_READY)
    {
      // Convert joystick value to motor movement
      // Joystick range: -512 to +512
      if (abs(cmd.joystickValue) > 50) // Deadzone
      {
        // Scale joystick to speed (max 2000 steps/sec as per motor config)
        float speedScale = (abs(cmd.joystickValue) / 512.0);
        float targetSpeed = speedScale * 2000.0;
        _stepper.setMaxSpeed(targetSpeed);

        // Calculate far target in joystick direction
        long currentPos = _stepper.currentPosition();
        long targetPos;
        if (cmd.joystickValue > 0)
        {
          targetPos = currentPos + 100000; // Large positive target
        }
        else
        {
          targetPos = currentPos - 100000; // Large negative target
        }

        // Only update target if we're not already moving in that direction
        // or if we've gotten close to the previous target
        if ((_stepper.distanceToGo() > 0 && cmd.joystickValue < 0) ||
            (_stepper.distanceToGo() < 0 && cmd.joystickValue > 0) ||
            abs(_stepper.distanceToGo()) < 1000)
        {
          _stepper.moveTo(targetPos);
        }
      }
      else
      {
        // Stop motor when joystick is centered
        _stepper.stop();
        _stepper.setMaxSpeed(2000.0); // Reset to default
      }
    }
    break;

  case CMD_BUTTON_PRESS:
    // Simulate button press based on button event
    if (cmd.buttonEvent == BTN_SHORT_PRESS)
    {
      // Process as short press (button event = 1)
      if (_currentState == STATE_READY)
      {
        handleReadyState(1);
      }
      else if (_currentState == STATE_DISABLED)
      {
        handleDisabledState(1);
      }
    }
    else if (cmd.buttonEvent == BTN_LONG_PRESS)
    {
      // Process as long press (button event = 2)
      if (_currentState == STATE_READY)
      {
        handleReadyState(2);
      }
      else if (_currentState == STATE_DISABLED)
      {
        handleDisabledState(2);
      }
    }
    break;

  case CMD_SET_MARKER:
    // Set marker at current position
    if (_currentState == STATE_DISABLED)
    {
      _markerSystem.saveMarker();
    }
    break;

  case CMD_START_PLAYBACK:
    // Start playback sequence
    if (_currentState == STATE_DISABLED)
    {
      Serial.println(F("\n=== STARTING PLAYBACK ==="));
      _currentState = STATE_PLAYBACK_DELAY;
    }
    break;

  case CMD_STOP:
    // Emergency stop - return to ready state
    if (enableLogging)
    {
      Serial.println(F("\n=== EMERGENCY STOP ==="));
    }
    _stepper.stop();
    digitalWrite(ENABLE_PIN, LOW);
    _currentState = STATE_READY;
    break;

  case CMD_ENABLE_MOTOR:
    // Enable motor
    digitalWrite(ENABLE_PIN, LOW);
    if (enableLogging)
    {
      Serial.println(F("[MOTOR] Enabled"));
    }
    break;

  case CMD_DISABLE_MOTOR:
    // Disable motor for manual positioning
    digitalWrite(ENABLE_PIN, HIGH);
    _markerSystem.clearMarkers();
    _currentState = STATE_DISABLED;
    if (enableLogging)
    {
      Serial.println(F("[MOTOR] Disabled for manual positioning"));
    }
    break;

  case CMD_HOME:
    // Trigger homing sequence
    if (enableLogging)
    {
      Serial.println(F("\n=== HOMING ==="));
    }
    _motorControl.homeX();
    _currentState = STATE_READY;
    break;

  case CMD_ENABLE_LOGGING:
    // Enable serial logging
    enableLogging = true;
    Serial.println(F("[LOGGING] Enabled"));
    break;

  case CMD_DISABLE_LOGGING:
    // Disable serial logging (may improve motion smoothness)
    Serial.println(F("[LOGGING] Disabled"));
    enableLogging = false;
    break;

  case CMD_NONE:
  default:
    // Ignore
    break;
  }
}

// ============================================================================
// MAIN UPDATE
// ============================================================================

void StateMachine::update()
{
  // Update encoder state (detects zero crossings)
  _encoder.update();

  // Update button (check for short/long press events)
  uint8_t buttonEvent = _markerSystem.updateButton();

  // State machine
  switch (_currentState)
  {
  case STATE_STARTUP:
    handleStartupState();
    break;

  case STATE_READY:
    handleReadyState(buttonEvent);
    break;

  case STATE_DISABLED:
    handleDisabledState(buttonEvent);
    break;

  case STATE_PLAYBACK_DELAY:
    handlePlaybackDelayState();
    break;

  case STATE_PLAYBACK:
    handlePlaybackState();
    break;
  }

  // Run stepper (except when disabled or waiting for playback)
  if (_currentState != STATE_DISABLED &&
      _currentState != STATE_PLAYBACK_DELAY)
  {
    _stepper.run();
  }
}

// ============================================================================
// STATE HANDLERS
// ============================================================================

void StateMachine::handleStartupState()
{
  static bool homingStarted = false;
  if (!homingStarted)
  {
    homingStarted = true;
    _motorControl.homeX();
    _currentState = STATE_READY;
  }
}

void StateMachine::handleReadyState(uint8_t buttonEvent)
{
  if (buttonEvent == 2)
  { // Long press
    Serial.println(F("\n=== MOTOR DISABLED ==="));
    Serial.println(F("Move motor manually to desired positions"));
    Serial.println(F("SHORT press to save position (max 3)"));
    Serial.println(F("LONG press to start playback sequence\n"));
    digitalWrite(ENABLE_PIN, HIGH);
    _markerSystem.clearMarkers();
    _currentState = STATE_DISABLED;
  }
}

void StateMachine::handleDisabledState(uint8_t buttonEvent)
{
  if (buttonEvent == 1)
  { // Short press
    // Save current encoder position as marker
    _markerSystem.saveMarker();
  }
  else if (buttonEvent == 2)
  { // Long press
    if (_markerSystem.getMarkerCount() == 0)
    {
      Serial.println(F("[Error] No markers saved!"));
      return;
    }
    Serial.println(F("\n=== STARTING PLAYBACK ==="));
    _markerSystem.getPlaybackState().delayStartTime = millis();
    _currentState = STATE_PLAYBACK_DELAY;
  }

  // Display position periodically (only if logging enabled)
  static unsigned long lastPrint = 0;
  if (enableLogging && millis() - lastPrint > 200)
  {
    long encoderRot = _encoder.getRotationCount();
    uint16_t encoderAngle = _encoder.getRawAngle();
    long calculatedStepperPos = _motorControl.encoderToStepperPosition(encoderRot, encoderAngle);
    long currentStepperPos = _stepper.currentPosition();
    long drift = currentStepperPos - calculatedStepperPos;

    Serial.print(F("DISABLED - Enc: "));
    Serial.print(encoderRot);
    Serial.print(F(":"));
    Serial.print(encoderAngle);
    Serial.print(F(" | Stepper: "));
    Serial.print(currentStepperPos);
    Serial.print(F(" | Drift: "));
    Serial.print(drift);
    Serial.print(F(" | Markers: "));
    Serial.print(_markerSystem.getMarkerCount());
    Serial.print(F("/"));
    Serial.println(MarkerSystem::MAX_MARKERS);
    lastPrint = millis();
  }
}

void StateMachine::handlePlaybackDelayState()
{
  PlaybackState &playback = _markerSystem.getPlaybackState();
  unsigned long elapsed = millis() - playback.delayStartTime;

  static unsigned long lastCountdown = 0;
  if (millis() - lastCountdown > 1000)
  {
    Serial.print(F("[Playback] Starting in "));
    Serial.print((MarkerSystem::PLAYBACK_DELAY_MS - elapsed) / 1000);
    Serial.println(F("s..."));
    lastCountdown = millis();
  }

  if (elapsed >= MarkerSystem::PLAYBACK_DELAY_MS)
  {
    // Verify we have at least 2 markers for playback
    if (_markerSystem.getMarkerCount() < 2) {
      Serial.print(F("[Error] Need at least 2 markers for playback. Currently have "));
      Serial.println(_markerSystem.getMarkerCount());
      _currentState = STATE_DISABLED;
      return;
    }

    Serial.println(F("[Playback] BEGIN!"));
    digitalWrite(ENABLE_PIN, LOW); // Enable motor
    delay(100);

    // Final sync to ensure accurate starting position
    _motorControl.syncStepperToEncoder();

    _markerSystem.initializePlayback();
    _currentState = STATE_PLAYBACK;
  }
}

void StateMachine::handlePlaybackState()
{
  if (_stepper.distanceToGo() != 0)
    return; // Still moving

  PlaybackState &playback = _markerSystem.getPlaybackState();
  const PositionMarker *markers = _markerSystem.getMarkers();
  uint8_t markerCount = _markerSystem.getMarkerCount();

  if (playback.movingToStart)
  {
    // Arrived at first marker, start the sequence
    Serial.println(F("[Playback] At first marker, starting sequence"));
    playback.movingToStart = false;
    playback.pauseStartTime = millis();
    playback.currentMarkerIndex = 1; // Skip first marker (already there)
    return;
  }

  if (playback.pauseStartTime > 0)
  {
    if (millis() - playback.pauseStartTime < MarkerSystem::MARKER_PAUSE_MS)
    {
      return; // Still pausing
    }
    playback.pauseStartTime = 0;
  }

  if (playback.currentMarkerIndex < markerCount)
  {
    long targetPos = _motorControl.encoderToStepperPosition(
        markers[playback.currentMarkerIndex].rotationCount,
        markers[playback.currentMarkerIndex].rawAngle);

    Serial.print(F("[Playback] Marker #"));
    Serial.println(playback.currentMarkerIndex + 1);
    _stepper.moveTo(targetPos);
    playback.pauseStartTime = millis();
    playback.currentMarkerIndex++;
  }
  else if (!playback.isComplete)
  {
    // Determine which end is furthest from last marker
    long lastMarkerPos = _motorControl.encoderToStepperPosition(
        markers[markerCount - 1].rotationCount,
        markers[markerCount - 1].rawAngle);

    long distToMin = abs(lastMarkerPos - _motorControl.getSafeMinLimit());
    long distToMax = abs(lastMarkerPos - _motorControl.getSafeMaxLimit());
    long endPos = (distToMin > distToMax) ? _motorControl.getSafeMinLimit() : _motorControl.getSafeMaxLimit();

    Serial.print(F("[Playback] Moving to opposite end: "));
    Serial.println(endPos);
    _stepper.moveTo(endPos);
    playback.isComplete = true;
  }
  else
  {
    Serial.println(F("\n=== PLAYBACK COMPLETE ==="));
    _currentState = STATE_READY;
    _markerSystem.clearMarkers();
  }
}
