/*
 * Position Marker System Implementation
 */

#include "MarkerSystem.h"

// ============================================================================
// CONSTRUCTOR
// ============================================================================

MarkerSystem::MarkerSystem(EncoderReader &encoder, MotorControl &motorControl, AccelStepper &stepper)
    : _encoder(encoder), _motorControl(motorControl), _stepper(stepper), _markerCount(0)
{
  // Initialize button state
  _button.currentState = false;
  _button.lastState = false;
  _button.pressStartTime = 0;
  _button.releaseTime = 0;
  _button.isPressed = false;
  _button.longPressTriggered = false;

  // Clear markers
  clearMarkers();
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void MarkerSystem::begin()
{
  // Nothing to initialize here - setup happens in constructor
}

// ============================================================================
// BUTTON HANDLING
// ============================================================================

uint8_t MarkerSystem::updateButton()
{
  uint8_t event = 0;
  _button.currentState = (digitalRead(JOYSTICK_SW) == LOW);
  unsigned long currentTime = millis();

  // Detect press with debouncing
  if (_button.currentState && !_button.lastState)
  {
    if (currentTime - _button.releaseTime > DEBOUNCE_DELAY)
    {
      _button.pressStartTime = currentTime;
      _button.isPressed = true;
      _button.longPressTriggered = false;
    }
  }

  // Detect long press while held
  if (_button.isPressed && !_button.longPressTriggered)
  {
    if (currentTime - _button.pressStartTime >= LONG_PRESS_TIME)
    {
      _button.longPressTriggered = true;
      event = 2; // Long press
    }
  }

  // Detect release
  if (!_button.currentState && _button.lastState)
  {
    if (currentTime - _button.pressStartTime > DEBOUNCE_DELAY)
    {
      _button.releaseTime = currentTime;
      if (_button.isPressed && !_button.longPressTriggered)
      {
        unsigned long duration = currentTime - _button.pressStartTime;
        if (duration < SHORT_PRESS_MAX)
        {
          event = 1; // Short press
        }
      }
      _button.isPressed = false;
    }
  }

  _button.lastState = _button.currentState;
  return event;
}

// ============================================================================
// MARKER MANAGEMENT
// ============================================================================

bool MarkerSystem::saveMarker()
{
  if (_markerCount >= MAX_MARKERS)
  {
    Serial.println(F("[Marker] Already at max (3) markers!"));
    return false;
  }

  long currentRotations = _encoder.getRotationCount();
  uint16_t currentRawAngle = _encoder.getRawAngle();
  long stepperPos = _motorControl.encoderToStepperPosition(currentRotations, currentRawAngle);

  _markers[_markerCount].rotationCount = currentRotations;
  _markers[_markerCount].rawAngle = currentRawAngle;
  _markers[_markerCount].isSet = true;

  Serial.print(F("[Marker] Saved #"));
  Serial.print(_markerCount + 1);
  Serial.print(F(" at stepper pos: "));
  Serial.println(stepperPos);

  _markerCount++;
  return true;
}

void MarkerSystem::clearMarkers()
{
  for (uint8_t i = 0; i < MAX_MARKERS; i++)
  {
    _markers[i].isSet = false;
  }
  _markerCount = 0;
}

// ============================================================================
// PLAYBACK HELPERS
// ============================================================================

long MarkerSystem::determineStartPosition()
{
  if (_markerCount == 0)
    return _motorControl.getMinLimit();

  long firstMarkerPos = _motorControl.encoderToStepperPosition(
      _markers[0].rotationCount, _markers[0].rawAngle);

  long distToMin = abs(firstMarkerPos - _motorControl.getMinLimit());
  long distToMax = abs(firstMarkerPos - _motorControl.getMaxLimit());

  return (distToMin < distToMax) ? _motorControl.getMinLimit() : _motorControl.getMaxLimit();
}

void MarkerSystem::sortMarkers(long startPosition)
{
  if (_markerCount <= 1)
    return;

  long markerPositions[MAX_MARKERS];
  for (uint8_t i = 0; i < _markerCount; i++)
  {
    markerPositions[i] = _motorControl.encoderToStepperPosition(
        _markers[i].rotationCount, _markers[i].rawAngle);
  }

  bool ascending = (startPosition == _motorControl.getMinLimit());
  for (uint8_t i = 0; i < _markerCount - 1; i++)
  {
    for (uint8_t j = 0; j < _markerCount - i - 1; j++)
    {
      bool swap = ascending ? (markerPositions[j] > markerPositions[j + 1]) : (markerPositions[j] < markerPositions[j + 1]);

      if (swap)
      {
        long tempPos = markerPositions[j];
        markerPositions[j] = markerPositions[j + 1];
        markerPositions[j + 1] = tempPos;

        PositionMarker tempMarker = _markers[j];
        _markers[j] = _markers[j + 1];
        _markers[j + 1] = tempMarker;
      }
    }
  }
}

void MarkerSystem::sortMarkersByPosition()
{
  if (_markerCount <= 1)
    return;

  // Bubble sort markers by stepper position (ascending)
  for (uint8_t i = 0; i < _markerCount - 1; i++)
  {
    for (uint8_t j = 0; j < _markerCount - i - 1; j++)
    {
      long posJ = _motorControl.encoderToStepperPosition(_markers[j].rotationCount, _markers[j].rawAngle);
      long posJplus1 = _motorControl.encoderToStepperPosition(_markers[j + 1].rotationCount, _markers[j + 1].rawAngle);

      if (posJ > posJplus1)
      {
        PositionMarker temp = _markers[j];
        _markers[j] = _markers[j + 1];
        _markers[j + 1] = temp;
      }
    }
  }

  Serial.println(F("[Playback] Markers sorted by position:"));
  for (uint8_t i = 0; i < _markerCount; i++)
  {
    long pos = _motorControl.encoderToStepperPosition(_markers[i].rotationCount, _markers[i].rawAngle);
    Serial.print(F("  Marker "));
    Serial.print(i + 1);
    Serial.print(F(": "));
    Serial.println(pos);
  }
}

// ============================================================================
// PLAYBACK INITIALIZATION
// ============================================================================

void MarkerSystem::initializePlayback()
{
  // Sort markers by position (ascending order)
  sortMarkersByPosition();

  _playback.currentMarkerIndex = 0;
  _playback.movingToStart = true; // Now means "moving to first marker"
  _playback.isComplete = false;

  // Calculate first marker's stepper position
  long firstMarkerPos = _motorControl.encoderToStepperPosition(
      _markers[0].rotationCount,
      _markers[0].rawAngle);

  Serial.print(F("[Playback] Moving to first marker at stepper pos: "));
  Serial.println(firstMarkerPos);
  _stepper.moveTo(firstMarkerPos);
}
