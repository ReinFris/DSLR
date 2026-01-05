/*
 * Position Marker System
 *
 * Manages position markers for playback sequences:
 * - Save/clear encoder positions
 * - Button handling (short/long press)
 * - Playback sequence management
 */

#ifndef MARKER_SYSTEM_H
#define MARKER_SYSTEM_H

#include <Arduino.h>
#include <AccelStepper.h>
#include "EncoderReader.h"
#include "MotorControl.h"

// ============================================================================
// DATA STRUCTURES
// ============================================================================

// Playback mode enumeration
enum PlaybackMode
{
  PLAYBACK_BY_POSITION,      // Play markers sorted by physical position
  PLAYBACK_BY_SELECTION_ORDER // Play markers in the order they were saved
};

// Position marker (encoder readings)
struct PositionMarker
{
  long rotationCount;
  uint16_t rawAngle;
  bool isSet;
};

// Button state tracking
struct ButtonState
{
  bool currentState;
  bool lastState;
  unsigned long pressStartTime;
  unsigned long releaseTime;
  bool isPressed;
  bool longPressTriggered;
};

// Playback sequence state
struct PlaybackState
{
  uint8_t currentMarkerIndex;
  unsigned long delayStartTime;
  unsigned long pauseStartTime;
  bool movingToStart;
  bool isComplete;
};

// ============================================================================
// MARKER SYSTEM CLASS
// ============================================================================

class MarkerSystem
{
public:
  // Constants
  static const uint8_t MAX_MARKERS = 10;
  static const unsigned long DEBOUNCE_DELAY = 50;
  static const unsigned long LONG_PRESS_TIME = 2000;
  static const unsigned long SHORT_PRESS_MAX = 500;
  static const unsigned long PLAYBACK_DELAY_MS = 5000;
  static const unsigned long MARKER_PAUSE_MS = 500;

  // Constructor
  MarkerSystem(EncoderReader &encoder, MotorControl &motorControl, AccelStepper &stepper);

  // Initialization
  void begin();

  // Button handling
  // Returns: 0 = no event, 1 = short press, 2 = long press
  uint8_t updateButton();

  // Marker management
  bool saveMarker();
  void clearMarkers();
  uint8_t getMarkerCount() const { return _markerCount; }
  const PositionMarker *getMarkers() const { return _markers; }

  // Playback
  void initializePlayback();
  PlaybackState &getPlaybackState() { return _playback; }

  // Playback mode management
  void setPlaybackMode(PlaybackMode mode) { _playbackMode = mode; }
  PlaybackMode getPlaybackMode() const { return _playbackMode; }
  void togglePlaybackMode();

  // State accessors
  const ButtonState &getButtonState() const { return _button; }

private:
  // Hardware references
  EncoderReader &_encoder;
  MotorControl &_motorControl;
  AccelStepper &_stepper;

  // Marker storage
  PositionMarker _markers[MAX_MARKERS];
  uint8_t _markerCount;

  // Button state
  ButtonState _button;

  // Playback state
  PlaybackState _playback;

  // Playback mode
  PlaybackMode _playbackMode;

  // Helper functions
  long determineStartPosition();
  void sortMarkers(long startPosition);
  void sortMarkersByPosition();
};

#endif // MARKER_SYSTEM_H
