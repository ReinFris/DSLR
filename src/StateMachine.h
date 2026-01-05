/*
 * State Machine Module
 *
 * Handles system state transitions and state-specific logic:
 * - Startup (homing)
 * - Ready (normal operation)
 * - Disabled (manual positioning)
 * - Playback delay (countdown)
 * - Playback (executing sequence)
 */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <Arduino.h>
#include <AccelStepper.h>
#include "EncoderReader.h"
#include "MotorControl.h"
#include "MarkerSystem.h"
#include "HardwareConfig.h"

// ============================================================================
// SYSTEM STATES
// ============================================================================

enum SystemState
{
  STATE_STARTUP,        // Before homing
  STATE_READY,          // Normal operation
  STATE_DISABLED,       // Motor disabled for manual positioning
  STATE_PLAYBACK_DELAY, // 5-second countdown
  STATE_PLAYBACK        // Executing playback sequence
};

// ============================================================================
// STATE MACHINE CLASS
// ============================================================================

class StateMachine
{
public:
  // Constructor
  StateMachine(EncoderReader &encoder, MotorControl &motorControl,
               MarkerSystem &markerSystem, AccelStepper &stepper);

  // Initialization
  void begin();

  // Main update function (call in loop())
  void update();

  // State accessor
  SystemState getCurrentState() const { return _currentState; }

private:
  // Hardware references
  EncoderReader &_encoder;
  MotorControl &_motorControl;
  MarkerSystem &_markerSystem;
  AccelStepper &_stepper;

  // Current state
  SystemState _currentState;

  // State handlers
  void handleStartupState();
  void handleReadyState(uint8_t buttonEvent);
  void handleDisabledState(uint8_t buttonEvent);
  void handlePlaybackDelayState();
  void handlePlaybackState();
};

#endif // STATE_MACHINE_H
