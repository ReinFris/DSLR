/*
 * EncoderReader.h
 *
 * A wrapper class for the AS5600 magnetic encoder that provides
 * easy access to raw angle, degrees, and rotation count.
 */

#ifndef ENCODER_READER_H
#define ENCODER_READER_H

#include <Arduino.h>
#include <AS5600.h>

class EncoderReader
{
private:
  AS5600 encoder;
  uint16_t lastAngle;        // Previous angle for detecting crossings
  uint16_t currentRawAngle;  // Cached current raw angle (offset-adjusted)
  long rotationCount;
  int16_t angleOffset;       // Offset in raw angle units (0-4095)
  float degreeOffset;        // Offset in degrees (0-360)

  // Helper method to calculate offset-adjusted angle from hardware
  uint16_t calculateRawAngle();

public:
  // Constructor
  EncoderReader();

  // Initialize the encoder
  // directionMode: 4 = default I2C direction mode
  void begin(uint8_t directionMode = 4);

  // Update internal state - call this regularly (e.g., in loop())
  // This method updates rotation count by detecting zero crossings
  void update();

  // Get raw angle value (0-4095, 12-bit resolution)
  uint16_t getRawAngle();

  // Get angle in degrees (0-360)
  float getDegrees();

  // Get total rotation count (can be negative for reverse rotation)
  long getRotationCount();

  // Setter methods
  // Set the current position as home (zero point) - offsets all future readings
  void setHome();

  // Set a specific raw angle as the offset (0-4095)
  void setAngleOffset(int16_t offset);

  // Set a specific degree as the offset (0-360)
  void setDegreeOffset(float offset);

  // Reset rotation count to zero (or a specific value)
  void setRotationCount(long count = 0);

  // Reset all offsets to zero
  void resetOffsets();

  // Diagnostic methods
  bool isConnected();
  bool detectMagnet();
  bool magnetTooStrong();
  bool magnetTooWeak();
};

#endif // ENCODER_READER_H
