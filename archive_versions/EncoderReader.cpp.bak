/*
 * EncoderReader.cpp
 *
 * Implementation of the EncoderReader class
 */

#include "EncoderReader.h"

// Constructor
EncoderReader::EncoderReader()
{
  lastAngle = 0;
  currentRawAngle = 0;
  rotationCount = 0;
  angleOffset = 0;
  degreeOffset = 0.0;
}

// Private helper method to calculate offset-adjusted angle from hardware
uint16_t EncoderReader::calculateRawAngle()
{
  int32_t rawAngle = (int32_t)encoder.rawAngle() - angleOffset;

  // Handle wraparound to keep value in range 0-4095
  while (rawAngle < 0)
    rawAngle += 4096;
  while (rawAngle >= 4096)
    rawAngle -= 4096;

  return (uint16_t)rawAngle;
}

// Initialize the encoder
void EncoderReader::begin(uint8_t directionMode)
{
  encoder.begin(directionMode);

  // Initialize cached values
  currentRawAngle = calculateRawAngle();
  lastAngle = currentRawAngle;

  rotationCount = 0;
}

// Update internal state - detects zero crossings for rotation counting
// This method does all the heavy lifting: reads hardware, calculates offset, detects crossings
void EncoderReader::update()
{
  // Read hardware and calculate offset-adjusted angle (heavy lifting done here)
  currentRawAngle = calculateRawAngle();

  // Detect zero crossing using difference method (more robust for fast rotation)
  // Calculate the shortest angular distance between angles
  int32_t diff = (int32_t)currentRawAngle - (int32_t)lastAngle;

  // If the absolute difference is greater than half a rotation (2048),
  // we've crossed the zero boundary and need to adjust
  if (diff > 2048)
  {
    // We wrapped backward (e.g., 100 -> 4000), so we actually moved backward
    rotationCount--;
  }
  else if (diff < -2048)
  {
    // We wrapped forward (e.g., 4000 -> 100), so we actually moved forward
    rotationCount++;
  }

  lastAngle = currentRawAngle;
}

// Get raw angle value (0-4095, 12-bit resolution) with offset applied
// Returns cached value calculated in update() - lightweight getter
uint16_t EncoderReader::getRawAngle()
{
  return currentRawAngle;
}

// Get angle in degrees (0-360) with offset applied
// Calculates from cached raw angle - lightweight calculation
float EncoderReader::getDegrees()
{
  return (currentRawAngle * 360.0) / 4096.0;
}

// Get total rotation count
long EncoderReader::getRotationCount()
{
  return rotationCount;
}

// Check if encoder is connected
bool EncoderReader::isConnected()
{
  return encoder.isConnected();
}

// Check if magnet is detected
bool EncoderReader::detectMagnet()
{
  return encoder.detectMagnet();
}

// Check if magnet is too strong
bool EncoderReader::magnetTooStrong()
{
  return encoder.magnetTooStrong();
}

// Check if magnet is too weak
bool EncoderReader::magnetTooWeak()
{
  return encoder.magnetTooWeak();
}

// Set the current position as home (zero point)
void EncoderReader::setHome()
{
  angleOffset = encoder.rawAngle();
  degreeOffset = (angleOffset * 360.0) / 4096.0;
  rotationCount = 0;

  // Recalculate cached values in new coordinate system
  currentRawAngle = calculateRawAngle();
  lastAngle = currentRawAngle;
}

// Set a specific raw angle as the offset
void EncoderReader::setAngleOffset(int16_t offset)
{
  angleOffset = offset;
  degreeOffset = (offset * 360.0) / 4096.0;

  // Recalculate cached values in new coordinate system
  currentRawAngle = calculateRawAngle();
  lastAngle = currentRawAngle;
}

// Set a specific degree as the offset
void EncoderReader::setDegreeOffset(float offset)
{
  degreeOffset = offset;
  angleOffset = (int16_t)((offset * 4096.0) / 360.0);

  // Recalculate cached values in new coordinate system
  currentRawAngle = calculateRawAngle();
  lastAngle = currentRawAngle;
}

// Reset rotation count to zero (or a specific value)
void EncoderReader::setRotationCount(long count)
{
  rotationCount = count;
}

// Reset all offsets to zero
void EncoderReader::resetOffsets()
{
  angleOffset = 0;
  degreeOffset = 0.0;
  rotationCount = 0;

  // Recalculate cached values (back to raw hardware values)
  currentRawAngle = calculateRawAngle();
  lastAngle = currentRawAngle;
}
