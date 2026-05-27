// =============================================================================
//  imu.h - BMI270 accelerometer bring-up + drag-launch detection.
// -----------------------------------------------------------------------------
//  Starts the Tab5's BMI270 (via the BSP sensor hub) and publishes the raw
//  acceleration vector to the EventBus. While a performance run is armed
//  (EventBus::perf_armed), it captures the at-rest gravity vector and then
//  timestamps the launch the instant the acceleration deviates past a
//  threshold - far faster (~10 ms) than speed-based detection, which is what
//  makes the drag-tree reaction time meaningful.
// =============================================================================
#pragma once

namespace imu {

// Bring up the IMU + launch detector. Call once after the BSP I2C bus is up.
void start();

} // namespace imu
