#ifndef MAGIC_WAND_IMU_FRAME_H_
#define MAGIC_WAND_IMU_FRAME_H_

// Builds a sensor -> wand axis remap from a still gravity sample.
// Call once at startup while the user holds the board in waving position.
bool ImuFrameCalibrate(int sample_count = 150);

// Rebuild wand frame from a gravity vector in sensor coordinates (runtime auto-cal).
bool ImuFrameRecalibrateFromSensorGravity(const float accel_sensor_g[3]);

// Rotate accel (g) and gyro (deg/s) into the wand coordinate frame.
void ImuFrameRemap(float accel_g[3], float gyro_dps[3]);

// Last calibrated gravity in wand frame (should be ~0 on X).
void ImuFrameGetWandGravity(float gravity_g[3]);

void ImuFramePrintCalibration();

#endif  // MAGIC_WAND_IMU_FRAME_H_
