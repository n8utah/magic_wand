#ifndef MAGIC_WAND_IMU_MPU6050_H_
#define MAGIC_WAND_IMU_MPU6050_H_

#include <cstdint>

// Prints every I2C address that ACKs on the configured bus.
void ImuScanI2cBus();

// Prints WHO_AM_I and other bring-up diagnostics to Serial.
void ImuPrintDiagnostics();

// Initializes Wire + MPU6050. Tries 0x68 and 0x69. Returns false on failure.
bool ImuBegin();

// Address selected during ImuBegin(), or 0 if not initialized.
uint8_t ImuActiveAddress();

// Configured sample rate in Hz (used by stroke pipeline).
float ImuAccelerationSampleRate();
float ImuGyroscopeSampleRate();

// True when a sample is due based on the configured sample rate.
bool ImuSampleReady();

// Reads one paired accel + gyro sample into buffers (units: g and deg/s).
// Returns false if the sensor read failed.
bool ImuReadSample(float accel_g[3], float gyro_dps[3]);

#endif  // MAGIC_WAND_IMU_MPU6050_H_
