#include "imu_frame.h"

#include <Arduino.h>
#include <math.h>

#include "imu_mpu6050.h"

#include "config.h"

namespace {

float basis_wand_x[3] = {1.0f, 0.0f, 0.0f};
float basis_wand_y[3] = {0.0f, 1.0f, 0.0f};
float basis_wand_z[3] = {0.0f, 0.0f, 1.0f};
float wand_gravity[3] = {0.0f, 1.0f, 0.0f};
int shaft_axis_index = 0;
bool calibrated = false;

float Dot(const float a[3], const float b[3]) {
  return (a[0] * b[0]) + (a[1] * b[1]) + (a[2] * b[2]);
}

void Cross(const float a[3], const float b[3], float out[3]) {
  out[0] = (a[1] * b[2]) - (a[2] * b[1]);
  out[1] = (a[2] * b[0]) - (a[0] * b[2]);
  out[2] = (a[0] * b[1]) - (a[1] * b[0]);
}

float Length(const float v[3]) { return sqrtf(Dot(v, v)); }

void Normalize(float v[3]) {
  const float len = Length(v);
  if (len < 0.0001f) {
    return;
  }
  v[0] /= len;
  v[1] /= len;
  v[2] /= len;
}

void TransformToWandFrame(const float sensor[3], float wand[3]) {
  wand[0] = Dot(sensor, basis_wand_x);
  wand[1] = Dot(sensor, basis_wand_y);
  wand[2] = Dot(sensor, basis_wand_z);
}

int PickShaftAxis(const float gravity_sensor[3]) {
  const float abs_g[3] = {fabsf(gravity_sensor[0]), fabsf(gravity_sensor[1]),
                          fabsf(gravity_sensor[2])};

  int shaft_axis = 0;
  if (abs_g[1] < abs_g[shaft_axis]) {
    shaft_axis = 1;
  }
  if (abs_g[2] < abs_g[shaft_axis]) {
    shaft_axis = 2;
  }
  return shaft_axis;
}

}  // namespace

bool ImuFrameCalibrate(int sample_count) {
  float gravity_sum[3] = {0.0f, 0.0f, 0.0f};
  int collected = 0;

  while (collected < sample_count) {
    if (!ImuSampleReady()) {
      continue;
    }

    float accel_g[3];
    float gyro_dps[3];
    if (!ImuReadSample(accel_g, gyro_dps)) {
      continue;
    }

    gravity_sum[0] += accel_g[0];
    gravity_sum[1] += accel_g[1];
    gravity_sum[2] += accel_g[2];
    collected += 1;
  }

  float gravity_sensor[3] = {
      gravity_sum[0] / collected,
      gravity_sum[1] / collected,
      gravity_sum[2] / collected,
  };
  Normalize(gravity_sensor);

  const int shaft_axis = PickShaftAxis(gravity_sensor);
  shaft_axis_index = shaft_axis;
  basis_wand_x[0] = basis_wand_x[1] = basis_wand_x[2] = 0.0f;
  basis_wand_x[shaft_axis] = 1.0f;

  Cross(basis_wand_x, gravity_sensor, basis_wand_z);
  if (Length(basis_wand_z) < 0.0001f) {
    return false;
  }
  Normalize(basis_wand_z);

  Cross(basis_wand_z, basis_wand_x, basis_wand_y);
  Normalize(basis_wand_y);

  TransformToWandFrame(gravity_sensor, wand_gravity);
  calibrated = true;
  return true;
}

void ImuFrameRemap(float accel_g[3], float gyro_dps[3]) {
  if (!calibrated) {
    return;
  }

  float remapped_accel[3];
  float remapped_gyro[3];
  TransformToWandFrame(accel_g, remapped_accel);
  TransformToWandFrame(gyro_dps, remapped_gyro);

  accel_g[0] = remapped_accel[0];
  accel_g[1] = remapped_accel[1];
  accel_g[2] = remapped_accel[2];

  gyro_dps[0] = remapped_gyro[0];
  gyro_dps[1] = remapped_gyro[1];
  gyro_dps[2] = remapped_gyro[2];

  if (kFlipWandStrokeY) {
    accel_g[1] = -accel_g[1];
    gyro_dps[1] = -gyro_dps[1];
  }
}

void ImuFrameGetWandGravity(float gravity_g[3]) {
  gravity_g[0] = wand_gravity[0];
  gravity_g[1] = wand_gravity[1];
  gravity_g[2] = wand_gravity[2];
}

void ImuFramePrintCalibration() {
  Serial.println("Wand frame calibration:");
  Serial.printf("  shaft axis (wand X) = sensor %c\n", 'X' + shaft_axis_index);
  Serial.printf("  gravity in wand frame: x=%+.3f y=%+.3f z=%+.3f (x should be ~0)\n",
                wand_gravity[0], wand_gravity[1], wand_gravity[2]);
  Serial.println("  Hold the board like a wand: long edge forward, then draw in the air.");
}
