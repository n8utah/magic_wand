#include "stroke_pipeline.h"

#include <math.h>

#include "config.h"

namespace {

constexpr int kStrokeMaxLength = kStrokeTransmitMaxLength * kStrokeTransmitStride;
constexpr int kMovingSampleCount = 50;

constexpr int kAccelerationDataLength = 600 * 3;
constexpr int kGyroscopeDataLength = 600 * 3;

float acceleration_data[kAccelerationDataLength] = {};
int acceleration_data_index = 0;
float acceleration_sample_rate = 0.0f;

float gyroscope_data[kGyroscopeDataLength] = {};
float orientation_data[kGyroscopeDataLength] = {};
int gyroscope_data_index = 0;
float gyroscope_sample_rate = 0.0f;

float current_velocity[3] = {0.0f, 0.0f, 0.0f};
float current_position[3] = {0.0f, 0.0f, 0.0f};
float current_gravity[3] = {0.0f, 0.0f, 0.0f};
float current_gyroscope_drift[3] = {0.0f, 0.0f, 0.0f};

int32_t stroke_length = 0;
int8_t last_stroke_motion_mode = kStrokeMotionDual;
int last_stroke_axis_x = 2;
int last_stroke_axis_y = 1;
bool last_stroke_tilt_compensated = false;
bool last_stroke_use_gravity_plane = false;
bool last_stroke_use_pca = false;
bool last_stroke_in_plane_pca = false;
bool last_stroke_gravity_tracked = false;
float stroke_gravity_snapshot[3] = {0.0f, 1.0f, 0.0f};
bool stroke_gravity_snapshot_valid = false;
uint8_t stroke_struct_buffer[kStrokeStructByteCount] = {};
int32_t* stroke_state = reinterpret_cast<int32_t*>(stroke_struct_buffer);
int32_t* stroke_transmit_length =
    reinterpret_cast<int32_t*>(stroke_struct_buffer + sizeof(int32_t));
int8_t* stroke_points =
    reinterpret_cast<int8_t*>(stroke_struct_buffer + (sizeof(int32_t) * 2));

float VectorMagnitude(const float* vec) {
  const float x = vec[0];
  const float y = vec[1];
  const float z = vec[2];
  return sqrtf((x * x) + (y * y) + (z * z));
}

void StoreGyroscopeSample(const float gyro_dps[3]) {
  const int gyroscope_index = (gyroscope_data_index % kGyroscopeDataLength);
  gyroscope_data_index += 3;
  float* entry = &gyroscope_data[gyroscope_index];
  entry[0] = gyro_dps[0];
  entry[1] = gyro_dps[1];
  entry[2] = gyro_dps[2];
}

void StoreAccelerometerSample(const float accel_g[3]) {
  const int acceleration_index = (acceleration_data_index % kAccelerationDataLength);
  acceleration_data_index += 3;
  float* entry = &acceleration_data[acceleration_index];
  entry[0] = accel_g[0];
  entry[1] = accel_g[1];
  entry[2] = accel_g[2];
}

void EstimateGravityDirection(float* gravity) {
  int samples_to_average = 100;
  if (samples_to_average >= acceleration_data_index) {
    samples_to_average = acceleration_data_index;
  }
  if (samples_to_average == 0) {
    return;
  }

  const int start_index =
      ((acceleration_data_index +
        (kAccelerationDataLength - (3 * (samples_to_average + 1)))) %
       kAccelerationDataLength);

  float x_total = 0.0f;
  float y_total = 0.0f;
  float z_total = 0.0f;
  for (int i = 0; i < samples_to_average; ++i) {
    const int index = ((start_index + (i * 3)) % kAccelerationDataLength);
    const float* entry = &acceleration_data[index];
    x_total += entry[0];
    y_total += entry[1];
    z_total += entry[2];
  }
  gravity[0] = x_total / samples_to_average;
  gravity[1] = y_total / samples_to_average;
  gravity[2] = z_total / samples_to_average;
}

void UpdateVelocity(int new_samples, float* gravity) {
  const float gravity_x = gravity[0];
  const float gravity_y = gravity[1];
  const float gravity_z = gravity[2];

  const int start_index =
      ((acceleration_data_index +
        (kAccelerationDataLength - (3 * (new_samples + 1)))) %
       kAccelerationDataLength);

  const float friction_fudge = 0.98f;

  for (int i = 0; i < new_samples; ++i) {
    const int index = ((start_index + (i * 3)) % kAccelerationDataLength);
    const float* entry = &acceleration_data[index];
    const float ax_minus_gravity = entry[0] - gravity_x;
    const float ay_minus_gravity = entry[1] - gravity_y;
    const float az_minus_gravity = entry[2] - gravity_z;

    current_velocity[0] += ax_minus_gravity;
    current_velocity[1] += ay_minus_gravity;
    current_velocity[2] += az_minus_gravity;

    current_velocity[0] *= friction_fudge;
    current_velocity[1] *= friction_fudge;
    current_velocity[2] *= friction_fudge;

    current_position[0] += current_velocity[0];
    current_position[1] += current_velocity[1];
    current_position[2] += current_velocity[2];
  }
}

void EstimateGyroscopeDrift(float* drift) {
  const bool is_moving = VectorMagnitude(current_velocity) > 0.1f;
  if (is_moving) {
    return;
  }

  int samples_to_average = 20;
  if (samples_to_average >= gyroscope_data_index) {
    samples_to_average = gyroscope_data_index;
  }
  if (samples_to_average == 0) {
    return;
  }

  const int start_index =
      ((gyroscope_data_index +
        (kGyroscopeDataLength - (3 * (samples_to_average + 1)))) %
       kGyroscopeDataLength);

  float x_total = 0.0f;
  float y_total = 0.0f;
  float z_total = 0.0f;
  for (int i = 0; i < samples_to_average; ++i) {
    const int index = ((start_index + (i * 3)) % kGyroscopeDataLength);
    const float* entry = &gyroscope_data[index];
    x_total += entry[0];
    y_total += entry[1];
    z_total += entry[2];
  }
  drift[0] = x_total / samples_to_average;
  drift[1] = y_total / samples_to_average;
  drift[2] = z_total / samples_to_average;
}

void UpdateOrientation(int new_samples, float* drift) {
  const float drift_x = drift[0];
  const float drift_y = drift[1];
  const float drift_z = drift[2];

  const int start_index =
      ((gyroscope_data_index + (kGyroscopeDataLength - (3 * new_samples))) %
       kGyroscopeDataLength);

  const float recip_sample_rate = 1.0f / gyroscope_sample_rate;

  for (int i = 0; i < new_samples; ++i) {
    const int index = ((start_index + (i * 3)) % kGyroscopeDataLength);
    const float* entry = &gyroscope_data[index];
    const float dx_minus_drift = entry[0] - drift_x;
    const float dy_minus_drift = entry[1] - drift_y;
    const float dz_minus_drift = entry[2] - drift_z;

    const float dx_normalized = dx_minus_drift * recip_sample_rate;
    const float dy_normalized = dy_minus_drift * recip_sample_rate;
    const float dz_normalized = dz_minus_drift * recip_sample_rate;

    float* current_orientation = &orientation_data[index];
    const int previous_index =
        (index + (kGyroscopeDataLength - 3)) % kGyroscopeDataLength;
    const float* previous_orientation = &orientation_data[previous_index];
    current_orientation[0] = previous_orientation[0] + dx_normalized;
    current_orientation[1] = previous_orientation[1] + dy_normalized;
    current_orientation[2] = previous_orientation[2] + dz_normalized;
  }
}

bool IsMoving(int samples_before) {
  constexpr float moving_threshold = 10.0f;

  if ((gyroscope_data_index - samples_before) < kMovingSampleCount) {
    return false;
  }

  const int start_index =
      ((gyroscope_data_index +
        (kGyroscopeDataLength - (3 * (kMovingSampleCount + samples_before)))) %
       kGyroscopeDataLength);

  float total = 0.0f;
  for (int i = 0; i < kMovingSampleCount; ++i) {
    const int index = ((start_index + (i * 3)) % kGyroscopeDataLength);
    float* current_orientation = &orientation_data[index];
    const int previous_index =
        (index + (kGyroscopeDataLength - 3)) % kGyroscopeDataLength;
    const float* previous_orientation = &orientation_data[previous_index];
    const float dx = current_orientation[0] - previous_orientation[0];
    const float dy = current_orientation[1] - previous_orientation[1];
    const float dz = current_orientation[2] - previous_orientation[2];
    total += (dx * dx) + (dy * dy) + (dz * dz);
  }
  return total > moving_threshold;
}

enum class StrokeMotionModeInternal : int8_t {
  kDual = kStrokeMotionDual,
  kVerticalLine = kStrokeMotionVertical,
  kHorizontalLine = kStrokeMotionHorizontal,
};

float StrokeAxisSign(int axis) {
  if (axis == kStrokeVerticalAxis) {
    return kStrokeVerticalSign;
  }
  if (axis == kStrokeHorizontalAxis) {
    return kStrokeHorizontalSign;
  }
  return kStrokeShaftSign;
}

void SelectDominantStrokeAxes(int start_index, int stroke_length, float ox, float oy,
                              float oz, int* axis_x, int* axis_y) {
  float variance[3] = {0.0f, 0.0f, 0.0f};
  constexpr float range = 90.0f;

  for (int j = 0; j < stroke_length; ++j) {
    const int index = ((start_index + (j * 3)) % kGyroscopeDataLength);
    const float* entry = &orientation_data[index];
    const float nx = (entry[0] - ox) / range;
    const float ny = (entry[1] - oy) / range;
    const float nz = (entry[2] - oz) / range;
    variance[0] += nx * nx;
    variance[1] += ny * ny;
    variance[2] += nz * nz;
  }

  int order[3] = {0, 1, 2};
  for (int i = 0; i < 2; ++i) {
    for (int j = i + 1; j < 3; ++j) {
      if (variance[order[j]] > variance[order[i]]) {
        const int tmp = order[i];
        order[i] = order[j];
        order[j] = tmp;
      }
    }
  }

  int primary = order[0];
  int secondary = order[1];
  if (order[1] == kStrokeVerticalAxis) {
    primary = kStrokeVerticalAxis;
    secondary = order[0];
  } else if (order[0] != kStrokeVerticalAxis) {
    primary = order[0];
    secondary = order[1];
  }

  *axis_x = primary;
  *axis_y = secondary;
}

void Normalize3(float v[3]) {
  const float len = sqrtf((v[0] * v[0]) + (v[1] * v[1]) + (v[2] * v[2]));
  if (len < 0.0001f) {
    return;
  }
  v[0] /= len;
  v[1] /= len;
  v[2] /= len;
}

void Cross3(const float a[3], const float b[3], float out[3]) {
  out[0] = (a[1] * b[2]) - (a[2] * b[1]);
  out[1] = (a[2] * b[0]) - (a[0] * b[2]);
  out[2] = (a[0] * b[1]) - (a[1] * b[0]);
}

void EstimateGravityOverStroke(int start_index, int stroke_length, float gravity[3]) {
  float x_total = 0.0f;
  float y_total = 0.0f;
  float z_total = 0.0f;
  for (int j = 0; j < stroke_length; ++j) {
    const int index = ((start_index + (j * 3)) % kAccelerationDataLength);
    const float* entry = &acceleration_data[index];
    x_total += entry[0];
    y_total += entry[1];
    z_total += entry[2];
  }
  gravity[0] = x_total / stroke_length;
  gravity[1] = y_total / stroke_length;
  gravity[2] = z_total / stroke_length;
}

void ApplyStrokeOutputMapping(float* x_axis, float* y_axis) {
  *x_axis *= kStrokeVerticalSign;
  *y_axis *= kStrokeHorizontalSign;
  if (kSwapWandStrokeXY) {
    const float tmp = *x_axis;
    *x_axis = *y_axis;
    *y_axis = tmp;
  }
}

void GravityPlaneBuildBasis(const float gravity[3], float u[3], float v[3]) {
  float g[3] = {gravity[0], gravity[1], gravity[2]};
  Normalize3(g);

  constexpr float shaft[3] = {1.0f, 0.0f, 0.0f};
  const float shaft_dot_g =
      (shaft[0] * g[0]) + (shaft[1] * g[1]) + (shaft[2] * g[2]);

  u[0] = shaft[0] - (g[0] * shaft_dot_g);
  u[1] = shaft[1] - (g[1] * shaft_dot_g);
  u[2] = shaft[2] - (g[2] * shaft_dot_g);
  float u_len = sqrtf((u[0] * u[0]) + (u[1] * u[1]) + (u[2] * u[2]));
  if (u_len < 0.15f) {
    Cross3(shaft, g, u);
    u_len = sqrtf((u[0] * u[0]) + (u[1] * u[1]) + (u[2] * u[2]));
  }
  if (u_len < 0.15f) {
    u[0] = 0.0f;
    u[1] = 1.0f;
    u[2] = 0.0f;
    v[0] = 0.0f;
    v[1] = 0.0f;
    v[2] = 1.0f;
    return;
  }
  u[0] /= u_len;
  u[1] /= u_len;
  u[2] /= u_len;

  Cross3(g, u, v);
  Normalize3(v);
}

void GravityPlaneProject(float nx, float ny, float nz, const float gravity[3],
                         float* x_axis, float* y_axis) {
  float u[3];
  float v[3];
  GravityPlaneBuildBasis(gravity, u, v);
  *x_axis = (u[0] * nx) + (u[1] * ny) + (u[2] * nz);
  *y_axis = (v[0] * nx) + (v[1] * ny) + (v[2] * nz);
}

void PropagateGravityBetween(int from_gyro_index, int to_gyro_index, float g[3],
                             const float gyro_drift[3]) {
  constexpr float kDegToRad = 0.0174532925f;
  const float dt = 1.0f / gyroscope_sample_rate;

  if (from_gyro_index == to_gyro_index) {
    return;
  }

  int idx = from_gyro_index;
  while (idx != to_gyro_index) {
    const float* gyro = &gyroscope_data[idx];
    const float omega_rad[3] = {(gyro[0] - gyro_drift[0]) * kDegToRad,
                                (gyro[1] - gyro_drift[1]) * kDegToRad,
                                (gyro[2] - gyro_drift[2]) * kDegToRad};
    float dg[3];
    Cross3(omega_rad, g, dg);
    g[0] += dg[0] * dt;
    g[1] += dg[1] * dt;
    g[2] += dg[2] * dt;
    Normalize3(g);
    idx = (idx + 3) % kGyroscopeDataLength;
  }
}

void OrthogonalizeAgainst(const float ref[3], float v[3]) {
  const float d = (ref[0] * v[0]) + (ref[1] * v[1]) + (ref[2] * v[2]);
  v[0] -= d * ref[0];
  v[1] -= d * ref[1];
  v[2] -= d * ref[2];
  Normalize3(v);
}

void SymCov3Mul(const float c[6], const float v[3], float out[3]) {
  out[0] = (c[0] * v[0]) + (c[1] * v[1]) + (c[2] * v[2]);
  out[1] = (c[1] * v[0]) + (c[3] * v[1]) + (c[4] * v[2]);
  out[2] = (c[2] * v[0]) + (c[4] * v[1]) + (c[5] * v[2]);
}

void ComputePcaBasisFromCov(const float c[6], float e1[3], float e2[3]) {
  float v1[3] = {1.0f, 1.0f, 1.0f};
  Normalize3(v1);
  for (int k = 0; k < 12; ++k) {
    float w[3];
    SymCov3Mul(c, v1, w);
    v1[0] = w[0];
    v1[1] = w[1];
    v1[2] = w[2];
    Normalize3(v1);
  }
  e1[0] = v1[0];
  e1[1] = v1[1];
  e1[2] = v1[2];

  float v2[3] = {1.0f, 0.0f, 0.0f};
  if (fabsf((v2[0] * e1[0]) + (v2[1] * e1[1]) + (v2[2] * e1[2])) > 0.9f) {
    v2[0] = 0.0f;
    v2[1] = 1.0f;
    v2[2] = 0.0f;
  }
  OrthogonalizeAgainst(e1, v2);
  for (int k = 0; k < 10; ++k) {
    float w[3];
    SymCov3Mul(c, v2, w);
    OrthogonalizeAgainst(e1, w);
    v2[0] = w[0];
    v2[1] = w[1];
    v2[2] = w[2];
  }
  e2[0] = v2[0];
  e2[1] = v2[1];
  e2[2] = v2[2];
  Normalize3(e2);
}

void ComputePcaBasis(const float samples[3][kStrokeTransmitMaxLength], int count,
                     float e1[3], float e2[3]) {
  float c[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  for (int i = 0; i < count; ++i) {
    const float x = samples[0][i];
    const float y = samples[1][i];
    const float z = samples[2][i];
    c[0] += x * x;
    c[1] += x * y;
    c[2] += x * z;
    c[3] += y * y;
    c[4] += y * z;
    c[5] += z * z;
  }
  const float inv = 1.0f / static_cast<float>(count);
  for (int i = 0; i < 6; ++i) {
    c[i] *= inv;
  }
  ComputePcaBasisFromCov(c, e1, e2);
}

void EncodeStrokePoint(float x_axis, float y_axis, int8_t* stroke_entry) {
  int32_t unchecked_x = static_cast<int32_t>(roundf(x_axis * 128.0f));
  stroke_entry[0] = (unchecked_x > 127)   ? 127
                    : (unchecked_x < -128) ? static_cast<int8_t>(-128)
                                           : static_cast<int8_t>(unchecked_x);

  int32_t unchecked_y = static_cast<int32_t>(roundf(y_axis * 128.0f));
  stroke_entry[1] = (unchecked_y > 127)   ? 127
                    : (unchecked_y < -128) ? static_cast<int8_t>(-128)
                                           : static_cast<int8_t>(unchecked_y);
}

bool IsStrokeTipped(const float gravity[3]) {
  return kStrokeTiltCompensate && (fabsf(gravity[0]) >= kStrokeTippedThresholdG);
}

void ComputeStrokeOrientationMean(int start_index, int stroke_length, float mean[3]) {
  mean[0] = 0.0f;
  mean[1] = 0.0f;
  mean[2] = 0.0f;
  for (int j = 0; j < stroke_length; ++j) {
    const int orientation_index =
        ((start_index + (j * 3)) % kGyroscopeDataLength);
    const float* orientation_entry = &orientation_data[orientation_index];
    mean[0] += orientation_entry[0];
    mean[1] += orientation_entry[1];
    mean[2] += orientation_entry[2];
  }
  const float inv_count = 1.0f / static_cast<float>(stroke_length);
  mean[0] *= inv_count;
  mean[1] *= inv_count;
  mean[2] *= inv_count;
}

void GetOrientationAt(int start_index, int sample_index, float out[3]) {
  const int orientation_index =
      ((start_index + (sample_index * 3)) % kGyroscopeDataLength);
  const float* orientation_entry = &orientation_data[orientation_index];
  out[0] = orientation_entry[0];
  out[1] = orientation_entry[1];
  out[2] = orientation_entry[2];
}

void NormalizeOrientationSample(const float orientation[3], const float mean[3],
                                int sample_index, int stroke_length,
                                const float start_orientation[3],
                                const float end_orientation[3], bool detrend,
                                float range, float out[3]) {
  float ox = orientation[0];
  float oy = orientation[1];
  float oz = orientation[2];
  if (detrend && (stroke_length > 1)) {
    const float t = static_cast<float>(sample_index) /
                    static_cast<float>(stroke_length - 1);
    const float one_minus_t = 1.0f - t;
    ox -= (start_orientation[0] * one_minus_t) + (end_orientation[0] * t);
    oy -= (start_orientation[1] * one_minus_t) + (end_orientation[1] * t);
    oz -= (start_orientation[2] * one_minus_t) + (end_orientation[2] * t);
  }
  out[0] = (ox - mean[0]) / range;
  out[1] = (oy - mean[1]) / range;
  out[2] = (oz - mean[2]) / range;
}

void SmoothProjectedStroke(float proj_x[kStrokeTransmitMaxLength],
                             float proj_y[kStrokeTransmitMaxLength], int count) {
  if (kStrokeProjectSmoothWindow < 3 || count < 3) {
    return;
  }

  const int half = kStrokeProjectSmoothWindow / 2;
  float scratch_x[kStrokeTransmitMaxLength];
  float scratch_y[kStrokeTransmitMaxLength];
  for (int j = 0; j < count; ++j) {
    scratch_x[j] = proj_x[j];
    scratch_y[j] = proj_y[j];
  }

  for (int j = 0; j < count; ++j) {
    int from = j - half;
    int to = j + half;
    if (from < 0) {
      from = 0;
    }
    if (to >= count) {
      to = count - 1;
    }
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    int samples = 0;
    for (int k = from; k <= to; ++k) {
      sum_x += scratch_x[k];
      sum_y += scratch_y[k];
      samples += 1;
    }
    const float inv = 1.0f / static_cast<float>(samples);
    proj_x[j] = sum_x * inv;
    proj_y[j] = sum_y * inv;
  }
}

void Detrend2d(float values_u[kStrokeMaxLength], float values_v[kStrokeMaxLength],
               int count) {
  if (!kStrokeTippedDetrend || (count < 2)) {
    return;
  }
  const float inv = 1.0f / static_cast<float>(count - 1);
  for (int axis = 0; axis < 2; ++axis) {
    float* values = (axis == 0) ? values_u : values_v;
    const float start = values[0];
    const float end = values[count - 1];
    for (int j = 0; j < count; ++j) {
      const float t = static_cast<float>(j) * inv;
      values[j] -= (start * (1.0f - t)) + (end * t);
    }
  }
}

void Compute2dPcaBasis(const float pu[kStrokeMaxLength], const float pv[kStrokeMaxLength],
                       int count, float e1_u[2], float e2_u[2]) {
  float cuu = 0.0f;
  float cuv = 0.0f;
  float cvv = 0.0f;
  for (int i = 0; i < count; ++i) {
    cuu += pu[i] * pu[i];
    cuv += pu[i] * pv[i];
    cvv += pv[i] * pv[i];
  }
  const float inv = 1.0f / static_cast<float>(count);
  cuu *= inv;
  cuv *= inv;
  cvv *= inv;

  float e1x = 1.0f;
  float e1y = 1.0f;
  float len = sqrtf((e1x * e1x) + (e1y * e1y));
  e1x /= len;
  e1y /= len;
  for (int k = 0; k < 10; ++k) {
    const float wx = (cuu * e1x) + (cuv * e1y);
    const float wy = (cuv * e1x) + (cvv * e1y);
    len = sqrtf((wx * wx) + (wy * wy));
    if (len < 0.0001f) {
      e1x = 1.0f;
      e1y = 0.0f;
      break;
    }
    e1x = wx / len;
    e1y = wy / len;
  }

  e1_u[0] = e1x;
  e1_u[1] = e1y;
  e2_u[0] = -e1y;
  e2_u[1] = e1x;
}

void FinalizeProjectedStroke(const float proj_x[kStrokeTransmitMaxLength],
                             const float proj_y[kStrokeTransmitMaxLength],
                             int transmit_length, int8_t* stroke_points,
                             float* x_min, float* y_min, float* x_max, float* y_max) {
  float x_total = 0.0f;
  float y_total = 0.0f;
  if (kStrokeCenterProjectedStroke) {
    for (int j = 0; j < transmit_length; ++j) {
      x_total += proj_x[j];
      y_total += proj_y[j];
    }
  }

  const float x_mean =
      kStrokeCenterProjectedStroke ? (x_total / transmit_length) : 0.0f;
  const float y_mean =
      kStrokeCenterProjectedStroke ? (y_total / transmit_length) : 0.0f;

  float max_abs_x = 0.0001f;
  float max_abs_y = 0.0001f;
  for (int j = 0; j < transmit_length; ++j) {
    const float ax = fabsf(proj_x[j] - x_mean);
    const float ay = fabsf(proj_y[j] - y_mean);
    if (ax > max_abs_x) {
      max_abs_x = ax;
    }
    if (ay > max_abs_y) {
      max_abs_y = ay;
    }
  }

  const float extent_max = (max_abs_x > max_abs_y) ? max_abs_x : max_abs_y;
  const float extent_min = (max_abs_x > max_abs_y) ? max_abs_y : max_abs_x;
  const float extent_ratio = extent_min / extent_max;
  const bool line_like =
      kStrokeNormalizePcaAspect && (extent_ratio < kStrokeLineExtentRatio);
  last_stroke_motion_mode = line_like ? ((max_abs_x > max_abs_y) ? kStrokeMotionHorizontal
                                                                 : kStrokeMotionVertical)
                                    : kStrokeMotionDual;

  for (int j = 0; j < transmit_length; ++j) {
    const float cx = proj_x[j] - x_mean;
    const float cy = proj_y[j] - y_mean;
    float x_axis = cx;
    float y_axis = cy;

    if (kStrokeNormalizePcaAspect) {
      if (line_like) {
        if (max_abs_x > max_abs_y) {
          x_axis = cx * (kStrokePcaTargetExtent / max_abs_x);
          y_axis = 0.0f;
        } else {
          x_axis = 0.0f;
          y_axis = cy * (kStrokePcaTargetExtent / max_abs_y);
        }
      } else {
        const float extent_scale = kStrokePcaTargetExtent / extent_max;
        x_axis = cx * extent_scale;
        y_axis = cy * extent_scale;
      }
    }

    ApplyStrokeOutputMapping(&x_axis, &y_axis);
    EncodeStrokePoint(x_axis, y_axis, &stroke_points[j * 2]);

    const bool is_first = (j == 0);
    if (is_first || (x_axis < *x_min)) {
      *x_min = x_axis;
    }
    if (is_first || (y_axis < *y_min)) {
      *y_min = y_axis;
    }
    if (is_first || (x_axis > *x_max)) {
      *x_max = x_axis;
    }
    if (is_first || (y_axis > *y_max)) {
      *y_max = y_axis;
    }
  }
}

void ProjectStrokeWithPca(int start_index, int stroke_length, int transmit_length,
                          float range, int8_t* stroke_points, float* x_min, float* y_min,
                          float* x_max, float* y_max) {
  float mean[3];
  ComputeStrokeOrientationMean(start_index, stroke_length, mean);

  float c[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  for (int j = 0; j < stroke_length; ++j) {
    float orientation[3];
    GetOrientationAt(start_index, j, orientation);
    const float sample[3] = {(orientation[0] - mean[0]) / range,
                             (orientation[1] - mean[1]) / range,
                             (orientation[2] - mean[2]) / range};
    c[0] += sample[0] * sample[0];
    c[1] += sample[0] * sample[1];
    c[2] += sample[0] * sample[2];
    c[3] += sample[1] * sample[1];
    c[4] += sample[1] * sample[2];
    c[5] += sample[2] * sample[2];
  }
  const float inv_count = 1.0f / static_cast<float>(stroke_length);
  for (int i = 0; i < 6; ++i) {
    c[i] *= inv_count;
  }

  float e1[3];
  float e2[3];
  ComputePcaBasisFromCov(c, e1, e2);

  float proj_x[kStrokeTransmitMaxLength];
  float proj_y[kStrokeTransmitMaxLength];
  for (int j = 0; j < transmit_length; ++j) {
    float orientation[3];
    GetOrientationAt(start_index, j * kStrokeTransmitStride, orientation);
    const float sample[3] = {(orientation[0] - mean[0]) / range,
                             (orientation[1] - mean[1]) / range,
                             (orientation[2] - mean[2]) / range};
    proj_x[j] = (e1[0] * sample[0]) + (e1[1] * sample[1]) + (e1[2] * sample[2]);
    proj_y[j] = (e2[0] * sample[0]) + (e2[1] * sample[1]) + (e2[2] * sample[2]);
  }

  FinalizeProjectedStroke(proj_x, proj_y, transmit_length, stroke_points, x_min, y_min,
                          x_max, y_max);
}

void ProjectStrokeWithInPlanePca(int start_index, int stroke_length, int transmit_length,
                                 float range, const float stroke_gravity[3],
                                 int8_t* stroke_points, float* x_min, float* y_min,
                                 float* x_max, float* y_max) {
  float mean[3];
  ComputeStrokeOrientationMean(start_index, stroke_length, mean);

  float start_orientation[3];
  float end_orientation[3];
  GetOrientationAt(start_index, 0, start_orientation);
  GetOrientationAt(start_index, stroke_length - 1, end_orientation);
  const bool detrend = kStrokeTippedDetrend && (stroke_length > 1);

  float plane_gravity[3] = {stroke_gravity[0], stroke_gravity[1], stroke_gravity[2]};
  if (kStrokeTippedUseAccelGravity) {
    EstimateGravityOverStroke(start_index, stroke_length, plane_gravity);
  }
  float u[3];
  float v[3];
  GravityPlaneBuildBasis(plane_gravity, u, v);

  float plane_u[kStrokeMaxLength];
  float plane_v[kStrokeMaxLength];
  for (int j = 0; j < stroke_length; ++j) {
    float orientation[3];
    GetOrientationAt(start_index, j, orientation);
    float sample[3];
    NormalizeOrientationSample(orientation, mean, j, stroke_length, start_orientation,
                               end_orientation, detrend, range, sample);
    plane_u[j] = (u[0] * sample[0]) + (u[1] * sample[1]) + (u[2] * sample[2]);
    plane_v[j] = (v[0] * sample[0]) + (v[1] * sample[1]) + (v[2] * sample[2]);
  }

  Detrend2d(plane_u, plane_v, stroke_length);

  float e1[2];
  float e2[2];
  Compute2dPcaBasis(plane_u, plane_v, stroke_length, e1, e2);

  float proj_x[kStrokeTransmitMaxLength];
  float proj_y[kStrokeTransmitMaxLength];
  for (int j = 0; j < transmit_length; ++j) {
    const int sample_index = j * kStrokeTransmitStride;
    proj_x[j] = (e1[0] * plane_u[sample_index]) + (e1[1] * plane_v[sample_index]);
    proj_y[j] = (e2[0] * plane_u[sample_index]) + (e2[1] * plane_v[sample_index]);
  }

  SmoothProjectedStroke(proj_x, proj_y, transmit_length);
  FinalizeProjectedStroke(proj_x, proj_y, transmit_length, stroke_points, x_min, y_min,
                          x_max, y_max);
}

void ProjectStrokeWithGravityPlane(int start_index, int stroke_length, int transmit_length,
                                   float range, const float stroke_gravity[3],
                                   int8_t* stroke_points, float* x_min, float* y_min,
                                   float* x_max, float* y_max) {
  if (kStrokeTippedInPlanePca) {
    ProjectStrokeWithInPlanePca(start_index, stroke_length, transmit_length, range,
                                stroke_gravity, stroke_points, x_min, y_min, x_max,
                                y_max);
    return;
  }

  float mean[3];
  ComputeStrokeOrientationMean(start_index, stroke_length, mean);

  float start_orientation[3];
  float end_orientation[3];
  GetOrientationAt(start_index, 0, start_orientation);
  GetOrientationAt(start_index, stroke_length - 1, end_orientation);
  const bool detrend = kStrokeTippedDetrend && (stroke_length > 1);

  float g_at[3] = {stroke_gravity[0], stroke_gravity[1], stroke_gravity[2]};
  if (kStrokeTippedUseAccelGravity) {
    EstimateGravityOverStroke(start_index, stroke_length, g_at);
  }
  Normalize3(g_at);
  int prev_orientation_index = start_index;

  float proj_x[kStrokeTransmitMaxLength];
  float proj_y[kStrokeTransmitMaxLength];
  for (int j = 0; j < transmit_length; ++j) {
    const int sample_index = j * kStrokeTransmitStride;
    const int orientation_index =
        ((start_index + (sample_index * 3)) % kGyroscopeDataLength);

    if (kStrokeTippedTrackGravityPerSample && (j > 0)) {
      PropagateGravityBetween(prev_orientation_index, orientation_index, g_at,
                              current_gyroscope_drift);
    }
    prev_orientation_index = orientation_index;

    float orientation[3];
    GetOrientationAt(start_index, sample_index, orientation);
    float sample[3];
    NormalizeOrientationSample(orientation, mean, sample_index, stroke_length,
                               start_orientation, end_orientation, detrend, range,
                               sample);
    GravityPlaneProject(sample[0], sample[1], sample[2], g_at, &proj_x[j], &proj_y[j]);
  }

  SmoothProjectedStroke(proj_x, proj_y, transmit_length);
  FinalizeProjectedStroke(proj_x, proj_y, transmit_length, stroke_points, x_min, y_min,
                          x_max, y_max);
}

bool StrokeUsesGravityPlane(const float gravity[3]) {
  if (kStrokeAlwaysUseGravityPlane) {
    return true;
  }
  if (kStrokeUseGravityPlane) {
    return true;
  }
  return kStrokeTiltCompensate && (fabsf(gravity[0]) >= kStrokeTippedThresholdG);
}

void ComputeStrokeAxes(float nx, float ny, float nz, const float gravity[3],
                       StrokeMotionModeInternal mode, int axis_x_idx, int axis_y_idx,
                       bool use_gravity_plane, bool* tilt_compensated, float* x_axis,
                       float* y_axis) {
  const float components[3] = {nx, ny, nz};
  *tilt_compensated =
      kStrokeTiltCompensate && (fabsf(gravity[0]) >= kStrokeTippedThresholdG);

  if (use_gravity_plane) {
    switch (mode) {
      case StrokeMotionModeInternal::kVerticalLine:
        GravityPlaneProject(nx, ny, nz, gravity, x_axis, y_axis);
        *y_axis = 0.0f;
        break;
      case StrokeMotionModeInternal::kHorizontalLine:
        GravityPlaneProject(nx, ny, nz, gravity, x_axis, y_axis);
        *x_axis = 0.0f;
        break;
      case StrokeMotionModeInternal::kDual:
      default:
        GravityPlaneProject(nx, ny, nz, gravity, x_axis, y_axis);
        break;
    }
    ApplyStrokeOutputMapping(x_axis, y_axis);
    return;
  }

  float raw_x = 0.0f;
  float raw_y = 0.0f;
  if (kStrokeUseWandPlane) {
    switch (mode) {
      case StrokeMotionModeInternal::kVerticalLine:
        raw_x = StrokeAxisSign(axis_x_idx) * components[axis_x_idx];
        break;
      case StrokeMotionModeInternal::kHorizontalLine:
        raw_y = StrokeAxisSign(axis_y_idx) * components[axis_y_idx];
        break;
      case StrokeMotionModeInternal::kDual:
      default:
        raw_x = StrokeAxisSign(axis_x_idx) * components[axis_x_idx];
        raw_y = StrokeAxisSign(axis_y_idx) * components[axis_y_idx];
        break;
    }
  } else {
    const float gy = gravity[1];
    const float gz = gravity[2];
    float gmag = sqrtf((gy * gy) + (gz * gz));
    if (gmag < 0.0001f) {
      gmag = 0.0001f;
    }
    const float ngy = gy / gmag;
    const float ngz = gz / gmag;
    const float xaxisz = -ngz;
    const float xaxisy = -ngy;
    const float yaxisz = -ngy;
    const float yaxisy = ngz;
    raw_x = (xaxisz * nz) + (xaxisy * ny);
    raw_y = (yaxisz * nz) + (yaxisy * ny);
  }

  *x_axis = raw_x;
  *y_axis = raw_y;
  ApplyStrokeOutputMapping(x_axis, y_axis);
}

StrokeMotionModeInternal ClassifyStrokeMotion(int start_index, int stroke_length,
                                              float ox, float oy, float oz) {
  float variance[3] = {0.0f, 0.0f, 0.0f};
  constexpr float range = 90.0f;

  for (int j = 0; j < stroke_length; ++j) {
    const int index = ((start_index + (j * 3)) % kGyroscopeDataLength);
    const float* entry = &orientation_data[index];
    const float nx = (entry[0] - ox) / range;
    const float ny = (entry[1] - oy) / range;
    const float nz = (entry[2] - oz) / range;
    variance[0] += nx * nx;
    variance[1] += ny * ny;
    variance[2] += nz * nz;
  }

  const float var_vertical = variance[kStrokeVerticalAxis];
  const float var_horizontal = variance[kStrokeHorizontalAxis];

  if (var_horizontal < (var_vertical * kStrokeSingleAxisRatio)) {
    return StrokeMotionModeInternal::kVerticalLine;
  }
  if (var_vertical < (var_horizontal * kStrokeSingleAxisRatio)) {
    return StrokeMotionModeInternal::kHorizontalLine;
  }
  return StrokeMotionModeInternal::kDual;
}

void UpdateStroke(int new_samples, bool* done_just_triggered) {
  constexpr int minimum_stroke_length = kMovingSampleCount + 10;
  constexpr float minimum_stroke_size = 0.2f;

  *done_just_triggered = false;

  for (int i = 0; i < new_samples; ++i) {
    const int current_head = (new_samples - (i + 1));
    const bool is_moving = IsMoving(current_head);
    const int32_t old_state = *stroke_state;

    if ((old_state == kStrokeWaiting) || (old_state == kStrokeDone)) {
      if (is_moving) {
        stroke_length = kMovingSampleCount;
        *stroke_state = kStrokeDrawing;
        stroke_gravity_snapshot[0] = current_gravity[0];
        stroke_gravity_snapshot[1] = current_gravity[1];
        stroke_gravity_snapshot[2] = current_gravity[2];
        stroke_gravity_snapshot_valid = true;
      }
    } else if (old_state == kStrokeDrawing) {
      if (!is_moving) {
        if (stroke_length > minimum_stroke_length) {
          *stroke_state = kStrokeDone;
        } else {
          stroke_length = 0;
          *stroke_state = kStrokeWaiting;
        }
      }
    }

    if (*stroke_state == kStrokeWaiting) {
      continue;
    }

    stroke_length += 1;
    if (stroke_length > kStrokeMaxLength) {
      stroke_length = kStrokeMaxLength;
    }

    const bool draw_last_point =
        ((i == (new_samples - 1)) && (*stroke_state == kStrokeDrawing));
    *done_just_triggered = ((old_state != kStrokeDone) && (*stroke_state == kStrokeDone));
    if (!(*done_just_triggered || draw_last_point)) {
      continue;
    }

    const int start_index =
        ((gyroscope_data_index +
          (kGyroscopeDataLength - (3 * (stroke_length + current_head)))) %
         kGyroscopeDataLength);

    const float* origin_entry = &orientation_data[start_index];
    float ox = origin_entry[0];
    float oy = origin_entry[1];
    float oz = origin_entry[2];

    if (!kStrokeUseStartOrigin) {
      float x_total = 0.0f;
      float y_total = 0.0f;
      float z_total = 0.0f;
      for (int j = 0; j < stroke_length; ++j) {
        const int index = ((start_index + (j * 3)) % kGyroscopeDataLength);
        const float* entry = &orientation_data[index];
        x_total += entry[0];
        y_total += entry[1];
        z_total += entry[2];
      }
      ox = x_total / stroke_length;
      oy = y_total / stroke_length;
      oz = z_total / stroke_length;
    }

    constexpr float range = 90.0f;

    float stroke_gravity[3] = {current_gravity[0], current_gravity[1],
                               current_gravity[2]};
    if (stroke_gravity_snapshot_valid) {
      stroke_gravity[0] = stroke_gravity_snapshot[0];
      stroke_gravity[1] = stroke_gravity_snapshot[1];
      stroke_gravity[2] = stroke_gravity_snapshot[2];
    }
    if (kStrokeUseGravityPlane) {
      EstimateGravityOverStroke(start_index, stroke_length, stroke_gravity);
    }

    int stroke_axis_x = kStrokeVerticalAxis;
    int stroke_axis_y = kStrokeHorizontalAxis;
    const bool tipped = IsStrokeTipped(stroke_gravity);
    const bool use_pca =
        kStrokeUsePcaProjection && (!kStrokePcaFlatOnly || !tipped);
    const bool use_gravity_plane =
        !use_pca && (StrokeUsesGravityPlane(stroke_gravity) || tipped);
    last_stroke_tilt_compensated = tipped;
    last_stroke_use_gravity_plane = use_gravity_plane;
    last_stroke_in_plane_pca = use_gravity_plane && kStrokeTippedInPlanePca;
    last_stroke_gravity_tracked =
        use_gravity_plane && !kStrokeTippedInPlanePca &&
        (kStrokeTippedTrackGravityPerSample || kStrokeTrackGravityPerSample);

    if (kStrokeUseWandPlane && kStrokeAdaptiveAxes && !use_gravity_plane) {
      SelectDominantStrokeAxes(start_index, stroke_length, ox, oy, oz, &stroke_axis_x,
                               &stroke_axis_y);
      last_stroke_axis_x = stroke_axis_x;
      last_stroke_axis_y = stroke_axis_y;
    }

    StrokeMotionModeInternal stroke_motion = StrokeMotionModeInternal::kDual;
    if (kStrokeUseWandPlane && kStrokeClassifyLineMotion && !use_gravity_plane) {
      stroke_motion = ClassifyStrokeMotion(start_index, stroke_length, ox, oy, oz);
      last_stroke_motion_mode = static_cast<int8_t>(stroke_motion);
    }

    *stroke_transmit_length = stroke_length / kStrokeTransmitStride;

    float x_min = 0.0f;
    float y_min = 0.0f;
    float x_max = 0.0f;
    float y_max = 0.0f;

    last_stroke_use_pca = use_pca;
    if (use_pca) {
      ProjectStrokeWithPca(start_index, stroke_length, *stroke_transmit_length, range,
                           stroke_points, &x_min, &y_min, &x_max, &y_max);
    } else if (use_gravity_plane) {
      ProjectStrokeWithGravityPlane(start_index, stroke_length, *stroke_transmit_length,
                                      range, stroke_gravity, stroke_points, &x_min,
                                      &y_min, &x_max, &y_max);
    } else {
      float g_at[3] = {stroke_gravity[0], stroke_gravity[1], stroke_gravity[2]};
      Normalize3(g_at);
      int prev_orientation_index = start_index;

      for (int j = 0; j < *stroke_transmit_length; ++j) {
        const int orientation_index =
            ((start_index + ((j * kStrokeTransmitStride) * 3)) % kGyroscopeDataLength);

        if (use_gravity_plane && kStrokeTrackGravityPerSample && (j > 0)) {
          PropagateGravityBetween(prev_orientation_index, orientation_index, g_at,
                                  current_gyroscope_drift);
        }
        prev_orientation_index = orientation_index;

        const float* orientation_entry = &orientation_data[orientation_index];
        const float nx = (orientation_entry[0] - ox) / range;
        const float ny = (orientation_entry[1] - oy) / range;
        const float nz = (orientation_entry[2] - oz) / range;

        const float* projection_gravity =
            (use_gravity_plane && kStrokeTrackGravityPerSample) ? g_at : stroke_gravity;

        float x_axis;
        float y_axis;
        bool tilt_compensated = false;
        ComputeStrokeAxes(nx, ny, nz, projection_gravity, stroke_motion, stroke_axis_x,
                          stroke_axis_y, use_gravity_plane, &tilt_compensated, &x_axis,
                          &y_axis);
        EncodeStrokePoint(x_axis, y_axis, &stroke_points[j * 2]);

        const bool is_first = (j == 0);
        if (is_first || (x_axis < x_min)) {
          x_min = x_axis;
        }
        if (is_first || (y_axis < y_min)) {
          y_min = y_axis;
        }
        if (is_first || (x_axis > x_max)) {
          x_max = x_axis;
        }
        if (is_first || (y_axis > y_max)) {
          y_max = y_axis;
        }
      }
    }

    if (*done_just_triggered) {
      const float x_range = (x_max - x_min);
      const float y_range = (y_max - y_min);
      if ((x_range < minimum_stroke_size) && (y_range < minimum_stroke_size)) {
        *done_just_triggered = false;
        *stroke_state = kStrokeWaiting;
        *stroke_transmit_length = 0;
        stroke_length = 0;
        stroke_gravity_snapshot_valid = false;
      }
    }
  }
}

}  // namespace

void StrokePipelineInit(float sample_rate_hz) {
  acceleration_sample_rate = sample_rate_hz;
  gyroscope_sample_rate = sample_rate_hz;
  *stroke_state = kStrokeWaiting;
  *stroke_transmit_length = 0;
  stroke_length = 0;
  stroke_gravity_snapshot_valid = false;
}

void StrokePipelineAddSample(const float accel_g[3], const float gyro_dps[3],
                             bool* done_just_triggered) {
  StoreGyroscopeSample(gyro_dps);
  StoreAccelerometerSample(accel_g);

  EstimateGyroscopeDrift(current_gyroscope_drift);
  UpdateOrientation(1, current_gyroscope_drift);
  UpdateStroke(1, done_just_triggered);

  EstimateGravityDirection(current_gravity);
  UpdateVelocity(1, current_gravity);
}

int32_t StrokePipelineGetState() { return *stroke_state; }

int32_t StrokePipelineGetTransmitLength() { return *stroke_transmit_length; }

const int8_t* StrokePipelineGetPoints() { return stroke_points; }

const uint8_t* StrokePipelineGetBuffer() { return stroke_struct_buffer; }

void StrokePipelineGetLastMotionMode(int8_t* motion_mode) {
  *motion_mode = last_stroke_motion_mode;
}

void StrokePipelineGetLastAxes(int* axis_x, int* axis_y) {
  *axis_x = last_stroke_axis_x;
  *axis_y = last_stroke_axis_y;
}

bool StrokePipelineGetLastTiltCompensated() { return last_stroke_tilt_compensated; }

bool StrokePipelineGetLastUsePca() { return last_stroke_use_pca; }

bool StrokePipelineGetLastInPlanePca() { return last_stroke_in_plane_pca; }

bool StrokePipelineGetLastUseGravityPlane() { return last_stroke_use_gravity_plane; }

bool StrokePipelineGetLastGravityTracked() { return last_stroke_gravity_tracked; }
