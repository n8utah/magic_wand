#include "stroke_pipeline.h"

#include <math.h>

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

    const float x_mean = x_total / stroke_length;
    const float y_mean = y_total / stroke_length;
    const float z_mean = z_total / stroke_length;
    constexpr float range = 90.0f;

    const float gy = current_gravity[1];
    const float gz = current_gravity[2];
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

    *stroke_transmit_length = stroke_length / kStrokeTransmitStride;

    float x_min = 0.0f;
    float y_min = 0.0f;
    float x_max = 0.0f;
    float y_max = 0.0f;
    for (int j = 0; j < *stroke_transmit_length; ++j) {
      const int orientation_index =
          ((start_index + ((j * kStrokeTransmitStride) * 3)) % kGyroscopeDataLength);
      const float* orientation_entry = &orientation_data[orientation_index];

      const float nx = (orientation_entry[0] - x_mean) / range;
      const float ny = (orientation_entry[1] - y_mean) / range;
      const float nz = (orientation_entry[2] - z_mean) / range;

      const float x_axis = (xaxisz * nz) + (xaxisy * ny);
      const float y_axis = (yaxisz * nz) + (yaxisy * ny);

      const int stroke_index = j * 2;
      int8_t* stroke_entry = &stroke_points[stroke_index];

      int32_t unchecked_x = static_cast<int32_t>(roundf(x_axis * 128.0f));
      stroke_entry[0] = (unchecked_x > 127)   ? 127
                        : (unchecked_x < -128) ? static_cast<int8_t>(-128)
                                               : static_cast<int8_t>(unchecked_x);

      int32_t unchecked_y = static_cast<int32_t>(roundf(y_axis * 128.0f));
      stroke_entry[1] = (unchecked_y > 127)   ? 127
                        : (unchecked_y < -128) ? static_cast<int8_t>(-128)
                                               : static_cast<int8_t>(unchecked_y);

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

    if (*done_just_triggered) {
      const float x_range = (x_max - x_min);
      const float y_range = (y_max - y_min);
      if ((x_range < minimum_stroke_size) && (y_range < minimum_stroke_size)) {
        *done_just_triggered = false;
        *stroke_state = kStrokeWaiting;
        *stroke_transmit_length = 0;
        stroke_length = 0;
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
