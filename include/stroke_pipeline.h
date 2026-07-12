#ifndef MAGIC_WAND_STROKE_PIPELINE_H_
#define MAGIC_WAND_STROKE_PIPELINE_H_

#include <cstddef>
#include <cstdint>

enum StrokeState : int32_t {
  kStrokeWaiting = 0,
  kStrokeDrawing = 1,
  kStrokeDone = 2,
};

constexpr int kStrokeTransmitStride = 2;
constexpr int kStrokeTransmitMaxLength = 160;
constexpr int kStrokeStructByteCount =
    (2 * sizeof(int32_t)) + (2 * sizeof(int8_t) * kStrokeTransmitMaxLength);

void StrokePipelineInit(float sample_rate_hz);

// Ingest one synchronized accel + gyro sample. Sets *done_just_triggered when
// a gesture completes.
void StrokePipelineAddSample(const float accel_g[3], const float gyro_dps[3],
                             bool* done_just_triggered);

int32_t StrokePipelineGetState();
int32_t StrokePipelineGetTransmitLength();
const int8_t* StrokePipelineGetPoints();
const uint8_t* StrokePipelineGetBuffer();

#endif  // MAGIC_WAND_STROKE_PIPELINE_H_
