// Magic Wand — ESP32 + MPU6050
// Phase 4: BLE + TFLite digit recognition

#include <Arduino.h>
#include <math.h>

#include "ble_stroke_service.h"
#include "ble_spell_caster.h"
#include "config.h"
#include "imu_frame.h"
#include "imu_mpu6050.h"
#include "rasterize_stroke.h"
#include "stroke_pipeline.h"
#include "spell_ble_protocol.h"
#include "tflite_runner.h"

namespace {

constexpr int kRasterWidth = 32;
constexpr int kRasterHeight = 32;
constexpr int kRasterChannels = 3;
constexpr int kRasterByteCount = kRasterWidth * kRasterHeight * kRasterChannels;

int8_t raster_buffer[kRasterByteCount];

void PrintStrokeRasterAndClassify() {
  const int32_t point_count = StrokePipelineGetTransmitLength();
  if (point_count < 2) {
    Serial.println("(stroke too short to render)");
    return;
  }

  const int8_t* points = StrokePipelineGetPoints();
  RasterizeStroke(const_cast<int8_t*>(points), point_count, 0.6f, 0.6f, kRasterWidth,
                  kRasterHeight, raster_buffer);

  Serial.printf("Gesture complete — %ld points", static_cast<long>(point_count));

  int axis_x = 0;
  int axis_y = 0;
  if (StrokePipelineGetLastUseWandAxes()) {
    Serial.println(" (stroke mode: wand tip)");
  } else if (StrokePipelineGetLastUsePca()) {
    Serial.println(" (stroke mode: PCA 3-axis)");
  } else if (StrokePipelineGetLastInPlanePca()) {
    Serial.print(" (stroke mode: in-plane PCA");
    if (StrokePipelineGetLastTiltCompensated()) {
      Serial.print(", tipped");
    }
    Serial.println(")");
  } else if (StrokePipelineGetLastUseGravityPlane()) {
    Serial.print(" (stroke mode: gravity plane");
    if (StrokePipelineGetLastGravityTracked()) {
      Serial.print(", gravity-tracked");
    }
    if (StrokePipelineGetLastTiltCompensated()) {
      Serial.print(", tipped");
    }
    Serial.println(")");
  } else {
    StrokePipelineGetLastAxes(&axis_x, &axis_y);
    Serial.printf(" (stroke axes: %c+%c)\n", 'X' + axis_x, 'X' + axis_y);
  }

  for (int y = 0; y < kRasterHeight; ++y) {
    for (int x = 0; x < kRasterWidth; ++x) {
      const int8_t* pixel =
          &raster_buffer[(y * kRasterWidth * kRasterChannels) + (x * kRasterChannels)];
      const bool ink =
          (pixel[0] > -128) || (pixel[1] > -128) || (pixel[2] > -128);
      Serial.print(ink ? '#' : '.');
    }
    Serial.println();
  }
  Serial.println();

  const char* label = "?";
  int8_t score = 0;
  if (TfliteRunnerClassify(raster_buffer, kRasterByteCount, &label, &score)) {
    Serial.printf("Found %s (%d)\n", label, score);
    if ((score >= kSpellActivationMinScore) && (label[0] != '\0')) {
      BleSpellCasterTryActivate(label[0]);
    }
    Serial.println();
  } else {
    Serial.println("Inference failed\n");
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("Magic Wand ESP32 — Phase 4 TFLite inference");

  if (!ImuBegin()) {
    Serial.println("ERROR: MPU6050 init failed.");
    ImuPrintDiagnostics();
    while (true) {
      delay(1000);
    }
  }

  Serial.println();
  Serial.println("Hold the board still in waving position (1.5 s)...");
  if (!ImuFrameCalibrate(150)) {
    Serial.println("WARN: orientation calibration failed, using raw sensor axes.");
  } else {
    ImuFramePrintCalibration();
  }

  StrokePipelineInit(ImuGyroscopeSampleRate());

  float seed_accel_g[3] = {0.0f, 0.0f, 0.0f};
  float seed_gyro_dps[3] = {0.0f, 0.0f, 0.0f};
  if (ImuReadSample(seed_accel_g, seed_gyro_dps)) {
    ImuFrameRemap(seed_accel_g, seed_gyro_dps);
    StrokePipelineResetOrientationFromGravity(seed_accel_g);
    StrokePipelineCaptureOrientationZero();
  }

  if (!TfliteRunnerBegin()) {
    Serial.println("ERROR: TFLite model init failed.");
    while (true) {
      delay(1000);
    }
  }
  Serial.println("TFLite model loaded (digits 0-9).");

  if (!BleStrokeServiceBegin()) {
    Serial.println("ERROR: BLE init failed.");
    while (true) {
      delay(1000);
    }
  }

  Serial.println();
  Serial.println("Web app: https://petewarden.github.io/magic_wand/website/index.html");
  Serial.println("Draw digits 0-9 in the air, pause to finish each gesture.");
  Serial.println("Hold still ~0.3 s in hand to zero, ~1 s on table to full calibrate.");
  Serial.println();
}

void loop() {
  if (!ImuSampleReady()) {
    return;
  }

  float accel_g[3];
  float gyro_dps[3];
  if (!ImuReadSample(accel_g, gyro_dps)) {
    return;
  }

  const float raw_accel_g[3] = {accel_g[0], accel_g[1], accel_g[2]};
  ImuFrameRemap(accel_g, gyro_dps);

  bool done_just_triggered = false;
  StrokePipelineAddSample(accel_g, gyro_dps, &done_just_triggered);

  float raw_accel_avg[3];
  float gyro_avg[3];
  const OrientationAutoZeroKind auto_zero =
      StrokePipelineUpdateAutoOrientation(raw_accel_g, gyro_dps, raw_accel_avg, gyro_avg);
  if (auto_zero != kOrientationAutoZeroNone) {
    if ((auto_zero == kOrientationAutoZeroFullSettle) &&
        kOrientationAutoZeroRecalibrateFrame) {
      ImuFrameRecalibrateFromSensorGravity(raw_accel_avg);
    }
    float wand_accel_g[3] = {raw_accel_avg[0], raw_accel_avg[1], raw_accel_avg[2]};
    float dummy_gyro[3] = {0.0f, 0.0f, 0.0f};
    ImuFrameRemap(wand_accel_g, dummy_gyro);
    StrokePipelineApplyAutoOrientationCalibration(wand_accel_g, gyro_avg);
    if (auto_zero == kOrientationAutoZeroFullSettle) {
      Serial.println("Orientation auto-calibrated (settled).");
    } else {
      Serial.println("Orientation zeroed (ready).");
    }
  }

  if (BleStrokeServiceIsConnected()) {
    BleStrokeServiceUpdate(StrokePipelineGetBuffer(), kStrokeStructByteCount);
    float orientation_deg[3];
    StrokePipelineGetCurrentOrientation(orientation_deg);
    BleStrokeServiceUpdateOrientation(orientation_deg[0], orientation_deg[1],
                                      orientation_deg[2]);
  }

  if (done_just_triggered) {
    PrintStrokeRasterAndClassify();
  }
}
