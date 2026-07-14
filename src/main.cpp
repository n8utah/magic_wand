// Magic Wand — ESP32 + MPU6050
// Phase 4: BLE + TFLite digit recognition

#include <Arduino.h>
#include <math.h>

#include "ble_stroke_service.h"
#include "config.h"
#include "imu_frame.h"
#include "imu_mpu6050.h"
#include "rasterize_stroke.h"
#include "stroke_pipeline.h"
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
  if (StrokePipelineGetLastUsePca()) {
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
    Serial.printf("Found %s (%d)\n\n", label, score);
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

  ImuFrameRemap(accel_g, gyro_dps);

  bool done_just_triggered = false;
  StrokePipelineAddSample(accel_g, gyro_dps, &done_just_triggered);

  if (BleStrokeServiceIsConnected()) {
    BleStrokeServiceUpdate(StrokePipelineGetBuffer(), kStrokeStructByteCount);
  }

  if (done_just_triggered) {
    PrintStrokeRasterAndClassify();
  }
}
