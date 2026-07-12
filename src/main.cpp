// Magic Wand — ESP32 + MPU6050
// Phase 3: BLE stroke service + web app compatibility

#include <Arduino.h>
#include <math.h>

#include "ble_stroke_service.h"
#include "config.h"
#include "imu_frame.h"
#include "imu_mpu6050.h"
#include "rasterize_stroke.h"
#include "stroke_pipeline.h"

namespace {

constexpr int kRasterWidth = 32;
constexpr int kRasterHeight = 32;
constexpr int kRasterChannels = 3;
constexpr int kRasterByteCount = kRasterWidth * kRasterHeight * kRasterChannels;

int8_t raster_buffer[kRasterByteCount];

void PrintStrokeRaster() {
  const int32_t point_count = StrokePipelineGetTransmitLength();
  if (point_count < 2) {
    Serial.println("(stroke too short to render)");
    return;
  }

  const int8_t* points = StrokePipelineGetPoints();
  RasterizeStroke(const_cast<int8_t*>(points), point_count, 0.6f, 0.6f, kRasterWidth,
                  kRasterHeight, raster_buffer);

  Serial.printf("Gesture complete — %ld points\n", static_cast<long>(point_count));
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
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("Magic Wand ESP32 — Phase 3 BLE + stroke pipeline");

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

  if (!BleStrokeServiceBegin()) {
    Serial.println("ERROR: BLE init failed.");
    while (true) {
      delay(1000);
    }
  }

  Serial.println();
  Serial.println("Open the web app in Chrome and connect via Bluetooth:");
  Serial.println("  https://petewarden.github.io/magic_wand/website/index.html");
  Serial.println("Draw gestures in the air, pause to finish each one.");
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
    PrintStrokeRaster();
  }
}
