#ifndef MAGIC_WAND_CONFIG_H_
#define MAGIC_WAND_CONFIG_H_

// Default I2C pins for ESP32 devkit (verify against your board).
constexpr int kI2cSdaPin = 21;
constexpr int kI2cSclPin = 22;

// MPU6050 I2C address when AD0 is tied to GND.
constexpr uint8_t kMpu6050Address = 0x68;

// Target sample rate used by orientation integration (must match actual rate).
constexpr float kImuSampleRateHz = 100.0f;

// Complementary filter: blend integrated gyro with accelerometer tilt (reduces drift).
constexpr bool kOrientationAccelFusion = true;
// Accel tilt correction cross-couples Y/Z during fast strokes; keep off while drawing.
constexpr bool kOrientationAccelFusionDuringStroke = false;
// Gyro weight per sample at 100 Hz (0.992 ≈ corrects over ~1 s when still).
constexpr float kOrientationFusionAlpha = 0.992f;
constexpr float kOrientationFusionAlphaMoving = 0.998f;

// User-facing orientation (matches 3D debug view): X=shaft roll, Y=left/right, Z=up/down.
constexpr float kOrientationOutputSignX = 1.0f;
constexpr float kOrientationOutputSignY = -1.0f;
constexpr float kOrientationOutputSignZ = 1.0f;

// When still (not drawing), re-calibrate wand frame + zero orientation to current pose.
constexpr bool kOrientationAutoZero = true;

// Quick path: hand-held ready pose (~100 Hz). 30 ≈ 0.3 s.
constexpr int kOrientationAutoZeroQuickSamples = 30;
constexpr float kOrientationAutoZeroQuickGyroThresholdDps = 12.0f;
constexpr int kOrientationAutoZeroQuickCooldownSamples = 50;
// Leaky decay when hand jitter breaks stillness (avoids hard reset to 0).
constexpr int kOrientationAutoZeroQuickDecaySamples = 4;

// Full path: set-down / pick-up with frame rebuild. 100 ≈ 1.0 s.
constexpr int kOrientationAutoZeroFullSamples = 100;
constexpr float kOrientationAutoZeroFullGyroThresholdDps = 4.0f;
constexpr int kOrientationAutoZeroFullCooldownSamples = 150;
// Rebuild sensor→wand frame from averaged gravity (full settle only).
constexpr bool kOrientationAutoZeroRecalibrateFrame = true;

// Set true if gestures render vertically mirrored (e.g. ^ appears as v).
constexpr bool kFlipWandStrokeY = true;

// Swap stroke X/Y output if shapes appear rotated 90 degrees (legacy Nano).
constexpr bool kSwapWandStrokeXY = false;

// Flat strokes: always project wand tip onto gravity-perpendicular canvas.
constexpr bool kStrokeUseWandAxisProjection = true;
// Tip distance along +X shaft (matches 3D viewer red sphere at x=1.65).
constexpr float kWandTipLength = 1.65f;
// Project stroke from tip pose at gesture start.
constexpr bool kStrokeWandAxisUseStartOrigin = true;
// When false, preserve both canvas axes (no line collapse).
constexpr bool kStrokeWandAxisCollapseLines = false;

// Project each stroke with 3D PCA when flat; gravity plane when tipped.
constexpr bool kStrokeUsePcaProjection = true;
constexpr bool kStrokePcaFlatOnly = true;

// Center projected strokes before encoding (keeps shapes in the canvas middle).
constexpr bool kStrokeCenterProjectedStroke = true;

// Scale projected output to equal aspect and ~fill the raster canvas.
constexpr bool kStrokeNormalizePcaAspect = true;
constexpr float kStrokePcaTargetExtent = 0.55f;

// Remove linear gyro drift before tipped projection (helps close loops).
constexpr bool kStrokeTippedDetrend = true;

// Tipped: PCA within the gravity plane (fixed basis, no per-sample frame wobble).
constexpr bool kStrokeTippedInPlanePca = true;

// Tipped: use accelerometer-averaged gravity for plane basis (more stable than gyro).
constexpr bool kStrokeTippedUseAccelGravity = true;

// Per-sample gravity rotation (off for in-plane PCA — causes double-line artifacts).
constexpr bool kStrokeTippedTrackGravityPerSample = false;

// Box-filter width for projected points (3 = light smooth). Set 1 to disable.
constexpr int kStrokeProjectSmoothWindow = 3;

// Legacy gravity-plane path (disabled when PCA is on).
constexpr bool kStrokeAlwaysUseGravityPlane = false;

// Track gravity direction sample-by-sample (legacy flat gravity-plane path only).
constexpr bool kStrokeTrackGravityPerSample = true;

// When tipped, switch from 3D PCA to gravity-plane / in-plane PCA.
constexpr bool kStrokeTiltCompensate = true;
// Min |gravity| on wand shaft axis (X) to count as tipped, in g.
constexpr float kStrokeTippedThresholdG = 0.07f;

// Legacy: force gravity plane even when not tipped.
constexpr bool kStrokeUseGravityPlane = false;

// Use wand-frame axis components when gravity plane is off.
constexpr bool kStrokeUseWandPlane = true;

// Adaptive 2-axis fallback when gravity plane is disabled.
constexpr bool kStrokeAdaptiveAxes = true;

// Use stroke-start orientation as origin (PCA uses stroke mean instead).
constexpr bool kStrokeUseStartOrigin = false;
constexpr int kStrokeVerticalAxis = 2;
constexpr int kStrokeHorizontalAxis = 1;

// Stroke direction signs (-1.0f = default Nano convention, flip to 1.0f if reversed).
constexpr float kStrokeVerticalSign = 1.0f;
constexpr float kStrokeHorizontalSign = 1.0f;
constexpr float kStrokeShaftSign = 1.0f;

// Weak axis is ignored for straight lines when below this fraction of the stronger axis.
constexpr float kStrokeSingleAxisRatio = 0.25f;

// When false, always use both axes (needed for digits/circles). Legacy axis picker.
constexpr bool kStrokeClassifyLineMotion = false;

// Extent ratio (min/max) below this → treat as a line; above → equal-aspect (circles).
constexpr float kStrokeLineExtentRatio = 0.12f;

// D23 — WS2812 data (same wiring as spell target).
constexpr int kWs2812DataPin = 23;
constexpr int kWs2812LedCount = 1;
constexpr uint8_t kWs2812Brightness = 64;

constexpr unsigned long kWandLedEffectMs = 2000;
constexpr unsigned long kWandLedFlashToggleMs = 120;

// LED feedback and spell cast attempt threshold.
constexpr int8_t kWandRecognitionMinScore = 50;

// WS2812 color order: NEO_RGB if red/green look swapped vs firmware intent.
constexpr bool kWs2812UseRgbOrder = true;

#endif  // MAGIC_WAND_CONFIG_H_
