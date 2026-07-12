#ifndef MAGIC_WAND_CONFIG_H_
#define MAGIC_WAND_CONFIG_H_

// Default I2C pins for ESP32 devkit (verify against your board).
constexpr int kI2cSdaPin = 21;
constexpr int kI2cSclPin = 22;

// MPU6050 I2C address when AD0 is tied to GND.
constexpr uint8_t kMpu6050Address = 0x68;

// Target sample rate used by orientation integration (must match actual rate).
constexpr float kImuSampleRateHz = 100.0f;

// Set true if gestures render vertically mirrored (e.g. ^ appears as v).
constexpr bool kFlipWandStrokeY = true;

#endif  // MAGIC_WAND_CONFIG_H_
