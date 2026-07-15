#ifndef TARGET_CONFIG_H_
#define TARGET_CONFIG_H_

// D23 on typical ESP32 devkit = GPIO23 (WS2812 data in).
constexpr int kWs2812DataPin = 23;
constexpr int kWs2812LedCount = 1;

// 0–255; keep moderate for a single LED at close range.
constexpr uint8_t kWs2812Brightness = 64;

// Delay between color steps in the cycle.
constexpr unsigned long kColorStepMs = 25;

#endif  // TARGET_CONFIG_H_
