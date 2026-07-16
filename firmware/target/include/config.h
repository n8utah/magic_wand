#ifndef TARGET_CONFIG_H_
#define TARGET_CONFIG_H_

// D2 on typical ESP32 devkit = GPIO2.
constexpr int kStatusLedPin = 2;

// Built-in LED on many devkits is active LOW; external LEDs on D2 are often active HIGH.
constexpr bool kStatusLedActiveLow = false;

// D23 on typical ESP32 devkit = GPIO23 (WS2812 data in).
constexpr int kWs2812DataPin = 23;
constexpr int kWs2812LedCount = 1;

// 0–255; keep moderate for a single LED at close range.
constexpr uint8_t kWs2812Brightness = 64;

// Delay between rainbow color steps while idle.
constexpr unsigned long kColorStepMs = 25;

// Toggle interval for both D2 and WS2812 during activation flash.
constexpr unsigned long kActivationFlashToggleMs = 120;

#endif  // TARGET_CONFIG_H_
