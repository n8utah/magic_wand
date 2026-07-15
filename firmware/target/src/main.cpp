// Halloween Target — single WS2812 color cycle on D23.

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

#include "config.h"

Adafruit_NeoPixel pixels(kWs2812LedCount, kWs2812DataPin, NEO_GRB + NEO_KHZ800);

uint32_t ColorWheel(uint8_t position) {
  position = 255 - position;
  if (position < 85) {
    return pixels.Color(255 - position * 3, 0, position * 3);
  }
  if (position < 170) {
    position -= 85;
    return pixels.Color(0, position * 3, 255 - position * 3);
  }
  position -= 170;
  return pixels.Color(position * 3, 255 - position * 3, 0);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pixels.begin();
  pixels.setBrightness(kWs2812Brightness);
  pixels.clear();
  pixels.show();

  Serial.println();
  Serial.printf("Halloween Target — WS2812 on D23 (GPIO %d)\n", kWs2812DataPin);
}

void loop() {
  for (uint16_t hue = 0; hue < 256; ++hue) {
    pixels.setPixelColor(0, ColorWheel(static_cast<uint8_t>(hue & 0xFF)));
    pixels.show();
    delay(kColorStepMs);
  }
}
