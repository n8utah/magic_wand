#include "led_controller.h"

#include <Adafruit_NeoPixel.h>

#include <Arduino.h>

#include "ble_spell_target.h"
#include "config.h"
#include "spell_ble_protocol.h"

namespace {

Adafruit_NeoPixel pixels(kWs2812LedCount, kWs2812DataPin, NEO_GRB + NEO_KHZ800);

bool flashing = false;
unsigned long flash_start_ms = 0;
unsigned long flash_end_ms = 0;
bool flash_state = false;
unsigned long last_toggle_ms = 0;

uint8_t color_wheel_index = 0;
unsigned long last_color_step_ms = 0;

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

void SetStatusLed(bool on) {
  const bool level = kStatusLedActiveLow ? !on : on;
  digitalWrite(kStatusLedPin, level ? HIGH : LOW);
}

void SetWs2812(bool on) {
  if (on) {
    pixels.setPixelColor(0, pixels.Color(255, 255, 255));
  } else {
    pixels.setPixelColor(0, 0);
  }
  pixels.show();
}

void UpdateIdleRainbow(unsigned long now) {
  if ((now - last_color_step_ms) < kColorStepMs) {
    return;
  }
  last_color_step_ms = now;
  pixels.setPixelColor(0, ColorWheel(color_wheel_index));
  pixels.show();
  color_wheel_index++;
}

}  // namespace

void LedControllerBegin() {
  pinMode(kStatusLedPin, OUTPUT);
  SetStatusLed(false);

  pixels.begin();
  pixels.setBrightness(kWs2812Brightness);
  pixels.clear();
  pixels.show();
  last_color_step_ms = millis();

  Serial.printf("LED controller: D2 status + WS2812 on D23 (GPIO %d)\n", kWs2812DataPin);
}

void LedControllerStartActivationFlash() {
  flashing = true;
  flash_state = true;
  flash_start_ms = millis();
  flash_end_ms = flash_start_ms + kSpellTargetFlashMs;
  last_toggle_ms = flash_start_ms;
  SetStatusLed(true);
  SetWs2812(true);
  Serial.printf("LED activation flash started (%lu ms)\n", kSpellTargetFlashMs);
}

void LedControllerStop() {
  flashing = false;
  SetStatusLed(false);
  pixels.clear();
  pixels.show();
}

bool LedControllerIsFlashing() { return flashing; }

void LedControllerLoop() {
  const unsigned long now = millis();

  if (!flashing) {
    UpdateIdleRainbow(now);
    return;
  }

  if (now >= flash_end_ms) {
    flashing = false;
    SetStatusLed(false);
    pixels.clear();
    pixels.show();
    Serial.printf("LED activation flash finished (%lu ms)\n", now - flash_start_ms);
    BleSpellTargetFinishActivation();
    return;
  }

  if ((now - last_toggle_ms) >= kActivationFlashToggleMs) {
    flash_state = !flash_state;
    last_toggle_ms = now;
    SetStatusLed(flash_state);
    SetWs2812(flash_state);
  }
}
