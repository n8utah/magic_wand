#include "led_controller.h"

#include <Adafruit_NeoPixel.h>

#include <Arduino.h>

#include "config.h"

namespace {

enum class EffectKind : uint8_t {
  kNone = 0,
  kBlueFlash,
  kRedFlash,
  kActivateSequence,
};

Adafruit_NeoPixel pixels(kWs2812LedCount, kWs2812DataPin,
                         (kWs2812UseRgbOrder ? NEO_RGB : NEO_GRB) + NEO_KHZ800);

EffectKind effect = EffectKind::kNone;
unsigned long effect_start_ms = 0;
bool flash_on = false;
unsigned long last_toggle_ms = 0;

void ShowColor(uint8_t red, uint8_t green, uint8_t blue) {
  pixels.setPixelColor(0, pixels.Color(red, green, blue));
  pixels.show();
}

void ShowOff() {
  pixels.setPixelColor(0, 0);
  pixels.show();
}

void FinishEffect() {
  effect = EffectKind::kNone;
  ShowOff();
}

void UpdateBlueRedFlash(unsigned long now, uint8_t red, uint8_t green, uint8_t blue) {
  if ((now - effect_start_ms) >= kWandLedEffectMs) {
    FinishEffect();
    return;
  }
  if ((now - last_toggle_ms) >= kWandLedFlashToggleMs) {
    flash_on = !flash_on;
    last_toggle_ms = now;
  }
  if (flash_on) {
    ShowColor(red, green, blue);
  } else {
    ShowOff();
  }
}

void UpdateActivateSequence(unsigned long now) {
  if ((now - effect_start_ms) >= kWandLedEffectMs) {
    FinishEffect();
    return;
  }
  const unsigned long elapsed = now - effect_start_ms;
  const unsigned long phase_ms = kWandLedEffectMs / 3;
  if (elapsed < phase_ms) {
    ShowColor(0, 0, 255);
  } else if (elapsed < (phase_ms * 2)) {
    ShowColor(255, 255, 255);
  } else {
    ShowColor(255, 0, 0);
  }
}

}  // namespace

void LedControllerBegin() {
  pixels.begin();
  pixels.setBrightness(kWs2812Brightness);
  ShowOff();
  Serial.printf("Wand WS2812 on D23 (GPIO %d)\n", kWs2812DataPin);
}

bool LedControllerIsBusy() { return effect != EffectKind::kNone; }

void LedControllerShowRecognitionResult(bool inference_ok, int8_t score,
                                        bool spell_activated) {
  effect_start_ms = millis();
  last_toggle_ms = effect_start_ms;
  flash_on = true;

  if (inference_ok && (score >= kWandRecognitionMinScore) && spell_activated) {
    effect = EffectKind::kActivateSequence;
    Serial.println("Wand LED: activate sequence (blue / white / red)");
    return;
  }
  if (inference_ok && (score >= kWandRecognitionMinScore)) {
    effect = EffectKind::kBlueFlash;
    Serial.println("Wand LED: recognized (blue flash)");
    return;
  }
  effect = EffectKind::kRedFlash;
  Serial.println("Wand LED: not recognized (red flash)");
}

void LedControllerLoop(bool is_recording_motion) {
  const unsigned long now = millis();

  if (effect == EffectKind::kBlueFlash) {
    UpdateBlueRedFlash(now, 0, 0, 255);
    return;
  }
  if (effect == EffectKind::kRedFlash) {
    UpdateBlueRedFlash(now, 255, 0, 0);
    return;
  }
  if (effect == EffectKind::kActivateSequence) {
    UpdateActivateSequence(now);
    return;
  }

  if (is_recording_motion) {
    ShowColor(0, 255, 0);
  } else {
    ShowOff();
  }
}
