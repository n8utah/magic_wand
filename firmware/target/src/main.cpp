// Halloween Target — spell BLE peripheral (ESP32-D0WD-V3 mitigations).

#include <Arduino.h>

#include "ble_spell_target.h"
#include "led_controller.h"

namespace {

void OnSpellActivate() { LedControllerStartActivationFlash(); }

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("Halloween Target — spell BLE peripheral");

  if (!BleSpellTargetBegin(OnSpellActivate)) {
    Serial.println("ERROR: BLE init failed.");
    while (true) {
      delay(1000);
    }
  }

  // GPIO2 only after BLE is up (Rev 3 boards are sensitive to early pin setup).
  LedControllerBegin();
}

void loop() {
  BleSpellTargetLoop();
  LedControllerLoop();
  delay(0);
}
