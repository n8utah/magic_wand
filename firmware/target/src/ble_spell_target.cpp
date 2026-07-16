#include "ble_spell_target.h"

#include <NimBLEDevice.h>

#include <Arduino.h>
#include <WiFi.h>

#include "spell_ble_protocol.h"
#include "spell_character.h"
#include "led_controller.h"

namespace {

NimBLECharacteristic* character_characteristic = nullptr;
SpellTargetActivateCallback activate_callback = nullptr;
char device_name[16] = "spell-0000";
bool connected = false;
bool activating = false;
volatile bool pending_activate = false;

void BuildDeviceName();
void UpdateAdvertisedCharacter();

class SpellServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server) override {
    connected = true;
    if (!LedControllerIsFlashing()) {
      activating = false;
    }
    Serial.printf("Spell BLE: central connected (peers=%u)\n", server->getConnectedCount());
  }

  void onDisconnect(NimBLEServer* server) override {
    connected = false;
    Serial.println("Spell BLE: central disconnected");
    (void)server;
    NimBLEDevice::startAdvertising();
    Serial.println("Spell BLE: advertising restarted");
  }
};

class SpellCharacterCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* characteristic) override {
    const std::string& value = characteristic->getValue();
    if (value.empty()) {
      return;
    }
    const char new_character = static_cast<char>(value[0]);
    if (BleSpellTargetSetCharacter(new_character)) {
      Serial.printf("Spell character updated to %c\n", new_character);
    } else {
      Serial.println("Spell character update rejected (invalid character)");
    }
  }
};

class SpellActivateCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* characteristic) override {
    const std::string& value = characteristic->getValue();
    Serial.printf("Spell activate: onWrite len=%u\n",
                  static_cast<unsigned>(value.size()));
    if (value.empty() || (static_cast<uint8_t>(value[0]) != kSpellActivateCommand)) {
      Serial.println("Spell activate: ignored (bad command)");
      return;
    }
    if (activating && LedControllerIsFlashing()) {
      Serial.println("Spell activate: ignored (already activating)");
      return;
    }
    pending_activate = true;
    Serial.println("Spell activate: queued");
  }
};

void BuildDeviceName() {
  const std::string address = NimBLEDevice::getAddress().toString();
  snprintf(device_name, sizeof(device_name), "%s", kSpellDeviceNamePrefix);
  const size_t prefix_len = strlen(device_name);
  if (address.length() >= 5 && prefix_len + 4 < sizeof(device_name)) {
    device_name[prefix_len + 0] = static_cast<char>(tolower(address[address.length() - 5]));
    device_name[prefix_len + 1] = static_cast<char>(tolower(address[address.length() - 4]));
    device_name[prefix_len + 2] = static_cast<char>(tolower(address[address.length() - 2]));
    device_name[prefix_len + 3] = static_cast<char>(tolower(address[address.length() - 1]));
    device_name[prefix_len + 4] = '\0';
  }
}

void UpdateAdvertisedCharacter() {
  if (character_characteristic == nullptr) {
    return;
  }
  const char character = SpellCharacterGet();
  character_characteristic->setValue(reinterpret_cast<const uint8_t*>(&character), 1);
}

}  // namespace

bool BleSpellTargetBegin(SpellTargetActivateCallback on_activate) {
  activate_callback = on_activate;

  WiFi.mode(WIFI_OFF);

  NimBLEDevice::init("spell-target");

  BuildDeviceName();
  Serial.print("Spell target BLE address ");
  Serial.println(NimBLEDevice::getAddress().toString().c_str());

  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new SpellServerCallbacks());

  NimBLEService* service = server->createService(kMagicWandServiceUuid);

  character_characteristic = service->createCharacteristic(
      kSpellCharacterUuid, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE |
                               NIMBLE_PROPERTY::WRITE_NR);
  character_characteristic->setCallbacks(new SpellCharacterCallbacks());
  const char default_character = '?';
  character_characteristic->setValue(reinterpret_cast<const uint8_t*>(&default_character), 1);

  NimBLECharacteristic* activate_characteristic = service->createCharacteristic(
      kSpellActivateUuid, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  activate_characteristic->setCallbacks(new SpellActivateCallbacks());

  service->start();

  SpellCharacterEnsureSaved();
  UpdateAdvertisedCharacter();

  NimBLEDevice::setDeviceName(device_name);

  // Match the wand peripheral setup that works reliably in Chrome on Windows.
  NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
  advertising->addServiceUUID(kMagicWandServiceUuid);
  advertising->setScanResponse(true);
  advertising->start();

  Serial.print("Spell BLE advertising as ");
  Serial.println(device_name);
  Serial.printf("Spell target ready — %s (character %c)\n", device_name, SpellCharacterGet());
  return true;
}

void BleSpellTargetLoop() {
  if (pending_activate && !activating && (activate_callback != nullptr)) {
    pending_activate = false;
    activating = true;
    Serial.println("Spell activate command received");
    activate_callback();
  }

  static unsigned long last_heartbeat_ms = 0;
  const unsigned long now = millis();
  if ((now - last_heartbeat_ms) >= 5000) {
    last_heartbeat_ms = now;
    NimBLEServer* server = NimBLEDevice::getServer();
    NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
    const uint32_t peers = server ? server->getConnectedCount() : 0;
    Serial.printf("Spell heartbeat: advertising=%d connections=%u\n",
                  (advertising != nullptr && advertising->isAdvertising()) ? 1 : 0,
                  peers);
  }
}

char BleSpellTargetGetCharacter() { return SpellCharacterGet(); }

bool BleSpellTargetSetCharacter(char character) {
  if (!SpellCharacterSet(character)) {
    return false;
  }
  UpdateAdvertisedCharacter();
  return true;
}

bool BleSpellTargetIsActivating() { return activating; }

void BleSpellTargetFinishActivation() {
  activating = false;
  pending_activate = false;
  connected = false;
  LedControllerStop();

  NimBLEServer* server = NimBLEDevice::getServer();
  if (server != nullptr) {
    while (server->getConnectedCount() > 0) {
      NimBLEConnInfo conn_info = server->getPeerInfo(0);
      server->disconnect(conn_info.getConnHandle());
    }
  }

  NimBLEDevice::startAdvertising();
  Serial.println("Spell target ready for next activation");
}

const char* BleSpellTargetGetDeviceName() { return device_name; }
