#include "ble_spell_caster.h"

#include <NimBLEDevice.h>

#include <Arduino.h>

#include "spell_ble_protocol.h"

namespace {

struct SpellCandidate {
  NimBLEAddress address;
  int rssi;
  char name[24];
};

SpellCandidate candidates[kSpellScanTopDeviceCount];
int candidate_count = 0;

bool IsSpellTargetName(const char* name) {
  if (name == nullptr) {
    return false;
  }
  const size_t prefix_length = strlen(kSpellDeviceNamePrefix);
  return strncmp(name, kSpellDeviceNamePrefix, prefix_length) == 0;
}

void ResetCandidates() {
  candidate_count = 0;
  for (int i = 0; i < kSpellScanTopDeviceCount; ++i) {
    candidates[i].rssi = -127;
    candidates[i].name[0] = '\0';
  }
}

void InsertCandidate(NimBLEAdvertisedDevice* device) {
  if (device == nullptr) {
    return;
  }

  const std::string& name = device->getName();
  if (name.empty() || !IsSpellTargetName(name.c_str())) {
    if (!device->isAdvertisingService(NimBLEUUID(kSpellServiceUuid))) {
      return;
    }
  }

  const int rssi = device->getRSSI();
  int insert_index = candidate_count;
  if (candidate_count < kSpellScanTopDeviceCount) {
    candidate_count += 1;
  } else if (rssi <= candidates[kSpellScanTopDeviceCount - 1].rssi) {
    return;
  } else {
    insert_index = kSpellScanTopDeviceCount - 1;
  }

  candidates[insert_index].address = device->getAddress();
  candidates[insert_index].rssi = rssi;
  if (!name.empty()) {
    strncpy(candidates[insert_index].name, name.c_str(),
            sizeof(candidates[insert_index].name) - 1);
    candidates[insert_index].name[sizeof(candidates[insert_index].name) - 1] = '\0';
  } else {
    snprintf(candidates[insert_index].name, sizeof(candidates[insert_index].name),
             "%s????", kSpellDeviceNamePrefix);
  }

  while (insert_index > 0 && candidates[insert_index].rssi > candidates[insert_index - 1].rssi) {
    const SpellCandidate tmp = candidates[insert_index];
    candidates[insert_index] = candidates[insert_index - 1];
    candidates[insert_index - 1] = tmp;
    insert_index -= 1;
  }
}

class SpellScanCallbacks : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice* advertised_device) override {
    InsertCandidate(advertised_device);
  }
};

bool ActivateTarget(const SpellCandidate& candidate, char spell_character) {
  NimBLEClient* client = NimBLEDevice::createClient();
  if (client == nullptr) {
    return false;
  }

  client->setConnectTimeout(5);
  if (!client->connect(candidate.address)) {
    Serial.printf("Spell: connect failed to %s\n", candidate.name);
    NimBLEDevice::deleteClient(client);
    return false;
  }

  NimBLERemoteService* service = client->getService(kSpellServiceUuid);
  if (service == nullptr) {
    client->disconnect();
    NimBLEDevice::deleteClient(client);
    return false;
  }

  NimBLERemoteCharacteristic* character_char =
      service->getCharacteristic(kSpellCharacterUuid);
  NimBLERemoteCharacteristic* activate_char =
      service->getCharacteristic(kSpellActivateUuid);
  if ((character_char == nullptr) || (activate_char == nullptr)) {
    client->disconnect();
    NimBLEDevice::deleteClient(client);
    return false;
  }

  const std::string value = character_char->readValue();
  if (value.empty()) {
    client->disconnect();
    NimBLEDevice::deleteClient(client);
    return false;
  }

  const char target_character = value[0];
  Serial.printf("Spell: %s has character %c (RSSI %d)\n", candidate.name, target_character,
                  candidate.rssi);

  if (target_character != spell_character) {
    client->disconnect();
    NimBLEDevice::deleteClient(client);
    return false;
  }

  const uint8_t activate_command = kSpellActivateCommand;
  if (!activate_char->writeValue(&activate_command, 1, true)) {
    client->disconnect();
    NimBLEDevice::deleteClient(client);
    return false;
  }

  Serial.printf("Spell: activated %s with %c\n", candidate.name, spell_character);
  client->disconnect();
  NimBLEDevice::deleteClient(client);
  return true;
}

}  // namespace

bool BleSpellCasterTryActivate(char spell_character) {
  if ((spell_character < 32) || (spell_character > 126)) {
    return false;
  }

  ResetCandidates();

  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new SpellScanCallbacks(), false);
  scan->setActiveScan(true);
  scan->setInterval(45);
  scan->setWindow(15);
  scan->setDuplicateFilter(false);
  scan->clearResults();

  Serial.printf("Spell: scanning for character %c ...\n", spell_character);
  scan->start(2, true);

  if (candidate_count == 0) {
    Serial.println("Spell: no targets found");
    return false;
  }

  for (int i = 0; i < candidate_count; ++i) {
    if (ActivateTarget(candidates[i], spell_character)) {
      return true;
    }
  }

  Serial.printf("Spell: no match for %c among top %d targets\n", spell_character,
                  candidate_count);
  return false;
}
