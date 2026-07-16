#include "ble_spell_caster.h"

#include <NimBLEDevice.h>

#include <Arduino.h>

#include "spell_ble_protocol.h"

namespace {

struct CachedTarget {
  NimBLEAddress address;
  int rssi;
  char name[24];
  unsigned long last_seen_ms;
  bool valid;
};

CachedTarget cache[kSpellScanTopDeviceCount];
NimBLEScan* background_scan = nullptr;
bool scan_paused_for_connect = false;

void StartBackgroundScanAsync(bool continue_results);

bool IsSpellTargetName(const char* name) {
  if (name == nullptr) {
    return false;
  }
  const size_t prefix_length = strlen(kSpellDeviceNamePrefix);
  return strncmp(name, kSpellDeviceNamePrefix, prefix_length) == 0;
}

bool IsSpellTargetDevice(NimBLEAdvertisedDevice* device) {
  if (device == nullptr) {
    return false;
  }
  const std::string& name = device->getName();
  if (!name.empty() && IsSpellTargetName(name.c_str())) {
    return true;
  }
  return device->isAdvertisingService(NimBLEUUID(kSpellServiceUuid));
}

void SortCacheByRssi() {
  for (int i = 0; i < kSpellScanTopDeviceCount; ++i) {
    for (int j = i + 1; j < kSpellScanTopDeviceCount; ++j) {
      if (!cache[j].valid) {
        continue;
      }
      if (!cache[i].valid || (cache[j].rssi > cache[i].rssi)) {
        const CachedTarget tmp = cache[i];
        cache[i] = cache[j];
        cache[j] = tmp;
      }
    }
  }
}

void SetTargetName(CachedTarget* target, const std::string& name) {
  if (!name.empty()) {
    strncpy(target->name, name.c_str(), sizeof(target->name) - 1);
    target->name[sizeof(target->name) - 1] = '\0';
  } else {
    snprintf(target->name, sizeof(target->name), "%s????", kSpellDeviceNamePrefix);
  }
}

void UpsertTarget(NimBLEAdvertisedDevice* device) {
  if (!IsSpellTargetDevice(device)) {
    return;
  }

  const unsigned long now = millis();
  const int rssi = device->getRSSI();
  const NimBLEAddress address = device->getAddress();
  const std::string& name = device->getName();

  int existing_index = -1;
  int empty_index = -1;
  int weakest_index = -1;
  int weakest_rssi = 127;
  int valid_count = 0;

  for (int i = 0; i < kSpellScanTopDeviceCount; ++i) {
    if (!cache[i].valid) {
      if (empty_index < 0) {
        empty_index = i;
      }
      continue;
    }
    if ((now - cache[i].last_seen_ms) > kSpellTargetCacheTimeoutMs) {
      cache[i].valid = false;
      if (empty_index < 0) {
        empty_index = i;
      }
      continue;
    }

    valid_count += 1;
    if (cache[i].address == address) {
      existing_index = i;
    }
    if (cache[i].rssi < weakest_rssi) {
      weakest_rssi = cache[i].rssi;
      weakest_index = i;
    }
  }

  if (existing_index >= 0) {
    cache[existing_index].rssi = rssi;
    cache[existing_index].last_seen_ms = now;
    if (!name.empty()) {
      SetTargetName(&cache[existing_index], name);
    }
    SortCacheByRssi();
    return;
  }

  int slot = empty_index;
  if (slot < 0) {
    if ((valid_count >= kSpellScanTopDeviceCount) && (rssi > weakest_rssi) &&
        (weakest_index >= 0)) {
      slot = weakest_index;
    } else {
      return;
    }
  }

  cache[slot].valid = true;
  cache[slot].address = address;
  cache[slot].rssi = rssi;
  cache[slot].last_seen_ms = now;
  SetTargetName(&cache[slot], name);
  SortCacheByRssi();
}

void ExpireStaleTargets() {
  const unsigned long now = millis();
  for (int i = 0; i < kSpellScanTopDeviceCount; ++i) {
    if (!cache[i].valid) {
      continue;
    }
    if ((now - cache[i].last_seen_ms) > kSpellTargetCacheTimeoutMs) {
      Serial.printf("Spell cache: dropped %s rssi=%d (timeout)\n", cache[i].name,
                    cache[i].rssi);
      cache[i].valid = false;
    }
  }
}

class SpellScanCallbacks : public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice* advertised_device) override {
    UpsertTarget(advertised_device);
  }
};

SpellScanCallbacks* scan_callbacks = nullptr;

bool ActivateTarget(const CachedTarget& target, char spell_character) {
  NimBLEClient* client = NimBLEDevice::createClient();
  if (client == nullptr) {
    return false;
  }

  client->setConnectTimeout(5);
  if (!client->connect(target.address)) {
    Serial.printf("Spell: connect failed to %s (rssi=%d)\n", target.name, target.rssi);
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
  Serial.printf("Spell: %s has character %c (RSSI %d)\n", target.name, target_character,
                  target.rssi);

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

  Serial.printf("Spell: activated %s with %c\n", target.name, spell_character);
  client->disconnect();
  NimBLEDevice::deleteClient(client);
  return true;
}

void LogCacheSnapshot() {
  const unsigned long now = millis();
  Serial.println("Spell cache (strongest RSSI first):");
  bool any = false;
  for (int i = 0; i < kSpellScanTopDeviceCount; ++i) {
    if (!cache[i].valid) {
      continue;
    }
    any = true;
    Serial.printf("  %s rssi=%d age=%lums\n", cache[i].name, cache[i].rssi,
                  now - cache[i].last_seen_ms);
  }
  if (!any) {
    Serial.println("  (empty)");
  }
}

void OnBackgroundScanEnd(NimBLEScanResults scan_results) {
  (void)scan_results;
  if (!scan_paused_for_connect) {
    StartBackgroundScanAsync(true);
  }
}

void StartBackgroundScanAsync(bool continue_results) {
  if (background_scan == nullptr || scan_paused_for_connect) {
    return;
  }
  if (background_scan->isScanning()) {
    return;
  }
  // Must use the callback overload — start(duration, bool) blocks forever when duration=0.
  background_scan->start(kSpellBackgroundScanPeriodSec, OnBackgroundScanEnd, continue_results);
}

void EnsureBackgroundScan() {
  StartBackgroundScanAsync(true);
}

void PauseBackgroundScanForConnect() {
  scan_paused_for_connect = true;
  if (background_scan != nullptr && background_scan->isScanning()) {
    background_scan->stop();
  }
}

void ResumeBackgroundScanAfterConnect() {
  scan_paused_for_connect = false;
  StartBackgroundScanAsync(true);
}

}  // namespace

bool BleSpellCasterBegin() {
  for (int i = 0; i < kSpellScanTopDeviceCount; ++i) {
    cache[i].valid = false;
    cache[i].name[0] = '\0';
    cache[i].rssi = -127;
    cache[i].last_seen_ms = 0;
  }

  background_scan = NimBLEDevice::getScan();
  if (background_scan == nullptr) {
    return false;
  }

  if (scan_callbacks == nullptr) {
    scan_callbacks = new SpellScanCallbacks();
  }
  background_scan->setAdvertisedDeviceCallbacks(scan_callbacks, true);
  background_scan->setActiveScan(true);
  background_scan->setInterval(45);
  background_scan->setWindow(15);
  background_scan->setDuplicateFilter(false);
  background_scan->setMaxResults(0);
  background_scan->clearResults();

  StartBackgroundScanAsync(false);
  Serial.printf("Spell: background scan started (%us windows, top %d, %lu ms timeout)\n",
                  kSpellBackgroundScanPeriodSec, kSpellScanTopDeviceCount,
                  kSpellTargetCacheTimeoutMs);
  return true;
}

void BleSpellCasterLoop() {
  ExpireStaleTargets();
  EnsureBackgroundScan();
}

int BleSpellCasterGetCachedTargets(SpellCachedTargetSnapshot* out, int max_entries) {
  if ((out == nullptr) || (max_entries <= 0)) {
    return 0;
  }

  ExpireStaleTargets();
  SortCacheByRssi();

  const unsigned long now = millis();
  int written = 0;
  for (int i = 0; i < kSpellScanTopDeviceCount; ++i) {
    if (!cache[i].valid) {
      continue;
    }
    if (written >= max_entries) {
      break;
    }
    strncpy(out[written].name, cache[i].name, sizeof(out[written].name) - 1);
    out[written].name[sizeof(out[written].name) - 1] = '\0';
    out[written].rssi_dbm = cache[i].rssi;
    out[written].last_seen_age_ms = now - cache[i].last_seen_ms;
    written += 1;
  }
  return written;
}

void BleSpellCasterLogCachedTargets() {
  ExpireStaleTargets();
  SortCacheByRssi();
  LogCacheSnapshot();
}

bool BleSpellCasterTryActivate(char spell_character) {
  if ((spell_character < 32) || (spell_character > 126)) {
    return false;
  }

  ExpireStaleTargets();
  SortCacheByRssi();

  int valid_count = 0;
  for (int i = 0; i < kSpellScanTopDeviceCount; ++i) {
    if (cache[i].valid) {
      valid_count += 1;
    }
  }

  Serial.printf("Spell: trying cached targets for %c (%d in cache) ...\n", spell_character,
                  valid_count);
  if (valid_count == 0) {
    Serial.println("Spell: no cached targets");
    return false;
  }

  PauseBackgroundScanForConnect();
  bool activated = false;
  for (int i = 0; i < kSpellScanTopDeviceCount; ++i) {
    if (!cache[i].valid) {
      continue;
    }
    if (ActivateTarget(cache[i], spell_character)) {
      activated = true;
      break;
    }
  }
  ResumeBackgroundScanAfterConnect();

  if (activated) {
    return true;
  }

  Serial.printf("Spell: no match for %c among cached targets\n", spell_character);
  return false;
}
