#ifndef MAGIC_WAND_BLE_SPELL_CASTER_H_
#define MAGIC_WAND_BLE_SPELL_CASTER_H_

#include <cstdint>

struct SpellCachedTargetSnapshot {
  char name[24];
  int rssi_dbm;
  unsigned long last_seen_age_ms;
};

bool BleSpellCasterBegin();
void BleSpellCasterLoop();

// Fills out[] with valid cache entries sorted strongest RSSI first. Returns count written.
int BleSpellCasterGetCachedTargets(SpellCachedTargetSnapshot* out, int max_entries);

// Print nearby spell targets by RSSI (strongest first).
void BleSpellCasterLogCachedTargets();

// Try cached spell targets (strongest RSSI first) for a matching character.
bool BleSpellCasterTryActivate(char spell_character);

#endif  // MAGIC_WAND_BLE_SPELL_CASTER_H_
