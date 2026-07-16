#include "spell_character.h"

#include <Preferences.h>

#include <Arduino.h>
#include <esp_random.h>

namespace {

constexpr char kPrefsNamespace[] = "spell";
constexpr char kPrefsCharacterKey[] = "char";
constexpr char kRandomCharacterPool[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";

Preferences preferences;
char stored_character = '?';
bool loaded = false;

char GenerateRandomCharacter() {
  const int pool_length = static_cast<int>(sizeof(kRandomCharacterPool)) - 1;
  const int index = random(0, pool_length);
  return kRandomCharacterPool[index];
}

}  // namespace

void SpellCharacterBegin() {
  randomSeed(esp_random());
  preferences.begin(kPrefsNamespace, false);
  loaded = true;

  if (preferences.isKey(kPrefsCharacterKey)) {
    stored_character = static_cast<char>(preferences.getUChar(kPrefsCharacterKey, '?'));
  } else {
    stored_character = GenerateRandomCharacter();
    preferences.putUChar(kPrefsCharacterKey, static_cast<uint8_t>(stored_character));
    Serial.printf("Generated spell character: %c\n", stored_character);
  }
}

void SpellCharacterEnsureSaved() {
  if (!loaded) {
    SpellCharacterBegin();
  }
}

char SpellCharacterGet() {
  SpellCharacterEnsureSaved();
  return stored_character;
}

bool SpellCharacterSet(char character) {
  if ((character < 32) || (character > 126)) {
    return false;
  }

  SpellCharacterEnsureSaved();
  stored_character = character;
  preferences.putUChar(kPrefsCharacterKey, static_cast<uint8_t>(stored_character));
  return true;
}
