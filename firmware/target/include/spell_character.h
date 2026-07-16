#ifndef TARGET_SPELL_CHARACTER_H_
#define TARGET_SPELL_CHARACTER_H_

#include <cstdint>

void SpellCharacterBegin();
char SpellCharacterGet();
bool SpellCharacterSet(char character);
void SpellCharacterEnsureSaved();

#endif  // TARGET_SPELL_CHARACTER_H_
