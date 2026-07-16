#ifndef TARGET_BLE_SPELL_TARGET_H_
#define TARGET_BLE_SPELL_TARGET_H_

#include <cstdint>

typedef void (*SpellTargetActivateCallback)();

bool BleSpellTargetBegin(SpellTargetActivateCallback on_activate);
void BleSpellTargetLoop();

char BleSpellTargetGetCharacter();
bool BleSpellTargetSetCharacter(char character);

bool BleSpellTargetIsActivating();
void BleSpellTargetFinishActivation();

const char* BleSpellTargetGetDeviceName();

#endif  // TARGET_BLE_SPELL_TARGET_H_
