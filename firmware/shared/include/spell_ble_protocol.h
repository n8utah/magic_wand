#ifndef SPELL_BLE_PROTOCOL_H_
#define SPELL_BLE_PROTOCOL_H_

#include <cstdint>

// Spell characteristics live on the same GATT service as the wand stroke service
// so Web Bluetooth can connect with the same service filter (0000 UUID).
constexpr char kMagicWandServiceUuid[] = "4798e0f2-0000-4d68-af64-8a8f5258404e";
constexpr char kSpellServiceUuid[] = "4798e0f2-0000-4d68-af64-8a8f5258404e";
constexpr char kSpellCharacterUuid[] = "4798e0f2-4001-4d68-af64-8a8f5258404e";
constexpr char kSpellActivateUuid[] = "4798e0f2-4002-4d68-af64-8a8f5258404e";

constexpr char kSpellDeviceNamePrefix[] = "spell-";
constexpr uint8_t kSpellActivateCommand = 0x01;
constexpr unsigned long kSpellTargetFlashMs = 3000;

constexpr int kSpellScanTopDeviceCount = 3;
constexpr int kSpellActivationMinScore = 40;

#endif  // SPELL_BLE_PROTOCOL_H_
