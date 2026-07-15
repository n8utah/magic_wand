#ifndef MAGIC_WAND_BLE_STROKE_SERVICE_H_
#define MAGIC_WAND_BLE_STROKE_SERVICE_H_

#include <cstddef>
#include <cstdint>

bool BleStrokeServiceBegin();
bool BleStrokeServiceIsConnected();
void BleStrokeServiceUpdate(const uint8_t* buffer, size_t length);
void BleStrokeServiceUpdateOrientation(float x_deg, float y_deg, float z_deg);

#endif  // MAGIC_WAND_BLE_STROKE_SERVICE_H_
