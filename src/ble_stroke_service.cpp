#include "ble_stroke_service.h"

#include <NimBLEDevice.h>

#include <Arduino.h>

namespace {

constexpr char kServiceUuid[] = "4798e0f2-0000-4d68-af64-8a8f5258404e";
constexpr char kStrokeCharacteristicUuid[] =
    "4798e0f2-300a-4d68-af64-8a8f5258404e";

NimBLECharacteristic* stroke_characteristic = nullptr;
bool connected = false;

class BleServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server) override {
    connected = true;
    Serial.println("BLE: central connected");
  }

  void onDisconnect(NimBLEServer* server) override {
    connected = false;
    Serial.println("BLE: central disconnected");
    NimBLEDevice::startAdvertising();
  }
};

String MakeDeviceName() {
  const std::string address = NimBLEDevice::getAddress().toString();
  String name = "BLESense-";
  if (address.length() >= 5) {
    name += static_cast<char>(toupper(address[address.length() - 5]));
    name += static_cast<char>(toupper(address[address.length() - 4]));
    name += static_cast<char>(toupper(address[address.length() - 2]));
    name += static_cast<char>(toupper(address[address.length() - 1]));
  }
  return name;
}

}  // namespace

bool BleStrokeServiceBegin() {
  NimBLEDevice::init("BLESense");

  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(new BleServerCallbacks());

  NimBLEService* service = server->createService(kServiceUuid);
  stroke_characteristic = service->createCharacteristic(
      kStrokeCharacteristicUuid, NIMBLE_PROPERTY::READ);
  stroke_characteristic->setValue(
      reinterpret_cast<uint8_t*>(const_cast<char*>("")),
      0);

  service->start();

  const String device_name = MakeDeviceName();
  NimBLEDevice::setDeviceName(device_name.c_str());

  NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
  advertising->addServiceUUID(kServiceUuid);
  advertising->setScanResponse(true);
  advertising->start();

  Serial.print("BLE advertising as ");
  Serial.println(device_name);
  return true;
}

bool BleStrokeServiceIsConnected() { return connected; }

void BleStrokeServiceUpdate(const uint8_t* buffer, size_t length) {
  if (!connected || stroke_characteristic == nullptr) {
    return;
  }
  stroke_characteristic->setValue(const_cast<uint8_t*>(buffer), length);
}
