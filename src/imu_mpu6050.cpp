#include "imu_mpu6050.h"

#include <Wire.h>

#include <Arduino.h>
#include <math.h>

#include "config.h"

namespace {

uint8_t active_address = 0;
unsigned long last_sample_us = 0;
const unsigned long sample_interval_us =
    static_cast<unsigned long>(1000000.0f / kImuSampleRateHz);

constexpr uint8_t kRegSmplrtDiv = 0x19;
constexpr uint8_t kRegConfig = 0x1A;
constexpr uint8_t kRegGyroConfig = 0x1B;
constexpr uint8_t kRegAccelConfig = 0x1C;
constexpr uint8_t kRegAccelXoutH = 0x3B;
constexpr uint8_t kRegPwrMgmt1 = 0x6B;
constexpr uint8_t kRegWhoAmI = 0x75;

constexpr float kAccelLsbPerG = 16384.0f;       // ±2g
constexpr float kGyroLsbPerDps = 131.0f;        // ±250 dps
constexpr uint8_t kCandidateAddresses[] = {0x68, 0x69};

void EnableI2cPullups() {
  pinMode(kI2cSdaPin, INPUT_PULLUP);
  pinMode(kI2cSclPin, INPUT_PULLUP);
}

void SetupWire() {
  EnableI2cPullups();
  Wire.begin(kI2cSdaPin, kI2cSclPin);
  Wire.setClock(100000);
  Wire.setTimeOut(1000);
  delay(100);
}

bool ProbeAddress(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

bool WriteRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool ReadRegister(uint8_t address, uint8_t reg, uint8_t* value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(address, static_cast<uint8_t>(1)) != 1) {
    return false;
  }
  *value = Wire.read();
  return true;
}

bool ReadRegisters(uint8_t address, uint8_t reg, uint8_t* buffer, size_t length) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(address, static_cast<uint8_t>(length)) != length) {
    return false;
  }
  for (size_t i = 0; i < length; ++i) {
    buffer[i] = Wire.read();
  }
  return true;
}

int16_t CombineBytes(uint8_t high, uint8_t low) {
  return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
}

bool WakeAndConfigure(uint8_t address) {
  if (!WriteRegister(address, kRegPwrMgmt1, 0x00)) {
    return false;
  }
  delay(10);

  if (!WriteRegister(address, kRegSmplrtDiv, 0x09)) {
    return false;
  }
  if (!WriteRegister(address, kRegConfig, 0x03)) {
    return false;
  }
  if (!WriteRegister(address, kRegGyroConfig, 0x00)) {
    return false;
  }
  if (!WriteRegister(address, kRegAccelConfig, 0x00)) {
    return false;
  }
  delay(10);
  return true;
}

bool ReadWhoAmI(uint8_t address, uint8_t* who_am_i) {
  return ReadRegister(address, kRegWhoAmI, who_am_i);
}

bool ReadRawSample(uint8_t address, int16_t accel_raw[3], int16_t gyro_raw[3]) {
  uint8_t buffer[14];
  if (!ReadRegisters(address, kRegAccelXoutH, buffer, sizeof(buffer))) {
    return false;
  }

  accel_raw[0] = CombineBytes(buffer[0], buffer[1]);
  accel_raw[1] = CombineBytes(buffer[2], buffer[3]);
  accel_raw[2] = CombineBytes(buffer[4], buffer[5]);
  gyro_raw[0] = CombineBytes(buffer[8], buffer[9]);
  gyro_raw[1] = CombineBytes(buffer[10], buffer[11]);
  gyro_raw[2] = CombineBytes(buffer[12], buffer[13]);
  return true;
}

bool SampleLooksSane(uint8_t address) {
  int16_t accel_raw[3];
  int16_t gyro_raw[3];
  if (!ReadRawSample(address, accel_raw, gyro_raw)) {
    return false;
  }

  const float ax = accel_raw[0] / kAccelLsbPerG;
  const float ay = accel_raw[1] / kAccelLsbPerG;
  const float az = accel_raw[2] / kAccelLsbPerG;
  const float magnitude = sqrtf((ax * ax) + (ay * ay) + (az * az));

  // Stationary board should read roughly 1g; allow clones with offset/noise.
  return (magnitude > 0.5f) && (magnitude < 2.0f);
}

}  // namespace

void ImuScanI2cBus() {
  SetupWire();

  Serial.printf("Scanning I2C bus (SDA=GPIO%d, SCL=GPIO%d)...\n", kI2cSdaPin,
                kI2cSclPin);

  int found = 0;
  for (uint8_t address = 1; address < 127; ++address) {
    if (!ProbeAddress(address)) {
      continue;
    }

    Serial.printf("  found device at 0x%02X\n", address);
    found += 1;
  }

  if (found == 0) {
    Serial.println("  no I2C devices found");
  }
}

void ImuPrintDiagnostics() {
  SetupWire();
  ImuScanI2cBus();

  Serial.println("MPU6050-compatible probe:");
  for (uint8_t address : kCandidateAddresses) {
    const bool ack = ProbeAddress(address);
    Serial.printf("  0x%02X ACK=%s", address, ack ? "yes" : "no");

    if (!ack) {
      Serial.println();
      continue;
    }

    uint8_t who_am_i = 0;
    if (ReadWhoAmI(address, &who_am_i)) {
      Serial.printf(" WHO_AM_I=0x%02X", who_am_i);
      if (who_am_i == 0x68) {
        Serial.print(" (genuine MPU6050)");
      } else {
        Serial.print(" (clone/compatible chip — supported)");
      }
    } else {
      Serial.print(" WHO_AM_I read failed");
    }

    if (WakeAndConfigure(address)) {
      Serial.printf(" init=%s", SampleLooksSane(address) ? "ok" : "bad data");
    } else {
      Serial.print(" init=failed");
    }
    Serial.println();
  }

  Serial.println();
  Serial.println("If no devices were found, check:");
  Serial.println("  - VCC -> 3.3V (not 5V on ESP32 I2C pins)");
  Serial.println("  - GND common between ESP32 and MPU6050");
  Serial.println("  - SDA -> D21 (GPIO21), SCL -> D22 (GPIO22)");
  Serial.println("  - AD0 pin: GND=0x68, 3.3V=0x69");
}

uint8_t ImuActiveAddress() { return active_address; }

bool ImuBegin() {
  active_address = 0;
  SetupWire();

  for (uint8_t address : kCandidateAddresses) {
    if (!ProbeAddress(address)) {
      continue;
    }

    uint8_t who_am_i = 0;
    if (ReadWhoAmI(address, &who_am_i)) {
      if (who_am_i == 0x68) {
        Serial.printf("Detected MPU6050 at 0x%02X (WHO_AM_I=0x68)\n", address);
      } else {
        Serial.printf(
            "Detected MPU6050-compatible chip at 0x%02X (WHO_AM_I=0x%02X)\n",
            address, who_am_i);
      }
    }

    if (!WakeAndConfigure(address)) {
      continue;
    }

    if (!SampleLooksSane(address)) {
      Serial.printf("WARN: readings from 0x%02X look invalid\n", address);
      continue;
    }

    active_address = address;
    last_sample_us = micros();
    Serial.printf("IMU ready at I2C address 0x%02X\n", active_address);
    return true;
  }

  return false;
}

float ImuAccelerationSampleRate() { return kImuSampleRateHz; }

float ImuGyroscopeSampleRate() { return kImuSampleRateHz; }

bool ImuSampleReady() {
  const unsigned long now = micros();
  if ((now - last_sample_us) >= sample_interval_us) {
    last_sample_us += sample_interval_us;
    return true;
  }
  return false;
}

bool ImuReadSample(float accel_g[3], float gyro_dps[3]) {
  if (active_address == 0) {
    return false;
  }

  int16_t accel_raw[3];
  int16_t gyro_raw[3];
  if (!ReadRawSample(active_address, accel_raw, gyro_raw)) {
    return false;
  }

  accel_g[0] = accel_raw[0] / kAccelLsbPerG;
  accel_g[1] = accel_raw[1] / kAccelLsbPerG;
  accel_g[2] = accel_raw[2] / kAccelLsbPerG;

  gyro_dps[0] = gyro_raw[0] / kGyroLsbPerDps;
  gyro_dps[1] = gyro_raw[1] / kGyroLsbPerDps;
  gyro_dps[2] = gyro_raw[2] / kGyroLsbPerDps;

  return true;
}
