#include <Arduino.h>
#include <stdint.h>

extern "C" {
  #include "ldc3114.h"
  #include "hal.h"
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.printf("init():\n");

  InitLDC();

  uint64_t manu_raw = readSingleRegister(MANUFACTURER_ID_ADDRESS, false);
  uint16_t manu = (uint16_t) (manu_raw & 0xFFFF);
  Serial.printf("  Manufacturer ID: 0x%04x\n", manu);

  uint64_t devid_raw = readSingleRegister(DEVICE_ID_ADDRESS, false);
  uint16_t devid = (uint16_t) (devid_raw & 0xFFFF);
  Serial.printf("  Device ID: 0x%04x\n", devid);
  
  uint16_t en = getRegisterValue(EN_ADDRESS);
  Serial.printf("  EN=0x%04x\n", en);

  uint16_t intpol = getRegisterValue(INTPOL_ADDRESS);
  Serial.printf("  INTPOL=0x%04x\n", intpol);

  uint16_t sens0 = getRegisterValue(SENSOR0_CONFIG_ADDRESS);
  Serial.printf("  SENSOR0_CONFIG=0x%04x\n", sens0);

  uint16_t sens1 = getRegisterValue(SENSOR1_CONFIG_ADDRESS);
  Serial.printf("  SENSOR1_CONFIG=0x%04x\n", sens1);

  uint16_t sens2 = getRegisterValue(SENSOR2_CONFIG_ADDRESS);
  Serial.printf("  SENSOR2_CONFIG=0x%04x\n", sens2);

  uint16_t sens3 = getRegisterValue(SENSOR3_CONFIG_ADDRESS);
  Serial.printf("  SENSOR3_CONFIG=0x%04x\n", sens3);

  Serial.printf("\n");
}

void loop() {
  readRawData();
  Serial.printf("loop():\n");

  uint16_t status = getRegisterValue(STATUS_ADDRESS);
  Serial.printf("  STATUS=0x%02x\n", status & 0xFF);

  uint16_t raw0 = getRegisterValue(RAW_DATA0_ADDRESS);
  Serial.printf("  RAW0=%u\n", raw0);

  uint16_t raw1 = getRegisterValue(RAW_DATA1_ADDRESS);
  Serial.printf("  RAW1=%u\n", raw1);

  uint16_t raw2 = getRegisterValue(RAW_DATA2_ADDRESS);
  Serial.printf("  RAW2=%u\n", raw2);

  uint16_t raw3 = getRegisterValue(RAW_DATA3_ADDRESS);
  Serial.printf("  RAW3=%u\n", raw3);

  Serial.printf("\n");
  delay(100);
}
