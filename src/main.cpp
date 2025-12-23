#include <Arduino.h>
#include <stdint.h>

extern "C" {
  #include "ldc3114.h"
  #include "hal.h"
}

#define LDC_REG_NAME_WIDTH 15

void printRegister(uint8_t);

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.printf("init():\n");

  InitLDC();

  printRegister(MANUFACTURER_ID_ADDRESS);
  printRegister(DEVICE_ID_ADDRESS);
  printRegister(STATUS_ADDRESS);
  printRegister(RESET_ADDRESS);
  printRegister(EN_ADDRESS);
  printRegister(NP_SCAN_RATE_ADDRESS);
  printRegister(LP_SCAN_RATE_ADDRESS);
  printRegister(LC_DIVIDER_ADDRESS);
  printRegister(CNTSC_ADDRESS);
  printRegister(INTPOL_ADDRESS);
  printRegister(SENSOR0_CONFIG_ADDRESS);
  printRegister(SENSOR1_CONFIG_ADDRESS);
  printRegister(SENSOR2_CONFIG_ADDRESS);
  printRegister(SENSOR3_CONFIG_ADDRESS);

  Serial.printf("\n");
}

void loop() {
  readRawData();
  Serial.printf("loop():\n");

  printRegister(STATUS_ADDRESS);
  printRegister(OUT_ADDRESS);
  printRegister(RAW_DATA0_ADDRESS);
  printRegister(RAW_DATA1_ADDRESS);
  printRegister(RAW_DATA2_ADDRESS);
  printRegister(RAW_DATA3_ADDRESS);

  Serial.printf("\n");
  delay(100);
}

void printRegister(uint8_t address) {
  assert(address < NUM_REGISTERS);
  const char *name = ldcRegisterNames[address];
  if (!name || !name[0]) {
    return;
  }

  // Get raw value and update registerMap
  uint64_t raw = readSingleRegister(address, false);

  // Mask according to size in bytes
  uint8_t size = ldcRegisterSize[address];
  uint64_t mask;
  if (size >= 8) {
    mask = ~0ULL;
  } else {
    mask = (1ULL << (size * 8)) - 1ULL;
  }

  unsigned hex_width = 2 * size;
  uint64_t value = raw & mask;
  printf(
    "  %*s: 0x%0*X\n",
    LDC_REG_NAME_WIDTH, name,
    hex_width, value
  );
}
