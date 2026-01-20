#include <Arduino.h>

// extern "C" {
  #include "ldc3114.h"
  // #include "hal.h"
// }
#include "common.h"
#include "utils.h"

void printHexWithPadding(uint64_t n, uint8_t size) {
  Serial.print("0x");
  for (int16_t i = size - 1; i >= 0; i--) {
    uint8_t b = (n >> (8*i)) & 0xFF;
    if (b < 0x10) {
      Serial.print('0');
    }
    Serial.print(b, HEX);
  }
}

uint64_t printRegister(uint8_t address, uint64_t opts) {
  assert(address < NUM_REGISTERS);
  const char* name = ldcRegisterNames[address];
  if (!name || !name[0]) {
    return 0;
  }

  // Get raw value and update registerMap
  uint64_t raw;
  if (opts & PREG_NO_READ) {
    raw = getRegisterValue(address);
  } else {
    raw = readSingleRegister(address, false);
  }

  // Mask according to size in bytes
  uint8_t size = ldcRegisterSize[address];
  uint64_t mask;
  if (size >= 8) {
    mask = ~0ULL;
  } else {
    mask = (1ULL << (size * 8)) - 1ULL;
  }

  size_t value = raw & mask;

  #if LOG_LEVEL > 0
    Serial.print("  ");
    Serial.print(name);
    Serial.print(": ");
    printHexWithPadding(value, size);
    Serial.println();
  #endif

  return value;
}
