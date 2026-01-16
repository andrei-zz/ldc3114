#ifndef UTILS_H_
#define UTILS_H_

#include <stdint.h>

#define LDC_REG_NAME_WIDTH 15

enum PrintRegisterOpts : uint64_t {
  PREG_DEFAULT = 0,
  PREG_NO_READ = 1 << 0,  // Bit 0: read from buffer instead of making I2C request
};

void printRegister(uint8_t address, uint64_t opts = PREG_DEFAULT);
void printHexWithPadding(uint64_t n, uint8_t size);

#endif  // UTILS_H_
