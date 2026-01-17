/**
 * https://www.ti.com/lit/ds/symlink/ldc3114.pdf
 */

#include <Arduino.h>
#include <stdint.h>

// extern "C" {
  #include "ldc3114.h"
  #include "hal.h"
// }
#include "common.h"
#include "utils.h"

void setup() {
  Serial.begin(115200);
  #if WAIT_FOR_SERIAL
    delay(5000);
    while (!Serial);
  #endif

  #if LOG_LEVEL > 0
    Serial.println("setup():");
  #endif
  InitLDC();

  #if LOG_LEVEL > 0
    // printRegister(MANUFACTURER_ID_ADDRESS);
    // printRegister(DEVICE_ID_ADDRESS);
    // printRegister(STATUS_ADDRESS);
    // printRegister(RESET_ADDRESS);
    // printRegister(EN_ADDRESS);
    // printRegister(NP_SCAN_RATE_ADDRESS);
    // printRegister(LP_SCAN_RATE_ADDRESS);
    // printRegister(LC_DIVIDER_ADDRESS);
    // printRegister(CNTSC_ADDRESS);
    // printRegister(INTPOL_ADDRESS);
    // printRegister(SENSOR0_CONFIG_ADDRESS);
    // printRegister(SENSOR1_CONFIG_ADDRESS);
    // printRegister(SENSOR2_CONFIG_ADDRESS);
    // printRegister(SENSOR3_CONFIG_ADDRESS);

    Serial.println();
  #endif
}

void loop() {
  static uint8_t ready = 0;
  // readRawData();

  #if LOG_LEVEL > 0
    Serial.println("loop():");

    // uint64_t status = printRegister(STATUS_ADDRESS, PREG_NO_READ);
    // // printRegister(OUT_ADDRESS, PREG_NO_READ);
    // printRegister(RAW_DATA0_ADDRESS, PREG_NO_READ);
    // printRegister(RAW_DATA1_ADDRESS, PREG_NO_READ);
    // printRegister(RAW_DATA2_ADDRESS, PREG_NO_READ);
    // printRegister(RAW_DATA3_ADDRESS, PREG_NO_READ);

    // Serial.println();
  #endif

  ready = readSingleRegister(STATUS_ADDRESS, false) & STATUS_CHIP_READY_MASK;
  if (ready == STATUS_CHIP_READY_MASK) {
    // set the FULL_RESET bit of the RESET register
    Serial.println("  Ready, resetting...");
    writeSingleRegister(RESET_ADDRESS, RESET_FULL_RESET_MASK);
  } else {
    Serial.println("  Not ready.");
    ready = readSingleRegister(STATUS_ADDRESS, false) & STATUS_CHIP_READY_MASK;
  }

  // check for the STATUS register to signal the device is ready after reset
  
  // poll the status register until the device is ready after internal reset
  while (ready != STATUS_CHIP_READY_MASK) {
    
  }
  Serial.println("  Ready!");

  delay(100);
}
