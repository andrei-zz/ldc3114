/**
 * https://www.ti.com/lit/ds/symlink/ldc3114.pdf
 */

#include <Arduino.h>
#include <stdint.h>

extern "C" {
  #include "ldc3114.h"
  #include "hal.h"
}
#include "common.h"
#include "utils.h"

void setup() {
  Serial.begin(115200);
  #if WAIT_FOR_SERIAL
    while (!Serial);
  #endif

  #if LOG_LEVEL > 0
    Serial.printf("init():\n");
  #endif
  InitLDC();

  #if LOG_LEVEL > 0
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
  #endif
}

void loop() {
  readRawData();

  #if LOG_LEVEL > 0
    Serial.printf("loop():\n");

    printRegister(STATUS_ADDRESS, PREG_NO_READ);
    printRegister(OUT_ADDRESS, PREG_NO_READ);
    printRegister(RAW_DATA0_ADDRESS, PREG_NO_READ);
    printRegister(RAW_DATA1_ADDRESS, PREG_NO_READ);
    printRegister(RAW_DATA2_ADDRESS, PREG_NO_READ);
    printRegister(RAW_DATA3_ADDRESS, PREG_NO_READ);

    Serial.printf("\n");
  #endif

  delay(100);
}
