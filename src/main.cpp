/**
 * https://www.ti.com/lit/ds/symlink/ldc3114.pdf
 */

#include <Arduino.h>
#include <stdint.h>

#include "ldc3114.h"
#include "hal.h"
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

    Serial.println();
  #endif
}

void loop() {
  static size_t n_iteration;

  readRawData();

  #if LOG_LEVEL > 0
    Serial.print("loop(): ");
    Serial.print(n_iteration++);
    Serial.println();

    uint64_t status = printRegister(STATUS_ADDRESS, PREG_NO_READ);
    #if LOG_LEVEL > 1
      if (status & STATUS_CHIP_READY_MASK) {
        Serial.println("    STATUS_CHIP_READY_Chipreadyafterinternalreset");
      } else {
        Serial.println("    STATUS_CHIP_READY_Chipnotreadyafterinternalreset");
      }
      if (status & STATUS_RDY_TO_WRITE_MASK) {
        Serial.println("    STATUS_RDY_TO_WRITE_Registersready");
      } else {
        Serial.println("    STATUS_RDY_TO_WRITE_Registersnotready");
      }
      if (status & STATUS_MAXOUT_MASK) {
        Serial.println("    STATUS_MAXOUT_Maximumoutputcode");
      } else {
        Serial.println("    STATUS_MAXOUT_Nomaximumoutputcode");
      }
      if (status & STATUS_FSM_WD_MASK) {
        Serial.println("    STATUS_FSM_WD_Errorinfinitestatemachine");
      } else {
        Serial.println("    STATUS_FSM_WD_Noerrorinfinitestatemachine");
      }
      if (status & STATUS_LC_WD_MASK) {
        Serial.println("    STATUS_LC_WD_ErrorinLCoscillatorinitialization");
      } else {
        Serial.println("    STATUS_LC_WD_NoerrorinLCoscillatorinitialization");
      }
      if (status & STATUS_TIMEOUT_MASK) {
        Serial.println("    STATUS_TIMEOUT_timeouterror");
      } else {
        Serial.println("    STATUS_TIMEOUT_notimeouterror");
      }
      if (status & STATUS_REGISTER_FLAG_MASK) {
        Serial.println("    STATUS_REGISTER_FLAG_Unexpectedregisterchange");
      } else {
        Serial.println("    STATUS_REGISTER_FLAG_Nounexpectedregisterchange");
      }
    #endif

    // printRegister(OUT_ADDRESS, PREG_NO_READ);
    printRegister(RAW_DATA0_ADDRESS, PREG_NO_READ);
    printRegister(RAW_DATA1_ADDRESS, PREG_NO_READ);
    printRegister(RAW_DATA2_ADDRESS, PREG_NO_READ);
    printRegister(RAW_DATA3_ADDRESS, PREG_NO_READ);

    Serial.println();
  #endif

  delay(200);
}
