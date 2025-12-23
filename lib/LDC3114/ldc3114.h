/**
 * \brief This header file contains all register map definitions for the LDC3114 device.
 *
 * \copyright Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef LDC3114_H_
#define LDC3114_H_

// Standard libraries
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>

// Custom libraries
#include "hal.h"


//****************************************************************************
//
// Select the device variant to use...
//
//****************************************************************************

#define CHANNEL_COUNT (4)   // LDC3114 -> 4 Channels
#define WORD_LENGTH_24BIT

/* Disable assertions when not in the CCS "Debug" configuration */
#ifndef _DEBUG
    #define NDEBUG
#endif


//****************************************************************************
//
// Constants
//
//****************************************************************************

#define NUM_REGISTERS           ((uint16_t) 256)

#ifdef EXAMPLE_CODE
#else



//****************************************************************************
//
// Global variables
//
//****************************************************************************

// Register names, used by EVM firmware
extern const char *ldcRegisterNames[];
extern const uint8_t ldcRegisterSize[];

#endif  // EXAMPLE_CODE


//****************************************************************************
//
// SPI command opcodes
//
//****************************************************************************

#define OPCODE_NULL                             ((uint16_t) 0x0000)
#define OPCODE_RESET                            ((uint16_t) 0x0011)
#define OPCODE_RREG                             ((uint16_t) 0xA000)
#define OPCODE_WREG                             ((uint16_t) 0x6000)
#define OPCODE_STANDBY                          ((uint16_t) 0x0022)
#define OPCODE_WAKEUP                           ((uint16_t) 0x0033)
#define OPCODE_LOCK                             ((uint16_t) 0x0555)
#define OPCODE_UNLOCK                           ((uint16_t) 0x0655)

//****************************************************************************
//
// Channel data structure
//
//****************************************************************************

typedef struct
{
    uint16_t response;
    uint16_t crc;
    int32_t channel0;

#if (CHANNEL_COUNT > 1)
    int32_t channel1;
#endif
#if (CHANNEL_COUNT > 2)
    int32_t channel2;
#endif
#if (CHANNEL_COUNT > 3)
    int32_t channel3;
#endif
#if (CHANNEL_COUNT > 4)
    int32_t channel4;
#endif
#if (CHANNEL_COUNT > 5)
    int32_t channel5;
#endif
#if (CHANNEL_COUNT > 6)
    int32_t channel6;
 #endif
 #if (CHANNEL_COUNT > 7)
    int32_t channel7;
#endif
} adc_channel_data;

//**********************************************************************************
//
// Function prototypes
//
//**********************************************************************************

void        ldcStartup(void);
uint64_t    readSingleRegister(uint8_t address, bool useBULK);
void        writeSingleRegister(uint8_t address, uint8_t data);
void        configureDevice(void);
void        configMode(void);
void        activeMode(void);
void        resetDevice(void);
void        restoreRegisterDefaults(void);
void        readButtonData(void);
void        readRawData(void);
void        readAllRegisters(void);
void        LPMode(void);
void        NPMode(void);
uint8_t     OUTxGPIO(void);

void        startTimer(uint32_t timerPeriod);
void        stopTimer(void);

// Getter functions
uint16_t    getRegisterValue(uint8_t address);
//void        getEVMID(void);

// Helper functions
uint8_t     upperByte(uint16_t uint16_Word);
uint8_t     lowerByte(uint16_t uint16_Word);
uint16_t    combineBytes(uint8_t upperByte, uint8_t lowerByte);




/* Register 0x00 (STATUS) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |   OUT_STATUS  |   CHIP_READY  |  RDY_TO_WRITE |     MAXOUT    |     FSM_WD    |     LC_WD     |    TIMEOUT    | REGISTER_FLAG |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STATUS register address */
    #define STATUS_ADDRESS                                                  ((uint8_t) 0x00)

    /* STATUS default (reset) value */
    #define STATUS_DEFAULT                                                  ((uint8_t) 0x40)

    /* STATUS register field masks */
    #define STATUS_OUT_STATUS_MASK                                          ((uint8_t) 0x80)
    #define STATUS_CHIP_READY_MASK                                          ((uint8_t) 0x40)
    #define STATUS_RDY_TO_WRITE_MASK                                        ((uint8_t) 0x20)
    #define STATUS_MAXOUT_MASK                                              ((uint8_t) 0x10)
    #define STATUS_FSM_WD_MASK                                              ((uint8_t) 0x08)
    #define STATUS_LC_WD_MASK                                               ((uint8_t) 0x04)
    #define STATUS_TIMEOUT_MASK                                             ((uint8_t) 0x02)
    #define STATUS_REGISTER_FLAG_MASK                                       ((uint8_t) 0x01)

    /* CHIP_READY field values */
    #define STATUS_CHIP_READY_Chipnotreadyafterinternalreset                ((uint8_t) 0x00)
    #define STATUS_CHIP_READY_Chipreadyafterinternalreset                   ((uint8_t) 0x40)

    /* RDY_TO_WRITE field values */
    #define STATUS_RDY_TO_WRITE_Registersnotready                           ((uint8_t) 0x00)
    #define STATUS_RDY_TO_WRITE_Registersready                              ((uint8_t) 0x20)

    /* MAXOUT field values */
    #define STATUS_MAXOUT_Nomaximumoutputcode                               ((uint8_t) 0x00)
    #define STATUS_MAXOUT_Maximumoutputcode                                 ((uint8_t) 0x10)

    /* FSM_WD field values */
    #define STATUS_FSM_WD_Noerrorinfinitestatemachine                       ((uint8_t) 0x00)
    #define STATUS_FSM_WD_Errorinfinitestatemachine                         ((uint8_t) 0x08)

    /* LC_WD field values */
    #define STATUS_LC_WD_NoerrorinLCoscillatorinitialization                ((uint8_t) 0x00)
    #define STATUS_LC_WD_ErrorinLCoscillatorinitialization                  ((uint8_t) 0x04)

    /* TIMEOUT field values */
    #define STATUS_TIMEOUT_notimeouterror                                   ((uint8_t) 0x00)
    #define STATUS_TIMEOUT_timeouterror                                     ((uint8_t) 0x02)

    /* REGISTER_FLAG field values */
    #define STATUS_REGISTER_FLAG_Nounexpectedregisterchange                 ((uint8_t) 0x00)
    #define STATUS_REGISTER_FLAG_Unexpectedregisterchange                   ((uint8_t) 0x01)



/* Register 0x01 (OUT) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |       0       |       0       |       0       |    DATA_RDY   |      OUT3     |      OUT2     |      OUT1     |      OUT0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* OUT register address */
    #define OUT_ADDRESS                                                     ((uint8_t) 0x01)

    /* OUT default (reset) value */
    #define OUT_DEFAULT                                                     ((uint8_t) 0x00)

    /* OUT register field masks */
    #define OUT_DATA_RDY_MASK                                               ((uint8_t) 0x10)
    #define OUT_OUT3_MASK                                                   ((uint8_t) 0x08)
    #define OUT_OUT2_MASK                                                   ((uint8_t) 0x04)
    #define OUT_OUT1_MASK                                                   ((uint8_t) 0x02)
    #define OUT_OUT0_MASK                                                   ((uint8_t) 0x01)

    /* DATA_RDY field values */
    #define OUT_DATA_RDY_DataCaptureinprogress                              ((uint8_t) 0x00)
    #define OUT_DATA_RDY_NewDataavailable                                   ((uint8_t) 0x10)

    /* OUT3 field values */
    #define OUT_OUT3_NobuttonpressdetectedonChannel3                        ((uint8_t) 0x00)
    #define OUT_OUT3_ButtonpressdetectedonChannel3                          ((uint8_t) 0x08)

    /* OUT2 field values */
    #define OUT_OUT2_NobuttonpressdetectedonChannel2                        ((uint8_t) 0x00)
    #define OUT_OUT2_ButtonpressdetectedonChannel2                          ((uint8_t) 0x04)

    /* OUT1 field values */
    #define OUT_OUT1_NobuttonpressdetectedonChannel1                        ((uint8_t) 0x00)
    #define OUT_OUT1_ButtonpressdetectedonChannel1                          ((uint8_t) 0x02)

    /* OUT0 field values */
    #define OUT_OUT0_NobuttonpressdetectedonChannel0                        ((uint8_t) 0x00)
    #define OUT_OUT0_ButtonpressdetectedonChannel0                          ((uint8_t) 0x01)



/* Register 0x02 (DATA0) definition
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 15    |     Bit 14    |     Bit 13    |     Bit 12    |     Bit 11    |     Bit 10    |     Bit 9     |     Bit 8     |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       0       |       0       |       0       |       0       |                        DATA0_11_8[3:0]                        |                                                         DATA0_7_0[7:0]                                                        |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DATA0 register address */
    #define DATA0_ADDRESS                                                   ((uint8_t) 0x02)

    /* DATA0 default (reset) value */
    #define DATA0_DEFAULT                                                   ((uint16_t) 0x0000)

    /* DATA0 register field masks */
    #define DATA0_DATA0_11_8_MASK                                           ((uint16_t) 0x0F00)
    #define DATA0_DATA0_7_0_MASK                                            ((uint16_t) 0x00FF)

    /* RESERVED0 field values */
    #define DATA0_RESERVED0_Defaultvalue                                    ((uint16_t) 0x0000)

    /* DATA0_11_8 field values */
    #define DATA0_DATA0_11_8_Defaultvalue                                   ((uint16_t) 0x0000)

    /* DATA0_7_0 field values */
    #define DATA0_DATA0_7_0_Defaultvalue                                    ((uint16_t) 0x0000)



/* Register 0x04 (DATA1) definition
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 15    |     Bit 14    |     Bit 13    |     Bit 12    |     Bit 11    |     Bit 10    |     Bit 9     |     Bit 8     |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       0       |       0       |       0       |       0       |                        DATA1_11_8[3:0]                        |                                                         DATA1_7_0[7:0]                                                        |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DATA1 register address */
    #define DATA1_ADDRESS                                                   ((uint8_t) 0x04)

    /* DATA1 default (reset) value */
    #define DATA1_DEFAULT                                                   ((uint16_t) 0x0000)

    /* DATA1 register field masks */
    #define DATA1_DATA1_11_8_MASK                                           ((uint16_t) 0x0F00)
    #define DATA1_DATA1_7_0_MASK                                            ((uint16_t) 0x00FF)

    /* RESERVED0 field values */
    #define DATA1_RESERVED0_Defaultvalue                                    ((uint16_t) 0x0000)

    /* DATA1_11_8 field values */
    #define DATA1_DATA1_11_8_Defaultvalue                                   ((uint16_t) 0x0000)

    /* DATA1_7_0 field values */
    #define DATA1_DATA1_7_0_Defaultvalue                                    ((uint16_t) 0x0000)



/* Register 0x06 (DATA2) definition
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 15    |     Bit 14    |     Bit 13    |     Bit 12    |     Bit 11    |     Bit 10    |     Bit 9     |     Bit 8     |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       0       |       0       |       0       |       0       |                        DATA2_11_8[3:0]                        |                                                         DATA2_7_0[7:0]                                                        |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DATA2 register address */
    #define DATA2_ADDRESS                                                   ((uint8_t) 0x06)

    /* DATA2 default (reset) value */
    #define DATA2_DEFAULT                                                   ((uint16_t) 0x0000)

    /* DATA2 register field masks */
    #define DATA2_DATA2_11_8_MASK                                           ((uint16_t) 0x0F00)
    #define DATA2_DATA2_7_0_MASK                                            ((uint16_t) 0x00FF)

    /* RESERVED0 field values */
    #define DATA2_RESERVED0_Defaultvalue                                    ((uint16_t) 0x0000)

    /* DATA2_11_8 field values */
    #define DATA2_DATA2_11_8_Defaultvalue                                   ((uint16_t) 0x0000)

    /* DATA2_7_0 field values */
    #define DATA2_DATA2_7_0_Defaultvalue                                    ((uint16_t) 0x0000)



/* Register 0x08 (DATA3) definition
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 15    |     Bit 14    |     Bit 13    |     Bit 12    |     Bit 11    |     Bit 10    |     Bit 9     |     Bit 8     |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       0       |       0       |       0       |       0       |                        DATA3_11_8[3:0]                        |                                                         DATA3_7_0[7:0]                                                        |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DATA3 register address */
    #define DATA3_ADDRESS                                                   ((uint8_t) 0x08)

    /* DATA3 default (reset) value */
    #define DATA3_DEFAULT                                                   ((uint16_t) 0x0000)

    /* DATA3 register field masks */
    #define DATA3_DATA3_11_8_MASK                                           ((uint16_t) 0x0F00)
    #define DATA3_DATA3_7_0_MASK                                            ((uint16_t) 0x00FF)

    /* RESERVED0 field values */
    #define DATA3_RESERVED0_Defaultvalue                                    ((uint16_t) 0x0000)

    /* DATA3_11_8 field values */
    #define DATA3_DATA3_11_8_Defaultvalue                                   ((uint16_t) 0x0000)

    /* DATA3_7_0 field values */
    #define DATA3_DATA3_7_0_Defaultvalue                                    ((uint16_t) 0x0000)



/* Register 0x0A (RESET) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |       0       |       0       |       0       |   FULL_RESET  |       0       |       0       |       0       |  CONFIG_MODE  |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* RESET register address */
    #define RESET_ADDRESS                                                   ((uint8_t) 0x0A)

    /* RESET default (reset) value */
    #define RESET_DEFAULT                                                   ((uint8_t) 0x00)

    /* RESET register field masks */
    #define RESET_FULL_RESET_MASK                                           ((uint8_t) 0x10)
    #define RESET_CONFIG_MODE_MASK                                          ((uint8_t) 0x01)

    /* RESERVED0 field values */
    #define RESET_RESERVED0_Defaultvalue                                    ((uint8_t) 0x00)

    /* FULL_RESET field values */
    #define RESET_FULL_RESET_Normaloperation                                ((uint8_t) 0x00)
    #define RESET_FULL_RESET_ResetsthedeviceandregisterconfigurationsAllregisterswillbereturnedtodefaultvaluesNormaloperationwillnotresumeuntilSTATUSCHIP_READY1    ((uint8_t) 0x10)

    /* RESERVED1 field values */
    #define RESET_RESERVED1_Defaultvalue                                    ((uint8_t) 0x00)

    /* CONFIG_MODE field values */
    #define RESET_CONFIG_MODE_Normaloperation                               ((uint8_t) 0x00)
    #define RESET_CONFIG_MODE_Holdsthedeviceinconfigurationmodebutmaintainscurrentregisterconfigurations    ((uint8_t) 0x01)



/* Register 0x0C (EN) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     LPEN3     |     LPEN2     |     LPEN1     |     LPEN0     |      EN3      |      EN2      |      EN1      |      EN0      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* EN register address */
    #define EN_ADDRESS                                                      ((uint8_t) 0x0C)

    /* EN default (reset) value */
    #define EN_DEFAULT                                                      ((uint8_t) 0x0F)

    /* EN register field masks */
    #define EN_LPEN3_MASK                                                   ((uint8_t) 0x80)
    #define EN_LPEN2_MASK                                                   ((uint8_t) 0x40)
    #define EN_LPEN1_MASK                                                   ((uint8_t) 0x20)
    #define EN_LPEN0_MASK                                                   ((uint8_t) 0x10)
    #define EN_EN3_MASK                                                     ((uint8_t) 0x08)
    #define EN_EN2_MASK                                                     ((uint8_t) 0x04)
    #define EN_EN1_MASK                                                     ((uint8_t) 0x02)
    #define EN_EN0_MASK                                                     ((uint8_t) 0x01)
    #define EN_ALL_MASK                                                     ((uint8_t) 0xFF)

    /* LPEN3 field values */
    #define EN_LPEN3_DisableChannel3lowpowermode                            ((uint8_t) 0x00)
    #define EN_LPEN3_EnableChannel3lowpowermode                             ((uint8_t) 0x80)

    /* LPEN2 field values */
    #define EN_LPEN2_DisableChannel2lowpowermode                            ((uint8_t) 0x00)
    #define EN_LPEN2_EnableChannel2lowpowermode                             ((uint8_t) 0x40)

    /* LPEN1 field values */
    #define EN_LPEN1_DisableChannel1lowpowermode                            ((uint8_t) 0x00)
    #define EN_LPEN1_EnableChannel1lowpowermode                             ((uint8_t) 0x20)

    /* LPEN0 field values */
    #define EN_LPEN0_DisableChannel0lowpowermode                            ((uint8_t) 0x00)
    #define EN_LPEN0_EnableChannel0lowpowermode                             ((uint8_t) 0x10)

    /* EN3 field values */
    #define EN_EN3_DisableChannel3                                          ((uint8_t) 0x00)
    #define EN_EN3_EnableChannel3                                           ((uint8_t) 0x08)

    /* EN2 field values */
    #define EN_EN2_DisableChannel2                                          ((uint8_t) 0x00)
    #define EN_EN2_EnableChannel2                                           ((uint8_t) 0x04)

    /* EN1 field values */
    #define EN_EN1_DisableChannel1                                          ((uint8_t) 0x00)
    #define EN_EN1_EnableChannel1                                           ((uint8_t) 0x02)

    /* EN0 field values */
    #define EN_EN0_DisableChannel0                                          ((uint8_t) 0x00)
    #define EN_EN0_EnableChannel0                                           ((uint8_t) 0x01)



/* Register 0x0D (NP_SCAN_RATE) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |       0       |       0       |       0       |       0       |     NPFSR     |      NPCS     |           NPSR[1:0]           |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* NP_SCAN_RATE register address */
    #define NP_SCAN_RATE_ADDRESS                                            ((uint8_t) 0x0D)

    /* NP_SCAN_RATE default (reset) value */
    #define NP_SCAN_RATE_DEFAULT                                            ((uint8_t) 0x01)

    /* NP_SCAN_RATE register field masks */
    #define NP_SCAN_RATE_NPFSR_MASK                                         ((uint8_t) 0x08)
    #define NP_SCAN_RATE_NPCS_MASK                                          ((uint8_t) 0x04)
    #define NP_SCAN_RATE_NPSR_MASK                                          ((uint8_t) 0x03)

    /* NPFSR field values */
    #define NP_SCAN_RATE_NPFSR_FastScanRateDisabled                         ((uint8_t) 0x00)
    #define NP_SCAN_RATE_NPFSR_FastScanRateEnabled                          ((uint8_t) 0x08)

    /* NPCS field values */
    #define NP_SCAN_RATE_NPCS_ContinuousScanDisabled                        ((uint8_t) 0x00)
    #define NP_SCAN_RATE_NPCS_ContinuousScanEnabled                         ((uint8_t) 0x04)

    /* NPSR field values */
    #define NP_SCAN_RATE_NPSR_80SPS                                         ((uint8_t) 0x00)
    #define NP_SCAN_RATE_NPSR_40SPS                                         ((uint8_t) 0x01)
    #define NP_SCAN_RATE_NPSR_20SPS                                         ((uint8_t) 0x02)
    #define NP_SCAN_RATE_NPSR_10SPS                                         ((uint8_t) 0x03)



/* Register 0x0E (GAIN0) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |       0       |       0       |                                           GAIN0[5:0]                                          |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GAIN0 register address */
    #define GAIN0_ADDRESS                                                   ((uint8_t) 0x0E)

    /* GAIN0 default (reset) value */
    #define GAIN0_DEFAULT                                                   ((uint8_t) 0x28)

    /* GAIN0 register field masks */
    #define GAIN0_GAIN0_MASK                                                ((uint8_t) 0x3F)



/* Register 0x0F (LP_SCAN_RATE) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |       0       |       0       |       0       |       0       |       0       |       0       |           LPSR[1:0]           |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* LP_SCAN_RATE register address */
    #define LP_SCAN_RATE_ADDRESS                                            ((uint8_t) 0x0F)

    /* LP_SCAN_RATE default (reset) value */
    #define LP_SCAN_RATE_DEFAULT                                            ((uint8_t) 0x02)

    /* LP_SCAN_RATE register field masks */
    #define LP_SCAN_RATE_LPSR_MASK                                          ((uint8_t) 0x03)

    /* LPSR field values */
    #define LP_SCAN_RATE_LPSR_5SPS                                          ((uint8_t) 0x00)
    #define LP_SCAN_RATE_LPSR_25SPS                                         ((uint8_t) 0x01)
    #define LP_SCAN_RATE_LPSR_125SPS                                        ((uint8_t) 0x02)
    #define LP_SCAN_RATE_LPSR_0625SPS                                       ((uint8_t) 0x03)



/* Register 0x10 (GAIN1) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |       0       |       0       |                                           GAIN1[5:0]                                          |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GAIN1 register address */
    #define GAIN1_ADDRESS                                                   ((uint8_t) 0x10)

    /* GAIN1 default (reset) value */
    #define GAIN1_DEFAULT                                                   ((uint8_t) 0x28)

    /* GAIN1 register field masks */
    #define GAIN1_GAIN1_MASK                                                ((uint8_t) 0x3F)



/* Register 0x11 (INTPOL) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |       0       |       0       |       0       |    BTRST_EN   |    BALG_EN    |     INTPOL    |   DIS_BTN_TO  |   DIS_BTB_MO  |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* INTPOL register address */
    #define INTPOL_ADDRESS                                                  ((uint8_t) 0x11)

    /* INTPOL default (reset) value */
    #define INTPOL_DEFAULT                                                  ((uint8_t) 0x18)

    /* INTPOL register field masks */
    #define INTPOL_BTRST_EN_MASK                                            ((uint8_t) 0x10)
    #define INTPOL_BALG_EN_MASK                                             ((uint8_t) 0x08)
    #define INTPOL_INTPOL_MASK                                              ((uint8_t) 0x04)
    #define INTPOL_DIS_BTN_TO_MASK                                          ((uint8_t) 0x02)
    #define INTPOL_DIS_BTB_MO_MASK                                          ((uint8_t) 0x01)

    /* BTRST_EN field values */
    #define INTPOL_BTRST_EN_DisableButtonAlgorithmRestart                   ((uint8_t) 0x00)
    #define INTPOL_BTRST_EN_EnableButtonAlgorithmRestart                    ((uint8_t) 0x10)

    /* BALG_EN field values */
    #define INTPOL_BALG_EN_DisableButtonAlgorithm                           ((uint8_t) 0x00)
    #define INTPOL_BALG_EN_EnableButtonAlgorithm                            ((uint8_t) 0x08)

    /* INTPOL field values */
    #define INTPOL_INTPOL_Activelow                                         ((uint8_t) 0x00)
    #define INTPOL_INTPOL_Activehigh                                        ((uint8_t) 0x04)

    /* DIS_BTN_TO field values */
    #define INTPOL_DIS_BTN_TO_EnableButtonTimeout                           ((uint8_t) 0x00)
    #define INTPOL_DIS_BTN_TO_DisableButtonTimeout                          ((uint8_t) 0x02)

    /* DIS_BTB_MO field values */
    #define INTPOL_DIS_BTB_MO_EnableMAXOUTcheck                             ((uint8_t) 0x00)
    #define INTPOL_DIS_BTB_MO_DisableMAXOUTcheck                            ((uint8_t) 0x01)



/* Register 0x12 (GAIN2) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |       0       |       0       |                                           GAIN2[5:0]                                          |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GAIN2 register address */
    #define GAIN2_ADDRESS                                                   ((uint8_t) 0x12)

    /* GAIN2 default (reset) value */
    #define GAIN2_DEFAULT                                                   ((uint8_t) 0x28)

    /* GAIN2 register field masks */
    #define GAIN2_GAIN2_MASK                                                ((uint8_t) 0x3F)



/* Register 0x13 (LP_BASE_INC) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |       0       |       0       |       0       |       0       |       0       |                   LPBI[2:0]                   |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* LP_BASE_INC register address */
    #define LP_BASE_INC_ADDRESS                                             ((uint8_t) 0x13)

    /* LP_BASE_INC default (reset) value */
    #define LP_BASE_INC_DEFAULT                                             ((uint8_t) 0x05)

    /* LP_BASE_INC register field masks */
    #define LP_BASE_INC_LPBI_MASK                                           ((uint8_t) 0x07)



/* Register 0x14 (GAIN3) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |       0       |       0       |                                           GAIN3[5:0]                                          |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GAIN3 register address */
    #define GAIN3_ADDRESS                                                   ((uint8_t) 0x14)

    /* GAIN3 default (reset) value */
    #define GAIN3_DEFAULT                                                   ((uint8_t) 0x28)

    /* GAIN3 register field masks */
    #define GAIN3_GAIN3_MASK                                                ((uint8_t) 0x3F)



/* Register 0x15 (NP_BASE_INC) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |       0       |       0       |       0       |       0       |       0       |                   NPBI[2:0]                   |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* NP_BASE_INC register address */
    #define NP_BASE_INC_ADDRESS                                             ((uint8_t) 0x15)

    /* NP_BASE_INC default (reset) value */
    #define NP_BASE_INC_DEFAULT                                             ((uint8_t) 0x03)

    /* NP_BASE_INC register field masks */
    #define NP_BASE_INC_NPBI_MASK                                           ((uint8_t) 0x07)



/* Register 0x16 (BTPAUSE_MAXWIN) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |    BTPAUSE3   |    BTPAUSE2   |    BTPAUSE1   |    BTPAUSE0   |    MAXWIN3    |    MAXWIN2    |    MAXWIN1    |    MAXWIN0    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* BTPAUSE_MAXWIN register address */
    #define BTPAUSE_MAXWIN_ADDRESS                                          ((uint8_t) 0x16)

    /* BTPAUSE_MAXWIN default (reset) value */
    #define BTPAUSE_MAXWIN_DEFAULT                                          ((uint8_t) 0x00)

    /* BTPAUSE_MAXWIN register field masks */
    #define BTPAUSE_MAXWIN_BTPAUSE3_MASK                                    ((uint8_t) 0x80)
    #define BTPAUSE_MAXWIN_BTPAUSE2_MASK                                    ((uint8_t) 0x40)
    #define BTPAUSE_MAXWIN_BTPAUSE1_MASK                                    ((uint8_t) 0x20)
    #define BTPAUSE_MAXWIN_BTPAUSE0_MASK                                    ((uint8_t) 0x10)
    #define BTPAUSE_MAXWIN_MAXWIN3_MASK                                     ((uint8_t) 0x08)
    #define BTPAUSE_MAXWIN_MAXWIN2_MASK                                     ((uint8_t) 0x04)
    #define BTPAUSE_MAXWIN_MAXWIN1_MASK                                     ((uint8_t) 0x02)
    #define BTPAUSE_MAXWIN_MAXWIN0_MASK                                     ((uint8_t) 0x01)

    /* BTPAUSE3 field values */
    #define BTPAUSE_MAXWIN_BTPAUSE3_ExcludeChannel3fromthemaxwingroup       ((uint8_t) 0x00)
    #define BTPAUSE_MAXWIN_BTPAUSE3_IncludeChannel3inthemaxwingroup         ((uint8_t) 0x80)

    /* BTPAUSE2 field values */
    #define BTPAUSE_MAXWIN_BTPAUSE2_ExcludeChannel2fromthemaxwingroup       ((uint8_t) 0x00)
    #define BTPAUSE_MAXWIN_BTPAUSE2_IncludeChannel2inthemaxwingroup         ((uint8_t) 0x40)

    /* BTPAUSE1 field values */
    #define BTPAUSE_MAXWIN_BTPAUSE1_ExcludeChannel1fromthemaxwingroup       ((uint8_t) 0x00)
    #define BTPAUSE_MAXWIN_BTPAUSE1_IncludeChannel1inthemaxwingroup         ((uint8_t) 0x20)

    /* BTPAUSE0 field values */
    #define BTPAUSE_MAXWIN_BTPAUSE0_ExcludeChannel0fromthemaxwingroup       ((uint8_t) 0x00)
    #define BTPAUSE_MAXWIN_BTPAUSE0_IncludeChannel0inthemaxwingroup         ((uint8_t) 0x10)

    /* MAXWIN3 field values */
    #define BTPAUSE_MAXWIN_MAXWIN3_NormalbaselinetrackingforChannel3regardlessofOUT3status  ((uint8_t) 0x00)
    #define BTPAUSE_MAXWIN_MAXWIN3_PausesbaselinetrackingforChannel3whenOUT3isasserted  ((uint8_t) 0x08)

    /* MAXWIN2 field values */
    #define BTPAUSE_MAXWIN_MAXWIN2_NormalbaselinetrackingforChannel2regardlessofOUT2status  ((uint8_t) 0x00)
    #define BTPAUSE_MAXWIN_MAXWIN2_PausesbaselinetrackingforChannel2whenOUT2isasserted  ((uint8_t) 0x04)

    /* MAXWIN1 field values */
    #define BTPAUSE_MAXWIN_MAXWIN1_NormalbaselinetrackingforChannel1regardlessofOUT1status  ((uint8_t) 0x00)
    #define BTPAUSE_MAXWIN_MAXWIN1_PausesbaselinetrackingforChannel1whenOUT1isasserted  ((uint8_t) 0x02)

    /* MAXWIN0 field values */
    #define BTPAUSE_MAXWIN_MAXWIN0_NormalbaselinetrackingforChannel0regardlessofOUT0status  ((uint8_t) 0x00)
    #define BTPAUSE_MAXWIN_MAXWIN0_PausesbaselinetrackingforChannel0whenOUT0isasserted  ((uint8_t) 0x01)



/* Register 0x17 (LC_DIVIDER) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |       0       |       0       |       0       |       0       |       0       |                   LCDIV[2:0]                  |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* LC_DIVIDER register address */
    #define LC_DIVIDER_ADDRESS                                              ((uint8_t) 0x17)

    /* LC_DIVIDER default (reset) value */
    #define LC_DIVIDER_DEFAULT                                              ((uint8_t) 0x03)

    /* LC_DIVIDER register field masks */
    #define LC_DIVIDER_LCDIV_MASK                                           ((uint8_t) 0x07)



/* Register 0x18 (HYST) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |       0       |       0       |       0       |       0       |                           HYST[3:0]                           |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* HYST register address */
    #define HYST_ADDRESS                                                    ((uint8_t) 0x18)

    /* HYST default (reset) value */
    #define HYST_DEFAULT                                                    ((uint8_t) 0x08)

    /* HYST register field masks */
    #define HYST_HYST_MASK                                                  ((uint8_t) 0x0F)



/* Register 0x19 (TWIST) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |       0       |       0       |       0       |       0       |       0       |                 ANTITWIST[2:0]                |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* TWIST register address */
    #define TWIST_ADDRESS                                                   ((uint8_t) 0x19)

    /* TWIST default (reset) value */
    #define TWIST_DEFAULT                                                   ((uint8_t) 0x00)

    /* TWIST register field masks */
    #define TWIST_ANTITWIST_MASK                                            ((uint8_t) 0x07)



/* Register 0x1A (COMMON_DEFORM) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |    ANTICOM3   |    ANTICOM2   |    ANTICOM1   |    ANTICOM0   |   ANTIDFORM3  |   ANTIDFORM2  |   ANTIDFORM1  |   ANTIDFORM0  |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* COMMON_DEFORM register address */
    #define COMMON_DEFORM_ADDRESS                                           ((uint8_t) 0x1A)

    /* COMMON_DEFORM default (reset) value */
    #define COMMON_DEFORM_DEFAULT                                           ((uint8_t) 0x00)

    /* COMMON_DEFORM register field masks */
    #define COMMON_DEFORM_ANTICOM3_MASK                                     ((uint8_t) 0x80)
    #define COMMON_DEFORM_ANTICOM2_MASK                                     ((uint8_t) 0x40)
    #define COMMON_DEFORM_ANTICOM1_MASK                                     ((uint8_t) 0x20)
    #define COMMON_DEFORM_ANTICOM0_MASK                                     ((uint8_t) 0x10)
    #define COMMON_DEFORM_ANTIDFORM3_MASK                                   ((uint8_t) 0x08)
    #define COMMON_DEFORM_ANTIDFORM2_MASK                                   ((uint8_t) 0x04)
    #define COMMON_DEFORM_ANTIDFORM1_MASK                                   ((uint8_t) 0x02)
    #define COMMON_DEFORM_ANTIDFORM0_MASK                                   ((uint8_t) 0x01)

    /* ANTICOM3 field values */
    #define COMMON_DEFORM_ANTICOM3_ExcludeChannel3fromtheanticommongroup    ((uint8_t) 0x00)
    #define COMMON_DEFORM_ANTICOM3_IncludeChannel3intheanticommongroup      ((uint8_t) 0x80)

    /* ANTICOM2 field values */
    #define COMMON_DEFORM_ANTICOM2_ExcludeChannel2fromtheanticommongroup    ((uint8_t) 0x00)
    #define COMMON_DEFORM_ANTICOM2_IncludeChannel2intheanticommongroup      ((uint8_t) 0x40)

    /* ANTICOM1 field values */
    #define COMMON_DEFORM_ANTICOM1_ExcludeChannel1fromtheanticommongroup    ((uint8_t) 0x00)
    #define COMMON_DEFORM_ANTICOM1_IncludeChannel1intheanticommongroup      ((uint8_t) 0x20)

    /* ANTICOM0 field values */
    #define COMMON_DEFORM_ANTICOM0_ExcludeChannel0fromtheanticommongroup    ((uint8_t) 0x00)
    #define COMMON_DEFORM_ANTICOM0_IncludeChannel0intheanticommongroup      ((uint8_t) 0x10)

    /* ANTIDFORM3 field values */
    #define COMMON_DEFORM_ANTIDFORM3_ExcludeChannel3fromtheantideformgroup  ((uint8_t) 0x00)
    #define COMMON_DEFORM_ANTIDFORM3_IncludeChannel3intheantideformgroup    ((uint8_t) 0x08)

    /* ANTIDFORM2 field values */
    #define COMMON_DEFORM_ANTIDFORM2_ExcludeChannel2fromtheantideformgroup  ((uint8_t) 0x00)
    #define COMMON_DEFORM_ANTIDFORM2_IncludeChannel2intheantideformgroup    ((uint8_t) 0x04)

    /* ANTIDFORM1 field values */
    #define COMMON_DEFORM_ANTIDFORM1_ExcludeChannel1fromtheantideformgroup  ((uint8_t) 0x00)
    #define COMMON_DEFORM_ANTIDFORM1_IncludeChannel1intheantideformgroup    ((uint8_t) 0x02)

    /* ANTIDFORM0 field values */
    #define COMMON_DEFORM_ANTIDFORM0_ExcludeChannel0fromtheantideformgroup  ((uint8_t) 0x00)
    #define COMMON_DEFORM_ANTIDFORM0_IncludeChannel0intheantideformgroup    ((uint8_t) 0x01)



/* Register 0x1C (OPOL_DPOL) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     OPOL3     |     OPOL2     |     OPOL1     |     OPOL0     |     DPOL3     |     DPOL2     |     DPOL1     |     DPOL0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* OPOL_DPOL register address */
    #define OPOL_DPOL_ADDRESS                                               ((uint8_t) 0x1C)

    /* OPOL_DPOL default (reset) value */
    #define OPOL_DPOL_DEFAULT                                               ((uint8_t) 0x0F)

    /* OPOL_DPOL register field masks */
    #define OPOL_DPOL_OPOL3_MASK                                            ((uint8_t) 0x80)
    #define OPOL_DPOL_OPOL2_MASK                                            ((uint8_t) 0x40)
    #define OPOL_DPOL_OPOL1_MASK                                            ((uint8_t) 0x20)
    #define OPOL_DPOL_OPOL0_MASK                                            ((uint8_t) 0x10)
    #define OPOL_DPOL_DPOL3_MASK                                            ((uint8_t) 0x08)
    #define OPOL_DPOL_DPOL2_MASK                                            ((uint8_t) 0x04)
    #define OPOL_DPOL_DPOL1_MASK                                            ((uint8_t) 0x02)
    #define OPOL_DPOL_DPOL0_MASK                                            ((uint8_t) 0x01)

    /* OPOL3 field values */
    #define OPOL_DPOL_OPOL3_Activelow                                       ((uint8_t) 0x00)
    #define OPOL_DPOL_OPOL3_Activehigh                                      ((uint8_t) 0x80)

    /* OPOL2 field values */
    #define OPOL_DPOL_OPOL2_Activelow                                       ((uint8_t) 0x00)
    #define OPOL_DPOL_OPOL2_Activehigh                                      ((uint8_t) 0x40)

    /* OPOL1 field values */
    #define OPOL_DPOL_OPOL1_Activelow                                       ((uint8_t) 0x00)
    #define OPOL_DPOL_OPOL1_Activehigh                                      ((uint8_t) 0x20)

    /* OPOL0 field values */
    #define OPOL_DPOL_OPOL0_Activelow                                       ((uint8_t) 0x00)
    #define OPOL_DPOL_OPOL0_Activehigh                                      ((uint8_t) 0x10)

    /* DPOL3 field values */
    #define OPOL_DPOL_DPOL3_BTN_DATA3decreasesasfSENSOR3increases           ((uint8_t) 0x00)
    #define OPOL_DPOL_DPOL3_DATA3increasesasfSENSOR3increases               ((uint8_t) 0x08)

    /* DPOL2 field values */
    #define OPOL_DPOL_DPOL2_BTN_DATA2decreasesasfSENSOR2increases           ((uint8_t) 0x00)
    #define OPOL_DPOL_DPOL2_DATA2increasesasfSENSOR2increases               ((uint8_t) 0x04)

    /* DPOL1 field values */
    #define OPOL_DPOL_DPOL1_BTN_DATA1decreasesasfSENSOR1increases           ((uint8_t) 0x00)
    #define OPOL_DPOL_DPOL1_DATA1increasesasfSENSOR1increases               ((uint8_t) 0x02)

    /* DPOL0 field values */
    #define OPOL_DPOL_DPOL0_BTN_DATA0decreasesasfSENSOR0increases           ((uint8_t) 0x00)
    #define OPOL_DPOL_DPOL0_DATA0increasesasfSENSOR0increases               ((uint8_t) 0x01)



/* Register 0x1E (CNTSC) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |          CNTSC3[1:0]          |          CNTSC2[1:0]          |          CNTSC1[1:0]          |          CNTSC0[1:0]          |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CNTSC register address */
    #define CNTSC_ADDRESS                                                   ((uint8_t) 0x1E)

    /* CNTSC default (reset) value */
    #define CNTSC_DEFAULT                                                   ((uint8_t) 0x55)

    /* CNTSC register field masks */
    #define CNTSC_CNTSC3_MASK                                               ((uint8_t) 0xC0)
    #define CNTSC_CNTSC2_MASK                                               ((uint8_t) 0x30)
    #define CNTSC_CNTSC1_MASK                                               ((uint8_t) 0x0C)
    #define CNTSC_CNTSC0_MASK                                               ((uint8_t) 0x03)

    /* CNTSC3 field values */
    #define CNTSC_CNTSC3_CNTSC3is0                                          ((uint8_t) 0x00)
    #define CNTSC_CNTSC3_CNTSC3is1                                          ((uint8_t) 0x40)
    #define CNTSC_CNTSC3_CNTSC3is2                                          ((uint8_t) 0x80)
    #define CNTSC_CNTSC3_CNTSC3is3                                          ((uint8_t) 0xC0)

    /* CNTSC2 field values */
    #define CNTSC_CNTSC2_CNTSC2is0                                          ((uint8_t) 0x00)
    #define CNTSC_CNTSC2_CNTSC2is1                                          ((uint8_t) 0x10)
    #define CNTSC_CNTSC2_CNTSC2is2                                          ((uint8_t) 0x20)
    #define CNTSC_CNTSC2_CNTSC2is3                                          ((uint8_t) 0x30)

    /* CNTSC1 field values */
    #define CNTSC_CNTSC1_CNTSC1is0                                          ((uint8_t) 0x00)
    #define CNTSC_CNTSC1_CNTSC1is1                                          ((uint8_t) 0x04)
    #define CNTSC_CNTSC1_CNTSC1is2                                          ((uint8_t) 0x08)
    #define CNTSC_CNTSC1_CNTSC1is3                                          ((uint8_t) 0x0C)

    /* CNTSC0 field values */
    #define CNTSC_CNTSC0_CNTSC0is0                                          ((uint8_t) 0x00)
    #define CNTSC_CNTSC0_CNTSC0is1                                          ((uint8_t) 0x01)
    #define CNTSC_CNTSC0_CNTSC0is2                                          ((uint8_t) 0x02)
    #define CNTSC_CNTSC0_CNTSC0is3                                          ((uint8_t) 0x03)



/* Register 0x20 (SENSOR0_CONFIG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |      RP0      |           FREQ0[1:0]          |                                  SENCYC0[4:0]                                 |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* SENSOR0_CONFIG register address */
    #define SENSOR0_CONFIG_ADDRESS                                          ((uint8_t) 0x20)

    /* SENSOR0_CONFIG default (reset) value */
    #define SENSOR0_CONFIG_DEFAULT                                          ((uint8_t) 0x04)

    /* SENSOR0_CONFIG register field masks */
    #define SENSOR0_CONFIG_RP0_MASK                                         ((uint8_t) 0x80)
    #define SENSOR0_CONFIG_FREQ0_MASK                                       ((uint8_t) 0x60)
    #define SENSOR0_CONFIG_SENCYC0_MASK                                     ((uint8_t) 0x1F)

    /* RP0 field values */
    #define SENSOR0_CONFIG_RP0_50OhmsRp4kOhms                               ((uint8_t) 0x00)
    #define SENSOR0_CONFIG_RP0_800OhmsRp10kOhms                             ((uint8_t) 0x80)

    /* FREQ0 field values */
    #define SENSOR0_CONFIG_FREQ0_1MHzto33MHz                                ((uint8_t) 0x00)
    #define SENSOR0_CONFIG_FREQ0_33MHzto10MHz                               ((uint8_t) 0x20)
    #define SENSOR0_CONFIG_FREQ0_10MHzto30MHz                               ((uint8_t) 0x40)



/* Register 0x22 (SENSOR1_CONFIG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |      RP1      |           FREQ1[1:0]          |                                  SENCYC1[4:0]                                 |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* SENSOR1_CONFIG register address */
    #define SENSOR1_CONFIG_ADDRESS                                          ((uint8_t) 0x22)

    /* SENSOR1_CONFIG default (reset) value */
    #define SENSOR1_CONFIG_DEFAULT                                          ((uint8_t) 0x04)

    /* SENSOR1_CONFIG register field masks */
    #define SENSOR1_CONFIG_RP1_MASK                                         ((uint8_t) 0x80)
    #define SENSOR1_CONFIG_FREQ1_MASK                                       ((uint8_t) 0x60)
    #define SENSOR1_CONFIG_SENCYC1_MASK                                     ((uint8_t) 0x1F)

    /* RP1 field values */
    #define SENSOR1_CONFIG_RP1_50OhmsRp4kOhms                               ((uint8_t) 0x00)
    #define SENSOR1_CONFIG_RP1_800OhmsRp10kOhms                             ((uint8_t) 0x80)

    /* FREQ1 field values */
    #define SENSOR1_CONFIG_FREQ1_1MHzto33MHz                                ((uint8_t) 0x00)
    #define SENSOR1_CONFIG_FREQ1_33MHzto10MHz                               ((uint8_t) 0x20)
    #define SENSOR1_CONFIG_FREQ1_10MHzto30MHz                               ((uint8_t) 0x40)



/* Register 0x24 (SENSOR2_CONFIG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |      RP2      |           FREQ2[1:0]          |                                  SENCYC2[4:0]                                 |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* SENSOR2_CONFIG register address */
    #define SENSOR2_CONFIG_ADDRESS                                          ((uint8_t) 0x24)

    /* SENSOR2_CONFIG default (reset) value */
    #define SENSOR2_CONFIG_DEFAULT                                          ((uint8_t) 0x04)

    /* SENSOR2_CONFIG register field masks */
    #define SENSOR2_CONFIG_RP2_MASK                                         ((uint8_t) 0x80)
    #define SENSOR2_CONFIG_FREQ2_MASK                                       ((uint8_t) 0x60)
    #define SENSOR2_CONFIG_SENCYC2_MASK                                     ((uint8_t) 0x1F)

    /* RP2 field values */
    #define SENSOR2_CONFIG_RP2_50OhmsRp4kOhms                               ((uint8_t) 0x00)
    #define SENSOR2_CONFIG_RP2_800OhmsRp10kOhms                             ((uint8_t) 0x80)

    /* FREQ2 field values */
    #define SENSOR2_CONFIG_FREQ2_1MHzto33MHz                                ((uint8_t) 0x00)
    #define SENSOR2_CONFIG_FREQ2_33MHzto10MHz                               ((uint8_t) 0x20)
    #define SENSOR2_CONFIG_FREQ2_10MHzto30MHz                               ((uint8_t) 0x40)



/* Register 0x25 (FTF0) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |       0       |       0       |       0       |       0       |       0       |           FTF0[1:0]           |       0       |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* FTF0 register address */
    #define FTF0_ADDRESS                                                    ((uint8_t) 0x25)

    /* FTF0 default (reset) value */
    #define FTF0_DEFAULT                                                    ((uint8_t) 0x02)

    /* FTF0 register field masks */
    #define FTF0_FTF0_MASK                                                  ((uint8_t) 0x06)

    /* FTF0 field values */
    #define FTF0_FTF0_FTF0is0                                               ((uint8_t) 0x00)
    #define FTF0_FTF0_FTF0is1                                               ((uint8_t) 0x02)
    #define FTF0_FTF0_FTF0is2                                               ((uint8_t) 0x04)
    #define FTF0_FTF0_FTF0is3                                               ((uint8_t) 0x06)



/* Register 0x26 (SENSOR3_CONFIG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |      RP3      |           FREQ3[1:0]          |                                  SENCYC3[4:0]                                 |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* SENSOR3_CONFIG register address */
    #define SENSOR3_CONFIG_ADDRESS                                          ((uint8_t) 0x26)

    /* SENSOR3_CONFIG default (reset) value */
    #define SENSOR3_CONFIG_DEFAULT                                          ((uint8_t) 0x04)

    /* SENSOR3_CONFIG register field masks */
    #define SENSOR3_CONFIG_RP3_MASK                                         ((uint8_t) 0x80)
    #define SENSOR3_CONFIG_FREQ3_MASK                                       ((uint8_t) 0x60)
    #define SENSOR3_CONFIG_SENCYC3_MASK                                     ((uint8_t) 0x1F)

    /* RP3 field values */
    #define SENSOR3_CONFIG_RP3_50OhmsRp4kOhms                               ((uint8_t) 0x00)
    #define SENSOR3_CONFIG_RP3_800OhmsRp10kOhms                             ((uint8_t) 0x80)

    /* FREQ3 field values */
    #define SENSOR3_CONFIG_FREQ3_1MHzto33MHz                                ((uint8_t) 0x00)
    #define SENSOR3_CONFIG_FREQ3_33MHzto10MHz                               ((uint8_t) 0x20)
    #define SENSOR3_CONFIG_FREQ3_10MHzto30MHz                               ((uint8_t) 0x40)



/* Register 0x28 (FTF1_2) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |           FTF2[1:0]           |           FTF1[1:0]           |       0       |       0       |       0       |       0       |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* FTF1_2 register address */
    #define FTF1_2_ADDRESS                                                  ((uint8_t) 0x28)

    /* FTF1_2 default (reset) value */
    #define FTF1_2_DEFAULT                                                  ((uint8_t) 0x50)

    /* FTF1_2 register field masks */
    #define FTF1_2_FTF2_MASK                                                ((uint8_t) 0xC0)
    #define FTF1_2_FTF1_MASK                                                ((uint8_t) 0x30)

    /* FTF2 field values */
    #define FTF1_2_FTF2_FTF2is0                                             ((uint8_t) 0x00)
    #define FTF1_2_FTF2_FTF2is1                                             ((uint8_t) 0x40)
    #define FTF1_2_FTF2_FTF2is2                                             ((uint8_t) 0x80)
    #define FTF1_2_FTF2_FTF2is3                                             ((uint8_t) 0xC0)

    /* FTF1 field values */
    #define FTF1_2_FTF1_FTF1is0                                             ((uint8_t) 0x00)
    #define FTF1_2_FTF1_FTF1is1                                             ((uint8_t) 0x10)
    #define FTF1_2_FTF1_FTF1is2                                             ((uint8_t) 0x20)
    #define FTF1_2_FTF1_FTF1is3                                             ((uint8_t) 0x30)



/* Register 0x2B (FTF3) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |       0       |       0       |       0       |       0       |       0       |       0       |           FTF3[1:0]           |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* FTF3 register address */
    #define FTF3_ADDRESS                                                    ((uint8_t) 0x2B)

    /* FTF3 default (reset) value */
    #define FTF3_DEFAULT                                                    ((uint8_t) 0x01)

    /* FTF3 register field masks */
    #define FTF3_FTF3_MASK                                                  ((uint8_t) 0x03)

    /* FTF3 field values */
    #define FTF3_FTF3_FTF3is0                                               ((uint8_t) 0x00)
    #define FTF3_FTF3_FTF3is1                                               ((uint8_t) 0x01)
    #define FTF3_FTF3_FTF3is2                                               ((uint8_t) 0x02)
    #define FTF3_FTF3_FTF3is3                                               ((uint8_t) 0x03)



/* Register 0x59 (RAW_DATA0) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 23    |     Bit 22    |     Bit 21    |     Bit 20    |     Bit 19    |     Bit 18    |     Bit 17    |     Bit 16    |     Bit 15    |     Bit 14    |     Bit 13    |     Bit 12    |     Bit 11    |     Bit 10    |     Bit 9     |     Bit 8     |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      RAW_DATA0_23:16[7:0]                                                     |                                                      RAW_DATA0_15:8[7:0]                                                      |                                                       RAW_DATA0_7:0[7:0]                                                      |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* RAW_DATA0 register address */
    #define RAW_DATA0_ADDRESS                                               ((uint8_t) 0x59)

    /* RAW_DATA0 default (reset) value */
    #define RAW_DATA0_DEFAULT                                               ((uint32_t) 0x000000)

    /* RAW_DATA0 register field masks */
    #define RAW_DATA0_RAW_DATA0_23:16_MASK                                  ((uint32_t) 0xFF0000)
    #define RAW_DATA0_RAW_DATA0_15:8_MASK                                   ((uint32_t) 0x00FF00)
    #define RAW_DATA0_RAW_DATA0_7:0_MASK                                    ((uint32_t) 0x0000FF)



/* Register 0x5C (RAW_DATA1) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 23    |     Bit 22    |     Bit 21    |     Bit 20    |     Bit 19    |     Bit 18    |     Bit 17    |     Bit 16    |     Bit 15    |     Bit 14    |     Bit 13    |     Bit 12    |     Bit 11    |     Bit 10    |     Bit 9     |     Bit 8     |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      RAW_DATA1_16:23[7:0]                                                     |                                                      RAW_DATA1_15:8[7:0]                                                      |                                                       RAW_DATA1_7:0[7:0]                                                      |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* RAW_DATA1 register address */
    #define RAW_DATA1_ADDRESS                                               ((uint8_t) 0x5C)

    /* RAW_DATA1 default (reset) value */
    #define RAW_DATA1_DEFAULT                                               ((uint32_t) 0x000000)

    /* RAW_DATA1 register field masks */
    #define RAW_DATA1_RAW_DATA1_16:23_MASK                                  ((uint32_t) 0xFF0000)
    #define RAW_DATA1_RAW_DATA1_15:8_MASK                                   ((uint32_t) 0x00FF00)
    #define RAW_DATA1_RAW_DATA1_7:0_MASK                                    ((uint32_t) 0x0000FF)



/* Register 0x5F (RAW_DATA2) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 23    |     Bit 22    |     Bit 21    |     Bit 20    |     Bit 19    |     Bit 18    |     Bit 17    |     Bit 16    |     Bit 15    |     Bit 14    |     Bit 13    |     Bit 12    |     Bit 11    |     Bit 10    |     Bit 9     |     Bit 8     |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      RAW_DATA2_23:16[7:0]                                                     |                                                      RAW_DATA2_15:8[7:0]                                                      |                                                       RAW_DATA2_7:0[7:0]                                                      |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* RAW_DATA2 register address */
    #define RAW_DATA2_ADDRESS                                               ((uint8_t) 0x5F)

    /* RAW_DATA2 default (reset) value */
    #define RAW_DATA2_DEFAULT                                               ((uint32_t) 0x000000)

    /* RAW_DATA2 register field masks */
    #define RAW_DATA2_RAW_DATA2_23:16_MASK                                  ((uint32_t) 0xFF0000)
    #define RAW_DATA2_RAW_DATA2_15:8_MASK                                   ((uint32_t) 0x00FF00)
    #define RAW_DATA2_RAW_DATA2_7:0_MASK                                    ((uint32_t) 0x0000FF)



/* Register 0x62 (RAW_DATA3) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 23    |     Bit 22    |     Bit 21    |     Bit 20    |     Bit 19    |     Bit 18    |     Bit 17    |     Bit 16    |     Bit 15    |     Bit 14    |     Bit 13    |     Bit 12    |     Bit 11    |     Bit 10    |     Bit 9     |     Bit 8     |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      RAW_DATA3_23:16[7:0]                                                     |                                                      RAW_DATA3_15:8[7:0]                                                      |                                                       RAW_DATA3_7:0[7:0]                                                      |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* RAW_DATA3 register address */
    #define RAW_DATA3_ADDRESS                                               ((uint8_t) 0x62)

    /* RAW_DATA3 default (reset) value */
    #define RAW_DATA3_DEFAULT                                               ((uint32_t) 0x000000)

    /* RAW_DATA3 register field masks */
    #define RAW_DATA3_RAW_DATA3_23:16_MASK                                  ((uint32_t) 0xFF0000)
    #define RAW_DATA3_RAW_DATA3_15:8_MASK                                   ((uint32_t) 0x00FF00)
    #define RAW_DATA3_RAW_DATA3_7:0_MASK                                    ((uint32_t) 0x0000FF)



/* Register 0xFC (MANUFACTURER_ID) definition
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 15    |     Bit 14    |     Bit 13    |     Bit 12    |     Bit 11    |     Bit 10    |     Bit 9     |     Bit 8     |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                  MANUFACTURER_ID_[15:8][7:0]                                                  |                                                   MANUFACTURER_ID_[7:0][7:0]                                                  |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MANUFACTURER_ID register address */
    #define MANUFACTURER_ID_ADDRESS                                         ((uint8_t) 0xFC)

    /* MANUFACTURER_ID default (reset) value */
    #define MANUFACTURER_ID_DEFAULT                                         ((uint16_t) 0x5449)

    /* MANUFACTURER_ID register field masks */
    #define MANUFACTURER_ID_MANUFACTURER_ID_[15:8]_MASK                     ((uint16_t) 0xFF00)
    #define MANUFACTURER_ID_MANUFACTURER_ID_[7:0]_MASK                      ((uint16_t) 0x00FF)



/* Register 0xFE (DEVICE_ID) definition
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 15    |     Bit 14    |     Bit 13    |     Bit 12    |     Bit 11    |     Bit 10    |     Bit 9     |     Bit 8     |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     DEVICE_ID_[15:8][7:0]                                                     |                                                      DEVICE_ID_[7:0][7:0]                                                     |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DEVICE_ID register address */
    #define DEVICE_ID_ADDRESS                                               ((uint8_t) 0xFE)

    /* DEVICE_ID default (reset) value */
    #define DEVICE_ID_DEFAULT                                               ((uint16_t) 0x4000)

    /* DEVICE_ID register field masks */
    #define DEVICE_ID_DEVICE_ID_[15:8]_MASK                                 ((uint16_t) 0xFF00)
    #define DEVICE_ID_DEVICE_ID_[7:0]_MASK                                  ((uint16_t) 0x00FF)



#endif /* LDC3114_H_ */
