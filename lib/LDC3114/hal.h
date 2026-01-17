/**
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
 * Modified for Arduino on nRF52
 */

#ifndef INTERFACE_H_
#define INTERFACE_H_

#include "ldc3114.h"

#ifdef EXAMPLE_CODE
#else
//#include "eeprom.h"
#endif

//****************************************************************************
//
// Standard libraries
//
//****************************************************************************

#include <stdbool.h>
#include <stdint.h>


//****************************************************************************
//
// Insert processor specific header file(s) here
//
//****************************************************************************

#ifdef EXAMPLE_CODE
/*  --- INSERT YOUR CODE HERE --- */
// #include "ti/devices/msp432e4/driverlib/driverlib.h"
#else
// #include "settings.h"
#endif



//****************************************************************************
//
// BoosterPack pinout...
//
//****************************************************************************
//
//                  LEFT                                RIGHT
//               /--------\                          /--------\
//        +3.3V -|3V3  +5V|- +5V                    -|PG1  GND|- GND
//              -|PD2  GND|- GND                    -|PK4  PM7|-
//              -|PP0  PB4|-                        -|PK5 *PP5|-
//              -|PP1  PB5|-                        -|PM0  PA7|-
//              -|PD4  PK0|-                   INTB -|PM1  RST|-
//              -|PD5  PK1|-                  LPWRB -|PM2  PQ2|-
//              -|PQ0  PK2|-                   OUT0 -|PH0  PQ3|-
//              -|PP4* PK3|-                   OUT1 -|PH1 *PP3|-
//          SCL -|PN5  PA4|-                   OUT2 -|PK6  PQ1|-
//          SDA -|PN4  PA5|-                   OUT3 -|PK7  PM6|-
//               \--------/                          \--------/
//
#ifdef EXAMPLE_CODE
#else
//
// * NOTE: These pins differ from the MSP-EXP432E401Y LaunchPad!
//  MSP432E LP:  PP4, PP5, PP3
//  PAMB (REV1): PA6, PN1, PN0
//  PAMB (REV2): PH3, PN3, PN2
//
#endif



//*****************************************************************************
//
// Pin definitions (MSP432E401Y)
//
//*****************************************************************************

#define INTB_PORT           (GPIO_PORTM_BASE)
#define INTB_PIN            (GPIO_PIN_1)
#define INTB_INT            (INT_GPIOM)

#define LPWRB_PORT          (GPIO_PORTM_BASE)
#define LPWRB_PIN           (GPIO_PIN_2)

#define OUT0_PORT           (GPIO_PORTH_BASE)
#define OUT0_PIN            (GPIO_PIN_0)

#define OUT1_PORT           (GPIO_PORTH_BASE)
#define OUT1_PIN            (GPIO_PIN_1)

#define OUT2_PORT           (GPIO_PORTK_BASE)
#define OUT2_PIN            (GPIO_PIN_6)

#define OUT3_PORT           (GPIO_PORTK_BASE)
#define OUT3_PIN            (GPIO_PIN_7)

#define EEPROM_WP_EN_PORT   (GPIO_PORTH_BASE)
#define EEPROM_WP_EN_PIN    (GPIO_PIN_3)


// #ifdef __cplusplus
// extern "C" {
// #endif


//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************

//void    getEVMID(void);
void    CheckI2C(void);
void    DetermineDevice(void);
void    InitLDC(void);
void    delay_ms(const uint32_t delay_time_ms);
void    delay_us(const uint32_t delay_time_us);
void    i2cSendArrays(const uint8_t dataTx[], const uint8_t byteLength);
void    i2cReceiveArrays(uint8_t dataRx[], const uint8_t byteLength);
bool    waitForINTBinterrupt(const uint32_t timeout_ms);
#ifdef  EXAMPLE_CODE
#else
bool    getINTBinterruptStatus(void);
void    setINTBinterruptStatus(const bool value);
void    enableINTBinterrupt(void);
#endif


//*****************************************************************************
//
// Macros
//
//*****************************************************************************
// /** Alias used for setting GPIOs pins to the logic "high" state */
// #ifndef HIGH
// #define HIGH                ((bool) true)
// #endif

// /** Alias used for setting GPIOs pins to the logic "low" state */
// #ifndef LOW
// #define LOW                 ((bool) false)
// #endif


// #ifdef __cplusplus
// }
// #endif


#endif /* INTERFACE_H_ */
