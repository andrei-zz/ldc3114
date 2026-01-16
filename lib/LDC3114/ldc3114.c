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

#include "ldc3114.h"

#ifdef EXAMPLE_CODE
#else
// Register names, used by EVM firmware
const char *ldcRegisterNames[NUM_REGISTERS] = {"STATUS","OUT", \
                                               "DATA0","","DATA1","","DATA2","","DATA3","", \
                                               "RESET","","EN","NP_SCAN_RATE","GAIN0","LP_SCAN_RATE", \
                                               "GAIN1","INTPOL","GAIN2","LP_BASE_INC","GAIN3", \
                                               "NP_BASE_INC","BTPAUSE_MAXWIN","LC_DIVIDER","HYST", \
                                               "TWIST","COMMON_DEFORM","","OPOL_DPOL","","CNTSC", \
                                               "","SENSOR0_CONFIG","","SENSOR1_CONFIG","", \
                                               "SENSOR2_CONFIG","FTF0","SENSOR3_CONFIG","","FTF1_2", \
                                               "","","FTF3","","","","","","","","","","","","","", \
                                               "","","","","","","","","","","","","","","","","","", \
                                               "","","","","","","","","","","","","","","RAW_DATA0", \
                                               "","","RAW_DATA1","","","RAW_DATA2","","","RAW_DATA3", \
                                               "","","","","","","","","","","","","","","","","","", \
                                               "","","","","","","","","","","","","","","","","","", \
                                               "","","","","","","","","","","","","","","","","","", \
                                               "","","","","","","","","","","","","","","","","","", \
                                               "","","","","","","","","","","","","","","","","","", \
                                               "","","","","","","","","","","","","","","","","","", \
                                               "","","","","","","","","","","","","","","","","","", \
                                               "","","","","","","","","","","","","","","","","","", \
                                               "","","","","","","","","","MANUFACTURER_ID","","DEVICE_ID",""};

// Register size in bytes formatted to the same lines as the registers above
const uint8_t ldcRegisterSize[NUM_REGISTERS] = { 1,1,2,0,2,0,2,0,2,0,1,0,1,1,1,1, \
                                                 1,1,1,1,1,1,1,1,1,1,1,0,1,0,1,0, \
                                                 1,0,1,0,1,1,1,0,1,0,0,1,0,0,0,0, \
                                                 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, \
                                                 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, \
                                                 0,0,0,0,0,0,0,0,0,3,0,0,3,0,0,3, \
                                                 0,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0, \
                                                 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, \
                                                 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, \
                                                 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, \
                                                 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, \
                                                 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, \
                                                 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, \
                                                 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, \
                                                 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, \
                                                 0,0,0,0,0,0,0,0,0,0,0,0,2,0,2,0};


#endif  // #ifdef EXAMPLE_CODE


//****************************************************************************
//
// Internal variables
//
//****************************************************************************

// Array used to recall device register map configurations */
static uint64_t             registerMap[NUM_REGISTERS];



//****************************************************************************
//
// Internal function prototypes
//
//****************************************************************************



//*****************************************************************************
//
// The interrupt handler for the timer interrupt.
//
//*****************************************************************************
void TIMER0IntHandler(void)
{
    // // Clear the timer interrupt
    // MAP_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // // Set Flag that timer has triggered.
    // collect_timer = true;


    // TODO: not needed when we have Arduino?
}



//****************************************************************************
//
//! Timer used to delay send data in collect mode
//!
//! Initializes timer to interrupt at specified period
//!
//! \fn void startTimer(uint32_t timerPeriod)
//! \param timerPeriod interrupt frequency in units of ms
//!
//! \return None.
//
//****************************************************************************
void startTimer(uint32_t timerPeriod)
{
    // uint32_t systemClock = getSysClockHz();

    // // Disable and clear timer interrupts
    // MAP_TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // MAP_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // // Enable processor interrupts
    // MAP_IntMasterEnable();

    // // Configure the two 32-bit periodic timers
    // MAP_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    // MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, (systemClock/ 1000000) * timerPeriod);

    // // Register the Timer 0A interrupt ISR
    // IntRegister(INT_TIMER0A, TIMER0IntHandler);

    // // Setup the interrupts for the timer timeouts
    // MAP_IntEnable(INT_TIMER0A);
    // MAP_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // // Enable the timer
    // MAP_TimerEnable(TIMER0_BASE, TIMER_A);


    // TODO: not needed when we have Arduino?
    (void) timerPeriod;
}



//****************************************************************************
//
//! Disables and clears timer
//!
//! \return None.
//
//****************************************************************************
void stopTimer()
{
    // uint32_t systemClock = getSysClockHz();

    // // Disable and clear timer interrupts
    // MAP_TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // MAP_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);


    // TODO: not needed when we have Arduino?
}



//*****************************************************************************
//
//! Getter function to access registerMap array from outside of this module.
//!
//! \fn uint64_t getRegisterValue(uint8_t address)
//!
//! NOTE: The internal registerMap arrays stores the last know register value,
//! since the last read or write operation to that register. This function
//! does not communicate with the device to retrieve the current register value.
//! For the most up-to-date register data or retrieving the value of a hardware
//! controlled register, it is recommend to use readSingleRegister() to read the
//! current register value.
//!
//! \return unsigned 64-bit register value.
//
//*****************************************************************************
uint64_t getRegisterValue(uint8_t address)
{
    assert(address < NUM_REGISTERS);
    return registerMap[address];
}



//*****************************************************************************
//
//! Example start up sequence for the LDC3114.
//!
//! \fn void ldcStartup(void)
//!
//! Before calling this function, the device must be powered,
//! the SPI/GPIO pins of the MCU must have already been configured,
//! and (if applicable) the external clock source should be provided to CLKIN.
//!
//! \return None.
//
//*****************************************************************************
void ldcStartup(void)
{
    /* (OPTIONAL) Provide additional delay time for power supply settling */
    delay_ms(50);

    /* (RECOMMENDED) Reset the device */
    resetDevice();

    /* (REQUIRED) Initialize internal 'registerMap' array with device default settings */
    restoreRegisterDefaults();

    /* (OPTIONAL) Configuration mode */
    configMode();

    /* (OPTIONAL) Configure device
     * Note. This requires configuration mode to have run first*/
    configureDevice();

    /* (OPTIONAL) Active processing mode
     * Note. This is required if configMode was uncommented*/
    activeMode();

    /* (OPTIONAL) Read back all registers */
//    readAllRegisters();

    /* (OPTIONAL) Check STATUS register */
//    readSingleRegister(STATUS_ADDRESS, false);
}



//*****************************************************************************
//
//! Reads the Button Data registers from the LDC3114.
//!
//! \fn void readButtonData(void)
//!
//! Before calling this function, the device must be powered and in active mode
//! Device must also have the Button algorithm enabled
//!
//! \return None.
//
//*****************************************************************************
void readButtonData(void)
{
    /* This function reads the button data registers and updates the MCU register map.
     * To use the button data afterwards, use the getRegisterValue(address) function
     * Example: data0 = getRegisterValue(DATA0_ADDRESS);
     *
     * (Recommended) Call this function after the data ready flag has been triggered from the
     * INTB interrupt handler
     *
     * The digital outputs from the device can also be read from GPIO of the MCU
     */

    readSingleRegister(STATUS_ADDRESS, false);  // Must read STATUS register before reading data
    readSingleRegister(OUT_ADDRESS, false);     // Reading OUT clears DRD_READY bit
    readSingleRegister(DATA0_ADDRESS, false);
    readSingleRegister(DATA1_ADDRESS, false);
    readSingleRegister(DATA2_ADDRESS, false);
    readSingleRegister(DATA3_ADDRESS, false);
}



//*****************************************************************************
//
//! Reads the Raw Data registers from the LDC3114.
//!
//! \fn void readRawData(void)
//!
//! Before calling this function, the device must be powered and in active mode
//! Device must also have the Button algorithm disabled
//!
//! \return None.
//
//*****************************************************************************
void readRawData(void)
{
    /* This function reads the button data registers and updates the MCU register map.
     * To use the button data afterwards, use the getRegisterValue(address) function
     * Example: data0 = getRegisterValue(RAW_DATA0_ADDRESS);
     *
     * (Recommended) Call this function after the data ready flag has been triggered from the
     * INTB interrupt handler
     */

    readSingleRegister(STATUS_ADDRESS, false);  // Must read STATUS register before reading data
    readSingleRegister(OUT_ADDRESS, false);     // Reading OUT clears DRD_READY bit
    readSingleRegister(RAW_DATA0_ADDRESS, false);
    readSingleRegister(RAW_DATA1_ADDRESS, false);
    readSingleRegister(RAW_DATA2_ADDRESS, false);
    readSingleRegister(RAW_DATA3_ADDRESS, false);
}



//*****************************************************************************
//
//! Reads the contents of a single register at the specified address.
//!
//! \fn uint16_t readSingleRegister(uint8_t address, bool useBULK)
//!
//! \param address is the 8-bit address of the register to read.
//! \param useBULK sends the data on the USB BULK channel
//! \return Returns the register read result.
//
//*****************************************************************************
uint64_t readSingleRegister(uint8_t address, bool useBULK)
{
//     int i;
//     int j;

//     /* Check that the register address is in range */
//     assert(address < NUM_REGISTERS);

//     // Build TX and RX byte array
//     uint8_t dataTx[6] = { 0 };      // 1 words, each 6 byte long = 6 bytes maximum
//     uint8_t dataRx[6] = { 0 };
//     uint8_t outPacket[10] = { 0 };    // 1 words, each 6 byte long, + 1 byte for device id/channel + address, + data Size, +frame ID = 10 bytes maximum

//     dataTx[0] = address;  // Address
//     i2cSendArrays(dataTx, 1); //set register address
//     i2cReceiveArrays(dataRx, ldcRegisterSize[address]);

//     // flip the button data registers and device ID registers to be in the correct order
//     uint8_t temp = dataRx[0];
//     switch(address)
//     {
//         case DATA0_ADDRESS:
//         case DATA1_ADDRESS:
//         case DATA2_ADDRESS:
//         case DATA3_ADDRESS:
//         case MANUFACTURER_ID_ADDRESS:
//         case DEVICE_ID_ADDRESS:
//             dataRx[0] = dataRx[1];
//             dataRx[1] = temp;
//             break;
//         case RAW_DATA0_ADDRESS:
//         case RAW_DATA1_ADDRESS:
//         case RAW_DATA2_ADDRESS:
//         case RAW_DATA3_ADDRESS:
//             dataRx[0] = dataRx[2];
//             dataRx[2] = temp;
//             break;
//         default:
//             break;
//     }


//     if(useBULK) // communicate register data over USB
//     {
//         j=0;
//         outPacket[j++] = 0; //frame ID to ensure aligned data
//         outPacket[j++] = deviceNumID;
//         outPacket[j++] = address;
//         outPacket[j++] = ldcRegisterSize[address];
//         if(isI2C)
//         {
//             outPacket[j++] = dataRx[0];
//         }
//     }

//     registerMap[address] = dataRx[0];

//     for(i = 1; i < ldcRegisterSize[address]; i++)
//     {
//         registerMap[address] = (registerMap[address] << 8) | dataRx[i];
//         if(useBULK)
//         {
//             outPacket[j] = dataRx[i];
//             j++;
//         }
//     }

//     uint32_t spaceAvailable = USBBufferSpaceAvailable(BULK_TX_BUFFER);
//     if(useBULK)
//     {
//         //Send data on BULK channel
// //        if(spaceAvailable < ldcRegisterSize[regSize_Select][address]+4)
// //        {
// //            json_console("Error, not enough buffer space. have: %d, need: %d", spaceAvailable, ldcRegisterSize[regSize_Select][address]+4);
// //        }

//         //wait for space to be available on the buffer
//         while(spaceAvailable < ldcRegisterSize[address]+4)
//         {
//             spaceAvailable = USBBufferSpaceAvailable(BULK_TX_BUFFER);
//         }

//         USBBufferWrite(BULK_TX_BUFFER, outPacket, ldcRegisterSize[address]+4);
//     }

//     return registerMap[address];


    // We never call this function with useBULK == true
    assert(!useBULK);

    // Check that the register address is in range
    assert(address < NUM_REGISTERS);

    // Build TX and RX byte array, packet only needed for useBULK
    uint8_t dataTx[6] = { 0 };      // 1 words, each 6 byte long = 6 bytes maximum
    uint8_t dataRx[6] = { 0 };

    dataTx[0] = address;
    i2cSendArrays(dataTx, 1);
    i2cReceiveArrays(dataRx, ldcRegisterSize[address]);

    // flip the button data registers and device ID registers to be in the correct order
    uint8_t temp = dataRx[0];
    switch (address) {
        case DATA0_ADDRESS:
        case DATA1_ADDRESS:
        case DATA2_ADDRESS:
        case DATA3_ADDRESS:
        case MANUFACTURER_ID_ADDRESS:
        case DEVICE_ID_ADDRESS:
            dataRx[0] = dataRx[1];
            dataRx[1] = temp;
            break;
        case RAW_DATA0_ADDRESS:
        case RAW_DATA1_ADDRESS:
        case RAW_DATA2_ADDRESS:
        case RAW_DATA3_ADDRESS:
            dataRx[0] = dataRx[2];
            dataRx[2] = temp;
            break;
        default:
            break;
    }

    registerMap[address] = 0;
    for (int i = 0; i < ldcRegisterSize[address]; i++) {
        registerMap[address] = 
            (registerMap[address] << 8) | dataRx[i];
    }

    return registerMap[address];
}


//*****************************************************************************
//
//! Writes data to a single register.
//!
//! \fn void writeSingleRegister(uint8_t address, uint16_t data)
//!
//! \param address is the address of the register to write to.
//! \param data is the value to write.
//!
//! This command will be ignored if device registers are locked.
//!
//! \return None.
//
//*****************************************************************************
void writeSingleRegister(uint8_t address, uint8_t data)
{
    /* Check that the register address is in range */
    assert(address < NUM_REGISTERS);

    // Build TX and RX byte array
    uint8_t dataTx[6] = { 0 };     // 1 words, up to 6 bytes each = 6 bytes maximum
    uint8_t dataRx[6] = { 0 };


    dataTx[1] = (data);   // Data
    dataTx[0] = address;  // Address
    i2cSendArrays(dataTx, ldcRegisterSize[address]+1); //send command

    // Update internal array
    registerMap[address] = data;

    // (RECOMMENDED) Read back register to confirm register write was successful //Usually done by GC
//    readSingleRegister(address, false);

}


//*****************************************************************************
//
//! Resets the device.
//!
//! \fn void resetDevice(void)
//!
//! \return None.
//
//*****************************************************************************
void resetDevice(void)
{
    // set the FULL_RESET bit of the RESET register
    writeSingleRegister(RESET_ADDRESS, RESET_FULL_RESET_MASK);

    // check for the STATUS register to signal the device is ready after reset
    uint8_t ready = readSingleRegister(STATUS_ADDRESS, false) & STATUS_CHIP_READY_MASK;
    while(ready != STATUS_CHIP_READY_MASK)
    {   // poll the status register until the device is ready after internal reset
        ready = readSingleRegister(STATUS_ADDRESS, false) & STATUS_CHIP_READY_MASK;
    }

}

//*****************************************************************************
//
//! Puts device in configuration mode
//!
//! \fn void configMode(void)
//!
//! \return None.
//
//*****************************************************************************
void configMode(void)
{
    // set the CONFIG_MODE bit of the RESET register
    writeSingleRegister(RESET_ADDRESS, RESET_CONFIG_MODE_MASK);

    // check for the STATUS register to signal the device is ready to write registers
    uint8_t ready = readSingleRegister(STATUS_ADDRESS, false) & STATUS_RDY_TO_WRITE_MASK;
    while(ready != STATUS_RDY_TO_WRITE_MASK)
    {   // poll the status register until the device is ready
        ready = readSingleRegister(STATUS_ADDRESS, false) & STATUS_RDY_TO_WRITE_MASK;
    }

}


//*****************************************************************************
//
//!
//! Puts device in active processing mode
//!
//! \fn void activeMode(void)
//!
//! \return None.
//
//*****************************************************************************
void activeMode(void)
{
    // set the CONFIG_MODE bit of the RESET register
    writeSingleRegister(RESET_ADDRESS, RESET_CONFIG_MODE_Normaloperation);

    // check for the STATUS register to signal the device is ready to write registers
    uint8_t ready = readSingleRegister(STATUS_ADDRESS, false) & STATUS_CHIP_READY_MASK;
    while(ready != STATUS_CHIP_READY_MASK)
    {   // poll the status register until the device is ready
        ready = readSingleRegister(STATUS_ADDRESS, false) & STATUS_CHIP_READY_MASK;
    }

}



//*****************************************************************************
//
//! Puts the device in Low Power mode
//!
//! \fn void LPMode(void)
//!
//! \return None.
//
//*****************************************************************************
void LPMode(void)
{
    // // write GPIO low for LP mode
    // MAP_GPIOPinWrite(LPWRB_PORT, LPWRB_PIN, LOW);


    // TODO: Low-power / normal mode GPIO control is not wired
}



//*****************************************************************************
//
//! Puts the device in Normal Power mode
//!
//! \fn void NPMode(void)
//!
//! \return None.
//
//*****************************************************************************
void NPMode(void)
{
    // // write GPIO High for NP mode
    // MAP_GPIOPinWrite(LPWRB_PORT, LPWRB_PIN, HIGH);


    // TODO: Low-power / normal mode GPIO control is not wired
}


//*****************************************************************************
//
//! Poll OUTx GPIO
//!
//! \fn void OUTxGPIO(void)
//!
//! \return None.
//
//*****************************************************************************
uint8_t OUTxGPIO(void)
{
    // // write GPIO low for LP mode
    // uint8_t result;
    // result = MAP_GPIOPinRead(OUT3_PORT, OUT3_PIN);
    // result = result|MAP_GPIOPinRead(OUT2_PORT, OUT2_PIN);
    // result = result>>4;
    // result = result|MAP_GPIOPinRead(OUT1_PORT, OUT1_PIN);
    // result = result|MAP_GPIOPinRead(OUT0_PORT, OUT0_PIN);

    // return result;


    // TODO: OUT0–OUT3 GPIOs are not monitored
    return 0;
}



//*****************************************************************************
//
//! Configures the device.
//!
//! \fn void configureDevice(void)
//!
//! NOTE: Change the settings as desired for application
//!
//! \return None.
//
//*****************************************************************************
void configureDevice(void)
{
    /* set the device configuration settings
     * You can remove any default settings if the device was reset */

    writeSingleRegister(EN_ADDRESS, EN_ALL_MASK);
    // writeSingleRegister(EN_ADDRESS, EN_EN0_MASK);
    writeSingleRegister(NP_SCAN_RATE_ADDRESS, NP_SCAN_RATE_NPSR_80SPS);
//    writeSingleRegister(GAIN0_ADDRESS, GAIN0_DEFAULT);
    writeSingleRegister(LP_SCAN_RATE_ADDRESS, LP_SCAN_RATE_LPSR_25SPS);
//    writeSingleRegister(GAIN1_ADDRESS, GAIN1_DEFAULT);
    writeSingleRegister(INTPOL_ADDRESS, (INTPOL_DEFAULT^INTPOL_BALG_EN_MASK)); // Puts device in raw data mode
//    writeSingleRegister(GAIN2_ADDRESS, GAIN2_DEFAULT);
//    writeSingleRegister(LP_BASE_INC_ADDRESS, LP_BASE_INC_DEFAULT);
//    writeSingleRegister(GAIN3_ADDRESS, GAIN3_DEFAULT);
//    writeSingleRegister(NP_BASE_INC_ADDRESS, NP_BASE_INC_DEFAULT);
//    writeSingleRegister(BTPAUSE_MAXWIN_ADDRESS, BTPAUSE_MAXWIN_DEFAULT);
    writeSingleRegister(LC_DIVIDER_ADDRESS, 4); // must be <8
//    writeSingleRegister(HYST_ADDRESS, HYST_DEFAULT);
//    writeSingleRegister(TWIST_ADDRESS, TWIST_DEFAULT);
//    writeSingleRegister(COMMON_DEFORM_ADDRESS, COMMON_DEFORM_DEFAULT);
//    writeSingleRegister(OPOL_DPOL_ADDRESS, OPOL_DPOL_DEFAULT);

    /* CNTSC must be set based on the button sampling window
     * See datasheet section 8.1.7 for more information */
    writeSingleRegister(CNTSC_ADDRESS, 4);

    /* SENSORx_CONFIG must be set for sensor design
     * See section 8.1.2 of the datasheet for more information */
    writeSingleRegister(SENSOR0_CONFIG_ADDRESS,
                        SENSOR0_CONFIG_RP0_50OhmsRp4kOhms|SENSOR0_CONFIG_FREQ0_33MHzto10MHz);
    writeSingleRegister(SENSOR1_CONFIG_ADDRESS,
                        SENSOR1_CONFIG_RP1_50OhmsRp4kOhms|SENSOR1_CONFIG_FREQ1_33MHzto10MHz);
    writeSingleRegister(SENSOR2_CONFIG_ADDRESS,
                        SENSOR2_CONFIG_RP2_50OhmsRp4kOhms|SENSOR2_CONFIG_FREQ2_33MHzto10MHz);
    writeSingleRegister(SENSOR3_CONFIG_ADDRESS,
                        SENSOR3_CONFIG_RP3_50OhmsRp4kOhms|SENSOR3_CONFIG_FREQ3_33MHzto10MHz);

//    writeSingleRegister(FTF0_ADDRESS, FTF0_DEFAULT);
//    writeSingleRegister(FTF1_2_ADDRESS, FTF1_2_DEFAULT);
//    writeSingleRegister(FTF3_ADDRESS, FTF3_DEFAULT);

}



//*****************************************************************************
//
//! Read all device registers.
//!
//! \fn void readAllRegisters(void)
//!
//! NOTE: Change the settings as desired for application
//!
//! \return None.
//
//*****************************************************************************
void readAllRegisters(void)
{
    // read all registers
    uint8_t i;
    for (i = 0; i < NUM_REGISTERS; i++){
        if (ldcRegisterSize[i] > 0){ //check if valid address from register size map
            readSingleRegister(i, false); // read the register
        }
    }
}

//*****************************************************************************
//
//! Updates the registerMap[] array to its default values.
//!
//! \fn void restoreRegisterDefaults(void)
//!
//! NOTES:
//! - If the MCU keeps a copy of the LDC3114 register settings in memory,
//! then it is important to ensure that these values remain in sync with the
//! actual hardware settings. In order to help facilitate this, this function
//! should be called after powering up or resetting the device.
//!
//! - Reading back all of the registers after resetting the device can
//! accomplish the same result.
//!
//! \return None.
//
//*****************************************************************************
void restoreRegisterDefaults(void)
{
    registerMap[STATUS_ADDRESS] = STATUS_DEFAULT;
    registerMap[OUT_ADDRESS] = OUT_DEFAULT;
    registerMap[DATA0_ADDRESS] = DATA0_DEFAULT;
    registerMap[DATA1_ADDRESS] = DATA1_DEFAULT;
    registerMap[DATA2_ADDRESS] = DATA2_DEFAULT;
    registerMap[DATA3_ADDRESS] = DATA3_DEFAULT;
    registerMap[RESET_ADDRESS] = RESET_DEFAULT;
    registerMap[EN_ADDRESS] = EN_DEFAULT;
    registerMap[NP_SCAN_RATE_ADDRESS] = NP_SCAN_RATE_DEFAULT;
    registerMap[GAIN0_ADDRESS] = GAIN0_DEFAULT;
    registerMap[LP_SCAN_RATE_ADDRESS] = LP_SCAN_RATE_DEFAULT;
    registerMap[GAIN1_ADDRESS] = GAIN1_DEFAULT;
    registerMap[INTPOL_ADDRESS] = INTPOL_DEFAULT;
    registerMap[GAIN2_ADDRESS] = GAIN2_DEFAULT;
    registerMap[LP_BASE_INC_ADDRESS] = LP_BASE_INC_DEFAULT;
    registerMap[GAIN3_ADDRESS] = GAIN3_DEFAULT;
    registerMap[NP_BASE_INC_ADDRESS] = NP_BASE_INC_DEFAULT;
    registerMap[BTPAUSE_MAXWIN_ADDRESS] = BTPAUSE_MAXWIN_DEFAULT;
    registerMap[LC_DIVIDER_ADDRESS] = LC_DIVIDER_DEFAULT;
    registerMap[HYST_ADDRESS] = HYST_DEFAULT;
    registerMap[TWIST_ADDRESS] = TWIST_DEFAULT;
    registerMap[COMMON_DEFORM_ADDRESS] = COMMON_DEFORM_DEFAULT;
    registerMap[OPOL_DPOL_ADDRESS] = OPOL_DPOL_DEFAULT;
    registerMap[CNTSC_ADDRESS] = CNTSC_DEFAULT;
    registerMap[SENSOR0_CONFIG_ADDRESS] = SENSOR0_CONFIG_DEFAULT;
    registerMap[SENSOR1_CONFIG_ADDRESS] = SENSOR1_CONFIG_DEFAULT;
    registerMap[SENSOR2_CONFIG_ADDRESS] = SENSOR2_CONFIG_DEFAULT;
    registerMap[FTF0_ADDRESS] = FTF0_DEFAULT;
    registerMap[SENSOR3_CONFIG_ADDRESS] = SENSOR3_CONFIG_DEFAULT;
    registerMap[FTF1_2_ADDRESS] = FTF1_2_DEFAULT;
    registerMap[FTF3_ADDRESS] = FTF3_DEFAULT;
    registerMap[RAW_DATA0_ADDRESS] = RAW_DATA0_DEFAULT;
    registerMap[RAW_DATA1_ADDRESS] = RAW_DATA1_DEFAULT;
    registerMap[RAW_DATA2_ADDRESS] = RAW_DATA2_DEFAULT;
    registerMap[RAW_DATA3_ADDRESS] = RAW_DATA3_DEFAULT;
    registerMap[MANUFACTURER_ID_ADDRESS] = MANUFACTURER_ID_DEFAULT;
    registerMap[DEVICE_ID_ADDRESS] = DEVICE_ID_DEFAULT;
}



//****************************************************************************
//
// Helper functions
//
//****************************************************************************


//*****************************************************************************
//
//! Takes a 16-bit word and returns the most-significant byte.
//!
//! \fn uint8_t upperByte(uint16_t uint16_Word)
//!
//! \param temp_word is the original 16-bit word.
//!
//! \return 8-bit most-significant byte.
//
//*****************************************************************************
uint8_t upperByte(uint16_t uint16_Word)
{
    uint8_t msByte;
    msByte = (uint8_t) ((uint16_Word >> 8) & 0x00FF);

    return msByte;
}



//*****************************************************************************
//
//! Takes a 16-bit word and returns the least-significant byte.
//!
//! \fn uint8_t lowerByte(uint16_t uint16_Word)
//!
//! \param temp_word is the original 16-bit word.
//!
//! \return 8-bit least-significant byte.
//
//*****************************************************************************
uint8_t lowerByte(uint16_t uint16_Word)
{
    uint8_t lsByte;
    lsByte = (uint8_t) (uint16_Word & 0x00FF);

    return lsByte;
}



//*****************************************************************************
//
//! Takes two 8-bit words and returns a concatenated 16-bit word.
//!
//! \fn uint16_t combineBytes(uint8_t upperByte, uint8_t lowerByte)
//!
//! \param upperByte is the 8-bit value that will become the MSB of the 16-bit word.
//! \param lowerByte is the 8-bit value that will become the LSB of the 16-bit word.
//!
//! \return concatenated 16-bit word.
//
//*****************************************************************************
uint16_t combineBytes(uint8_t upperByte, uint8_t lowerByte)
{
    uint16_t combinedValue;
    combinedValue = ((uint16_t) upperByte << 8) | ((uint16_t) lowerByte);

    return combinedValue;
}


