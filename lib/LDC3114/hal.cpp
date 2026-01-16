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

#include <Arduino.h>
#include <Wire.h>

extern "C" {
#include "hal.h"
}
#include "utils.h"

//****************************************************************************
//
// Internal variables
//
//****************************************************************************

// Flag to indicate if a /INTB interrupt has occurred
static volatile bool flag_INTB_INTERRUPT = false;

//I2C variables
#define HS_PREAMBLE       0x0F
//int SLAVE_ADDRESS_EXT = 0x40; //replaced with global Address_Select for changeability
bool HS_Preamble_Sent = false;

uint8_t  Address_Select = 0x2A;
uint16_t deviceID = 0;
uint8_t  regSize_Select = 0;  // TODO: unused

//****************************************************************************
//
// Internal function prototypes
//
//****************************************************************************
void InitGPIO(void);
void InitI2C2(void);
// void GPIO_INTB_IRQHandler(void);

// void MY_I2CMasterInitExpClk(uint32_t ui32Base, uint32_t ui32I2CClk, bool bFast);

#define I2C_O_MTPR              0x0000000C  // I2C Master Timer Period
#define I2C_O_PP                0x00000FC0  // I2C Peripheral Properties

#define I2C_CLK_FREQ 400000

//****************************************************************************
//
// External Functions (prototypes declared in hal.h)
//
//****************************************************************************

#ifdef EXAMPLE_CODE
#else
bool getINTBinterruptStatus(void)
{
    return flag_INTB_INTERRUPT;
}

void setINTBinterruptStatus(const bool value)
{
    flag_INTB_INTERRUPT = value;
}

void enableINTBinterrupt(void)
{
    // //INTB is enabled as interrupt
    // GPIOIntRegister(INTB_PORT, GPIO_INTB_IRQHandler);
    // MAP_GPIOPinTypeGPIOInput(INTB_PORT, INTB_PIN);
    // MAP_GPIOIntTypeSet(INTB_PORT, INTB_PIN, GPIO_FALLING_EDGE);
    // MAP_GPIOIntEnable(INTB_PORT, INTB_PIN);
    // MAP_IntEnable(INTB_INT);

    // TODO: not needed for polling approach, implement when we use INTB
}
#endif

//*****************************************************************************
//
//! Initializes MCU peripherals for interfacing with the ADC.
//!
//! \fn void InitADC(void)
//!
//! \return None.
//
//*****************************************************************************
void InitLDC(void)
{
    // IMPORTANT: Make sure device is powered before setting GPIOs pins to HIGH state.

    //Check if EVM is I2C, else use SPI (done before GPIO setups, for temp GPIO config)
    CheckI2C();

    // Initialize GPIOs pins used by EVM
    InitGPIO();

    // Initialize I2C peripheral used by EVM
    InitI2C2();
    DetermineDevice();

    // //enable timer peripheral
    // MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    // Run ADC startup function
    ldcStartup();
}




//****************************************************************************
//
// Timing functions
//
//****************************************************************************



//*****************************************************************************
//
//! Provides a timing delay with 'ms' resolution.
//!
//! \fn void delay_ms(const uint32_t delay_time_ms)
//!
//! \param delay_time_ms is the number of milliseconds to delay.
//!
//! \return None.
//
//*****************************************************************************
void delay_ms(const uint32_t delay_time_ms)
{
    /* --- INSERT YOUR CODE HERE --- */

    // const uint32_t cycles_per_loop = 3;
    // MAP_SysCtlDelay( delay_time_ms * getSysClockHz() / (cycles_per_loop * 1000u) );

    delay(delay_time_ms);
}



//*****************************************************************************
//
//! Provides a timing delay with 'us' resolution.
//!
//! \fn void delay_us(const uint32_t delay_time_us)
//!
//! \param delay_time_us is the number of microseconds to delay.
//!
//! \return None.
//
//*****************************************************************************
void delay_us(const uint32_t delay_time_us)
{
    /* --- INSERT YOUR CODE HERE --- */

    // const uint32_t cycles_per_loop = 3;
    // MAP_SysCtlDelay( delay_time_us * getSysClockHz() / (cycles_per_loop * 1000000u) );

    delayMicroseconds(delay_time_us);
}




//****************************************************************************
//
// GPIO initialization
//
//****************************************************************************



//*****************************************************************************
//
//! Configures the MCU's GPIO pins that interface with the ADC.
//!
//! \fn void InitGPIO(void)
//!
//! \return None.
//
//*****************************************************************************
void InitGPIO(void)
{
    /* --- INSERT YOUR CODE HERE --- */
    // NOTE: Not all hardware implementations may control each of these pins...

    /* Enable the clock to the GPIO Port K, M, and H and wait for it to be ready */
//    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
//    while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK)));
//    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
//    while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM)));
//    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
//    while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOH)));

    // /* Configure the GPIO for 'INTB' as input with falling edge interrupt */
    // GPIOIntRegister(INTB_PORT, GPIO_INTB_IRQHandler);
    // MAP_GPIOPinTypeGPIOInput(INTB_PORT, INTB_PIN);
    // MAP_GPIOIntTypeSet(INTB_PORT, INTB_PIN, GPIO_FALLING_EDGE);
    // MAP_GPIOIntEnable(INTB_PORT, INTB_PIN);
    // MAP_IntEnable(INTB_INT);

    // // Set LPWRB pin to high for NP mode
    // MAP_GPIOPinTypeGPIOOutput(LPWRB_PORT, LPWRB_PIN);
    // MAP_GPIOPinWrite(LPWRB_PORT, LPWRB_PIN, HIGH);
    // MAP_GPIOPadConfigSet(LPWRB_PORT, LPWRB_PIN, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);

    // // Map OUTx pins as inputs
    // MAP_GPIOPinTypeGPIOInput(OUT0_PORT, OUT0_PIN);
    // MAP_GPIOPinTypeGPIOInput(OUT1_PORT, OUT1_PIN);
    // MAP_GPIOPinTypeGPIOInput(OUT2_PORT, OUT2_PIN);
    // MAP_GPIOPinTypeGPIOInput(OUT3_PORT, OUT3_PIN);

}


#if 0

//*****************************************************************************
//
// Interrupt handler for INTB GPIO
//
//*****************************************************************************

//*****************************************************************************
//
//! Interrupt handler for /INTB falling edge interrupt.
//!
//! \fn void GPIO_INTB_IRQHandler(void)
//!
//! \return None.
//
//*****************************************************************************
void GPIO_INTB_IRQHandler(void)
{
    /* --- INSERT YOUR CODE HERE --- */
    //NOTE: You many need to rename or register this interrupt function for your processor

    // Possible ways to handle this interrupt:
    // If you decide to read data here, you may want to disable other interrupts to avoid partial data reads.

    // In this example we set a flag and exit the interrupt routine. In the main program loop, your application can examine
    // all state flags and decide which state (operation) to perform next.

    /* Get the interrupt status from the GPIO and clear the status */
    uint32_t getIntStatus = MAP_GPIOIntStatus(INTB_PORT, true);

    /* Check if the INTB pin triggered the interrupt */
    if(getIntStatus & INTB_PIN)
    {
        /* Interrupt action: Set a flag */
        flag_INTB_INTERRUPT = true;
    }

    /* Clear interrupt */
    MAP_GPIOIntClear(INTB_PORT, getIntStatus);

    // NOTE: We add a short delay at the end to prevent re-entrance. Refer to E2E issue:
    // https://e2e.ti.com/support/microcontrollers/tiva_arm/f/908/p/332605/1786938#1786938
    SysCtlDelay(3);
}

#endif


//****************************************************************************
//
// GPIO helper functions
//
//****************************************************************************


//*****************************************************************************
//
//! Waits for the INTB interrupt or until the specified timeout occurs.
//!
//! \fn bool waitForINTBinterrupt(const uint32_t timeout_ms)
//!
//! \param timeout_ms number of milliseconds to wait before timeout event.
//!
//! \return Returns 'true' if INTB interrupt occurred before the timeout.
//
//*****************************************************************************
bool waitForINTBinterrupt(const uint32_t timeout_ms)
{
    /* --- INSERT YOUR CODE HERE ---
     * Poll the INTB GPIO pin until it goes low. To avoid potential infinite
     * loops, you may also want to implement a timer interrupt to occur after
     * the specified timeout period, in case the INTB pin is not active.
     * Return a boolean to indicate if INTB went low or if a timeout occurred.
     */

#ifdef EXAMPLE_CODE
    // Convert ms to a # of loop iterations, OR even better use a timer here...
#else
    // TODO: In a future revision, utilize an internal timer to implement the timeout feature
#endif
    uint64_t start = millis();

    // Reset interrupt flag
    flag_INTB_INTERRUPT = false;

    // Wait for INTB interrupt or timeout
    // TODO: call yield() in this loop if stuffs hang
    while (
        !flag_INTB_INTERRUPT && 
        (millis() - start < timeout_ms)
    );

    bool occurred = flag_INTB_INTERRUPT;

    // Reset interrupt flag
    flag_INTB_INTERRUPT = false;

    // flag_INTB_INTERRUPT true before reset indicates that an interrupt occurred
    return occurred;
}



//****************************************************************************
//
// I2C Communication
//
//****************************************************************************

//*****************************************************************************
//
//! Configures the MCU's I2C peripheral, for interfacing with the ADC.
//!
//! \fn void InitI2C2(void)
//!
//! \return None.
//
//*****************************************************************************
void InitI2C2(void)
{
    // //I2C
    // /* Enable GPIO for Configuring the I2C Interface Pins*/
    // MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    // /* Wait for the Peripheral to be ready for programming*/
    // while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPION));

    // /* Configure Pins for I2C2 Master Interface*/
    // MAP_GPIOPinConfigure(GPIO_PN5_I2C2SCL);
    // MAP_GPIOPinConfigure(GPIO_PN4_I2C2SDA);

    // MAP_GPIOPinTypeI2C(GPIO_PORTN_BASE, GPIO_PIN_4);
    // MAP_GPIOPinTypeI2CSCL(GPIO_PORTN_BASE, GPIO_PIN_5);

    // /* Since there are no board pull up's we shall enable the weak internal
    //  * pull up */ //
    // GPION->PUR |= (GPIO_PIN_4 | GPIO_PIN_5); //We do have a board pull-up

    // /* Stop the Clock, Reset and Enable I2C Module in Master Function */
    // MAP_SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C2);
    // MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C2);
    // MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);

    // /* Wait for the Peripheral to be ready for programming */
    // while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C2));

    // /* Initialize and Configure the Master Module */
    // MY_I2CMasterInitExpClk(I2C2_BASE, getSysClockHz(), true);

    // /* Check if the Bus is Busy or not */
    // while(MAP_I2CMasterBusBusy(I2C2_BASE));


    Wire.begin();
    Wire.setClock(I2C_CLK_FREQ);
    HS_Preamble_Sent = false;
}

//*****************************************************************************
//
//! Sends I2C Data.
//!
//! \fn void i2cSendArrays(const uint8_t dataTx[], const uint8_t byteLength)
//!
//! \param const uint8_t dataTx[] byte array of I2C data to send on SDA.
//!
//! \param uint8_t byteLength number of bytes to send.
//!
//! NOTE: Make sure 'dataTx[]' contains at least as many bytes of data,
//! as indicated by 'byteLength'.
//!
//! \return None.
//
//*****************************************************************************

void i2cSendArrays(const uint8_t dataTx[], const uint8_t byteLength)
{

    // int i;

    // //Set slave address with write bit
    // MAP_I2CMasterSlaveAddrSet(I2C2_BASE, Address_Select, false);

    // if (byteLength > 1){
    //     //Send first byte
    //     MAP_I2CMasterDataPut(I2C2_BASE, dataTx[0]);
    //     MAP_I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    //     while(MAP_I2CMasterBusy(I2C2_BASE)){}    // Delay until transmission completes //May need to duplicate 1-3 times when running at 100kHz and 400kHz
    //     while(MAP_I2CMasterBusy(I2C2_BASE)){}
    //     while(MAP_I2CMasterBusy(I2C2_BASE)){}
    //     while(MAP_I2CMasterBusy(I2C2_BASE)){}

    //     //Send remaining bytes
    //     for (i = 1; i < byteLength-1; i++)
    //     {
    //         MAP_I2CMasterDataPut(I2C2_BASE, dataTx[i]);
    //         MAP_I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    //         while(MAP_I2CMasterBusy(I2C2_BASE)){}    // Delay until transmission completes //May need to duplicate 1-3 times when running at 100kHz and 400kHz
    //         while(MAP_I2CMasterBusy(I2C2_BASE)){}
    //         while(MAP_I2CMasterBusy(I2C2_BASE)){}
    //         while(MAP_I2CMasterBusy(I2C2_BASE)){}
    //     }
    //     MAP_I2CMasterDataPut(I2C2_BASE, dataTx[byteLength-1]);
    //     MAP_I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    //     while(MAP_I2CMasterBusy(I2C2_BASE)){}    // Delay until transmission completes //May need to duplicate 1-3 times when running at 100kHz and 400kHz
    //     while(MAP_I2CMasterBusy(I2C2_BASE)){}
    //     while(MAP_I2CMasterBusy(I2C2_BASE)){}
    //     while(MAP_I2CMasterBusy(I2C2_BASE)){}
    // }
    // else{
    //     MAP_I2CMasterDataPut(I2C2_BASE, dataTx[0]);
    //     MAP_I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    //     while(MAP_I2CMasterBusy(I2C2_BASE)){}    // Delay until transmission completes //May need to duplicate 1-3 times when running at 100kHz and 400kHz
    //     while(MAP_I2CMasterBusy(I2C2_BASE)){}
    //     while(MAP_I2CMasterBusy(I2C2_BASE)){}
    //     while(MAP_I2CMasterBusy(I2C2_BASE)){}
    // }


    Wire.beginTransmission(Address_Select);
    for (uint8_t i = 0; i < byteLength; i++) {
        Wire.write(dataTx[i]);
    }
    Wire.endTransmission();
}

//*****************************************************************************
//
//! Receives I2C Data
//!
//! \fn void i2cReceiveArrays(uint8_t dataRx[], const uint8_t byteLength)
//!
//! \param uint8_t dataRx[] byte array of I2C data captured on SDA.
//!
//! \param uint8_t byteLength number of bytes to receive.
//!
//! NOTE: Make sure 'dataRx[]' contains at least as larg as the number of bytes of data,
//! as indicated by 'byteLength'.
//!
//! \return None.
//
//*****************************************************************************

void i2cReceiveArrays(uint8_t dataRx[], const uint8_t byteLength)
{

    // int i;

    // //Set slave address with read bit
    // MAP_I2CMasterSlaveAddrSet(I2C2_BASE, Address_Select, true);

    // if (byteLength > 1){
    //     //Read first byte
    //     MAP_I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    //     while(MAP_I2CMasterBusy(I2C2_BASE)){}    // Delay until transmission completes //May need to duplicate 1-3 times when running at 100kHz and 400kHz
    //     while(MAP_I2CMasterBusy(I2C2_BASE)){}
    //     while(MAP_I2CMasterBusy(I2C2_BASE)){}
    //     while(MAP_I2CMasterBusy(I2C2_BASE)){}
    //     dataRx[0] = MAP_I2CMasterDataGet(I2C2_BASE);

    //     //Read remaining bytes
    //     for (i = 1; i < byteLength-1; i++)
    //     {
    //         MAP_I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    //         while(MAP_I2CMasterBusy(I2C2_BASE)){}    // Delay until transmission completes //May need to duplicate 1-3 times when running at 100kHz and 400kHz
    //         while(MAP_I2CMasterBusy(I2C2_BASE)){}
    //         while(MAP_I2CMasterBusy(I2C2_BASE)){}
    //         while(MAP_I2CMasterBusy(I2C2_BASE)){}
    //         dataRx[i] = MAP_I2CMasterDataGet(I2C2_BASE);
    //     }
    //     MAP_I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    //     while(MAP_I2CMasterBusy(I2C2_BASE)){}    // Delay until transmission completes //May need to duplicate 1-3 times when running at 100kHz and 400kHz
    //     while(MAP_I2CMasterBusy(I2C2_BASE)){}
    //     while(MAP_I2CMasterBusy(I2C2_BASE)){}
    //     while(MAP_I2CMasterBusy(I2C2_BASE)){}
    //     dataRx[byteLength-1] = MAP_I2CMasterDataGet(I2C2_BASE);
    // }
    // else{
    //     MAP_I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    //     while(MAP_I2CMasterBusy(I2C2_BASE)){}    // Delay until transmission completes //May need to duplicate 1-3 times when running at 100kHz and 400kHz
    //     while(MAP_I2CMasterBusy(I2C2_BASE)){}
    //     while(MAP_I2CMasterBusy(I2C2_BASE)){}
    //     while(MAP_I2CMasterBusy(I2C2_BASE)){}
    //     dataRx[0] = MAP_I2CMasterDataGet(I2C2_BASE);
    // }


    uint8_t received = Wire.requestFrom(Address_Select, byteLength);

    uint8_t i = 0;
    while (Wire.available() && i < byteLength) {
        dataRx[i++] = Wire.read();
    }

    // Padding
    while (i < byteLength) {
        dataRx[i++] = 0;
    }
}


//*****************************************************************************
//
//! Checks to see if the EVM uses I2C by looking for the pull up resistors
//!
//! \fn void CheckI2C(void)
//!
//! \return None.
//
//*****************************************************************************

void CheckI2C(void)
{
    // uint8_t test1 = 0;
    // uint8_t test2 = 0;

    // isSPI = false;
    // isI2C = false;

    // //I2C on PN4 and PN5, should make dynamic
    // MAP_GPIOPinTypeGPIOInput(GPIO_PORTN_BASE, GPIO_PIN_4);
    // test1 = MAP_GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_4);

    // MAP_GPIOPinTypeGPIOInput(GPIO_PORTN_BASE, GPIO_PIN_5);
    // test2 = MAP_GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_5);

    // if((test1 & GPIO_PIN_4) == GPIO_PIN_4)
    // {
    //     if((test2 & GPIO_PIN_5) == GPIO_PIN_5)
    //     {
    //         isI2C = true;
    //     }
    //     else
    //     {
    //         isSPI = true;
    //     }
    // }
    // else
    // {
    //     isSPI = true;
    // }


    // Our LDC3144 is soldered, uses I2C
    // isI2C = true;
    // isSPI = false;
    HS_Preamble_Sent = false;
}


//*****************************************************************************
//
//! Determines which specific device is connected to the EVM by checking for
//! the device id across different addresses/chip-selects.
//!
//! This also sets the address/chip-select to the first device found.
//!
//! \fn void DetermineDevice(void)
//!
//! \return None.
//
//*****************************************************************************

void DetermineDevice(void)
{
    uint64_t value = 0;

    Address_Select = 0x2A;
    value = readSingleRegister(MANUFACTURER_ID_ADDRESS, false);
    if(value == MANUFACTURER_ID_DEFAULT)
    {
        deviceID = readSingleRegister(DEVICE_ID_ADDRESS, false) & 0xFFFF;
        if (deviceID == DEVICE_ID_DEFAULT)
        {
            regSize_Select = 0;
        }
        else
        {
            regSize_Select = 1;
        }

        Serial.print("LDC device found at ");
        printHexWithPadding(Address_Select, 1);
        Serial.print(", deviceID=");
        printHexWithPadding(deviceID, ldcRegisterSize[DEVICE_ID_ADDRESS]);
        Serial.print("\n");
        return;
    }

    Serial.print("ERROR: No LDC device found at I2C ");
    printHexWithPadding(Address_Select, 1);
    Serial.print("\n");
}


#if 0

//*****************************************************************************
//
//! Overwrite i2c init function to choose custom HS frequency
//
//*****************************************************************************
void MY_I2CMasterInitExpClk(uint32_t ui32Base, uint32_t ui32I2CClk, bool bFast)
{
    uint32_t ui32SCLFreq;
    uint32_t ui32TPR;

    //
    // Check the arguments.
    //
    ASSERT(_I2CBaseValid(ui32Base));

    //
    // Must enable the device before doing anything else.
    //
    I2CMasterEnable(ui32Base);

    //
    // Get the desired SCL speed.
    //
    if(bFast == true)
    {
        ui32SCLFreq = 400000;
    }
    else
    {
        ui32SCLFreq = 100000;
    }

    //
    // Compute the clock divider that achieves the fastest speed less than or
    // equal to the desired speed.  The numerator is biased to favor a larger
    // clock divider so that the resulting clock is always less than or equal
    // to the desired clock, never greater.
    //
    ui32TPR = ((ui32I2CClk + (2 * 10 * ui32SCLFreq) - 1) /
               (2 * 10 * ui32SCLFreq)) - 1;
    HWREG(ui32Base + I2C_O_MTPR) = ui32TPR;

    //
    // Check to see if this I2C peripheral is High-Speed enabled.  If yes, also
    // choose the fastest speed that is less than or equal to 2.94Mbps // was 3.4 Mbps.
    //
    if(HWREG(ui32Base + I2C_O_PP) & I2C_PP_HS)
    {
        ui32TPR = ((ui32I2CClk + (2 * 3 * 2940000) - 1) /
                   (2 * 3 * 2940000)) - 1;
        HWREG(ui32Base + I2C_O_MTPR) = I2C_MTPR_HS | ui32TPR;
    }
}

#endif
