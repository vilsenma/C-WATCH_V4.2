/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using MPLAB? Code Configurator

  @Description:
    This header file provides implementations for pin APIs for all pins selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB? Code Configurator - v2.25.2
        Device            :  PIC16F1786
        Version           :  1.01
    The generated drivers are tested against the following:
        Compiler          :  XC8 v1.34
        MPLAB             :  MPLAB X v2.35 or v3.00
 */

/*
Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 */

#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

/*
#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set DAC1OUT1 aliases
#define DAC1OUT1_TRIS               TRISA2
#define DAC1OUT1_LAT                LATA2
#define DAC1OUT1_PORT               RA2
#define DAC1OUT1_WPU                WPUA2
#define DAC1OUT1_ANS                ANSA2
#define DAC1OUT1_SetHigh()    do { LATA2 = 1; } while(0)
#define DAC1OUT1_SetLow()   do { LATA2 = 0; } while(0)
#define DAC1OUT1_Toggle()   do { LATA2 = ~LATA2; } while(0)
#define DAC1OUT1_GetValue()         RA2
#define DAC1OUT1_SetDigitalInput()    do { TRISA2 = 1; } while(0)
#define DAC1OUT1_SetDigitalOutput()   do { TRISA2 = 0; } while(0)

#define DAC1OUT1_SetPullup()    do { WPUA2 = 1; } while(0)
#define DAC1OUT1_ResetPullup()   do { WPUA2 = 0; } while(0)
#define DAC1OUT1_SetAnalogMode()   do { ANSA2 = 1; } while(0)
#define DAC1OUT1_SetDigitalMode()   do { ANSA2 = 0; } while(0)

// get/set VREFPos aliases
#define VREFPos_TRIS               TRISA3
#define VREFPos_LAT                LATA3
#define VREFPos_PORT               RA3
#define VREFPos_WPU                WPUA3
#define VREFPos_ANS                ANSA3
#define VREFPos_SetHigh()    do { LATA3 = 1; } while(0)
#define VREFPos_SetLow()   do { LATA3 = 0; } while(0)
#define VREFPos_Toggle()   do { LATA3 = ~LATA3; } while(0)
#define VREFPos_GetValue()         RA3
#define VREFPos_SetDigitalInput()    do { TRISA3 = 1; } while(0)
#define VREFPos_SetDigitalOutput()   do { TRISA3 = 0; } while(0)

#define VREFPos_SetPullup()    do { WPUA3 = 1; } while(0)
#define VREFPos_ResetPullup()   do { WPUA3 = 0; } while(0)
#define VREFPos_SetAnalogMode()   do { ANSA3 = 1; } while(0)
#define VREFPos_SetDigitalMode()   do { ANSA3 = 0; } while(0)

// get/set channel_AN4 aliases
#define channel_AN4_TRIS               TRISA5
#define channel_AN4_LAT                LATA5
#define channel_AN4_PORT               RA5
#define channel_AN4_WPU                WPUA5
#define channel_AN4_ANS                ANSA5
#define channel_AN4_SetHigh()    do { LATA5 = 1; } while(0)
#define channel_AN4_SetLow()   do { LATA5 = 0; } while(0)
#define channel_AN4_Toggle()   do { LATA5 = ~LATA5; } while(0)
#define channel_AN4_GetValue()         RA5
#define channel_AN4_SetDigitalInput()    do { TRISA5 = 1; } while(0)
#define channel_AN4_SetDigitalOutput()   do { TRISA5 = 0; } while(0)

#define channel_AN4_SetPullup()    do { WPUA5 = 1; } while(0)
#define channel_AN4_ResetPullup()   do { WPUA5 = 0; } while(0)
#define channel_AN4_SetAnalogMode()   do { ANSA5 = 1; } while(0)
#define channel_AN4_SetDigitalMode()   do { ANSA5 = 0; } while(0)

// get/set IO_RC2 aliases
#define IO_RC2_TRIS               TRISC2
#define IO_RC2_LAT                LATC2
#define IO_RC2_PORT               RC2
#define IO_RC2_WPU                WPUC2
#define IO_RC2_SetHigh()    do { LATC2 = 1; } while(0)
#define IO_RC2_SetLow()   do { LATC2 = 0; } while(0)
#define IO_RC2_Toggle()   do { LATC2 = ~LATC2; } while(0)
#define IO_RC2_GetValue()         RC2
#define IO_RC2_SetDigitalInput()    do { TRISC2 = 1; } while(0)
#define IO_RC2_SetDigitalOutput()   do { TRISC2 = 0; } while(0)

#define IO_RC2_SetPullup()    do { WPUC2 = 1; } while(0)
#define IO_RC2_ResetPullup()   do { WPUC2 = 0; } while(0)


// get/set IO_RC3 aliases
#define IO_RC3_TRIS               TRISC3
#define IO_RC3_LAT                LATC3
#define IO_RC3_PORT               RC3
#define IO_RC3_WPU                WPUC3
#define IO_RC3_SetHigh()    do { LATC3 = 1; } while(0)
#define IO_RC3_SetLow()   do { LATC3 = 0; } while(0)
#define IO_RC3_Toggle()   do { LATC3 = ~LATC3; } while(0)
#define IO_RC3_GetValue()         RC3
#define IO_RC3_SetDigitalInput()    do { TRISC3 = 1; } while(0)
#define IO_RC3_SetDigitalOutput()   do { TRISC3 = 0; } while(0)

#define IO_RC3_SetPullup()    do { WPUC3 = 1; } while(0)
#define IO_RC3_ResetPullup()   do { WPUC3 = 0; } while(0)
// get/set TX aliases
#define TX_TRIS               TRISC6
#define TX_LAT                LATC6
#define TX_PORT               RC6
#define TX_WPU                WPUC6
#define TX_SetHigh()    do { LATC6 = 1; } while(0)
#define TX_SetLow()   do { LATC6 = 0; } while(0)
#define TX_Toggle()   do { LATC6 = ~LATC6; } while(0)
#define TX_GetValue()         RC6
#define TX_SetDigitalInput()    do { TRISC6 = 1; } while(0)
#define TX_SetDigitalOutput()   do { TRISC6 = 0; } while(0)

#define TX_SetPullup()    do { WPUC6 = 1; } while(0)
#define TX_ResetPullup()   do { WPUC6 = 0; } while(0)
// get/set RX aliases
#define RX_TRIS               TRISC7
#define RX_LAT                LATC7
#define RX_PORT               RC7
#define RX_WPU                WPUC7
#define RX_SetHigh()    do { LATC7 = 1; } while(0)
#define RX_SetLow()   do { LATC7 = 0; } while(0)
#define RX_Toggle()   do { LATC7 = ~LATC7; } while(0)
#define RX_GetValue()         RC7
#define RX_SetDigitalInput()    do { TRISC7 = 1; } while(0)
#define RX_SetDigitalOutput()   do { TRISC7 = 0; } while(0)

#define RX_SetPullup()    do { WPUC7 = 1; } while(0)
#define RX_ResetPullup()   do { WPUC7 = 0; } while(0)

#define MCU_TX_SetHigh()    do { LATC4 = 1; } while(0)
#define MCU_TX_SetLow()   do { LATC4 = 0; } while(0)
#define MCU_RX_SetHigh()    do { LATC5 = 1; } while(0)
#define MCU_RX_SetLow()   do { LATC5 = 0; } while(0)
*/


/*LED 引脚定义*/
#define LED0_SetHigh()    do { LATA7 = 1; } while(0)
#define LED0_SetLow()   do { LATA7 = 0; } while(0)

#define LED1_SetHigh()    do { LATA4 = 1; } while(0)
#define LED1_SetLow()   do { LATA4 = 0; } while(0)

#define KEY_SetHigh()    do { LATC4 = 1; } while(0)
#define KEY_SetLow()   do { LATC4 = 0; } while(0)


/*nRF24L01 引脚定义*/


#define WL_CE_SetHigh() do{LATC7 = 1;}while(0)
#define WL_CE_SetLow() do{LATC7 = 0;}while(0)

#define WL_CSN_SetHigh() do{LATC5 = 1;}while(0)
#define WL_CSN_SetLow() do{LATC5 = 0;}while(0)

#define WL_MISO_SetHigh() do{LATB5 = 1;}while(0)
#define WL_MISO_SetLow() do{LATB5 = 0;}while(0)
#define WL_MISO_GetValue()  RB5

#define WL_IRQ_SetHigh() do{LATB4 = 1;}while(0)
#define WL_IRQ_SetLow() do{LATB4 = 0;}while(0)
#define WL_IRQ_GetValue()  RB4

#define WL_SCK_SetHigh() do{LATB3 = 1;}while(0)
#define WL_SCK_SetLow() do{LATB3 = 0;}while(0)

#define WL_MOSI_SetHigh() do{LATB2 = 1;}while(0)
#define WL_MOSI_SetLow() do{LATB2 = 0;}while(0)

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    GPIO and peripheral I/O initialization
 * @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize(void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);

#endif // PIN_MANAGER_H
/**
 End of File
 */