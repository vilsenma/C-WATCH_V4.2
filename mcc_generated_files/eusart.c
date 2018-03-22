/**
  EUSART Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    eusart.c

  @Summary
    This is the generated driver implementation file for the EUSART driver using MPLAB?Code Configurator

  @Description
    This header file provides implementations for driver APIs for EUSART.
    Generation Information :
        Product Revision  :  MPLAB?Code Configurator - v2.25.2
        Device            :  PIC16F1786
        Driver Version    :  2.00
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

/**
  Section: Included Files
 */
#include "eusart.h"

/**
  Section: Macro Declarations
 */
#define EUSART_TX_BUFFER_SIZE 8
#define EUSART_RX_BUFFER_SIZE 8

/**
  Section: Global Variables
 */

static uint8_t eusartTxHead = 0;
static uint8_t eusartTxTail = 0;
static uint8_t eusartTxBuffer[EUSART_TX_BUFFER_SIZE];//
volatile uint8_t eusartTxBufferRemaining;

static uint8_t eusartRxHead = 0;
static uint8_t eusartRxTail = 0;
static uint8_t eusartRxBuffer[EUSART_RX_BUFFER_SIZE];//
volatile uint8_t eusartRxCount;

/**
  Section: EUSART APIs
 */

void EUSART_Initialize(void) 
{
    // disable interrupts before changing states
    PIE1bits.RCIE = 0;
    PIE1bits.TXIE = 0;

    // Set the EUSART module to the options selected in the user interface.

    // ABDEN disabled; WUE disabled; RCIDL idle; ABDOVF no_overflow; SCKP async_noninverted_sync_fallingedge; BRG16 16bit_generator; 
    BAUD1CON = 0x48;

    // ADDEN disabled; RX9 8-bit; RX9D 0x0; FERR no_error; CREN disabled; SPEN enabled; SREN disabled; OERR no_error; 
    RC1STA = 0x90;

    // CSRC slave_mode; TRMT TSR_empty; TXEN enabled; BRGH hi_speed; SYNC asynchronous; SENDB sync_break_complete; TX9D 0x0; TX9 8-bit; 
    TX1STA = 0x26;

    //SPBRG�Ĵ���ֵ = ����Ƶ�� / (Ŀ�겨���� * 16) - 1
    //Baud Rate = 9600; SP1BRG 415; 
    // Baud Rate = 9600; SP1BRGL 0x9f; 
    SP1BRGL = 0x9F;
    // Baud Rate = 9600; SP1BRGH 0x01; 
    SP1BRGH = 0x01;
  
    
 /*   
    // Baud Rate = 19200; SP1BRGL 207; 
    SP1BRGL = 0xCF;
    // Baud Rate = 19200; SP1BRGH 0; 
    SP1BRGH = 0x00;
*/


    
    // initializing the driver state
    eusartTxHead = 0;
    eusartTxTail = 0;
    eusartTxBufferRemaining = sizeof (eusartTxBuffer);

    eusartRxHead = 0;
    eusartRxTail = 0;
    eusartRxCount = 0;

    // enable receive interrupt
    PIE1bits.RCIE = 1;
}

uint8_t EUSART_Read(void) 
{
    uint8_t readValue = 0;

    while (0 == eusartRxCount) 
    {
    }

    PIE1bits.RCIE = 0;

    readValue = eusartRxBuffer[eusartRxTail++];
    if (sizeof (eusartRxBuffer) <= eusartRxTail) 
    {
        eusartRxTail = 0;
    }
    eusartRxCount--;
    PIE1bits.RCIE = 1;

    return readValue;
}

void EUSART_Write(uint8_t txData) 
{
    while (0 == eusartTxBufferRemaining) 
    {
    }

    if (0 == PIE1bits.TXIE) 
    {
        TX1REG = txData;
    } 
    else 
    {
        PIE1bits.TXIE = 0;
        eusartTxBuffer[eusartTxHead++] = txData;
        if (sizeof (eusartTxBuffer) <= eusartTxHead) 
        {
            eusartTxHead = 0;
        }
        eusartTxBufferRemaining--;
    }
    PIE1bits.TXIE = 1;
}

void EUSART_Transmit_ISR(void) 
{
    
    // add your EUSART interrupt custom code
    if (sizeof (eusartTxBuffer) > eusartTxBufferRemaining) 
    {
        TX1REG = eusartTxBuffer[eusartTxTail++];
        if (sizeof (eusartTxBuffer) <= eusartTxTail) 
        {
            eusartTxTail = 0;
        }
        eusartTxBufferRemaining++;
    } 
    else 
    {
        PIE1bits.TXIE = 0;
    }
}
extern uint8_t command; 
void EUSART_Receive_ISR(void) 
{
    if (1 == RC1STAbits.OERR) 
    {
        // EUSART error - restart

        RC1STAbits.CREN = 0;
        RC1STAbits.CREN = 1;
    }

    // buffer overruns are ignored
    //command = RC1REG;
 
}
/**
  End of File
 */
