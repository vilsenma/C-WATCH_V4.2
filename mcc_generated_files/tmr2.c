/**
  TMR2 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    tmr2.c

  @Summary
    This is the generated driver implementation file for the TMR2 driver using MPLAB? Code Configurator

  @Description
    This source file provides APIs for TMR2.
    Generation Information :
        Product Revision  :  MPLAB? Code Configurator - v2.25.2
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

#include <xc.h>
#include "tmr2.h"
#include "dac1.h"
#include "pin_manager.h"

/**
  Section: TMR2 APIs
 */

void TMR2_Initialize(void) {
    // Set TMR2 to the options selected in the User Interface

    // TMR2ON off; T2CKPS 1:16; T2OUTPS 1:2; 
    T2CON = 0x0A;

    // PR2 129; 
    PR2 = 0x81;

    // TMR2 0x0; 
    TMR2 = 0x00;

    // Clearing IF flag before enabling the interrupt.
    PIR1bits.TMR2IF = 0;

    // Enabling TMR2 interrupt.
    PIE1bits.TMR2IE = 1;

    // Start TMR2
    TMR2_StartTimer();
}

void TMR2_StartTimer(void) {
    // Start the Timer by writing to TMRxON bit
    T2CONbits.TMR2ON = 1;
}

void TMR2_StopTimer(void) {
    // Stop the Timer by writing to TMRxON bit
    T2CONbits.TMR2ON = 0;
}

uint8_t TMR2_ReadTimer(void) {
    uint8_t readVal;

    readVal = TMR2;

    return readVal;
}

void TMR2_WriteTimer(uint8_t timerVal) {
    // Write to the Timer2 register
    TMR2 = timerVal;
}

void TMR2_LoadPeriodRegister(uint8_t periodVal) {
    PR2 = periodVal;
}


extern uint8_t  Hz_10_flag;
extern uint8_t  DAC_flag;
extern uint8_t  Res_AD_flag;
extern uint8_t RES_FLAG;
extern uint32_t TIME_FLAG_num;
extern uint8_t TIME_FLAG;//时间标志


void TMR2_ISR(void) 
{
    // clear the TMR2 interrupt flag
    PIR1bits.TMR2IF = 0;
    
    if(TIME_FLAG==1)//说明正在测试内阻
    {
        LED1_SetLow(); // 红灯
        LED0_SetLow();
        KEY_SetLow(); //打开放点回路
        
        Hz_10_flag++;
        if(Hz_10_flag>49)
        {
            Hz_10_flag=0;
            DAC_flag++;
            if(DAC_flag>1)
            {
                DAC_flag=0;
            }
            if(DAC_flag==1)
            {
                //DAC1_SetOutput(0x54);
                DAC1_SetOutput(0x61);
                //DAC1_SetOutput(0x55);
                
                //DAC1_SetOutput(0xff);
                
                Res_AD_flag++;
            }
            else 
            {       
                DAC1_SetOutput(0x00);
            }
        }

        if(Res_AD_flag>50)
        { 
            Res_AD_flag=0;
            RES_FLAG=1;
        }
    }
    else
    {

        TIME_FLAG_num++; 
        if(TIME_FLAG_num==90000)
        {
            TIME_FLAG_num=0;
            TIME_FLAG=1;    
        }
      
    }
    
}