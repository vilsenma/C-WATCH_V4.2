/**
  Generated Pin Manager File

  Company:
    Microchip Technology Inc.

  File Name:
    pin_manager.c

  Summary:
    This is the Pin Manager file generated using MPLAB? Code Configurator

  Description:
    This header file provides implementations for pin APIs for all pins selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB? Code Configurator - v2.25.2
        Device            :  PIC16F1786
        Driver Version    :  1.02
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

#include <xc.h>
#include "pin_manager.h"

/* 引脚初始化*/
void PIN_MANAGER_Initialize(void) 
{
    //Another way to set
    OPTION_REGbits.nWPUEN = 0; // enable weak pullups (each pin must be enabled individually)

    // PORT A Assignments
    TRISAbits.TRISA0 = 1; // RA0 = nc
    TRISAbits.TRISA1 = 1; // RA1 = nc
    TRISAbits.TRISA2 = 0; // RA2 = nc
    TRISAbits.TRISA3 = 1; // RA3 = nc
    TRISAbits.TRISA4 = 0; // RA4 = nc
    TRISAbits.TRISA5 = 1; // RA5 = nc
    TRISAbits.TRISA6 = 0; // RA6 = nc
    TRISAbits.TRISA7 = 0; // RA7 = nc

    ANSELA = 0x00; // all port A pins are digital I/O
    
    //LATA = 0x90;        //PA置零,但不包括LED
    //TRISA = 0x2B;       //引脚状态RA0~RA7  00101011  （数据方向，0为输出，1为输入，无用引脚需要设置成输出零状态） 
    //ANSELA = 0x6F;      //模拟控制寄存器设置 01101111 1为输入模拟，0输出数字。
    //WPUA = 0x00;        //弱上拉寄存器设置 1使能上拉，0禁止上拉

    LATB = 0x00;        //PB置零  
    TRISB = 0xF2;       //11010010   RB1输入
    ANSELB = 0x4F;      //01001111
    WPUB = 0x00;
  
    LATC = 0x00;        //PC置零
    TRISC = 0x00;        
    WPUC = 0x00;

    TRISE = 0x08;      //1000
    WPUE = 0x00;

    OPTION_REGbits.nWPUEN = 0x01; 

    APFCON1 = 0x20;     //备用引脚设置寄存器
    APFCON2 = 0x00;

    //无用引脚 置成输出模式 低电平
    
    LATA2 = 0;
    LATA7 = 0;
    
    LATB0 = 0;
    
    LATC0 = 0;
    LATC1 = 0;
    LATC2 = 0;
    LATC3 = 0;
    LATC4 = 0;
    LATC6 = 0;
    
    
}
/**
 End of File
 */