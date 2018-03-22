/**
  Generated Interrupt Manager Source File

  @Company:
    Microchip Technology Inc.

  @File Name:
    interrupt_manager.c

  @Summary:
    This is the Interrupt Manager file generated using MPLAB? Code Configurator

  @Description:
    This header file provides implementations for global interrupt handling.
    For individual peripheral handlers please see the peripheral driver for
    all modules selected in the GUI.
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

#include "interrupt_manager.h"
#include "mcc.h"
#include "mcu_eusart.h"
#include "NRF24L01.h"
#include "../main.h"

extern uint8_t RES_FLAG;
extern uint8_t TIME_FLAG;//时间标志

uint8_t ReviceOK=0;

uint8_t x2[10]={0x01,0xb1,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xB0};
/****************************************************************
*中断服务函数
*功能：针对接收到的指令进行测量数据返回操作
*返回数据：1.测量内阻数据返回
*		   2.检验码校验返回
*		   3.
*
****************************************************************/
void interrupt INTERRUPT_InterruptManager(void) 
{

    if((IOCIF==1)&&(IOCIE==1))
    {
        IOCIE=0;//关闭电平变化中断
        IOCBF4=0;//清除PB4电平变化标志
        
        //接受无线数据进行处理    
        NRF24L01_RxPacket(RX_BUF);
        ReviceOK=0;

		//NRF24L01_SendMSG(RX_BUF);	//this is for a transmission test.
		
        if(RX_BUF[0]==TUREDATA.Data.Addr||RX_BUF[0]==0x00)//比对地址信息
        {
            for(int i=0;i<10;i++)
            {
                TX_BUF[i]=0;
            }
            
            LED0_SetLow();//收到本中端信息
            
            if(RES_FLAG||TIME_FLAG)//采集内阻时返回忙
            {          
                if(RX_BUF[0]!=0x00)//不是广播数据
                {    
                    TX_BUF[0]=RX_BUF[0];
                    TX_BUF[1]=0xB2;
                    TX_BUF[2]=0xff; 
                    TX_BUF[9]=BCC(TX_BUF,3);
                    NRF24L01_SendMSG(TX_BUF);
                }
                IOCIE=1;//打开电平外部中断
                return ;
            }   
            else if(BCC(RX_BUF,9)!=RX_BUF[9])//校验码错误直接返回数据
            {
                           
                if(RX_BUF[0]!=0x00)
                {   
  
                    TX_BUF[0]=RX_BUF[0];
                    TX_BUF[1]=RX_BUF[1];
                    TX_BUF[2]=0x84;
                    TX_BUF[9]=BCC(TX_BUF,9);
                    NRF24L01_SendMSG(TX_BUF);
                }
                IOCIE=1;//打开电平外部中断
                return ;
            }
            else
            {
                ReviceOK=1;
            	if(1==ReviceOK)//校验正确
            	{  
                	ReviceOK=0;
                	if(RES_FLAG)//采集内阻时返回忙
                	{   
                    	if(RX_BUF[0]!=0x00)//不是广播数据
                    	{    
                        	TX_BUF[0]=RX_BUF[0];
                        	TX_BUF[1]=0xB2;
                        	TX_BUF[2]=0xff;
                        	TX_BUF[9]=BCC(TX_BUF,3);
                        	NRF24L01_SendMSG(TX_BUF);
                    	}
                    	IOCIE=1;//打开电平外部中断
                    	return ;
                	}
            
                	else//分析功能码，进行处理。
                	{
                    	TUREDATA.Data.CMD=RX_BUF[1];
                    
                    	switch(TUREDATA.Data.CMD)
                    	{
                        	case 0xB1:
                            	//发送缓存中实际采集数据
                            	if(RX_BUF[0]==0x00){;}//广播无回复
                            	else
                            	{
                                	NRF24L01_SendMSG((uint8_t *)TUREDATA.DataBuffer);
                            	}
                            	break;
                        	case 0xB2:
                            	//采集内阻
                            	if(0==RES_ING)//未进行采集内阻
                            	{    
                
                                	if(RX_BUF[0]==0x00){;}//广播无回复
                                	else
                                	{
                                    	NRF24L01_SendMSG(RX_BUF);  
                                	}
                          
                                	if(TUREDATA.Data.CellHValue>0)//电压判断
                                	{      
                                    	RES_ING=1;
                                    	TIME_FLAG=1;
                                    	TMR2_StartTimer();//产生10Hz方波
                                	}
                                	else//电压判断，低电压不采集内阻
                                	{
                                    	RES_ING=0;                            
                                	}   
                            	}
                            	else//采集内阻忙回复
                            	{
                                	if(RX_BUF[0]!=0x00)
                                	{    
                                    	TX_BUF[0]=RX_BUF[0];
                                    	TX_BUF[1]=RX_BUF[1];
                                    	TX_BUF[2]=0xff;
                                    	TX_BUF[9]=BCC(TX_BUF,3);
                                    	NRF24L01_SendMSG(TX_BUF);
                                	}
                                
                            	}
                            	break;
                        	case 0xB6:
                            	//标定指令
                            	CONFIG_SET=RX_BUF[2];
                            	CONFIG_BUF[0]=RX_BUF[3];
                            	CONFIG_BUF[1]=RX_BUF[4];
                            	break;
                        	default:
                            	break;
                    	}        
            
                	}    
            	}
            }    
            __delay_ms(10); 
            LED0_SetHigh();//收到本中端信息       
        }    
        IOCIE=1;//打开电平外部中断
    }
    else if (PIE1bits.TMR2IE == 1 && PIR1bits.TMR2IF == 1)
    {
        TMR2_ISR(); //判断是否是在测量内阻
    }
    else if (PIE1bits.TMR1IE == 1 && PIR1bits.TMR1IF == 1) 
    {
        TMR1_ISR(); //重载定时器1，并标记SAMPLE_FLAG（完成定时采集内阻任务）
    }
    else
    {  
        
       
    }
}
/**
 End of File
 */
