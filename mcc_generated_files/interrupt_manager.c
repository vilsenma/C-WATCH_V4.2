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
extern uint8_t TIME_FLAG;//ʱ���־

uint8_t ReviceOK=0;

uint8_t x2[10]={0x01,0xb1,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xB0};
/****************************************************************
*�жϷ�����
*���ܣ���Խ��յ���ָ����в������ݷ��ز���
*�������ݣ�1.�����������ݷ���
*		   2.������У�鷵��
*		   3.
*
****************************************************************/
void interrupt INTERRUPT_InterruptManager(void) 
{

    if((IOCIF==1)&&(IOCIE==1))
    {
        IOCIE=0;//�رյ�ƽ�仯�ж�
        IOCBF4=0;//���PB4��ƽ�仯��־
        
        //�����������ݽ��д���    
        NRF24L01_RxPacket(RX_BUF);
        ReviceOK=0;

		//NRF24L01_SendMSG(RX_BUF);	//this is for a transmission test.
		
        if(RX_BUF[0]==TUREDATA.Data.Addr||RX_BUF[0]==0x00)//�ȶԵ�ַ��Ϣ
        {
            for(int i=0;i<10;i++)
            {
                TX_BUF[i]=0;
            }
            
            LED0_SetLow();//�յ����ж���Ϣ
            
            if(RES_FLAG||TIME_FLAG)//�ɼ�����ʱ����æ
            {          
                if(RX_BUF[0]!=0x00)//���ǹ㲥����
                {    
                    TX_BUF[0]=RX_BUF[0];
                    TX_BUF[1]=0xB2;
                    TX_BUF[2]=0xff; 
                    TX_BUF[9]=BCC(TX_BUF,3);
                    NRF24L01_SendMSG(TX_BUF);
                }
                IOCIE=1;//�򿪵�ƽ�ⲿ�ж�
                return ;
            }   
            else if(BCC(RX_BUF,9)!=RX_BUF[9])//У�������ֱ�ӷ�������
            {
                           
                if(RX_BUF[0]!=0x00)
                {   
  
                    TX_BUF[0]=RX_BUF[0];
                    TX_BUF[1]=RX_BUF[1];
                    TX_BUF[2]=0x84;
                    TX_BUF[9]=BCC(TX_BUF,9);
                    NRF24L01_SendMSG(TX_BUF);
                }
                IOCIE=1;//�򿪵�ƽ�ⲿ�ж�
                return ;
            }
            else
            {
                ReviceOK=1;
            	if(1==ReviceOK)//У����ȷ
            	{  
                	ReviceOK=0;
                	if(RES_FLAG)//�ɼ�����ʱ����æ
                	{   
                    	if(RX_BUF[0]!=0x00)//���ǹ㲥����
                    	{    
                        	TX_BUF[0]=RX_BUF[0];
                        	TX_BUF[1]=0xB2;
                        	TX_BUF[2]=0xff;
                        	TX_BUF[9]=BCC(TX_BUF,3);
                        	NRF24L01_SendMSG(TX_BUF);
                    	}
                    	IOCIE=1;//�򿪵�ƽ�ⲿ�ж�
                    	return ;
                	}
            
                	else//���������룬���д���
                	{
                    	TUREDATA.Data.CMD=RX_BUF[1];
                    
                    	switch(TUREDATA.Data.CMD)
                    	{
                        	case 0xB1:
                            	//���ͻ�����ʵ�ʲɼ�����
                            	if(RX_BUF[0]==0x00){;}//�㲥�޻ظ�
                            	else
                            	{
                                	NRF24L01_SendMSG((uint8_t *)TUREDATA.DataBuffer);
                            	}
                            	break;
                        	case 0xB2:
                            	//�ɼ�����
                            	if(0==RES_ING)//δ���вɼ�����
                            	{    
                
                                	if(RX_BUF[0]==0x00){;}//�㲥�޻ظ�
                                	else
                                	{
                                    	NRF24L01_SendMSG(RX_BUF);  
                                	}
                          
                                	if(TUREDATA.Data.CellHValue>0)//��ѹ�ж�
                                	{      
                                    	RES_ING=1;
                                    	TIME_FLAG=1;
                                    	TMR2_StartTimer();//����10Hz����
                                	}
                                	else//��ѹ�жϣ��͵�ѹ���ɼ�����
                                	{
                                    	RES_ING=0;                            
                                	}   
                            	}
                            	else//�ɼ�����æ�ظ�
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
                            	//�궨ָ��
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
            LED0_SetHigh();//�յ����ж���Ϣ       
        }    
        IOCIE=1;//�򿪵�ƽ�ⲿ�ж�
    }
    else if (PIE1bits.TMR2IE == 1 && PIR1bits.TMR2IF == 1)
    {
        TMR2_ISR(); //�ж��Ƿ����ڲ�������
    }
    else if (PIE1bits.TMR1IE == 1 && PIR1bits.TMR1IF == 1) 
    {
        TMR1_ISR(); //���ض�ʱ��1�������SAMPLE_FLAG����ɶ�ʱ�ɼ���������
    }
    else
    {  
        
       
    }
}
/**
 End of File
 */
