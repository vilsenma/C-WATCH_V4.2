//4.2

/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using MPLAB? Code Configurator

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
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
additional information regarding your rights and obligations.0

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

#include<math.h>
#include "mcc_generated_files/mcc.h"
#include"mcc_generated_files/NRF24L01.h"
#include"main.h"


#define ulong unsigned long

//#define collection

unsigned char collection ; 
unsigned char LED1_COUNT;

long filter_jz[80]=
{  
      490,   975,  1451,  1913,  2357,  2778,  3172,  3536,
      3865,  4157,  4410,  4619,  4785,  4904,  4976,  5000,
      4976,  4904,  4785,  4619,  4410,  4157,  3865,  3536,
      3172,  2778,  2357,  1913,  1451,   975,   490,     0,
      -490,  -975, -1451, -1913, -2357, -2778, -3172, -3536,
     -3865, -4157, -4410, -4619, -4785, -4904, -4976, -5000,
     -4976, -4904, -4785, -4619, -4410, -4157, -3865, -3536,
     -3172, -2778, -2357, -1913, -1451,  -975,  -490,     0,
       490,   975,  1451,  1913,  2357,  2778,  3172,  3536,
      3865,  4157,  4410,  4619,  4785,  4904,  4976,  5000
};

/*�ɼ�������ر���*/
uint8_t  Hz_10_flag=0;//10HZ��־λ
uint8_t  Res_AD_flag=0;//�շŵ��־λ
uint8_t  DAC_flag=0;//DA��ת��־λ
uint8_t  RES_TIME=0;//�����������
uint8_t  RES_FLAG=0;//����ɼ��Ƿ���ɱ�־,�ڶ�ʱ��2�ж�����б��
uint8_t  SAMPLE_FLAG=0;//��ʱ�ɼ������־
uint8_t  TIME_FLAG=0;//ʱ���־�������ڲ��������־
uint32_t TIME_FLAG_num=0;

uint16_t RES_buf[6];
uint16_t RES_temp;

uint16_t  vol_buf;
float     vol_data;

uint32_t  Running=0;

//�궨ֵ�洢����
uint16_t RES_Calibration;
uint16_t VOL_Calibration;
uint16_t TEP_Calibration;

float Res_temp=0;


void main(void) 
{
    
    //����洢�������        
    uint8_t i,j,count;
    uint32_t sum;

    collection = 0;
    LED1_COUNT=0;
  
    // initialize the device 
    SYSTEM_Initialize();//ϵͳ��ʼ��
    
    TMR1_StopTimer();//��ʱ��1��ͣ
    TMR2_StopTimer();//��ʱ��2��ͣ
   
    DAC1_SetOutput(0x00); //DA���0
    
    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    INTERRUPT_IOInterruptEnable();
    
    INTERRUPT_BP4INIT();//�½��ش���
    
    CLRWDT();//ι��
    // Disable the Global Interrupts
    //INTERRUPT_GlobalIn terruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
 
    EEPROOM_INIT(); //��ȡ�궨ֵ
    
    KEY_SetHigh();//�Ͽ��ŵ��·
    
    INTERRUPT_BP4REST();
    NRF24L01_Init();//24L01��ʼ��
    INTERRUPT_BP4INIT();//�½��ش���
    
    while(NRF24L01_Check())//��ⲻ��NRF24L01
    {
        LED0_SetLow(); 
        __delay_ms(500);
        LED0_SetHigh();
        __delay_ms(500);
    }    
    
    INTERRUPT_BP4REST();
    RX_Mode();//����ģʽ
    __delay_ms(1);
    INTERRUPT_BP4INIT();//�½��ش���
    
    LED1_SetHigh(); // ���
    LED0_SetHigh();
    
    while (1) 
    {  
 
        CLRWDT();//ι��
        
        if(RES_FLAG==1)//�ɼ������־
        {
            
            TMR1_StartTimer();
            RES_buf[RES_TIME]=sam_res(); 
            
            if(1==collection) //����궨
            {    
                TUREDATA.Data.CellLRes=RES_buf[RES_TIME];
                TUREDATA.Data.CellHRes=RES_buf[RES_TIME]>>8;
            }
            
            TMR1_StopTimer();
            TMR2_StopTimer();
          
            RES_TIME++;
            
            RES_FLAG=0; 
            DAC1_SetOutput(0x00);//��������Ϳ��Է�������
            
            LED1_SetHigh(); // ���
            LED0_SetHigh();
            KEY_SetHigh(); //�Ͽ��ŵ��·
            
            if(RES_TIME==5)
            {
                RES_TIME=0;
                for(j=0;j<5-1;j++)//������С�����������
                {
                    for(i=0;i<5-j;i++)
                    {
                        if(RES_buf[i]>RES_buf[i+1])
                        {
                            RES_buf[5]=RES_buf[i];
                            RES_buf[i]=RES_buf[i+1];
                            RES_buf[i+1]=RES_buf[5];
                        }
                    }
                }	
                sum=0;
                for(count=1;count<4;count++)//ȥ��ͷβ�����������С��ֵ�����
                {
                    sum+=RES_buf[count];
                    RES_ING=0;//�ɼ����
                } 
                
                RES_temp=sum/3;
             
                TUREDATA.Data.CellLRes=RES_temp;
                TUREDATA.Data.CellHRes=RES_temp>>8;        
                
                TIME_FLAG=0;
            }
            else
            {
                TIME_FLAG=0;
                TMR2_StartTimer();//����10Hz����    
            }
        }
        else//����״̬�ۼ�
        {
            Running++;
         
            //if(Running>562500)//4S
            if(Running>281250)//2S    
            //if(Running>0x100000)
            {
                Running=0;
                LED1_COUNT++;
                if(3<LED1_COUNT)
                {    
                    LED1_SetLow();
                    __delay_ms(10);
                    LED1_SetHigh();
                    LED1_COUNT=0;
                } 
                
                vol_data=Get_ADC(channel_AN1);//���ʵ�ʵ�ѹֵ(��ص�ѹ)
                vol_data=((vol_data*61/10)+0.243);
                vol_data*=10;
                //���б궨����
                vol_data*=VOL_Calibration;
                vol_buf=(uint16_t)vol_data;
            
                TUREDATA.Data.CellLValue=vol_buf;
                TUREDATA.Data.CellHValue=vol_buf>>8;
              
                vol_data=Get_ADC(channel_AN0);//���ʵ�ʵ�ѹֵ(�����ѹ)
                Res_temp=((3.3*5.1/vol_data)-5.1);//���ݵ�ѹ���������ֵȻ������¶�
            
                vol_data=ResToTemp(Res_temp*1000);
                vol_data*=TEP_Calibration;
                vol_data/=10;
                vol_buf=(uint16_t)vol_data;
            
                TUREDATA.Data.CellLTemt=vol_buf;
                TUREDATA.Data.CellHTemt=vol_buf>>8;
                
                TUREDATA.Data.BCC=BCC((uint8_t *)TUREDATA.DataBuffer,9);
                
            }
            
        }          
        
        if(CONFIG_SET!=0)//�궨��ع���
        {
            switch(CONFIG_SET)
            {   
                case 0xAA://�궨��ַ
                    TUREDATA.Data.Addr=CONFIG_BUF[1];
                    DATAEE_WriteByte(ADDR_EEPROM,TUREDATA.Data.Addr); 
                    NRF24L01_SendMSG(RX_BUF); 
                    break;
                case 0xCC://�궨��ѹ
                    vol_data=Get_ADC(channel_AN1);//���ʵ�ʵ�ѹֵ(��ص�ѹ)
                    vol_data=((vol_data*61/10)+0.243);
                    vol_data*=10;
                    //���б궨����
                    
                    VOL_Calibration=(uint16_t)((CONFIG_BUF[1]+(CONFIG_BUF[0]<<8))/vol_data);
                    DATAEE_WriteByte(VOL_HEEPROM,VOL_Calibration>>8);
                    DATAEE_WriteByte(VOL_LEEPROM,VOL_Calibration);
                    NRF24L01_SendMSG(RX_BUF);
                    break;
                case 0xDD://�궨�¶�
                    
                    vol_data=Get_ADC(channel_AN0);//���ʵ�ʵ�ѹֵ(�����ѹ)
                    Res_temp=((3.3*5.1/vol_data)-5.1);//���ݵ�ѹ���������ֵȻ������¶�
            
                    vol_data=ResToTemp(Res_temp*1000);
                    vol_data/=10;
                                      
                    TEP_Calibration=(uint16_t)((CONFIG_BUF[1]+(CONFIG_BUF[0]<<8))/vol_data);
                                       
                    DATAEE_WriteByte(TEP_HEEPROM,TEP_Calibration>>8);
                    DATAEE_WriteByte(TEP_LEEPROM,TEP_Calibration);
                    NRF24L01_SendMSG(RX_BUF);
                    break;
                case 0xEE:
                    collection=1;    
                    break;
                    
                case 0xFF://�궨����
                    RES_Calibration=(CONFIG_BUF[1]+(CONFIG_BUF[0]<<8));
                    DATAEE_WriteByte(RES_HEEPROM,RES_Calibration>>8);
                    DATAEE_WriteByte(RES_LEEPROM,RES_Calibration); 
                    NRF24L01_SendMSG(RX_BUF);
                    break;
                default://����������
                    break;
            } 
            CONFIG_SET=0;
        }    
    }
}


uint16_t sam_res(void)//����ɼ��������
{
    uint8_t k;
    uint32_t count,num,res;
    
    long p1=0,p2=0,p3=0,r1[4],r2[4];

    SAMPLE_FLAG=0;
    for(k=0;k<4;k++)
    { 
        for(count=0,p1=0,p2=0,p3=0;count<2048;count++)
        {
            for(;SAMPLE_FLAG==0;) 
            {
            
            }
            SAMPLE_FLAG=0;
            num=ADC_GetConversion(channel_AN4);
            num>>=4;
            p1+=(filter_jz[count%64])*(long)num;
            p2+=(filter_jz[count%64+16])*(long)num;
        }
        r1[k]=p1;
        r2[k]=p2; 

    }

	for(k=0;k<4;k++)
	{
		r1[k]/=130000;
        r2[k]/=130000;
		r1[k]*=r1[k];
        r2[k]*=r2[k];
        r1[k]+=r2[k];
    }
	for(k=0,p1=0,p2=r1[0],p3=r1[0];k<4;k++)
	{
		p1+=r1[k];
		if(p2>r1[k])
            p2=r1[k];
		if(p3<r1[k])
            p3=r1[k];
	}      
    p1-=p2+p3;
	
	res=srt(p1);
    
    res=res*RES_Calibration*13/8192;
    
    //res=res*RES_Calibration*130/8192;
    
    //return (uint32_t)((30575*(long)i)/(8192));
    return res;
    
}


int srt(long k)
{
    int j=0,l;    
    long i;
    if(k>=0) 
    for(l=0x4000;l!=0;l=l>>1)
    {
        j|=l;
        i=(long)j;
        i*=(long)j;
        if(i>k)
        {
            j=j&~l;
        }   
        else if(i==k) 
            break;
    }
    else j=0x7fff;
    return j;
}




float Get_ADC(adc_channel_t channel)
{
	uint8_t count,i,j;
	uint16_t temp;
	uint16_t value_buf[15];
	uint32_t sum=0;
    float result=0;
	
    for(count=0;count<15;count++)//�����ɼ�15����ֵ
	{
        value_buf[count]=(ADC_GetConversion(channel)>>4);
    }
    for(j=0;j<14;j++)//������С�����������
	{
        for(i=0;i<15-j;i++)
		{
            if(value_buf[i]>value_buf[i+1])
			{
                temp=value_buf[i];
                value_buf[i]=value_buf[i+1];
                value_buf[i+1]=temp;
            }
        }
    }	
	for(count=6;count<9;count++)//ȥ��ͷβ�����������С��ֵ�����
    {
        sum+=value_buf[count];
    }
    
    result=(((sum/3)*2.5)/4095);//ȡƽ��ֵ��ת����ʵ��ֵ
    
    return result;
    //return(uint16_t)(sum/3);//ȡƽ��ֵ
}




void EEPROOM_INIT(void)
{
    //��ȡ��ַ
    TUREDATA.Data.Addr=DATAEE_ReadByte(ADDR_EEPROM);
    if(TUREDATA.Data.Addr==0xFF||TUREDATA.Data.Addr==0x00)
    {
        TUREDATA.Data.Addr=0x01;
        DATAEE_WriteByte(ADDR_EEPROM,TUREDATA.Data.Addr);
    }   
    
    //��ȡ����궨ֵ
    RES_Calibration=DATAEE_ReadByte(RES_HEEPROM);
    RES_Calibration<<=8;
    RES_Calibration+=DATAEE_ReadByte(RES_LEEPROM);
    
    if(RES_Calibration==0xFFFF||RES_Calibration==0x0000)
    {
        RES_Calibration=500;
        DATAEE_WriteByte(RES_HEEPROM,RES_Calibration>>8);
        DATAEE_WriteByte(RES_LEEPROM,RES_Calibration);      
    }    
    //��ȡ��ѹ�궨ֵ
    VOL_Calibration=DATAEE_ReadByte(VOL_HEEPROM);
    VOL_Calibration<<=8;
    VOL_Calibration+=DATAEE_ReadByte(VOL_LEEPROM);
    
    if(VOL_Calibration==0xFFFF||VOL_Calibration==0x0000)
    {
        VOL_Calibration=100;
        DATAEE_WriteByte(VOL_HEEPROM,VOL_Calibration>>8);
        DATAEE_WriteByte(VOL_LEEPROM,VOL_Calibration);
    } 
    
    //��ȡ�¶ȱ궨ֵ
    TEP_Calibration=DATAEE_ReadByte(TEP_HEEPROM);
    TEP_Calibration<<=8;
    TEP_Calibration+=DATAEE_ReadByte(TEP_LEEPROM);
    
    
    if(TEP_Calibration==0xFFFF||TEP_Calibration==0x0000)
    {
        TEP_Calibration=100;

        DATAEE_WriteByte(TEP_HEEPROM,TEP_Calibration>>8);
        DATAEE_WriteByte(TEP_LEEPROM,TEP_Calibration);
        
    }
    
}   

/*�����Ӧ��ֵ����λ��ŷķ������������¶ȵ�10��,������֯����120K����Ϊû�в��봫����*/
float ResToTemp(float res)
{

    //���㹫ʽ  R=R��25�棩*exp[B*(1/t-1/298.15)]    B=3980  R(25��)=10000��10K���裩
    
    float t=0;
    
    //exp(3980*(1/t-1/298.15))=res/10000;
    //1/t-1/298.15=log(res/10000)/3980;
    //1/t=log(res/10000)/3980+1/298.15;
    
    if(res>120000)
    {
        t=0;
    }
    else
    {    
        //t=1/((log((float)(res/10000))/3980)+(1/298.15));
        t=1/((log((float)(res/10000))/3450)+(1/298.15));
        //t=(t-273)*10;
        t-=273;
    }
    
    return t;
   
}

/*���У�����*/
uint8_t BCC(uint8_t *buffer ,uint8_t size)
{
    uint8_t i,result=0;
    
    for(i=0;i<size;i++)
    {
        result^=*buffer;
        buffer++;
    }    
    return result;
}   


void NRF24L01_SendMSG(uint8_t *MSG)
{
    uint8_t state;
    
    INTERRUPT_BP4REST();//�ر��ж�        
    TX_Mode();//����ģʽ
     __delay_ms(1);
    state=NRF24L01_TxPacket(MSG);  

    if(state==0xff)
    {
        NRF24L01_Init();//24L01��ʼ��
    }  
    RX_Mode();//����ģʽ
    __delay_ms(1);
    INTERRUPT_BP4INIT();//���ж�

}        


