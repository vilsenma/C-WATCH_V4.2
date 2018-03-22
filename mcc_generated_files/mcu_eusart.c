#include "mcu_eusart.h"
#include "pin_manager.h"


/*****************************
*�������ƣ�MCU_Send_Word
*��    �룺Ҫ���͵��ֽ�
*��    ������
*��    �ܣ����ڷ��͵����ֽ�
******************************/

void MCU_Send_Word(uint8_t word)
{
        uint8_t count,temp;
        temp=word;
        MCU_eusart_State=1;
        
        LATC4=0;//��ʼλ
        SendState=0;
        //TMR0_StartTimer();
   
        while(!SendState);
        SendState=0;
        
        for(count=0;count<8;count++)
        {
            LATC4=(temp&0x01);

            while(!SendState);
            SendState=0;
            temp=temp>>1;
            
        }
        LATC4=1;//ֹͣλ
        while(!SendState);
        SendState=0;
        
        //TMR0_StopTimer();
        MCU_eusart_State=0;
}

/*****************************
*�������ƣ�MCU_Send_Msg
*��    �룺Ҫ���͵��ַ���
*��    ������
*��    �ܣ����ڷ����ַ���
******************************/

void MCU_Send_Msg(uint8_t *msg,uint8_t size)
{
    uint8_t count;
    for(count=0;count<size;count++)
    {
        MCU_Send_Word(*msg);
        msg++;
    }    
       
}



/*****************************
*�������ƣ�receive
*��    �룺��
*��    ���������ֽ�
*��    �ܣ����ڽ��յ����ֽ�
******************************/

unsigned char  receive(void)
{
    unsigned char j;
    unsigned char newrec_data=0;
               
    MCU_eusart_State=2;     //����״̬
    
        ReviceState=0;
        //TMR0_StartTimer();
    
        while(!ReviceState)
        ReviceState=0;
        
        //�ܷ�������
    /*
        for(j=0;j<8;j++)
        {   
             //LATC2=~LATC2;
            ReviceState=0;
            while(!ReviceState)
            ReviceState=0;
        
            if(!RC5)    
            {        
                newrec_data|=0x80;
 
            }
            else
            {
                //newrec_data&=0x7f;
                
            }
           
            newrec_data=newrec_data>>1;
        }
    */    
/*
        while(1==RC5)
        {
            while(!ReviceState)
            ReviceState=0;
            break;        
        }
*/       
    while(1)
    {
        ;
        //while(!ReviceState)
        //ReviceState=0;
        //LATC2=~LATC2;
    }        
    
    
    //TMR0_StopTimer();
    MCU_eusart_State=0;
    
    return newrec_data;        
}

