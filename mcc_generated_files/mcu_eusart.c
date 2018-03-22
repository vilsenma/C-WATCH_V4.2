#include "mcu_eusart.h"
#include "pin_manager.h"


/*****************************
*函数名称：MCU_Send_Word
*输    入：要发送的字节
*输    出：无
*功    能：串口发送单个字节
******************************/

void MCU_Send_Word(uint8_t word)
{
        uint8_t count,temp;
        temp=word;
        MCU_eusart_State=1;
        
        LATC4=0;//起始位
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
        LATC4=1;//停止位
        while(!SendState);
        SendState=0;
        
        //TMR0_StopTimer();
        MCU_eusart_State=0;
}

/*****************************
*函数名称：MCU_Send_Msg
*输    入：要发送的字符串
*输    出：无
*功    能：串口发送字符串
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
*函数名称：receive
*输    入：无
*输    出：单个字节
*功    能：串口接收单个字节
******************************/

unsigned char  receive(void)
{
    unsigned char j;
    unsigned char newrec_data=0;
               
    MCU_eusart_State=2;     //接受状态
    
        ReviceState=0;
        //TMR0_StartTimer();
    
        while(!ReviceState)
        ReviceState=0;
        
        //能发不能收
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

