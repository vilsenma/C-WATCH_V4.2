#include <xc.h>
#include "tmr1.h"
#include "tmr0.h"
#include "mcu_eusart.h"
#include "pin_manager.h"


/**********************************************
 * 
 *TIME0 初始化
 * 
 *********************************************/
void TMR0_Initialize(void) {
  
  
    OPTION_REGbits.TMR0CS=0;    //FOSC/4 4MHz 选择内部周期
    //OPTION_REGbits.TMR0SE=0;  
    OPTION_REGbits.PSA=0;       //分频设置
    OPTION_REGbits.PS=1;        //000~111 = 2；4；8；16；32；64；128；256 分频设置
    
    
    INTCONbits.TMR0IE=0;    //关闭TMR0中断  
    //INTCONbits.IOCIF=0;//清除TMR0中断标志位
    
    //TMR0bits.TMR0=0xA3;//设置中断初始值

    
    
    
}


void TMR0_StartTimer(void) {
    INTCONbits.TMR0IF=0;
    TMR0bits.TMR0=0xa9;
    INTCONbits.TMR0IE=1;    //使能TMR0中断
}

void TMR0_StopTimer(void) {
    INTCONbits.TMR0IE=0;    //关闭TMR0中断
}


void TMR0_ISR(void) {
    
    INTCONbits.TMR0IF=0;//清除TMR0中断标志位

    if(1==MCU_eusart_State)
    {    
        SendState=1;
    }
    
    if(2==MCU_eusart_State)
    {    
        ReviceState=1; 
        //LATC2=~LATC2;
    }
    
    if(MCU_eusart_State==3)
    {
        LATC2=~LATC2;
    }    
    
    TMR0bits.TMR0=0xA9;
    
}

void UPDATE_TIM0(void){
    
    INTCONbits.TMR0IF=0;
    TMR0=0xa9;
}
