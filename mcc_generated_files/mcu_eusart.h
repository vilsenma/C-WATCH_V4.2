/* 
 * File:   mcu_eusart.h
 * Author: Administrator
 *
 * Created on June 20, 2016, 3:38 PM
 */

#ifndef MCU_EUSART_H
#define	MCU_EUSART_H

#include <xc.h>
#include <stdbool.h>
#include <stdint.h>



#ifdef	__cplusplus
extern "C" {
#endif

    
#define  SIZE 20
uint8_t BitCount=0,RecMSG;
uint8_t MCU_eusart_State=0;         //0为无，1为发送状态，2为接受状态。  
uint8_t SendState=0,ReviceState=0;
uint8_t MCU_SendBuff[SIZE];
uint8_t MCU_SendBuff[SIZE];



void MCU_Send_Word(uint8_t word);

void MCU_Send_Msg(uint8_t *msg,uint8_t size);

uint8_t MCU_Revice(void);

unsigned char  receive(void);

#ifdef	__cplusplus
}
#endif

#endif	/* MCU_EUSART_H */

