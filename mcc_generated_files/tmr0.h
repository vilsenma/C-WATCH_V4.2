/* 
 * File:   tmr0.h
 * Author: Administrator
 *
 * Created on June 20, 2016, 4:04 PM
 */

#ifndef TMR0_H
#define	TMR0_H

#include <stdbool.h>
#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
#endif   
    
void TMR0_Initialize(void);

void TMR0_StartTimer(void);
 
void TMR0_StopTimer(void);

void TMR0_ISR(void);

void UPDATE_TIM0(void);

#ifdef	__cplusplus
}
#endif

#endif	/* TMR0_H */

