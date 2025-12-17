#ifndef __SYSTICK_H
#define __SYSTICK_H 			   
#include <stm32f10x.h>

void SysTick_init(u8 SYSCLK,u16 tick_timer);
void SysTick_us_init(u8 SYSCLK,u16 tick_timer);

#endif




