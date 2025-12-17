#ifndef __USART_H
#define __USART_H
#include "sys.h"
#include "stdio.h"	 

//如果想串口中断接收，不要注释以下宏定义
#define EN_USART1_RX //使能串口1接收
#define EN_USART2_RX //使能串口2接收

void uart_init1(u32 pclk2,u32 bound);
void uart_init2(u32 pclk2,u32 bound);

void UART1ComReply(void);
void UART2ComReply(void);


#endif	   

