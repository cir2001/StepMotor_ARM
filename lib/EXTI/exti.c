#include "exti.h"
#include "delay.h" 
#include "port.h"
//////////////////////////////////////////////////////////////////////////////////	 
//
//							  
//////////////////////////////////////////////////////////////////////////////////   

u16 u16Exti0Count;

u8 EXIT0_flag;

//外部中断2服务程序
void EXTI2_IRQHandler(void)
{
	
	EXTI->PR=1<<2;  //清除LINE2上的中断标志位  
}
//外部中断3服务程序
void EXTI3_IRQHandler(void)
{
	
	EXTI->PR=1<<3;  //清除LINE3上的中断标志位  
}
//外部中断4服务程序
void EXTI4_IRQHandler(void)
{
	
	EXTI->PR=1<<4;  //清除LINE4上的中断标志位  
}		   
//外部中断初始化程序
//初始化PA0/PE2/PE3/PE4为中断输入.
void EXTIX_Init(void)
{
//	Ex_NVIC_Config(GPIO_A,0,FTIR); 	//下升沿触发 PA0
	Ex_NVIC_Config(GPIO_B,0,FTIR); 	//下升沿触发 PB0
	
	
	MY_NVIC_Init(2,2,EXTI0_IRQn,2);	//抢占2，子优先级3，组2
//	MY_NVIC_Init(2,1,EXTI1_IRQn,2);	//抢占2，子优先级3，组2
}

//===============================


//==================================================================
//外部中断0服务程序
void EXTI0_IRQHandler(void)
{
	EXTI->PR=1<<0;  //清除LINE0上的中断标志位  
	
	EXIT0_flag = 1;
	
	u16Exti0Count++;
	if(u16Exti0Count>=50)
	{
		u16Exti0Count = 0;

	}
}
//===============================


//==================================================================
//外部中断1服务程序
void EXTI1_IRQHandler(void)
{
	EXTI->PR=1<<1;  //清除LINE0上的中断标志位  
	u16Exti0Count++;
	if(u16Exti0Count>=50)
	{
		u16Exti0Count = 0;

	}	
}










