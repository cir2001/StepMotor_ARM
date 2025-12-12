#include "timer.h"
#include "port.h"
#include "usart.h"
#include "delay.h"
//==========================================


//*******************************************
extern u8 USART1_TX_BUF[128],USART2_TX_BUF[128];
extern u8 TxD1pt,TxD2pt,TxD1Num,TxD2Num;
int ii=0;

u16 t1,t2;
u32 t,p;

//**** Timer2 IRQHandler **********************************  	 
//===============================
//定时2中断服务程序	 未使用
//===============================  	 
void TIM2_IRQHandler(void)
{
	if(TIM2->SR&0X0001)//溢出中断
	{
		t2++;
		t1++;
		t++;
		if(t2>=2000)
		{
			t2 = 0;
			LMain = !LMain;		
		}
		if(t1>=15)
		{
			t1 = 0;
			//Motor_Step = !Motor_Step;
			Motor_Step = 0;
		}
		
		if(t>=3000)
		{
			t = 0;
			p += 200;
			if(p>1000)
			{
				p=0;
			}
			//TIM2->ARR = p+9;
		}
	}
	TIM2->SR&=~(1<<0);//清除中断标志位 	    
}
//**** Timer2 IRQHandler **********************************  	 
//===============================
//定时3中断服务程序	 未使用
//===============================  	 
void TIM3_IRQHandler(void)
{
	if(TIM3->SR&0X0001)//溢出中断
	{
	
	}
	TIM3->SR&=~(1<<0);//清除中断标志位 	    
}
//**** Timer2 IRQHandler **********************************  	 
//===============================
//定时4中断服务程序	 未使用
//===============================  	 
void TIM4_IRQHandler(void)
{
	if(TIM4->SR&0X0001)//溢出中断
	{
	
	}
	TIM4->SR&=~(1<<0);//清除中断标志位 	    
}
//=======Timer2 Init ============
//通用定时器初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//===============================
void Timer2_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<0;//TIM2时钟使能    
 	TIM2->ARR=arr;     //设定计数器自动重装值   
	TIM2->PSC=psc;     //预分频器7200,得到10Khz的计数时钟
	//这两个东东要同时设置才可以使用中断
	TIM2->DIER|=1<<0;   //允许更新中断				
	TIM2->DIER|=1<<6;   //允许触发中断	   
 	MY_NVIC_Init(1,3,TIM2_IRQn,2);//抢占1，子优先级1，组2									 
	TIM2->CR1|=0x01;    //使能定时器2
	//TIM2->CR1&=0xfffe;    //禁止定时器2
}





