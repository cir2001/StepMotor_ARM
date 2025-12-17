#include <stm32f10x.h>
#include "sys.h"
#include "timer.h"
#include "port.h"
#include "usart.h"
#include "stmflash.h"
#include "spi.h"
//========================================================
// 初始化系统TickTime时钟，中断方式
// SYSCLK 系统时钟，MHz
// tick_time TickTime 间隔时间，ms
// 例：systick_init（72，100）， 72MHz系统初始化TickTime时间间隔100ms
//===============================
#define BufferSize 16


extern u8 USART2_RX_BUF[128];     //接收缓冲,最大128个字节.
extern u8 USART2_TX_BUF[128];     //发送缓冲,最大128个字节.
extern u8 USART1_RX_BUF[128];     //接收缓冲,最大128个字节.
extern u8 USART1_TX_BUF[128];     //发送缓冲,最大128个字节.
extern u8 TxD1pt,TxD2pt,TxD1Num,TxD2Num;

u8 spi2_buf[BufferSize]={0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x0d,0x0a};

u8 u8Led1_Counter;

void SysTick_init(u8 SYSCLK,u16 tick_time)
{
u8  tick_us;
u16 tick_ms;
	SysTick->CTRL&=0xfffffff8;    //bit2置0,选择处理器时钟  AHB/8	  72MHz/8=9MHz
	tick_us=SYSCLK/8;             // fac_us=72/8=9		    
	tick_ms=(u16)tick_us*1000;    // fac_ms=9*1000=9000

	SysTick->LOAD=(u32)tick_time*tick_ms;  //时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;         //清空计数器
	SysTick->CTRL=0x03;         //开始倒数  
}						
//========================================================
// 初始化系统TickTime时钟，中断方式
// SYSCLK 系统时钟，MHz
// tick_time TickTime 间隔时间，us
// 例：systick_us_init（72，100）， 72MHz系统初始化TickTime时间间隔100us
//===============================
void SysTick_us_init(u8 SYSCLK,u16 tick_time)
{
u8  tick_us;
	SysTick->CTRL&=0xfffffff8;    //bit2置0,选择处理器时钟  AHB/8	  72MHz/8=9MHz
	tick_us=SYSCLK/8;             // fac_us=72/8=9		    

	SysTick->LOAD=(u32)tick_time*tick_us;  //时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;         //清空计数器
	SysTick->CTRL=0x03;         //开始倒数  
}						
//========================================================
// SysTick 中断服务程序	  可变定时参数，电机加减速控制
//--------------------------------------------

//===============================
void SysTick_Handler(void)
{
//	u8 i;
	u8Led1_Counter++;
	if(u8Led1_Counter == 50)
	{
		//LED1 = !LED1;
		u8Led1_Counter = 0;
	}
//	for(i=0;i<BufferSize;i++)
//	{
//		spi2_buf[i] = SPI2_ReadWriteByte(0xff);
//	}	
//	if(SPI2_ReadWriteByte(0xff) == '#')
//	{
//		for(i=1;i<BufferSize;i++)
//		{
//			spi2_buf[i] = SPI2_ReadWriteByte(0xff);
//		}
//		UART1ComReplyTest();
//		SPI2_ReadWriteByte(0x55);
//	}else return;
		
  
	
}


