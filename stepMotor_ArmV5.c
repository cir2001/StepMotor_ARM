////////////////////////////////////////////////////////////
//  Project Name: stepMotor stm32f103C6t6程序  自制PCB
//	File Name: stepMotor.C	 
//  芯片：STM32F103C6T6A	
//  version: V0.0
//	Author: ZJ
//	At:	xi'an China
//  Date From:202501030
//	**********************************
//	功能：
//		42 stepmotor control drv8825 module
//		CAN send receive
//		0.96oled display
//		uart2 send receive PC		
//		
// 	date		comment
//  2025-10-30  初始化
//	2025-11-12	VScode开发
//
//*********** 引脚连接表 ***********************//
//	0.96OLED(SD1306)SPI1		stm32f103
//		GND						GND
//		Vcc						3.3V
//		D0	（SPI时钟线）		 PA5
//		D1	（SPI数据线）		 PA7
//		RES （复位）			 PC13
//		DC  （数据或命令切换）	  PC14
//		CS   (片选)				 PA4
//----------------
//	drv8825 modual              stm32f103
//		EN						PA9
//		STEP					PB13
//		DIR						PB12
//		M0						PA8
//		M1						PB15
//		M2						PB14
//==================================================
//说明：
//	
//---- UART2 ------
// UART2   115200，1start，8bit，1stop，no parity  	未使用
//******************************************************//
#include <stm32f10x.h>
#include "usart.h"		
#include "port.h"
#include "timer.h"
#include "stmflash.h"
#include "delay.h"

#include "exti.h" 
#include "SysTick.h"
#include "stdint.h"			//定义bool变量需要加入这行
#include "stdbool.h"		//定义bool变量需要加入这行

#include "math.h"

#include "spi.h"
#include "oled.h"
#include "can.h"
//-------------------
#define BufferSize 16
//--------------------
//设置FLASH 保存地址(必须为偶数，且其值要大于本代码所占用FLASH的大小+0X08000000)
#define FLASH_SAVE_ADDR  0X08010000 
#define BufferSize 16
//---------------------
u16 FlashRead[32];
extern u8 USART2_RX_BUF[128];     	//接收缓冲,最大128个字节.
extern u8 USART2_TX_BUF[128];     	//发送缓冲,最大128个字节.
extern u8  USART1_RX_BUF[128];     	//接收缓冲,最大128个字节.
extern u8  USART1_TX_BUF[128];     	//发送缓冲,最大128个字节.
extern u8 TxD1pt,TxD2pt,TxD1Num,TxD2Num;

extern u8 EXIT0_flag;
extern u8 key_press;
//=====================================================
short Gyro[3],Acc[3];

u16 res,u8Led0_Counter;

u8 i;
u8 canbuf[8]={0x05,0x01,0x05,0x01,0x05,0x05,0x06,0x07};
u8 key;
u8 canRXbuf[8];
//==========================================================
int main(void)
{
//****** 初始化 *******//
	Stm32_Clock_Init(9);     		//系统时钟设置
	
	PORT_Init();					//IO端口初始化
	delay_ms(200);
	
	SPI1_Init();					//0.96OLED初始化
	delay_ms(200);
	OLED_Init();
	delay_ms(200);
	
	uart_init2(36,57600);	 		//串口2初始化为57600 APB1/2预分频（与上位机通讯）
	delay_ms(200);
	
	Timer2_Init(99,71); 			//1ms
	delay_ms(200);
	
	CAN_Mode_Init();				//CAN初始化,波特率500Kbps    
	delay_ms(200);
	
	JTAG_Set(SWD_ENABLE);	 		//设置成SWD调试模式，关闭JTAG模式
	delay_ms(200);
//******** 变量初始化 **********//
	u8Led0_Counter = 0;
	
	key_press = 0;
	
	//OLED_ShowNum(0, 0, 12345678, 8, 16);
	//OLED_ShowString(1,16,"OLED TEST");
	//OLED_Clear();
	//OLED_ShowString(0,0," LX: ");
	//OLED_ShowString(0,16," LY: ");
	//OLED_ShowString(0,32," RX: ");
	//OLED_ShowString(0,48," RY: ");
	
	Motor_M0 = 1;  
	Motor_M1 = 1;
	Motor_M2 = 0;
	//Motor_Sleep = 1;
	Motor_Ena = 0;
	
	Motor_Dir = 1;

	LCan = 1;
//*********** 主循环程序 ****************//
	while(1)
	{	
		u8Led0_Counter++;

		if(u8Led0_Counter == 1000)
		{
			res++;
			u8Led0_Counter = 0;
			if(res == 100)
			{
				LMain = !LMain;
				res = 0;
				//USART2->DR = 0x32;
				res=CAN_Send_Msg(canbuf,8);				//发送8个字节 
				if(res)OLED_ShowString(0,0,"Failed");	//提示发送失败
				else OLED_ShowString(0,16,"OK");	//提示发送成功				
			}
		}
		
		key=CAN_Receive_Msg(canRXbuf);
		if(key)//接收到有数据
		{			
			OLED_Clear();
			LCan = !LCan;
 			for(i=0;i<key;i++)
			{									    
				OLED_ShowNum(i*8,40,canRXbuf[i],1,16);	//显示数据
 			}
		}
	}   // while(1) end
}		// int main(void) end

