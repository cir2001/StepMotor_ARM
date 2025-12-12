////////////////////////////////////////////////////////////
//  Project Name: stepMotor stm32f103C6t6程序  自制PCB V0.1
//	File Name: stepMotor.C	 
//  芯片：STM32F103C6T6A	
//  version: V0.1
//	Author: ZJ
//	At:	xi'an China
//  Date From:202501030
//	**********************************
//	功能：
//		1.42 stepmotor control drv8825 module
//		2.CAN send receive
//		3.0.96oled display
//		4.uart2 send receive PC		
//		5.AS5600 read angle
// 	date		comment
//  2025-10-30  初始化
//	2025-11-12	VScode开发
//	2025-11-20	添加CAN收发功能
//	2025-12-05	添加AS5600读角度功能
//*********** 引脚连接表 ***********************//
//	0.96OLED(SD1306)SPI1		stm32f103
//		GND						GND
//		Vcc						3.3V
//		D0	（SPI时钟线SCK）	 PA5
//		D1	（SPI数据线SDO）	 PA7
//		RES （复位）			 PC13
//		DC  （数据或命令切换）	  PC14	OLED_A0
//		CS   (片选)				 PA4   OLED_CS
//----------------
//	drv8825 modual              stm32f103
//		ENA						PA9
//		STEP					PB13
//		DIR						PB12
//		M0						PA8
//		M1						PB15
//		M2						PB14
//---------------
//	 AS5600 IIC 				stm32f103
//		GND						GND
//		Vcc						3.3V
//		SCL						PB10
//		SDA						PB11
//      DIR                     PB0
//      OUT					 	PA0
//      GPO 					PB1
//---------------
//	 CAN						stm32f103
//		CTX						PA12
//		CRX						PA11
//==================================================
//说明：
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
#include "AS5600.h"
//-------------------
#define BufferSize 16
//--------------------
//设置FLASH 保存地址(必须为偶数，且其值要大于本代码所占用FLASH的大小+0X08000000)
#define FLASH_SAVE_ADDR  0X08010000 
#define BufferSize 16
//--------------------------
char hex2char(u8 value);
//---------------------
u16 FlashRead[32];
extern u8 USART2_RX_BUF[128];     	//接收缓冲,最大128个字节.
extern u8 USART2_TX_BUF[128];     	//发送缓冲,最大128个字节.
extern u8  USART1_RX_BUF[128];     	//接收缓冲,最大128个字节.
extern u8  USART1_TX_BUF[128];     	//发送缓冲,最大128个字节.
extern u8 TxD1pt,TxD2pt,TxD1Num,TxD2Num;

extern u8 EXIT0_flag;
extern u8 key_press;

extern u8 CAN_RX_Flag;
extern u8 CAN_RX_BUF[8];

extern u16 CAN_RX_ID;
//=====================================================
short Gyro[3],Acc[3];

u16 res,u8Led0_Counter;

u8 temp_1,temp_2,temp_3;
u8 i;
u8 canbuf[8]={0x05,0x01,0x05,0x01,0x05,0x05,0x06,0x07};
u8 key;
u8 canRXbuf[8];
u8 u8can,u8can1,u8can2;
unsigned char h2c_id[3],h2c[8][2];
//==========================================================
int main(void)
{
	float angle_deg;
    u16 raw_angle;
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
	OLED_ShowString(0, 0,  "LY-STM32");
	OLED_ShowString(0, 24, "Count:");
	
	Motor_M0 = 1;  
	Motor_M1 = 1;
	Motor_M2 = 0;
	//Motor_Sleep = 1;
	Motor_Ena = 0;
	Motor_Dir = 1;


//*********** 主循环程序 ****************//
	while(1)
	{	
		u8Led0_Counter++;

		if(u8Led0_Counter == 1000)
		{
			res++;
			u8Led0_Counter = 0;
			raw_angle = AS5600_GetRawAngle();
        
			// 转换公式: 360度 / 4096步 = 0.08789
			angle_deg = raw_angle * 0.087890625f;

			// --- 准备 OLED 显存 (GRAM) ---
			// 1. 清除显存 (清除上一帧的数字，防止重叠)
			//OLED_Clear(); 
			
			// 2. 画静态文字
			//OLED_ShowString(0, 0,  "LY-STM32");
			//OLED_ShowString(0, 24, "Count:");
			
			// 3. 画动态变量
			// 参数：x, y, 数字变量, 位数, 字体大小
			OLED_ShowNum(56, 24, res, 8, 16); 
			//OLED_ShowNum(32, 0, angle_deg, 4, 16);
			
			// --- 第三步：核心！将显存刷到屏幕 ---
			OLED_Refresh_Gram(); 
			
			// --- 第四步：控制刷新率 ---
			delay_ms(10); // 约 20fps，人眼看着舒服，且数字变化能看清

			if(res == 100)
			{
				LED0 = !LED0;
				LED1 = !LED1;
				res = 0;
				//USART2->DR = 0x32;
				//res=CAN_Send_Msg(canbuf,8);				//发送8个字节 
				//if(res)OLED_ShowString(0,0,"Failed");	//提示发送失败
				//else OLED_ShowString(0,16,"OK");	//提示发送成功				
			}
		}
		//key=CAN_Receive_Msg(canRXbuf);
		if(key)//接收到有数据
		{			
			OLED_Clear();
 			for(i=0;i<key;i++)
			{									    
				OLED_ShowNum(i*8,40,canRXbuf[i],1,16);	//显示数据
 			}
		}
		if(CAN_RX_Flag == 1)
		{
			CAN_RX_Flag = 0;
			temp_1 = (CAN_RX_ID >> 8) & 0x0F;   
			temp_2 = (CAN_RX_ID >> 4) & 0x0F; 
			temp_3 = CAN_RX_ID & 0x0F; 

			h2c_id[0] = hex2char(temp_1);  
			h2c_id[1] = hex2char(temp_2);  
			h2c_id[2] = hex2char(temp_3); 
			//--- 第一行显示 ---
			OLED_ShowString(0,0,"rec_id:");
			for(u8can=0;u8can<3;u8can++)
			{	
				OLED_ShowChar(u8can*12+60,0,h2c_id[u8can],16,1);	//显示ID
			}
			//--- 第二行显示 ---
			OLED_ShowString(0,16,"REC:");		
			//接收数据转换
			for(u8can=0;u8can<8;u8can++)
			{
				temp_1 = (CAN_RX_BUF[u8can] >> 4) & 0x0F;   
				temp_2 = CAN_RX_BUF[u8can] & 0x0F; 

				h2c[u8can][0] = hex2char(temp_1);  
				h2c[u8can][1] = hex2char(temp_2);  
			}
			//--- 第三行显示 ---
			for(u8can1=0;u8can1<4;u8can1++)
			{
				for(u8can2=0;u8can2<2;u8can2++)
				{
					OLED_ShowChar(u8can1*30+u8can2*12,32,h2c[u8can1][u8can2],16,1);	//显示前四个字节
				}
			}
			//--- 第四行显示 ---
			for(u8can1=4;u8can1<8;u8can1++)
			{
				for(u8can2=0;u8can2<2;u8can2++)
				{
				
					OLED_ShowChar((u8can1-4)*30+u8can2*12,48,h2c[u8can1][u8can2],16,1);	//显示后四个字节					
				}			
			}
		}
	}   // while(1) end
}		// int main(void) end
//*************************************************
//
//
//
//*************************************************
char hex2char(u8 value)
{
    if (value < 10)
        return '0' + value;
    else
        return 'A' + (value - 10);
}
//*************************************************
//
//
//
//*************************************************

