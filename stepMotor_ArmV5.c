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
#include <stdlib.h>
#include "math.h"

#include "spi.h"
#include "oled.h"
#include "can.h"
#include "AS5600.h"
#include "stepMotor.h"
//-------------------
#define BufferSize 16
//--------------------
//设置FLASH 保存地址(必须为偶数，且其值要大于本代码所占用FLASH的大小+0X08000000)
#define FLASH_SAVE_ADDR  0X08010000 
#define BufferSize 16

// ===================================================================
//  全局变量
// ===================================================================
// 闭环控制相关
extern int64_t Theory_Pos_Scaled; // 理论位置 (放大版)
extern int64_t Actual_Pos_Scaled; // 实际位置 (放大版)
extern int32_t Position_Error; // 误差 (实际编码器单位)
extern u16 Last_AS5600_Raw;           // 上次编码器读数

// 速度控制相关
extern int32_t Target_Speed_Hz;   // 用户设定的目标速度
extern int32_t Real_Output_Hz;   // 实际发给电机的速度 (包含补偿)

// 调试/显示相关
volatile float Display_RPM = 0.0f;
volatile float Display_Angle = 0.0f;

volatile int32_t User_Cmd_Speed = 0;   // 串口发来的最终目标
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
extern u8 u8Uart2_flag;

extern u16 CAN_RX_ID;
extern int recv_uart2_val;
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

u32 oled_tick;
//==========================================================
int main(void)
{
	int i;
	//int32_t target_speed = 0;  // 目标速度 (来自串口)
    int32_t current_speed = 0; // 当前实际速度
	int show_rpm;
	int show_ang;
//****** 初始化 *******//
	Stm32_Clock_Init(9);     		//系统时钟设置
	
	PORT_Init();					//IO端口初始化
	delay_ms(200);
	
	SPI1_Init();					//0.96OLED初始化
	delay_ms(200);
	OLED_Init();
	delay_ms(200);
	
	uart_init2(36,115200);	 		//串口2初始化为57600 APB1/2预分频（与上位机通讯）
	delay_ms(200);

	//Timer2_Init(999,71);				//定时器2初始化 1ms
	Timer2_Init(4999,71);				//定时器2初始化 5ms
	delay_ms(200);

	StepMotor_Init();				//步进电机初始化
	delay_ms(200);
	
	CAN_Mode_Init();				//CAN初始化,波特率500Kbps    
	delay_ms(200);
	
	JTAG_Set(SWD_ENABLE);	 		//设置成SWD调试模式，关闭JTAG模式
	delay_ms(200);
//******** 变量初始化 **********//
	u8Led0_Counter = 0;
	key_press = 0;

	// OLED 静态显示 (画表格、写固定的字)
    // 这些字只写一次，循环里不要重复刷，提高效率
    OLED_Clear();
    /*OLED_ShowString(0, 0,  "MODE: Velocity");
    OLED_ShowString(0, 24, "RPM :");
    OLED_ShowString(0, 48, "Ang :");*/

	OLED_ShowString(0, 0, "Mode: ClosedLoop");
	OLED_ShowString(0, 16, "Target:");
    OLED_ShowString(0, 32, "Err :");
    OLED_ShowString(0, 48, "Spd :");

    OLED_Refresh_Gram(); // 第一次刷显存
	// 简单的加速启动示例 
	/*for( i = 1000; i <= 25000; i += 100) 
	{
		StepMotor_SetSpeed(i);
		delay_ms(5); // 给电机一点反应时间
	}*/
	// 加速完后，它就会一直保持 25000 转
	// 读取一次初始角度，防止上电第一帧数据跳变
    Last_AS5600_Raw = AS5600_GetRawAngle(); 

    // 4. 设定一个初始速度进行测试
    // 目标：2000Hz (约 0.6圈/秒)
    //Target_Speed_Hz = 2000;
//*********** 主循环程序 ****************//
	while(1)
	{	
		// ============================
        // 主循环：处理紧急任务
        // ============================
		// 1. 检查是否有新指令
        if (u8Uart2_flag)
        {
			// 从串口2接收新指令
            User_Cmd_Speed = recv_uart2_val;
            // 清除标志位
            u8Uart2_flag = 0;
			//StepMotor_SetSpeed(User_Cmd_Speed);
			Target_Speed_Hz = User_Cmd_Speed;
        }
		// ============================
        // 主循环：处理非紧急任务
        // ============================
        // delay_ms(1) 是整个主循环的心跳
        delay_ms(1); 
        oled_tick++;
        // 每 100ms 刷新一次屏幕 (1ms * 100 = 100ms)
        if (oled_tick >= 1000) 
        {
            oled_tick = 0; // 清零计时器

            // --- A. 准备数据 ---
            // 为了显示稳定，可以把浮点数转成整数，或者保留1位小数
          	/*show_rpm = (int)Target_Speed_Hz;
            show_ang = (int)Last_AS5600_Raw;

			//show_rpm = (int)1500;
            //show_ang = (int)355;

            // --- B. 写入显存 (只更新变化的数字) ---
            // 显示转速 (在 x=48, y=24 的位置)
            // 这里的 " " 是为了覆盖掉上一帧可能残留的多余位数
            // 比如 1000 变 9，不清除就会变成 9000
            OLED_ShowString(48, 24, "      "); // 先清除数字区域
            if(show_rpm < 0) {
                 OLED_ShowString(48, 24, "-");
                 OLED_ShowNum(56, 24, -show_rpm, 6, 16);
            } else {
					OLED_ShowString(48, 24, "+");
                 OLED_ShowNum(56, 24, show_rpm, 6, 16);
            }

            // 显示角度 (在 x=48, y=48 的位置)
            OLED_ShowString(48, 48, "      "); // 先清除
            OLED_ShowNum(48, 48, show_ang, 5, 16);

            // --- C. 发送至屏幕 (最耗时的一步) ---
            OLED_Refresh_Gram(); */
            

			// 显示误差 (Position Error)
			OLED_ShowString(48, 16, "      "); // 清除旧数字
            if(Actual_Pos_Scaled < 0) {
                OLED_ShowString(48, 16, "-");
                OLED_ShowNum(56, 16, -Target_Speed_Hz, 6, 16);
            } else {
                OLED_ShowNum(48, 16, Target_Speed_Hz, 7, 16);
            }

            OLED_ShowString(48, 32, "      "); // 清除旧数字
            if(Position_Error < 0) {
                OLED_ShowString(48, 32, "-");
                OLED_ShowNum(56, 32, -Position_Error, 6, 16);
            } else {
                OLED_ShowNum(48, 32, Position_Error, 7, 16);
            }

            // 显示实际输出频率 (Real Output Hz)
            OLED_ShowString(48, 48, "      ");
            if(Real_Output_Hz < 0) {
                 OLED_ShowString(48, 48, "-");
                 OLED_ShowNum(56, 48, -Real_Output_Hz, 6, 16);
            } else {
                 OLED_ShowNum(48, 48, Real_Output_Hz, 7, 16);
            }
            
            OLED_Refresh_Gram();
            // --- D. LED指示灯 (可选，证明系统没死机) ---
            LMain = !LMain;
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

