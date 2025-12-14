#include "timer.h"
#include "port.h"
#include "usart.h"
#include "delay.h"
#include "AS5600.h"
#include "stepMotor.h"
#include <stdlib.h>
//==========================================


//*******************************************
extern u8 USART1_TX_BUF[128],USART2_TX_BUF[128];
extern u8 TxD1pt,TxD2pt,TxD1Num,TxD2Num;
// 闭环控制相关
volatile int64_t Theory_Pos_Scaled = 0; // 理论位置 (放大版)
volatile int64_t Actual_Pos_Scaled = 0; // 实际位置 (放大版)
volatile int32_t Position_Error = 0; // 误差 (实际编码器单位)
u16 Last_AS5600_Raw = 0;           // 上次编码器读数

// 速度控制相关
volatile int32_t Target_Speed_Hz = 0;   // 用户设定的目标速度
volatile int32_t Real_Output_Hz = 0;   // 实际发给电机的速度 (包含补偿)
u16 t1,t2;
extern u8 u8Uart2_flag_test;
//**** Timer2 IRQHandler **********************************  	 
//===============================
//定时2中断服务程序	 未使用
//===============================  	 
void TIM2_IRQHandler(void)
{
    // 变量定义
    uint16_t curr_raw = 0;
    int32_t delta_enc = 0;
    int64_t delta_theory = 0;
    int64_t err_scaled = 0;
    int32_t compensation = 0;
    int32_t final_speed = 0;
	int32_t temp_target = 0;
	int32_t max_change; 
    int32_t last_speed;
	int64_t total_numerator=0;
    
    // 静态变量 (保持状态)
    static int32_t remainder_accum = 0; // 余数累加器
    
    // 检查更新中断
    if(TIM2->SR & 0X0001)
    {
        TIM2->SR &= ~(1<<0); // 【建议】进中断先清标志位，防止重入风险

        // LED1 闪烁测试
        t1++;
        if(t1 > 100) 
		{
			 t1 = 0;
			 LED1 = !LED1; 
		}
		// 测试电机旋转一周as5600旋转的角度，发送旋转一周的指令StepMotor_SetSpeed(1000);
        if(u8Uart2_flag_test == 1)
        {
            t2++;
            if(t2 >= 640) // 约 3.2秒超时
            {
                t2 = 0;
                u8Uart2_flag_test = 0;
                //Target_Speed_Hz = 0; // 建议直接改目标速度，而不是直接操作电机
            }
        }

        // ============================================================
        // 1. 读取 AS5600 并处理位置
        // ============================================================
        curr_raw = AS5600_GetRawAngle();
        
        // 【注意】如果Error一直是正数且增大，请解开下面这行的注释，注释掉上面那行
        // delta_enc = -(curr_raw - Last_AS5600_Raw); 
        delta_enc = curr_raw - Last_AS5600_Raw; 

        // 处理过零点 (0-4095)
        if (delta_enc > 2048)       delta_enc -= 4096;
        else if (delta_enc < -2048) delta_enc += 4096;
        
        Last_AS5600_Raw = curr_raw;

        // 更新实际位置
        Actual_Pos_Scaled += (int64_t)delta_enc * SCALING_FACTOR;


        // ============================================================
        // 2. 更新理论位置 
        // ============================================================
        if (Target_Speed_Hz != 0) 
        {
            // 【核心修改】这里必须用 Target_Speed_Hz (纯净的目标速度)
            // -----------------------------------------------------------
            //total_numerator = ((int64_t)Target_Speed_Hz * RATIO_SCALED) + remainder_accum;
			total_numerator = ((int64_t)Target_Speed_Hz * RATIO_SCALED) + remainder_accum;
            // -----------------------------------------------------------
            
            delta_theory = total_numerator / 200;  
            remainder_accum = total_numerator % 200; 
            
            Theory_Pos_Scaled += delta_theory;
        }
        else 
        {
            Theory_Pos_Scaled = Actual_Pos_Scaled;
            remainder_accum = 0; 
        }
        // ============================================================
        // 3. 计算误差 & 限制
        // ============================================================
        err_scaled = Theory_Pos_Scaled - Actual_Pos_Scaled;
        Position_Error = (int32_t)(err_scaled / SCALING_FACTOR);

        // 误差饱和限制 (防止误差无限累积)
        if (Position_Error > 2000) 
        {
            Theory_Pos_Scaled -= (Position_Error - 2000) * SCALING_FACTOR;
            Position_Error = 2000;
        }
        else if (Position_Error < -2000)
        {
            Theory_Pos_Scaled -= (Position_Error + 2000) * SCALING_FACTOR;
            Position_Error = -2000;
        }


        // ============================================================
        // 4. 计算补偿 (P控制)
        // ============================================================
        compensation = 0;
        if (Target_Speed_Hz != 0 && abs(Position_Error) > COMP_DEADZONE) 
        {
            compensation = (int32_t)(Position_Error * COMP_KP);
            
            // 静态限幅
            if (compensation > 3000) compensation = 3000;
            if (compensation < -3000) compensation = -3000;
        }
        // ============================================================
        // 5. 计算最终速度 (含加速度限制 Slew Rate Limit)
        // ============================================================
        temp_target = 0;

        // 1. 算出理想的目标值 (可能发生剧烈跳变)
		temp_target = Target_Speed_Hz + compensation;

		// 2. 加速度限制 (防止电机堵转的核心！)
		// 限制每 5ms 最多改变 100Hz (相当于 20000Hz/s 的加速度)
		max_change = 100; 
		last_speed = Real_Output_Hz;

		if (temp_target > last_speed + max_change)
			final_speed = last_speed + max_change;
		else if (temp_target < last_speed - max_change)
			final_speed = last_speed - max_change;
		else
			final_speed = temp_target;
        
        // 更新历史值
        Real_Output_Hz = final_speed; 
        
        // ============================================================
        // 6. 执行驱动 (必须取消注释！)
        // ============================================================
        StepMotor_SetSpeed(final_speed);
        
    }
    // TIM2->SR&=~(1<<0); // 建议移到函数最开头
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





