#ifndef __STEPMOTOR_H
#define __STEPMOTOR_H			  	 
	 
#include "stm32f10x.h"
//////////////////////////////////////////////////////////////////////////////////	 
//
//
//
//********************************************************************************
// ================= 硬件参数 =================
// 机械参数
#define MICRO_STEP        16    // 细分
#define STEPPER_FULL_STEP 200   // 电机物理步数
// 一圈需要的脉冲数 = 200 * 16 = 3200
#define PULSES_PER_REV    (STEPPER_FULL_STEP * MICRO_STEP) 
// 编码器一圈数值
#define ENCODER_RES       4096 

// 定点数运算参数 (使用 int64_t 避免 double 性能问题)
#define SCALING_FACTOR    1000  // 放大1000倍
// 比例系数: 1个脉冲 = 多少个编码器单位 (放大后)
// (4096 / 3200) * 1000 = 1280
#define RATIO_SCALED      ((ENCODER_RES * SCALING_FACTOR) / PULSES_PER_REV)

// 丢步补偿参数
#define COMP_DEADZONE     15    // 误差死区 (编码器单位, 约1.3度)
#define COMP_KP           10.0f // 补偿力度 (P参数)
// ==========================================

void StepMotor_Init(void);
void StepMotor_SetSpeed(int32_t speed_hz);
#endif  
	 
