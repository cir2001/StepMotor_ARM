#ifndef __AS5600_H
#define __AS5600_H
#include "myiic.h"   
//////////////////////////////////////////////////////////////////////////////////	 
//
//
//
//////////////////////////////////////////////////////////////////////////////////
#define AS5600_DIR_CW()   (GPIOB->BRR  = GPIO_BRR_BR0)  // PB0 Low
#define AS5600_DIR_CCW()  (GPIOB->BSRR = GPIO_BSRR_BS0) // PB0 High

#define AS5600_Magnet_PIN_Detected() ((GPIOB->IDR & GPIO_IDR_IDR1) == 0)
// 注意：AS5600 GPO 低电平表示检测到磁铁 (Active Low)

// AS5600 I2C 地址 (7-bit: 0x36)
#define AS5600_ADDR        0x36
// AS5600 原始角度寄存器 (12-bit)
#define AS5600_RAW_ANGLE   0x0C

// AS5600 设备地址 (7-bit: 0x36)
// 写地址: 0x36 << 1 = 0x6C
// 读地址: 0x6D
#define AS5600_ADDR_WRITE   0x6C
#define AS5600_ADDR_READ    0x6D
#define AS5600_STATUS_REG   0x0B  //状态寄存器

void AS5600_Init(void); //初始化IIC
u16 AS5600_GetRawAngle(void) ; //读取 AS5600 的原始角度 (0-4095)

#endif
