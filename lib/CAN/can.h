#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	    
//////////////////////////////////////////////////////////////////////////////////	 
//
//////////////////////////////////////////////////////////////////////////////////
// 定义 CAN ID
#define CAN_ID_MOTOR_FB    0x103  // F103 发回给 F407 的反馈
#define CAN_ID_FILTER_START   0x400  // 32位mask模式下允许0x100-0x10F 的ID通过 F407 发给 F103 

/*typedef struct {
    int32_t RealSpeed;   // 4字节，实际速度
    int32_t Position;    // 4字节，累计位置 (部分)
} Motor_Feedback_t;*/

// CAN 初始化函数
u8 CAN_Mode_Init(void);         //CAN初始化

void CAN_Send_Feedback(int32_t real_speed, int32_t pos_error); // 发送反馈数据
void CAN_Filter_MODE_16BIT_LIST(void);
void CAN_Filter_MODE_16BIT_MASK(void);
void CAN_Filter_MODE_32BIT_LIST(void);
void CAN_Filter_MODE_32BIT_MASK(void);

#endif


