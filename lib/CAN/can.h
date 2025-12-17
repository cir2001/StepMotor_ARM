#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	    
//////////////////////////////////////////////////////////////////////////////////	 
//
//////////////////////////////////////////////////////////////////////////////////

//CAN接收RX0中断使能
#define CAN_RX0_INT_ENABLE	1		//0，不使能；1，使能		

// 定义 CAN ID
#define CAN_ID_MOTOR_CMD   0x301  // F407 发给 F103 的指令
#define CAN_ID_MOTOR_FB    0x201  // F103 发回给 F407 的反馈
#define CAN_ID_FILTER_START   0x300  // 32位mask模式下允许0x100-0x10F 的ID通过


// 简单的协议结构 (8字节)
typedef struct {
    int32_t TargetSpeed; // 4字节，目标速度 (放大后的整数)
    uint8_t Command;     // 1字节，例如 1=Enable, 0=Disable
    uint8_t Reserved[3]; // 保留位
} Motor_Cmd_t;

typedef struct {
    int32_t RealSpeed;   // 4字节，实际速度
    int32_t Position;    // 4字节，累计位置 (部分)
} Motor_Feedback_t;

// CAN 初始化函数
u8 CAN_Mode_Init(void);         //CAN初始化
u8 CAN_Tx_Msg(u32 id,u8 ide,u8 rtr,u8 len,u8 *dat);		//发送数据
u8 CAN_Msg_Pend(u8 fifox);								//查询邮箱报文
void CAN_Rx_Msg(u8 fifox,u32 *id,u8 *ide,u8 *rtr,u8 *len,u8 *dat);//接收数据
u8 CAN_Tx_Staus(u8 mbox);  								//返回发送状态
u8 CAN_Send_Msg(u8* msg,u8 len);						//发送数据
u8 CAN_Receive_Msg(u8 *buf);							//接收数据

void CAN_Send_Feedback(int32_t real_speed, int32_t pos_error); // 发送反馈数据
void CAN_Filter_MODE_16BIT_LIST(void);
void CAN_Filter_MODE_16BIT_MASK(void);
void CAN_Filter_MODE_32BIT_LIST(void);
void CAN_Filter_MODE_32BIT_MASK(void);

#endif


