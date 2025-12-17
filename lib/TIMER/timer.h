#ifndef __TIMER_H
#define __TIMER_H

#include "sys.h"
//==============================================
void Timer1_Init(u16 arr,u16 psc);
void Timer2_Init(u16 arr,u16 psc);
void Timer3_Init(u16 arr,u16 psc);
void Timer4_Init(u16 arr,u16 psc);
void Timer5_Init(u16 arr,u16 psc);

void PWM_Init_Timer1(u16 arr,u16 psc);
void PWM_Init_Timer3(u16 arr,u16 psc);
void PWM_Init_Timer4(u16 arr,u16 psc);

void Encoder_Init_Timer2(void);
void Encoder_Init_Timer3(void);
void Encoder_Init_Timer4(void);

void SignData2ASCII5(s16 ConverDataX,s16 ConverDataY,s16 ConverDataZ);
void MPU_Gyro_Sample(void);
void MPU_Acc_Sample(void);

void Gyro2ASCII(float Gx,float Gy,float Gz);

void GetMotorPulse(void);
void SpeedControl(void);
void AngleControl(void);
void SetMotorPWM(float MotorAPWM,float MotorBPWM,float MotorCPWM);

void MotorOutPut(void);
void fData2ASCII(float fx,float fy,float fz);

void fData2ASCII9(float f1,float f2,float f3,float f4,float f5,float f6,float f7,float f8,float f9);
void fData2ASCII6(float f1,float f2,float f3,float f4,float f5,float f6);

void Forward_Kinematics(float a,float b,float c);
void Inverse_Kinematics(float a,float b,float c);
#endif























