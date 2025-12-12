#ifndef __PORT_H
#define __PORT_H	 
#include "sys.h"
//=============================	   
//====PortA====
								//PA0   	AD_CH1				A-In(ADC_CH0)			Pin14
#define OLED_CS		PAout(1)	//PA1   	AD_CH2				A-In(ADC_CH1)	  	Pin15
								//PA2   	N.U						D-in							Pin16
								//PA3   	N.U						D-in							Pin17
#define OLED_DC		PAout(4)	//PA4   	AD_CH3				A-In(ADC_CH4)			Pin20
															//PA5   	AD_CH4				A-In(ADC_CH5)			Pin21
															//PA6   	AD_CH5				A-In(ADC_CH6)			Pin22
															//PA7   	AD_CH6				A-In(ADC_CH7)			Pin23
#define Motor_M1   	PAout(8)	//PA8  	 	LED0					D-Out							Pin41
#define Motor_M0   	PAout(9)	//PA9   	UART1_TxD  		D-Out							Pin42
#define Motor_Ena  	PAout(10)	//PA10  	UART1_RxD  		D-In							Pin43
								//PA11  	N.U						D-In							Pin44
								//PA12  	N.U						D-In							Pin45
								//PA13  	JTMS														Pin46
								//PA14  	JTCK														Pin49
								//PA15  	N.U						D-In							Pin50
//====PortB====
//#define AS_DIR   	PBout(0)	//PB0			PWM_OUT_CH1		D-Out(ADC-CH8)		Pin26
//#define AS_GPO   	PBout(1)	//PB1			PWM_OUT_CH2		D-Out(ADC-CH9)		Pin27
								//PB2   	Boot1														Pin28
								//PB3			N.U						D-In							Pin55
								//PB4   	N.U						D-Out 							Pin56
								//PB5   	N.U						D-Out							Pin57
								//PB6   	PWM_OUT_CH3		D-Out							Pin58
								//PB7   	PWM_OUT_CH4		D-Out							Pin59	
								//PB8   	PWM_OUT_CH5		D-Out							Pin62
								//PB9   	PWM_OUT_CH6		D-Out							Pin62
								//PB11  	N.U						D-Out							Pin30
#define LMain   	PBout(12)	//PB12  	N.U						D-Out							Pin33
#define Motor_Dir	PBout(13)	//PB13  	N.U						D-Out							Pin34
#define Motor_Step  PBout(14)	//PB14  	N.U						D-In							Pin35
#define Motor_M2  	PBout(15)	//PB15  	N.U						D-Out							Pin36
//====PortC====
								//PC0   	N.U						D-In(ADC-CH10)		Pin8
								//PC1   	N.U						D-In(ADC-CH11)		Pin9
								//PC2   	N.U						D-In(ADC-CH12)		Pin10
								//PC3	  	N.U						D-In(ADC-CH13)		Pin11
								//PC4	  	N.U						D-Out							Pin24
								//PC5	  	N.U						D-Out							Pin25
								//PC6	  	N.U						D-Out							Pin37
								//PC7	  	N.U						D-Out							Pin38
								//PC8	  	N.U						D-Out							Pin39
								//PC9	  	N.U						D-In							Pin40
								//PC10 	 	N.U						D-In							Pin51
								//PC11		N.U						D-In							Pin52
								//PC12   	N.U						D-In							Pin53
#define LED1   PCout(13)		//PC13   	N.U						D-In							Pin2
#define LED0   PCout(14)		//PC14   	N.U						D-In							Pin3
								//PC15   	N.U						D-In							Pin4

//====PortD====
								//PD0    8MHz															Pin5
								//PD1    8MHz															Pin6
								//PD2    LED1						D-Out							Pin54


#define OLED_CS_Clr() 	OLED_CS=0; 
#define OLED_CS_Set() 	OLED_CS=1; 

#define OLED_RS_Clr() 	OLED_DC=0; 
#define OLED_RS_Set() 	OLED_DC=1; 

void PORT_Init(void);	//端口初始化	    
#endif
