#include <stm32f10x.h>	   
#include "port.h"
//初始化GPIO		    
void PORT_Init(void)
{
	//RCC->APB2ENR|=1;				//配置PB3,PB4为普通IO
	//AFIO->MAPR = 0x02000000;		//配置PB3,PB4为普通IO
	RCC->APB2ENR|=1<<2;    			//使能PORTA时钟	   	 
	RCC->APB2ENR|=1<<3;    			//使能PORTB时钟	   	 
	RCC->APB2ENR|=1<<4;   	  		//使能PORTC时钟	   	 
	RCC->APB2ENR|=1<<5;    			//使能PORTD时钟	
//----PortA----	   	    		 
	GPIOA->CRL&=0XFFFFFFF0; 		//PA0		模拟输入				ADC_CH1				A-In(ADC-CH0)		Pin10  
	GPIOA->CRL|=0X00000008;			//PA0		推挽输出 						
	GPIOA->CRL&=0XFFFFFF0F; 		//PA1		模拟输入				ADC_CH2				A-In(ADC-CH1)		Pin11 		
	GPIOA->CRL|=0X00000030;			//PA1		推挽输出 						
									//PA2		推挽复用输出  			UART2_TxD			D-Out				Pin12  
									//PA3		推挽复用输出   			UART2_RxD			D-In   	 			Pin13
	GPIOA->CRL&=0XFFF0FFFF; 		//PA4 		推挽复用输出	  		MD0					D-Out				Pin14 
	GPIOA->CRL|=0X00030000; 		 	
	//GPIOA->CRL&=0XFF0FFFFF; 		//PA5		推挽复用输出			MD1					D-Out 				Pin15	
	//GPIOA->CRL&=0X00300000; 
	//GPIOA->CRL&=0XF0FFFFFF; 		//PA6		模拟输入				ADC_CH5				A-In(ADC-CH6)		Pin16 
	//GPIOA->CRL|=0X08000000; 		 
	//GPIOA->CRL&=0X0FFFFFFF; 		//PA7		模拟输入	 			ADC_CH6				A-In(ADC-CH7)		Pin17  
	//GPIOA->CRL|=0X30000000; 		
//-------------
	GPIOA->CRH&=0XFFFFFFF0; 		//PA8   	推挽复用输出  								D-Out				Pin29
	GPIOA->CRH|=0X00000003; 		
	GPIOA->CRH&=0XFFFFFF0F;			//PA9 		推挽复用输出 								D-Out				Pin30 
	GPIOA->CRH|=0X00000030; 	
	GPIOA->CRH&=0XFFFFF0FF;			//PA10		上拉输入模式	  							D-In				Pin31 
	GPIOA->CRH|=0X00000300;
 	GPIOA->CRH&=0XFFFF0FFF; 		//PA11  	推挽复用输出 								D-Out				Pin32
	GPIOA->CRH|=0X00003000; 		
 	GPIOA->CRH&=0XFFF0FFFF; 		//PA12		上拉下拉输入模式							D-In				Pin33
	GPIOA->CRH|=0X00030000; 		
									//PA13  							JTMS								   Pin34
									//PA14 								JTCK								   Pin37
	//GPIOA->CRH&=0X0FFFFFFF; 		//PA15								JTDI								   Pin38
	//GPIOA->CRH|=0X80000000; 		
		 
  	GPIOA->ODR =0xFFFFFFFF; 		//PA 		全部上拉
//----PortB----	 
	GPIOB->CRL&=0XFFFFFFF0; 		
	GPIOB->CRL|=0X00000003;			//PB0		推挽输出 				PWM1				D-Out				Pin18
	GPIOB->CRL&=0XFFFFFF0F; 		
	GPIOB->CRL|=0X00000030; 		//PB1		推挽输出 				PWM2				D-Out				Pin19
									//PB2								Boot1									Pin20
	GPIOB->CRL&=0XFFFF0FFF;
	GPIOB->CRL|=0X00003000; 		//PB3		上拉下拉输入			D-In									Pin39  
	GPIOB->CRL&=0XFFF0FFFF;
	GPIOB->CRL|=0X00030000; 		//PB4		推挽输出				D-Out									Pin40  
	GPIOB->CRL&=0XFF0FFFFF;
	GPIOB->CRL|=0X00300000; 		//PB5		推挽输出 				D-Out									Pin41 
	GPIOB->CRL&=0XF0FFFFFF;
	GPIOB->CRL|=0X03000000; 		//PB6		推挽输出 				PWM3				D-Out				Pin42  
	GPIOB->CRL&=0X0FFFFFFF;
	GPIOB->CRL|=0X30000000; 		//PB7		推挽输出 				PWM4				D-Out				Pin43 
//-------------
	GPIOB->CRH&=0XFFFFFFF0;
	GPIOB->CRH|=0X00000003; 		//PB8		推挽输出 		 		PWM5				D-Out				Pin45
	GPIOB->CRH&=0XFFFFFF0F;			
	GPIOB->CRH|=0X00000030; 		//PB9		推挽输出 				PWM6				D-Out				Pin46
	GPIOB->CRH&=0XFFFFF0FF;
	GPIOB->CRH|=0X00000300; 		//PB10		推挽输出 									D-Out				Pin21
	GPIOB->CRH&=0XFFFF0FFF;
	GPIOB->CRH|=0X00003000; 		//PB11		推挽输出									D-Out				Pin22
	GPIOB->CRH&=0XFFF0FFFF;
	GPIOB->CRH|=0X00030000; 		//PB12		推挽输出									D-Out				Pin25
	GPIOB->CRH&=0XFF0FFFFF;
	GPIOB->CRH|=0X00300000; 		//PB13		复用					SPI2-SCK(GT32-SCK)						Pin26
	GPIOB->CRH&=0XF0FFFFFF;														
	GPIOB->CRH|=0X03000000;			//PB14		复用					AIN1									Pin27
	GPIOB->CRH&=0X0FFFFFFF;	
	GPIOB->CRH|=0X30000000;			//PB15		复用					AIN2									Pin28


	GPIOB->ODR =0xFFFFFFFF; 		//PB 		全部上拉
//----PortC----	   	 
	GPIOC->CRL&=0XFFFFFFF0;
	GPIOC->CRL|=0X00000003; 		//PC0		推挽输出									D-In(ADC-CH10)		Pin8  
	GPIOC->CRL&=0XFFFFFF0F;
	GPIOC->CRL|=0X00000080; 		//PC1		上拉下拉输入模式		 					 D-In(ADC-CH11)		Pin9  
	GPIOC->CRL&=0XFFFFF0FF;
	GPIOC->CRL|=0X00000300; 		//PC2		上拉下拉输入模式				 			 D-In(ADC-CH12)		Pin10  
	GPIOC->CRL&=0XFFFF0FFF;
	GPIOC->CRL|=0X00003000; 		//PC3		上拉下拉输入模式		 					 D-In(ADC-CH13)		Pin11  
	GPIOC->CRL&=0XFFF0FFFF;
	GPIOC->CRL|=0X00030000; 		//PC4		推挽输出									D-Out				Pin24  
	GPIOC->CRL&=0XFF0FFFFF;
	GPIOC->CRL|=0X00300000; 		//PC5		上拉下拉输入模式							 D-In				Pin25  
	GPIOC->CRL&=0XF0FFFFFF;
	GPIOC->CRL|=0X03000000; 		//PC6		上拉下拉输入模式		 LCD_A0				 D-Out				Pin37  
	GPIOC->CRL&=0X0FFFFFFF;
	GPIOC->CRL|=0X30000000; 		//PC7		上拉下拉输入模式							 D-In				Pin38  
//--------------
	GPIOC->CRH&=0XFFFFFFF0;
	GPIOC->CRH|=0X00000003; 		//PC8		上拉下拉输入模式							 D-In	 			Pin39  
	GPIOC->CRH&=0XFFFFFF0F;
	GPIOC->CRH|=0X00000030; 		//PC9		上拉下拉输入模式							 D-In 				Pin40 
	GPIOC->CRH&=0XFFFFF0FF;
	GPIOC->CRH|=0X00000300; 		//PC10		上拉下拉输入模式							D-In				Pin51 
	GPIOC->CRH&=0XFFFF0FFF;
	GPIOC->CRH|=0X00003000; 		//PC11		上拉下拉输入模式							D-In 				Pin52 
	GPIOC->CRH&=0XFFF0FFFF;
	GPIOC->CRH|=0X00030000; 		//PC12		上拉下拉输入模式							D-In 				Pin53 
	GPIOC->CRH&=0XFF0FFFFF;
	GPIOC->CRH|=0X00300000; 		//PC13		上拉下拉输入模式							D-In				Pin2
	GPIOC->CRH&=0XF0FFFFFF;
	GPIOC->CRH|=0X03000000; 		//PC14		上拉下拉输入模式							D-In				Pin3
	GPIOC->CRH&=0X0FFFFFFF;
	GPIOC->CRH|=0X30000000; 		//PC15		上拉下拉输入模式							D-In				Pin4

 	GPIOC->ODR =0xFFFFFFFF; 		//PC 		全部上拉
											  
//	JTAG_Set(JTAG_SWD_DISABLE);//关闭JTAG和SWD   
}



