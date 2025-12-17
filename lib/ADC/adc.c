#include <stm32f10x.h>
#include "adc.h"
#include "port.h"
#include "math.h"

extern u8 u8SPIReadBuf,u8AD7799InitEnd;

extern u16 u16ADCTimer;		   
//初始化STM32-ADC,规则通道
//===============================
void STM32Adc_Init(void)
{    
	RCC->APB2ENR|=1<<9;    //ADC1时钟使能	  
	RCC->APB2RSTR|=1<<9;   //ADC1复位
	RCC->APB2RSTR&=~(1<<9);//复位结束	    
	RCC->CFGR&=~(3<<14);   //分频因子清零	
	//SYSCLK/DIV2=12M ADC时钟设置为12M,ADC最大时钟不能超过14M!
	//否则将导致ADC准确度下降! 
	RCC->CFGR|=2<<14;      // 72MHz/6=12MHz(PCLK2=72MHz)	 

	ADC1->CR1&=0XF0FFFF;   //工作模式清零
	ADC1->CR1|=0<<16;      //独立工作模式  	bit19:16=0000
	ADC1->CR1&=~(1<<8);    //非扫描模式	  	bit8=0
	ADC1->CR2&=~(1<<1);    //单次转换模式	bit1=0
	ADC1->CR2&=~(7<<17);	   
	ADC1->CR2|=7<<17;	   //软件控制转换  	bit19:17=111(SWSTART)
	ADC1->CR2|=1<<20;      //使用用外部触发(SWSTART)!!!	必须使用一个事件来触发
	ADC1->CR2&=~(1<<11);   //右对齐	 bit11=0
	ADC1->SQR1&=~(0XF<<20);
	ADC1->SQR1&=0<<20;     //1个转换在规则序列中 也就是只转换规则序列1 	bit23:20=0		   
	//设置通道3的采样时间
//  AD转换周期=41.5+12.5 在12MHz的ADCLK下，AD转换时间=(41.5+12.5)*(1/12000000）= 4.5us	 
	ADC1->SMPR2|=4<<12;     //通道4  41.5个ADCCLK周期	 
	ADC1->SMPR2|=4<<9;      //通道3  41.5个ADCCLK周期	 

	ADC1->CR2|=1<<0;	    //开启AD转换器	 
	ADC1->CR2|=1<<3;        //使能复位校准  
	while(ADC1->CR2&1<<3);  //等待校准结束 			 
    //该位由软件设置并由硬件清除。在校准寄存器被初始化后该位将被清除。 		 
	ADC1->CR2|=1<<2;        //开启AD校准	   
	while(ADC1->CR2&1<<2);  //等待校准结束
	//该位由软件设置以开始校准，并在校准结束时由硬件清除  
}				  
//获得ADC值
//ch:通道值 0~3
/*u16 Get_Adc(u8 ch)   
{
	//设置转换序列	  		 
	ADC1->SQR3&=0XFFFFFFE0;//规则序列1 通道ch
	ADC1->SQR3|=ch;		  			    
	ADC1->CR2|=1<<22;       //启动规则转换通道 
	while(!(ADC1->SR&1<<1));//等待转换结束	 	   
	return ADC1->DR;		//返回adc值	
}	*/
//=========================================================
// 启动STM32 ADC
// u8 ch：ADC通道号
//=========================================================
void Start_STM32Adc(u8 ch)   
{
	//设置转换序列	  		 
	ADC1->SQR3&=0XFFFFFFE0;		//设定通道ch为规则序列中的第1个转换 
	ADC1->SQR3|=ch;				//ch	    
	ADC1->CR2|=1<<22;   		//启动规则转换通道 	SWSTART=1
}










