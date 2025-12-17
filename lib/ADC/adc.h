#ifndef __ADC_H
#define __ADC_H	

#define CH1_ADC    0 //通道0  右手Y轴
#define CH2_ADC    1 //通道1  右手X轴
#define ADC_CH2    2 //通道2
#define ADC_CH3    3 //通道3	   
#define CH3_ADC    4 //通道4	   
#define CH4_ADC    5 //通道5	   
#define CH5_ADC    6 //通道6	   
#define CH6_ADC    7 //通道7	   
#define ADC_CH8    8 //通道8	  左手Y轴 
#define ADC_CH9    9 //通道9	  左手X轴  
#define ADC_CH10  10 //通道10	   
#define ADC_CH11  11 //通道11	   
#define ADC_CH12  12 //通道12	   
#define ADC_CH13  13 //通道13	   
#define ADC_CH14  14 //通道14	   
#define ADC_CH15  15 //通道15	   
//=============================================
void STM32Adc_Init(void);
void Start_STM32Adc(u8 ch);
 
#endif 







