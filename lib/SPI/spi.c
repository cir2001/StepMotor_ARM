#include "spi.h"
#include "port.h"
//////////////////////////////////////////////////////////////////////////////////	 
//SPI驱动 代码	   			  
//////////////////////////////////////////////////////////////////////////////////
//主模式
//当设置为软件nss管理 设置ssm位1，ssi位1
//需仔细阅读手册配置说明
//-------------------------------------------------------------------------------
u8 u8Led1_Count;
//SPI口初始化
//这里针是对SPI1的初始化
void SPI1_Init(void)
{	 
	RCC->APB2ENR|=1<<2;  		//PORTA时钟使能 	 
	RCC->APB2ENR|=1<<12;  		//SPI1时钟使能 
	
	
	GPIOA->CRL&=0X0000FFFF;
	GPIOA->CRL|=0XB3B30000;		//PA5/PA6/PA7复用 	 PA5:SPI1_SCL PA6:OLED_RES推挽模式(SPI1_MISO) PA7:SPI1_MOSI   
	GPIOA->ODR|=7<<5;   	 	//PA5/PA6/PA7上拉
	
	//对SPI1属于APB1的外设.时钟频率最大为36M.
	SPI1->CR1|=3<<3; 		//Fsck=Fpclk1/256
	SPI1->CR1|=1<<1; 		//空闲模式下SCK为1 CPOL=1
	SPI1->CR1|=1<<0; 		//数据采样从第二个时间边沿开始,CPHA=1 
	SPI1->CR1|=0<<7; 		//MSBfirst
	SPI1->CR1|=1<<9; 		//软件nss管理 ssm位
	SPI1->CR1|=1<<8;  		//ssi位 配置为主模式，在NSS软件模式下，设置ssm位1，ssi位1
	SPI1->CR1|=1<<2; 		//SPI主机
	//SPI2->CR1|=0<<2; 		//SPI从机
	SPI1->CR1|=0<<11;		//8bit数据格式
	SPI1->CR1|=0<<10;		//全双工模式	
	//SPI2->CR1|=1<<13;		//硬件CRC校验使能
	
	SPI1->CR1|=1<<6; 		//SPI设备使能
		
	//SPI2->CR2|=1<<6; 		//SPI接收缓冲区非空中断使能 	
	//MY_NVIC_Init(1,2,SPI2_IRQn,2);//组2， 优先级 
	
	SPI1_ReadWriteByte(0xff);//启动传输		 
}   
//SPI1速度设置函数
//SpeedSet:0~7
//SPI速度=fAPB1/2^(SpeedSet+1)
//APB1时钟一般为36Mhz
void SPI1_SetSpeed(u8 SpeedSet)
{
	SpeedSet&=0X07;			//限制范围
	SPI1->CR1&=0XFFC7; 
	SPI1->CR1|=SpeedSet<<3;	//设置SPI2速度  
	SPI1->CR1|=1<<6; 		//SPI设备使能	  
} 
//SPI1 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u8 SPI1_ReadWriteByte(u8 TxData)
{		
	u16 retry=0;				 
	while((SPI1->SR&1<<1)==0)		//等待发送区空
	{
		retry++;
		if(retry>=0XFFFE)return 0; 	//超时退出
	}			  
	SPI1->DR=TxData;	 	  		//发送一个byte 
	retry=0;
	while((SPI1->SR&1<<0)==0) 		//等待接收一个byte  
	{
		retry++;
		if(retry>=0XFFFE)return 0;	//超时退出
	}	  						    
	return SPI1->DR;          		//返回收到的数据    
}


//这里针是对SPI2的初始化
void SPI2_Init(void)
{	 
	RCC->APB2ENR|=1<<3;  	//PORTB时钟使能 	 
	RCC->APB1ENR|=1<<14;   	//SPI2时钟使能 
	//这里只针对SPI口初始化
	GPIOB->CRH&=0X0000FFFF; 
	GPIOB->CRH|=0XB8B30000;	//PB13/14/15复用 	 PB13-SPI2_SCL PB14-SPI2_MISO PB15-SPI2_MOSI   	
	GPIOB->ODR|=7<<13;   	//PB13/14/15上拉
	
	//这里只针对SPI口初始化
	//RCC->APB1RSTR|=1<<14;	//复位SPI2
	//RCC->APB1RSTR&=~(1<<14);//停止复位SPI2
	
	
	//对SPI2属于APB1的外设.时钟频率最大为36M.
	SPI2->CR1|=7<<3; 		//Fsck=Fpclk1/256
	SPI2->CR1|=1<<1; 		//空闲模式下SCK为1 CPOL=1
	SPI2->CR1|=0<<0; 		//数据采样从第二个时间边沿开始,CPHA=1 
	SPI2->CR1|=0<<7; 		//MSBfirst
	SPI2->CR1|=1<<9; 		//软件nss管理 ssm位
	SPI2->CR1|=1<<8;  		//ssi位 配置为主模式，在NSS软件模式下，设置ssm位1，ssi位1
	SPI2->CR1|=1<<2; 		//SPI主机
	//SPI2->CR1|=0<<2; 		//SPI从机
	SPI2->CR1|=0<<11;		//8bit数据格式
	SPI2->CR1|=0<<10;		//全双工模式	
	//SPI2->CR1|=1<<13;		//硬件CRC校验使能
	
	SPI2->CR1|=1<<6; 		//SPI设备使能
		
	//SPI2->CR2|=1<<6; 		//SPI接收缓冲区非空中断使能 	
	//MY_NVIC_Init(1,2,SPI2_IRQn,2);//组2， 优先级 
	
	SPI2_ReadWriteByte(0xff);//启动传输		 
}   
//SPI2速度设置函数
//SpeedSet:0~7
//SPI速度=fAPB1/2^(SpeedSet+1)
//APB1时钟一般为36Mhz
void SPI2_SetSpeed(u8 SpeedSet)
{
	SpeedSet&=0X07;			//限制范围
	SPI2->CR1&=0XFFC7; 
	SPI2->CR1|=SpeedSet<<3;	//设置SPI2速度  
	SPI2->CR1|=1<<6; 		//SPI设备使能	  
} 
//SPI2 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u8 SPI2_ReadWriteByte(u8 TxData)
{			
	while((SPI2->SR&1<<1)==0);		//等待发送区空 
	SPI2->DR=TxData;	 	  		//发送一个byte  
	while((SPI2->SR&1<<0)==0);		//等待接收完一个byte  
 	return SPI2->DR;          		//返回收到的数据	
}


//========================================================
// SPI2_IRQHandler  中断服务程序	
//===============================
void SPI2_IRQHandler(void)
{
	//u8 i;
	u8Led1_Count++;
	if(u8Led1_Count == 50)
	{

		u8Led1_Count = 0;
	}
}
