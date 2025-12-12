#include "usart.h"	  
#include "port.h"	  
#include "stmflash.h"
#include "timer.h"

//设置FLASH 保存地址(必须为偶数，且其值要大于本代码所占用FLASH的大小+0X08000000)
#define FLASH_SAVE_ADDR  0X08010000 	
u16 FalshSave[32];
//////////////////////////////////////////////////////////////////
u8  USART1_RX_BUF[128];     //接收缓冲,最大128个字节.
u8  USART1_TX_BUF[128];     //发送缓冲,最大128个字节.
u8  USART2_RX_BUF[128];     //接收缓冲,最大128个字节.
u8  USART2_TX_BUF[128];     //发送缓冲,最大128个字节.
u8  u8TxD1Busy,u8ErrCode,TxD1Num,TxD2Num,TxD1pt,TxD2pt,RxD1pt,RxD2pt,RxD1Buf,RxD2Buf;
u8  u8RxD2Com;

u8  u8test1,u8test2,u8test3,u8test4,u8test5,u8test6,u8test7,u8test8;
u8  rData1Temp,rData2Temp;
u8 key_press;
//=========================================================
// USART1 中断服务程序		  与上位机通讯
//=========================================================
#ifdef EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
//接收状态
//bit7，接收完成标志
//bit6，接收到0x0d
//bit5~0，接收到的有效字节数目
//===============================
u8 u8TxD2Busy;
u32 u32Com;
//===============================
void USART1_IRQHandler(void)      // 
{
	//==== UART1 TxD ========================================== 
	if(USART1->SR&(1<<6))//Trasmission complete
	{
		if(TxD1pt < TxD1Num)
		{
		  USART1->DR = USART1_TX_BUF[TxD1pt];
		  TxD1pt ++;
		}else
		{
		  u8TxD1Busy = 0;		  //USART1发送结束标志
		  TxD1pt = TxD1Num;
		}
		USART1->SR&=!(1<<6);    //TC=0
	}  // */	 
	//==== UART1 RxD ==========================================  											 
	if(USART1->SR&(1<<5))		//接收到数据
	{
		RxD1Buf=USART1->DR;
		USART1->SR&=!(1<<5);
		switch(RxD1Buf)	 {
			case '#':
				USART1_RX_BUF[0]='#';
				RxD1pt=1;
			break; 
			case '.':		// 连接PC机通讯，结束符<.>
				switch(USART1_RX_BUF[1])	{
//==== A ====
					case 'A':	// 
							UART1ComReply();		// 指令回报
							break;
//==== S ==== 
					case 'S':		// 
					
							break;
//==== K ===== 
					case 'K':		//
						
							break;
//==== G ===== 
					case 'G':		// 
							
							break;} 
//=======defailt====
			default:
				USART1_RX_BUF[RxD1pt]=RxD1Buf;
				RxD1pt++;
				if(RxD1pt>=40) {RxD1pt=40; u8ErrCode=115;}	// 接收指令错误
			break;	} 
	}  											 
} 
#endif										 
//=========================================================
// USART2 中断服务程序		  与下位机通讯
//=========================================================
#ifdef EN_USART2_RX   //如果使能了接收
//串口2中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
//接收状态
//bit7，接收完成标志
//bit6，接收到0x0d
//bit5~0，接收到的有效字节数目
//===============================
u8 u8TxD2Busy;

u32 u32Com;
//===============================
void USART2_IRQHandler(void)      // 
{
//========UART2 PE校验错误判断===============================
	if(USART2->SR&1)		//PE校验错误，软件清零，看说明书
	{			
		rData1Temp=USART2->DR;
		USART2->SR&=!1;	
	}				  
//========UART2 ORE过载错误判断===============================
	if(USART2->SR&(1<<3))
	{				
		rData2Temp=USART2->DR;	
		USART2->SR&=!(1<<3);				
	}	
//========UART2 FE帧错误判断===============================
	if(USART2->SR&(1<<1))		//FE帧错误，软件清零，看说明书
	{			
		USART2->SR&=!(1<<1);	
		rData2Temp=USART2->DR;		
	}			
	//==== UART2 RxD ==========================================  											 
	if(USART2->SR&(1<<5))		//接收到数据
	{
		RxD2Buf=USART2->DR;
		USART2->SR&=!(1<<5);
		switch(RxD2Buf)	 {
			case '#':
				USART2_RX_BUF[0]='#';
				RxD2pt=1;
			break; 
			case  '.':		// 连接PC机通讯，结束符<.>
				switch(USART2_RX_BUF[1])	{
//==== D ===== 
					case 'D':		// 
							u8test1=USART2_RX_BUF[2]-0x30;
							key_press = 1;											
							break;
//==== S ==== 
					case 'S':		// 
							u8test1=USART2_RX_BUF[2]-0x30;//#S1,0,1,6000,6000,6000.

							break;
//==== H ===== 
					case 'H':		// 

							break;
//==== G ===== 
					case 'G':		// 
							u8test1=USART2_RX_BUF[2];

							break;
//==== P ===== 
					case 'P':		// 

							break;

//==== r ===== 
					case 'r':		// 
						
							break;} 
//=======defailt====
			default:
				USART2_RX_BUF[RxD2pt]=RxD2Buf;
				RxD2pt++;
				if(RxD2pt>=40) {RxD2pt=40; u8ErrCode=115;}	// 接收指令错误
			break;	} 
	}  		
//==== UART2 TxD ====  											 
	if(USART2->SR&(1<<6))//Trasmission complete
	{
		USART2->SR&=!(1<<6);    //TC=0
		if(TxD2pt < TxD2Num)
		{
		  USART2->DR = USART2_TX_BUF[TxD2pt];
		  TxD2pt ++;
//		  USART1->SR&=!(1<<6);    //TC=0
		}else
		{
		  u8TxD2Busy = 0;
		  TxD2pt = TxD2Num;
		}
	}  // */	 	
} 
#endif										 
//=========================================================
//初始化IO 串口1
//pclk2:PCLK2时钟频率(Mhz)
//bound:波特率
//=========================================================
void uart_init1(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV
	mantissa=temp;				 						 		 //得到整数部分
	fraction=(temp-mantissa)*16; 					 //得到小数部分	 
  mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<2;   								 //使能PORTA口时钟  
	RCC->APB2ENR|=1<<14;  								 //使能串口时钟 
	GPIOA->CRH&=0XFFFFF00F;								 //IO状态设置	  RxD1浮空输入，TxD1复用输出
	GPIOA->CRH|=0X000004B0;								 //IO状态设置
		  
	RCC->APB2RSTR|=1<<14;   //复位串口1
	RCC->APB2RSTR&=~(1<<14);//停止复位	   	   
//波特率设置
 	USART1->BRR=mantissa; // 波特率设置	 
	USART1->CR1|=0X200C;  //1位停止,无校验位.
//#ifdef EN_USART1_RX		  //如果使能了接收
//使能接收中断
//	USART1->SR&=!(1<<6);    //TC
//	USART1->SR&=!(1<<7);    //TC
//	USART1->CR1|=1<<8;    //PE中断使能(校验错中断)
	USART1->CR1|=1<<5;    //接收缓冲区非空中断使能	    	
	USART1->CR1|=1<<6;    //发送缓冲区非空中断使能	    	
	MY_NVIC_Init(3,3,USART1_IRQn,2);//组2， 优先级 
//#endif
}
//=========================================================
//初始化IO 串口2
//pclk2:PCLK2时钟频率(Mhz)
//bound:波特率
//=========================================================
void uart_init2(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分	 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<2;   //使能PORTA口时钟  
	RCC->APB1ENR|=1<<17;  //使能串口时钟 
	GPIOA->CRL&=0XFFFF00FF;//IO状态设置	  RxD2浮空输入，TxD2复用输出
	GPIOA->CRL|=0X00004B00;//IO状态设置
		  
	RCC->APB1RSTR|=1<<17;   //复位串口2
	RCC->APB1RSTR&=~(1<<17);//停止复位	   	   
//波特率设置
 	USART2->BRR=mantissa; // 波特率设置	 
	USART2->CR1|=0X200C;  //1位停止,无校验位.
//#ifdef EN_USART2_RX		  //如果使能了接收
//使能接收中断
//	USART1->SR&=!(1<<6);    //TC
//	USART1->SR&=!(1<<7);    //TC
//	USART2->CR1|=1<<8;    //PE中断使能(校验错中断)
	USART2->CR1|=1<<5;    //接收缓冲区非空中断使能	    	
	USART2->CR1|=1<<6;    //发送完成中断使能	    	
	MY_NVIC_Init(1,2,USART2_IRQn,3);//组2， 优先级 
//#endif
}
//=========================================================
// UART1 指令回报
//
//				
//=========================================================
void UART1ComReply()
{
	USART1_TX_BUF[1]='A';
	USART1_TX_BUF[2]=0x31;	
	USART1_TX_BUF[3]=0x32;
	
	USART1_TX_BUF[4]=0x0d;
	USART1_TX_BUF[5]=0x0a;	

	TxD1pt = 1;
	TxD1Num= 6;
	USART1->DR = USART1_TX_BUF[0];
}
//=========================================================
// UART2 指令回报
// 
//			
//=========================================================
void UART2ComReply(u8 ComName)
{
	USART2_TX_BUF[0] = 0x30;
	USART2_TX_BUF[1] = 0x30;
	USART2_TX_BUF[2] = 0x30;
	USART2_TX_BUF[3] = 0x30;
	USART2_TX_BUF[4] = 0x30;
	USART2_TX_BUF[5] = 0x30;
	USART2_TX_BUF[6] = 0x0d;
	USART2_TX_BUF[7] = 0x0a;
	
	TxD2pt = 1;
	TxD2Num= 8;
	USART2->DR = USART2_TX_BUF[0];
}


