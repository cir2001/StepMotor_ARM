#include "can.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
#include "oled.h"
#include "port.h"
//////////////////////////////////////////////////////////////////////////////////	 
u8 CAN_RX_Flag;
u8 CAN_RX_BUF[8];

u16 CAN_RX_ID;
extern int32_t Target_Speed_Hz;
//////////////////////////////////////////////////////////////////////////////////
//CAN初始化
//位时间设置：
//tsjw:重新同步跳跃时间单元.范围:1~3;
//tbs2:时间段2的时间单元.范围:1~8;
//tbs1:时间段1的时间单元.范围:1~16;
//brp :波特率分频器.范围:1~1024;(实际要加1,也就是1~1024) tq=(brp)*tpclk1
//注意以上参数任何一个都不能设为0,否则会乱.
//波特率=Fpclk1/((tbs1+tbs2+1)*brp);
//Fpclk1的时钟在初始化的时候设置为36M
//因此，brp  = (36000000 / 18) / 500000; 500000将波特率设置为500kbps,18为 TSEG1+TSEG2+SJW的值
//如果设置SJW=1, BS1（TSEG1）=12, BS2（TSEG2）=5
//参数       可设置的范围    实际设置的有效范围
//SJW        0-3						1-4tq		
//BS1 (TS1)  0-15						1-16tq
//BS2 (TS2)  0-7						1-8tq     注：寄存器值是 (实际值 - 1)
//则波特率为:36M/((1+12+5)*4)=500Kbps；(1+12+5)为实际设置在寄存器的值
//返回值:0,初始化OK;
//    其他,初始化失败;
u8 CAN_Mode_Init(void)
{
	u16 i=0;
	u8 brp;
	/* 1. 使能 CAN1 和 GPIOA 时钟 */
	RCC->APB1ENR |= (1 << 25);    	//使能 CAN1 时钟  CAN使用的是APB1的时钟(max:36M)
	RCC->APB2ENR |= (1 << 2);     	//使能PORTA时钟	 	 
	
	/* 2. 配置 PA11 (CAN_RX)为上拉输入、PA12(CAN_TX)为复用推挽输出 */
	GPIOA->CRH &= 0XFFF00FFF; 
	GPIOA->CRH |= 0X000B8000;		//PA11 RX,PA12 TX推挽输出   	 
  	GPIOA->ODR |= 3<<11;			//PA 上拉
					    
	/* 3. 退出 SLEEP 模式 (若处于睡眠状态) */
	CAN1->MCR = 0x00000;			//退出睡眠模式(同时设置所有位为0)
	
	/* 4. 进入初始化模式 */
	CAN1->MCR |= 1<<0;				//请求CAN进入初始化模式
	while((CAN1->MSR & (1<<0)) == 0)
	{
		i++;
		if(i > 100) return 2;		//进入初始化模式失败
	}
	
	/* 5. 关闭 TTCM/ABOM/AWUM/RFLM/TXFP (均为disable) */
	CAN1->MCR |= 0<<7;		//关闭非时间触发通信模式
	CAN1->MCR |= 0<<6;		//关闭软件自动离线管理
	CAN1->MCR |= 0<<5;		//关闭睡眠模式通过软件唤醒(清除CAN1->MCR的SLEEP位)	
	CAN1->MCR |= 0<<3;		//关闭报文不锁定,新的覆盖旧的
	CAN1->MCR |= 0<<2;		//关闭优先级由报文标识符决定
	
	/* 6. 关闭自动重传 (NART=1)  */
	CAN1->MCR |= 1<<4;		//关闭禁止报文自动传送
	
	/* 7. 设置波特率  */
	/* Note: this calculations fit for PCLK1 = 36MHz */
  	brp  = (36000000 / 18) / 500000;                     // baudrate is set to 500k bit/s
	/* set BTR register so that sample point is at about 72% bit time from bit start */
	/* TSEG1 = 15, TSEG2 = 4, SJW = 2 => 1 CAN bit = 18 TQ, sample at 72%    */
  	CAN1->BTR &= ~(((        0x03) << 24) | ((        0x07) << 20) | ((         0x0F) << 16) | (          0x1FF)); 
	//--- 波特率:Fpclk1/((Tbs1+Tbs2+1)*Fdiv) ---//
	CAN1->BTR |=  ((((2-1) & 0x03) << 24) | (((4-1) & 0x07) << 20) | (((13-1) & 0x0F) << 16) | ((brp-1) & 0x1FF)); 
	
	// 【修改】强制开启回环模式 (LBKM = Bit 30)
	//CAN1->BTR |= (1 << 30); //测试MCU内部回环测试
	
	/* 8. 退出初始化模式 */
	CAN1->MCR &= ~(1<<0);		//请求CAN退出初始化模式
	while((CAN1->MSR & (1<<0)) == 1)
	{
		i++;
		if(i > 0XFFF0) return 3;//退出初始化模式失败
	}
	
	/* 9. 配置过滤器0 (可配置的过滤器0-13) */
	CAN1->FMR |= 1<<0;			//过滤器组工作在初始化模式
	CAN1->FA1R &= ~(1<<0);		//过滤器0不激活

	CAN_Filter_MODE_32BIT_MASK();

#if CAN_RX0_INT_ENABLE
 	//使用中断接收
	CAN1->IER|=1<<1;		//FIFO0消息挂号中断允许.	    
	MY_NVIC_Init(2,0,USB_LP_CAN1_RX0_IRQn,2);//组2
#endif
	return 0;
}   
//id:标准ID(11位)/扩展ID(11位+18位)	    
//ide:0,标准帧;1,扩展帧
//rtr:0,数据帧;1,远程帧
//len:要发送的数据长度(固定为8个字节,在时间触发模式下,有效数据为6个字节)
//*dat:数据指针.
//返回值:0~3,邮箱编号.0XFF,无有效邮箱.
u8 CAN_Tx_Msg(u32 id,u8 ide,u8 rtr,u8 len,u8 *dat)
{	   
	u8 mbox;	  
	if(CAN1 -> TSR & (1 << 26)) mbox=0;			//邮箱0为空
	else if(CAN1 -> TSR & (1 << 27)) mbox=1;	//邮箱1为空
	else if(CAN1 -> TSR & (1 << 28)) mbox=2;	//邮箱2为空
	else return 0XFF;							//无空邮箱,无法发送 
	CAN1 -> sTxMailBox[mbox].TIR = 0;			//清除之前的设置
	if(ide == 0)	//标准帧
	{
		id &= 0x7ff;		//取低11位stdid
		id <<= 21;		  
	}else			//扩展帧
	{
		id &= 0X1FFFFFFF;	//取低32位extid
		id <<= 3;									   
	}
	CAN1 -> sTxMailBox[mbox].TIR |= id;		 
	CAN1 -> sTxMailBox[mbox].TIR |= ide << 2;	  
	CAN1 -> sTxMailBox[mbox].TIR |= rtr << 1;
	len &= 0X0F;//得到低四位
	CAN1 -> sTxMailBox[mbox].TDTR &= ~(0X0000000F);
	CAN1 -> sTxMailBox[mbox].TDTR |= len;	//设置DLC.
	//待发送数据存入邮箱.
	CAN1 -> sTxMailBox[mbox].TDHR = (((u32)dat[7] << 24) |
									((u32)dat[6] << 16) |
 									((u32)dat[5] << 8) |
									((u32)dat[4]));
	CAN1 -> sTxMailBox[mbox].TDLR=(((u32)dat[3] << 24) |
									((u32)dat[2] << 16) |
 									((u32)dat[1] << 8) |
									((u32)dat[0]));
	CAN1 -> sTxMailBox[mbox].TIR |= 1 << 0; 	//请求发送邮箱数据
	return mbox;
}
//获得发送状态.
//mbox:邮箱编号;
//返回值:发送状态. 0,挂起;0X05,发送失败;0X07,发送成功.
u8 CAN_Tx_Staus(u8 mbox)
{	
	u8 sta=0;					    
	switch (mbox)
	{
		case 0: 
			sta |= CAN1 -> TSR & (1 << 0);				//RQCP0
			sta |= CAN1 -> TSR & (1 << 1);				//TXOK0
			sta |= ((CAN1 -> TSR & (1 << 26)) >> 24);	//TME0
			break;
		case 1: 
			sta |= CAN1 -> TSR & (1 << 8) >> 8;			//RQCP1
			sta |= CAN1 -> TSR & (1 << 9) >> 8;			//TXOK1
			sta |= ((CAN1 -> TSR & (1 << 27)) >> 25);	//TME1	   
			break;
		case 2: 
			sta |= CAN1 -> TSR & (1 << 16) >>16;		//RQCP2
			sta |= CAN1 -> TSR & (1 << 17) >>16;		//TXOK2
			sta |= ((CAN1 -> TSR & (1 << 28)) >>26);	//TME2
			break;
		default:
			sta = 0X05;//邮箱号不对,肯定失败.
		break;
	}
	return sta;
} 
//得到在FIFO0/FIFO1中接收到的报文个数.
//fifox:0/1.FIFO编号;
//返回值:FIFO0/FIFO1中的报文个数.
u8 CAN_Msg_Pend(u8 fifox)
{
	if(fifox == 0) return CAN1 -> RF0R&0x03; 
	else if(fifox == 1) return CAN1 -> RF1R&0x03; 
	else return 0;
}
//接收数据
//fifox:邮箱号
//id:标准ID(11位)/扩展ID(11位+18位)	    
//ide:0,标准帧;1,扩展帧
//rtr:0,数据帧;1,远程帧
//len:接收到的数据长度(固定为8个字节,在时间触发模式下,有效数据为6个字节)
//dat:数据缓存区
void CAN_Rx_Msg(u8 fifox,u32 *id,u8 *ide,u8 *rtr,u8 *len,u8 *dat)
{	   
	*ide = CAN1 -> sFIFOMailBox[fifox].RIR&0x04;	//得到标识符选择位的值  
 	if(*ide == 0)//标准标识符
	{
		*id = CAN1 -> sFIFOMailBox[fifox].RIR>>21;
	}else	   //扩展标识符
	{
		*id = CAN1 -> sFIFOMailBox[fifox].RIR>>3;
	}
	*rtr = CAN1 -> sFIFOMailBox[fifox].RIR & 0x02;	//得到远程发送请求值.
	*len = CAN1 -> sFIFOMailBox[fifox].RDTR & 0x0F;	//得到DLC
 	//*fmi=(CAN1->sFIFOMailBox[FIFONumber].RDTR>>8)&0xFF;//得到FMI
	//接收数据
	dat[0] = (CAN1 -> sFIFOMailBox[fifox].RDLR) & 0XFF;
	dat[1] = (CAN1 -> sFIFOMailBox[fifox].RDLR>>8) & 0XFF;
	dat[2] = (CAN1 -> sFIFOMailBox[fifox].RDLR>>16) & 0XFF;
	dat[3] = (CAN1 -> sFIFOMailBox[fifox].RDLR>>24) & 0XFF;    
	dat[4] = (CAN1 -> sFIFOMailBox[fifox].RDHR) & 0XFF;
	dat[5] = (CAN1 -> sFIFOMailBox[fifox].RDHR>>8) & 0XFF;
	dat[6] = (CAN1 -> sFIFOMailBox[fifox].RDHR>>16) & 0XFF;
	dat[7] = (CAN1 -> sFIFOMailBox[fifox].RDHR>>24) & 0XFF;    
  	if(fifox == 0) CAN1->RF0R |= 0X20;//释放FIFO0邮箱
	else if(fifox == 1) CAN1->RF1R |= 0X20;//释放FIFO1邮箱	 
}

#if CAN_RX0_INT_ENABLE	//使能RX0中断
//中断服务函数			    
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	/*u32 id;
	u8 ide,rtr,len;     
 	CAN_Rx_Msg(0,&id,&ide,&rtr,&len,CAN_RX_BUF);
	CAN_RX_ID = id & 0xFFFF;
	CAN_RX_Flag = 1;*/
	if(CAN1->RF0R & CAN_RF0R_FMP0) // FIFO0 有数据
    {
		u32 id;
		u8 len;
		int32_t recv_val;
		LED0 = !LED0;
		// 1. 读取 ID (标准帧 ID 在 RIR 寄存器高 11 位)
        id = (CAN1->sFIFOMailBox[0].RIR >> 21) & 0x7FF;; // 读取标准 ID
        
		// 2. 读取数据长度 DLC
        len = CAN1->sFIFOMailBox[0].RDTR & 0x0F;

        // 3. 读取数据 (RDLR 低4字节, RDHR 高4字节)
        // 假设前 4 字节是 int32 速度
        recv_val = 0;
        
        // 直接从寄存器拼装数据 (假设小端发送)
        // RDLR: [Data3 | Data2 | Data1 | Data0]
        recv_val = CAN1->sFIFOMailBox[0].RDLR; // 直接读出整个32位寄存器作为int32
        // 如果需要逐字节序处理：
        // ((CAN1->sFIFOMailBox[0].RDLR >> 0) & 0xFF) ...

        // 4. 处理数据 (这里不需要再判断 ID 了，因为硬件只放行 100-10F)
        if(recv_val > 25000) recv_val = 25000;
        if(recv_val < -25000) recv_val = -25000;
        
        Target_Speed_Hz = recv_val;

        // 释放邮箱
        CAN1->RF0R |= CAN_RF0R_RFOM0;
    }
}
#endif

//can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
//len:数据长度(最大为8)				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//		其他,失败;
u8 CAN_Send_Msg(u8* msg,u8 len)
{	
	u8 mbox;
	u16 i=0;	  	 						       
    mbox=CAN_Tx_Msg(0X12,0,0,len,msg);
	while((CAN_Tx_Staus(mbox) != 0X07) && (i<0XFFF)) i++;//等待发送结束
	if(i >= 0XFFF) return 1;							//发送失败?
	return 0;										//发送成功;
}
//can口接收数据查询
//buf:数据缓存区;	 
//返回值:0,无数据被收到;
//		 其他,接收的数据长度;
u8 CAN_Receive_Msg(u8 *buf)
{		   		   
	u32 id;
	u8 ide,rtr,len; 
	if(CAN_Msg_Pend(0)==0)return 0;			//没有接收到数据,直接退出 	 
  	CAN_Rx_Msg(0,&id,&ide,&rtr,&len,buf); 	//读取数据
    if(id!=0x12||ide!=0||rtr!=0)len=0;		//接收错误	   
	return len;	
}
//--------------------------------------------------------------------------
//can口发送一组数据至上位机F407(固定格式:CAN_ID_MOTOR_FB,标准帧,数据帧)
//real_speed:当前实际的速度
//pos_error:位置偏差.		
//--------------------------------------------------------------------------
void CAN_Send_Feedback(int32_t real_speed, int32_t pos_error)
{
    // 检查邮箱0是否空闲
    if ((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) 
    {
        // 1. 配置 TIR (ID 寄存器)
        // 明确设置：标准 ID (<< 21), IDE=0, RTR=0 (数据帧)
        // 暂不设置 TXRQ
        CAN1->sTxMailBox[0].TIR = ((uint32_t)CAN_ID_MOTOR_FB << 21); 

        // 2. 配置 TDTR (数据长度和控制)
        // 数据长度为 8 字节
        // IDE 和 RTR 默认为 0 (标准ID，数据帧)
        CAN1->sTxMailBox[0].TDTR = 8; 

        // 3. 填充 TDLR (Data Low Register, 字节 0-3)
        // 直接赋值 32 位数据，无需手动拆分字节
        CAN1->sTxMailBox[0].TDLR = (uint32_t)real_speed;
        
        // 4. 填充 TDHR (Data High Register, 字节 4-7)
        // 直接赋值 32 位数据
        CAN1->sTxMailBox[0].TDHR = (uint32_t)pos_error;

        // 5. 触发发送 (设置 TXRQ 位)
        CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ; 
    }
}
//--------------------------------------------------------------------------
//16bit list mode 4个ids
//--------------------------------------------------------------------------
void CAN_Filter_MODE_16BIT_LIST(void)
{
	//--- 允许通过的id 4*16bit list mode test ---//
	/*u16 id0 = 0x0058;
	u16 id1 = 0x0158;
	u16 id2 = 0x0258;
	u16 id3 = 0x0358;*/
	//****  16bit*4 list mode 可以通过4个指定的ID ****//
	/*CAN1->FM1R |= 1<<0;		//过滤器0工作在标识符list模式
	CAN1->FS1R &= ~(1<<0); 		//过滤器0位宽为16位*4.

	//--- FR1 高 16 位 id0，低 16 位 id1 ---//
	CAN1->sFilterRegister[0].FR1 = 
			((u32)id0 << 21) | (id1 << 5);
	//--- FR2 高 16 位 id3，低 16 位 id4 ---//			
	CAN1->sFilterRegister[0].FR2 = 
			((u32)id2 << 21) | (id3 << 5);*/

	//*************************************************************
	// 过滤器配置 - 接收 0x100, 0x101, 0x102, 0x103
	// 模式设置	
	CAN1->FS1R &= ~(1 << 0);       // 设为 16位宽
	CAN1->FM1R |= (1 << 0);       // 设为 列表模式 (List Mode)

	// --- 写入 ID 和 Mask ---
	// 目标范围: 0x100 ~ 0x103
	// 标准 ID 在寄存器的高 11 位 [31:21]
	// FR1 (ID): 0x100, 0x10	, 0x102, 0x103
	CAN1->sFilterRegister[0].FR1 = (0x100 << 21) |
	                                   (0x101 << 5);	// 低 16 位
	// FR2 (ID): 0x102, 0x10		, 0x103, 0x104
	CAN1->sFilterRegister[0].FR2 = (0x102 << 21) |
	                                   (0x103 << 5);	// 低 16 位	// 关联与激活
	CAN1->FFA1R &= ~(1 << 0);     // 关联到 FIFO 0
	CAN1->FA1R |= (1 << 0);       // 激活过滤器0
	CAN1->FMR &= ~(1 << 0);       // 过滤器组进入正常模式
}
//--------------------------------------------------------------------------
//16bit mask mode 2个ids
//--------------------------------------------------------------------------
void CAN_Filter_MODE_16BIT_MASK(void)
{
	//--- 允许通过的id 2*16bit mask mode test ---//
	/*u16 id0 = 0x200;		//起始id 0x200
	//-- mask允许0x200-0x20F通过过滤器，0x0c 表示RTR=0 IDE=1
	// bit7 bit6 bit5 bit4(RTR) bit3(IDE) bit2 bit1 bit0
	//   0    0    0    0            1      1    0    0
	// bit2为1的理解：保留位 不对CAN ID起作用，实现更严格的匹配
	u16 mask0 = 0xFE0C;		
	u16 id1 = 0x0300;
	u16 mask1 = 0xFE0C;*/
	//****  16bit*2 mask mode 可以通过一个范围内id，例如：0x200-0x02F ****//
	/*CAN1->FM1R &= ~(1<<0);		//过滤器0工作在标识符mask模式
	CAN1->FS1R &= ~(1<<0); 		//过滤器0位宽为16位*4.

	//--- FR1 高 16 位 id0，低 16 位 id1 ---//
	CAN1->sFilterRegister[0].FR1 = 
			((u32)mask0 << 16) | (id0 << 5);
	//--- FR2 高 16 位 id3，低 16 位 id4 ---//			
	CAN1->sFilterRegister[0].FR2 = 
			((u32)mask1 << 16) | (id1 << 5);*/
	
	//*********************************************************************
	// 过滤器配置 - 接收 0x100, 0x101, 0x102, 0x103
	// 模式设置	
	CAN1->FS1R &= ~(1 << 0);       // 设为 16位宽
	CAN1->FM1R &= ~(1 << 0);      // 设为 掩码模式 (Mask Mode)
	// --- 写入 ID 和 Mask ---
	// 目标范围: 0x100 ~ 0x103
	// 标准 ID 在寄存器的高 11 位 [31:21]
	// FR1 (ID): 0x100, 0x101, 0x102, 0x103
	CAN1->sFilterRegister[0].FR1 = (0x100 << 21) |
	                                   (0x101 << 5);	// 低 16 位			
	// FR2 (Mask): 0x102, 0x103, 0x104, 0x105
	CAN1->sFilterRegister[0].FR2 = (0x102 << 21) |
	                                   (0x103 << 5);	// 低 16 位
	// 关联与激活
	CAN1->FFA1R &= ~(1 << 0);     // 关联到 FIFO 0
	CAN1->FA1R |= (1 << 0);       // 激活过滤器0
	CAN1->FMR &= ~(1 << 0);       // 过滤器组进入正常模式
}
//--------------------------------------------------------------------------
//32bit list mode 4个ids
//--------------------------------------------------------------------------
void CAN_Filter_MODE_32BIT_LIST(void)
{
	// 过滤器配置 - 接收 0x100, 0x101, 0x102, 0x103
	// 模式设置	
	CAN1->FS1R |= (1 << 0);       // 设为 32位宽
	CAN1->FM1R |= (1 << 0);       // 设为 列表模式 (List Mode)
	// --- 写入 ID 和 Mask ---
	// 目标范围: 0x100 ~ 0x103
	// 标准 ID 在寄存器的高 11 位 [31:21]
	// FR1 (ID): 0x100, 0x101, 0x102, 0x103
	CAN1->sFilterRegister[0].FR1 = (0x100 << 21) |
	                                   (0x101 << 5);	// 低 16 位			
	// FR2 (ID): 0x102, 0x103, 0x104, 0x105
	CAN1->sFilterRegister[0].FR2 = (0x102 << 21) |
	                                   (0x103 << 5);	// 低 16 位
	// 关联与激活
	CAN1->FFA1R &= ~(1 << 0);     // 关联到 FIFO 0
	CAN1->FA1R |= (1 << 0);       // 激活过滤器0
	CAN1->FMR &= ~(1 << 0);       // 过滤器组进入正常模式
}
//--------------------------------------------------------------------------
//32bit mask mode 
//--------------------------------------------------------------------------
void CAN_Filter_MODE_32BIT_MASK(void)
{
	//****  32bit mask mode 所有ID均通过过滤器，即没有屏蔽的设置 ****//
	/*CAN1->FM1R |= 0<<0;		//过滤器0工作在标识符屏蔽位模式
	CAN1->FS1R |= 1<<1; 		//过滤器0位宽为32位.
	
	//--- 1个32位ID id由于屏蔽寄存器FR2设置全0，因此可以设为初始值，全部id通过 ---//
	CAN1->sFilterRegister[0].FR1 = 0X00000000;
	//--- 1个32位MASK, 全0表示不关心每一位，也就是所有id通过 ---//
	CAN1->sFilterRegister[0].FR2 = 0X00000000;*/

	//*******************************************************************
	// 过滤器配置 - 接收 0x100 ~ 0x10F
	// 模式设置
	CAN1->FS1R |= (1 << 0);       // 设为 32位宽
	CAN1->FM1R &= ~(1 << 0);      // 设为 掩码模式 (Mask Mode)

	// --- 写入 ID 和 Mask ---
	// 目标范围: 0x100 ~ 0x10F
	// 标准 ID 在寄存器的高 11 位 [31:21]
	// FR1 (ID): 基准值 0x100
	CAN1->sFilterRegister[0].FR1 = ((u32)CAN_ID_FILTER_START << 21);
	// FR2 (Mask): 掩码设置。1 表示必须匹配 FR1 对应位
    // ID Mask: 0x7F0 (匹配前 7 位)
    // IDE Mask: 1 << 2 (匹配 IDE=0，即标准帧)
    // RTR Mask: 1 << 4 (匹配 RTR=0，即数据帧)
    CAN1->sFilterRegister[0].FR2 = ((u32)0x7F0 << 21) | // ID 掩码 (0x7F0)
                                   (1 << 2) |           // IDE 掩码 (匹配标准帧)
                                   (1 << 4);            // RTR 掩码 (匹配数据帧)

	//--- 1个32位ID id由于屏蔽寄存器FR2设置全0，因此可以设为初始值，全部id通过 ---//
	/*CAN1->sFilterRegister[0].FR1 = 0X00000000;
	//--- 1个32位MASK, 全0表示不关心每一位，也就是所有id通过 ---//
	CAN1->sFilterRegister[0].FR2 = 0X00000000;*/
    
    CAN1->FFA1R &= ~(1<<0); // FIFO0
    CAN1->FA1R |= 1<<0;     // 激活
    CAN1->FMR &= ~(1<<0);   // 退出初始化
}


