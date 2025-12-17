#include "delay.h"
#include "AS5600.h"
//////////////////////////////////////////////////////////////////////////////////	 
//								  
//////////////////////////////////////////////////////////////////////////////////


//初始化IIC接口
void AS5600_Init(void)
{
	IIC_Init();     // 开启I2C接口 (PB10, PB11)
    
    //--- 配置 DIR 和 GPO 引脚 ---
    // 1. 开启 GPIOB 时钟
    RCC->APB2ENR |= 1<<3; 

    // 2. 配置 PB0 (DIR) 为推挽输出
    // DIR引脚控制旋转方向: Low=CW, High=CCW
    // 操作 CRL 寄存器的 bits 0-3 (对应 PB0)
    GPIOB->CRL &= ~(0x0000000F); // 清除 PB0 设置
    GPIOB->CRL |=  (0x00000003); // MODE=11(50MHz), CNF=00(Push-Pull) -> 0x3

    // 3. 配置 PB1 (GPO) 为输入上拉
    // 用于检测磁铁状态 (需确认硬件是否连接 PB1)
    // 操作 CRL 寄存器的 bits 4-7 (对应 PB1)
    GPIOB->CRL &= ~(0x000000F0); // 清除 PB1 设置
    GPIOB->CRL |=  (0x00000080); // MODE=00(Input), CNF=10(Pull-up/down) -> 0x8

    // 4. 设置 PB1 内部上拉 (因为是输入模式，写ODR为1即为上拉)
    GPIOB->ODR |= GPIO_ODR_ODR1; 

    // 5. 初始化默认方向 (顺时针)
    AS5600_DIR_CW();
}
//************************************************************************
//* 读取 AS5600 的原始角度 (0-4095)
// ***********************************************************************/
u16 AS5600_GetRawAngle(void)
{
    u8 high_byte, low_byte;

    // 1. 发送写命令，设置寄存器指针到 0x0C
    IIC_Start();
    IIC_Send_Byte(AS5600_ADDR_WRITE); // 发送设备写地址
    if (IIC_Wait_Ack()) {             // 等待应答 (0表示应答成功)
        IIC_Stop();
        return 0; // 错误处理：设备未响应
    }

    IIC_Send_Byte(AS5600_RAW_ANGLE);  // 发送寄存器地址
    if (IIC_Wait_Ack()) {
        IIC_Stop();
        return 0;
    }

    // 2. 发送读命令，开始读取数据
    IIC_Start();                      // 重复起始信号 (Restart)
    IIC_Send_Byte(AS5600_ADDR_READ);  // 发送设备读地址
    if (IIC_Wait_Ack()) {
        IIC_Stop();
        return 0;
    }

    // 3. 读取高 8 位
    // 传入 1 表示读取后发送 ACK，请求下一个字节
    high_byte = IIC_Read_Byte(1);     

    // 4. 读取低 8 位
    // 传入 0 表示读取后发送 NACK，告诉从机读完了
    low_byte = IIC_Read_Byte(0);      

    IIC_Stop();                       // 停止信号

    // 5. 组合数据
    return ((u16)high_byte << 8) | low_byte;
}
///************************************************************************
// 读取磁铁状态 (可选功能)
//	return 1: 磁铁正常, 0: 异常
// ***********************************************************************/
u8 AS5600_GetStatus(void)
{
    u8 status;
    IIC_Start();
    IIC_Send_Byte(AS5600_ADDR_WRITE);
    IIC_Wait_Ack();
    IIC_Send_Byte(0x0B); // STATUS 寄存器
    IIC_Wait_Ack();
    
    IIC_Start();
    IIC_Send_Byte(AS5600_ADDR_READ);
    IIC_Wait_Ack();
    
    status = IIC_Read_Byte(0); // 只读一个字节，发送 NACK
    IIC_Stop();
    
    // 检查 MD (Magnet Detected) 位，第5位
    if (status & (1 << 5)) {
        return 1; // 检测到磁铁
    }
    return 0;
}

