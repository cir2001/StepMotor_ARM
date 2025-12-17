#include "stepMotor.h"
#include "port.h"

void StepMotor_Init(void)
{
    // ==========================================
    // 1. 开启时钟
    // ==========================================
    // TIM1 和 GPIOB 都挂载在 APB2 总线上
    RCC->APB2ENR |= 1 << 11;	//使能TIM1时钟
    RCC->APB2ENR |= 1<<3;       // 开启 GPIOB 时钟

    // ==========================================
    // 2. 配置 GPIO 模式 (PB14 = STEP)
    // ==========================================
    // PB14 必须配置为 复用推挽输出 (AF_PP, 0xB)
    // 假设 PB12, PB13, PB15 是其他控制脚，这里只演示 PB14 配置
    
    // 清除 PB14 配置 (CRH 控制 PB8-15)
    // PB14 在 CRH 的 [27:24] 位
    GPIOB->CRH &= 0xF0FFFFFF; 
    
    // 设置 PB14 为 复用推挽输出 (0xB)
    GPIOB->CRH |= 0x0B000000; 
    GPIOB->ODR |= 1 << 14;  // 确保 PB14 初始为高电平

    Motor_M2 = 1;   
    Motor_M1 = 0;   
    Motor_M0 = 0; 
    
    Motor_Ena = 0;  // 使能 (Low有效，锁死电机)
    Motor_Dir = 1;  // 方向 (High 正转，Low 反转)

    // ==========================================
    // 3. 配置 TIM1 PWM (针对 PB14 / CH2N)
    // ==========================================
    
    // 3.1 预分频
    // TIM1 在 APB2 上，频率也是 72MHz
    TIM1->PSC = 71;  // 1MHz 计数频率

    // 3.2 初始速度设为 0 (安全)
    TIM1->ARR = 0;   
    TIM1->CCR2 = 0;  // 注意：PB14 对应的是 Capture/Compare Register 2

    // 3.3 配置 PWM 模式
    // 操作 CCMR1 寄存器中的 OC2M (通道2)
    // 设置为 PWM 模式 1 (110) 并开启预装载 (OC2PE)
    // 注意：CCMR1 的高8位控制通道2，低8位控制通道1
    TIM1->CCMR1 |= (6 << 12) | (1 << 11); // OC2M=110, OC2PE=1

    // 3.4 开启互补输出通道 (CH2N)
    // 普通定时器用 CC2E，但在 PB14 上必须用 CC2NE (N代表互补)
    TIM1->CCER |= TIM_CCER_CC2NE; 

    // ==========================================
    // 4. 开启主输出 (MOE)
    // ==========================================
    TIM1->BDTR |= TIM_BDTR_MOE;

    // 5. 开启定时器
    TIM1->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN; // 预装载使能和计数器使能
}
// 设定电机速度 (在 PID 计算后调用)
// speed_hz: 目标频率 (Hz)，正数正转，负数反转，0 停止
void StepMotor_SetSpeed(int32_t speed_hz) 
{
    uint16_t arr_val;
    // 1. 处理方向
    if (speed_hz > 0) {
        Motor_Dir = 1; // Set Bit (High)
    } else if (speed_hz < 0) {
        Motor_Dir = 0;  // Reset Bit (Low)
        speed_hz = -speed_hz;            // 取绝对值
    } else {
        // 速度为 0，停止输出脉冲
        // 将 ARR 设为 0 或者关闭 CC1E 都可以，这里设为极大值让它几乎不动，或者直接关定时器
        TIM1->ARR = 0; 
        TIM1->CCR2 = 0;
        return;
    }

    // 2. 限制最大频率 (防止电机堵转/啸叫)
    // 假设 1/16 细分，42 电机极限大概在 40kHz 左右，根据实际情况调整
    if (speed_hz > 25000) speed_hz = 25000;
    if (speed_hz < 5) speed_hz = 5; // 防止除以0或频率过低

    // 3. 计算 ARR 值
    // 计数器频率是 1MHz (1,000,000 Hz)
    // ARR = (TimerClock / TargetFreq) - 1
    arr_val = (1000000 / speed_hz) - 1;

    // 如果当前 ARR 是 0 (说明之前是停止状态)，
    // 或者 CNT 已经超过了新的 ARR (说明新周期比当前跑的还短)，
    // 由于开启了“预装载机制” (Preload)，当 ARR 初始值为 0 时，新的参数无法被加载到影子寄存器中。
    // 由于设置了“跑完这一圈再改参数”，但因为第一圈长度为 0（永远跑不完），所以新参数永远没机会生效。
    // 必须"强制"产生一次更新事件，让新参数立即生效！
    if (TIM1->ARR == 0 || TIM1->CNT > arr_val) 
    {
        TIM1->ARR = arr_val;
        TIM1->CCR2 = arr_val / 2;
        
        // 产生更新事件 (Update Generation)
        // 这会立即把影子寄存器的值(ARR, CCR) 搬运到 影子寄存器
        TIM1->EGR |= TIM_EGR_UG; 
        
        // 注意：TIM1 是高级定时器，产生 UG 会自动清除重复计数器
        // 并且如果开启了中断，会进入一次中断服务函数(如果不需要进中断，请先关中断)
    }
    else 
    {
        // 正常运行时的平滑调速
        TIM1->ARR = arr_val;
        TIM1->CCR2 = arr_val / 2;
        // 这里不需要 EGR，等待当前周期自然结束即可平滑过渡
    }
}

