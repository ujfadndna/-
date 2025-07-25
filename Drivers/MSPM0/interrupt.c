#include "ti_msp_dl_config.h"
#include "interrupt.h"
#include "clock.h"
#include "mpu6050.h"
#include "bno08x_uart_rvc.h"
#include "wit.h"
#include "vl53l0x.h"
#include "lsm6dsv16x.h"
#include "imu660rb.h"
#include "Drivers/PID/pid.h"
#include "motor.h"
#include <stdio.h>

uint8_t enable_group1_irq = 0;

// 只保留两驱小车编码器全局变量
volatile int32_t Left_Encoder_Count = 0;   // 左轮编码器计数
volatile int32_t Right_Encoder_Count = 0;  // 右轮编码器计数

// 全局变量：左右电机速度（米/秒）和累计距离（米）
volatile int32_t Left_Motor_Speed = 0;
volatile int32_t Right_Motor_Speed = 0;
volatile float Left_Motor_Distance = 0.0f;
volatile float Right_Motor_Distance = 0.0f;

// 全局变量：左右电机物理速度（米/秒）
volatile float Left_Motor_Velocity = 0.0f;
volatile float Right_Motor_Velocity = 0.0f;

// 上次计数值（静态变量，仅在本文件使用）
static int32_t last_Left_Encoder_Count = 0;
static int32_t last_Right_Encoder_Count = 0;

// 外部声明距离换算函数
extern float DISTANCE(int pulse_count);

void Interrupt_Init(void)
{
    if(enable_group1_irq)
    {
        // 启用GROUP1中断 (包含编码器中断)
        NVIC_EnableIRQ(GPIO_Encoder_GPIOA_INT_IRQN);
        NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
        
        // 确保编码器GPIO中断已启用
        DL_GPIO_enableInterrupt(GPIO_Encoder_E1A_PORT, GPIO_Encoder_E1A_PIN);
        DL_GPIO_enableInterrupt(GPIO_Encoder_E2A_PORT, GPIO_Encoder_E2A_PIN);

        //Enable Exit of GPIO_A_GROUP
	    NVIC_EnableIRQ(GPIOA_INT_IRQn);
	    //Enable Timer PIT_FOR_CUSTOM
	    // NVIC_EnableIRQ(PIT_FOR_CUSTOM_INST_INT_IRQN);
        // DL_TimerG_startCounter(PIT_FOR_CUSTOM_INST);
	    //Enable Timer PIT_FOR_CONTROL
	    NVIC_EnableIRQ(PIT_FOR_CONTROL_INST_INT_IRQN);
        DL_TimerG_startCounter(PIT_FOR_CONTROL_INST);

        //Enable UART_0
        NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
        NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    }
}

void SysTick_Handler(void)
{
    tick_ms++;
}

#if defined UART_BNO08X_INST_IRQHandler
void UART_BNO08X_INST_IRQHandler(void)
{
    uint8_t checkSum = 0;
    extern uint8_t bno08x_dmaBuffer[19];

    DL_DMA_disableChannel(DMA, DMA_BNO08X_CHAN_ID);
    uint8_t rxSize = 18 - DL_DMA_getTransferSize(DMA, DMA_BNO08X_CHAN_ID);

    if(DL_UART_isRXFIFOEmpty(UART_BNO08X_INST) == false)
        bno08x_dmaBuffer[rxSize++] = DL_UART_receiveData(UART_BNO08X_INST);

    for(int i=2; i<=14; i++)
        checkSum += bno08x_dmaBuffer[i];

    if((rxSize == 19) && (bno08x_dmaBuffer[0] == 0xAA) && (bno08x_dmaBuffer[1] == 0xAA) && (checkSum == bno08x_dmaBuffer[18]))
    {
        bno08x_data.index = bno08x_dmaBuffer[2];
        bno08x_data.yaw = (int16_t)((bno08x_dmaBuffer[4]<<8)|bno08x_dmaBuffer[3]) / 100.0;
        bno08x_data.pitch = (int16_t)((bno08x_dmaBuffer[6]<<8)|bno08x_dmaBuffer[5]) / 100.0;
        bno08x_data.roll = (int16_t)((bno08x_dmaBuffer[8]<<8)|bno08x_dmaBuffer[7]) / 100.0;
        bno08x_data.ax = (bno08x_dmaBuffer[10]<<8)|bno08x_dmaBuffer[9];
        bno08x_data.ay = (bno08x_dmaBuffer[12]<<8)|bno08x_dmaBuffer[11];
        bno08x_data.az = (bno08x_dmaBuffer[14]<<8)|bno08x_dmaBuffer[13];
    }
    
    uint8_t dummy[4];
    DL_UART_drainRXFIFO(UART_BNO08X_INST, dummy, 4);

    DL_DMA_setDestAddr(DMA, DMA_BNO08X_CHAN_ID, (uint32_t) &bno08x_dmaBuffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_BNO08X_CHAN_ID, 18);
    DL_DMA_enableChannel(DMA, DMA_BNO08X_CHAN_ID);
}
#endif

#if defined UART_WIT_INST_IRQHandler
void UART_WIT_INST_IRQHandler(void)
{
    uint8_t checkSum, packCnt = 0;
    extern uint8_t wit_dmaBuffer[33];

    DL_DMA_disableChannel(DMA, DMA_WIT_CHAN_ID);
    uint8_t rxSize = 32 - DL_DMA_getTransferSize(DMA, DMA_WIT_CHAN_ID);

    if(DL_UART_isRXFIFOEmpty(UART_WIT_INST) == false)
        wit_dmaBuffer[rxSize++] = DL_UART_receiveData(UART_WIT_INST);

    while(rxSize >= 11)
    {
        checkSum=0;
        for(int i=packCnt*11; i<(packCnt+1)*11-1; i++)
            checkSum += wit_dmaBuffer[i];

        if((wit_dmaBuffer[packCnt*11] == 0x55) && (checkSum == wit_dmaBuffer[packCnt*11+10]))
        {
            if(wit_dmaBuffer[packCnt*11+1] == 0x51)
            {
                wit_data.ax = (int16_t)((wit_dmaBuffer[packCnt*11+3]<<8)|wit_dmaBuffer[packCnt*11+2]) / 2.048; //mg
                wit_data.ay = (int16_t)((wit_dmaBuffer[packCnt*11+5]<<8)|wit_dmaBuffer[packCnt*11+4]) / 2.048; //mg
                wit_data.az = (int16_t)((wit_dmaBuffer[packCnt*11+7]<<8)|wit_dmaBuffer[packCnt*11+6]) / 2.048; //mg
                wit_data.temperature =  (int16_t)((wit_dmaBuffer[packCnt*11+9]<<8)|wit_dmaBuffer[packCnt*11+8]) / 100.0; //°C
            }
            else if(wit_dmaBuffer[packCnt*11+1] == 0x52)
            {
                wit_data.gx = (int16_t)((wit_dmaBuffer[packCnt*11+3]<<8)|wit_dmaBuffer[packCnt*11+2]) / 16.384; //°/S
                wit_data.gy = (int16_t)((wit_dmaBuffer[packCnt*11+5]<<8)|wit_dmaBuffer[packCnt*11+4]) / 16.384; //°/S
                wit_data.gz = (int16_t)((wit_dmaBuffer[packCnt*11+7]<<8)|wit_dmaBuffer[packCnt*11+6]) / 16.384; //°/S
            }
            else if(wit_dmaBuffer[packCnt*11+1] == 0x53)
            {
                wit_data.roll  = (int16_t)((wit_dmaBuffer[packCnt*11+3]<<8)|wit_dmaBuffer[packCnt*11+2]) / 32768.0 * 180.0; //°
                wit_data.pitch = (int16_t)((wit_dmaBuffer[packCnt*11+5]<<8)|wit_dmaBuffer[packCnt*11+4]) / 32768.0 * 180.0; //°
                wit_data.yaw   = (int16_t)((wit_dmaBuffer[packCnt*11+7]<<8)|wit_dmaBuffer[packCnt*11+6]) / 32768.0 * 180.0; //°
                wit_data.version = (int16_t)((wit_dmaBuffer[packCnt*11+9]<<8)|wit_dmaBuffer[packCnt*11+8]);
            }
        }

        rxSize -= 11;
        packCnt++;
    }
    
    uint8_t dummy[4];
    DL_UART_drainRXFIFO(UART_WIT_INST, dummy, 4);

    DL_DMA_setDestAddr(DMA, DMA_WIT_CHAN_ID, (uint32_t) &wit_dmaBuffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_WIT_CHAN_ID, 32);
    DL_DMA_enableChannel(DMA, DMA_WIT_CHAN_ID);
}
#endif

void GROUP1_IRQHandler(void)
{
    // 检查GPIOA中断 (右轮编码器E2A)
    uint32_t gpioA_status = DL_GPIO_getEnabledInterruptStatus(GPIOA, GPIO_Encoder_E2A_PIN);
    if ((gpioA_status & GPIO_Encoder_E2A_PIN) == GPIO_Encoder_E2A_PIN) {
        // 读取B相状态判断方向
        if (DL_GPIO_readPins(GPIO_Encoder_E2B_PORT, GPIO_Encoder_E2B_PIN)) {
            Right_Encoder_Count--;  // B相高电平时反向
        } else {
            Right_Encoder_Count++;  // B相低电平时正向
        }
        // 清除中断标志
        DL_GPIO_clearInterruptStatus(GPIO_Encoder_E2A_PORT, GPIO_Encoder_E2A_PIN);
    }

    // 检查GPIOB中断 (左轮编码器E1A)
    uint32_t gpioB_status = DL_GPIO_getEnabledInterruptStatus(GPIOB, GPIO_Encoder_E1A_PIN);
    if ((gpioB_status & GPIO_Encoder_E1A_PIN) == GPIO_Encoder_E1A_PIN) {
        // 读取B相状态判断方向
        if (DL_GPIO_readPins(GPIO_Encoder_E1B_PORT, GPIO_Encoder_E1B_PIN)) {
            Left_Encoder_Count--;   // B相高电平时反向
        } else {
            Left_Encoder_Count++;   // B相低电平时正向
        }
        // 清除中断标志
        DL_GPIO_clearInterruptStatus(GPIO_Encoder_E1A_PORT, GPIO_Encoder_E1A_PIN);
    }

    // IMU660RB INT1中断处理
#if defined(GPIO_IMU660RB_PIN_IMU660RB_INT1_PORT) && defined(GPIO_IMU660RB_PIN_IMU660RB_INT1_PIN)
    uint32_t imu_status = DL_GPIO_getEnabledInterruptStatus(GPIO_IMU660RB_PIN_IMU660RB_INT1_PORT, GPIO_IMU660RB_PIN_IMU660RB_INT1_PIN);
    if (imu_status & GPIO_IMU660RB_PIN_IMU660RB_INT1_PIN) {
        Read_IMU660RB();
        DL_GPIO_clearInterruptStatus(GPIO_IMU660RB_PIN_IMU660RB_INT1_PORT, GPIO_IMU660RB_PIN_IMU660RB_INT1_PIN);
    }
#endif
}


//Timer interrupt function(20ms)
void PIT_FOR_CONTROL_INST_IRQHandler(void)
{
    // 计算本周期内编码器脉冲数增量
    int32_t left_delta = Left_Encoder_Count - last_Left_Encoder_Count;
    int32_t right_delta = Right_Encoder_Count - last_Right_Encoder_Count;
    // 更新上次计数
    last_Left_Encoder_Count = Left_Encoder_Count;
    last_Right_Encoder_Count = Right_Encoder_Count;
    // 累加距离（米）
    Left_Motor_Distance += DISTANCE(left_delta);
    Right_Motor_Distance += DISTANCE(right_delta);
    // 本周期走过的距离（米）
    float left_distance = DISTANCE(left_delta);
    float right_distance = DISTANCE(right_delta);
    // 计算物理速度（厘米/秒），周期为20ms=0.02s
    Left_Motor_Velocity = left_distance / 0.02f * 100;
    Right_Motor_Velocity = right_distance / 0.02f * 100;
    // 更新速度（脉冲数/周期）
    Left_Motor_Speed = left_delta;
    Right_Motor_Speed = right_delta;
    
    // PID速度环控制（左轮和右轮）
    float target_speed = 1.0f; // 目标速度（厘米/秒），可根据实际需求调整 
    Motor_Set_L_Speed(Speed_Control(target_speed, Left_Motor_Velocity));
    Motor_Set_R_Speed(Speed_Control(target_speed, Right_Motor_Velocity));

    // 通过UART_0_INST发送左右电机速度
    char uart_buf[64];
    int len = sprintf(uart_buf, "L:%.3f R:%.3f\r\n", Left_Motor_Velocity, Right_Motor_Velocity);
    for (int i = 0; i < len; i++) {
        DL_UART_Main_transmitData(UART_0_INST, uart_buf[i]);
    }
}

