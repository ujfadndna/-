#include "motor.h"
#include <stdlib.h>
#include "ti_msp_dl_config.h"

#define PWM_MAX_VALUE 3199

/******************************************************************
 * 函 数 名 称：Motor_Init
 * 函 数 说 明：电机初始化
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 备       注：启动PWM定时器并初始化GPIO引脚
******************************************************************/
void Motor_Init(void) 
{
    // 确保PWM定时器完全初始化
    SYSCFG_DL_PWM_Motor_init();
    
    // 启动PWM定时器
    DL_TimerG_startCounter(PWM_Motor_INST);
    
    // 初始化电机控制引脚为低电平
    DL_GPIO_clearPins(GPIO_Motor_AIN1_PORT, GPIO_Motor_AIN1_PIN);
    DL_GPIO_clearPins(GPIO_Motor_AIN2_PORT, GPIO_Motor_AIN2_PIN);
    DL_GPIO_clearPins(GPIO_Motor_BIN1_PORT, GPIO_Motor_BIN1_PIN);
    DL_GPIO_clearPins(GPIO_Motor_BIN2_PORT, GPIO_Motor_BIN2_PIN);
}

/******************************************************************
 * 函 数 名 称：Motor_On
 * 函 数 说 明：启用电机驱动器
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 备       注：TB6612驱动器不需要额外的使能引脚
******************************************************************/
void Motor_On(void)
{
    // TB6612驱动器通过AIN1/AIN2和BIN1/BIN2控制，不需要额外的STBY引脚
    // 如果你的硬件有STBY引脚，请在SysConfig中配置并取消注释下面的代码
    // DL_GPIO_setPins(STBY_PORT, STBY_PIN);
}

/******************************************************************
 * 函 数 名 称：Motor_Set_L_Speed
 * 函 数 说 明：设置左电机速度
 * 函 数 形 参：speed - 速度值（-100到100）
 * 函 数 返 回：无
 * 备       注：负值后退，正值前进，0停止
******************************************************************/
void Motor_Set_L_Speed(int8_t speed)
{
    uint32_t compareValue = 0;
    
    // 参数范围检查
    if(speed < -100) speed = -100;
    if(speed > 100) speed = 100;
    
    if(speed == 0)  // 停止
    {
        DL_GPIO_clearPins(GPIO_Motor_AIN1_PORT, GPIO_Motor_AIN1_PIN);
        DL_GPIO_clearPins(GPIO_Motor_AIN2_PORT, GPIO_Motor_AIN2_PIN);
        DL_TimerG_setCaptureCompareValue(PWM_Motor_INST, 0, GPIO_PWM_Motor_C0_IDX);
    }
    else
    {
        // 计算PWM比较值，按照你的成功代码思路
        compareValue = (uint32_t)(PWM_MAX_VALUE * abs(speed) / 100);
        
        // 设置PWM值
        DL_TimerG_setCaptureCompareValue(PWM_Motor_INST, compareValue, GPIO_PWM_Motor_C0_IDX);
        
        if(speed > 0)  // 前进 - 按照你的成功代码：AIN1=1, AIN2=0
        {
            DL_GPIO_setPins(GPIO_Motor_AIN1_PORT, GPIO_Motor_AIN1_PIN);
            DL_GPIO_clearPins(GPIO_Motor_AIN2_PORT, GPIO_Motor_AIN2_PIN);
        }
        else  // 后退 - AIN1=0, AIN2=1
        {
            DL_GPIO_clearPins(GPIO_Motor_AIN1_PORT, GPIO_Motor_AIN1_PIN);
            DL_GPIO_setPins(GPIO_Motor_AIN2_PORT, GPIO_Motor_AIN2_PIN);
        }
    }
}

/******************************************************************
 * 函 数 名 称：Motor_Set_R_Speed
 * 函 数 说 明：设置右电机速度
 * 函 数 形 参：speed - 速度值（-100到100）
 * 函 数 返 回：无
 * 备       注：负值后退，正值前进，0停止
******************************************************************/
void Motor_Set_R_Speed(int8_t speed)
{
    uint32_t compareValue = 0;
    
    // 参数范围检查
    if(speed < -100) speed = -100;
    if(speed > 100) speed = 100;
    
    if(speed == 0)  // 停止
    {
        DL_GPIO_clearPins(GPIO_Motor_BIN1_PORT, GPIO_Motor_BIN1_PIN);
        DL_GPIO_clearPins(GPIO_Motor_BIN2_PORT, GPIO_Motor_BIN2_PIN);
        DL_TimerG_setCaptureCompareValue(PWM_Motor_INST, 0, GPIO_PWM_Motor_C1_IDX);
    }
    else
    {
        // 计算PWM比较值，按照你的成功代码思路
        compareValue = (uint32_t)(PWM_MAX_VALUE * abs(speed) / 100);
        
        // 设置PWM值
        DL_TimerG_setCaptureCompareValue(PWM_Motor_INST, compareValue, GPIO_PWM_Motor_C1_IDX);
        
        if(speed > 0)  // 前进 - 按照你的成功代码：BIN1=1, BIN2=0
        {
            DL_GPIO_setPins(GPIO_Motor_BIN1_PORT, GPIO_Motor_BIN1_PIN);
            DL_GPIO_clearPins(GPIO_Motor_BIN2_PORT, GPIO_Motor_BIN2_PIN);
        }
        else  // 后退 - BIN1=0, BIN2=1
        {
            DL_GPIO_clearPins(GPIO_Motor_BIN1_PORT, GPIO_Motor_BIN1_PIN);
            DL_GPIO_setPins(GPIO_Motor_BIN2_PORT, GPIO_Motor_BIN2_PIN);
        }
    }
}

/******************************************************************
 * 函 数 名 称：Motor_Test
 * 函 数 说 明：电机测试函数（复制你的成功代码）
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 备       注：直接使用你验证过的成功代码
******************************************************************/
void Motor_Test(void)
{
    // 你的成功代码
    DL_TimerG_setCaptureCompareValue(PWM_Motor_INST, 500, GPIO_PWM_Motor_C0_IDX);
    DL_TimerG_setCaptureCompareValue(PWM_Motor_INST, 500, GPIO_PWM_Motor_C1_IDX);
    
    DL_GPIO_setPins(GPIO_Motor_AIN1_PORT, GPIO_Motor_AIN1_PIN);
    DL_GPIO_clearPins(GPIO_Motor_AIN2_PORT, GPIO_Motor_AIN2_PIN);
    DL_GPIO_setPins(GPIO_Motor_BIN1_PORT, GPIO_Motor_BIN1_PIN);
    DL_GPIO_clearPins(GPIO_Motor_BIN2_PORT, GPIO_Motor_BIN2_PIN);
}