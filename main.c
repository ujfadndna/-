/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in thes
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "main.h"
#include "Drivers/PID/pid.h"

uint8_t oled_buffer[32];

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();

    // MPU6050_Init();
    // Ultrasonic_Init();
    // BNO08X_Init();
    // WIT_Init();
    // VL53L0X_Init();
    // LSM6DSV16X_Init();
    IMU660RB_Init();
    // OLED_Init();
    

    /* Don't remove this! */
    Interrupt_Init();

    Motor_Set_L_Speed(5);
    
    while (1) 
    {

    }
}

// 根据编码器脉冲数计算轮子行驶的距离（单位：米）
float DISTANCE(int pulse_count) {
    const float ENCODER_PPR = 13.0f;         // 编码器每圈脉冲数（MG310）
    const float GEAR_RATIO = 20.0f;          // 减速比（MG310）
    const float WHEEL_DIAMETER = 0.048f;     // 轮子直径（单位：米，48mm）
    const float WHEEL_CIRCUMFERENCE = 3.1416f * WHEEL_DIAMETER; // 轮子周长
    const float PULSE_PER_TURN = ENCODER_PPR * GEAR_RATIO;      // 轮子转一圈对应的总脉冲数
    const float DIST_PER_PULSE = WHEEL_CIRCUMFERENCE / PULSE_PER_TURN; // 每个脉冲对应的距离
    return pulse_count * DIST_PER_PULSE;     // 总距离 = 脉冲数 × 单脉冲距离
}