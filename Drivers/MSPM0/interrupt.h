#ifndef _INTERRUPT_H_
#define _INTERRUPT_H_

#include <stdint.h>

extern uint8_t enable_group1_irq;

// 编码器全局变量声明
extern volatile int32_t Left_Encoder_Count;   // 左轮编码器计数
extern volatile int32_t Right_Encoder_Count;  // 右轮编码器计数

void Interrupt_Init(void);

#endif  /* #ifndef _INTERRUPT_H_ */