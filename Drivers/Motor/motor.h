#ifndef MOTOR_H
#define MOTOR_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

// Function declarations
void Motor_Init(void);
void Motor_On(void);
void Motor_Set_L_Speed(int8_t speed);
void Motor_Set_R_Speed(int8_t speed);
void Motor_Test(void);

#endif // MOTOR_H