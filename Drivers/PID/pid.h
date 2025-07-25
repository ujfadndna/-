#ifndef __PID_H
#define __PID_H

#include <stdint.h>


typedef struct
{
	float target_val;
	float actual_val;
	float err;
	float err_last;
	float err_sum;
	float Kp;
	float Ki;
	float Kd;
	float ControllerOutput;
}Pid_TypeDef;

void PID_Init(Pid_TypeDef *PID, float Kp, float Ki, float Kd);
void PID_SetTarget(Pid_TypeDef *PID, float target_val);
float P_realize(Pid_TypeDef *pid, float actual_val);
float PI_realize(Pid_TypeDef *pid, float actual_val);
float PID_realize(Pid_TypeDef *pid, float actual_val, float Deadband, float limit);
void PID_Init_All(void);
int8_t Speed_Control(float target_speed, float actual_speed);

extern Pid_TypeDef Pid_ASpeed, Pid_BSpeed, Pid_Angle;


#endif 

