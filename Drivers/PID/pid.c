#include "pid.h"
#include <stdint.h>

//1.pid.cжһ��Pid_TypeDef���͵ı���
//2.��main.c�г�ʼ��PID���൱�ڴ���һ����
//3.ʹ��PID_SetTarget������PIDĿ��ֵ
//4.��PID���㺯������ִ�к���
//5.��main��pid��ʼ�������е���PIDֵ

Pid_TypeDef Pid_ASpeed, Pid_BSpeed, Pid_Angle;
Pid_TypeDef Speed_PID;

/**
  * @   MOTOR PIDʼ��,�൱�ڴ���һ��PID��
  * @��  �� PID�ṹ�壬PID��������
  * @����ֵ ��
  */
void PID_Init(Pid_TypeDef *PID, float Kp, float Ki, float Kd)
{
	PID->target_val = 0.0; 
	PID->actual_val = 0.00;
	PID->err = 0.0;
	PID->err_last = 0.0; 
	PID->err_sum = 0.0;
	PID->Kp = Kp;
	PID->Ki = Ki;
	PID->Kd = Kd;
	PID->ControllerOutput = 0;
}


void PID_Init_All(void)
{
    PID_Init(&Speed_PID, 5.0f, 0.0f, 0.00f); // Kp, Ki, Kd �ɵ�
}

/**
  * @��  �� ����Ŀ��ֵ
  * @��  �� PID�ṹ�壬Ŀ��ֵ
  * @����ֵ ��
  */
void PID_SetTarget(Pid_TypeDef *PID, float target_val)
{
	PID->target_val = target_val;
}


/**
  * @��  �� P�������ƻ���
  * @��  �� PID�ṹ�壬ʵ��ֵ
  * @����ֵ P�������ʵ���?
  */
float P_realize(Pid_TypeDef *pid, float actual_val)
{
	pid->actual_val = actual_val;//����ʵ��ֵ
	pid->err = pid->target_val - pid->actual_val;//���??=Ŀ��ֵ - ʵ��ֵ
	//�������ƻ��� ���??=Kp*��ǰ���??
	pid->ControllerOutput = pid->Kp * pid->err;
	return pid->ControllerOutput;//����
}


/**
  * @��  �� PI���ֿ��ƻ���
  * @��  �� PID�ṹ�壬ʵ��ֵ
  * @����ֵ PI�������ʵ���?
  */
float PI_realize(Pid_TypeDef *pid, float actual_val)
{
	pid->actual_val = actual_val;
	pid->err = pid->target_val - pid->actual_val;
	pid->err_sum += pid->err;//����ۼ��?
	//ʹ��PI���� ���??=Kp*��ǰ���??+Ki*����ۼ��?
	pid->ControllerOutput = pid->Kp * pid->err + pid->Ki*pid->err_sum;
	
	return pid->ControllerOutput ;
	
}

/**
  * @��  �� PID���ֿ��ƻ���
  * @��  �� PID�ṹ�壬ʵ��ֵ, ����ֵ
  * @����ֵ PID�������ʵ���?
  */
float PID_realize(Pid_TypeDef *pid, float actual_val, float Deadband,  float limit)
{
	pid->actual_val = actual_val;
	pid->err = pid->target_val - pid->actual_val;
	/*����*/
	if((pid->err<Deadband ) && (pid->err>-Deadband))
	{pid->err = 0.0f;}
	
	pid->err_sum += pid->err;
	/*�����޷�*/
	 if (pid->err_sum >= limit) 
	{pid->err_sum =limit;}
      else if (pid->err_sum < -limit)  
    {pid->err_sum = -limit;}
	
	pid->ControllerOutput = pid->Kp * pid->err + pid->Ki*pid->err_sum + pid->Kd*(pid->err - pid->err_last);
	
	pid->err_last = pid->err;
	
	return pid->ControllerOutput;
}

int8_t Speed_Control(float target_speed, float actual_speed)
{
    PID_SetTarget(&Speed_PID, target_speed);
    float output = PID_realize(&Speed_PID, actual_speed, 1.0f, 100.0f);
    if (output > 100.0f) output = 100.0f;
    if (output < -100.0f) output = -100.0f;
    return (int8_t)output;
}
