/*
 * PID.c
 *
 *  Created on: May 15, 2024
 *      Author: xange
 */


#include "foc/common/PID.h"

void PID_Init(PID_S *pid,TIM_HandleTypeDef *TIM_Handle){

	pid->TIM_Handle = TIM_Handle;

	pid->error_prev = 0;
	pid->out = 0;
	pid->measurment_prev = 0;
	pid->differentiator = 0;
	pid->integral = 0;

	pid->t_prev = __HAL_TIM_GET_COUNTER(pid->TIM_Handle);


}

float PID(PID_S *pid, float setpoint,float measurment){

	uint32_t t_now = __HAL_TIM_GET_COUNTER(pid->TIM_Handle);

	float ts = (t_now - pid->t_prev)*1e-7;

	if(ts <= 0 || ts > 0.5f) ts = 1e-3f;

	//Error signal
	float error = setpoint - measurment;

	//Proportional
	float proportional = pid->kp * error;

	//Integral
	pid->integral = pid->integral + 0.5f * pid->ki * ts * (error + pid->error_prev);

	//Anti-wind-up integrator clamping
	pid->integral = _constrain(pid->integral,-pid->limit,pid->limit);


	//Derivative
	pid->differentiator = -(2.0f * pid->kd * (measurment - pid->measurment_prev) + (2.0f * pid->d_lp - ts) * pid->differentiator) / (2.0f * pid->d_lp + ts);

	//Compute ouput
	pid->out = proportional + pid->integral + pid->differentiator;

	//Apply limit
	pid->out = _constrain(pid->out,-pid->limit,pid->limit);

	pid->error_prev = error;
	pid->measurment_prev = measurment;
	pid->t_prev = t_now;

	return pid->out;


}





