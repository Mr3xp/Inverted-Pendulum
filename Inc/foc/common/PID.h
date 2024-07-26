/*
 * PID.h
 *
 *  Created on: May 15, 2024
 *      Author: xange
 */

#ifndef INC_FOC_COMMON_PID_H_
#define INC_FOC_COMMON_PID_H_

#include "stm32g4xx_hal.h"
#include "misc.h"

typedef struct{

	TIM_HandleTypeDef *TIM_Handle;

	//Controller gain
	float kp;
	float ki;
	float kd;
	//dirivative low pass time constant fc = 1 / (2*pi*d_lp))
	float d_lp;
	//out limit
	float limit;
	float error_prev;
	float integral;
	float out;
	float differentiator;
	float measurment_prev;

	uint32_t t_prev;

}PID_S;

void PID_Init(PID_S *pid,TIM_HandleTypeDef *TIM_Handle);
float PID(PID_S *pid, float setpoint,float measurment);
void PID_Reset(PID_S *pid);

#endif /* INC_FOC_COMMON_PID_H_ */
