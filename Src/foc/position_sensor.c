/*
 * position_sensor.c
 *
 *  Created on: Jun 12, 2024
 *      Author: xange
 */

#include "foc/position_sensor.h"
#include "math.h"

void POS_SENSOR_Init(POS_SENSOR* pos,MT6816 *mag,TIM_HandleTypeDef *TIM_Handle){

	pos->TIM_Handle = TIM_Handle;
	pos->angle_prev =0.0f;
	pos->angle_prev_ts =0.;
	pos->vel_angle_prev =0.0f;
	pos->vel_angle_prev_ts =0;
	pos->full_rotations =0;
	pos->vel_full_rotations =0;
	pos->min_elapsed_time = 0.0001;
	pos->mag = mag;
	pos->angle_prev_ts=__HAL_TIM_GET_COUNTER(pos->TIM_Handle);
	MT6816_ReadAngle_PWM(mag);
	HAL_Delay(1);
	pos->vel_angle_prev = MT6816_ReadAngle_PWM(mag);
	pos->vel_angle_prev_ts = __HAL_TIM_GET_COUNTER(pos->TIM_Handle);
	HAL_Delay(1);
	MT6816_ReadAngle_PWM(mag);
	HAL_Delay(1);
	pos->angle_prev = MT6816_ReadAngle_PWM(mag);
	pos->angle_prev_ts=__HAL_TIM_GET_COUNTER(pos->TIM_Handle);
}

void POS_UPDATE(POS_SENSOR* pos){

	float val = MT6816_ReadAngle_PWM(pos->mag);
	if(val<0) return;
	pos->angle_prev_ts =__HAL_TIM_GET_COUNTER(pos->TIM_Handle);
	float d_agnle = val - pos->angle_prev;
	if(fabsf(d_agnle) > (0.8*_2PI)) pos->full_rotations+=(d_agnle >0) ? -1 : 1;
	pos->angle_prev = val;

}

float POS_GET_VELOCITY(POS_SENSOR* pos){

	float ts = (pos->angle_prev_ts - pos->vel_angle_prev_ts)*1e-7f;
	if(ts <0.0f){
		pos->vel_angle_prev = pos->angle_prev;
		pos->vel_full_rotations = pos->full_rotations;
		pos->vel_angle_prev_ts = pos->angle_prev_ts;
	}

	if(ts < pos->min_elapsed_time){
		return pos->velocity;
	}

	pos->velocity = ( (float)(pos->full_rotations - pos->vel_full_rotations)*_2PI + (pos->angle_prev - pos->vel_angle_prev)) /ts;
	pos->vel_angle_prev = pos->angle_prev;
	pos->vel_full_rotations = pos->full_rotations;
	pos->vel_angle_prev_ts = pos->angle_prev_ts;
	return pos->velocity*57.2957795;
}

float POS_GET_MECHANICHAL_ANGLE(POS_SENSOR* pos){
	return pos->angle_prev;
}
float POS_GET_ANGLE(POS_SENSOR* pos){
	return (float)pos->full_rotations*_2PI +pos->angle_prev;
}
double POS_GET_PRECISE_ANGLE(POS_SENSOR* pos){
	return (double) pos->full_rotations *(double)_2PI+(double)pos->angle_prev;
}
int32_t POS_GET_FULL_ROTATIONS(POS_SENSOR* pos){
	return 0;
}











