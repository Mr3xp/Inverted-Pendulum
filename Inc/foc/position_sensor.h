/*
 * position_sensor.h
 *
 *  Created on: Jun 12, 2024
 *      Author: xange
 */

#ifndef INC_FOC_POSITION_SENSOR_H_
#define INC_FOC_POSITION_SENSOR_H_

#include "stm32g4xx_hal.h"
#include "MT6816.h"
#include "foc/common/misc.h"




typedef struct{

	TIM_HandleTypeDef *TIM_Handle;

	float velocity;
	float angle_prev;
	long angle_prev_ts;
	float vel_angle_prev;
	long vel_angle_prev_ts;
	int32_t full_rotations;
	int32_t vel_full_rotations;

	float min_elapsed_time;

	MT6816 *mag;

}POS_SENSOR;


void POS_SENSOR_Init(POS_SENSOR* pos,MT6816 *mag,TIM_HandleTypeDef *TIM_Handle);

void POS_UPDATE(POS_SENSOR* pos);
float POS_GET_VELOCITY(POS_SENSOR* pos);


float POS_GET_MECHANICHAL_ANGLE(POS_SENSOR* pos);
float POS_GET_ANGLE(POS_SENSOR* pos);
double POS_GET_PRECISE_ANGLE(POS_SENSOR* pos);
int32_t POS_GET_FULL_ROTATIONS(POS_SENSOR* pos);



#endif /* INC_FOC_POSITION_SENSOR_H_ */
