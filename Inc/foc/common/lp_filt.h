/*
 * lp_filt.h
 *
 *  Created on: May 15, 2024
 *      Author: xange
 */

#ifndef INC_FOC_COMMON_LP_FILT_H_
#define INC_FOC_COMMON_LP_FILT_H_

#include "stm32g4xx_hal.h"


typedef struct{

	TIM_HandleTypeDef *TIM_Handle;

	float Out[2];
	float Coeff[2];
	float rc;

	uint32_t t_prev;
}lowpass_filt;


void LowPassFilter_Init(lowpass_filt *filt, float CutOffFreqHz,TIM_HandleTypeDef *TIM_Handle);

float LowPassFilterUpdate(lowpass_filt *filt, float value);


#endif /* INC_FOC_COMMON_LP_FILT_H_ */
