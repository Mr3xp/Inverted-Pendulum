/*
 * lp_filt.c0
 *
 *  Created on: May 15, 2024
 *      Author: xange
 */


#include "foc/common/lp_filt.h"


void LowPassFilter_Init(lowpass_filt *filt, float CutOffFreqHz, TIM_HandleTypeDef *TIM_Handle){

	filt->TIM_Handle = TIM_Handle;

	filt->rc = 1.0f / (6.28318530718f * CutOffFreqHz);
	filt->t_prev = __HAL_TIM_GET_COUNTER(filt->TIM_Handle);

	filt->Coeff[0] = filt->t_prev / (filt->t_prev + filt->rc);
	filt->Coeff[1] = 1-filt->Coeff[0];


	filt->Out[0] = 0.0f;
	filt->Out[1] = 0.0f;

}

float LowPassFilterUpdate(lowpass_filt *filt, float value){

	uint32_t t_now = __HAL_TIM_GET_COUNTER(filt->TIM_Handle);

	float dt = (t_now - filt->t_prev)*1e-7f;

	if( dt < 0.0f ) dt=1e-3f;


	filt->Coeff[0] = filt->rc / (filt->rc + dt);
	filt->Out[1] = filt->Out[0];

	filt->Out[0] = filt->Coeff[0] * value + filt->Coeff[1] * filt->Out[0];

	return filt->Out[0];
}


