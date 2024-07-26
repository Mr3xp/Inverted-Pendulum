/*
 * cordic_trig.c
 *
 *  Created on: May 4, 2024
 *      Author: xange
 */

#include "cordic_trig.h"
#include "foc/common/misc.h"
#include "arm_math.h"


CORDIC_HandleTypeDef thisCordic;

void CORDIC_Conif(CORDIC_HandleTypeDef *hcordig){
		__HAL_RCC_CORDIC_CLK_ENABLE();
	     CORDIC_ConfigTypeDef sConfig;
	     hcordig->Instance = CORDIC;
	     if (HAL_CORDIC_Init(hcordig) != HAL_OK) {
	         Error_Handler();

	     }

		sConfig.Function = CORDIC_FUNCTION_COSINE;
	     sConfig.Precision = CORDIC_PRECISION_6CYCLES;
	     sConfig.Scale = CORDIC_SCALE_0;
	     sConfig.NbWrite = CORDIC_NBWRITE_1;
	     sConfig.NbRead = CORDIC_NBREAD_2;
	     sConfig.InSize = CORDIC_INSIZE_32BITS;
	     sConfig.OutSize = CORDIC_OUTSIZE_32BITS;
	     if (HAL_CORDIC_Configure(hcordig, &sConfig) != HAL_OK) {
	         /* Channel Configuration Error */
	         Error_Handler();

	     }

	/*
	     LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CORDIC);
	     LL_CORDIC_Config(CORDIC, LL_CORDIC_FUNCTION_COSINE
					,LL_CORDIC_PRECISION_6CYCLES
					,LL_CORDIC_SCALE_0
					,LL_CORDIC_NBWRITE_1
					,LL_CORDIC_NBREAD_2
					,LL_CORDIC_INSIZE_32BITS
					,LL_CORDIC_OUTSIZE_32BITS);

	return true;
*/
}


float sin_cordic(float a){

	a=fmod(a, _PI_2);
	if(a>_PI) a -= _2PI;
	if(a<-_PI) a+= _2PI;

	CORDIC->WDATA = (q31_t) ((a/_PI) * 0x80000000);

	q31_t out_cos = (int32_t) CORDIC->RDATA; //read cosine result
	q31_t out_sin = (int32_t) CORDIC->RDATA; //read sine result

	return (float) out_sin / (float) 0x80000000;
}

float _cos(float a){

	a=fmod(a, _PI_2);
	if(a>_PI) a -= _2PI;
	if(a<-_PI) a+= _2PI;

	CORDIC->WDATA = (q31_t) ((a/_PI) * 0x80000000);

	q31_t out_cos = (int32_t) CORDIC->RDATA; //read cosine result
	q31_t out_sin = (int32_t) CORDIC->RDATA; //read sine result

	return (float) out_cos / (float) 0x80000000;
}

void _sincos(float a, float* sine, float* cos){

	a=fmod(a, _PI_2);
	if(a>_PI) a -= _2PI;
	if(a<-_PI) a+= _2PI;

	CORDIC->WDATA = (q31_t) ((a/_PI) * 0x80000000);

	q31_t out_cos = (int32_t) CORDIC->RDATA; //read cosine result
	q31_t out_sin = (int32_t) CORDIC->RDATA; //read sine result

	*sine = (float) out_sin / (float) 0x80000000;
	*cos = (float) out_cos / (float) 0x80000000;

}


