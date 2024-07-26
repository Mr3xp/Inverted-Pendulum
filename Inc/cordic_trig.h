/*
 * cordic_trig.h
 *
 *  Created on: May 4, 2024
 *      Author: xange
 */

#ifndef INC_CORDIC_TRIG_H_
#define INC_CORDIC_TRIG_H_

#include "stm32g4xx_hal.h"

void CORDIC_Conif(CORDIC_HandleTypeDef *hcordig);


float sin_cordic(float a);

#endif /* INC_CORDIC_TRIG_H_ */
