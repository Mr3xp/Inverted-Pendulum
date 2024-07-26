/*
 * current_sense.h
 *
 *  Created on: Jun 14, 2024
 *      Author: xange
 */

#ifndef INC_FOC_CURRENT_SENSE_H_
#define INC_FOC_CURRENT_SENSE_H_

#include "stm32g4xx_hal.h"
#include "foc/common/misc.h"
#include "foc/driver.h"


extern volatile uint16_t InjADC_Reading[3];

typedef struct{

	float shunt_resistor;
	float phase_gain;

	float offset_ia;
	float offset_ib;
	float offset_ic;

	uint16_t phase_current[3];

	DRIVER *driver;



}CURRENT_SENSE;


uint8_t CURRENT_SENSE_Init(CURRENT_SENSE *current,DRIVER *driver);

uint8_t CURRENT_SENSE_ALIGN(CURRENT_SENSE *current);


PhaseCurrent_s GetPhaseCurrents(CURRENT_SENSE *current);

ABCurrent_s GetABCurrents(PhaseCurrent_s current);

DQCurrents_s GetDQCurrents(ABCurrent_s current, float angle_el);

DQCurrents_s GetFOCCurrents(CURRENT_SENSE *current, float angle_el);

void CURRENT_SENSE_CALIBRATE_OFFSET(CURRENT_SENSE *current);


#endif /* INC_FOC_CURRENT_SENSE_H_ */
