/*
 * current_sense.c
 *
 *  Created on: Jun 14, 2024
 *      Author: xange
 */

#include "foc/current_sense.h"


uint8_t CURRENT_SENSE_Init(CURRENT_SENSE *current,DRIVER *driver){

	uint8_t status=1;
	 CURRENT_SENSE_CALIBRATE_OFFSET(current);
	 current->driver = driver;
	 current->phase_gain = 1.0f/0.05/10;

	 //CURRENT_SENSE_ALIGN(current);

	 return status;
}

uint8_t CURRENT_SENSE_ALIGN(CURRENT_SENSE *current){


	uint8_t status=0;
	DRIVER_Enable(current->driver);
	DRIVER_SET_PWM(current->driver, 3.0, 0.0, 0.0);
	HAL_Delay(1000);
	PhaseCurrent_s c = GetPhaseCurrents(current);
	for (uint8_t i=0;i<100;i++){
		PhaseCurrent_s c1 = GetPhaseCurrents(current);
		c.a = c.a * 0.6f + c1.a * 0.4f;
		c.b = c.b * 0.6f + c1.b * 0.4f;
		c.c = c.c * 0.6f + c1.c * 0.4f;
		HAL_Delay(3);
	}
	DRIVER_SET_PWM(current->driver, 0, 0, 0);
	float ab_ratio = c.b ? fabs(c.a / c.b) : 0;
	float ac_ratio = c.c ? fabs(c.a / c.c) : 0;

	if(ab_ratio >1.5f){
		current->phase_gain *= _sign(c.a);
	}
	else if(ac_ratio>1.5f){
		current->phase_gain *= _sign(c.b);
	}
	//else if()

	//DRIVER_Disable(current->driver);


	return status;
}


PhaseCurrent_s GetPhaseCurrents(CURRENT_SENSE *current){

	PhaseCurrent_s phase_current;
	phase_current.a=((float) (InjADC_Reading[0]*3.3/4095.0f)-current->offset_ia)*current->phase_gain;
	phase_current.b=((float) (InjADC_Reading[1]*3.3/4095.0f)-current->offset_ib)*current->phase_gain;
	phase_current.c=((float) (InjADC_Reading[2]*3.3/4095.0f)-current->offset_ic)*current->phase_gain;

	return phase_current;
}

ABCurrent_s GetABCurrents(PhaseCurrent_s current){

	float i_a,i_b;
	ABCurrent_s ABCurrent;

	float mid = (1.f/3) * (current.a + current.b + current.c);
	float a = current.a - mid;
	float b = current.b - mid;
	i_a = a;
	i_b = _1_SQRT3 * a + _2_SQRT3 * b;

	ABCurrent.a = i_a;
	ABCurrent.b = i_b;

	return ABCurrent;
}

DQCurrents_s GetDQCurrents(ABCurrent_s current, float angle_el){

	//Calculate park transform
	float ct;
	float st;
	DQCurrents_s DQCurrent;

	st = sin(angle_el);
	ct = cos(angle_el);

	DQCurrent.d = current.a * ct + current.b * st;
	DQCurrent.q = current.b * ct - current.a * st;

	return DQCurrent;
}

DQCurrents_s GetFOCCurrents(CURRENT_SENSE *current,float angle_el){

	//Read phase currents
	PhaseCurrent_s phase_current = GetPhaseCurrents(current);
	//Calculate clarke transform
	ABCurrent_s ABCurrent = GetABCurrents(phase_current);
	//Calcualte Parke transform
	DQCurrents_s DQCurrent = GetDQCurrents(ABCurrent, angle_el);

	return DQCurrent;

}

void CURRENT_SENSE_CALIBRATE_OFFSET(CURRENT_SENSE *current){

	const uint16_t calibration_value = 1000;
	current->offset_ia = 0;
	current->offset_ib = 0;
	current->offset_ic = 0;

	for(uint16_t i;i<calibration_value;i++){
		current->offset_ia +=(float) (InjADC_Reading[0]*3.3/4095.0f);
		current->offset_ib +=(float) (InjADC_Reading[1]*3.3/4095.0f);
		current->offset_ic +=(float) (InjADC_Reading[2]*3.3/4095.0f);

		HAL_Delay(1);
	}

	current->offset_ia = current->offset_ia / calibration_value;
	current->offset_ib = current->offset_ib / calibration_value;
	current->offset_ic = current->offset_ic / calibration_value;


}



