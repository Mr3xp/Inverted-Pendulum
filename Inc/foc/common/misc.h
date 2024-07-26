/*
 * misc.h
 *
 *  Created on: May 10, 2024
 *      Author: xange
 */

#ifndef INC_FOC_MISC_H_
#define INC_FOC_MISC_H_

//#include "main.h"
#include "stm32g4xx_hal.h"
#include "math.h"
#include "cordic_trig.h"
#include "MT6816.h"


//==============Defines====================
#define _PI		3.14159265359f
#define _PI_2 	1.57079632679f
#define _PI_3	1.0471975512f
#define _PI_6 	0.52359877559f
#define _2PI 	6.28318530718f
#define _3PI_2 	4.71238898038f

#define _SQRT2 	1.41421356237f
#define _SQRT3 	1.73205080757f
#define _SQRT3_2 0.86602540378f
#define _1_SQRT3 0.57735026919f
#define _2_SQRT3 1.15470053838f

#define _RPM_TO_RADS 0.10471975512f

//===========Functions defines==============
#define _sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )
#define _constrain(a,min,max) ((a)<(min)?(min):((a)>(max)?(max):(a)))
#define _sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )






//===================Structs=================

typedef struct {
	float d;
	float q;
}DQCurrents_s;

typedef struct {
	float d;
	float q;
}DQVoltage_s;

typedef struct {
	float a;
	float b;
	float c;
}PhaseCurrent_s;

typedef struct ABCurrent_s{
	float a;
	float b;
}ABCurrent_s;




float _normalizeAngle(float angle);

float _electricalAngle(float shaft_angle, int pole_pairs);

float _atan2(float y, float x);

float _sqrtApprox(float number);

















#endif /* INC_FOC_MISC_H_ */



