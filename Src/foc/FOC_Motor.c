/*
 * FOC_Motor.c
 *
 *  Created on: Jun 17, 2024
 *      Author: xange
 */

#include "foc/FOC_Motor.h"


uint8_t FOCMotor_Init(FOC_MOTOR *motor, POS_SENSOR *pos, DRIVER *driver, CURRENT_SENSE *current,uint8_t Controller,TIM_HandleTypeDef *TIM_Handle,lowpass_filt *LPF_Velocity){

	motor->TIM_Handle = TIM_Handle;
	uint8_t status =0;
	motor->pos = pos;
	motor->driver = driver;
	motor->current = current;

	motor->LPF_Velocity = LPF_Velocity;


	motor->velocity_limit = 2000.0f;
	motor->voltage_limit = 12.0f;
	motor->current_limit = 2.0f;

	motor->target = 0;
	motor->current_sp = 0;
	motor->current_dq.d = 0;
	motor->current_dq.q = 0;

	motor->voltage_emf = 0;
	motor->Ualpha =0;
	motor->Ubeta = 0;

	motor->pole_pairs = 6;
	motor->phase_reistance = 0.250;
	motor->KV = 2450.0f;

	motor->Controller_type = velocity_open_loop;
	motor->sensor_dir = cw;




	return status;

}

uint8_t FOCMotor_Enable(FOC_MOTOR *motor){

	DRIVER_Enable(motor->driver);
	//DRIVER_SET_PWM(motor->driver,0,0,0);

	return 1;
}
uint8_t FOCMotor_Disabel(FOC_MOTOR *motor){

	DRIVER_Disable(motor->driver);
	return 1;
}

uint8_t FOCMotor_AlignCurrentSense(FOC_MOTOR *motor);
uint8_t FOCMotor_AlignSensor(FOC_MOTOR *motor);
uint8_t FOCMotor_AbsoluteZeroAlign(FOC_MOTOR *motor);


float FOCMotor_ShaftAngle(FOC_MOTOR *motor){

	return	motor->sensor_dir * LowPassFilterUpdate(motor->LPF_Angle, POS_GET_ANGLE(motor->pos));
}
float FOCMotor_ShaftVelocity(FOC_MOTOR *motor){

	//return motor->sensor_dir * LowPassFilterUpdate(motor->LPF_Velocity,	POS_GET_VELOCITY(motor->pos));
	return motor->sensor_dir * 	POS_GET_VELOCITY(motor->pos);
}
float FOCMotor_ElectricalAngle(FOC_MOTOR *motor){

	return _normalizeAngle( (float) (motor->sensor_dir * motor->pole_pairs) * POS_GET_MECHANICHAL_ANGLE(motor->pos));
}


uint8_t FOCMotor_FOC_Init(FOC_MOTOR *motor){




}
void FOCMotor_LoopFOC(FOC_MOTOR *motor){

	POS_UPDATE(motor->pos);




}
void FOCMotor_Move(FOC_MOTOR *motor, float target_new){


	motor->shaft_velocity= FOCMotor_ShaftVelocity(motor);

	motor->target = target_new;
	motor->voltage_emf = motor->shaft_velocity /(motor->KV * _SQRT3)/_RPM_TO_RADS;

	motor->shaft_velocity_sp = motor->target;
	FOCMotor_VelocityOpenLoop(motor, motor->shaft_velocity_sp);

}


void FOCMotor_SetPhaseVoltage(FOC_MOTOR *motor, float Uq, float Ud, float angle_el){

	float center = motor->voltage_limit/2;
	float sa = sin(angle_el);
	float ca = cos(angle_el);

	motor->Ualpha = ca * Ud - sa * Uq;
	motor->Ubeta = sa *Ud + ca * Uq;

	motor->Ua = motor->Ualpha;
	motor->Ub = -0.5 * motor->Ualpha + _SQRT3_2 * motor->Ubeta;
	motor->Uc = -0.5 * motor->Ualpha - _SQRT3_2 * motor->Ubeta;

	float umin = fmin(motor->Ua , fmin(motor->Ub,motor->Uc));
	float umax = fmax(motor->Ua , fmax(motor->Ub,motor->Uc));

	center -= (umax+umin) /2;

	motor->Ua += center;
	motor->Ub += center;
	motor->Uc += center;

	DRIVER_SET_PWM(motor->driver, motor->Ua, motor->Ub, motor->Uc);


}


float FOCMotor_VelocityOpenLoop(FOC_MOTOR *motor, float target_vel){

	long now_us = __HAL_TIM_GET_COUNTER(motor->TIM_Handle);

	float ts = ( now_us - motor->open_loop_us) * 1e-7f;

	if(ts <=0 || ts >0.5f) ts = 1e-3f;

	motor->shaft_agnle = _normalizeAngle(motor->shaft_agnle + target_vel * ts);

	motor->shaft_velocity = target_vel;

	float Uq = _constrain(motor->current_limit * motor->phase_reistance +fabs(motor->voltage_emf),-motor->voltage_limit,motor->voltage_limit);

	motor->current_dq.q = (Uq - fabs(motor->voltage_emf))/motor->phase_reistance;

	FOCMotor_SetPhaseVoltage(motor, Uq, 0, _electricalAngle(motor->shaft_agnle, motor->pole_pairs));

	motor->open_loop_us = __HAL_TIM_GET_COUNTER(motor->TIM_Handle);

	return Uq;




}
float FOCMotor_AngleOpenLoop(FOC_MOTOR *motor, float target_angle);
