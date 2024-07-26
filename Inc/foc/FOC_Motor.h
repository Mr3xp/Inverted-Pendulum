/*
 * FOC_Motor.h
 *
 *  Created on: Jun 17, 2024
 *      Author: xange
 */

#ifndef INC_FOC_FOC_MOTOR_H_
#define INC_FOC_FOC_MOTOR_H_

#include "stm32g4xx_hal.h"
#include "foc/current_sense.h"
#include "foc/position_sensor.h"
#include "foc/driver.h"
#include "foc/common/PID.h"
#include "foc/common/lp_filt.h"
#include "foc/common/misc.h"

enum Controller {

	torque				= 0x00,
	velocity 			= 0x01,
	angle 				= 0x02,
	velocity_open_loop	= 0x03,
	angle_open_loop 	= 0x04

};

enum torqu_controller {
	voltage 	=0x00,
	dc_current 	=0x01,
	foc_current =0x02

};

enum Direction {
	cw  =1,
	ccw =-1,
	unknow = 0
};



typedef struct{

	TIM_HandleTypeDef *TIM_Handle;
	uint8_t Controller_type;
	uint8_t pole_pairs;
	float	phase_reistance;
	float	KV;

	POS_SENSOR *pos;
	DRIVER *driver;
	CURRENT_SENSE *current;

	float target;
	float feed_forward_velocity;
	float shaft_agnle;
	float electrical_angle;
	float shaft_velocity;
	float current_sp;
	float shaft_angle_sp;
	float shaft_velocity_sp;
	DQCurrents_s current_dq;
	float voltage_emf;
	float Ualpha,Ubeta,Ua,Ub,Uc;

	float voltage_limit,current_limit,velocity_limit;

	PID_S *PID_Current_d;
	PID_S *PID_Velocity;
	PID_S *PID_Angle;

	lowpass_filt *LPF_Current_q;
	lowpass_filt *LPF_Current_d;
	lowpass_filt *LPF_Velocity;
	lowpass_filt *LPF_Angle;

	int8_t sensor_dir;

	long open_loop_us;




}FOC_MOTOR;



uint8_t FOCMotor_Init(FOC_MOTOR *motor, POS_SENSOR *pos, DRIVER *driver, CURRENT_SENSE *current,uint8_t Controller,TIM_HandleTypeDef *TIM_Handle,lowpass_filt *LPF_Velocity);

uint8_t FOCMotor_Enable(FOC_MOTOR *motor);
uint8_t FOCMotor_Disabel(FOC_MOTOR *motor);

uint8_t FOCMotor_AlignCurrentSense(FOC_MOTOR *motor);
uint8_t FOCMotor_AlignSensor(FOC_MOTOR *motor);
uint8_t FOCMotor_AbsoluteZeroAlign(FOC_MOTOR *motor);


float FOCMotor_ShaftAngle(FOC_MOTOR *motor);
float FOCMotor_ShaftVelocity(FOC_MOTOR *motor);
float FOCMotor_ElectricalAngle(FOC_MOTOR *motor);


uint8_t FOCMotor_FOC_Init(FOC_MOTOR *motor);
void FOCMotor_LoopFOC(FOC_MOTOR *motor);
void FOCMotor_Move(FOC_MOTOR *motor, float target_new);


void FOCMotor_SetPhaseVoltage(FOC_MOTOR *motor, float Uq, float Ud, float angle_el);
float FOCMotor_VelocityOpenLoop(FOC_MOTOR *motor, float target_vel);
float FOCMotor_AngleOpenLoop(FOC_MOTOR *motor, float target_angle);

















#endif /* INC_FOC_FOC_MOTOR_H_ */
