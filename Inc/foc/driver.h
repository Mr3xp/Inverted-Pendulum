/*
 * driver.h
 *
 *  Created on: Jun 14, 2024
 *      Author: xange
 */

#ifndef INC_FOC_DRIVER_H_
#define INC_FOC_DRIVER_H_

#include "stm32g4xx_hal.h"
#include "foc/common/misc.h"


typedef struct{

	TIM_HandleTypeDef *htim;
	GPIO_TypeDef *Enable_PinBank;
	uint16_t Enable_Pin;

	float voltage_limit;
	float voltage_powe_supply;

	float dc_a;
	float dc_b;
	float dc_c;

	float PWM_RANGE;





}DRIVER;



uint8_t DRIVER_Init(DRIVER *driver,TIM_HandleTypeDef *htim,GPIO_TypeDef *Enable_PinBank,uint16_t Enable_Pin,float PWM_RANGE);



void DRIVER_Enable(DRIVER *driver);
void DRIVER_Disable(DRIVER *driver);


void DRIVER_SET_PWM(DRIVER *driver, float Ua, float Ub, float Uc);
void DRIVER_SET_DUTY(DRIVER *driver, uint32_t channel, uint32_t duty);

#endif /* INC_FOC_DRIVER_H_ */
