/*
 * driver.c
 *
 *  Created on: Jun 14, 2024
 *      Author: xange
 */

#include "foc/driver.h"



uint8_t DRIVER_Init(DRIVER *driver,TIM_HandleTypeDef *htim,GPIO_TypeDef *Enable_PinBank,uint16_t Enable_Pin,float PWM_RANGE){

	uint8_t status = 0;
	driver->htim = htim;
	driver->Enable_PinBank = Enable_PinBank;
	driver->Enable_Pin = Enable_Pin;
	driver->PWM_RANGE = PWM_RANGE;
	driver->voltage_powe_supply = 12;
	driver->voltage_limit = 12;


	//DRIVER_SET_PWM(driver,0,0,0);
	//Channel 1
	HAL_TIM_PWM_Start(driver->htim, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(driver->htim, TIM_CHANNEL_1);
	//Channel 2
	HAL_TIM_PWM_Start(driver->htim, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(driver->htim, TIM_CHANNEL_2);
	//Channel 3
	HAL_TIM_PWM_Start(driver->htim, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(driver->htim, TIM_CHANNEL_3);



	DRIVER_Enable(driver);


	return status;

}


void DRIVER_Enable(DRIVER *driver){

	HAL_GPIO_WritePin(driver->Enable_PinBank, driver->Enable_Pin, GPIO_PIN_SET);
	DRIVER_SET_PWM(driver, 0, 0, 0);


}

void DRIVER_Disable(DRIVER *driver){

	DRIVER_SET_PWM(driver, 0, 0, 0);
	HAL_GPIO_WritePin(driver->Enable_PinBank, driver->Enable_Pin, GPIO_PIN_RESET);

}


void DRIVER_SET_PWM(DRIVER *driver, float Ua, float Ub, float Uc){

	Ua = _constrain(Ua, 0, driver->voltage_limit);
	Ub = _constrain(Ub, 0, driver->voltage_limit);
	Uc = _constrain(Uc, 0, driver->voltage_limit);

	driver->dc_a = _constrain(Ua/driver->voltage_powe_supply,0.0f,1.0f);
	driver->dc_b = _constrain(Ub/driver->voltage_powe_supply,0.0f,1.0f);
	driver->dc_c = _constrain(Uc/driver->voltage_powe_supply,0.0f,1.0f);

	DRIVER_SET_DUTY(driver, TIM_CHANNEL_3, driver->dc_a*driver->PWM_RANGE);
	DRIVER_SET_DUTY(driver, TIM_CHANNEL_2, driver->dc_b*driver->PWM_RANGE);
	DRIVER_SET_DUTY(driver, TIM_CHANNEL_1, driver->dc_c*driver->PWM_RANGE);

}


void DRIVER_SET_DUTY(DRIVER *driver, uint32_t channel, uint32_t duty){

	__HAL_TIM_SET_COMPARE(driver->htim, channel, duty);
	//HAL_TIM_GenerateEvent(driver->htim, TIM_EVENTSOURCE_UPDATE);

}
