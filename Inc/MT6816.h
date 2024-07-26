/*
 * mt6816.h
 *
 *  Created on: May 11, 2024
 *      Author: xange
 */

#ifndef INC_MT6816_H_
#define INC_MT6816_H_

#include "stm32g4xx_hal.h"

#define _2PI 6.28318530718f
#define RESOLUTION 16384.0f

#define MT6816_REG_ANGLE_HB 0x03
#define MT6816_REG_ANGLE_LB 0x04
#define MT6816_REG_ANGLE_SPEED 0x05
#define ANGLE_TO_RAD 0.00153588974f


typedef struct {

	//SPI Handle
	SPI_HandleTypeDef *SPI_Handle;
	GPIO_TypeDef *csPinBank;
	uint16_t	csPin;

	//PWM
	TIM_HandleTypeDef *TIM_Handle;


	//Error checks
	uint8_t Mag_Warn;
	uint8_t Pariry_check;

	float angle;

}MT6816;


	/*
	 * Initialization
	 */
	uint8_t MT6816_Init(MT6816 *magSensor, SPI_HandleTypeDef *SPI_Handle, GPIO_TypeDef *csPinBank, uint16_t csPin);
	uint8_t MT6816_Init_PWM(MT6816 *magSensor, TIM_HandleTypeDef *TIM_Handle);

	/*
	 * Simple read/write of register
	 */
	uint8_t MT6816_ReadRegister(MT6816 *magSensor, uint8_t regAddr, uint8_t *data);
	uint8_t MT6816_WriteRegister(MT6816 *magSensor, uint8_t regAddr, uint8_t data);

	/*
	 * Read angle registers
	 */
	uint8_t MT6816_ReadAngle(MT6816 *magSensor);
	float MT6816_ReadAngle_PWM(MT6816 *magSensor);



#endif /* INC_MT6816_H_ */
