/*
 * TMC6200.h
 *
 *  Created on: May 11, 2024
 *      Author: xange
 */

#ifndef INC_TMC6200_H_
#define INC_TMC6200_H_

#include "stm32g4xx_hal.h"


#define	TMC6200_REG_GCONF 			0x00
#define TMC6200_REG_GSTAT 			0x01
#define TMC6200_REG_IOIN_OUTPUT 	0x04
#define TMC6200_REG_OTP_PROG		0x06
#define TMC6200_REG_OTP_READ 		0x07
#define TMC6200_REG_FACTORY_CONF	0x08
#define TMC6200_REG_SHORT_CONF 		0x09
#define TMC6200_REG_DRV_CONF 		0x0A


typedef struct {

	//SPI Handle
	SPI_HandleTypeDef *SPI_Handle;
	GPIO_TypeDef *csPinBank;
	uint16_t csPin;




}TMC6200;



uint8_t TMC6200_Init(TMC6200 *driver,SPI_HandleTypeDef *SPI_Handle, GPIO_TypeDef *csPinBank, uint16_t csPin);

uint8_t TMC6200_ReadRegister(TMC6200 *driver, uint8_t regAddr, uint32_t *data);
uint8_t TMC6200_WriteRegister(TMC6200 *driver, uint8_t regAddr, uint32_t data);


#endif /* INC_TMC6200_H_ */
