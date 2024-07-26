/*
 * TMC6200.c
 *
 *  Created on: May 11, 2024
 *      Author: xange
 */

#include "TMC6200.h"


uint8_t TMC6200_Init(TMC6200 *driver,SPI_HandleTypeDef * SPI_Handle, GPIO_TypeDef *csPinBank, uint16_t csPin){


	driver->SPI_Handle = SPI_Handle;
	driver->csPinBank = csPinBank;
	driver->csPin = csPin;


	uint8_t status=0;
	uint32_t data=0;


	/* TMC6200_REG_GCONF Global configuration flags
	 * Bit 0: disable driver
	 * Bit 1: interface mode. 0 individuals signal H + L. 1 H input control, L enable (reset default =1)
	 * Bit 2: fault direct. 0 fault output when at least one mosfet. 1 for each action of fault
	 * Bit 3: unused
	 * Bit 5-4 amplification of current amplifiers. 0 Current ampli *5. 1 current ampli *10. 2 current ampli *10. 3 current ampli *20
	 * Bit 6: amplifier. 0 amplifiers on. 1 off.
	 * Bit 7 test mode always 0.
	 * Bit 31-8 unused.
	 *
	 */
	//HAL_GPIO_WritePin(driver->csPinBank, driver->csPin, GPIO_PIN_RESET);
	//HAL_Delay(1);
	//HAL_GPIO_WritePin(driver->csPinBank, driver->csPin, GPIO_PIN_SET);

	status += TMC6200_WriteRegister(driver, TMC6200_REG_GCONF, 0x50);
	HAL_Delay(100);
	status += TMC6200_ReadRegister(driver, TMC6200_REG_GCONF, &data);
	HAL_Delay(100);
	status += TMC6200_ReadRegister(driver, TMC6200_REG_GSTAT, &data);
	HAL_Delay(100);
	status += TMC6200_ReadRegister(driver, TMC6200_REG_IOIN_OUTPUT, &data);
	HAL_Delay(100);
	status += TMC6200_ReadRegister(driver, TMC6200_REG_IOIN_OUTPUT, &data);
	HAL_Delay(100);
	status += TMC6200_ReadRegister(driver, TMC6200_REG_DRV_CONF, &data);
	HAL_Delay(100);
	return status;


}

uint8_t TMC6200_ReadRegister(TMC6200 *driver, uint8_t regAddr, uint32_t *data){

	uint8_t status =0;
	uint8_t txBuff[5] = {regAddr, 0x00, 0x00, 0x00, 0x00};
	uint8_t rxBuff[5];

	HAL_GPIO_WritePin(driver->csPinBank, driver->csPin, GPIO_PIN_RESET);
	HAL_Delay(10);
	status = (HAL_SPI_TransmitReceive(driver->SPI_Handle, txBuff, rxBuff, 5, HAL_MAX_DELAY) == HAL_OK);
	//HAL_Delay(1);
	HAL_GPIO_WritePin(driver->csPinBank, driver->csPin, GPIO_PIN_SET);

	*data = (uint32_t) (rxBuff[1]<<24 | rxBuff[2]<<16 | rxBuff[3]<<8 | rxBuff[4]);

	return status;
}


uint8_t TMC6200_WriteRegister(TMC6200 *driver, uint8_t regAddr, uint32_t data){

	uint8_t status =0;
	uint8_t rxBuff[5];

	uint8_t txBuff[5] ={regAddr | 0x80, (data >> 24) & 0xFF, (data >> 16) & 0xFF, (data >> 8) & 0xFF, data & 0xFF};

	HAL_GPIO_WritePin(driver->csPinBank, driver->csPin, GPIO_PIN_RESET);
	HAL_Delay(10);
	status = (HAL_SPI_TransmitReceive(driver->SPI_Handle, txBuff, rxBuff, 5, HAL_MAX_DELAY) == HAL_OK);
	HAL_GPIO_WritePin(driver->csPinBank, driver->csPin, GPIO_PIN_SET);


	return status;
}





