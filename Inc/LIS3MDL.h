/*
 * LIS3MDL.h
 *
 *  Created on: May 13, 2024
 *      Author: xange
 */

#ifndef INC_LIS3MDL_H_
#define INC_LIS3MDL_H_

#include "stm32g4xx_hal.h"

#define LIS3MDL_REG_CHIP_ID 		0x0f
#define LIS3MDL_REG_CHIP_VALUE		0x3D
#define LIS3MDL_REG_CTRL_1			0x20
#define LIS3MDL_REG_CTRL_2			0x21
#define LIS3MDL_REG_CTRL_3			0x22
#define LIS3MDL_REG_CTRL_4			0x23
#define LIS3MDL_REG_CTRL_5			0x24
#define LIS3MDL_REG_STATUS			0x27
#define LIS3MDL_REG_OUT_X_LSB		0x28
#define LIS3MDL_REG_OUT_X_MSB		0x29
#define LIS3MDL_REG_OUT_Y_LSB		0x2A
#define LIS3MDL_REG_OUT_Y_MSB		0x2B
#define LIS3MDL_REG_OUT_Z_LSB		0x2C
#define LIS3MDL_REG_OUT_Z_MSB		0x2D
#define LIS3MDL_REG_TEMP_OUT_LSB	0x2E
#define LIS3MDL_REG_TEMP_OUT_MSB	0x2F
#define LIS3MDL_REG_INT_CFG			0x30
#define LIS3MDL_REG_INT_SRC			0x31
#define LIS3MDL_REG_INT_THS_LSB		0x32
#define LIS3MDL_REG_INT_THS_MSB		0x33


typedef struct{

	//SPI Handle
	SPI_HandleTypeDef *SPI_Handle;
	GPIO_TypeDef *csBank;
	uint16_t csPin;

	//DMA
	uint8_t readingMagFlag;
	uint8_t magTxBuff[7];
	volatile uint8_t magRxBuff[7];

	//Conversion rate
	float magConversion;

	//Magnetometer data (x,y,z) in gauss.
	float mag_gauss[3];

	//Temperature data in Celcius
	float temp_C;

}LIS3DML;

	//Initialization
	uint8_t LIS3MDL_Init(LIS3DML *mag, SPI_HandleTypeDef *SPI_Handle, GPIO_TypeDef *csBank, uint16_t csPin);

	/*
	 * Simple read/write operation
	 */
	uint8_t LIS3MDL_ReadRegister(LIS3DML *mag, uint8_t regAddr, uint8_t *data);
	uint8_t LIS3MDL_WriteRegister(LIS3DML *mag, uint8_t regAddr, uint8_t data);

	/*
	 * Reading magnetometer data
	 */
	uint8_t LIS3MDL_ReadMagnetometer(LIS3DML *mag);

	/*
	 * DMA Read magnetometer data
	 */
	uint8_t LIS3MDL_ReadMag_DMA(LIS3DML *mag);
	void LIS3MDL_ReadMag_DMA_Complete(LIS3DML *mag);












#endif /* INC_LIS3MDL_H_ */
