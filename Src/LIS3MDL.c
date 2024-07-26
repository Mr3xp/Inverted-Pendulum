/*
 * LIS3MDL.c
 *
 *  Created on: May 13, 2024
 *      Author: xange
 */


#include "LIS3MDL.h"



uint8_t LIS3MDL_Init(LIS3DML *mag, SPI_HandleTypeDef *SPI_Handle, GPIO_TypeDef *csBank, uint16_t csPin){

	mag->SPI_Handle = SPI_Handle;
	mag->csBank = csBank;
	mag->csPin = csPin;

	mag->readingMagFlag = 0;
	uint8_t status = 0;
	uint8_t chip_id_check;





	status += LIS3MDL_ReadRegister(mag, LIS3MDL_REG_CHIP_ID, &chip_id_check);
	HAL_Delay(1);

	if(chip_id_check != LIS3MDL_REG_CHIP_VALUE ){
		return status = 0 ;
	}

	/*
	 * Bit 7: temp en
	 * Bit 6-5:operative mode selection. 1 1 uhpm
	 * Bit 4-2: Data output rate. for fast_odr not relevant
	 * Bit 1: fast output data rate
	 * Bit 0: self test.
	 */
	status += LIS3MDL_WriteRegister(mag, LIS3MDL_REG_CTRL_1, 0b01100010);//odr 135 Hz
	HAL_Delay(1);

	/*
	 * Bit 7: must be 0
	 * Bit 6-5: Full scale
	 * Bit 4: must be 0
	 * Bit 3: reboot memory
	 * Bit 2: soft reset
	 * Bit 1-0: must be 0
	 */
	status += LIS3MDL_WriteRegister(mag, LIS3MDL_REG_CTRL_2, 0b01000000); //+-12 gauss
	HAL_Delay(1);

	/*
	 * Bit 7: must be 0
	 * Bit 6: must be 0
	 * Bit 5: low power
	 * Bit 4: must be 0
	 * Bit 3: must be 0
	 * Bit 2: Spi mode default 0
	 * Bit 1-0: system operation mode, 0 0 continuous conversion mode
	 */
	status += LIS3MDL_WriteRegister(mag, LIS3MDL_REG_CTRL_3, 0b00000000);
	HAL_Delay(1);

	/*
	 * Bit 7: must be 0
	 * Bit 6: must be 0
	 * Bit 5: must be 0
	 * Bit 4: must be 0
	 * Bit 3-2: z axis operative mode 11
	 * Bit 1: data selection lsb at lowest address 0
	 * Bit 0: must be 0
	 */
	status += LIS3MDL_WriteRegister(mag, LIS3MDL_REG_CTRL_4, 0b00001100);
	HAL_Delay(1);

	/*
	 * Bit 7: fast read
	 * Bit 6: block data update
	 */
	status += LIS3MDL_WriteRegister(mag, LIS3MDL_REG_CTRL_5, 0b00000000);
	HAL_Delay(1);

	mag->magConversion = 1/ (float)2281;

	mag->magTxBuff[0] = LIS3MDL_REG_OUT_X_LSB | 0xC0;

	return status;


}


uint8_t LIS3MDL_ReadRegister(LIS3DML *mag, uint8_t regAddr, uint8_t *data){

	uint8_t txBuff[2] = { regAddr | 0x80, 0x00};
	uint8_t rxBuff[2];

	HAL_GPIO_WritePin(mag->csBank, mag->csPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(mag->SPI_Handle, txBuff, rxBuff, 2, HAL_MAX_DELAY ) == HAL_OK);
	HAL_GPIO_WritePin(mag->csBank, mag->csPin, GPIO_PIN_SET);

	*data = rxBuff[1];

	return status;
}


uint8_t LIS3MDL_WriteRegister(LIS3DML *mag, uint8_t regAddr, uint8_t data){

	uint8_t txBuff[2] = { regAddr, 0x00};
	uint8_t rxBuff[2];

	HAL_GPIO_WritePin(mag->csBank, mag->csPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(mag->SPI_Handle, txBuff, rxBuff, 2, HAL_MAX_DELAY ) == HAL_OK);
	HAL_GPIO_WritePin(mag->csBank, mag->csPin, GPIO_PIN_SET);

	return status;
}




uint8_t LIS3MDL_ReadMagnetometer(LIS3DML *mag){

	uint8_t txBuff[7] = { LIS3MDL_REG_OUT_X_LSB | 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // bit 7 read bit and 6 auto increment.
	uint8_t rxBuff[7];

	HAL_GPIO_WritePin(mag->csBank, mag->csPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(mag->SPI_Handle, txBuff, rxBuff, 7, HAL_MAX_DELAY ) == HAL_OK);
	HAL_GPIO_WritePin(mag->csBank, mag->csPin, GPIO_PIN_SET);

	int16_t magX = (int16_t) ((rxBuff[2]<<8) | rxBuff[1]);
	int16_t magY = (int16_t) ((rxBuff[4]<<8) | rxBuff[3]);
	int16_t magZ = (int16_t) ((rxBuff[6]<<8) | rxBuff[5]);

	mag->mag_gauss[0] = mag->magConversion * magX;
	mag->mag_gauss[1] = mag->magConversion * magY;
	mag->mag_gauss[2] = mag->magConversion * magZ;

	return status;
}



uint8_t LIS3MDL_ReadMag_DMA(LIS3DML *mag){

	HAL_GPIO_WritePin(mag->csBank, mag->csPin, GPIO_PIN_RESET);

	if(HAL_SPI_TransmitReceive_DMA(mag->SPI_Handle, mag->magTxBuff, mag->magRxBuff, 7) == HAL_OK){

		mag->readingMagFlag=1;
		return 1;
	}
	else{

		HAL_GPIO_WritePin(mag->csBank, mag->csPin, GPIO_PIN_SET);
		return 0;

	}

}



void LIS3DML_ReadMag_DMA_Complete(LIS3DML *mag){

	HAL_GPIO_WritePin(mag->csBank, mag->csPin, GPIO_PIN_SET);
	mag->readingMagFlag = 0;

	int16_t magX = (int16_t) ((mag->magRxBuff[2]<<8) | mag->magRxBuff[1]);
	int16_t magY = (int16_t) ((mag->magRxBuff[4]<<8) | mag->magRxBuff[3]);
	int16_t magZ = (int16_t) ((mag->magRxBuff[6]<<8) | mag->magRxBuff[5]);

	mag->mag_gauss[0] = mag->magConversion * magX;
	mag->mag_gauss[1] = mag->magConversion * magY;
	mag->mag_gauss[2] = mag->magConversion * magZ;


}



