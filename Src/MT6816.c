/*
 * mt6816.c
 *
 *  Created on: May 11, 2024
 *      Author: xange
 */

#include "MT6816.h"




uint8_t MT6816_Init(MT6816 *magSensor, SPI_HandleTypeDef *SPI_Handle, GPIO_TypeDef *csPinBank, uint16_t csPin){

	 magSensor->SPI_Handle = SPI_Handle;
	 magSensor->csPinBank = csPinBank;
	 magSensor->csPin = csPin;

	 //read if mag is there, what chip doesnt have chip id, anyway.
	 uint8_t mag_detect,status=0;

	 HAL_GPIO_WritePin(magSensor->csPinBank, magSensor->csPin, GPIO_PIN_RESET);
	 HAL_Delay(1);
	 HAL_GPIO_WritePin(magSensor->csPinBank, magSensor->csPin, GPIO_PIN_SET);

	 status+=MT6816_ReadRegister(magSensor, MT6816_REG_ANGLE_LB , &mag_detect);
	 if((mag_detect & 0x02)==2){
		 return 0;
	 }

	 return status;

}

uint8_t MT6816_Init_PWM(MT6816 *magSensor, TIM_HandleTypeDef *TIM_Handle){

	magSensor->TIM_Handle = TIM_Handle;
	HAL_TIM_Base_Start(magSensor->TIM_Handle);
	HAL_TIM_IC_Start(magSensor->TIM_Handle, TIM_CHANNEL_2);

	uint8_t status=0;


	return status;
}





uint8_t MT6816_ReadRegister(MT6816 *magSensor, uint8_t regAddr, uint8_t *data){

	uint8_t txBuff[2] = { regAddr | 0x80, 0x00};
	uint8_t rxBuff[2];

	HAL_GPIO_WritePin(magSensor->csPinBank, magSensor->csPin, GPIO_PIN_RESET);
	uint8_t status= (HAL_SPI_TransmitReceive(magSensor->SPI_Handle, txBuff, rxBuff, 2, HAL_MAX_DELAY)==HAL_OK);
	HAL_GPIO_WritePin(magSensor->csPinBank, magSensor->csPin, GPIO_PIN_SET);

	*data = rxBuff[1];

	return status;
}

uint8_t MT6816_WriteRegister(MT6816 *magSensor, uint8_t regAddr, uint8_t data){

	uint8_t txBuff[2] = { regAddr, 0x00};
	uint8_t rxBuff[2];

	HAL_GPIO_WritePin(magSensor->csPinBank, magSensor->csPin, GPIO_PIN_RESET);
	uint8_t status= (HAL_SPI_TransmitReceive(magSensor->SPI_Handle, txBuff, rxBuff, 2, HAL_MAX_DELAY)==HAL_OK);
	HAL_GPIO_WritePin(magSensor->csPinBank, magSensor->csPin, GPIO_PIN_SET);

	return status;

}


uint8_t MT6816_ReadAngle(MT6816 *magSensor){

	uint8_t txBuff[3] = { MT6816_REG_ANGLE_HB | 0x80, 0x00,0x00};
	uint8_t rxBuff[3];

	HAL_GPIO_WritePin(magSensor->csPinBank, magSensor->csPin, GPIO_PIN_RESET);
	uint8_t status= (HAL_SPI_TransmitReceive(magSensor->SPI_Handle, txBuff, rxBuff, 3, HAL_MAX_DELAY)==HAL_OK);
	HAL_GPIO_WritePin(magSensor->csPinBank, magSensor->csPin, GPIO_PIN_SET);

	magSensor->Mag_Warn = rxBuff[2] & 0x02;
	//if(magSensor->Mag_Warn == 2){
		//return 0;
	//}

	uint16_t angle_raw =  ((rxBuff[1]<<8)| rxBuff[2])>>2;

	magSensor->angle = ((float)angle_raw/RESOLUTION )* 360.0f;

	return status;
}



float MT6816_ReadAngle_PWM(MT6816 *magSensor){

	uint32_t icvalue;
	float angle_rad;


	icvalue = HAL_TIM_ReadCapturedValue(magSensor->TIM_Handle, TIM_CHANNEL_2);
	angle_rad = (icvalue-16) * 0.088;

	return angle_rad;
}













