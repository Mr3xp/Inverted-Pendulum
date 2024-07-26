/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DEBUG_Pin GPIO_PIN_14
#define DEBUG_GPIO_Port GPIOC
#define PHASE_WH_Pin GPIO_PIN_0
#define PHASE_WH_GPIO_Port GPIOC
#define PHASE_VH_Pin GPIO_PIN_1
#define PHASE_VH_GPIO_Port GPIOC
#define PHASE_UH_Pin GPIO_PIN_2
#define PHASE_UH_GPIO_Port GPIOC
#define SENSE_U_Pin GPIO_PIN_0
#define SENSE_U_GPIO_Port GPIOA
#define SENSE_V_Pin GPIO_PIN_1
#define SENSE_V_GPIO_Port GPIOA
#define SENSE_W_Pin GPIO_PIN_2
#define SENSE_W_GPIO_Port GPIOA
#define BAT_MON_Pin GPIO_PIN_6
#define BAT_MON_GPIO_Port GPIOA
#define PHASE_WL_Pin GPIO_PIN_7
#define PHASE_WL_GPIO_Port GPIOA
#define PHASE_VL_Pin GPIO_PIN_0
#define PHASE_VL_GPIO_Port GPIOB
#define PHASE_UL_Pin GPIO_PIN_1
#define PHASE_UL_GPIO_Port GPIOB
#define PWM_INPUT_2_Pin GPIO_PIN_2
#define PWM_INPUT_2_GPIO_Port GPIOB
#define SPI2_CS1_Pin GPIO_PIN_12
#define SPI2_CS1_GPIO_Port GPIOB
#define PWM_INPUT_1_Pin GPIO_PIN_6
#define PWM_INPUT_1_GPIO_Port GPIOC
#define PWM_INPUT1_DIR_Pin GPIO_PIN_7
#define PWM_INPUT1_DIR_GPIO_Port GPIOC
#define DRV_EN_Pin GPIO_PIN_9
#define DRV_EN_GPIO_Port GPIOC
#define PWM_INPUT2_DIR_Pin GPIO_PIN_8
#define PWM_INPUT2_DIR_GPIO_Port GPIOA
#define SPI3_INT3_Pin GPIO_PIN_15
#define SPI3_INT3_GPIO_Port GPIOA
#define SPI3_CS1_Pin GPIO_PIN_2
#define SPI3_CS1_GPIO_Port GPIOD
#define SPI3_IN1_Pin GPIO_PIN_4
#define SPI3_IN1_GPIO_Port GPIOB
#define SPI3_IN1_EXTI_IRQn EXTI4_IRQn
#define SPI3_CS2_Pin GPIO_PIN_6
#define SPI3_CS2_GPIO_Port GPIOB
#define SPI2_CS2_Pin GPIO_PIN_9
#define SPI2_CS2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
