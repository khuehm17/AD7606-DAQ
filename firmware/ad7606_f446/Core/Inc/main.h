/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AD_RST_Pin GPIO_PIN_2
#define AD_RST_GPIO_Port GPIOC
#define AD_OS2_Pin GPIO_PIN_3
#define AD_OS2_GPIO_Port GPIOC
#define AD_OS1_Pin GPIO_PIN_0
#define AD_OS1_GPIO_Port GPIOA
#define AD_OS0_Pin GPIO_PIN_1
#define AD_OS0_GPIO_Port GPIOA
#define AD_CS_Pin GPIO_PIN_4
#define AD_CS_GPIO_Port GPIOA
#define AD_SCK_Pin GPIO_PIN_5
#define AD_SCK_GPIO_Port GPIOA
#define AD_DOU_A_Pin GPIO_PIN_6
#define AD_DOU_A_GPIO_Port GPIOA
#define AD_BUSY_Pin GPIO_PIN_4
#define AD_BUSY_GPIO_Port GPIOC
#define AD_FRST_Pin GPIO_PIN_5
#define AD_FRST_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
