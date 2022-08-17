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
enum ADC_CHANNEL {
	ADC_CHANNEL_1 = 0,
	ADC_CHANNEL_2 = 1,
	ADC_CHANNEL_3 = 2,
	ADC_CHANNEL_4 = 3,
	ADC_CHANNEL_5 = 4,
	ADC_CHANNEL_6 = 5,
	ADC_CHANNEL_7 = 6,
	ADC_CHANNEL_8 = 7,
};

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define ADC_DATA_BUF_SIZE		(8U)
#define VOLTAGE_DATA_BUF_SIZE	(8U)
#define USB_TX_DATA_BUF_SIZE	(8U)
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void AD7606_RST(void);
void AD7606_OS_SET(void);
void AD7606_Delay (uint32_t Delay);
void AD7606_StartReadBytes(SPI_HandleTypeDef *hspi, int16_t *pDst, uint16_t Length);
void AD7606_ConvertToVoltage (uint16_t Length, int16_t *pSrc, float *pDst);
void AD7606_CVST_START(void);
void AD7606_CVST_STOP(void);
void LED_Toggle(void);

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
#define USER_LED_Pin GPIO_PIN_13
#define USER_LED_GPIO_Port GPIOD
#define AD_CVST_Pin GPIO_PIN_3
#define AD_CVST_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
