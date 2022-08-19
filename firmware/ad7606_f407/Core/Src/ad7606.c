/*
 * ad7606.c
 * Description: Device Driver Source File for AD7606 Module
 *
 *  Created on: Aug 19, 2022
 *      Author: hphnngcquan
 */

/* Includes ------------------------------------------------------------------*/
#include "ad7606.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* External functions --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/*
================================================================================
              ##### Initialization and de-initialization functions #####
================================================================================
*/

/**
 * @brief  Initialize all possible handle pointer to needed interfaces
 * @param  dev pointer to a AD7606_HandleTyeDef structure that contains
 *               the configuration information for AD7606 module.
 * @param  spiHandle pointer to a SPI_HandleTypeDef structure that contains
 *               the configuration information for SPI module.
 * @param  dmaHandle pointer to a DMA_HandleTypeDef structure that contains
 *               the configuration information for DMA controller.
 * @param  timHandle pointer to a TIM_HandleTypeDef structure that contains
 *               the configuration information for TIM mmodule.
 * @retval None
 */
void AD7606_Init(AD7606_HandleTyeDef *dev, SPI_HandleTypeDef *spiHandle,
		DMA_HandleTypeDef *dmaHandle, TIM_HandleTypeDef *timHandle)
{
	dev->hspi = spiHandle;
	dev->hdmarx = dmaHandle;
	dev->htim = timHandle;
	return;
}


/**
 * @brief  Reset AD7606 Module before using
 * @retval None
 */
void AD7606_RST(void)
{
	AD_RST_H;
	HAL_Delay(1);
	AD_RST_L;
}


/**
 * @brief  Init oversampling rate for AD7606
 * @param osRate,
 * @retval None
 */
void AD7606_OS_SET(uint8_t osRate)
{
	switch (osRate)
	{
	case OS_NO_RATIO:	//000
		AD_OS0_L;
		AD_OS1_L;
		AD_OS2_L;
		break;
	case OS_RATIO_2:	//001
		AD_OS0_H;
		AD_OS1_L;
		AD_OS2_L;
		break;
	case OS_RATIO_4:	//010
		AD_OS0_L;
		AD_OS1_H;
		AD_OS2_L;
		break;
	case OS_RATIO_8:	//011
		AD_OS0_H;
		AD_OS1_H;
		AD_OS2_L;
		break;
	case OS_RATIO_16:	//100
		AD_OS0_L;
		AD_OS1_L;
		AD_OS2_H;
		break;
	case OS_RATIO_32:	//101
		AD_OS0_H;
		AD_OS1_L;
		AD_OS2_H;
		break;
	case OS_RATIO_64:	//110
		AD_OS0_L;
		AD_OS1_H;
		AD_OS2_H;
		break;
	default:
		AD_OS0_L;
		AD_OS1_L;
		AD_OS2_L;
		break;

	}
}

/*
 ================================================================================
 	 	 	 	 	 ##### AD7606 operation functions #####
 ================================================================================
 */

/**
 * @brief  Read an amount of data from AD7606 module via SPI interface
 * @param  DEV pointer to a AD7606_HandleTyeDef structure that contains
 *               the configuration information for AD7606 module.
 * @param  pAdcDataBufPtr pointer to Analog data buffer
 * @param  Size amount of data to be received
 * @param  Timeout Timeout duration
 * @retval HAL status
 */
HAL_StatusTypeDef AD7606_StartReadBytes(AD7606_HandleTyeDef *dev,
		int16_t *pAdcDataBuf, uint16_t Size, uint32_t Timeout)
{
	//Set Transaction Information
	dev->pAdcDataBufPtr = pAdcDataBuf;
	dev->adcDataBufSize = Size;
	//wait until the BUSY pin is at reset state (Logic "0")
	while (HAL_GPIO_ReadPin(AD_BUSY_GPIO_Port, AD_BUSY_Pin) == GPIO_PIN_SET);
	return HAL_SPI_Receive(dev->hspi, (uint8_t*) dev->pAdcDataBufPtr,
			dev->adcDataBufSize, Timeout);
}


/**
 * @brief  Read an amount of data from AD7606 module via SPI interface with DMA
 * @param  dev pointer to a AD7606_HandleTyeDef structure that contains
 *               the configuration information for AD7606 module.
 * @param  pAdcDataBufPtr pointer to Analog data buffer
 * @param  Size amount of data to be received
 * @retval HAL status
 */
HAL_StatusTypeDef AD7606_StartReadBytes_DMA(AD7606_HandleTyeDef *dev,
		int16_t *pAdcDataBuf, uint16_t Size)
{
	//Set Transaction Information
	dev->pAdcDataBufPtr = pAdcDataBuf;
	dev->adcDataBufSize = Size;
	//wait until the BUSY pin is at reset state (Logic "0")
	while (HAL_GPIO_ReadPin(AD_BUSY_GPIO_Port, AD_BUSY_Pin) == GPIO_PIN_SET);
	HAL_Delay(0.0000015);
	return HAL_SPI_Receive_DMA(dev->hspi, (uint8_t*) dev->pAdcDataBufPtr,
			dev->adcDataBufSize);
}

/**
 * @brief  Drive CONVERSION START pins A and B of AD7606 using TIM_PWM
 * @param  dev pointer to a AD7606_HandleTyeDef structure that contains
 *               the configuration information for AD7606 module.
 * @retval HAL status
 */
HAL_StatusTypeDef AD7606_CVST_START(AD7606_HandleTyeDef *dev)
{

	return HAL_TIM_PWM_Start(dev->htim, TIM_CHANNEL_2);
}


/**
 * @brief  Stop CONVERSION START pins A and B of AD7606 using TIM_PWM
 * @param  dev pointer to a AD7606_HandleTyeDef structure that contains
 *               the configuration information for AD7606 module.
 * @retval HAL status
 */
HAL_StatusTypeDef AD7606_CVST_STOP(AD7606_HandleTyeDef *dev)
{
	return HAL_TIM_PWM_Stop(dev->htim, TIM_CHANNEL_2);
}

/**
 * @brief  Convert from Analog data to Voltage with reference voltage 2.5V, RANGE: 10V
 * @param  dev pointer to a AD7606_HandleTyeDef structure that contains
 *               the configuration information for AD7606 module.
 * @retval HAL status
 */

void AD7606_ConvertToVoltage(uint16_t channelNum, int16_t *pAdcDataBuf,
		float *pVoltDataBuf)
{
	uint16_t i;
	for (i = 0; i < channelNum; i++)
	{
		pVoltDataBuf[i] = ((float) pAdcDataBuf[i] * AD7606_RANGE_VALUE_10)
				/ 32768.0;
	}
	return;
}

