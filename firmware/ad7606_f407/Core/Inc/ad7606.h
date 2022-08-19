/*
 * ad7606.h
 * Description: Device Driver Header File for AD7606 Module

 *  Created on: Aug 19, 2022
 *      Author: hphnngcquan
 */

#ifndef INC_AD7606_H_
#define INC_AD7606_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
/* Exported types ------------------------------------------------------------*/
/**
 * @brief  AD7606 Handle Structure Definition
 */
typedef struct {
	SPI_HandleTypeDef 		*hspi;				/*!< SPI Handle TypeDef Declaration 			*/

	TIM_HandleTypeDef		*htim;				/*!< TIM Handle TypeDef Declaration 			*/

	DMA_HandleTypeDef		*hdmarx;			/*!< DMA RX Handle TypeDef Declaration 			*/

	int16_t 				*pAdcDataBufPtr;	/*!< Pointer to AD7606 Analog Data Buffer       */

	uint16_t 				adcDataBufSize;		/*!< AD7606 Analog Data Buffer Size       		*/

	float 					*pVoltDataBufPtr;	/*!< Pointer to AD7606 Voltage Data Buffer      */

	uint16_t 				voltDataBufSize;	/*!< AD7606 Voltage Data Buffer Size      		*/

} AD7606_HandleTyeDef;

/**
  * @brief  AD7606 Channel Structure
  */
typedef enum
{
	ADC_CHANNEL_1 = 1,	/*!< AD7606 Channel 2 */
	ADC_CHANNEL_2 = 2,	/*!< AD7606 Channel 3 */
	ADC_CHANNEL_3 = 3,	/*!< AD7606 Channel 4 */
	ADC_CHANNEL_4 = 4,	/*!< AD7606 Channel 5 */
	ADC_CHANNEL_5 = 5,	/*!< AD7606 Channel 6 */
	ADC_CHANNEL_6 = 6,	/*!< AD7606 Channel 7 */
	ADC_CHANNEL_7 = 7,	/*!< AD7606 Channel 8 */
	ADC_CHANNEL_8 = 0,	/*!< AD7606 Channel 8 */
}AD7606_ChannelTypeDef;


/**
  * @brief  AD7606 Over-sampling Ratio Structure
  */
typedef enum
{
	OS_NO_RATIO		= 1,	/*!< AD7606 No Over-sampling  			*/
	OS_RATIO_2 		= 2,	/*!< AD7606 Over-sampling Ratio = 2 	*/
	OS_RATIO_4 		= 4,	/*!< AD7606 Over-sampling Ratio = 4 	*/
	OS_RATIO_8 		= 8,	/*!< AD7606 Over-sampling Ratio = 8  	*/
	OS_RATIO_16 	= 16,	/*!< AD7606 Over-sampling Ratio = 16  	*/
	OS_RATIO_32 	= 32,	/*!< AD7606 Over-sampling Ratio = 32  	*/
	OS_RATIO_64 	= 64,	/*!< AD7606 Over-sampling Ratio = 64  	*/
} AD7606_OSRateTypeDef;

/**
  * @brief  AD7606 Number of channels to read
  */
typedef enum
{
	READ_1_CHANNEL = 1,		/*!< AD7606 Read 1 channel 		*/
	READ_2_CHANNEL = 2,		/*!< AD7606 Read 2 channels 	*/
	READ_3_CHANNEL = 3,		/*!< AD7606 Read 3 channels 	*/
	READ_4_CHANNEL = 4,		/*!< AD7606 Read 4 channels 	*/
	READ_5_CHANNEL = 5,		/*!< AD7606 Read 5 channels 	*/
	READ_6_CHANNEL = 6,	/*!< AD7606 Read 6 channels 	*/
	READ_7_CHANNEL = 7,		/*!< AD7606 Read 7 channels 	*/
	READ_8_CHANNEL = 8,		/*!< AD7606 Read 8 channels 	*/
}AD7606_NumChannelTypeDef;

/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/


#define ADC_DATA_BUF_SIZE		(8U)	/*!< AD7606 Analog Data Buffer Size 	*/
#define VOLTAGE_DATA_BUF_SIZE	(8U)	/*!< AD7606 Voltage Data Buffer Size    */
#define USB_TX_DATA_BUF_SIZE	(8U)	/*!< USB VCP Transmit Data Buffer SIze  */


#define AD7606_RANGE_VALUE_10	(10U) /*!< Analog Input Range 10V */




/* Exported functions prototypes ---------------------------------------------*/
void AD7606_Init (AD7606_HandleTyeDef *dev, SPI_HandleTypeDef *spiHandle, DMA_HandleTypeDef *dmaHandle, TIM_HandleTypeDef *timHandle);
void AD7606_RST(void);
void AD7606_OS_SET(uint8_t osRate);
void AD7606_ConvertToVoltage (uint16_t channelNum, int16_t *pAdcDataBuf, float *pVoltDataBuf);
HAL_StatusTypeDef AD7606_StartReadBytes(AD7606_HandleTyeDef *dev, int16_t *pAdcDataBuf, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef AD7606_StartReadBytes_DMA(AD7606_HandleTyeDef *dev, int16_t *pAdcDataBuf, uint16_t Size);
HAL_StatusTypeDef AD7606_CVST_START(AD7606_HandleTyeDef *dev);
HAL_StatusTypeDef AD7606_CVST_STOP(AD7606_HandleTyeDef *dev);

/* Private defines -----------------------------------------------------------*/

/* GPIO State assignment defines for AD_RST Pin */
#define AD_RST_H	HAL_GPIO_WritePin(AD_RST_GPIO_Port, AD_RST_Pin, GPIO_PIN_SET)		/*!< Reset AD_RST pin  */
#define AD_RST_L	HAL_GPIO_WritePin(AD_RST_GPIO_Port, AD_RST_Pin, GPIO_PIN_RESET)		/*!< Set AD_RST pin  */

/* GPIO State assignment defines for AD_OS0 Pin */
#define AD_OS0_H	HAL_GPIO_WritePin(AD_OS0_GPIO_Port, AD_OS0_Pin, GPIO_PIN_SET)		/*!< Reset AD_OS0 pin  */
#define AD_OS0_L	HAL_GPIO_WritePin(AD_OS0_GPIO_Port, AD_OS0_Pin, GPIO_PIN_RESET)		/*!< Set AD_OS0 pin  */

/* GPIO State assignment defines for AD_OS1 Pin */
#define AD_OS1_H	HAL_GPIO_WritePin(AD_OS1_GPIO_Port, AD_OS1_Pin, GPIO_PIN_SET)		/*!< Reset AD_OS1 pin  */
#define AD_OS1_L	HAL_GPIO_WritePin(AD_OS1_GPIO_Port, AD_OS1_Pin, GPIO_PIN_RESET)		/*!< Set AD_OS1 pin  */

/* GPIO State assignment defines for AD_OS2 Pin */
#define AD_OS2_H	HAL_GPIO_WritePin(AD_OS2_GPIO_Port, AD_OS2_Pin, GPIO_PIN_SET)		/*!< Reset AD_OS2 pin  */
#define AD_OS2_L	HAL_GPIO_WritePin(AD_OS1_GPIO_Port, AD_OS1_Pin, GPIO_PIN_RESET)		/*!< Set AD_OS2 pin  */



#endif /* INC_AD7606_H_ */
