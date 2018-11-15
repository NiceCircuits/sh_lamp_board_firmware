/**
 ******************************************************************************
 * File Name          : ADC.c
 * Description        : This file provides code for the configuration
 *                      of the ADC instances.
 ******************************************************************************
 * This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * Copyright (c) 2018 STMicroelectronics International N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

#include "gpio.h"
#include "dma.h"

#include "cmsis_os.h"
#include "stm32f7xx_hal.h"
#include "tim.h"
#include "debug.h"

enum
{
	ADC_NUMBER_OF_CHANNELS = 4, ADC_NUMBER_OF_ADC = 3
};

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;

QueueHandle_t ADC_queue;
uint16_t adc_data1[256];
uint16_t adc_data2[256];
uint8_t temp[40];

const uint32_t adc_channels[ADC_NUMBER_OF_ADC][ADC_NUMBER_OF_CHANNELS] =
{
{ ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_VREFINT, ADC_CHANNEL_TEMPSENSOR },
{ ADC_CHANNEL_10, ADC_CHANNEL_11, ADC_CHANNEL_12, ADC_CHANNEL_13 },
{ ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3 } };

ADC_HandleTypeDef* hadcs[ADC_NUMBER_OF_ADC] =
{ &hadc1, &hadc2, &hadc3 };

ADC_TypeDef * adcs[ADC_NUMBER_OF_ADC] =
{ ADC1, ADC2, ADC3 };

static void ADC_conversion_complete_callback(DMA_HandleTypeDef *hdma);
static void ADC_conversion_error_callback(DMA_HandleTypeDef *hdma);

static HAL_StatusTypeDef ADC_start(ADC_HandleTypeDef* hadc, uint16_t* pData1, uint16_t* pData2, uint32_t Length);

void vAdcTask(void *pvParameters)
{
	uint16_t* last_data = NULL;
	BaseType_t status;

	__HAL_ADC_ENABLE(&hadc1);
	__HAL_ADC_ENABLE(&hadc2);
	__HAL_ADC_ENABLE(&hadc3);
	hadc1.Init.DMAContinuousRequests = ENABLE;
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_4);

	ADC_queue = xQueueCreate(1, sizeof(uint16_t*));
	if (ADC_queue == NULL)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	ADC_start(&hadc1, adc_data1, adc_data2, ADC_NUMBER_OF_ADC * ADC_NUMBER_OF_CHANNELS);
	while (1)
	{
		status = xQueueReceive(ADC_queue, &last_data, 250);
		if (status)
		{
//			debug_print("0,%d,", last_data==adc_data1);
//			for (int i = 0; i < ADC_NUMBER_OF_ADC * ADC_NUMBER_OF_CHANNELS; i++)
//			{
//				debug_print("%d,", last_data[i]);
//			}
//			debug_print("0\r\n");
//			HAL_GPIO_TogglePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin);
		}
	}
}

static HAL_StatusTypeDef ADC_start(ADC_HandleTypeDef* hadc, uint16_t* pData1, uint16_t* pData2, uint32_t Length)
{
	__IO uint32_t counter = 0;

	/* Check the parameters */
	assert_param(IS_FUNCTIONAL_STATE(hadc->Init.ContinuousConvMode));
	assert_param(IS_ADC_EXT_TRIG_EDGE(hadc->Init.ExternalTrigConvEdge));
	assert_param(IS_FUNCTIONAL_STATE(hadc->Init.DMAContinuousRequests));

	/* Process locked */
	__HAL_LOCK(hadc);

	/* Check if ADC peripheral is disabled in order to enable it and wait during
	 Tstab time the ADC's stabilization */
	if ((hadc->Instance->CR2 & ADC_CR2_ADON) != ADC_CR2_ADON)
	{
		/* Enable the Peripheral */
		__HAL_ADC_ENABLE(hadc);

		/* Delay for temperature sensor stabilization time */
		/* Compute number of CPU cycles to wait for */
		counter = (ADC_STAB_DELAY_US * (SystemCoreClock / 1000000));
		while (counter != 0)
		{
			counter--;
		}
	}

	/* Start conversion if ADC is effectively enabled */
	if (HAL_IS_BIT_SET(hadc->Instance->CR2, ADC_CR2_ADON))
	{
		/* Set ADC state                                                          */
		/* - Clear state bitfield related to regular group conversion results     */
		/* - Set state bitfield related to regular group operation                */
		ADC_STATE_CLR_SET(hadc->State,
				HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR,
				HAL_ADC_STATE_REG_BUSY);

		/* If conversions on group regular are also triggering group injected,    */
		/* update ADC state.                                                      */
		if (READ_BIT(hadc->Instance->CR1, ADC_CR1_JAUTO) != RESET)
		{
			ADC_STATE_CLR_SET(hadc->State, HAL_ADC_STATE_INJ_EOC, HAL_ADC_STATE_INJ_BUSY);
		}

		/* State machine update: Check if an injected conversion is ongoing */
		if (HAL_IS_BIT_SET(hadc->State, HAL_ADC_STATE_INJ_BUSY))
		{
			/* Reset ADC error code fields related to conversions on group regular */
			CLEAR_BIT(hadc->ErrorCode, (HAL_ADC_ERROR_OVR | HAL_ADC_ERROR_DMA));
		}
		else
		{
			/* Reset ADC all error code fields */
			ADC_CLEAR_ERRORCODE(hadc);
		}

		/* Process unlocked */
		/* Unlock before starting ADC conversions: in case of potential           */
		/* interruption, to let the process to ADC IRQ Handler.                   */
		__HAL_UNLOCK(hadc);

		/* Set the DMA transfer complete callback */
		hadc->DMA_Handle->XferCpltCallback = ADC_conversion_complete_callback;
		hadc->DMA_Handle->XferM1CpltCallback = ADC_conversion_complete_callback;

		/* Set the DMA half transfer complete callback */
		//hadc->DMA_Handle->XferHalfCpltCallback = ADC_conversion_half_complete_callback;
		/* Set the DMA error callback */
		hadc->DMA_Handle->XferErrorCallback = ADC_conversion_error_callback;

		/* Manage ADC and DMA start: ADC overrun interruption, DMA start, ADC     */
		/* start (in case of SW start):                                           */

		/* Clear regular group conversion flag and overrun flag */
		/* (To ensure of no unknown state from potential previous ADC operations) */
		__HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_EOC);

		/* Enable ADC overrun interrupt */
		__HAL_ADC_ENABLE_IT(hadc, ADC_IT_OVR);

		if (hadc->Init.DMAContinuousRequests != DISABLE)
		{
			/* Enable the selected ADC DMA request after last transfer */
			ADC->CCR |= ADC_CCR_DDS;
		}
		else
		{
			/* Disable the selected ADC EOC rising on each regular channel conversion */
			ADC->CCR &= ~ADC_CCR_DDS;
		}

		/* Enable the DMA Stream in double buffer mode*/
		HAL_DMAEx_MultiBufferStart_IT(hadc->DMA_Handle, (uint32_t) &ADC->CDR, (uint32_t) pData1, (uint32_t) pData2, Length);

		/* if no external trigger present enable software conversion of regular channels */
		if ((hadc->Instance->CR2 & ADC_CR2_EXTEN) == RESET)
		{
			/* Enable the selected ADC software conversion for regular group */
			hadc->Instance->CR2 |= (uint32_t) ADC_CR2_SWSTART;
		}
	}

	/* Return function status */
	return HAL_OK;
}

/* ADC init function */
void ADC_Init(void)
{
	ADC_MultiModeTypeDef multimode;
	ADC_ChannelConfTypeDef sConfig;
	ADC_HandleTypeDef* hadc;

	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_CC4;

	for (int adc = 0; adc < ADC_NUMBER_OF_ADC; adc++)
	{
		hadc = hadcs[adc];
		/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
		 */
		hadc->Instance = adcs[adc];
		hadc->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
		hadc->Init.Resolution = ADC_RESOLUTION_12B;
		hadc->Init.ScanConvMode = ADC_SCAN_ENABLE;
		hadc->Init.ContinuousConvMode = DISABLE;
		hadc->Init.DiscontinuousConvMode = DISABLE;
		hadc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
		hadc->Init.NbrOfConversion = ADC_NUMBER_OF_CHANNELS;
		hadc->Init.DMAContinuousRequests = DISABLE;
		hadc->Init.EOCSelection = ADC_EOC_SEQ_CONV;

		if (HAL_ADC_Init(hadc) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
		 */
		for (uint_fast8_t ch = 0; ch < ADC_NUMBER_OF_CHANNELS; ch++)
		{
			sConfig.Channel = adc_channels[adc][ch];
			sConfig.Rank = ch + 1;
			sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
			if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
			{
				_Error_Handler(__FILE__, __LINE__);
			}
		}
	}
	/**Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_TRIPLEMODE_REGSIMULT;
	multimode.DMAAccessMode = ADC_DMAACCESSMODE_1;
	multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
	ADC->CCR |= ADC_CCR_DDS;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

	GPIO_InitTypeDef GPIO_InitStruct;
	if (adcHandle->Instance == ADC1)
	{
		/* ADC1 clock enable */
		__HAL_RCC_ADC1_CLK_ENABLE()
		;

		/**ADC1 GPIO Configuration
		 PA4     ------> ADC1_IN4
		 PA5     ------> ADC1_IN5
		 */
		GPIO_InitStruct.Pin = I7_IN_Pin | I8_IN_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* ADC1 DMA Init */
		/* ADC1 Init */
		hdma_adc1.Instance = DMA2_Stream0;
		hdma_adc1.Init.Channel = DMA_CHANNEL_0;
		hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
		hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
		hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
		hdma_adc1.Init.Mode = DMA_CIRCULAR;
		hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
		hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		__HAL_LINKDMA(adcHandle, DMA_Handle, hdma_adc1);

	}
	else if (adcHandle->Instance == ADC2)
	{
		/* ADC2 clock enable */
		__HAL_RCC_ADC2_CLK_ENABLE()
		;

		/**ADC2 GPIO Configuration
		 PC0     ------> ADC2_IN10
		 PC1     ------> ADC2_IN11
		 PC2     ------> ADC2_IN12
		 PC3     ------> ADC2_IN13
		 */
		GPIO_InitStruct.Pin = LINE_IN_Pin | I1_IN_Pin | I2_IN_Pin | I3_IN_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	}
	else if (adcHandle->Instance == ADC3)
	{
		/* ADC3 clock enable */
		__HAL_RCC_ADC3_CLK_ENABLE()
		;

		/**ADC3 GPIO Configuration
		 PA0-WKUP     ------> ADC3_IN0
		 PA1     ------> ADC3_IN1
		 PA2     ------> ADC3_IN2
		 PA3     ------> ADC3_IN3
		 */
		GPIO_InitStruct.Pin = I4_IN_Pin | I5_IN_Pin | THERM_IN_Pin | I6_IN_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

	if (adcHandle->Instance == ADC1)
	{
		/* Peripheral clock disable */
		__HAL_RCC_ADC1_CLK_DISABLE();

		/**ADC1 GPIO Configuration
		 PA4     ------> ADC1_IN4
		 PA5     ------> ADC1_IN5
		 */
		HAL_GPIO_DeInit(GPIOA, I7_IN_Pin | I8_IN_Pin);

		/* ADC1 DMA DeInit */
		HAL_DMA_DeInit(adcHandle->DMA_Handle);
	}
	else if (adcHandle->Instance == ADC2)
	{
		/* Peripheral clock disable */
		__HAL_RCC_ADC2_CLK_DISABLE();

		/**ADC2 GPIO Configuration
		 PC0     ------> ADC2_IN10
		 PC1     ------> ADC2_IN11
		 PC2     ------> ADC2_IN12
		 PC3     ------> ADC2_IN13
		 */
		HAL_GPIO_DeInit(GPIOC, LINE_IN_Pin | I1_IN_Pin | I2_IN_Pin | I3_IN_Pin);
	}
	else if (adcHandle->Instance == ADC3)
	{
		/* Peripheral clock disable */
		__HAL_RCC_ADC3_CLK_DISABLE();

		/**ADC3 GPIO Configuration
		 PA0-WKUP     ------> ADC3_IN0
		 PA1     ------> ADC3_IN1
		 PA2     ------> ADC3_IN2
		 PA3     ------> ADC3_IN3
		 */
		HAL_GPIO_DeInit(GPIOA, I4_IN_Pin | I5_IN_Pin | THERM_IN_Pin | I6_IN_Pin);
	}
}

static void ADC_conversion_complete_callback(DMA_HandleTypeDef *hdma)
{
	uint16_t* data = adc_data2;
	BaseType_t xHigherPriorityTaskWoken;

	/* Retrieve ADC handle corresponding to current DMA handle */
	ADC_HandleTypeDef* hadc = (ADC_HandleTypeDef*) ((DMA_HandleTypeDef*) hdma)->Parent;

	/* Update state machine on conversion status if not in error state */
	if (HAL_IS_BIT_CLR(hadc->State, HAL_ADC_STATE_ERROR_INTERNAL | HAL_ADC_STATE_ERROR_DMA))
	{
		/* Update ADC state machine */
		SET_BIT(hadc->State, HAL_ADC_STATE_REG_EOC);

		if (HAL_IS_BIT_SET(hdma->Instance->CR, DMA_SxCR_CT))
		{
			data = adc_data1;
		}
		xQueueSendFromISR(ADC_queue, &data, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
	else
	{
		/* Call DMA error callback */
		hadc->DMA_Handle->XferErrorCallback(hdma);
	}
}

static void ADC_conversion_error_callback(DMA_HandleTypeDef *hdma)
{
	ADC_HandleTypeDef* hadc = (ADC_HandleTypeDef*) ((DMA_HandleTypeDef*) hdma)->Parent;
	hadc->State = HAL_ADC_STATE_ERROR_DMA;
	/* Set ADC error code to DMA error */
	hadc->ErrorCode |= HAL_ADC_ERROR_DMA;
	_Error_Handler(__FILE__, __LINE__);
}

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
