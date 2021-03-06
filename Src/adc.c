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
#include "can.h"
#include "config.h"
#include <stdbool.h>
#include <math.h>

enum
{
	ADC_N_CHANNELS_PER_ADC = 4,
	ADC_NUMBER_OF_ADC = 3,
	ADC_N_CHANNELS = ADC_NUMBER_OF_ADC * ADC_N_CHANNELS_PER_ADC,
	ADC_SAMPLE_BUFFER_LENGTH = ADC_N_CHANNELS * ADC_SAMPLES_PER_CYCLE,
	ADC_AVERAGE_COEFFICIENT = 1 << 4, // TODO: 1<<6 or more // new_avg = new_sample/coeff + old_avg * (coeff-1)/coeff https://en.wikipedia.org/wiki/Moving_average#Exponential_moving_average
	ADC_ZERO_AVERAGE_COEFFICIENT = 1 << 10, // TODO: 1 << 13,
	PLL_AVERAGE_COEFFICIENT = 1 << 10,
	PLL_XOR_GAIN =  64,
	ADC_MAX_CODE = ((1 << 12) - 1), // Maximum code read from ADC
	ADC_N_I_CHANNELS = 8, // Number of current measuring channels
	ACTIVE_POWER_AVERAGE_COEFFICIENT = 16,
};

typedef enum
{
	ADC_INDEX_I7,
	ADC_INDEX_LINE,
	ADC_INDEX_I4,
	ADC_INDEX_I8,
	ADC_INDEX_I1,
	ADC_INDEX_I5,
	ADC_INDEX_VREF,
	ADC_INDEX_I2,
	ADC_INDEX_THERM,
	ADC_INDEX_TEMPSENSOR,
	ADC_INDEX_I3,
	ADC_INDEX_I6
} adc_index_t;

const adc_index_t ADC_I_CHANNELS[ADC_N_I_CHANNELS] =
{ ADC_INDEX_I1, ADC_INDEX_I2, ADC_INDEX_I3, ADC_INDEX_I4, ADC_INDEX_I5, ADC_INDEX_I6, ADC_INDEX_I7, ADC_INDEX_I8 };

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;

QueueHandle_t ADC_queue;
uint16_t adc_data1[ADC_SAMPLES_PER_CYCLE][ADC_N_CHANNELS];
uint16_t adc_data2[ADC_SAMPLES_PER_CYCLE][ADC_N_CHANNELS];
int32_t adc_averaged_cycle[ADC_SAMPLES_PER_CYCLE][ADC_N_CHANNELS];
uint32_t adc_zeros[ADC_N_CHANNELS] =
{ [0 ... (ADC_N_CHANNELS - 1)] = ADC_ZERO_AVERAGE_COEFFICIENT * (1 << 11) };
uint32_t adc_zeros_debug[ADC_SAMPLES_PER_CYCLE][ADC_N_CHANNELS];
uint8_t pll_signal[ADC_SAMPLES_PER_CYCLE] =
{ [0 ... (ADC_SAMPLES_PER_CYCLE / 2 - 1)] = 0, [(ADC_SAMPLES_PER_CYCLE / 2) ... (ADC_SAMPLES_PER_CYCLE - 1)]=1 };
int32_t pll_diff = 0;
uint16_t pll_diff_arr[ADC_SAMPLES_PER_CYCLE];
int32_t pll_diff_avg;
uint8_t temp[40];
int16_t timer_period_diff;
uint16_t pll_locked;
uint8_t pll_locked_arr[ADC_SAMPLES_PER_CYCLE];
uint64_t adc_rms_accumulator[ADC_N_CHANNELS];
int64_t adc_active_power_accumulator[ADC_N_I_CHANNELS];
uint16_t adc_rms[ADC_N_CHANNELS];
int32_t adc_active_power[ADC_N_I_CHANNELS];
// ================================== asserts ==================================
const int64_t adc_rms_accumulator_max_assert = ADC_MAX_CODE * ADC_MAX_CODE * ADC_SAMPLES_PER_CYCLE;
const int32_t adc_zeros_max_assert = ADC_ZERO_AVERAGE_COEFFICIENT * ADC_MAX_CODE; // assert for maximum adc_zeros capacity - will throw warning if exceeded
const int32_t pll_diff_avg_max = PLL_XOR_GAIN*PLL_AVERAGE_COEFFICIENT*ADC_SAMPLES_PER_CYCLE-1;

const uint32_t adc_channels[ADC_NUMBER_OF_ADC][ADC_N_CHANNELS_PER_ADC] =
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

static HAL_StatusTypeDef ADC_start(ADC_HandleTypeDef* hadc, uint16_t *pData1, uint16_t *pData2, uint32_t Length);

void vAdcTask(void *pvParameters)
{
	uint16_t (*last_data)[ADC_N_CHANNELS] = NULL;
	BaseType_t status;
	int32_t timer_period_new = ADC_TIMER_PERIOD;
	bool xor;
	uint16_t pll_diff_min = UINT16_MAX;
	uint16_t pll_diff_max = 0;
	uint8_t pll_diff_cnt = 0;
	int32_t temp_int32;

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

	ADC_start(&hadc1, (uint16_t*) adc_data1, (uint16_t*) adc_data2, ADC_SAMPLE_BUFFER_LENGTH);
	while (1)
	{
		// ================================== receive new samples ==================================
		HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_RESET);
		status = xQueueReceive(ADC_queue, &last_data, 250);
		HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_SET);
		if (status)
		{
			// new ADC data available. ADC is storing new data in second buffer now
			// ================================== average cycles ==================================
			for (int j = 0; j < ADC_SAMPLES_PER_CYCLE; j++)
			{
				for (int i = 0; i < ADC_N_CHANNELS; i++)
				{
					adc_averaged_cycle[j][i] = (uint64_t) adc_averaged_cycle[j][i] * (ADC_AVERAGE_COEFFICIENT - 1)
							/ ADC_AVERAGE_COEFFICIENT + last_data[j][i];
					adc_zeros[i] = (uint64_t) adc_zeros[i] * (ADC_ZERO_AVERAGE_COEFFICIENT - 1) / ADC_ZERO_AVERAGE_COEFFICIENT
							+ last_data[j][i];
					adc_zeros_debug[j][i] = adc_zeros[i];
				}
			}
			// ================================== calculate RMS voltage/current ==================================
			for (int i = 0; i < ADC_N_CHANNELS; i++)
			{
				adc_rms_accumulator[i] = 0;
				for (int j = 0; j < ADC_SAMPLES_PER_CYCLE; j++)
				{
					temp_int32 = last_data[j][i] - adc_zeros[i] / ADC_ZERO_AVERAGE_COEFFICIENT;
					adc_rms_accumulator[i] += (uint64_t) ((int64_t) temp_int32 * (int64_t) temp_int32);
				}
				adc_rms[i] = (uint16_t) ((uint32_t) sqrtf((float) adc_rms_accumulator[i] / ADC_SAMPLES_PER_CYCLE));
			}
			// ================================== calculate active power ==================================
			for (int i = 0; i < ADC_N_I_CHANNELS; i++)
			{
				adc_active_power_accumulator[i] = 0;
				for (int j = 0; j < ADC_SAMPLES_PER_CYCLE; j++)
				{
					adc_active_power_accumulator[i] += (((int64_t) last_data[j][ADC_I_CHANNELS[i]]
							- adc_zeros[ADC_I_CHANNELS[i]] / ADC_ZERO_AVERAGE_COEFFICIENT)
							* ((int64_t) last_data[j][ADC_INDEX_LINE] - adc_zeros[ADC_INDEX_LINE] / ADC_ZERO_AVERAGE_COEFFICIENT));
				}
				adc_active_power[i] =
						(int32_t) (adc_active_power_accumulator[i] / ACTIVE_POWER_AVERAGE_COEFFICIENT / ADC_SAMPLES_PER_CYCLE
								+ (int64_t) adc_active_power[i] * (ACTIVE_POWER_AVERAGE_COEFFICIENT - 1)
										/ ACTIVE_POWER_AVERAGE_COEFFICIENT);
			}
			// ================================== PLL ==================================
			pll_diff_avg = 0;
			for (int j = 0; j < ADC_SAMPLES_PER_CYCLE; j++)
			{
				// exor of input signal and pll signal
				xor = (last_data[j][ADC_INDEX_LINE] > adc_zeros[ADC_INDEX_LINE] / ADC_ZERO_AVERAGE_COEFFICIENT)
						!= pll_signal[j];
				pll_diff = pll_diff * (PLL_AVERAGE_COEFFICIENT - 1) / PLL_AVERAGE_COEFFICIENT + xor * PLL_XOR_GAIN;
				if (pll_diff > PLL_AVERAGE_COEFFICIENT* PLL_XOR_GAIN)
				{
					pll_diff = PLL_AVERAGE_COEFFICIENT* PLL_XOR_GAIN;
				}
				if (pll_diff < 0)
				{
					pll_diff = 0;
				}
				pll_diff_arr[j] = pll_diff;
				// mean exor per cycle
				pll_diff_avg += pll_diff;
				// check pll
				if (pll_diff < pll_diff_min)
				{
					pll_diff_min = pll_diff;
				}
				if (pll_diff > pll_diff_max)
				{
					pll_diff_max = pll_diff;
				}
			}
			if (PLL_ON)
			{
				timer_period_new = ADC_TIMER_PERIOD_MIN
						+ pll_diff_avg * (ADC_TIMER_PERIOD_MAX - ADC_TIMER_PERIOD_MIN) / PLL_AVERAGE_COEFFICIENT
								/ ADC_SAMPLES_PER_CYCLE/ PLL_XOR_GAIN;
				change_adc_timer_period(timer_period_new);
				timer_period_diff = timer_period_new - ADC_TIMER_PERIOD;
			}
			// ================================== debug print ==================================
			//debug_print("%d\r\n", delta_avg);

			// ================================== check PLL lock status ==================================
			pll_diff_cnt++;
			if (pll_diff_cnt >= F_MAINS)
			{
				// ================================== debug print ==================================
				for (int j = 0; j < ADC_SAMPLES_PER_CYCLE; j++)
				{
					debug_print_push("%d,%d,%d\r\n",
							adc_averaged_cycle[j][ADC_INDEX_LINE] / ADC_AVERAGE_COEFFICIENT
									- adc_zeros[ADC_INDEX_LINE] / ADC_ZERO_AVERAGE_COEFFICIENT, pll_signal[j],
							//adc_rms[ADC_INDEX_LINE], adc_rms[ADC_INDEX_I8],
							pll_diff_avg);
				}
				debug_print_send();
				pll_locked = (pll_diff_max - pll_diff_min) < config_struct.pll_lock_threshold;
				pll_diff_cnt = 0;
				pll_diff_min = UINT16_MAX;
				pll_diff_max = 0;
			}
		}

	}
}

static HAL_StatusTypeDef ADC_start(ADC_HandleTypeDef* hadc, uint16_t *pData1, uint16_t *pData2, uint32_t Length)
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
		hadc->Init.NbrOfConversion = ADC_N_CHANNELS_PER_ADC;
		hadc->Init.DMAContinuousRequests = DISABLE;
		hadc->Init.EOCSelection = ADC_EOC_SEQ_CONV;

		if (HAL_ADC_Init(hadc) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
		 */
		for (uint_fast8_t ch = 0; ch < ADC_N_CHANNELS_PER_ADC; ch++)
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
	uint16_t *data = (uint16_t *) adc_data2;
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
			data = (uint16_t *) adc_data1;
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
