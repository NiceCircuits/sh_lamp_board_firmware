/**
 ******************************************************************************
 * File Name          : USART.c
 * Description        : This file provides code for the configuration
 *                      of the USART instances.
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
#include "usart.h"

#include "gpio.h"

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_tx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

	huart1.Instance = USART1;
	huart1.Init.BaudRate = 921600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}
/* USART6 init function */

void MX_USART6_UART_Init(void)
{

	huart6.Instance = USART6;
	huart6.Init.BaudRate = 115200;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart6) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

	GPIO_InitTypeDef GPIO_InitStruct;
	if (uartHandle->Instance == USART1)
	{
		/* USART1 clock enable */
		__HAL_RCC_USART1_CLK_ENABLE()
		;

		/**USART1 GPIO Configuration
		 PA9     ------> USART1_TX
		 PA10     ------> USART1_RX
		 */
		GPIO_InitStruct.Pin = DEBUG_TX_Pin | DEBUG_RX_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* USART1 DMA Init */
		/* USART1_TX Init */
		hdma_usart1_tx.Instance = DMA2_Stream7;
		hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
		hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_usart1_tx.Init.Mode = DMA_NORMAL;
		hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
		hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		__HAL_LINKDMA(uartHandle, hdmatx, hdma_usart1_tx);

	}
	else if (uartHandle->Instance == USART6)
	{
		/* USART6 clock enable */
		__HAL_RCC_USART6_CLK_ENABLE()
		;

		/**USART6 GPIO Configuration
		 PC6     ------> USART6_TX
		 PC7     ------> USART6_RX
		 */
		GPIO_InitStruct.Pin = IO_TX_Pin | IO_RX_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	}
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

	if (uartHandle->Instance == USART1)
	{
		/* Peripheral clock disable */
		__HAL_RCC_USART1_CLK_DISABLE();

		/**USART1 GPIO Configuration
		 PA9     ------> USART1_TX
		 PA10     ------> USART1_RX
		 */
		HAL_GPIO_DeInit(GPIOA, DEBUG_TX_Pin | DEBUG_RX_Pin);

	}
	else if (uartHandle->Instance == USART6)
	{
		/* Peripheral clock disable */
		__HAL_RCC_USART6_CLK_DISABLE();

		/**USART6 GPIO Configuration
		 PC6     ------> USART6_TX
		 PC7     ------> USART6_RX
		 */
		HAL_GPIO_DeInit(GPIOC, IO_TX_Pin | IO_RX_Pin);

	}
}

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
