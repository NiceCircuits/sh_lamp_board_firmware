/**
 ******************************************************************************
 * File Name          : CAN.c
 * Description        : This file provides code for the configuration
 *                      of the CAN instances.
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
#include "can.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "debug.h"
#include "outputs.h"
#include "config.h"
#include "can_protocol.h"
#include <string.h>

typedef struct
{
	CAN_RxHeaderTypeDef header;
	uint8_t data[8];
} CAN_RX_frame_t;

typedef struct
{
	CAN_TxHeaderTypeDef header;
	uint8_t data[8];
} CAN_TX_frame_t;

CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
uint32_t TxMailbox;

QueueHandle_t CAN_RX_queue;
QueueHandle_t CAN_TX_queue;
static StaticQueue_t CAN_RX_static_queue_buffer, CAN_TX_static_queue_buffer;
CAN_RX_frame_t CAN_RX_static_queue_data_buffer[CAN_RX_QUEUE_LENGTH];
CAN_TX_frame_t CAN_TX_static_queue_data_buffer[CAN_TX_QUEUE_LENGTH];
CAN_RX_frame_t CAN_RX_frame;
CAN_TX_frame_t CAN_TX_frame;
transistion_info_t message;

void vCanTask(void *pvParameters)
{
	BaseType_t status;
	CAN_FilterTypeDef sFilterConfig;

	CAN_RX_queue = xQueueCreateStatic(CAN_RX_QUEUE_LENGTH, sizeof(CAN_RX_frame_t),
			(uint8_t*) CAN_RX_static_queue_data_buffer, &CAN_RX_static_queue_buffer);
	CAN_TX_queue = xQueueCreateStatic(CAN_TX_QUEUE_LENGTH, sizeof(CAN_TX_frame_t),
			(uint8_t*) CAN_TX_static_queue_data_buffer, &CAN_TX_static_queue_buffer);

	// config filtering for module
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = (EVENT_FRAME_ID_FILTER >> 16) & 0xFFFF;
	sFilterConfig.FilterIdLow = EVENT_FRAME_ID_FILTER & 0xFFFF;
	sFilterConfig.FilterMaskIdHigh = (EVENT_FRAME_ID_MASK >> 16) & 0xFFFF;
	sFilterConfig.FilterMaskIdLow = EVENT_FRAME_ID_MASK & 0xFFFF;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
	{
		/* Filter configuration Error */
		Error_Handler();
	}

	// Start the CAN peripheral
	if (HAL_CAN_Start(&hcan1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		Error_Handler();
	}

	while (1)
	{

		// receive new messages
		status = xQueueReceive(CAN_RX_queue, &CAN_RX_frame, 0);
		if (status)
		{
			debug_print("CAN recv: 0x%lX, 0x%lX, 0x%lX, 0x%lX:", CAN_RX_frame.header.StdId, CAN_RX_frame.header.ExtId,
					CAN_RX_frame.header.IDE, CAN_RX_frame.header.RTR);
			for (int i = 0; i < CAN_RX_frame.header.DLC; i++)
			{
				debug_print(" %lX", CAN_RX_frame.data[i]);
			}
			debug_print("\r\n");
			if (CAN_RX_frame.header.DLC == 2)
			{
				message.device_id = (uint8_t) (CAN_RX_frame.header.StdId);
				message.input_id = CAN_RX_frame.data[0];
				message.messsage_type = CAN_RX_frame.data[1];
				status = xQueueSend(output_control_message_queue, &message, 0);
				if (status != pdTRUE)
				{
					Error_Handler();
				}
			}
		}
		// transmit messages
		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1))
		{
			status = xQueueReceive(CAN_TX_queue, &CAN_TX_frame, 0);
			if (status)
			{
				if (HAL_CAN_AddTxMessage(&hcan1, &(CAN_TX_frame.header), CAN_TX_frame.data, &TxMailbox) != HAL_OK)
				{
					Error_Handler();
				}
				else
				{
					debug_print("CAN send: 0x%lX, 0x%lX, 0x%lX, 0x%lX:", CAN_TX_frame.header.StdId, CAN_TX_frame.header.ExtId,
							CAN_TX_frame.header.IDE, CAN_TX_frame.header.RTR);
					for (int i = 0; i < CAN_TX_frame.header.DLC; i++)
					{
						debug_print(" %lX", CAN_TX_frame.data[i]);
					}
					debug_print("\r\n");

				}
			}
		}
	}
}

BaseType_t CAN_send(uint16_t ID, uint8_t data_length, uint8_t* data)
{
	assert_param(data_length <= 8);
	assert_param(ID <= 0x800);
	assert_param(data!=NULL);

	CAN_TX_frame.header.IDE = 0;
	CAN_TX_frame.header.RTR = 0;
	CAN_TX_frame.header.ExtId = 0;
	CAN_TX_frame.header.StdId = ID;
	CAN_TX_frame.header.DLC = data_length;
	memcpy(CAN_TX_frame.data, data, data_length);
	return xQueueSend(CAN_TX_queue, &CAN_TX_frame, 0);
}

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 25;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_6TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = ENABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

	GPIO_InitTypeDef GPIO_InitStruct;
	if (canHandle->Instance == CAN1)
	{
		/* CAN1 clock enable */
		__HAL_RCC_CAN1_CLK_ENABLE()
		;

		/**CAN1 GPIO Configuration
		 PA11     ------> CAN1_RX
		 PA12     ------> CAN1_TX
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* CAN1 interrupt Init */
		HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

	}
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

	if (canHandle->Instance == CAN1)
	{
		/* Peripheral clock disable */
		__HAL_RCC_CAN1_CLK_DISABLE();

		/**CAN1 GPIO Configuration
		 PA11     ------> CAN1_RX
		 PA12     ------> CAN1_TX
		 */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);

	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RX_frame_t RX;
	BaseType_t xHigherPriorityTaskWoken;

	/* Get RX message */
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &(RX.header), RX.data) != HAL_OK)
	{
		/* Reception Error */
		Error_Handler();
	}
	xQueueSendFromISR(CAN_RX_queue, &RX, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
