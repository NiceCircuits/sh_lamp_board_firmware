/*
 * outputs.c
 *
 *  Created on: Nov 25, 2018
 *      Author: 74hc0
 */

#include "gpio.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "debug.h"
#include "outputs.h"
#include "config.h"
#include "main.h"
#include <stdbool.h>

static uint8_t group_states[N_GROUPS];

static GPIO_PinState output_states[N_OUTPUTS];

static const GPIO_TypeDef * output_ports[N_OUTPUTS] =
{ REL1_OUT_GPIO_Port, REL2_OUT_GPIO_Port, REL3_OUT_GPIO_Port, REL4_OUT_GPIO_Port, REL5_OUT_GPIO_Port,
REL6_OUT_GPIO_Port, REL7_OUT_GPIO_Port, REL8_OUT_GPIO_Port };

static const uint16_t output_pins[N_OUTPUTS] =
{ REL1_OUT_Pin, REL2_OUT_Pin, REL3_OUT_Pin, REL4_OUT_Pin, REL5_OUT_Pin, REL6_OUT_Pin, REL7_OUT_Pin, REL8_OUT_Pin };

SemaphoreHandle_t outputs_sync_semaphore;

QueueHandle_t output_control_message_queue;
static StaticQueue_t message_static_queue_buffer;
transistion_info_t message_static_queue_data_buffer[MESSAGE_QUEUE_LENGTH];
transistion_info_t message;

void vOutputsTask(void *pvParameters)
{
	uint16_t state_table_index, state_table_index_next, transition_table_index, transition_table_index_next;
	const transistion_info_t *checked_message;
	outputs_sync_semaphore = xSemaphoreCreateBinary();

	if (outputs_sync_semaphore == NULL)
	{
		Error_Handler();
	}
	output_control_message_queue = xQueueCreateStatic(MESSAGE_QUEUE_LENGTH, sizeof(transistion_info_t),
			(uint8_t*) message_static_queue_data_buffer, &message_static_queue_buffer);
	if (output_control_message_queue == NULL)
	{
		Error_Handler();
	}
	while (1)
	{

		while (xQueueReceive(output_control_message_queue, &message, 0))
		{
			HAL_GPIO_TogglePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin);
			// check if state change needed
			for (uint8_t i = 0; i < N_GROUPS; i++)
			{
				state_table_index = config_struct.group_table[i].state_table_index;
				state_table_index_next = config_struct.group_table[i + 1].state_table_index;
				if (state_table_index != 0xFFFF)
				{
					if (((state_table_index + group_states[i]) >= state_table_index_next) && (i != (N_GROUPS - 1)))
					{
						Error_Handler();
					}
					transition_table_index =
							config_struct.state_table[state_table_index + group_states[i]].transition_table_index;
					transition_table_index_next =
							config_struct.state_table[state_table_index + group_states[i] + 1].transition_table_index;
					for (uint16_t j = transition_table_index; j < transition_table_index_next; j++)
					{
						checked_message = &(config_struct.transition_table[j]);
						if ((message.device_id == checked_message->device_id) && (message.input_id == checked_message->input_id)
								&& (message.messsage_type == checked_message->messsage_type))
						{
							// message fits transition - change state
							group_states[i] = checked_message->next_state;
							// Calculate desired output states
							for (uint8_t k = 0; k < N_OUTPUTS; k++)
							{
								// check if output is affected by group
								if (((config_struct.group_table[i].output_mask) >> k) & 1)
								{
									// set output state
									output_states[k] = ((config_struct.state_table[state_table_index + group_states[i]].output_states)
											>> k) & 1;
								}
							}
							break;
						}
					}
				}
				else
				{
					// end of table
					break;
				}
			}
		}

		// Synchronize outputs switching with mains zero
		xSemaphoreTake(outputs_sync_semaphore, OUTPUT_SYNCHRONIZATION_DELAY_MAX);
		// change output states
		for (uint8_t i = 0; i < N_OUTPUTS; i++)
		{
			HAL_GPIO_WritePin((GPIO_TypeDef *) output_ports[i], (uint16_t) output_pins[i], output_states[i]);
		}
	}
}
