/*
 * outputs.h
 *
 *  Created on: Nov 25, 2018
 *      Author: 74hc0
 */

#ifndef OUTPUTS_H_
#define OUTPUTS_H_

#include "FreeRTOS.h"

void vOutputsTask(void *pvParameters);

extern SemaphoreHandle_t outputs_sync_semaphore;
extern QueueHandle_t message_queue;

enum
{
	OUTPUT_SYNCHRONIZATION_DELAY_MAX = 100,
	MESSAGE_QUEUE_LENGTH = 128
};

#endif /* OUTPUTS_H_ */
