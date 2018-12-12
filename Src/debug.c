/*
 * debug.c
 *
 *  Created on: Nov 1, 2018
 *      Author: 74hc0
 */

#include <stdlib.h>
#include <stdarg.h>
#include "usart.h"
#include "debug.h"
#include "can.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"

static char debug_print_buffer[DEBUG_PRINT_BUFFER_SIZE];

void debug_print(const char* format, ...)
{
	va_list args;
	int count;
	va_start(args, format);
	count = vsnprintf(debug_print_buffer, (size_t) DEBUG_PRINT_BUFFER_SIZE, format, args);
	HAL_UART_Transmit(&huart1, (uint8_t*) debug_print_buffer, (uint16_t) count, 1000);
	va_end(args);
}

void vDebugTask(void *pvParameters)
{
	uint8_t cnt = 0;
	while (1)
	{
		osDelay(1000);
//		CAN_send(0x100, 1, &cnt);
//		cnt++;
	}
}
