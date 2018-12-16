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

#if DEBUG_ENABLE
static char debug_print_buffer[2][DEBUG_PRINT_BUFFER_SIZE];
static char* debug_active_buffer = debug_print_buffer[0];
static size_t debug_buffer_index = 0;

void debug_print(const char* format, ...)
{
	va_list args;
	int count;
	va_start(args, format);
	while (huart1.gState != HAL_UART_STATE_READY)
	{
	}
	count = vsnprintf(debug_print_buffer, (size_t) DEBUG_PRINT_BUFFER_SIZE, format, args);
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*) debug_print_buffer, (uint16_t) count);
	va_end(args);
}

void debug_print_push(const char* format, ...)
{
	va_list args;
	int count;
	va_start(args, format);
	if (debug_buffer_index < DEBUG_PRINT_BUFFER_SIZE)
	{
		count = vsnprintf(debug_active_buffer + debug_buffer_index, (size_t) DEBUG_PRINT_BUFFER_SIZE - debug_buffer_index,
				format, args);
		debug_buffer_index += count;
	}
	va_end(args);
}

void debug_print_send()
{
	while (huart1.gState != HAL_UART_STATE_READY)
	{
	}
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*) debug_active_buffer, (uint16_t) debug_buffer_index);
	if (debug_active_buffer == debug_print_buffer[0])
	{
		debug_active_buffer = debug_print_buffer[1];
	}
	else
	{
		debug_active_buffer = debug_print_buffer[0];
	}
	debug_buffer_index=0;
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
#else
void debug_print(const char* format, ...)
{
	UNUSED(format);
}

void debug_print_push(const char* format, ...)
{
	UNUSED(format);
}

void debug_print_send()
{

}

void vDebugTask(void *pvParameters)
{
	while (1)
	{
		osDelay(10000);
	}
}
#endif
