/*
 * debug.h
 *
 *  Created on: Nov 1, 2018
 *      Author: 74hc0
 */

#ifndef DEBUG_H_
#define DEBUG_H_

/* Variable declarations ------------------------------------------------------------------*/
/* Constants definitions ------------------------------------------------------------------*/
enum
{
	DEBUG_PRINT_BUFFER_SIZE = 1024
};
/* Function declaration ------------------------------------------------------------------*/
void debug_print(const char* format, ...);
void debug_print_push(const char* format, ...);
void debug_print_send();
void vDebugTask(void *pvParameters);

#endif /* DEBUG_H_ */
