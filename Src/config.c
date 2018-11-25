/*
 * config.c
 *
 *  Created on: Nov 24, 2018
 *      Author: 74hc0
 */

#include "config.h"

__attribute__((section(".flash_info"))) const info_struct_t info_struct =
{ .id = 0xFF };

__attribute__((section(".flash_data"))) const config_struct_t config_struct =
{
	.group_table ={ [0 ... (N_GROUPS - 1)] = {0xFFFF} },
	.state_table ={ [0 ... (N_STATES - 1)] = {0xFFFF} },
	.transition_table ={ [0 ... (N_TRANSITIONS - 1)] = {0xFF} }
};

