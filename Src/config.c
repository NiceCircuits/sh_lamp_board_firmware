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
	.group_table ={
			{1, 0},
			{2, 2},
			{4, 4},
			{8, 6},
			{16, 8},
			{32, 10},
			{64, 12},
			{128, 14}
	},
	.state_table ={
			{0, 0},
			{1, 1},
			{0, 2},
			{2, 3},
			{0, 4},
			{4, 5},
			{0, 6},
			{8, 7},
			{0, 8},
			{16, 9},
			{0, 10},
			{32, 11},
			{0, 12},
			{64, 13},
			{0, 14},
			{128, 15},
			[16 ... (N_STATES - 1)] = {0xFFFF, 0xFFFF}
	},
	.transition_table ={
			{1, 0, 1, 1},
			{1, 0, 1, 0},
			{1, 1, 1, 1},
			{1, 1, 1, 0},
			{1, 2, 1, 1},
			{1, 2, 1, 0},
			{1, 3, 1, 1},
			{1, 3, 1, 0},
			{1, 4, 1, 1},
			{1, 4, 1, 0},
			{1, 5, 1, 1},
			{1, 5, 1, 0},
			{1, 6, 1, 1},
			{1, 6, 1, 0},
			{1, 7, 1, 1},
			{1, 7, 1, 0},
			[16 ... (N_TRANSITIONS - 1)] = {0xFF, 0xFF, 0xFF, 0xFF}
	}
};

