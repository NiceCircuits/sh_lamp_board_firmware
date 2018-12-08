/*
 * config.h
 *
 *  Created on: Nov 24, 2018
 *      Author: 74hc0
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <inttypes.h>

enum
{
	N_GROUPS = 8, N_STATES = 256, N_TRANSITIONS = 4192, N_OUTPUTS = 8
};

typedef struct
{
	uint32_t sw_version;
	uint32_t hw_version;
	uint8_t id;
} info_struct_t;

typedef struct
{
	uint16_t output_mask;
	uint16_t state_table_index;
} group_info_t;

typedef struct
{
	uint16_t output_states;
	uint16_t transition_table_index;
} state_info_t;

typedef struct
{
	uint8_t device_id;
	uint8_t input_id;
	uint8_t messsage_type;
	uint8_t next_state;
} transistion_info_t;

typedef struct
{
	group_info_t group_table[N_GROUPS];
	state_info_t state_table[N_STATES];
	transistion_info_t transition_table[N_TRANSITIONS];
} config_struct_t;

extern const config_struct_t config_struct;
extern const info_struct_t info_struct;

#endif /* CONFIG_H_ */
