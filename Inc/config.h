/*
 * config.h
 *
 *  Created on: Nov 24, 2018
 *      Author: 74hc0
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <inttypes.h>

// output configuration
enum
{
	N_GROUPS = 8, N_STATES = 256, N_TRANSITIONS = 4192, N_OUTPUTS = 8
};

// ADC configuration
enum
{
	F_APB1 = 100000000, // default APB1 frequency
	F_MAINS = 50, // mains voltage frequency
	ADC_SAMPLES_PER_CYCLE = 64, // samples per one cycle of mains
	ADC_SAMPLERATE = F_MAINS * ADC_SAMPLES_PER_CYCLE, // ADC smple rate
	ADC_TIMER_PERIOD = F_APB1 / ADC_SAMPLERATE - 1, // starting period of ADC clocking timer
	ADC_TIMER_PERIOD_MIN = ADC_TIMER_PERIOD * 99 / 100, // PLL change limits
	ADC_TIMER_PERIOD_MAX = ADC_TIMER_PERIOD * 101 / 100, // PLL change limits
};

typedef struct
{
	uint32_t sw_version;
	uint32_t hw_version;
	uint8_t ip;
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
