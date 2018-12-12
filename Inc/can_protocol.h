/*
 * protocol.h
 *
 *  Created on: Nov 17, 2018
 *      Author: 74hc0
 */

#ifndef CAN_PROTOCOL_H_
#define CAN_PROTOCOL_H_

/**
 * CAN protocol
 *
 * Message types are defined by their ID
 * - ID = 0x000..0x0FF - input/output events (button press, output change etc.). ID = module IP
 * - ID = 0x100..0x1FF - internal state of module. Send every n seconds. ID = module IP + 0x100
 * - ID = 0x200..0x2FF - outputs state of module. Send every n seconds. ID = module IP + 0x200
 * - ID = 0x600..0x6FF - reserved for module configuration change
 * - ID = 0x700..0x7FF - reserved for bootloader
 *
 * Input/output event frame
 * ID = 0x000..0x0FF = module IP
 * Data0 = input/output number:
 * 		0x00..0x1F - digital inputs
 * 		0x20..0x3F - digital outputs
 * 		0x40..0x5F - digital output groups
 */
enum
{
	EVENT_FRAME_ID_OFFSET = 0,
	EVENT_FRAME_ID_FILTER = (0 << 21) // STDID - bits 8..10 must be 0
	|(0<<3) // EXTID - not important
	|(0<<2) // IDE - must be 0
	|(0<<1), // RTR - must be 0
	EVENT_FRAME_ID_MASK = (0x700 << 21) // STDID - bits 8..10 must match
	|(0<<3) // EXTID - not important
	|(1<<2) // IDE - must be 0
	|(1<<1), // RTR - must be 0
	EVENT_FRAME_DIN_NUMBER_OFFSET = 0,
	EVENT_FRAME_DOUT_NUMBER_OFFSET = 0x20,
	EVENT_FRAME_DGROUP_NUMBER_OFFSET = 0x40
};

typedef enum
{
	I_MESSAGE_INVALID, // message ID 0 is invalid
	I_MESSAGE_BUTTON_PUSH, // sent immediately after button is pushed
	I_MESSAGE_BUTTON_RELEASE, // sent immediately after button is released
	I_MESSAGE_BUTTON_SHORT_PRESS, // sent after button is pressed and released after less than TODO ms
	I_MESSAGE_BUTTON_LONG_PRESS, // sent after button is pressed and released after at least TODO ms
	I_MESSAGE_BUTTON_SECOND_PUSH, // sent after button is pressed, released and pressed again less than TODO ms after release
	I_MESSAGE_BUTTON_DOUBLE_SHORT_PRESS, // sent after button is twice for less than TODO ms with less than TODO ms time between first release and second press
	I_MESSAGE_BUTTON_DOUBLE_LONG_PRESS,
	I_MESSAGE_BUTTON_LONG_PLUS_SHORT_PRESS,
	I_MESSAGE_BUTTON_SHORT_PLUS_LONG_PRESS
} input_message_t;

typedef enum
{
	O_MESSAGE_INVALID, // message ID 0 is invalid
	O_MESSAGE_OUTPUT_IS_ON,
	O_MESSAGE_OUTPUT_IS_OFF,
//	O_MESSAGE_OUTPUT_IS,
//	O_MESSAGE_OUTPUT_VALUE_IS,
//	O_MESSAGE_SET_OUTPUT_ON,
//	O_MESSAGE_SET_OUTPUT_OFF,
//	O_MESSAGE_SET_OUTPUT_STATE,
//	O_MESSAGE_SET_OUTPUT_VALUE
} output_message_t;

typedef enum
{
	OG_MESSAGE_INVALID, // message ID 0 is invalid
	OG_MESSAGE_GROUP_IS_ON,
	OG_MESSAGE_GROUP_IS_OFF,
} output_group_message_t;

#endif /* CAN_PROTOCOL_H_ */
