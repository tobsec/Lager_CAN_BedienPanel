#ifndef LAGERLIGHT_PROTOCOL_H
#define LAGERLIGHT_PROTOCOL_H

#define ACTION_MASK (0b11100000u)
#define ACTION_SHIFT 5u

#define GET_ACTION(dataByte) ((actionType_e)((dataByte & ACTION_MASK) >> ACTION_SHIFT))
#define SET_ACTION(dataByte, action) (dataByte |= (uint8_t)((action << ACTION_SHIFT) & ACTION_MASK))

#define OUTID_MASK (0b00011111u)

#define GET_OUTID(dataByte) ((uint8_t)(dataByte & OUTID_MASK))

typedef enum
{
	ACTION_REQUEST_VALUE = 0x00u,
	ACTION_SET_VALUE     = 0x01u,
	ACTION_INC_VALUE     = 0x02u,
	ACTION_DEC_VALUE     = 0x03u,
	ACTION_TOGGLE_OUTPUT = 0x04u,
	ACTION_RESET         = 0x05u,
	ACTION_UNUSED_06     = 0x06u,
	ACTION_RESPONSE      = 0x07u
} actionType_e;

#endif