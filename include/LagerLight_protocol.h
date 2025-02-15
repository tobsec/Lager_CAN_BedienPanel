#ifndef LAGERLIGHT_PROTOCOL_H
#define LAGERLIGHT_PROTOCOL_H


/**
 * @brief Action Mask and Shift Definitions
 * 
 * The action is encoded in the highest 3 bits of the first data byte:
 * - ACTION_MASK: 0b11100000 (mask for extracting the action)
 * - ACTION_SHIFT: 5 (shifts the action bits to the least significant position)
 * 
 * The actions are defined by the `actionType_e` enum, which includes various commands
 * such as setting, requesting, incrementing, and toggling values.
 */
#define ACTION_MASK (0b11100000u)
#define ACTION_SHIFT 5u

/**
 * @brief Extract Action from Data Byte
 * 
 * The action is encoded in the highest 3 bits of the first data byte.
 * The function `GET_ACTION` extracts the action from the byte and casts it to
 * the `actionType_e` enum.
 */
#define GET_ACTION(dataByte) ((actionType_e)((dataByte & ACTION_MASK) >> ACTION_SHIFT))

/**
 * @brief Set Action in Data Byte
 * 
 * The `SET_ACTION` function allows setting the action in the highest 3 bits
 * of the data byte by shifting the action value into the correct position.
 */
#define SET_ACTION(dataByte, action) (dataByte |= (uint8_t)((action << ACTION_SHIFT) & ACTION_MASK))

/**
 * @brief Extract Output ID from Data Byte
 * 
 * The `GET_OUTID` macro extracts the lower 5 bits of the data byte to retrieve
 * the Output ID. The Output ID is typically used to identify which device or output
 * the action refers to.
 */
#define OUTID_MASK (0b00011111u)

#define GET_OUTID(dataByte) ((uint8_t)(dataByte & OUTID_MASK))

/**
 * @brief Action Type Enum
 * 
 * The action types are encoded in the highest 3 bits of the first data byte:
 * - **ACTION_REQUEST_VALUE**: Request a value from a device.
 * - **ACTION_SET_VALUE**: Set a value in the device.
 * - **ACTION_INC_VALUE**: Increment the value.
 * - **ACTION_DEC_VALUE**: Decrement the value.
 * - **ACTION_TOGGLE_OUTPUT**: Toggle an output state.
 * - **ACTION_RESET**: Reset the destination device.
 * - **ACTION_UNUSED_06**: Reserved for future use.
 * - **ACTION_RESPONSE**: Response to a request.
 */
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

/**
 * @brief Extracts the source ID and destination ID from the CAN ID
 * 
 * The CAN ID is an 11-bit value, and the source and destination addresses
 * are extracted using bitwise operations as follows:
 * - **srcID**: Extracted from bits 10-6 (5 bits)
 * - **destID**: Extracted from bits 5-1 (5 bits)
 * - **bit**: Extracted from the least significant bit (bit 0)
 */
#define GET_SRC_ID(id) ((uint8_t)((id & 0b0000011111000000) >> 6u))
#define GET_DEST_ID(id) ((uint8_t)((id & 0b0000000000111110) >> 1u))
#define GET_BIT(id) ((uint8_t)((id & 0x0001)))

/**
 * @brief Macros to encode the CAN Message ID for transmission
 * 
 * These macros are used to encode the source ID, destination ID, and unused bit into
 * two bytes to be transmitted via SPI. The message ID is 11 bits long and includes:
 * - 5 bits for the source ID
 * - 5 bits for the destination ID
 * - 1 unused bit
 * 
 * The message encoding is split into two bytes:
 * - **Byte 1**: Contains the source ID (bits 3-7) and the destination ID (bits 0-2).
 * - **Byte 2**: Contains the destination ID (bits 6-7) and the unused bit (bit 5).
 */
#define ENCODE_CANID_HIGHBYTE(srcID, destID) ((uint8_t)((srcID << 3) | (destID >> 2)))
#define ENCODE_CANID_LOWBYTE(destID, bit) ((uint8_t)((destID << 6) | (bit << 5)))


#endif