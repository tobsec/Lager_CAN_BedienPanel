#ifndef CAN_H
#define CAN_H

/**
 * @brief CAN Message ID structure
 * 
 * The CAN Message ID contains the following fields:
 * - **srcID**: 5-bit source address of the device sending the message.
 * - **destID**: 5-bit destination address of the device intended to receive the message.
 * - **bit**: The least significant bit is reserved for unused data or flags.
 */
typedef struct
{
	uint8_t srcID;
	uint8_t destID;
	uint8_t bit;
} CANMessageID;

/**
 * @brief CAN Message structure
 * 
 * The CAN Message includes the message ID (source, destination, and flags), along
 * with the following data:
 * - **rtr**: Remote Transmission Request (used to request data from a device)
 * - **length**: Length of the data field (0 to 8 bytes)
 * - **arrivalTime**: The timestamp when the message was received (unused)
 * - **data[8]**: Array of up to 8 bytes of data that may contain actions and associated data.
 */
typedef struct
{
	CANMessageID  id;
	uint8_t   rtr;
	uint8_t   length;
	uint16_t arrivalTime;
	uint8_t   data[8u];
} CANMessage;

void can_addMessageToTxBuffer(CANMessage *msg);
void can_sendBufferedMessage(CANMessage *p_message);
CANMessage can_getMessageFromBuffer();
void can_loop();
void can_init();

#endif