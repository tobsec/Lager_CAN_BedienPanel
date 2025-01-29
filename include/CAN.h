#ifndef CAN_H
#define CAN_H

typedef struct
{
	uint8_t srcID;
	uint8_t destID;
	uint8_t bit;
} CANMessageID;

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
void can_txLoop();
void can_init();

#endif