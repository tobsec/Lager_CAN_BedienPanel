#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "CAN.h"
#include "mcp2515_defs.h"
#include "UART.h"
#include "MCP2515.h"
#include "avr_hwAb.h"

#include "LagerLight_protocol.h"
#include "config.h"

volatile uint8_t buf_rx = 0u;
volatile uint8_t buf_tx = 0u;

volatile uint8_t bufTx_rx = 0u;
volatile uint8_t bufTx_tx = 0u;



static CANMessage receivedMsg[RXBUF_LEN]; // Puffer f�r eingegangene CAN Messages
static CANMessage txMsg[TXBUF_LEN]; // Puffer f�r ausgehende Messages

void copyMsg(CANMessage *in, CANMessage *out)
{
	out->id.bit = in->id.bit;
	out->id.destID = in->id.destID;
	out->id.srcID = in->id.srcID;
	out->length = in->length;
	out->rtr = in->rtr;
	out->arrivalTime = in->arrivalTime;
	for (uint8_t i = 0u; i < in->length; i++)
	{
		out->data[i] = in->data[i];
	}
}

void printMessage(CANMessage * msg)
{
	char temp[10];
	uart_puts("srcID: ");
	itoa(msg->id.srcID, temp, 10);
	uart_puts(temp);
	uart_puts("desID: ");
	itoa(msg->id.destID, temp, 10);
	uart_puts(temp);
	uart_puts("bit: ");
	itoa(msg->id.bit, temp, 10);
	uart_puts(temp);
	uart_puts(" Length: ");
	itoa(msg->length, temp, 10);
	uart_puts(temp);
	uart_puts(" RTR: ");
	itoa(msg->rtr, temp, 10);
	uart_puts(temp);
	uart_puts(" Data: ");
	for (uint8_t i=0; i<msg->length; i++) {
		itoa(msg->data[i], temp, 16);
		uart_putc('(');
		uart_putc(msg->data[i]);
		uart_putc(')');
		uart_puts(temp);
		uart_putc('|');
	}
	uart_puts("\r\n");
}

uint8_t can_get_message(CANMessage *p_message)
{
	// Status auslesen
	uint8_t status = mcp2515_read_rx_status();

	if (bit_is_set(status, 6))
	{
		// Nachricht in Puffer 0

		PORT_CS &= ~(1 << P_CS); // CS Low
		spi_putc(SPI_READ_RX);
	}
	else if (bit_is_set(status, 7))
	{
		// Nachricht in Puffer 1

		PORT_CS &= ~(1 << P_CS); // CS Low
		spi_putc(SPI_READ_RX | 0x04u);
		// uart_puts("Puffer1hatMsg\r\n");
	}
	else
	{
		/* Fehler: Keine neue Nachricht vorhanden */
		PORT_CS |= (1 << P_CS);
		return 0xFFu;
	}

	// Standard ID auslesen
	uint16_t idTemp = 0u;
	idTemp = (uint16_t)(spi_putc(0xFFu) << 3u);
	idTemp |= (uint16_t)(spi_putc(0xFFu) >> 5u);

	p_message->id.srcID = GET_SRC_ID(idTemp);
	p_message->id.destID = GET_DEST_ID(idTemp);
	p_message->id.bit = GET_BIT(idTemp);

	spi_putc(0xFFu); // EXTENDED IDENTIFIER HIGH
	spi_putc(0xFFu); // EXTENDED IDENTIFIER LOW

	// Laenge auslesen
	uint8_t length = spi_putc(0xFFu) & 0x0Fu;
	p_message->length = length;

	if (length > 8u)
		length = 8u;

	// Daten auslesen
	for (uint8_t i = 0u; i < length; i++)
	{
		p_message->data[i] = spi_putc(0xFFu);
	}

	PORT_CS |= (1 << P_CS);

	if (bit_is_set(status, 3))
	{
		p_message->rtr = 1u;
	}
	else
	{
		p_message->rtr = 0u;
	}

	p_message->arrivalTime = 0u;

	// Interrupt Flag loeschen
	if (bit_is_set(status, 6))
	{
		mcp2515_bit_modify(CANINTF, (1 << RX0IF), 0u);
	}
	else
	{
		mcp2515_bit_modify(CANINTF, (1 << RX1IF), 0u);
	}

	return (status & 0x07u); // Return Filter Match (S.67)
}

uint8_t can_send_message(CANMessage *p_message)
{
	uint8_t status, address;

	// Status des MCP2515 auslesen
	PORT_CS &= ~(1 << P_CS);
	spi_putc(SPI_READ_STATUS);
	status = spi_putc(0xff);
	spi_putc(0xff);
	PORT_CS |= (1 << P_CS);

	/* Statusbyte:
	 *
	 * Bit  Funktion
	 *  2   TXB0CNTRL.TXREQ
	 *  4   TXB1CNTRL.TXREQ
	 *  6   TXB2CNTRL.TXREQ
	 */

	if (bit_is_clear(status, 2))
	{
		address = 0x00;
	}
	else if (bit_is_clear(status, 4))
	{
		address = 0x02;
	}
	else if (bit_is_clear(status, 6))
	{
		address = 0x04;
	}
	else
	{
		/* Alle Puffer sind belegt,
		   Nachricht kann nicht verschickt werden */
		return 0;
	}

	PORT_CS &= ~(1 << P_CS); // CS Low
	spi_putc(SPI_WRITE_TX | address);

	// Standard ID einstellen
#ifdef debug
	char temp[16];
	uart_puts("ID_HIGH: ");
	itoa((uint8_t)((p_message->id.srcID << 3) | (p_message->id.destID >> 2)), temp, 2);
	uart_puts(temp);

	uart_puts("ID_LOW: ");
	itoa((uint8_t)((p_message->id.destID << 6) | (p_message->id.bit << 5)), temp, 2);
	uart_puts(temp);
#endif
	spi_putc(ENCODE_CANID_HIGHBYTE(p_message->id.srcID, p_message->id.destID));
	spi_putc(ENCODE_CANID_LOWBYTE(p_message->id.destID, p_message->id.bit));

	// Extended ID
	spi_putc(0x00);
	spi_putc(0x00);

	uint8_t length = p_message->length;

	if (length > 8)
	{
		length = 8;
	}

	// Ist die Nachricht ein "Remote Transmit Request" ?
	if (p_message->rtr)
	{
		/* Ein RTR hat zwar eine Laenge,
		   aber enthaelt keine Daten */

		// Nachrichten Laenge + RTR einstellen
		spi_putc((1 << RTR) | length);
	}
	else
	{
		// Nachrichten Laenge einstellen
		spi_putc(length);

		// Daten
		for (uint8_t i = 0; i < length; i++)
		{
			spi_putc(p_message->data[i]);
		}
	}
	PORT_CS |= (1 << P_CS); // CS auf High

	asm volatile("nop");

	/* CAN Nachricht verschicken
	   die letzten drei Bit im RTS Kommando geben an welcher
	   Puffer gesendet werden soll */
	PORT_CS &= ~(1 << P_CS); // CS wieder Low
	if (address == 0x00)
	{
		spi_putc(SPI_RTS | 0x01);
	}
	else
	{
		spi_putc(SPI_RTS | address);
	}
	PORT_CS |= (1 << P_CS); // CS auf High

	return 1;
}

void addMessageToBuffer(CANMessage * msg)
{
	copyMsg(msg, &receivedMsg[buf_tx]);
	if (buf_tx >= (RXBUF_LEN - 1u))
	{
		buf_tx = 0u;
	}
	else
	{
		buf_tx++;
	}
}

CANMessage can_getMessageFromBuffer()
{
	CANMessage msg = { 
		.id = { .srcID = 0u, .destID = 0u, .bit = 0u },
		.rtr = 0u,
		.length = 0u,
		.arrivalTime = 0u,
		.data = { 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u }
		};

	cli(); // Critical section, accessing buffers
	if (buf_tx != buf_rx)
	{
		copyMsg(&receivedMsg[buf_rx], &msg);
		if (buf_rx >= (RXBUF_LEN - 1u))
		{
			buf_rx = 0u;
		}
		else
		{
			buf_rx++;
		}
	}
	sei(); // End of critical section
	return msg;
}

void can_addMessageToTxBuffer(CANMessage *msg)
{
	copyMsg(msg, &txMsg[bufTx_tx]);
	if (bufTx_tx >= 24u)
	{
		bufTx_tx = 0u;
	}
	else
	{
		bufTx_tx++;
	}
}

CANMessage getMessageFromTxBuffer()
{
	CANMessage msg = { 
		.id = { .srcID = 0u, .destID = 0u, .bit = 0u },
		.rtr = 0u,
		.length = 0u,
		.arrivalTime = 0u,
		.data = { 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u }
		};

	if (bufTx_tx != bufTx_rx)
	{
		copyMsg(&txMsg[bufTx_rx], &msg);
		if (bufTx_rx >= 24u)
		{
			bufTx_rx=0u;
		}
		else
		{
			bufTx_rx++;
		}
	}
	return msg;
}

void can_sendBufferedMessage(CANMessage *p_message)
{
    if (can_send_message(p_message) == 0u)
    {
        can_addMessageToTxBuffer(p_message);
    }
}

void can_loop()
{
	CANMessage msg;

#ifndef CAN_USE_INTERRUPTS
	// Polling Mode: Check if MCP2515 INT pin (PD3) is LOW
	if ((PIND & (1 << PIND3)) == 0u)
	{
		if (can_get_message(&msg) != 0xFFu)
		{
			addMessageToBuffer(&msg);
		}
	}
#endif

    msg = getMessageFromTxBuffer();
	if (msg.id.destID != 0)
	{
        can_sendBufferedMessage(&msg); // Alle Puffer voll-> Nachricht wieder im FIFO einreihen
	}
}

void can_init()
{
    mcp2515_init();

#ifdef CAN_USE_INTERRUPTS
    initInterrupt(); // Enable external interrupt for CAN reception
#else
    DDRD &= ~(1 << PORTD3);  // Set PD3 as input
    PORTD |= (1 << PORTD3);  // Enable pull-up
#endif

    memset(receivedMsg, 0, sizeof(receivedMsg));
    memset(txMsg, 0, sizeof(txMsg));
}

#ifdef CAN_USE_INTERRUPTS
ISR(INT1_vect)
{
	CANMessage msg;
	while (can_get_message(&msg) != 0xFFu)
	{
		addMessageToBuffer(&msg);
	}
}
#endif
