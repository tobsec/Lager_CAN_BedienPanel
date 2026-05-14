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

// Bus-off / TX-stall recovery. We previously re-queued every failed TX
// forever — when the bus actually went bus-off, the MCP2515 stops ACKing
// and every send returns "all buffers busy". The retry loop then pinned
// the firmware (panel felt frozen). Now we count consecutive failures
// and, after CAN_TX_FAIL_THRESHOLD strikes, force a controller re-init
// so we don't sit idle waiting for the MCP2515's internal 128*11
// recessive-bit recovery to fire.
#ifndef CAN_TX_FAIL_THRESHOLD
#define CAN_TX_FAIL_THRESHOLD 50u
#endif
static uint8_t consecutiveTxFails = 0u;

// EFLG.TXB0 (bit 5) is the "Transmit Error — Bus Off" flag. We poll it
// cheaply from can_loop() at ~1/256 invocations so we catch a hard
// bus-off even when the application has no traffic to push.
static uint8_t busOffPollDiv = 0u;

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
	// Drop-newest on overflow: if advancing the write pointer would
	// catch the read pointer the buffer is full and we'd silently
	// clobber an unread message. Better to lose the newest frame than
	// to corrupt the queue and confuse the consumer.
	uint8_t next = (buf_tx >= (RXBUF_LEN - 1u)) ? 0u : (uint8_t)(buf_tx + 1u);
	if (next == buf_rx)
	{
		return;
	}
	copyMsg(msg, &receivedMsg[buf_tx]);
	buf_tx = next;
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
	// Same drop-newest discipline as the RX buffer. Previously this
	// used a hard-coded wrap at 24 (the TXBUF_LEN was 32 but the bound
	// was wrong) and never checked for full — overruns silently
	// overwrote messages the consumer hadn't seen yet.
	uint8_t next = (bufTx_tx >= (TXBUF_LEN - 1u)) ? 0u : (uint8_t)(bufTx_tx + 1u);
	if (next == bufTx_rx)
	{
		return;
	}
	copyMsg(msg, &txMsg[bufTx_tx]);
	bufTx_tx = next;
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
		if (bufTx_rx >= (TXBUF_LEN - 1u))
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
        // All three MCP2515 TX buffers report busy. Normally transient
        // (arbitration backoff). When the bus is healthy this clears
        // within microseconds; consecutiveTxFails goes back to 0 next
        // call. When the bus is bus-off the MCP2515 never finishes any
        // pending TX, so every call hits this branch.
        consecutiveTxFails++;
        if (consecutiveTxFails >= CAN_TX_FAIL_THRESHOLD)
        {
            uart_puts("CAN TX stall - reinit\r\n");
            can_init();
            // can_init wipes buffers and zeros consecutiveTxFails.
            // Don't re-queue this frame — let the app drive new traffic.
            return;
        }
        can_addMessageToTxBuffer(p_message);
    }
    else
    {
        consecutiveTxFails = 0u;
    }
}

void can_loop()
{
	CANMessage msg;

	// Cheap bus-off poll. EFLG.TXB0 (bit 5) latches when the controller
	// has crossed the 256-error bus-off threshold. Hitting it means the
	// MCP2515 won't TX anything until either the silicon's own recovery
	// fires (128 occurrences of 11 recessive bits) or we re-init it.
	busOffPollDiv++;
	if (busOffPollDiv == 0u)
	{
		uint8_t eflg = mcp2515_read_register(EFLG);
		if (eflg & (1u << TXB0))
		{
			uart_puts("CAN bus-off - reinit\r\n");
			can_init();
		}
	}

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

    // Reset SW state too — if we were called from a recovery path the
    // old ring-buffer pointers reference stale slots that are about to
    // be zeroed out, and consecutiveTxFails is what triggered us.
    uint8_t sreg = SREG;
    cli();
    buf_rx = 0u;
    buf_tx = 0u;
    bufTx_rx = 0u;
    bufTx_tx = 0u;
    consecutiveTxFails = 0u;
    busOffPollDiv = 0u;
    memset(receivedMsg, 0, sizeof(receivedMsg));
    memset(txMsg, 0, sizeof(txMsg));
    SREG = sreg;
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
