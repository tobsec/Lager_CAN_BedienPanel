/*
 * MCP2515.c
 *
 * Created: 7/20/2014 6:09:39 PM
 *  Author: Tobias
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include "MCP2515.h"
#include "mcp2515_defs.h"
#include "UART.h"

#include "config.h"


void spi_init(void)
{
	// Aktivieren der Pins für das SPI Interface
	DDR_SPI  |= (1<<P_SCK)|(1<<P_MOSI);
	PORT_SPI &= ~((1<<P_SCK)|(1<<P_MOSI)|(1<<P_MISO));
	
	DDR_CS   |= (1<<P_CS);
	PORT_CS  |= (1<<P_CS);
	
	// Aktivieren des SPI Master Interfaces, fosc = fclk / 2
	SPCR = (1<<SPE)|(1<<MSTR);
	SPSR = (1<<SPI2X);
}

uint8_t spi_putc( uint8_t data )
{
	// Sendet ein Byte
	SPDR = data;
	
	// Wartet bis Byte gesendet wurde
	while( !( SPSR & (1<<SPIF) ) )
	;
	
	return SPDR;
}

void mcp2515_write_register( uint8_t adress, uint8_t data )
{
	// /CS des MCP2515 auf Low ziehen
	PORT_CS &= ~(1<<P_CS);
	
	spi_putc(SPI_WRITE);
	spi_putc(adress);
	spi_putc(data);
	
	// /CS Leitung wieder freigeben
	PORT_CS |= (1<<P_CS);
}

uint8_t mcp2515_read_register(uint8_t adress)
{
	uint8_t data;
	
	// /CS des MCP2515 auf Low ziehen
	PORT_CS &= ~(1<<P_CS);
	
	spi_putc(SPI_READ);
	spi_putc(adress);
	
	data = spi_putc(0xff);
	
	// /CS Leitung wieder freigeben
	PORT_CS |= (1<<P_CS);
	
	return data;
}

void mcp2515_bit_modify(uint8_t adress, uint8_t mask, uint8_t data)
{
	// /CS des MCP2515 auf Low ziehen
	PORT_CS &= ~(1<<P_CS);
	
	spi_putc(SPI_BIT_MODIFY);
	spi_putc(adress);
	spi_putc(mask);
	spi_putc(data);
	
	// /CS Leitung wieder freigeben
	PORT_CS |= (1<<P_CS);
}

uint8_t mcp2515_read_status(uint8_t type)
{
	uint8_t data;
	
	PORT_CS &= ~(1<<P_CS);
	
	spi_putc(type);
	data = spi_putc(0xff);
	
	PORT_CS |= (1<<P_CS);
	
	return data;
}

void mcp2515_init(void)
{
    // SPI Interface initialisieren
    spi_init();
   
    // MCP2515 per Software Reset zuruecksetzten,
    // danach ist der MCP2515 im Configuration Mode
    PORT_CS &= ~(1<<P_CS);
    spi_putc( SPI_RESET );
    _delay_ms(1);
    PORT_CS |= (1<<P_CS);
   
    // etwas warten bis sich der MCP2515 zurueckgesetzt hat
    _delay_ms(10);
   
    /*
     *  Einstellen des Bit Timings
     * 
     *  Fosc       = 16MHz
     *  BRP        = 7                (teilen durch 8)
     *  TQ = 2 * (BRP + 1) / Fosc  (=> 1 uS)
     * 
     *  Sync Seg   = 1TQ
     *  Prop Seg   = (PRSEG + 1) * TQ  = 1 TQ
     *  Phase Seg1 = (PHSEG1 + 1) * TQ = 3 TQ
     *  Phase Seg2 = (PHSEG2 + 1) * TQ = 3 TQ
     * 
     *  Bus speed  = 1 / (Total # of TQ) * TQ
     *             = 1 / 8 * TQ = 125 kHz
     */
   
#if defined MCP_CLOCK_16MHZ
    // BRP = 7
    mcp2515_write_register( CNF1, (1<<BRP0)|(1<<BRP1)|(1<<BRP2) );
#elif defined MCP_CLOCK_8MHZ
    // BRP = 3
    mcp2515_write_register( CNF1, (1<<BRP0)|(1<<BRP1) );
#else
#error Specify clock settings!
#endif

    // Prop Seg und Phase Seg1 einstellen
    mcp2515_write_register( CNF2, (1<<BTLMODE)|(1<<PHSEG11) );
   
    // Wake-up Filter deaktivieren, Phase Seg2 einstellen
    mcp2515_write_register( CNF3, (1<<PHSEG21) );
   
    // Aktivieren der Rx Buffer Interrupts
    mcp2515_write_register( CANINTE, (1<<RX1IE)|(1<<RX0IE) );

#if defined MCP_CLOCKOUT_ENABLE
   // CLKOUT aktivieren, Prescaler System Clock/2
    mcp2515_bit_modify(CANCTRL, (1<<CLKEN)|(1<<CLKPRE1)|(1<<CLKPRE0), (1<<CLKEN)|(1<<CLKPRE0)); 
   
   _delay_ms(100); // Sicherheitshalber warten bis CLKOUT stabil
#endif
   		
#if defined MCP_CLOCK_16MHZ
   if (mcp2515_read_register(CNF1) == ((1<<BRP2)|(1<<BRP1)|(1<<BRP0))) {
	   uart_puts("MCP2515 initialize\r\n");
   } else uart_puts("ERROR!\r\n");
#elif defined MCP_CLOCK_8MHZ
   if (mcp2515_read_register(CNF1) == ((1<<BRP1)|(1<<BRP0))) {
	   uart_puts("MCP2515 initialize\r\n");
   } else uart_puts("ERROR!\r\n");
#endif

    /*
     *  Einstellen der Filter
     *
   
   RXM: Receive Buffer Operating Mode bits
   11 = Turn mask/filters off; receive any message
   10 = Receive only valid messages with extended identifiers that meet filter criteria
   -> 01 = Receive only valid messages with standard identifiers that meet filter criteria
   00 = Receive all valid messages using either standard or extended identifiers that meet filter criteria
     */
    // Buffer 0 : Empfangen aller Nachrichten
    mcp2515_write_register( RXB0CTRL, (0<<RXM1)|(1<<RXM0) );
   
    // Buffer 1 : Empfangen aller Nachrichten
    mcp2515_write_register( RXB1CTRL, (0<<RXM1)|(1<<RXM0) );
   
    // Alle Bits der Empfangsmaske loeschen,
    // damit werden alle Nachrichten empfangen
    mcp2515_write_register( RXM0SIDH, 0 );
    mcp2515_write_register( RXM0SIDL, 0 );
    mcp2515_write_register( RXM0EID8, 0 );
    mcp2515_write_register( RXM0EID0, 0 );
    mcp2515_write_register( RXF0SIDH, 0 );
    mcp2515_write_register( RXF0SIDL, 0 );
   
    mcp2515_write_register( RXM1SIDH, 0xFF );
    mcp2515_write_register( RXM1SIDL, 0xE0 );
    mcp2515_write_register( RXM1EID8, 0 );
    mcp2515_write_register( RXM1EID0, 0 );
   
    /*
     *  Einstellen der Pin Funktionen
     */
   
    // Deaktivieren der Pins RXnBF Pins (High Impedance State)
	mcp2515_write_register( BFPCTRL, 0);
    //mcp2515_write_register( BFPCTRL, (1<<B0BFE)|(1<<B1BFE)|(1<<B0BFM)|(B1BFM) ); // RX0BF und RX1BF zeigen volle Puffer an
	
	// RX0BF als Digitalen Ausgang nutzen
	mcp2515_bit_modify( BFPCTRL, (1<<B0BFE)|(1<<B0BFM), (1<<B0BFE));
   
    // TXnRTS Bits als Inputs schalten
    mcp2515_write_register( TXRTSCTRL, 0 );
   
    // Device zurueck in den normalen Modus versetzten
    mcp2515_bit_modify( CANCTRL, 0xE0, 0);
}

uint8_t mcp2515_read_rx_status(void)
{
	uint8_t data;
	
	// /CS des MCP2515 auf Low ziehen
	PORT_CS &= ~(1<<P_CS);
	
	spi_putc(SPI_RX_STATUS);
	data = spi_putc(0xff);
	
	// Die Daten werden noch einmal wiederholt gesendet,
	// man braucht also nur eins der beiden Bytes auswerten.
	spi_putc(0xff);
	
	// /CS Leitung wieder freigeben
	PORT_CS |= (1<<P_CS);
	
	return data;
}

