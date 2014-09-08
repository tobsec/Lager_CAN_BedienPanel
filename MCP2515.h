/*
 * MCP2515.h
 *
 * Created: 7/20/2014 6:09:49 PM
 *  Author: Tobias
 */ 


#ifndef MCP2515_H_
#define MCP2515_H_

	#define DDR_CS      DDRB
	#define PORT_CS     PORTB
	#define P_CS        2

	#define DDR_SPI     DDRB
	#define PORT_SPI    PORTB
	#define P_MISO      4
	#define P_MOSI      3
	#define P_SCK       5

	#define R_CNF1  (1<<BRP2)|(1<<BRP1)|(1<<BRP0)
	#define R_CNF2  (1<<BTLMODE)|(1<<PHSEG11)
	#define R_CNF3  (1<<PHSEG21)

	

	void spi_init(void);

	uint8_t spi_putc( uint8_t data );

	void mcp2515_write_register( uint8_t adress, uint8_t data );

	uint8_t mcp2515_read_register(uint8_t adress);

	void mcp2515_bit_modify(uint8_t adress, uint8_t mask, uint8_t data);

	uint8_t mcp2515_read_status(uint8_t type);

	void mcp2515_init(void);
	
	uint8_t mcp2515_read_rx_status(void);

#endif /* MCP2515_H_ */