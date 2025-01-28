/*
 * UART.c
 *
 * Created: 7/20/2014 5:57:19 PM
 *  Author: Tobias
 */

// #define UART_ATMEGA8
// #define UART_ATMEGA328
#define UART_OFF

#include "UART.h"

#include <avr/io.h>
#include <util/delay.h>

#define BAUD 9600UL
#include <util/setbaud.h>


#if defined UART_ATMEGA8
void uart_init(void) {
	UBRRH = UBRRH_VALUE;
	UBRRL = UBRRL_VALUE;
	#if USE_2X
	/* U2X-Modus erforderlich */
	UCSRA |= (1 << U2X);
	#else
	/* U2X-Modus nicht erforderlich */
	UCSRA &= ~(1 << U2X);
	#endif
	UCSRB |= (1<<TXEN);  // UART TX einschalten
	UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);  // Asynchron 8N1
}

int uart_putc(unsigned char c) {
	while (!(UCSRA & (1<<UDRE))) { } /* warten bis Senden moeglich */
	UDR = c;                      /* sende Zeichen */
	return 0;
}

void uart_puts (char *s) {
	while (*s) {   
		uart_putc(*s);
		s++;
	}
}
#elif defined UART_ATMEGA328
// UART initialization
void uart_init(void) {
    // Set baud rate registers
    UBRR0H = UBRRH_VALUE;   // Set high byte of baud rate
    UBRR0L = UBRRL_VALUE;   // Set low byte of baud rate

    // Set frame format (8 data bits, no parity, 1 stop bit)
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8 data bits, 1 stop bit
    
    // Enable transmitter
    UCSR0B = (1 << TXEN0);  // Enable transmitter
}

// Send a single character via UART
int uart_putc(unsigned char c) {
    // Wait for the transmit buffer to be empty
    while (!(UCSR0A & (1 << UDRE0))) { }  
    // Put the character into the transmit buffer
    UDR0 = c;               
    return 0;
}

// Send a string of characters via UART
void uart_puts(char *s) {
    while (*s) {
        uart_putc(*s);  // Send each character in the string
        s++;
    }
}
#elif defined UART_OFF
// UART initialization
void uart_init(void) {

}

// Send a single character via UART
int uart_putc(unsigned char c) {
    return 0;
}

// Send a string of characters via UART
void uart_puts(char *s) {

}
#endif