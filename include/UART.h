/*
 * UART.h
 *
 * Created: 7/20/2014 5:58:17 PM
 *  Author: Tobias
 */ 


#ifndef UART_H_
#define UART_H_



void uart_init(void);
int uart_putc(unsigned char c);
void uart_puts (char *s);


#endif /* UART_H_ */