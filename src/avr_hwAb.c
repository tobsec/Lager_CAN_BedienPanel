#include <avr/io.h>

void initTimer()
{
#if defined(__AVR_ATmega8__)
    // Configuration for ATmega8
    TCCR0 = (1 << CS01) | (1 << CS00); // Prescaler 64
    TCNT0 = 194;                       // Preload for 1ms (71Hz)
    TIMSK |= (1 << TOIE0);              // Enable Overflow Interrupt

#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__)
    // Configuration for ATmega328P
    TCCR0B = (1 << CS01) | (1 << CS00); // Prescaler 64
    TCNT0 = 194;                        // Preload for 1ms
    TIMSK0 |= (1 << TOIE0);              // Enable Overflow Interrupt

#else
    #error "Unsupported MCU! Define ATmega8 or ATmega328P."
#endif
}

void initInterrupt()
{
#if defined(__AVR_ATmega8__)
    // Configuration for ATmega8
    DDRD &= ~(1 << PORTD3);     // Set PD3 as input
    PORTD |= (1 << PORTD3);     // Enable pull-up
    GICR |= (1 << INT1);        // Enable INT1
    MCUCR = (0 << ISC11) | (0 << ISC10); // Low level trigger

#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__)
    // Configuration for ATmega328P
    DDRD &= ~(1 << PORTD3);     // Set PD3 as input
    PORTD |= (1 << PORTD3);     // Enable pull-up
    EIMSK |= (1 << INT1);       // Enable INT1
    EICRA = (0 << ISC11) | (0 << ISC10); // Low level trigger

#else
    #error "Unsupported MCU! Define ATmega8 or ATmega328P."
#endif
}