#include <avr/io.h>

void initTimer()
{
// Configure Timer 0 to overflow at approximately 2016.1 Hz
// using a prescaler of 64 and preload value of 194.

// Timer Configuration:
// - F_CPU = 8 MHz (internal clock)
// - Prescaler = 64 (TCCR0B = (1 << CS01) | (1 << CS00))
// - Timer width = 8-bit (counts from 0 to 255)

// Preload Value:
// - We start the timer at TCNT0 = 194, so the timer will count from 194 to 255.
// - Number of ticks before overflow = 256 - 194 = 62 ticks

// Timer Tick Period:
// - The timer frequency with prescaler 64 is: 
//   f_timer = F_CPU / prescaler = 8,000,000 / 64 = 125,000 Hz
//   Thus, each tick takes 1 / 125,000 = 8 microseconds (8 µs).

// Overflow Time Calculation:
// - The time for one overflow is the number of ticks multiplied by the tick period:
//   overflow_time = 62 ticks * 8 µs = 496 µs.

// Overflow Frequency:
// - The overflow frequency is the reciprocal of the overflow time:
//   overflow_frequency = 1 / 496 µs ≈ 2,016.1 Hz.
#if defined(__AVR_ATmega8__)
    // Configuration for ATmega8
    TCCR0 = (1 << CS01) | (1 << CS00); // Prescaler 64
    TCNT0 = 194;                       // Preload for 1ms (71Hz)
    TIMSK |= (1 << TOIE0);              // Enable Overflow Interrupt

#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328PB__)
    // Configuration for ATmega328P
    TCCR0A = 0;
    TCCR0B = (1 << CS01) | (1 << CS00); // Prescaler 64
    TCNT0 = 194;                        // Preload for 1ms
    TIMSK0 = (1 << TOIE0);              // Enable Overflow Interrupt

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