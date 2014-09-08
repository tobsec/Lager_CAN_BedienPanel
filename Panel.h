/*
 * Panel.h
 *
 * Created: 7/23/2014 7:56:07 PM
 *  Author: Tobias
 */ 


#ifndef PANEL_H_
#define PANEL_H_

static uint8_t channel;
volatile uint8_t leds[8];
volatile uint8_t leds2[8];

// Panel 1000 CPU


#define F_CPU 8000000UL

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>

void set_adr(uint8_t c);

void setLedBar(uint8_t number, uint8_t intense); 
void setPanelUpDownOnOffLeds(uint8_t outID, uint8_t value);
void setPanelLed(uint8_t led, uint8_t adr);
void clearPanelLed(uint8_t led, uint8_t adr);
void clearAllSceneLeds();
void initPanel();

void data_out(uint8_t out);

#endif /* PANEL_H_ */