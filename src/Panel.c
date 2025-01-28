/*
 * Panel.c
 *
 * Created: 7/23/2014 7:55:55 PM
 *  Author: Tobias
 */ 

#define F_CPU 8000000UL

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include "Panel.h"
#include "PanelLEDdefs.h"

void set_adr(uint8_t c) {
	// DDRB |= (1<<PORTB0); // A0
	// DDRB |= (1<<PORTB1); // A1
	// DDRD |= (1<<PORTD2); // A2
	if (c<=7) {
		PORTB = (PORTB&0b11111100) | (c&0b00000011);
		PORTD = (PORTD&0b11111011) | (c&0b00000100);
	}
}

void setLedBar(uint8_t number, uint8_t intense) {
	// Number 0|1|2|3
	// Intense 0=0% 1-25=10% 26-50=20% 51-75=30% 76-100=40% 101-125=50% 126-150=60% 151-175=70% 176-200=80% 201-225=90% 226-255=100%
	uint8_t temp=0;
	if (number<4) { // rechtes 4 Kanal Panel
		for (uint8_t i=0; i<=7; i++) {
			if ((intense>(i*25))) temp |= (0b00000001<<i);
		}
		leds[number] = temp;
	
		temp=0;
		if (intense>200) temp |= 0b00000001; // 90%
		if (intense>225) temp |= 0b00000010; // 100%
	
		leds[4] = ((leds[4]&(~(0b00000011<<(number*2))) | (temp<<(number*2))));
	} else if (number==4) { // linkes 1 Kanal
		for (uint8_t i=0; i<=7; i++) {
			if ((intense>(i*25))) temp |= (0b00000001<<i);
		}
		leds2[7] = temp;
		
		temp=0;
		if (intense>200) temp |= 0b00000001; // 90%
		if (intense>225) temp |= 0b00000010; // 100%
		
		leds2[5] = leds2[5]&(~(0b00000011)) | (temp);
	}		
}

void initPanel() {
	DDRD |= (1<<PORTD5)|(1<<PORTD6)|(1<<PORTD7); // PD5 - SerialData | PD6 - LatchClock | PD7 - ShiftClock
	DDRB |= (1<<PORTB0); // A0
	DDRB |= (1<<PORTB1); // A1
	DDRD |= (1<<PORTD2); // A2
	
	set_adr(1);
	data_out(0);

}

void data_out(uint8_t out) { // Ausgabe ins Schieberegister
	for (uint8_t i=0; i<=7; i++) {
		if (out & 0x80) PORTD |= (1<<PORTD5);
		else PORTD &= ~(1<<PORTD5);
		//_delay_us(1);
		PORTD |= (1<<PORTD7);
		//_delay_us(1);
		PORTD &= ~(1<<PORTD7);
		out = (out<<1);
		//_delay_us(1);
	}
	PORTD |= (1<<PORTD6);
	//_delay_us(1);
	PORTD &= ~(1<<PORTD6);
}

void setPanelLed(uint8_t led, uint8_t adr) {
	if ((adr&0xF0)==0) {
		leds[(adr&0x0F)] |= led;
	} else {
		leds2[(adr&0x0F)] |= led;
	}
}

void clearPanelLed(uint8_t led, uint8_t adr) {
	if ((adr&0xF0)==0) {
		leds[(adr&0x0F)] &= ~led;
	} else {
		leds2[(adr&0x0F)] &= ~led;
	}
}

void clearAllSceneLeds() {
	leds2[6]=0x00;
	leds2[5] &= ~(0b11000000);
}

void setPanelUpDownOnOffLeds(uint8_t outID, uint8_t value) {
	if (outID==0) {
		if (value==0) {
			clearPanelLed(LED_P3_CH1_ONOFF, LED_P3_CH1_ONOFF_ADR);
			clearPanelLed(LED_P3_CH1_UP, LED_P3_CH1_UP_ADR);
			clearPanelLed(LED_P3_CH1_DOWN,LED_P3_CH1_DOWN_ADR);
		} else {
			setPanelLed(LED_P3_CH1_ONOFF,LED_P3_CH1_ONOFF_ADR);
			if (value<255) setPanelLed(LED_P3_CH1_UP,LED_P3_CH1_UP_ADR);
			else clearPanelLed(LED_P3_CH1_UP,LED_P3_CH1_UP_ADR);
			if (value>15) setPanelLed(LED_P3_CH1_DOWN,LED_P3_CH1_DOWN_ADR);
			else clearPanelLed(LED_P3_CH1_DOWN,LED_P3_CH1_DOWN_ADR);
		}
	} else if (outID==1) {
		if (value==0) {
			clearPanelLed(LED_P3_CH2_ONOFF, LED_P3_CH2_ONOFF_ADR);
			clearPanelLed(LED_P3_CH2_UP, LED_P3_CH2_UP_ADR);
			clearPanelLed(LED_P3_CH2_DOWN,LED_P3_CH2_DOWN_ADR);
		} else {
			setPanelLed(LED_P3_CH2_ONOFF,LED_P3_CH2_ONOFF_ADR);
			if (value<255) setPanelLed(LED_P3_CH2_UP,LED_P3_CH2_UP_ADR);
			else clearPanelLed(LED_P3_CH2_UP,LED_P3_CH2_UP_ADR);
			if (value>15) setPanelLed(LED_P3_CH2_DOWN,LED_P3_CH2_DOWN_ADR);
			else clearPanelLed(LED_P3_CH2_DOWN,LED_P3_CH2_DOWN_ADR);
		}			
	} else if (outID==2) {
		if (value==0) {
			clearPanelLed(LED_P3_CH3_ONOFF, LED_P3_CH3_ONOFF_ADR);
			clearPanelLed(LED_P3_CH3_UP, LED_P3_CH3_UP_ADR);
			clearPanelLed(LED_P3_CH3_DOWN,LED_P3_CH3_DOWN_ADR);
		} else {
			setPanelLed(LED_P3_CH3_ONOFF,LED_P3_CH3_ONOFF_ADR);
			if (value<255) setPanelLed(LED_P3_CH3_UP,LED_P3_CH3_UP_ADR);
			else clearPanelLed(LED_P3_CH3_UP,LED_P3_CH3_UP_ADR);
			if (value>15) setPanelLed(LED_P3_CH3_DOWN,LED_P3_CH3_DOWN_ADR);
			else clearPanelLed(LED_P3_CH3_DOWN,LED_P3_CH3_DOWN_ADR);
		}			
	} else if (outID==3) {
		if (value==0) {
			clearPanelLed(LED_P3_CH4_ONOFF, LED_P3_CH4_ONOFF_ADR);
			clearPanelLed(LED_P3_CH4_UP, LED_P3_CH4_UP_ADR);
			clearPanelLed(LED_P3_CH4_DOWN,LED_P3_CH4_DOWN_ADR);
		} else {
			setPanelLed(LED_P3_CH4_ONOFF,LED_P3_CH4_ONOFF_ADR);
			if (value<255) setPanelLed(LED_P3_CH4_UP,LED_P3_CH4_UP_ADR);
			else clearPanelLed(LED_P3_CH4_UP,LED_P3_CH4_UP_ADR);
			if (value>15) setPanelLed(LED_P3_CH4_DOWN,LED_P3_CH4_DOWN_ADR);
			else clearPanelLed(LED_P3_CH4_DOWN,LED_P3_CH4_DOWN_ADR);
		}
		
	} else if (outID==4) {
		if (value==0) {
			clearPanelLed(LED_P3_CH5_ONOFF, LED_P3_CH5_ONOFF_ADR);
			clearPanelLed(LED_P3_CH5_UP, LED_P3_CH5_UP_ADR);
			clearPanelLed(LED_P3_CH5_DOWN,LED_P3_CH5_DOWN_ADR);
		} else {
			setPanelLed(LED_P3_CH5_ONOFF,LED_P3_CH5_ONOFF_ADR);
			if (value<255) setPanelLed(LED_P3_CH5_UP,LED_P3_CH5_UP_ADR);
			else clearPanelLed(LED_P3_CH5_UP,LED_P3_CH5_UP_ADR);
			if (value>15) setPanelLed(LED_P3_CH5_DOWN,LED_P3_CH5_DOWN_ADR);
			else clearPanelLed(LED_P3_CH5_DOWN,LED_P3_CH5_DOWN_ADR);
		}
	
	}
	
}
