/*
 * ATmega8_MCP2515.c
 *
 * Created: 7/19/2014 7:29:49 PM
 *  Author: Tobias
 */ 

#define F_CPU 8000000UL

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#include <avr/wdt.h>

#include "mcp2515_defs.h"
//#include "UART.h"
#include "MCP2515.h"
#include "Panel.h"
#include "PanelLEDdefs.h"
#include "LagerLight_protocol.h"

#define MCU_ID_EEPROM_ADDR 256
#define MCU_ID eeprom_read_word((uint16_t*) MCU_ID_EEPROM_ADDR)


typedef struct  
{
	uint8_t srcID;
	uint8_t destID;
	uint8_t bit;
} CANMessageID;

typedef struct
{
	CANMessageID  id;
	uint8_t   rtr;
	uint8_t   length;
	uint16_t arrivalTime;
	uint8_t   data[8];
} CANMessage;

static uint8_t hjg=0;
static uint8_t hjh=0;

uint8_t switches[8];
uint8_t switchesHold[8];

uint8_t value = 128;
uint8_t value2 = 0;

uint8_t buf_rx = 0;
uint8_t buf_tx = 0;

uint8_t bufTx_rx = 0;
uint8_t bufTx_tx = 0;
	
static uint8_t ticks=0;
static uint16_t time=0; // Wird alle 10ms hochgez�hlt 
static CANMessage receivedMsg[25]; // Puffer f�r eingegangene CAN Messages
static CANMessage txMsg[25]; // Puffer f�r ausgehende Messages
static uint8_t outputValues[5]={0,0,0,0,0};
static uint8_t programmLightSzenzeMode=0;
static uint8_t saveActualValues=0;

uint8_t eeByteArray1[10][5] EEMEM = {	{255,255,255,255,255}, // 5 Kan�le auf 10 Speicherpl�tzen
										{0,0,0,0,0},
										{128,128,128,128,128},
										{0,0,0,0,0},
										{0,0,0,0,0},
										{0,0,0,0,0},
										{0,0,0,0,0},
										{0,0,0,0,0},
										{0,0,0,0,0},
										{0,0,0,0,0}
}; 
	
//const uint8_t thisID = 21; // eindeutige ID von diesem Ger�t (0+31 sind reserviert)
#define thisID (uint8_t) 21

CANMessage defaultMsgToLightActor11 = {
	{thisID,	//srcID
	11,			//destID
	0},			//IDbit
	0,			//RTR
	1,			//Length muss bei opt. Wert ge�ndert werden
	0,			//ArrivalTime
	0			//Data
};

void initTimer() {
	TCCR0 = (1<<CS01) | (1<<CS00); // Takteiler 64 vorher 8
	TCNT0 = 194; // Preload 131 => T=1ms --> 6x1ms => T=0,06s => f=71Hz vorher 6
	TIMSK  |= (1<<TOIE0);  // Overflow Interrupt erlauben
}

void initInterrupt() {
	DDRD &= ~(1<<PORTD3);		// Set PD3 as input (Using for interrupt INT1)
	PORTD |= (1<<PORTD3);		// Enable PD3 pull-up resistor
	GICR = (1<<INT1);					// Enable INT1
	MCUCR = (0<<ISC11) | (0<<ISC10);	// Low-Level generates Interrupt Request ( /* Trigger INT1 on falling edge */ )
}

/*
void addMessage(CANMessage * p_msg) {
	if (p_msg->id<25) {
		copyMsg(p_msg, &receivedMsg[p_msg->id]);
		//printMessage(&receivedMsg[p_msg->id]);
		receivedMsg[p_msg->id].arrivalTime = time;
	}
	//uint8_t addMessage(CANMessage * p_msg)
	//{
		//uint8_t i=0;
		//while (receivedMsg[i].id != 0) {
			//i++;
			//if (i==50) break;
		//}
		//if (i<50) {
			////receivedMsg[i] = (CANMessage *) malloc(sizeof(CANMessage));
			//copyMsg(p_msg, &receivedMsg[i]);
			//receivedMsg[i].arrivalTime = time;
			////copyMsg(p_msg, receivedMsg[i]);
			//return 0;
		//}
		//return 1;
	//}
}*/
/*
CANMessage * searchMessage(void) {
	uint8_t i=0;
	while (receivedMsg[i].id == 0) {
		i++;
		if (i==25) break;
	}
	if (i<25) {
		return &receivedMsg[i];
		//free(receivedMsg[i]);
	} else return 0;
}*/

/*
CANMessage * searchMessageID(uint16_t id) {
	uint8_t i=0;
	while (receivedMsg[i].id != id) {
		i++;
		if (i==25) break;
	}
	if (i<25) {
		return &receivedMsg[i];
	} else return 0;
}*/

void copyMsg(CANMessage * in, CANMessage * out) {
	out->id.bit = in->id.bit;
	out->id.destID = in->id.destID;
	out->id.srcID = in->id.srcID;
	out->length = in->length;
	out->rtr = in->rtr;
	out->arrivalTime = in->arrivalTime;
	for (uint8_t i=0; i<in->length; i++)
	{
		out->data[i]=in->data[i];
	}
}
/*
void printMessage(CANMessage * msg) {
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
}*/

uint8_t can_get_message(CANMessage *p_message)
{
	// Status auslesen
	uint8_t status = mcp2515_read_rx_status();
	
	if (bit_is_set(status,6))
	{
		// Nachricht in Puffer 0
		
		PORT_CS &= ~(1<<P_CS);    // CS Low
		spi_putc(SPI_READ_RX);
	}
	else if (bit_is_set(status,7))
	{
		// Nachricht in Puffer 1
		
		PORT_CS &= ~(1<<P_CS);    // CS Low
		spi_putc(SPI_READ_RX | 0x04);
		//uart_puts("Puffer1hatMsg\r\n");
	}
	else {
		/* Fehler: Keine neue Nachricht vorhanden */
		PORT_CS |= (1<<P_CS);
		return 0xff;
	}
	
	// Standard ID auslesen
	uint16_t idTemp = 0;
	idTemp =  (uint16_t) spi_putc(0xff) << 3;
	idTemp |= (uint16_t) spi_putc(0xff) >> 5;
	
	p_message->id.srcID = (uint8_t) ((idTemp&0b0000011111000000)>>6);
	p_message->id.destID = (uint8_t) ((idTemp&0b0000000000111110)>>1);
	p_message->id.bit = (uint8_t) ((idTemp&0x0001));
	
	spi_putc(0xff); //EXTENDED IDENTIFIER HIGH
	spi_putc(0xff); //EXTENDED IDENTIFIER LOW
	
	// Laenge auslesen
	uint8_t length = spi_putc(0xff) & 0x0f;
	p_message->length = length;
	
	// Daten auslesen
	for (uint8_t i=0;i<length;i++) {
		p_message->data[i] = spi_putc(0xff);
	}
	
	PORT_CS |= (1<<P_CS);
	
	if (bit_is_set(status,3)) {
		p_message->rtr = 1;
	} else {
		p_message->rtr = 0;
	}
	
	p_message->arrivalTime=0;
	
	// Interrupt Flag loeschen
	if (bit_is_set(status,6)) {
		mcp2515_bit_modify(CANINTF, (1<<RX0IF), 0);
	} else {
		mcp2515_bit_modify(CANINTF, (1<<RX1IF), 0);
	}
	
	return (status & 0x07); // Return Filter Match (S.67)
}

uint8_t can_send_message(CANMessage *p_message)
{
    uint8_t status, address;
   
    // Status des MCP2515 auslesen
    PORT_CS &= ~(1<<P_CS);
    spi_putc(SPI_READ_STATUS);
    status = spi_putc(0xff);
    spi_putc(0xff);
    PORT_CS |= (1<<P_CS);
   
    /* Statusbyte:
     *
     * Bit  Funktion
     *  2   TXB0CNTRL.TXREQ
     *  4   TXB1CNTRL.TXREQ
     *  6   TXB2CNTRL.TXREQ
     */
   
    if (bit_is_clear(status, 2)) {
        address = 0x00;
    }
    else if (bit_is_clear(status, 4)) {
        address = 0x02;
    }
    else if (bit_is_clear(status, 6)) {
        address = 0x04;
    }
    else {
        /* Alle Puffer sind belegt,
           Nachricht kann nicht verschickt werden */
        return 0;
    }
   
    PORT_CS &= ~(1<<P_CS);    // CS Low
    spi_putc(SPI_WRITE_TX | address);
   
    // Standard ID einstellen
   // spi_putc((uint8_t) (p_message->id>>3));
   // spi_putc((uint8_t) (p_message->id<<5));
   
   spi_putc((uint8_t) ((p_message->id.srcID<<3)|(p_message->id.destID>>2)));
   
   #ifdef debug
   char temp[16];
   uart_puts("ID_HIGH: ");
   itoa((uint8_t) ((p_message->id.srcID<<3)|(p_message->id.destID>>2)), temp, 2);
   uart_puts(temp);
   
   uart_puts("ID_LOW: ");
   itoa((uint8_t) ((p_message->id.destID<<6)|(p_message->id.bit<<5)), temp, 2);
   uart_puts(temp);
   #endif
   
   spi_putc((uint8_t) ((p_message->id.destID<<6)|(p_message->id.bit<<5)));
   
    // Extended ID
    spi_putc(0x00);
    spi_putc(0x00);
   
    uint8_t length = p_message->length;
   
    if (length > 8) {
        length = 8;
    }
   
    // Ist die Nachricht ein "Remote Transmit Request" ?
    if (p_message->rtr)
    {
        /* Ein RTR hat zwar eine Laenge,
           aber enthaelt keine Daten */
       
        // Nachrichten Laenge + RTR einstellen
        spi_putc((1<<RTR) | length);
    }
    else
    {
        // Nachrichten Laenge einstellen
        spi_putc(length);
       
        // Daten
        for (uint8_t i=0;i<length;i++) {
            spi_putc(p_message->data[i]);
        }
    }
    PORT_CS |= (1<<P_CS);      // CS auf High
   
    asm volatile ("nop");
   
    /* CAN Nachricht verschicken
       die letzten drei Bit im RTS Kommando geben an welcher
       Puffer gesendet werden soll */
    PORT_CS &= ~(1<<P_CS);    // CS wieder Low
    if (address == 0x00) {
        spi_putc(SPI_RTS | 0x01);
    } else {
        spi_putc(SPI_RTS | address);
    }
    PORT_CS |= (1<<P_CS);      // CS auf High
   
    return 1;
}

uint8_t readSwitch() {
	uint8_t temp = PINC & 0b00111111;
	temp |= ((PIND & (1<<PIND4))<<2);
	temp |= (PINB & (1<<PINB7));

	return temp;
}

void addMessageToBuffer(CANMessage * msg) {
	copyMsg(msg, &receivedMsg[buf_tx]);
	if (buf_tx>=24) buf_tx=0;
	else buf_tx++;
}

CANMessage getMessageFromBuffer() {
	CANMessage msg = {0,0,0,0,0};
	if (buf_tx != buf_rx) {
		copyMsg(&receivedMsg[buf_rx], &msg);
		if (buf_rx>=24) buf_rx=0;
		else buf_rx++;
	}
	return msg;
}

void addMessageToTxBuffer(CANMessage * msg) {
	copyMsg(msg, &txMsg[bufTx_tx]);
	if (bufTx_tx>=24) bufTx_tx=0;
	else bufTx_tx++;
}

CANMessage getMessageFromTxBuffer() {
	CANMessage msg = {0,0,0,0,0};
	if (bufTx_tx != bufTx_rx) {
		copyMsg(&txMsg[bufTx_rx], &msg);
		if (bufTx_rx>=24) bufTx_rx=0;
		else bufTx_rx++;
	}
	return msg;
}

int main(void)
{
	wdt_enable(WDTO_500MS);
	initPanel();
	
	//uart_init();

	uint16_t mcuId = MCU_ID;
	if (mcuId == 0xFFFF)
	{
		// EEPROM empty, need to program MCU ID
		eeprom_write_word(MCU_ID_EEPROM_ADDR, BUS_ID);
	}

	mcp2515_init();
	initTimer();
	initInterrupt();
	
	for (uint8_t i=0; i<25; i++) {
		receivedMsg[i].id.destID=0;
	}
	// Ports f�r Taster als Eingang und PullUps setzen
	DDRB &= ~(1<<PORTB7);
	DDRC &= ~0b00111111;
	DDRD &= ~(1<<PORTD4);
	PORTB |= (1<<PORTB7);
	PORTC |= 0b00111111;
	PORTD |= (1<<PORTD4);
	
	// Uart-Rx Pin als Ausgang f�r Panel-Enable
	DDRD |= (1<<PORTD0);
	PORTD |= (1<<PORTD0);
	
	for (uint8_t i=0; i<=7; i++) {
		leds[i]=0X00;
		leds2[i]=0X00;
		switches[i]=0xFF;
		switchesHold[i]=0xFF;
	}
	
	leds2[0]=0x10; // Nur eine LED leuchtet nach Reset (ganz unten links)
	
	sei();
	
	// Nach Start: Request f�r alle Kan�le senden
	defaultMsgToLightActor11.data[0]=0; // Action 0, outID 0
	can_send_message(&defaultMsgToLightActor11);
	_delay_ms(10);
	defaultMsgToLightActor11.data[0]=1; // Action 0, outID 1
	can_send_message(&defaultMsgToLightActor11);
	_delay_ms(10);
	defaultMsgToLightActor11.data[0]=2; // Action 0, outID 2
	can_send_message(&defaultMsgToLightActor11);
	_delay_ms(10);
	defaultMsgToLightActor11.data[0]=3; // Action 0, outID 3
	can_send_message(&defaultMsgToLightActor11);
	_delay_ms(10);
	defaultMsgToLightActor11.data[0]=4; // Action 0, outID 4
	can_send_message(&defaultMsgToLightActor11);
	_delay_ms(10);
	
	wdt_enable(WDTO_250MS); // Watchdog auf 250ms einschalten
	
    while(1){
		/*
		if (saveActualValues>32) { // noch kein Req. verschickt?
			defaultMsgToLightActor11.data[0]=0; // Action 0, outID 0
			can_send_message(&defaultMsgToLightActor11);
			_delay_ms(10);
			defaultMsgToLightActor11.data[0]=1; // Action 0, outID 1
			can_send_message(&defaultMsgToLightActor11);
			_delay_ms(10);
			defaultMsgToLightActor11.data[0]=2; // Action 0, outID 2
			can_send_message(&defaultMsgToLightActor11);
			_delay_ms(10);
			defaultMsgToLightActor11.data[0]=3; // Action 0, outID 3
			can_send_message(&defaultMsgToLightActor11);
			_delay_ms(10);
			defaultMsgToLightActor11.data[0]=4; // Action 0, outID 3
			can_send_message(&defaultMsgToLightActor11);
			_delay_ms(10);
			saveActualValues-=32;
		}*/
		if (saveActualValues>0 && saveActualValues<11) {
			for (uint8_t i=0; i<=4; i++) {
				eeprom_write_byte(&eeByteArray1[saveActualValues-1][i],outputValues[i]);
			}
			saveActualValues=0;
		}
		
		CANMessage msg = getMessageFromBuffer();
		
		if (msg.id.destID == BUS_ID || msg.id.destID == BROADCAST_ID)
		{
			if (msg.length > 0u)
			{
				actionType_e action = GET_ACTION(msg.data[0u]);
				outID = GET_OUTID(msg.data[0u]);

				switch (action)
				{
					case ACTION_RESPONSE:
					{
						if (msg.length > 1u) // ResponseValue
						{ 
							if (msg.id.srcID == 11)
							{
								if (outID>=0 && outID <=4)
								{
									setLedBar(outID,msg.data[1]);						
									setPanelUpDownOnOffLeds(outID,msg.data[1]);	
									if (outputValues[outID]!=msg.data[1])
									{ // Neuer empfangener Wert deaktiviert SzenenTastenLed
										clearAllSceneLeds();
										outputValues[outID]=msg.data[1]; // Holds last State zum speichern
									}									
								}
							}
						}
						break;
					}
			
					case ACTION_RESET:
					{
						cli();                 // disable interrupts
						wdt_enable(WDTO_15MS); // watchdog timeout 15ms
						while(1);              // wait for watchdog to reset mcu
						break;
					}
					
					default:
					{
						break;
					}
				}
			}
		}
		
		msg = getMessageFromTxBuffer();
		if (msg.id.destID != 0)	{
			if (can_send_message(&msg)==0) addMessageToTxBuffer(&msg); // Alle Puffer voll-> Nachricht wieder im FIFO einreihen
		}
		
    }
}

ISR (TIMER0_OVF_vect) { // jede 1ms 0,25ms ALLE 0,5ms/500us
	//if (ticks>=1) {// Nach 10ms ----> Nach 0,5ms
	if (channel>15) {
		if (channel==16) data_out(0);
		if (channel>=25) { // Totzeit bestimmt Helligkeit der LEDs
			channel=255; // Wird unten inkrementiert 255 -> 0
		}
	} else if (channel>7) { // Panel 1 und Panel 2
		if (channel==8) PORTD &= ~(1<<PORTD0);
		data_out(0);
		set_adr(channel-8);
		data_out(leds2[channel-8]);	
	} else { // Panel 3 (4 Leuchtbalken)
		if (channel==0) PORTD |= (1<<PORTD0);
		data_out(0);
		set_adr(channel);
		data_out(leds[channel]);
	switches[channel]=readSwitch();
	}
	channel++;

	if (hjg<20) hjg++; // Nach 10ms
	else {
		hjg=0;
		if (hjh<10) { // Nochmal auf 10
			hjh++;
			if (hjh==1) {	
				
				wdt_reset(); // Alle 100ms Watchdog Reseten um sicherzustellen dass die LEDs nicht �berhitzen
				
				if (~switches[2]&0b00000010) { //P3, Ch0, BtnDown
					defaultMsgToLightActor11.data[0]=(0b01100000|0);
					addMessageToTxBuffer(&defaultMsgToLightActor11);
				} else if (~switches[2]&0b00000001) { //P3, Ch0, BtnUp
					defaultMsgToLightActor11.data[0]=(0b01000000|0);
					addMessageToTxBuffer(&defaultMsgToLightActor11);
				} else if (~switches[2]&0b00000100) { //P3, Ch0, BtnOnOff
					if (~switchesHold[2]&0b00000100) { // zuvor schon gedr�ckt?
							// warten bis zum loslassen
						} else { // einmal beim dr�cken ausf�hren
							defaultMsgToLightActor11.data[0]=(0b10000000|0);
							addMessageToTxBuffer(&defaultMsgToLightActor11);
							switchesHold[2] &= ~0b00000100;
						}
				} else switchesHold[2] |= 0b00000100;
			}
			if (hjh==2) {			
				if (~switches[2]&0b00010000) { //P3, Ch1, BtnDown
					defaultMsgToLightActor11.data[0]=(0b01100000|1);
					addMessageToTxBuffer(&defaultMsgToLightActor11);	
				} else if (~switches[2]&0b00001000) { //P3, Ch1, BtnUp
					defaultMsgToLightActor11.data[0]=(0b01000000|1);
					addMessageToTxBuffer(&defaultMsgToLightActor11);		
				} else if (~switches[2]&0b00100000) { //P3, Ch1, BtnOnOff
					if (~switchesHold[2]&0b00100000) { // zuvor schon gedr�ckt?
						// warten bis zum loslassen
					} else { // einmal beim dr�cken ausf�hren
						defaultMsgToLightActor11.data[0]=(0b10000000|1);
						addMessageToTxBuffer(&defaultMsgToLightActor11);
						switchesHold[2] &= ~0b00100000;
					}
				} else switchesHold[2] |= 0b00100000;
			}
			if (hjh==3) {		
				if (~switches[2]&0b10000000) { //P3, Ch2, BtnDown
					defaultMsgToLightActor11.data[0]=(0b01100000|2);
					addMessageToTxBuffer(&defaultMsgToLightActor11);
				} else if (~switches[2]&0b01000000) { //P3, Ch2, BtnUp
					defaultMsgToLightActor11.data[0]=(0b01000000|2);
					addMessageToTxBuffer(&defaultMsgToLightActor11);
				} else if (~switches[3]&0b00000001) { //P3, Ch3, BtnOnOff
					if (~switchesHold[3]&0b00000001) { // zuvor schon gedr�ckt?
						// warten bis zum loslassen
					} else { // einmal beim dr�cken ausf�hren
						defaultMsgToLightActor11.data[0]=(0b10000000|2);
						addMessageToTxBuffer(&defaultMsgToLightActor11);
						switchesHold[3] &= ~0b00000001;
					}
				} else switchesHold[3] |= 0b00000001;
			}
			if (hjh==4) {		
				if (~switches[3]&0b00000100) { //P3, Ch3, BtnDown
					defaultMsgToLightActor11.data[0]=(0b01100000|3);
					addMessageToTxBuffer(&defaultMsgToLightActor11);
				} else if (~switches[3]&0b00000010) { //P3, Ch3, BtnUp
					defaultMsgToLightActor11.data[0]=(0b01000000|3);
					addMessageToTxBuffer(&defaultMsgToLightActor11);
				} else if (~switches[3]&0b00001000) { //P3, Ch3, BtnOnOff
					if (~switchesHold[3]&0b00001000) { // zuvor schon gedr�ckt?
						// warten bis zum loslassen
					} else { // einmal beim dr�cken ausf�hren
						defaultMsgToLightActor11.data[0]=(0b10000000|3);
						addMessageToTxBuffer(&defaultMsgToLightActor11);
						switchesHold[3] &= ~0b00001000;
					}
				} else switchesHold[3] |= 0b00001000;
			}
			if (hjh==5) {
				if (~switches[5]&LED_P3_CH5_DOWN) { //P3, Ch3, BtnDown
					defaultMsgToLightActor11.data[0]=(0b01100000|4);
					addMessageToTxBuffer(&defaultMsgToLightActor11);
				} else if (~switches[5]&LED_P3_CH5_UP) { //P3, Ch3, BtnUp
					defaultMsgToLightActor11.data[0]=(0b01000000|4);
					addMessageToTxBuffer(&defaultMsgToLightActor11);
				} else if (~switches[5]&LED_P3_CH5_ONOFF) { //P3, Ch3, BtnOnOff
					if (~switchesHold[5]&LED_P3_CH5_ONOFF) { // zuvor schon gedr�ckt?
					// warten bis zum loslassen
					} else { // einmal beim dr�cken ausf�hren
						defaultMsgToLightActor11.data[0]=(0b10000000|4);
						addMessageToTxBuffer(&defaultMsgToLightActor11);
						switchesHold[5] &= ~LED_P3_CH5_ONOFF;
					}
				} else switchesHold[5] |= LED_P3_CH5_ONOFF;
			}
			if (hjh==6) {
				if (~switches[5]&0b00000100) { //LightSzene ProgrammBtn
					if (~switchesHold[5]&0b00000100) { // zuvor schon gedr�ckt?
						// warten bis zum loslassen
					} else { // einmal beim dr�cken ausf�hren
						if (programmLightSzenzeMode==0) {
							clearAllSceneLeds();
							programmLightSzenzeMode=1;
							setPanelLed(0b00000100,0x15);
						}							
						else {
							programmLightSzenzeMode=0;
							clearPanelLed(0b00000100,0x15);
						}	
												
						switchesHold[5] &= ~0b00000100;
					}
				} else switchesHold[5] |= 0b00000100;						
				
				if (~switches[4]&0b00000001) {
					if(~switchesHold[4]&0b00000001) {
					} else {
						if (programmLightSzenzeMode==1) {
							saveActualValues=1; //+32; // Speicherplatz 1 + 32 zeigt an dass noch kein Req. verschickt wurde
							programmLightSzenzeMode=0;
							clearPanelLed(0b00000100,0x15);
						} else {							
							//Lichtszene aus EEPROM holen und ausf�hren
							for (uint8_t i=0; i<=4; i++) {
								defaultMsgToLightActor11.data[0]=(0b00100000|i);
								defaultMsgToLightActor11.data[1]=eeprom_read_byte(&eeByteArray1[0][i]);
								outputValues[i]=defaultMsgToLightActor11.data[1];
								defaultMsgToLightActor11.length=2;
								addMessageToTxBuffer(&defaultMsgToLightActor11);
								defaultMsgToLightActor11.length=1;
							}
						}						
						clearAllSceneLeds();
						setPanelLed(0b00000001,0x16); // Wann wird LED wieder gel�scht?
						switchesHold[4] &= ~0b00000001;	
					}
				} else switchesHold[4] |= 0b00000001;
				
				if (~switches[4]&0b00000010) {
					if(~switchesHold[4]&0b00000010) {
					} else {
						if (programmLightSzenzeMode==1) {
							saveActualValues=2; //+32; // Speicherplatz 1 + 32 zeigt an dass noch kein Req. verschickt wurde
							programmLightSzenzeMode=0;
							clearPanelLed(0b00000100,0x15);
						} else {
							//Lichtszene aus EEPROM holen und ausf�hren
							for (uint8_t i=0; i<=4; i++) {
								defaultMsgToLightActor11.data[0]=(0b00100000|i);
								defaultMsgToLightActor11.data[1]=eeprom_read_byte(&eeByteArray1[1][i]);
								outputValues[i]=defaultMsgToLightActor11.data[1];
								defaultMsgToLightActor11.length=2;
								addMessageToTxBuffer(&defaultMsgToLightActor11);
								defaultMsgToLightActor11.length=1;
							}
						}
						clearAllSceneLeds();
						setPanelLed(0b00000010,0x16); // Wann wird LED wieder gel�scht?
						switchesHold[4] &= ~0b00000010;
					}
				} else switchesHold[4] |= 0b00000010;
			}
			
			if (hjh==7) {
				if (~switches[4]&0b00000100) {
					if(~switchesHold[4]&0b00000100) {
					} else {
						if (programmLightSzenzeMode==1) {
							saveActualValues=3; //+32; // Speicherplatz 1 + 32 zeigt an dass noch kein Req. verschickt wurde
							programmLightSzenzeMode=0;
							clearPanelLed(0b00000100,0x15);
						} else {
							//Lichtszene aus EEPROM holen und ausf�hren
							for (uint8_t i=0; i<=4; i++) {
								defaultMsgToLightActor11.data[0]=(0b00100000|i);
								defaultMsgToLightActor11.data[1]=eeprom_read_byte(&eeByteArray1[2][i]);
								outputValues[i]=defaultMsgToLightActor11.data[1];
								defaultMsgToLightActor11.length=2;
								addMessageToTxBuffer(&defaultMsgToLightActor11);
								defaultMsgToLightActor11.length=1;
							}
						}
						clearAllSceneLeds();
						setPanelLed(0b00000100,0x16); // Wann wird LED wieder gel�scht?
						switchesHold[4] &= ~0b00000100;
					}
				} else switchesHold[4] |= 0b00000100;
				if (~switches[4]&0b00001000) {
					if(~switchesHold[4]&0b00001000) {
					} else {
						if (programmLightSzenzeMode==1) {
							saveActualValues=4; //+32; // Speicherplatz 1 + 32 zeigt an dass noch kein Req. verschickt wurde
							programmLightSzenzeMode=0;
							clearPanelLed(0b00000100,0x15);
						} else {
							//Lichtszene aus EEPROM holen und ausf�hren
							for (uint8_t i=0; i<=4; i++) {
								defaultMsgToLightActor11.data[0]=(0b00100000|i);
								defaultMsgToLightActor11.data[1]=eeprom_read_byte(&eeByteArray1[3][i]);
								outputValues[i]=defaultMsgToLightActor11.data[1];
								defaultMsgToLightActor11.length=2;
								addMessageToTxBuffer(&defaultMsgToLightActor11);
								defaultMsgToLightActor11.length=1;
							}
						}
						clearAllSceneLeds();
						setPanelLed(0b00001000,0x16); // Wann wird LED wieder gel�scht?
						switchesHold[4] &= ~0b00001000;
					}
				} else switchesHold[4] |= 0b00001000;
				if (~switches[4]&0b00010000) {
					if(~switchesHold[4]&0b00010000) {
					} else {
						if (programmLightSzenzeMode==1) {
							saveActualValues=5; //+32; // Speicherplatz 1 + 32 zeigt an dass noch kein Req. verschickt wurde
							programmLightSzenzeMode=0;
							clearPanelLed(0b00000100,0x15);
						} else {
							//Lichtszene aus EEPROM holen und ausf�hren
							for (uint8_t i=0; i<=4; i++) {
								defaultMsgToLightActor11.data[0]=(0b00100000|i);
								defaultMsgToLightActor11.data[1]=eeprom_read_byte(&eeByteArray1[4][i]);
								outputValues[i]=defaultMsgToLightActor11.data[1];
								defaultMsgToLightActor11.length=2;
								addMessageToTxBuffer(&defaultMsgToLightActor11);
								defaultMsgToLightActor11.length=1;
							}
						}
						clearAllSceneLeds();
						setPanelLed(0b00010000,0x16); // Wann wird LED wieder gel�scht?
						switchesHold[4] &= ~0b00010000;
					}
				} else switchesHold[4] |= 0b00010000;
			}
			if (hjh==8) {
				if (~switches[4]&0b00100000) {
					if(~switchesHold[4]&0b00100000) {
					} else {
						if (programmLightSzenzeMode==1) {
							saveActualValues=6; //+32; // Speicherplatz 1 + 32 zeigt an dass noch kein Req. verschickt wurde
							programmLightSzenzeMode=0;
							clearPanelLed(0b00000100,0x15);
						} else {
							//Lichtszene aus EEPROM holen und ausf�hren
							for (uint8_t i=0; i<=4; i++) {
								defaultMsgToLightActor11.data[0]=(0b00100000|i);
								defaultMsgToLightActor11.data[1]=eeprom_read_byte(&eeByteArray1[5][i]);
								outputValues[i]=defaultMsgToLightActor11.data[1];
								defaultMsgToLightActor11.length=2;
								addMessageToTxBuffer(&defaultMsgToLightActor11);
								defaultMsgToLightActor11.length=1;
							}
						}
						clearAllSceneLeds();
						setPanelLed(0b00100000,0x16); // Wann wird LED wieder gel�scht?
						switchesHold[4] &= ~0b00100000;
					}
				} else switchesHold[4] |= 0b00100000;
				if (~switches[4]&0b01000000) {
					if(~switchesHold[4]&0b01000000) {
					} else {
						if (programmLightSzenzeMode==1) {
							saveActualValues=7; //+32; // Speicherplatz 1 + 32 zeigt an dass noch kein Req. verschickt wurde
							programmLightSzenzeMode=0;
							clearPanelLed(0b00000100,0x15);
						} else {
							//Lichtszene aus EEPROM holen und ausf�hren
							for (uint8_t i=0; i<=4; i++) {
								defaultMsgToLightActor11.data[0]=(0b00100000|i);
								defaultMsgToLightActor11.data[1]=eeprom_read_byte(&eeByteArray1[6][i]);
								outputValues[i]=defaultMsgToLightActor11.data[1];
								defaultMsgToLightActor11.length=2;
								addMessageToTxBuffer(&defaultMsgToLightActor11);
								defaultMsgToLightActor11.length=1;
							}
						}
						clearAllSceneLeds();
						setPanelLed(0b01000000,0x16); // Wann wird LED wieder gel�scht?
						switchesHold[4] &= ~0b01000000;
					}
				} else switchesHold[4] |= 0b01000000;
				if (~switches[4]&0b10000000) {
					if(~switchesHold[4]&0b10000000) {
					} else {
						if (programmLightSzenzeMode==1) {
							saveActualValues=8; //+32; // Speicherplatz 1 + 32 zeigt an dass noch kein Req. verschickt wurde
							programmLightSzenzeMode=0;
							clearPanelLed(0b00000100,0x15);
						} else {
							//Lichtszene aus EEPROM holen und ausf�hren
							for (uint8_t i=0; i<=4; i++) {
								defaultMsgToLightActor11.data[0]=(0b00100000|i);
								defaultMsgToLightActor11.data[1]=eeprom_read_byte(&eeByteArray1[7][i]);
								outputValues[i]=defaultMsgToLightActor11.data[1];
								defaultMsgToLightActor11.length=2;
								addMessageToTxBuffer(&defaultMsgToLightActor11);
								defaultMsgToLightActor11.length=1;
							}
						}
						clearAllSceneLeds();
						setPanelLed(0b10000000,0x16); // Wann wird LED wieder gel�scht?
						switchesHold[4] &= ~0b10000000;
					}
				} else switchesHold[4] |= 0b10000000;
			}
			if (hjh==9) {
				if (~switches[5]&0b00000001) {
					if(~switchesHold[5]&0b00000001) {
					} else {
						if (programmLightSzenzeMode==1) {
							saveActualValues=9; //+32; // Speicherplatz 1 + 32 zeigt an dass noch kein Req. verschickt wurde
							programmLightSzenzeMode=0;
							clearPanelLed(0b00000100,0x15);
						} else {
							//Lichtszene aus EEPROM holen und ausf�hren
							for (uint8_t i=0; i<=4; i++) {
								defaultMsgToLightActor11.data[0]=(0b00100000|i);
								defaultMsgToLightActor11.data[1]=eeprom_read_byte(&eeByteArray1[8][i]);
								outputValues[i]=defaultMsgToLightActor11.data[1];
								defaultMsgToLightActor11.length=2;
								addMessageToTxBuffer(&defaultMsgToLightActor11);
								defaultMsgToLightActor11.length=1;
							}
						}
						clearAllSceneLeds();
						setPanelLed(0b01000000,0x15); // Wann wird LED wieder gel�scht?
						switchesHold[5] &= ~0b00000001;
					}
				} else switchesHold[5] |= 0b00000001;
				if (~switches[5]&0b00000010) {
					if(~switchesHold[5]&0b00000010) {
					} else {
						if (programmLightSzenzeMode==1) {
							saveActualValues=10; //+32; // Speicherplatz 1 + 32 zeigt an dass noch kein Req. verschickt wurde
							programmLightSzenzeMode=0;
							clearPanelLed(0b00000100,0x15);
						} else {
							//Lichtszene aus EEPROM holen und ausf�hren
							for (uint8_t i=0; i<=4; i++) {
								defaultMsgToLightActor11.data[0]=(0b00100000|i);
								defaultMsgToLightActor11.data[1]=eeprom_read_byte(&eeByteArray1[9][i]);
								outputValues[i]=defaultMsgToLightActor11.data[1];
								defaultMsgToLightActor11.length=2;
								addMessageToTxBuffer(&defaultMsgToLightActor11);
								defaultMsgToLightActor11.length=1;
							}
						}
						clearAllSceneLeds();
						setPanelLed(0b10000000,0x15); // Wann wird LED wieder gel�scht?
						switchesHold[5] &= ~0b00000010;
					}
				} else switchesHold[5] |= 0b00000010;
			}
			
			
		} else hjh=0;						
	}		
		
			
		time++;
		//ticks=0;
	//} else ticks++;
	
	//TCNT0 = 131;
	//TCNT0 = 200;
	//TCNT0 = 6;
	TCNT0 = 194;
}

ISR(INT1_vect) {
	CANMessage msg;
	while (can_get_message(&msg)!=0xFF) {
		addMessageToBuffer(&msg);
	}
}

