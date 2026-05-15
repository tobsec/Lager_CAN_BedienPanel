/*
 * ATmega8_MCP2515.c
 *
 * Created: 7/19/2014 7:29:49 PM
 *  Author: Tobias
 */ 

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#include <avr/wdt.h>

//#include "UART.h"
#include "Panel.h"
#include "PanelLEDdefs.h"
#include "LagerLight_protocol.h"
#include "CAN.h"
#include "avr_hwAb.h"

#include "config.h"


#define MCU_ID_EEPROM_ADDR 256
#define MCU_ID eeprom_read_word((uint16_t*) MCU_ID_EEPROM_ADDR)



uint8_t switches[8];
uint8_t switchesHold[8];

uint8_t value = 128u;
uint8_t value2 = 0u;


// volatile + 16-bit on an 8-bit MCU: the ISR updates this every ~0.5 ms and
// the main loop reads it from the scene-recall drain. Without volatile the
// compiler may cache the read; without an atomic guard the high byte can
// shift between the two LDS instructions. Use volatile here and read it
// atomically with ATOMIC_BLOCK below.
static volatile uint16_t time = 0u; // Wird alle 10ms hochgez�hlt

static uint8_t outputValue[OUT_CHANNELS] = OUT_VALUE_START;
static uint8_t programmLightSzenzeMode = 0u;
static uint8_t saveActualValues = 0u;

// Scene-recall state machine. The old code blasted all 5 channel SET frames
// into the TX FIFO in a tight for-loop inside the timer ISR — fine for the
// queue, but the gateway-side RX queue could drop them when the gateway's
// main loop was busy (e.g. doing an MQTT TLS publish). Now we just record
// which scene to recall + which channel we're up to, and the main loop
// drains one frame at a time with a gap.
#define SCENE_RECALL_GAP_TICKS  2u  // ~16 ms at the ~8 ms ISR tick
static volatile uint8_t pendingSceneIdx     = 0xFFu;   // 0..9 = scene to recall, 0xFF = idle
static volatile uint8_t pendingSceneChannel = 0u;
static uint16_t         lastSceneTick       = 0u;

// Called from the ISR when a scene button is pressed. Just records the
// recall request — the actual frames are emitted from the main loop.
static void requestSceneRecall(uint8_t sceneIdx)
{
	pendingSceneIdx     = sceneIdx;
	pendingSceneChannel = 0u;
	lastSceneTick       = 0u; // emit first frame on the next main-loop pass
}

uint8_t eeByteArray1[10][5] EEMEM = {{255, 255, 255, 255, 255}, // 5 Kan�le auf 10 Speicherpl�tzen
									 {0, 0, 0, 0, 0},
									 {128, 128, 128, 128, 128},
									 {0, 0, 0, 0, 0},
									 {0, 0, 0, 0, 0},
									 {0, 0, 0, 0, 0},
									 {0, 0, 0, 0, 0},
									 {0, 0, 0, 0, 0},
									 {0, 0, 0, 0, 0},
									 {0, 0, 0, 0, 0}};

CANMessage defaultMsgToLightActor11 =
{ 
		.id = { .srcID = THIS_ID, .destID = LIGHTACTOR_11_ID, .bit = 0u },
		.rtr = 0u,
		.length = 1u,
		.arrivalTime = 0u,
		.data = { 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u }
};


uint8_t readSwitch()
{
	uint8_t temp = PINC & 0b00111111;
	temp |= ((PIND & (1 << PIND4)) << 2);
	temp |= (PINB & (1 << PINB7));

	return temp;
}

void initD1()
{
	DDRD |= (1<<PORTD1);  // Init PD1 as output for debugging
}

void toggleD1()
{
	PORTD ^= (1 << PD1);  // Toggle PD1 for debugging
}



int main(void)
{
	wdt_enable(WDTO_500MS);
	initPanel();
	
	//uart_init();

	uint16_t mcuId = MCU_ID;
	if (mcuId == 0xFFFFu)
	{
		// EEPROM empty, need to program MCU ID
		eeprom_write_word((uint16_t*) MCU_ID_EEPROM_ADDR, THIS_ID);
	}

	can_init();
	initTimer();
	
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
	
	for (uint8_t i = 0u; i <= 7u; i++)
	{
		leds[i]=0X00u;
		leds2[i]=0X00u;
		switches[i]=0xFFu;
		switchesHold[i]=0xFFu;
	}
	
	leds2[0u] = 0x10u; // Nur eine LED leuchtet nach Reset (ganz unten links)
	
	sei();

	// Nach Start: Request fuer alle Kanaele senden. 20 ms zwischen den
	// Frames damit die 3 MCP2515-TX-Buffer Zeit zum Leeren haben und
	// die gateway-RX nicht ueberlaeuft. Use a stack-local CANMessage —
	// the global defaultMsgToLightActor11 is owned by the TIMER0 ISR
	// (button-press handlers) and must not be mutated from main.
	for (uint8_t i = 0u; i < 5u; i++)
	{
		CANMessage req = defaultMsgToLightActor11;
		req.data[0u] = i; // Action 0 (REQUEST_VALUE), outID i
		can_sendBufferedMessage(&req);
		_delay_ms(20);
	}
	
	wdt_enable(WDTO_250MS); // Watchdog auf 250ms einschalten

    while(1)
    {

		if ((saveActualValues > 0u) && (saveActualValues < 11u))
		{
			for (uint8_t i = 0u; i <= 4u; i++)
			{
				eeprom_write_byte(&eeByteArray1[saveActualValues-1u][i],outputValue[i]);
			}
			saveActualValues = 0u;
		}

		// Drain pending scene recall: one SET_VALUE frame per ~16 ms.
		// `time` is 16-bit and updated from the ISR; read it atomically
		// to avoid a byte tear. The frame itself is built on the stack
		// (NOT in the shared defaultMsgToLightActor11) because the ISR
		// is mutating that global concurrently for button presses.
		if (pendingSceneIdx < 10u && pendingSceneChannel < 5u)
		{
			uint16_t tNow;
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { tNow = time; }
			if ((uint16_t)(tNow - lastSceneTick) >= SCENE_RECALL_GAP_TICKS)
			{
				uint8_t ch = pendingSceneChannel;
				CANMessage sceneMsg = defaultMsgToLightActor11;
				sceneMsg.data[0] = (0b00100000 | ch);
				sceneMsg.data[1] = eeprom_read_byte(&eeByteArray1[pendingSceneIdx][ch]);
				sceneMsg.length  = 2;
				outputValue[ch]  = sceneMsg.data[1];
				can_addMessageToTxBuffer(&sceneMsg);
				lastSceneTick = tNow;
				pendingSceneChannel++;
				if (pendingSceneChannel >= 5u)
				{
					pendingSceneIdx = 0xFFu;
				}
			}
		}
		
		CANMessage msg = can_getMessageFromBuffer();
		uint8_t outID = 0u;

		if (msg.id.destID == THIS_ID || msg.id.destID == BROADCAST_ID)
		{
			if (msg.length > 0u)
			{
				actionType_e action = GET_ACTION(msg.data[0u]);
				outID = GET_OUTID(msg.data[0u]);

				if (outID >= 0u && outID <= (OUT_CHANNELS - 1u))
				{
					switch (action)
					{
						case ACTION_RESPONSE:
						{
							if (msg.length > 1u) // ResponseValue
							{ 
								if (msg.id.srcID == 11)
								{
									if (outID>=0 && outID <=OUT_CHANNELS)
									{
										setLedBar(outID,msg.data[1]);						
										setPanelUpDownOnOffLeds(outID,msg.data[1]);	
										if (outputValue[outID]!=msg.data[1])
										{ // Neuer empfangener Wert deaktiviert SzenenTastenLed
											clearAllSceneLeds();
											outputValue[outID]=msg.data[1]; // Holds last State zum speichern
										}									
									}
								}
							}
							break;
						}
						
						case ACTION_REQUEST_VALUE:
						{
							msg.id.destID = msg.id.srcID; // Return to sender
							msg.id.srcID = THIS_ID;
							SET_ACTION(msg.data[0u], ACTION_RESPONSE); // Set all action bits (111) for response, don't touch outputID

							msg.data[1u] = outputValue[outID];
							msg.length = 2u;
							can_sendBufferedMessage(&msg);
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
		}
		
		can_loop();
    }
}

ISR (TIMER0_OVF_vect) // jede 1ms 0,25ms ALLE 0,5ms/500us
{	
	static uint8_t channel = 0u;
	static uint8_t cnt_10ms = 0u;
	static uint8_t cnt_10x10ms = 0u;

	// toggleD1();
	
	//if (ticks>=1) {// Nach 10ms ----> Nach 0,5ms
	if (channel > 15)
	{
		if (channel == 16) data_out(0);
		if (channel >= 25) // Totzeit bestimmt Helligkeit der LEDs
		{
			channel=255; // Wird unten inkrementiert 255 -> 0
		}
	}
	else if (channel > 7) // Panel 1 und Panel 2
	{
		if (channel == 8) PORTD &= ~(1<<PORTD0);
		data_out(0);
		set_adr(channel-8);
		data_out(leds2[channel-8]);	
	}
	else // Panel 3 (4 Leuchtbalken)
	{
		if (channel == 0) PORTD |= (1<<PORTD0);
		data_out(0);
		set_adr(channel);
		data_out(leds[channel]);
		switches[channel] = readSwitch();
	}
	channel++;

	if (cnt_10ms < 20)
	{
		cnt_10ms++; // Nach 10ms
	}
	else
	{
		cnt_10ms=0;
		if (cnt_10x10ms < 10) // Nochmal auf 10
		{
			cnt_10x10ms++;
			if (cnt_10x10ms ==  1)
			{	
				
				wdt_reset(); // Alle 100ms Watchdog Reseten um sicherzustellen dass die LEDs nicht �berhitzen
				
				if (~switches[2]&0b00000010) //P3, Ch0, BtnDown
				{
					defaultMsgToLightActor11.data[0]=(0b01100000|0);
					can_addMessageToTxBuffer(&defaultMsgToLightActor11);
				}
				else if (~switches[2]&0b00000001) //P3, Ch0, BtnUp
				{
					defaultMsgToLightActor11.data[0]=(0b01000000|0);
					can_addMessageToTxBuffer(&defaultMsgToLightActor11);
				}
				else if (~switches[2]&0b00000100) //P3, Ch0, BtnOnOff
				{
					if (~switchesHold[2]&0b00000100) // zuvor schon gedr�ckt?
					{
							// warten bis zum loslassen
					}
					else
					{
						// einmal beim dr�cken ausf�hren
						defaultMsgToLightActor11.data[0]=(0b10000000|0);
						can_addMessageToTxBuffer(&defaultMsgToLightActor11);
						switchesHold[2] &= ~0b00000100;
					}
				}
				else
				{
					switchesHold[2] |= 0b00000100;
				}
			}
			if (cnt_10x10ms == 2)
			{
				if (~switches[2] & 0b00010000)
				{
					// P3, Ch1, BtnDown
					defaultMsgToLightActor11.data[0] = (0b01100000 | 1);
					can_addMessageToTxBuffer(&defaultMsgToLightActor11);
				}
				else if (~switches[2] & 0b00001000)
				{
					// P3, Ch1, BtnUp
					defaultMsgToLightActor11.data[0] = (0b01000000 | 1);
					can_addMessageToTxBuffer(&defaultMsgToLightActor11);
				}
				else if (~switches[2] & 0b00100000)
				{
					// P3, Ch1, BtnOnOff
					if (~switchesHold[2] & 0b00100000)
					{
						// zuvor schon gedr�ckt?
						// warten bis zum loslassen
					}
					else
					{
						// einmal beim dr�cken ausf�hren
						defaultMsgToLightActor11.data[0] = (0b10000000 | 1);
						can_addMessageToTxBuffer(&defaultMsgToLightActor11);
						switchesHold[2] &= ~0b00100000;
					}
				}
				else
				 {
					switchesHold[2] |= 0b00100000;
				 }
			}
			if (cnt_10x10ms == 3)
			{
				if (~switches[2] & 0b10000000)
				{ // P3, Ch2, BtnDown
					defaultMsgToLightActor11.data[0] = (0b01100000 | 2);
					can_addMessageToTxBuffer(&defaultMsgToLightActor11);
				}
				else if (~switches[2] & 0b01000000)
				{ // P3, Ch2, BtnUp
					defaultMsgToLightActor11.data[0] = (0b01000000 | 2);
					can_addMessageToTxBuffer(&defaultMsgToLightActor11);
				}
				else if (~switches[3] & 0b00000001)
				{ // P3, Ch3, BtnOnOff
					if (~switchesHold[3] & 0b00000001)
					{	// zuvor schon gedr�ckt?
						// warten bis zum loslassen
					}
					else
					{ // einmal beim dr�cken ausf�hren
						defaultMsgToLightActor11.data[0] = (0b10000000 | 2);
						can_addMessageToTxBuffer(&defaultMsgToLightActor11);
						switchesHold[3] &= ~0b00000001;
					}
				}
				else
				{
					switchesHold[3] |= 0b00000001;
				}
			}
			if (cnt_10x10ms == 4)
			{
				if (~switches[3] & 0b00000100)
				{
					// P3, Ch3, BtnDown
					defaultMsgToLightActor11.data[0] = (0b01100000 | 3);
					can_addMessageToTxBuffer(&defaultMsgToLightActor11);
				}
				else if (~switches[3] & 0b00000010)
				{
					// P3, Ch3, BtnUp
					defaultMsgToLightActor11.data[0] = (0b01000000 | 3);
					can_addMessageToTxBuffer(&defaultMsgToLightActor11);
				}
				else if (~switches[3] & 0b00001000)
				{
					// P3, Ch3, BtnOnOff
					if (~switchesHold[3] & 0b00001000)
					{
						// zuvor schon gedr�ckt?
						// warten bis zum loslassen
					}
					else
					{
						// einmal beim dr�cken ausf�hren
						defaultMsgToLightActor11.data[0] = (0b10000000 | 3);
						can_addMessageToTxBuffer(&defaultMsgToLightActor11);
						switchesHold[3] &= ~0b00001000;
					}
				}
				else
				{
					switchesHold[3] |= 0b00001000;
				}
			}
			if (cnt_10x10ms == 5)
			{
				if (~switches[5] & LED_P3_CH5_DOWN)
				{
					// P3, Ch3, BtnDown
					defaultMsgToLightActor11.data[0] = (0b01100000 | 4);
					can_addMessageToTxBuffer(&defaultMsgToLightActor11);
				}
				else if (~switches[5] & LED_P3_CH5_UP)
				{
					// P3, Ch3, BtnUp
					defaultMsgToLightActor11.data[0] = (0b01000000 | 4);
					can_addMessageToTxBuffer(&defaultMsgToLightActor11);
				}
				else if (~switches[5] & LED_P3_CH5_ONOFF)
				{
					// P3, Ch3, BtnOnOff
					if (~switchesHold[5] & LED_P3_CH5_ONOFF)
					{
						// zuvor schon gedr�ckt?
						// warten bis zum loslassen
					}
					else
					{
						// einmal beim dr�cken ausf�hren
						defaultMsgToLightActor11.data[0] = (0b10000000 | 4);
						can_addMessageToTxBuffer(&defaultMsgToLightActor11);
						switchesHold[5] &= ~LED_P3_CH5_ONOFF;
					}
				}
				else
				{
					switchesHold[5] |= LED_P3_CH5_ONOFF;
				}
			}
			if (cnt_10x10ms == 6)
			{
				if (~switches[5] & 0b00000100)
				{
					// LightSzene ProgrammBtn
					if (~switchesHold[5] & 0b00000100)
					{
						// zuvor schon gedr�ckt?
						// warten bis zum loslassen
					}
					else
					{
						// einmal beim dr�cken ausf�hren
						if (programmLightSzenzeMode == 0)
						{
							clearAllSceneLeds();
							programmLightSzenzeMode = 1;
							setPanelLed(0b00000100, 0x15);
						}
						else
						{
							programmLightSzenzeMode = 0;
							clearPanelLed(0b00000100, 0x15);
						}

						switchesHold[5] &= ~0b00000100;
					}
				}
				else
				{
					switchesHold[5] |= 0b00000100;
				}

				if (~switches[4] & 0b00000001)
				{
					if (~switchesHold[4] & 0b00000001)
					{
						//
					}
					else
					{
						if (programmLightSzenzeMode == 1)
						{
							saveActualValues = 1; //+32; // Speicherplatz 1 + 32 zeigt an dass noch kein Req. verschickt wurde
							programmLightSzenzeMode = 0;
							clearPanelLed(0b00000100, 0x15);
						}
						else
						{
							requestSceneRecall(0);
						}
						clearAllSceneLeds();
						setPanelLed(0b00000001, 0x16); // Wann wird LED wieder gel�scht?
						switchesHold[4] &= ~0b00000001;
					}
				}
				else
				{
					switchesHold[4] |= 0b00000001;
				}

				if (~switches[4] & 0b00000010)
				{
					if (~switchesHold[4] & 0b00000010)
					{
						//
					}
					else
					{
						if (programmLightSzenzeMode == 1)
						{
							saveActualValues = 2; //+32; // Speicherplatz 1 + 32 zeigt an dass noch kein Req. verschickt wurde
							programmLightSzenzeMode = 0;
							clearPanelLed(0b00000100, 0x15);
						}
						else
						{
							requestSceneRecall(1);
						}
						clearAllSceneLeds();
						setPanelLed(0b00000010, 0x16); // Wann wird LED wieder gel�scht?
						switchesHold[4] &= ~0b00000010;
					}
				}
				else
				{
					switchesHold[4] |= 0b00000010;
				}
			}

			if (cnt_10x10ms == 7)
			{
				if (~switches[4] & 0b00000100)
				{
					if (~switchesHold[4] & 0b00000100)
					{
						//
					}
					else
					{
						if (programmLightSzenzeMode == 1)
						{
							saveActualValues = 3; //+32; // Speicherplatz 1 + 32 zeigt an dass noch kein Req. verschickt wurde
							programmLightSzenzeMode = 0;
							clearPanelLed(0b00000100, 0x15);
						}
						else
						{
							requestSceneRecall(2);
						}
						clearAllSceneLeds();
						setPanelLed(0b00000100, 0x16); // Wann wird LED wieder gel�scht?
						switchesHold[4] &= ~0b00000100;
					}
				}
				else
				{
					switchesHold[4] |= 0b00000100;
				}

				if (~switches[4] & 0b00001000)
				{
					if (~switchesHold[4] & 0b00001000)
					{
						//
					}
					else
					{
						if (programmLightSzenzeMode == 1)
						{
							saveActualValues = 4; //+32; // Speicherplatz 1 + 32 zeigt an dass noch kein Req. verschickt wurde
							programmLightSzenzeMode = 0;
							clearPanelLed(0b00000100, 0x15);
						}
						else
						{
							requestSceneRecall(3);
						}
						clearAllSceneLeds();
						setPanelLed(0b00001000, 0x16); // Wann wird LED wieder gel�scht?
						switchesHold[4] &= ~0b00001000;
					}
				}
				else
				{
					switchesHold[4] |= 0b00001000;
				}

				if (~switches[4] & 0b00010000)
				{
					if (~switchesHold[4] & 0b00010000)
					{
						//
					}
					else
					{
						if (programmLightSzenzeMode == 1)
						{
							saveActualValues = 5; //+32; // Speicherplatz 1 + 32 zeigt an dass noch kein Req. verschickt wurde
							programmLightSzenzeMode = 0;
							clearPanelLed(0b00000100, 0x15);
						}
						else
						{
							requestSceneRecall(4);
						}
						clearAllSceneLeds();
						setPanelLed(0b00010000, 0x16); // Wann wird LED wieder gel�scht?
						switchesHold[4] &= ~0b00010000;
					}
				}
				else
				{
					switchesHold[4] |= 0b00010000;
				}
			}

			if (cnt_10x10ms == 8)
			{
				if (~switches[4] & 0b00100000)
				{
					if (~switchesHold[4] & 0b00100000)
					{
						//
					}
					else
					{
						if (programmLightSzenzeMode == 1)
						{
							saveActualValues = 6; //+32; // Speicherplatz 1 + 32 zeigt an dass noch kein Req. verschickt wurde
							programmLightSzenzeMode = 0;
							clearPanelLed(0b00000100, 0x15);
						}
						else
						{
							requestSceneRecall(5);
						}
						clearAllSceneLeds();
						setPanelLed(0b00100000, 0x16); // Wann wird LED wieder gel�scht?
						switchesHold[4] &= ~0b00100000;
					}
				}
				else
				{
					switchesHold[4] |= 0b00100000;
				}
				
				if (~switches[4] & 0b01000000)
				{
					if (~switchesHold[4] & 0b01000000)
					{
						//
					}
					else
					{
						if (programmLightSzenzeMode == 1)
						{
							saveActualValues = 7; //+32; // Speicherplatz 1 + 32 zeigt an dass noch kein Req. verschickt wurde
							programmLightSzenzeMode = 0;
							clearPanelLed(0b00000100, 0x15);
						}
						else
						{
							requestSceneRecall(6);
						}
						clearAllSceneLeds();
						setPanelLed(0b01000000, 0x16); // Wann wird LED wieder gel�scht?
						switchesHold[4] &= ~0b01000000;
					}
				}
				else
				{
					switchesHold[4] |= 0b01000000;
				}

				if (~switches[4] & 0b10000000)
				{
					if (~switchesHold[4] & 0b10000000)
					{
						//
					}
					else
					{
						if (programmLightSzenzeMode == 1)
						{
							saveActualValues = 8; //+32; // Speicherplatz 1 + 32 zeigt an dass noch kein Req. verschickt wurde
							programmLightSzenzeMode = 0;
							clearPanelLed(0b00000100, 0x15);
						}
						else
						{
							requestSceneRecall(7);
						}
						clearAllSceneLeds();
						setPanelLed(0b10000000, 0x16); // Wann wird LED wieder gel�scht?
						switchesHold[4] &= ~0b10000000;
					}
				}
				else
				{
					switchesHold[4] |= 0b10000000;
				}
			}

			if (cnt_10x10ms == 9)
			{
				if (~switches[5] & 0b00000001)
				{
					if (~switchesHold[5] & 0b00000001)
					{
						//
					}
					else
					{
						if (programmLightSzenzeMode == 1)
						{
							saveActualValues = 9; //+32; // Speicherplatz 1 + 32 zeigt an dass noch kein Req. verschickt wurde
							programmLightSzenzeMode = 0;
							clearPanelLed(0b00000100, 0x15);
						}
						else
						{
							requestSceneRecall(8);
						}
						clearAllSceneLeds();
						setPanelLed(0b01000000, 0x15); // Wann wird LED wieder gel�scht?
						switchesHold[5] &= ~0b00000001;
					}
				}
				else
				{
					switchesHold[5] |= 0b00000001;
				}

				if (~switches[5] & 0b00000010)
				{
					if (~switchesHold[5] & 0b00000010)
					{
						//
					}
					else
					{
						if (programmLightSzenzeMode == 1)
						{
							saveActualValues = 10; //+32; // Speicherplatz 1 + 32 zeigt an dass noch kein Req. verschickt wurde
							programmLightSzenzeMode = 0;
							clearPanelLed(0b00000100, 0x15);
						}
						else
						{
							requestSceneRecall(9);
						}
						clearAllSceneLeds();
						setPanelLed(0b10000000, 0x15); // Wann wird LED wieder gel�scht?
						switchesHold[5] &= ~0b00000010;
					}
				}
				else
				{
					switchesHold[5] |= 0b00000010;
				}
			}
		}
		else
		{
			cnt_10x10ms = 0;
		}
	}

	time++;

	TCNT0 = 194;
}

