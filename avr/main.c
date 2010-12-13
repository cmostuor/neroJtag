/* 
 * Copyright (C) 2010 Chris McClelland
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *  
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <string.h>
#include <LUFA/Version.h>
#include <LUFA/Drivers/USB/USB.h>
#include "desc.h"
#include "types.h"
#ifdef DEBUG
	#include "usart.h"
#endif

// The minimum number of bytes necessary to store this many bits
#define bitsToBytes(x) ((x>>3) + (x&7 ? 1 : 0))

// Bit masks on Port B for the four JTAG lines
enum {
	TCK = 0x80,
	TMS = 0x40,
	TDO = 0x20,
	TDI = 0x10
};

enum SendType {
	SEND_ZEROS,
	SEND_ONES,
	SEND_DATA,
	SEND_MASK
};

enum {
	IS_RESPONSE_NEEDED = 0,
	IS_LAST = 1,
	SEND_TYPE = 2
};

enum {
	CMD_CLOCK_DATA = 0x80,
	CMD_CLOCK_STATE_MACHINE,
	CMD_CLOCK
};

int main(void) {
	REGCR |= (1 << REGDIS);  // Disable regulator: using JTAG supply rail, which may be 3.3V.
	MCUSR &= ~(1 << WDRF);
	wdt_disable();
	clock_prescale_set(clock_div_1);
	PORTB = 0x00;
	DDRB = 0x00;
	#ifdef DEBUG
		usartInit(38400);
		usartSendFlashString(PSTR("NanduinoJTAG...\r"));
	#endif
	sei();
	USB_Init();
	for ( ; ; ) {
		USB_USBTask();
	}
}

void EVENT_USB_Device_Connect(void) {
	// Connected
	PORTB = 0x00;
	DDRB = TCK | TMS | TDI;
}

void EVENT_USB_Device_Disconnect(void) {
	// Disconnected
	DDRB = 0x00;
}

void EVENT_USB_Device_ConfigurationChanged(void) {
	if ( !(Endpoint_ConfigureEndpoint(IN_ENDPOINT_ADDR,
	                                  EP_TYPE_BULK,
	                                  ENDPOINT_DIR_IN,
	                                  ENDPOINT_SIZE,
	                                  ENDPOINT_BANK_SINGLE)) )
	{
		#ifdef DEBUG
			usartSendFlashString(PSTR("Failed to configure IN endpoint!\r"));
		#endif
	}
	if ( !(Endpoint_ConfigureEndpoint(OUT_ENDPOINT_ADDR,
	                                  EP_TYPE_BULK,
	                                  ENDPOINT_DIR_OUT,
	                                  ENDPOINT_SIZE,
	                                  ENDPOINT_BANK_SINGLE)) )
	{
		#ifdef DEBUG
			usartSendFlashString(PSTR("Failed to configure OUT endpoint!\r"));
		#endif
	}
}

void EVENT_USB_Device_UnhandledControlRequest(void) {
	switch ( USB_ControlRequest.bRequest ) {
		case CMD_CLOCK_DATA:
			if ( USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR) ) {
				uint32 numBits;
				uint8 sendType, flagByte, i, tdiByte, tdoByte;
				uint16 bitsRead, bitsRemaining;
				uint8 inBuffer[ENDPOINT_SIZE];
				uint8 *bufPtr;
				Endpoint_ClearSETUP();
				Endpoint_Read_Control_Stream_LE(&numBits, 4);
				sendType = USB_ControlRequest.wValue;
				sendType >>= SEND_TYPE;
				sendType &= SEND_MASK;
				flagByte = USB_ControlRequest.wValue;
				if ( sendType == SEND_DATA || flagByte & (1<<IS_RESPONSE_NEEDED) ) {
					Endpoint_ClearStatusStage();
				}
				while ( numBits ) {
					if ( sendType == SEND_DATA ) {
						Endpoint_SelectEndpoint(OUT_ENDPOINT_ADDR);
						while ( !Endpoint_IsOUTReceived() );
					}
					if ( numBits >= (ENDPOINT_SIZE<<3) ) {
						if ( sendType == SEND_DATA ) {
							Endpoint_Read_Stream_LE(inBuffer, ENDPOINT_SIZE);
						}
						bitsRead = ENDPOINT_SIZE<<3;
					} else {
						if ( sendType == SEND_DATA ) {
							Endpoint_Read_Stream_LE(inBuffer, bitsToBytes(numBits));
						}
						bitsRead = numBits;
					}

					#ifdef DEBUG
						if ( flagByte & (1<<IS_RESPONSE_NEEDED) ) {
							usartSendFlashString(PSTR("\rRequest("));
						} else {
							usartSendFlashString(PSTR("\rSend("));
						}
						if ( sendType == SEND_DATA ) {
							for ( i = 0; i < bitsToBytes(bitsRead); i++ ) {
								usartSendByteHex(inBuffer[i]);
							}
						} else {
							usartSendByte('0');
							usartSendByte('x');
							usartSendByteHex(sendType ? 0xFF : 0x00);
							usartSendFlashString(PSTR(", 0x"));
							usartSendByteHex(bitsToBytes(bitsRead));
						}
						usartSendByte(')');
					#endif

					if ( numBits == bitsRead && flagByte & (1<<IS_LAST) ) {
						// Last chunk needs Shift-DR exit on last bit
						bitsRemaining = bitsRead;
						bufPtr = inBuffer;
						while ( bitsRemaining ) {
							if ( sendType == SEND_DATA ) {
								tdiByte = *bufPtr;
							} else {
								tdiByte = sendType ? 0xFF : 0x00;
							}
							tdoByte = 0x00;
							i = 1;
							while ( i && bitsRemaining ) {
								bitsRemaining--;
								if ( !bitsRemaining ) {
									PORTB |= TMS; // Exit Shift-DR state on next clock
								}
								if ( tdiByte & 1 ) {
									PORTB |= TDI;
								} else {
									PORTB &= ~TDI;
								}
								tdiByte >>= 1;
								if ( PINB & TDO ) {
									tdoByte |= i;
								}
								PORTB |= TCK;
								PORTB &= ~TCK;
								i <<= 1;
							}
							*bufPtr++ = tdoByte;
						}
					} else {
						// Not last chunk or Shift-DR exit not required
						bitsRemaining = bitsRead;
						bufPtr = inBuffer;
						while ( bitsRemaining ) {
							if ( sendType == SEND_DATA ) {
								tdiByte = *bufPtr;
							} else {
								tdiByte = sendType ? 0xFF : 0x00;
							}
							tdoByte = 0x00;
							i = 1;
							while ( i && bitsRemaining ) {
								bitsRemaining--;
								if ( tdiByte & 1 ) {
									PORTB |= TDI;
								} else {
									PORTB &= ~TDI;
								}
								tdiByte >>= 1;
								if ( PINB & TDO ) {
									tdoByte |= i;
								}
								PORTB |= TCK;
								PORTB &= ~TCK;
								i <<= 1;
							}
							*bufPtr++ = tdoByte;
						}
					}
					Endpoint_ClearOUT();
					if ( flagByte & (1<<IS_RESPONSE_NEEDED) ) {
						Endpoint_SelectEndpoint(IN_ENDPOINT_ADDR);
						#ifdef DEBUG
							usartSendFlashString(PSTR(" = "));
							for ( i = 0; i < bitsToBytes(bitsRead); i++ ) {
								usartSendByteHex(inBuffer[i]);
							}
						#endif
						Endpoint_Write_Stream_LE(inBuffer, bitsToBytes(bitsRead));
						Endpoint_ClearIN();
					}
					numBits -= bitsRead;
				}
				if ( sendType != SEND_DATA && !(flagByte & (1<<IS_RESPONSE_NEEDED)) ) {
					Endpoint_ClearStatusStage();
				}
			}
			break;

		case CMD_CLOCK_STATE_MACHINE:
			if ( USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR) ) {
				uint8 transitionCount = USB_ControlRequest.wValue;
				uint32 data;
				Endpoint_ClearSETUP();
				Endpoint_Read_Control_Stream_LE(&data, 4);
				while ( transitionCount-- ) {
					if ( data & 1 ) {
						PORTB |= TMS;
					} else {
						PORTB &= ~TMS;
					}
					PORTB |= TCK;
					PORTB &= ~TCK;
					data >>= 1;
				}
				Endpoint_ClearStatusStage();
			}
			break;

		case CMD_CLOCK:
			if ( USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR) ) {
				uint32 numCycles = USB_ControlRequest.wValue;
				numCycles <<= 16;
				numCycles |= USB_ControlRequest.wIndex;
				Endpoint_ClearSETUP();
				while ( numCycles-- ) {
					PORTB |= TCK;
					PORTB &= ~TCK;
				}
				Endpoint_ClearStatusStage();
			}
			break;
	}
}
