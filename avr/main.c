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
#include "jtag.h"
#include "sync.h"
#include "vendorCommands.h"
#ifdef DEBUG
	#include "usart.h"
#endif

// Called once at startup
//
int main(void) {
	REGCR |= (1 << REGDIS);  // Disable regulator: using JTAG supply rail, which may be 3.3V.
	MCUSR &= ~(1 << WDRF);
	wdt_disable();
	clock_prescale_set(clock_div_1);
	PORTB = 0x00;
	jtagSetEnabled(false);
	#ifdef DEBUG
		usartInit(38400);
		usartSendFlashString(PSTR("NanduinoJTAG...\r"));
	#endif
	sei();
	USB_Init();
	for ( ; ; ) {
		USB_USBTask();
		if ( jtagIsShiftPending() ) {
			jtagShiftExecute();
		} else if ( syncIsEnabled() ) {
			syncExecute();
		}
	}
}

// Called when a vendor command is received
//
void EVENT_USB_Device_ControlRequest(void) {
	switch ( USB_ControlRequest.bRequest ) {
	case CMD_MODE_STATUS:
		if ( USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR) ) {
			// Enable sync mode if wValue is nonzero
			uint16 wBits = USB_ControlRequest.wValue;
			uint16 wMask = USB_ControlRequest.wIndex;
			Endpoint_ClearSETUP();
			if ( wMask & MODE_SYNC ) {
				// Sync mode does a loopback, so endpoints can be sync'd with the host software
				syncSetEnabled(wBits & MODE_SYNC ? true : false);
			} else if ( wMask & MODE_JTAG ) {
				// When in JTAG mode, the JTAG lines are driven; tristate otherwise
				jtagSetEnabled(wBits & MODE_JTAG ? true : false);
			}
			Endpoint_ClearStatusStage();
		} else if ( USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_VENDOR) ) {
			uint8 statusBuffer[16];
			statusBuffer[0] = 'N';                     // Magic bytes (my cat's name)
			statusBuffer[1] = 'E';
			statusBuffer[2] = 'M';
			statusBuffer[3] = 'I';
			statusBuffer[4] = 0x00;                    // Last operation diagnostic code
			statusBuffer[5] = 0x00;                    // Flags
			statusBuffer[6] = 0x24;                    // NeroJTAG endpoints
			statusBuffer[7] = 0x00;                    // CommFPGA endpoints
			statusBuffer[8] = 0x00;                    // Reserved
			statusBuffer[9] = 0x00;                    // Reserved
			statusBuffer[10] = 0x00;                   // Reserved
			statusBuffer[11] = 0x00;                   // Reserved
			statusBuffer[12] = 0x00;                   // Reserved
			statusBuffer[13] = 0x00;                   // Reserved
			statusBuffer[14] = 0x00;                   // Reserved
			statusBuffer[15] = 0x00;                   // Reserved
			Endpoint_ClearSETUP();
			Endpoint_Write_Control_Stream_LE(statusBuffer, 16);
			Endpoint_ClearStatusStage();
		}
		break;

	case CMD_JTAG_CLOCK_DATA:
		if ( USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR) ) {
			uint32 numBits;
			Endpoint_ClearSETUP();
			Endpoint_Read_Control_Stream_LE(&numBits, 4);
			jtagShiftBegin(numBits, (uint8)USB_ControlRequest.wValue);
			Endpoint_ClearStatusStage();
			// Now that numBits & flagByte are set, this operation will continue in mainLoop()...
		}
		break;

	case CMD_JTAG_CLOCK_FSM:
		if ( USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR) ) {
			uint32 bitPattern;
			const uint8 transitionCount = (uint8)USB_ControlRequest.wValue;
			Endpoint_ClearSETUP();
			Endpoint_Read_Control_Stream_LE(&bitPattern, 4);
			Endpoint_ClearStatusStage();
			jtagClockFSM(bitPattern, transitionCount);
		}
		break;

	case CMD_JTAG_CLOCK:
		if ( USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR) ) {
			uint32 numCycles = USB_ControlRequest.wIndex;
			Endpoint_ClearSETUP();
			numCycles <<= 16;
			numCycles |= USB_ControlRequest.wValue;
			jtagClocks(numCycles);
			Endpoint_ClearStatusStage();
		}
		break;
	}
}

void EVENT_USB_Device_Connect(void) {
	// Connected
	PORTB = 0x00;
}

void EVENT_USB_Device_Disconnect(void) { }

void EVENT_USB_Device_ConfigurationChanged(void) {
	if ( !(Endpoint_ConfigureEndpoint(OUT_ENDPOINT_ADDR,
	                                  EP_TYPE_BULK,
	                                  ENDPOINT_DIR_OUT,
	                                  ENDPOINT_SIZE,
	                                  ENDPOINT_BANK_SINGLE)) )
	{
		#ifdef DEBUG
			usartSendFlashString(PSTR("Failed to configure EP2OUT!\r"));
		#endif
	}
	if ( !(Endpoint_ConfigureEndpoint(IN_ENDPOINT_ADDR,
	                                  EP_TYPE_BULK,
	                                  ENDPOINT_DIR_IN,
	                                  ENDPOINT_SIZE,
	                                  ENDPOINT_BANK_SINGLE)) )
	{
		#ifdef DEBUG
			usartSendFlashString(PSTR("Failed to configure EP4IN!\r"));
		#endif
	}
}
