#include <stdbool.h>
#include <stdint.h>

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#define BAUD 57600
#include <util/setbaud.h>

#include "blinds.h"

/* Serial protocol: one-byte
 * ccaaaaaa
 * 76543210
 * c - command (01 -> open, 10 -> close, 11 -> stop)
 * a - address, 0 for broadcast
 */

volatile uint8_t requestedCommand;

ISR(USART_RX_vect)
{
	requestedCommand = UDR0;
}

int main(void)
{
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;
#if USE_2X
	UCSR0A |= (1 << U2X0);
#else
	UCSR0A &= ~(1 << U2X0);
#endif

	/* Configure serial port for 8N1, receive-only with receive completed intr. */
	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
	UCSR0B |= (1 << RXCIE0) | (1 << RXEN0);

	/* Enable interrupts */
	sei();

	blinds_init();

	for(;;)
	{
		if(requestedCommand) /* A new transmission request has arrived over UART */
		{
			uint8_t address = requestedCommand & 0x3f;
			uint8_t command = (requestedCommand & 0xc0) >> 6;

			switch(command)
			{
				case 1:
					command = BLINDS_UP;
					break;
				case 2:
					command = BLINDS_DOWN;
					break;
				default:
					command = BLINDS_STOP;
					break;
			}

			blinds_send_command(address, command);
			requestedCommand = 0;
		}

		if(bitNr == BLINDS_PACKET_RX_BITS) /* A new command has been received over the air */
		{
			if(blinds_is_valid_header((uint8_t *)bitBuffer))
			{
				/* TODO: handle packet */
			}

			bitNr = 0; /* Allow the ISR to read a new packet */
		}
	}

	return 0;
}