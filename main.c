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
 * c - action (01 -> open, 10 -> close, 11 -> stop)
 * a - address, 0 for broadcast
 */

volatile uint8_t requestedCommand;

ISR(USART_RX_vect)
{
	requestedCommand = UDR0;
}

uint8_t uart_command_to_blinds_address(uint8_t uartCommand)
{
	return uartCommand & 0x3f;
}

blinds_action_t uart_command_to_blinds_action(uint8_t uartCommand)
{
	switch((uartCommand & 0xc0) >> 6)
	{
		case 1:
			return BLINDS_UP;
		case 2:
			return BLINDS_DOWN;
		default:
			return BLINDS_STOP;
	}
}

uint8_t blinds_address_to_uart_command(uint8_t blindsAddress)
{
	return blindsAddress & 0x3f;
}

uint8_t blinds_action_to_uart_command(blinds_action_t blindsAction)
{
	switch(blindsAction)
	{
		case BLINDS_UP:
			return 1 << 6;
		case BLINDS_STOP:
			return 3 << 6;
		case BLINDS_DOWN:
			return 2 << 6;
		default:
			return 0;
	}
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

	/* Configure serial port for 8N1, TX + RX with receive completed intr. */
	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
	UCSR0B |= (1 << RXCIE0) | (1 << TXEN0) | (1 << RXEN0);

	/* Enable interrupts */
	sei();

	blinds_init();

	for(;;)
	{
		if(requestedCommand) /* A new transmission request has arrived over UART */
		{
			uint8_t address = uart_command_to_blinds_address(requestedCommand);
			blinds_action_t action = uart_command_to_blinds_action(requestedCommand);

			blinds_rx_disable(); /* Disable reception while transmitting */
			blinds_send_command(address, action);
			blinds_rx_enable();

			requestedCommand = 0;
		}

		if(bitNr == BLINDS_PACKET_RX_BITS) /* A new command has been received over the air */
		{
			if(blinds_is_valid_header((uint8_t *)bitBuffer))
			{
				uint8_t address = blinds_get_address_from_packet((uint8_t *)bitBuffer);
				blinds_action_t action = blinds_get_action_from_packet((uint8_t *)bitBuffer);
				if(action != BLINDS_INVALID_ACTION)
				{
					uint8_t uartCommand = 0;
					uartCommand |= blinds_address_to_uart_command(address);
					uartCommand |= blinds_action_to_uart_command(action);

					UDR0 = uartCommand;
					while(!(UCSR0A & (1 << UDRE0)))
						; /* Wait for transmission to finish */
				}
			}

			bitNr = 0; /* Allow the ISR to read a new packet */
		}
	}

	return 0;
}