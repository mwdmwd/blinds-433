#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#define BAUD 57600
#include <util/setbaud.h>

#define BLINDS_TRANSMISSION_REPEATS 5
#define BLINDS_TX_DDR DDRB
#define BLINDS_TX_PORT PORTB
#define BLINDS_TX_BIT (1 << 1)
#define BLINDS_SYMBOL_TIME 400 /* uS */

#define BLINDS_PREABMLE_ON_TIME (BLINDS_SYMBOL_TIME * 12)
#define BLINDS_PREABMLE_OFF_TIME (BLINDS_SYMBOL_TIME * 4)

#define BLINDS_BIT0_ON_TIME BLINDS_SYMBOL_TIME
#define BLINDS_BIT0_OFF_TIME (BLINDS_SYMBOL_TIME * 2)
#define BLINDS_BIT1_ON_TIME (BLINDS_SYMBOL_TIME * 2)
#define BLINDS_BIT1_OFF_TIME BLINDS_SYMBOL_TIME

#define BLINDS_US_PER_TIMER_TICK 4
#define BLINDS_GRACE_TICKS 40
#define BLINDS_US_TO_TICKS(t) ((int)((float)t / BLINDS_US_PER_TIMER_TICK) - BLINDS_GRACE_TICKS)

#define BLINDS_PACKET_RX_BITS 40

#define BLINDS_TRANSMIT_ON               \
	do                                   \
	{                                    \
		BLINDS_TX_PORT |= BLINDS_TX_BIT; \
	} while(0)
#define BLINDS_TRANSMIT_OFF               \
	do                                    \
	{                                     \
		BLINDS_TX_PORT &= ~BLINDS_TX_BIT; \
	} while(0)

void blinds_init(void)
{
	BLINDS_TRANSMIT_OFF;
	BLINDS_TX_DDR |= BLINDS_TX_BIT;

	/* Configure Timer1 for input capture and enable the interrupt */
	TCCR1B |= (1 << ICES1) | (1 << ICNC1); /* Rising edge, noise canceler */
	TCCR1B |= (1 << CS11) | (1 << CS10);   /* /64 prescaler, 1 tick = 4 uS */
	TIMSK1 |= (1 << ICIE1) | (1 << TOIE1); /* Enable input capture and overflow intrs. */
}

void blinds_send_preamble(void)
{
	BLINDS_TRANSMIT_ON;
	_delay_us(BLINDS_PREABMLE_ON_TIME);

	BLINDS_TRANSMIT_OFF;
	_delay_us(BLINDS_PREABMLE_OFF_TIME);
}

void blinds_send_bit(_Bool bit)
{
	BLINDS_TRANSMIT_ON;
	if(bit)
	{
		_delay_us(BLINDS_BIT1_ON_TIME);
		BLINDS_TRANSMIT_OFF;
		_delay_us(BLINDS_BIT1_OFF_TIME);
	}
	else
	{
		_delay_us(BLINDS_BIT0_ON_TIME);
		BLINDS_TRANSMIT_OFF;
		_delay_us(BLINDS_BIT0_OFF_TIME);
	}
}

void blinds_send_nibble(uint8_t nibble)
{
	for(int8_t i = 3; i >= 0; --i)
	{
		blinds_send_bit(nibble & (1 << i));
	}
}

void blinds_send_byte(uint8_t byte)
{
	blinds_send_nibble((byte & 0xf0) >> 4);
	blinds_send_nibble(byte & 0x0f);
}

uint8_t const blinds_packet_header_bytes[] = {0x3e, 0x35, 0x6e};
uint8_t const blinds_packet_header_finalnibble = 0xa;

void blinds_send_header(void)
{
	for(uint8_t i = 0; i < sizeof(blinds_packet_header_bytes); ++i)
	{
		blinds_send_byte(blinds_packet_header_bytes[i]);
	}
	blinds_send_nibble(blinds_packet_header_finalnibble);
}

typedef enum
{
	BLINDS_UP,
	BLINDS_STOP,
	BLINDS_DOWN
} blinds_command_t;

uint8_t const blinds_command_values[] = {
    0x11, /* Up */
    0x55, /* Stop */
    0x33  /* Down */
};

void blinds_send_command(uint8_t address, blinds_command_t command)
{
	for(uint8_t i = 0; i < BLINDS_TRANSMISSION_REPEATS; ++i)
	{
		blinds_send_preamble();
		blinds_send_header();
		blinds_send_nibble(address);
		blinds_send_byte(blinds_command_values[command]);
	}
}

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

volatile _Bool inPacket;
volatile uint8_t bitBuffer[8];
volatile uint8_t bitNr;

ISR(TIMER1_CAPT_vect)
{
	_Bool wasRising = (TCCR1B & (1 << ICES1));
	uint16_t highTime;

	if(wasRising)
	{
		TCNT1 = 0; /* Reset timer, we'll read it on the next falling edge */
	}
	else if((highTime = TCNT1) > BLINDS_US_TO_TICKS(BLINDS_PREABMLE_ON_TIME) && bitNr < BLINDS_PACKET_RX_BITS)
	{
		inPacket = true; /* A preamble was just received, prepare for data */
		bitNr = 0;
	}
	else if(inPacket)
	{
		if(highTime > BLINDS_US_TO_TICKS(BLINDS_BIT1_ON_TIME))
		{
			bitBuffer[bitNr / 8] |= (1 << (7 - bitNr % 8));
		}
		else
		{
			bitBuffer[bitNr / 8] &= ~(1 << (7 - bitNr % 8));
		}

		if(++bitNr >= BLINDS_PACKET_RX_BITS)
		{
			inPacket = false; /* Finished reading packet */
		}
	}

	TCCR1B ^= (1 << ICES1); /* Select opposite edge */
}

ISR(TIMER1_OVF_vect)
{
	if(bitNr < BLINDS_PACKET_RX_BITS) /* Don't let a fully-read packet get destroyed */
	{
		inPacket = false;
		bitNr = 0;
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

	/* Configure serial port for 8N1, receive-only with receive completed intr. */
	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
	UCSR0B |= (1 << RXCIE0) | (1 << RXEN0);

	/* Enable interrupts */
	sei();

	blinds_init();

	for(;;)
	{
		if(requestedCommand)
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
			if(!memcmp((uint8_t *)bitBuffer, blinds_packet_header_bytes, sizeof(blinds_packet_header_bytes)))
			{
				/* TODO: also compare the final nibble? */
				/* TODO: handle packet */
			}

			bitNr = 0; /* Allow the ISR to read a new packet */
		}
	}

	return 0;
}