#include <avr/interrupt.h>
#include <avr/io.h>
#include <string.h>
#include <util/delay.h>

#include "blinds.h"

uint8_t const blinds_packet_header_bytes[] = {0x3e, 0x35, 0x6e};
uint8_t const blinds_packet_header_final_nibble = 0xa;

uint8_t const blinds_command_values[] = {
    0x11, /* Up */
    0x55, /* Stop */
    0x33  /* Down */
};

static void blinds_send_preamble(void);
static void blinds_send_bit(_Bool bit);
static void blinds_send_nibble(uint8_t nibble);
static void blinds_send_byte(uint8_t byte);
static void blinds_send_header(void);

volatile _Bool inPacket;
volatile uint8_t bitBuffer[BLINDS_PACKET_RX_BITS / 8];
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

void blinds_init(void)
{
	BLINDS_TRANSMIT_OFF;
	BLINDS_TX_DDR |= BLINDS_TX_BIT;

	/* Configure Timer1 for input capture and enable the interrupt */
	TCCR1B |= (1 << ICES1) | (1 << ICNC1); /* Rising edge, noise canceler */
	TCCR1B |= (1 << CS11) | (1 << CS10);   /* /64 prescaler, 1 tick = 4 uS */
	TIMSK1 |= (1 << ICIE1) | (1 << TOIE1); /* Enable input capture and overflow intrs. */
}

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

_Bool blinds_is_valid_header(uint8_t const *header)
{
	if(!memcmp(header, blinds_packet_header_bytes, sizeof(blinds_packet_header_bytes)))
	{
		return (header[sizeof(blinds_packet_header_bytes)] & 0xf0) >> 4 == blinds_packet_header_final_nibble;
	}
	else
	{
		return false;
	}
}

static void blinds_send_preamble(void)
{
	BLINDS_TRANSMIT_ON;
	_delay_us(BLINDS_PREABMLE_ON_TIME);

	BLINDS_TRANSMIT_OFF;
	_delay_us(BLINDS_PREABMLE_OFF_TIME);
}

static void blinds_send_bit(_Bool bit)
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

static void blinds_send_nibble(uint8_t nibble)
{
	for(int8_t i = 3; i >= 0; --i)
	{
		blinds_send_bit(nibble & (1 << i));
	}
}

static void blinds_send_byte(uint8_t byte)
{
	blinds_send_nibble((byte & 0xf0) >> 4);
	blinds_send_nibble(byte & 0x0f);
}

static void blinds_send_header(void)
{
	for(uint8_t i = 0; i < sizeof(blinds_packet_header_bytes); ++i)
	{
		blinds_send_byte(blinds_packet_header_bytes[i]);
	}
	blinds_send_nibble(blinds_packet_header_final_nibble);
}