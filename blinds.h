#ifndef BLINDS_H_235237862
#define BLINDS_H_235237862

#include <stdint.h>
#include <stdbool.h>

#define BLINDS_TRANSMISSION_REPEATS 5
#define BLINDS_TX_DDR DDRB
#define BLINDS_TX_PORT PORTB
#define BLINDS_TX_BIT (1 << 1)

#define BLINDS_SYMBOL_TIME 400 /* uS */
#define BLINDS_US_PER_TIMER_TICK 4
#define BLINDS_GRACE_TICKS 40

#define BLINDS_PACKET_RX_BITS 40

typedef enum
{
	BLINDS_UP,
	BLINDS_STOP,
	BLINDS_DOWN,
	BLINDS_INVALID_ACTION
} blinds_action_t;

extern volatile uint8_t bitBuffer[BLINDS_PACKET_RX_BITS / 8]; /* The packet is placed here */
extern volatile uint8_t bitNr;                                /* A full packet is ready when this == BLINDS_PACKET_RX_BITS */

void blinds_init(void);
void blinds_send_command(uint8_t address, blinds_action_t command);

void blinds_rx_enable(void);
void blinds_rx_disable(void);

_Bool blinds_is_valid_header(uint8_t const *header);
blinds_action_t blinds_get_action_from_packet(uint8_t const *packet);
uint8_t blinds_get_address_from_packet(uint8_t const *packet);

#define BLINDS_PREAMBLE_ON_TIME (BLINDS_SYMBOL_TIME * 12)
#define BLINDS_PREAMBLE_OFF_TIME (BLINDS_SYMBOL_TIME * 4)

#define BLINDS_BIT0_ON_TIME BLINDS_SYMBOL_TIME
#define BLINDS_BIT0_OFF_TIME (BLINDS_SYMBOL_TIME * 2)
#define BLINDS_BIT1_ON_TIME (BLINDS_SYMBOL_TIME * 2)
#define BLINDS_BIT1_OFF_TIME BLINDS_SYMBOL_TIME

#define BLINDS_US_TO_TICKS(t) ((int)((float)t / BLINDS_US_PER_TIMER_TICK) - BLINDS_GRACE_TICKS)

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

#endif