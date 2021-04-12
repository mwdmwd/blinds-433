#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

struct wav
{
	uint32_t sampleRate;

	uint32_t samplesWritten;
	FILE *file;
};

struct wav *wav_open(char const *path, uint32_t sampleRate)
{
	struct wav *wav = calloc(1, sizeof(struct wav));
	if(!wav)
		goto out_noalloc;

	FILE *file = fopen(path, "w");
	if(!file)
		goto out_nofile;

	wav->sampleRate = sampleRate;

	wav->file = file;

	/* Write out RIFF header with 0 size */
	fwrite("RIFF\0\0\0\0WAVE", 1, 12, file);

	uint32_t temp;
	/* Write out fmt subchunk */
	fwrite("fmt \x10\x00\x00\x00\x01\x00\x01\x00", 1, 12, file);
	fwrite(&sampleRate, 4, 1, file);
	temp = sampleRate * 1 * 2; /* Byte rate = sampleRate * 1ch * 2bytes/sample */
	fwrite(&temp, 4, 1, file);
	temp = 2; /* Block align */
	fwrite(&temp, 2, 1, file);
	fwrite("\x10\x00", 2, 1, file); /* Bits per sample */

	/* Write out data subchunk header with 0 size */
	fwrite("data\0\0\0\0", 1, 8, file);

	return wav;

out_nofile:
	free(wav);
out_noalloc:
	return wav;
}

void wav_write_sample(struct wav *wav, int16_t sample)
{
	fwrite(&sample, 2, 1, wav->file);
	++wav->samplesWritten;
}

void wav_close(struct wav *wav)
{
	uint32_t temp;

	temp = wav->samplesWritten * 2; /* Subchunk 2 size */
	fseek(wav->file, 40, SEEK_SET);
	fwrite(&temp, 4, 1, wav->file);

	temp += 36; /* RIFF ChunkSize */
	fseek(wav->file, 4, SEEK_SET);
	fwrite(&temp, 4, 1, wav->file);

	fclose(wav->file);
	free(wav);
}

int16_t sin16(uint64_t time)
{
	return sin(time / 20.0 * M_PI * 2) * INT16_MAX;
}

_Bool isOn;
uint64_t simTime;
struct wav *outWav;

void _delay_us(uint64_t micros)
{
	while(micros--)
	{
		++simTime;
		wav_write_sample(outWav, sin16(simTime) * (isOn ? 1 : 0));
	}
}

#define BLINDS_SYMBOL_TIME 400 /* uS */

#define BLINDS_TRANSMIT_ON \
	do                     \
	{                      \
		isOn = true;       \
	} while(0)
#define BLINDS_TRANSMIT_OFF \
	do                      \
	{                       \
		isOn = false;       \
	} while(0)

void blinds_init(void)
{
	BLINDS_TRANSMIT_OFF;
}

void blinds_send_preamble(void)
{
	BLINDS_TRANSMIT_ON;
	_delay_us(BLINDS_SYMBOL_TIME * 12);

	BLINDS_TRANSMIT_OFF;
	_delay_us(BLINDS_SYMBOL_TIME * 4);
}

void blinds_send_bit(_Bool bit)
{
	printf("%c", '0' + bit);
	BLINDS_TRANSMIT_ON;
	if(bit)
	{
		_delay_us(BLINDS_SYMBOL_TIME * 2);
		BLINDS_TRANSMIT_OFF;
		_delay_us(BLINDS_SYMBOL_TIME);
	}
	else
	{
		_delay_us(BLINDS_SYMBOL_TIME);
		BLINDS_TRANSMIT_OFF;
		_delay_us(BLINDS_SYMBOL_TIME * 2);
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

uint8_t const blinds_command_values[] =
    {
        0x11, /* Up */
        0x55, /* Stop */
        0x33  /* Down */
};

void blinds_send_command(uint8_t address, blinds_command_t command)
{
	blinds_send_preamble();
	blinds_send_header();
	blinds_send_nibble(address);
	blinds_send_byte(blinds_command_values[command]);
}

int main(int argc, char *argv[])
{
	blinds_init();
	outWav = wav_open("output.wav", 44100);

	_delay_us(400);
	blinds_send_command(7, BLINDS_DOWN);
	_delay_us(400);

	wav_close(outWav);
	return 0;
}
