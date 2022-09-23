#include <stdbool.h>
#include <stdint.h>
#include "Crc8.hpp"

static uint8_t crc8_lut[256];

void Crc8Init(const uint8_t poly)
{
	for (int idx = 0; idx < 256; ++idx) {
		uint8_t crc = idx;

		for (int shift = 0; shift < 8; ++shift) {
			crc = (crc << 1) ^ ((crc & 0x80) ? poly : 0);
		}

		crc8_lut[idx] = crc & 0xff;
	}
}

uint8_t Crc8Calc(const uint8_t *data, uint8_t size)
{
	uint8_t crc = 0;

	while (size--) {
		crc = crc8_lut[crc ^ *data++];
	}

	return crc;
}