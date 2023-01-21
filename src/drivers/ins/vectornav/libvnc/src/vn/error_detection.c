#include "vn/error_detection.h"

uint8_t VnChecksum8_compute(char const *data, size_t length)
{
	uint8_t xorVal = 0;
	size_t i;

	for (i = 0; i < length; i++)
	{
		xorVal ^= data[i];
	}

	return xorVal;
}

uint16_t VnCrc16_compute(char const *data, size_t length)
{
	size_t i;
	uint16_t crc = 0;

	for (i = 0; i < length; i++)
	{
		crc = (uint16_t) (crc >> 8) | (crc << 8);

		crc ^= (uint8_t) data[i];
		crc ^= (uint16_t) (((uint8_t) (crc & 0xFF)) >> 4);
		crc ^= (uint16_t) ((crc << 8) << 4);
		crc ^= (uint16_t) (((crc & 0xFF) << 4) << 1);
	}

	return crc;
}
