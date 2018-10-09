/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "SMBus.hpp"

SMBus::SMBus(int bus_num, uint16_t address) :
	I2C("BATT_SMBUS_I2C", nullptr, bus_num, address, 100000)
{
	init();
}

SMBus::~SMBus()
{
}

int SMBus::read_word(const uint8_t cmd_code, void *data)
{
	// 2 data bytes + pec byte
	int result = transfer(&cmd_code, 1, (uint8_t *)data, 3);

	if (result == PX4_OK) {
		// Check PEC.
		uint8_t addr = get_device_address() << 1;
		uint8_t full_data_packet[5];
		full_data_packet[0] = addr | 0x00;
		full_data_packet[1] = cmd_code;
		full_data_packet[2] = addr | 0x01;
		memcpy(&full_data_packet[3], data, 2);

		uint8_t pec = get_pec(full_data_packet, sizeof(full_data_packet) / sizeof(full_data_packet[0]));

		if (pec != ((uint8_t *)data)[2]) {
			result = -EINVAL;
		}
	}

	return result;
}

int SMBus::block_read(const uint8_t cmd_code, void *data, const uint8_t length, bool use_pec)
{
	unsigned byte_count = 0;
	// Length of data (32max). byte_count(1), cmd_code(2), pec(1) (optional)
	uint8_t rx_data[32 + 4];

	int result = transfer(&cmd_code, 1, (uint8_t *)rx_data, length + 2);

	byte_count = rx_data[0];

	// addr1, addr2, byte_count,
	memcpy(data, &rx_data[1], byte_count);

	// addr(wr), cmd_code, addr(r), byte_count, rx_data[]
	uint8_t device_address = get_device_address();
	uint8_t full_data_packet[32 + 4] = {};

	full_data_packet[0] = (device_address << 1) | 0x00;
	full_data_packet[1] = cmd_code;
	full_data_packet[2] = (device_address << 1) | 0x01;
	full_data_packet[3] = byte_count;

	memcpy(&full_data_packet[4], &rx_data[1], byte_count);

	uint8_t pec = get_pec(full_data_packet, byte_count + 4);

	// First byte is byte count, followed by data.
	if (pec != ((uint8_t *)rx_data)[byte_count + 1]) {
		result = -EINVAL;
	}

	return result;
}

int SMBus::block_write(const uint8_t cmd_code, void *data, uint8_t byte_count, bool use_pec)
{
	// cmd code, byte count, data[byte_count], pec (optional)
	uint8_t buf[32 + 3];

	buf[0] = cmd_code;
	buf[1] = (uint8_t)byte_count;
	memcpy(&buf[2], data, byte_count);

	if (use_pec) {
		uint8_t pec = get_pec(buf, byte_count + 2);
		buf[byte_count + 2] = pec;
		byte_count++;
	}

	unsigned i = 0;

	// If block_write fails, try up to 10 times.
	while (i < 10) {
		int result = transfer((uint8_t *)buf, byte_count + 2, nullptr, 0);

		if (result != PX4_OK) {
			i++;

			if (i == 10) {
				PX4_WARN("Block_write failed 10 times");
				result = -ENODATA;
			}
		}
	}

	return result;
}

uint8_t SMBus::get_pec(uint8_t *buff, const uint8_t len)
{
	// Initialise CRC to zero.
	uint8_t crc = 0;
	uint8_t shift_register = 0;
	bool invert_crc;

	// Calculate crc for each byte in the stream.
	for (uint8_t i = 0; i < len; i++) {
		// Load next data byte into the shift register
		shift_register = buff[i];

		// Calculate crc for each bit in the current byte.
		for (uint8_t j = 0; j < 8; j++) {
			invert_crc = (crc ^ shift_register) & 0x80;
			crc <<= 1;
			shift_register <<= 1;

			if (invert_crc) {
				crc ^= SMBUS_PEC_POLYNOMIAL;
			}
		}
	}

	return crc;
}