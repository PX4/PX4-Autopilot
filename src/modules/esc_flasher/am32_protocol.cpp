/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

/**
 * @file am32_protocol.cpp
 *
 * AM32 ESC bootloader protocol — bit-banged UART implementation.
 */

#include "am32_protocol.hpp"

#include <string.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/micro_hal.h>
#include <drivers/drv_hrt.h>

namespace am32 {

// Bootloader handshake packet (8 zero bytes + 0x0D + "BLHeli" + CRC)
static const uint8_t boot_init[] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x0D, 'B', 'L', 'H', 'e', 'l', 'i', 0xF4, 0x7D
};

uint16_t crc16(const uint8_t *buf, uint16_t len)
{
	uint16_t crc = 0;

	for (uint16_t i = 0; i < len; i++) {
		uint8_t xb = buf[i];

		for (uint8_t j = 0; j < 8; j++) {
			if (((xb & 0x01) ^ (crc & 0x0001)) != 0) {
				crc = (crc >> 1) ^ 0xA001;

			} else {
				crc >>= 1;
			}

			xb >>= 1;
		}
	}

	return crc;
}

int send_packet(uint32_t gpio, const uint8_t *tx_data, uint16_t tx_len,
		uint8_t *rx_data, uint8_t rx_len)
{
	// Build GPIO configurations from the port+pin value
	uint32_t gpio_out = (gpio & (GPIO_PORT_MASK | GPIO_PIN_MASK))
			    | GPIO_OUTPUT | GPIO_OUTPUT_SET | GPIO_PULLUP;
	uint32_t gpio_in  = (gpio & (GPIO_PORT_MASK | GPIO_PIN_MASK))
			    | GPIO_INPUT | GPIO_PULLUP;

	// Start as output HIGH
	px4_arch_configgpio(gpio_out);

	// --- critical section: interrupts off for timing accuracy ---
	irqstate_t irq_state = px4_enter_critical_section();

	// Transmit bytes (19200 8N1, LSB-first)
	for (uint16_t k = 0; k < tx_len; k++) {
		uint8_t byte = tx_data[k];

		// Start bit (LOW)
		px4_arch_gpiowrite(gpio_out, 0);
		hrt_abstime t = hrt_absolute_time();

		while (hrt_absolute_time() - t < BIT_TIME_US) {}

		// 8 data bits, LSB first
		for (uint8_t i = 0; i < 8; i++) {
			px4_arch_gpiowrite(gpio_out, (byte >> i) & 1);
			t = hrt_absolute_time();

			while (hrt_absolute_time() - t < BIT_TIME_US) {}
		}

		// Stop bit (HIGH)
		px4_arch_gpiowrite(gpio_out, 1);
		t = hrt_absolute_time();

		while (hrt_absolute_time() - t < BIT_TIME_US) {}
	}

	// Switch to input for receiving
	px4_arch_configgpio(gpio_in);

	// Receive response bytes
	uint8_t read_count = 0;

	if (rx_len > 0 && rx_data != nullptr) {
		for (uint8_t i = 0; i < rx_len; i++) {

			// Wait for start bit (line goes LOW)
			hrt_abstime t = hrt_absolute_time();

			while (px4_arch_gpioread(gpio_in)) {
				if (hrt_absolute_time() - t > RX_TIMEOUT_US) {
					// Timeout — restore output HIGH and bail
					px4_arch_configgpio(gpio_out);
					px4_leave_critical_section(irq_state);
					return ERR_TIMEOUT;
				}
			}

			// Half-bit delay to centre sampling window
			t = hrt_absolute_time();

			while (hrt_absolute_time() - t < HALF_BIT_TIME_US) {}

			// Read 8 data bits
			uint8_t byte = 0;

			for (uint8_t bit = 0; bit < 8; bit++) {
				t = hrt_absolute_time();

				while (hrt_absolute_time() - t < BIT_TIME_US) {}

				if (px4_arch_gpioread(gpio_in)) {
					byte |= (1 << bit);
				}
			}

			rx_data[read_count++] = byte;

			// Wait through stop bit
			t = hrt_absolute_time();

			while (hrt_absolute_time() - t < BIT_TIME_US) {}
		}
	}

	// Restore pin to output HIGH
	px4_arch_configgpio(gpio_out);

	// Inter-packet gap (8 bit-times)
	hrt_abstime t = hrt_absolute_time();

	while (hrt_absolute_time() - t < BIT_TIME_US * 8) {}

	px4_leave_critical_section(irq_state);

	return read_count;
}

int handshake(uint32_t gpio, uint8_t device_info[DEVICE_INFO_SIZE])
{
	int ret = send_packet(gpio, boot_init, sizeof(boot_init),
			      device_info, DEVICE_INFO_SIZE);

	if (ret == DEVICE_INFO_SIZE) {
		return OK;
	}

	return (ret < 0) ? ret : ERR_NACK;
}

int set_address(uint32_t gpio, uint16_t address)
{
	uint8_t cmd[6];
	cmd[0] = CMD_SET_ADDRESS;
	cmd[1] = 0x00;
	cmd[2] = (address >> 8) & 0xFF;   // high byte
	cmd[3] = address & 0xFF;          // low byte

	uint16_t crc = crc16(cmd, 4);
	cmd[4] = crc & 0xFF;
	cmd[5] = (crc >> 8) & 0xFF;

	uint8_t response;
	int ret = send_packet(gpio, cmd, sizeof(cmd), &response, 1);

	if (ret == 1 && response == ACK) {
		return OK;
	}

	return (ret < 0) ? ret : ERR_NACK;
}

int write_chunk(uint32_t gpio, const uint8_t *data, uint16_t len)
{
	if (len == 0 || len > MAX_CHUNK_SIZE || data == nullptr) {
		return ERR_INVALID;
	}

	// 1) CMD_SET_BUFFER — tell bootloader how many bytes follow
	uint8_t size_cmd[6];
	size_cmd[0] = CMD_SET_BUFFER;
	size_cmd[1] = 0x00;

	if (len == MAX_CHUNK_SIZE) {
		size_cmd[2] = 1;    // 256 encoded as (1, 0)
		size_cmd[3] = 0;

	} else {
		size_cmd[2] = 0;
		size_cmd[3] = (uint8_t)len;
	}

	uint16_t crc = crc16(size_cmd, 4);
	size_cmd[4] = crc & 0xFF;
	size_cmd[5] = (crc >> 8) & 0xFF;

	// SET_BUFFER has no response
	send_packet(gpio, size_cmd, sizeof(size_cmd), nullptr, 0);

	// 2) Send data + CRC — bootloader ACKs after receiving
	uint8_t buf[MAX_CHUNK_SIZE + 2];
	memcpy(buf, data, len);
	crc = crc16(data, len);
	buf[len]     = crc & 0xFF;
	buf[len + 1] = (crc >> 8) & 0xFF;

	uint8_t response;
	int ret = send_packet(gpio, buf, len + 2, &response, 1);

	if (ret != 1 || response != ACK) {
		return (ret < 0) ? ret : ERR_NACK;
	}

	// 3) CMD_PROG_FLASH — commit buffer to flash
	uint8_t prog_cmd[4];
	prog_cmd[0] = CMD_PROG_FLASH;
	prog_cmd[1] = 0x00;
	crc = crc16(prog_cmd, 2);
	prog_cmd[2] = crc & 0xFF;
	prog_cmd[3] = (crc >> 8) & 0xFF;

	ret = send_packet(gpio, prog_cmd, sizeof(prog_cmd), &response, 1);

	if (ret == 1 && response == ACK) {
		return OK;
	}

	return (ret < 0) ? ret : ERR_NACK;
}

int read_flash(uint32_t gpio, uint8_t *buf, uint16_t len)
{
	if (len == 0 || buf == nullptr) {
		return ERR_INVALID;
	}

	// Address must already be set via set_address()
	uint8_t cmd[4];
	cmd[0] = CMD_READ_FLASH;
	cmd[1] = (uint8_t)len;

	uint16_t crc = crc16(cmd, 2);
	cmd[2] = crc & 0xFF;
	cmd[3] = (crc >> 8) & 0xFF;

	int ret = send_packet(gpio, cmd, sizeof(cmd), buf, (uint8_t)len);

	if (ret == (int)len) {
		return OK;
	}

	return (ret < 0) ? ret : ERR_NACK;
}

int run_app(uint32_t gpio)
{
	static const uint8_t cmd[] = {0x00, 0x00, 0x00, 0x00};

	// No response expected
	send_packet(gpio, cmd, sizeof(cmd), nullptr, 0);

	return OK;
}

} // namespace am32
