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
 * @file am32_protocol.hpp
 *
 * AM32 ESC bootloader protocol — bit-banged UART at 19200 baud.
 *
 * Extracted from Matt's proof-of-concept and cleaned up into a
 * function-based API for reuse by the esc_flasher module.
 */

#pragma once

#include <stdint.h>

namespace am32
{

// AM32 bootloader command bytes
static constexpr uint8_t CMD_RUN         = 0x00;
static constexpr uint8_t CMD_PROG_FLASH  = 0x01;
static constexpr uint8_t CMD_READ_FLASH  = 0x03;
static constexpr uint8_t CMD_SET_BUFFER  = 0xFE;
static constexpr uint8_t CMD_SET_ADDRESS = 0xFF;

// Bootloader ACK byte
static constexpr uint8_t ACK = 0x30;

// AM32 firmware memory layout (STM32 flash addresses)
static constexpr uint32_t FIRMWARE_ADDR     = 0x08001000;
static constexpr uint32_t FIRMWARE_TAG_ADDR = 0x08007BE0;
static constexpr uint16_t FIRMWARE_TAG_SIZE = 16;

// Protocol constants
static constexpr uint16_t MAX_CHUNK_SIZE    = 256;
static constexpr uint32_t BIT_TIME_US       = 52;   // ~19200 baud
static constexpr uint32_t HALF_BIT_TIME_US  = 26;
static constexpr uint32_t RX_TIMEOUT_US     = 100000; // 100 ms

// Device info from handshake response
static constexpr uint8_t DEVICE_INFO_SIZE = 9;

// Return codes
static constexpr int OK          =  0;
static constexpr int ERR_TIMEOUT = -1;
static constexpr int ERR_NACK    = -2;
static constexpr int ERR_CRC     = -3;
static constexpr int ERR_INVALID = -4;

/**
 * CRC-16 with polynomial 0xA001 (AM32 bootloader).
 */
uint16_t crc16(const uint8_t *buf, uint16_t len);

/**
 * Bit-banged UART send/receive on a GPIO pin.
 *
 * Enters a critical section (interrupts disabled) for the
 * duration of the transfer to maintain 19200-baud timing.
 *
 * @param gpio     GPIO configuration value (port + pin bits)
 * @param tx_data  Bytes to transmit
 * @param tx_len   Number of bytes to transmit
 * @param rx_data  Buffer for response (may be nullptr if rx_len == 0)
 * @param rx_len   Expected number of response bytes
 * @return         Number of bytes received, or negative error code
 */
int send_packet(uint32_t gpio, const uint8_t *tx_data, uint16_t tx_len,
		uint8_t *rx_data, uint8_t rx_len);

/**
 * Perform the bootloader handshake.
 *
 * Sends the AM32 boot-init sequence and reads back 9 bytes
 * of device information.
 *
 * @param gpio        Target ESC GPIO
 * @param device_info Output buffer for 9-byte device info
 * @return            OK or negative error code
 */
int handshake(uint32_t gpio, uint8_t device_info[DEVICE_INFO_SIZE]);

/**
 * Set the flash write/read address.
 *
 * @param gpio    Target ESC GPIO
 * @param address 16-bit flash address
 * @return        OK or negative error code
 */
int set_address(uint32_t gpio, uint16_t address);

/**
 * Write a chunk to flash.
 *
 * Sends CMD_SET_BUFFER + size, then data + CRC, then
 * CMD_PROG_FLASH and waits for ACK.  Caller must call
 * set_address() first.
 *
 * @param gpio  Target ESC GPIO
 * @param data  Data to write (max MAX_CHUNK_SIZE bytes)
 * @param len   Byte count
 * @return      OK or negative error code
 */
int write_chunk(uint32_t gpio, const uint8_t *data, uint16_t len);

/**
 * Read flash memory.
 *
 * Caller must call set_address() first.
 *
 * @param gpio  Target ESC GPIO
 * @param buf   Output buffer
 * @param len   Number of bytes to read
 * @return      OK or negative error code
 */
int read_flash(uint32_t gpio, uint8_t *buf, uint16_t len);

/**
 * Send the RUN_APP command so the ESC boots its application.
 *
 * @param gpio  Target ESC GPIO
 * @return      OK (always; no response expected)
 */
int run_app(uint32_t gpio);

} // namespace am32
