/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file drv_bitbang_uart.h
 *
 * Architecture-agnostic interface for bit-bang UART.
 *
 * Currently implemented for STM32F7/H7 in:
 *   platforms/nuttx/src/px4/stm/stm32_common/bitbang_uart/
 *
 * NOTE: Currently only single-wire (half-duplex) operation is implemented.
 * The single_wire flag is provided in the API to make this explicit and to
 * allow future extension to dual-wire (full-duplex) operation.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include <px4_platform_common/defines.h>

__BEGIN_DECLS

/**
 * Initialize bit-bang UART
 *
 * @param baudrate    Baudrate (typically 19200)
 * @param single_wire True for single-wire half-duplex (TX and RX share one GPIO).
 *                    False for dual-wire full-duplex (separate TX/RX GPIOs) — not yet implemented.
 * @return 0 on success, negative error code on failure
 */
int bitbang_uart_init(uint32_t baudrate, bool single_wire);

/**
 * Deinitialize bit-bang UART
 *
 * @return 0 on success, negative error code on failure
 */
int bitbang_uart_deinit(void);

/**
 * Transmit a single byte
 *
 * @param channel Timer/GPIO channel
 * @param byte    Byte to transmit
 * @return 0 on success, negative error code on failure
 */
int bitbang_uart_write_byte(uint8_t channel, uint8_t byte);

/**
 * Transmit multiple bytes
 *
 * @param channel Timer/GPIO channel
 * @param data    Pointer to data buffer
 * @param length  Number of bytes to transmit
 * @return Number of bytes transmitted, or negative error code on failure
 */
int bitbang_uart_write(uint8_t channel, const uint8_t *data, size_t length);

/**
 * Read received bytes (blocking with timeout)
 *
 * @param channel    Timer/GPIO channel
 * @param data       Pointer to buffer for received data
 * @param length     Number of bytes to read
 * @param timeout_us Timeout in microseconds (0 = block until length bytes received)
 * @return Number of bytes actually read
 */
int bitbang_uart_read(uint8_t channel, uint8_t *data, size_t length, uint32_t timeout_us);

__END_DECLS
