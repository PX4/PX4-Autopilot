/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file uart8_test.cpp
 *
 * UART8 test command - debug UART8 transmission issues
 *
 * @author PX4 Development Team
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

static void debug_uart8_registers();

extern "C" __EXPORT int uart8_test_main(int argc, char *argv[]);

int uart8_test_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_INFO("Usage: uart8_test <message>");
		PX4_INFO("   or: uart8_test debug");
		return 1;
	}

	if (strcmp(argv[1], "debug") == 0) {
		debug_uart8_registers();
		return 0;
	}

	// Test UART8 transmission with provided message
	PX4_INFO("=== UART8 Test Command ===");

	// Check GPIO/clock state before test
	debug_uart8_registers();

	int fd = ::open("/dev/ttyS2", O_WRONLY | O_NOCTTY);

	if (fd < 0) {
		PX4_ERR("Failed to open /dev/ttyS2: %s", strerror(errno));
		return 1;
	}

	// Prepare message
	char message[256];
	snprintf(message, sizeof(message), "UART8_CMD_TEST: %s\n", argv[1]);

	// Check clock before write
	const uint32_t RCC_BASE = 0x58024400;
	volatile uint32_t *rcc_apb1lenr = (volatile uint32_t *)(RCC_BASE + 0xE8);
	uint32_t clock_before = (*rcc_apb1lenr >> 31) & 1;

	ssize_t bytes_written = ::write(fd, message, strlen(message));

	uint32_t clock_after = (*rcc_apb1lenr >> 31) & 1;

	PX4_INFO("Wrote %d bytes to UART8: '%s'", (int)bytes_written, message);
	PX4_INFO("Clock before/after write: %lu/%lu", clock_before, clock_after);

	::fsync(fd);
	::close(fd);

	// Check GPIO/clock state after test
	debug_uart8_registers();

	return 0;
}

static void debug_uart8_registers()
{
	// STM32H7 register addresses
	const uint32_t UART8_BASE = 0x40007c00;
	const uint32_t GPIOE_BASE = 0x58020000;
	const uint32_t RCC_BASE = 0x58024400;

	// Read registers
	volatile uint32_t *uart8_cr1 = (volatile uint32_t *)(UART8_BASE + 0x00);
	volatile uint32_t *uart8_isr = (volatile uint32_t *)(UART8_BASE + 0x1C);
	volatile uint32_t *gpioe_afrl = (volatile uint32_t *)(GPIOE_BASE + 0x20);
	volatile uint32_t *gpioe_moder = (volatile uint32_t *)(GPIOE_BASE + 0x00);
	volatile uint32_t *rcc_apb1lenr = (volatile uint32_t *)(RCC_BASE + 0xE8);

	uint32_t afrl_val = *gpioe_afrl;
	uint32_t pe0_af = (afrl_val >> 0) & 0xF;
	uint32_t pe1_af = (afrl_val >> 4) & 0xF;
	uint32_t moder_val = *gpioe_moder;
	uint32_t pe0_mode = (moder_val >> 0) & 3;
	uint32_t pe1_mode = (moder_val >> 2) & 3;
	uint32_t uart8_clock = (*rcc_apb1lenr >> 31) & 1;

	PX4_INFO("=== UART8 Debug ===");
	PX4_INFO("PE0: mode=%lu, af=%lu %s", pe0_mode, pe0_af, (pe0_af == 8) ? "(UART8_RX)" : "");
	PX4_INFO("PE1: mode=%lu, af=%lu %s", pe1_mode, pe1_af, (pe1_af == 8) ? "(UART8_TX)" : "");
	PX4_INFO("UART8 clock: %lu %s", uart8_clock, uart8_clock ? "(enabled)" : "(disabled)");
	PX4_INFO("UART8 CR1: 0x%08lx, ISR: 0x%08lx", *uart8_cr1, *uart8_isr);
}
