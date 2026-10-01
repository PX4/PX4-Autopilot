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
 * @file passthrough.c
 *
 * Transparent bridge between a USB CDC/ACM port and the secondary MCU's
 * bootloader UART.
 *
 * The secondary MCU has no USB; its bootloader speaks the PX4 protocol over
 * UART7 (wired to the primary, PE7/PE8). The primary bootloader exposes a
 * second USB CDC/ACM (/dev/ttyACM1, set up in usb.c) and, while it is otherwise
 * idle, shuttles bytes between that port and UART7. The host therefore talks to
 * the secondary's bootloader directly, through the primary's USB.
 *
 * board_bootloader_idle() is called from the shared bootloader's idle loop (see
 * bl.c, guarded by BOARD_HAS_PASSTHROUGH).
 */

#include "board_config.h"
#include "bl.h"

#include <stdbool.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <nuttx/serial/tioctl.h>

#define PASSTHROUGH_USB_DEVICE		"/dev/ttyACM1"	/* second CDC/ACM (if02) */
#define PASSTHROUGH_UART_DEVICE		"/dev/ttyS0"	/* UART7 to the secondary */
#define PASSTHROUGH_UART_BAUDRATE	2000000

static int g_usb_fd  = -1;
static int g_uart_fd = -1;

static int passthrough_open_uart(void)
{
	int fd = open(PASSTHROUGH_UART_DEVICE, O_RDWR | O_NONBLOCK);

	if (fd < 0) {
		return -1;
	}

	struct termios t;

	tcgetattr(fd, &t);

	t.c_cflag &= ~(CSIZE | PARENB | CSTOPB);

	t.c_cflag |= CS8;

	cfsetspeed(&t, PASSTHROUGH_UART_BAUDRATE);

	tcsetattr(fd, TCSANOW, &t);

	/* The primary<->secondary UART7 is wired TX-to-TX and RX-to-RX (not
	 * crossed over in hardware), so swap RX/TX on this (primary) end. */
	ioctl(fd, TIOCSSWAP, SER_SWAP_ENABLED);

	return fd;
}

/* Write the whole buffer, tolerating short non-blocking writes. Bounded so a
 * stalled sink (host or secondary not draining) can never hang the bootloader. */
static void passthrough_write(int fd, const uint8_t *buf, size_t len)
{
	size_t off = 0;
	unsigned spins = 10000;

	while (off < len && spins-- > 0) {
		ssize_t w = write(fd, buf + off, len - off);

		if (w > 0) {
			off += (size_t)w;
			spins = 10000;
		}
	}
}

bool board_bootloader_idle(void)
{
	if (g_usb_fd < 0) {
		g_usb_fd = open(PASSTHROUGH_USB_DEVICE, O_RDWR | O_NONBLOCK);
	}

	if (g_uart_fd < 0) {
		g_uart_fd = passthrough_open_uart();
	}

	if (g_usb_fd < 0 || g_uart_fd < 0) {
		return false;
	}

	bool active = false;
	uint8_t buf[64];
	ssize_t n;

	/* host -> secondary */
	n = read(g_usb_fd, buf, sizeof(buf));

	if (n > 0) {
		passthrough_write(g_uart_fd, buf, (size_t)n);
		active = true;
	}

	/* secondary -> host */
	n = read(g_uart_fd, buf, sizeof(buf));

	if (n > 0) {
		passthrough_write(g_usb_fd, buf, (size_t)n);
		active = true;
	}

	return active;
}

void board_bootloader_finalize(void)
{
	/* Closing the NuttX UART runs its shutdown and disables the UART7
	 * interrupt; otherwise the app would take an unexpected UART7 IRQ on the
	 * next byte from the secondary before it has configured the port. */
	if (g_uart_fd >= 0) {
		close(g_uart_fd);
		g_uart_fd = -1;
	}

	if (g_usb_fd >= 0) {
		close(g_usb_fd);
		g_usb_fd = -1;
	}
}
