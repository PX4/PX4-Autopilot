/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: Anton Babushkin <anton.babushkin@me.com>
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
 * @file screen.c
 *
 * Using screen to read serial ports.
 *
 * @author Tim Dyer <>
 */

#include <px4_defines.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <systemlib/err.h>
#include <poll.h>
#include <termios.h>

__EXPORT int screen_main(int argc, char *argv[]);

int
open_uart(const char *device);


int screen_main(int argc, char *argv[])
{
	int uart = open_uart("/dev/ttyS2");

	//static const int timeout_ms = 1000;

	for (;;)
	{
		char c;
		ssize_t size = read(uart, &c, 1);
		if (size)
		{
			printf("%c", c);
		}

		/*
		struct pollfd fds;
		fds.fd = uart;
		fds.events = POLLIN;

		char c;

		if (poll(&fds, 1, timeout_ms) > 0) {
			read(uart, &c, 1);
			printf("%c", c);
		} else {
			warnx("UART timeout on TX/RX port");
			//return ERROR;
		}
		*/
	}

	return OK;
}

int
open_uart(const char *device)
{
	/* baud rate */
	static const speed_t speed = B1000000;

	/* open uart */
	const int uart = open(device, O_RDWR | O_NOCTTY);

	if (uart < 0) {
		err(1, "ERR: opening %s", device);
	}

	/* Back up the original uart configuration to restore it after exit */
	int termios_state;
	struct termios uart_config_original;

	if ((termios_state = tcgetattr(uart, &uart_config_original)) < 0) {
		close(uart);
		err(1, "ERR: %s: %d", device, termios_state);
	}

	/* Fill the struct for the new configuration */
	struct termios uart_config;
	tcgetattr(uart, &uart_config);

	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* Set baud rate */
	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		close(uart);
		err(1, "ERR: %s: %d (cfsetispeed, cfsetospeed)",
		    device, termios_state);
	}

	if ((termios_state = tcsetattr(uart, TCSANOW, &uart_config)) < 0) {
		close(uart);
		err(1, "ERR: %s (tcsetattr)", device);
	}

	return uart;
}
