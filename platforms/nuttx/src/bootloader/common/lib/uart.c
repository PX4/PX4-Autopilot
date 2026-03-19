/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "uart.h"
#include <string.h>
#include<stdio.h>
#include<fcntl.h>
#include<termios.h>
#include <stdlib.h>
//#define SERIAL_TRACE

static int g_uart_df[2] = {-1, -1};

#if defined(SERIAL_TRACE)
int in = 0;
int out = 0;
char in_trace[254];
char out_trace[254];
#endif


static void uart_cinit_port(unsigned port, void *config, const char *default_device)
{
	char *init =  strdup(config ? (char *) config : "");
	char *pt = strtok((char *) init, ",");
	char *device = (char *)default_device;
	int baud = 115200;

	if (pt != NULL) {
		device = pt;
		pt = strtok(NULL, ",");

		if (pt != NULL) {
			baud = atol(pt);
		}
	}

	int fd = open(device, O_RDWR | O_NONBLOCK);

	if (fd >= 0) {

		g_uart_df[port] = fd;
		struct termios t;

		tcgetattr(g_uart_df[port], &t);
		t.c_cflag &= ~(CSIZE | PARENB | CSTOPB);
		t.c_cflag |= (CS8);
		cfsetspeed(&t, baud);
		tcsetattr(g_uart_df[port], TCSANOW, &t);
	}

	free(init);
}

void uart_cinit(void *config)
{
	uart_cinit_port(0, config, "/dev/ttyS0");
}

void uart2_cinit(void *config)
{
	uart_cinit_port(1, config, "/dev/ttyS1");
}

static void uart_cfini_port(unsigned port)
{
	if (g_uart_df[port] >= 0) {
		close(g_uart_df[port]);
		g_uart_df[port] = -1;
	}
}

void uart_cfini(void)
{
	uart_cfini_port(0);
}

void uart2_cfini(void)
{
	uart_cfini_port(1);
}

static int uart_cin_port(unsigned port)
{
	int c = -1;

	if (g_uart_df[port] >= 0) {
		char b;

		if (read(g_uart_df[port], &b, 1) == 1) {
			c = b;
#if defined(SERIAL_TRACE)
			in_trace[in] = b;
			in++;
			in &= 255;
#endif
		}
	}

	return c;
}

int uart_cin(void)
{
	return uart_cin_port(0);
}

int uart2_cin(void)
{
	return uart_cin_port(1);
}

static void uart_cout_port(unsigned port, uint8_t *buf, unsigned len)
{
	if (g_uart_df[port] < 0) {
		return;
	}

	uint32_t timeout = 1000;

	for (unsigned i = 0; i  < len; i++) {
		while (timeout) {
			if (write(g_uart_df[port], &buf[i], 1) == 1) {
#if defined(SERIAL_TRACE)
				out_trace[out] = buf[i];
				out++;
				out &= 255;
#endif
				timeout = 1000;
				break;

			}  else {
				if (timeout-- == 0) {
					return;
				}
			}
		}
	}
}

void uart_cout(uint8_t *buf, unsigned len)
{
	uart_cout_port(0, buf, len);
}

void uart2_cout(uint8_t *buf, unsigned len)
{
	uart_cout_port(1, buf, len);
}
