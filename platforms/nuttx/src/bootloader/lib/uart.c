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
//#define SERIAL_TRACE

int g_uart_df = -1;

#if defined(SERIAL_TRACE)
int in = 0;
int out = 0;
char in_trace[254];
char out_trace[254];
#endif


void uart_cinit(void *config)
{
	char *init =  strdup(config ? (char *) config : "");
	char *pt = strtok((char *) init, ",");
	char *device = "/dev/ttyS0";
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

		g_uart_df = fd;
		struct termios t;

		tcgetattr(g_uart_df, &t);
		t.c_cflag &= ~(CSIZE | PARENB | CSTOPB);
		t.c_cflag |= (CS8);
		cfsetspeed(&t, baud);
		tcsetattr(g_uart_df, TCSANOW, &t);
	}

	free(init);
}

void uart_cfini(void)
{
	close(g_uart_df);
	g_uart_df = -1;
}
int uart_cin(void)
{
	int c = -1;

	if (g_uart_df >= 0) {
		char b;

		if (read(g_uart_df, &b, 1) == 1) {
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
void uart_cout(uint8_t *buf, unsigned len)
{
	uint32_t timeout = 1000;

	for (unsigned i = 0; i  < len; i++) {
		while (timeout) {
			if (write(g_uart_df, &buf[i], 1) == 1) {
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
