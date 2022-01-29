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

#include "cdcacm.h"
#include <string.h>
#include<stdio.h>
#include<fcntl.h>
#include<termios.h>

#include <sys/types.h>
#include <sys/boardctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <debug.h>
#include "cdcacm.h"

//#define SERIAL_TRACE

int g_usb_df = -1;
char *g_device = 0;
char *g_init = 0;

#if defined(SERIAL_TRACE)
int in = 0;
int out = 0;
// must be power of 2 length
#define TRACE_SIZE      256
#define TRACE_WRAP(f)   ((f) &=(TRACE_SIZE-1))
char in_trace[TRACE_SIZE];
char out_trace[TRACE_SIZE];
#endif

void usb_cinit(void *config)
{
#if defined(SERIAL_TRACE)
	in = 0;
	out = 0;
	memset(in_trace, 0, sizeof(in_trace));
	memset(out_trace, 0, sizeof(out_trace));
#endif
	g_init =  strdup(config ? (char *) config : "");
	char *pt = strtok((char *) g_init, ",");

	if (pt != NULL) {
		g_device = pt;
	}

	g_usb_df = -1;
	int fd = open(g_device, O_RDWR | O_NONBLOCK);

	if (fd >= 0) {
		g_usb_df = fd;
		free(g_init);
	}
}
void usb_cfini(void)
{
	close(g_usb_df);
	g_usb_df = -1;
}
int usb_cin(void)
{
	int c = -1;

	if (g_usb_df < 0) {

		int fd = open(g_device, O_RDWR | O_NONBLOCK);

		if (fd < 0) {
			return c;
		}

		g_usb_df = fd;
		free(g_init);
	}

	char b;

	if (read(g_usb_df, &b, 1) == 1) {
		c = b;
#if defined(SERIAL_TRACE)
		in_trace[in++] = b;
		TRACE_WRAP(in);
#endif
	}

	return c;
}
void usb_cout(uint8_t *buf, unsigned len)
{
	write(g_usb_df, buf, len);
#if defined(SERIAL_TRACE)

	if (len > TRACE_SIZE) {
		len = TRACE_SIZE;
		out = 0;
	}

	for (unsigned i = 0; i < len; i++) {
		out_trace[out++] = buf[i];
		TRACE_WRAP(out);
	}

#endif
}
