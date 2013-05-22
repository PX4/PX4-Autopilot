/****************************************************************************
 *
 *   Copyright (C) 2012,2013 PX4 Development Team. All rights reserved.
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
 * @file serial.c
 *
 * Serial communication for the PX4IO module.
 */

#include <stdint.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include <systemlib/hx_stream.h>

//#define DEBUG
#include "px4io.h"

static uint8_t		tx_buf[66];		/* XXX hardcoded magic number */

static hx_stream_t	if_stream;

static void		serial_callback(void *arg, const void *data, unsigned length);

void
interface_init(void)
{

	int fd = open("/dev/ttyS1", O_RDWR, O_NONBLOCK);
	if (fd < 0) {
		debug("serial fail");
		return;
	}

	/* configure serial port - XXX increase port speed? */
	struct termios t;
	tcgetattr(fd, &t);
	cfsetspeed(&t, 115200);
	t.c_cflag &= ~(CSTOPB | PARENB);
	tcsetattr(fd, TCSANOW, &t);

	/* allocate the HX stream we'll use for communication */
	if_stream = hx_stream_init(fd, serial_callback, NULL);

	/* XXX add stream stats counters? */

	debug("serial init");
}

void
interface_tick()
{
	/* process incoming bytes */
	hx_stream_rx(if_stream);
}

static void
serial_callback(void *arg, const void *data, unsigned length)
{
	const uint8_t *message = (const uint8_t *)data;

	/* malformed frame, ignore it */
	if (length < 2)
		return;

	/* it's a write operation, pass it to the register API */
	if (message[0] & PX4IO_PAGE_WRITE) {
		registers_set(message[0] & ~PX4IO_PAGE_WRITE, message[1],
			(const uint16_t *)&message[2],
			(length - 2) / 2);

		return;
	}

	/* it's a read - must contain length byte */
	if (length != 3)
		return;
	uint16_t *registers;
	unsigned count;

	tx_buf[0] = message[0];
	tx_buf[1] = message[1];

	/* get registers for response, send an empty reply on error */
	if (registers_get(message[0], message[1], &registers, &count) < 0)
		count = 0;

	/* fill buffer with message, limited by length */
#define TX_MAX		((sizeof(tx_buf) - 2) / 2)
	if (count > TX_MAX)
		count = TX_MAX;
	if (count > message[2])
		count = message[2];
	memcpy(&tx_buf[2], registers, count * 2);

	/* try to send the message */
	hx_stream_send(if_stream, tx_buf, count * 2 + 2);
}