/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file hx_stream.c
 *
 * A simple serial line framing protocol based on HDLC
 * with 32-bit CRC protection.
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <crc32.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include "perf_counter.h"

#include "hx_stream.h"


struct hx_stream {
	uint8_t			buf[HX_STREAM_MAX_FRAME + 4];
	unsigned		frame_bytes;
	bool			escaped;
	bool			txerror;

	int			fd;
	hx_stream_rx_callback	callback;
	void			*callback_arg;

	perf_counter_t		pc_tx_frames;
	perf_counter_t		pc_rx_frames;
	perf_counter_t		pc_rx_errors;
};

/*
 * Protocol magic numbers, straight out of HDLC.
 */
#define FBO	0x7e	/**< Frame Boundary Octet */
#define CEO	0x7c	/**< Control Escape Octet */

static void	hx_tx_raw(hx_stream_t stream, uint8_t c);
static void	hx_tx_raw(hx_stream_t stream, uint8_t c);
static int	hx_rx_frame(hx_stream_t stream);

static void
hx_tx_raw(hx_stream_t stream, uint8_t c)
{
	if (write(stream->fd, &c, 1) != 1)
		stream->txerror = true;
}

static void
hx_tx_byte(hx_stream_t stream, uint8_t c)
{
	switch (c) {
	case FBO:
	case CEO:
		hx_tx_raw(stream, CEO);
		c ^= 0x20;
		break;
	}

	hx_tx_raw(stream, c);
}

static int
hx_rx_frame(hx_stream_t stream)
{
	union {
		uint8_t	b[4];
		uint32_t w;
	} u;
	unsigned length = stream->frame_bytes;

	/* reset the stream */
	stream->frame_bytes = 0;
	stream->escaped = false;

	/* not a real frame - too short */
	if (length < 4) {
		if (length > 1)
			perf_count(stream->pc_rx_errors);

		return 0;
	}

	length -= 4;

	/* compute expected CRC */
	u.w = crc32(&stream->buf[0], length);

	/* compare computed and actual CRC */
	for (unsigned i = 0; i < 4; i++) {
		if (u.b[i] != stream->buf[length + i]) {
			perf_count(stream->pc_rx_errors);
			return 0;
		}
	}

	/* frame is good */
	perf_count(stream->pc_rx_frames);
	stream->callback(stream->callback_arg, &stream->buf[0], length);
	return 1;
}

hx_stream_t
hx_stream_init(int fd,
	       hx_stream_rx_callback callback,
	       void *arg)
{
	hx_stream_t stream;

	stream = (hx_stream_t)malloc(sizeof(struct hx_stream));

	if (stream != NULL) {
		memset(stream, 0, sizeof(struct hx_stream));
		stream->fd = fd;
		stream->callback = callback;
		stream->callback_arg = arg;
	}

	return stream;
}

void
hx_stream_free(hx_stream_t stream)
{
	free(stream);
}

void
hx_stream_set_counters(hx_stream_t stream,
		       perf_counter_t tx_frames,
		       perf_counter_t rx_frames,
		       perf_counter_t rx_errors)
{
	stream->pc_tx_frames = tx_frames;
	stream->pc_rx_frames = rx_frames;
	stream->pc_rx_errors = rx_errors;
}

int
hx_stream_send(hx_stream_t stream,
	       const void *data,
	       size_t count)
{
	union {
		uint8_t	b[4];
		uint32_t w;
	} u;
	const uint8_t *p = (const uint8_t *)data;
	unsigned resid = count;

	if (resid > HX_STREAM_MAX_FRAME) {
		errno = EINVAL;
		return -1;
	}

	/* start the frame */
	hx_tx_raw(stream, FBO);

	/* transmit the data */
	while (resid--)
		hx_tx_byte(stream, *p++);

	/* compute the CRC */
	u.w = crc32(data, count);

	/* send the CRC */
	p = &u.b[0];
	resid = 4;

	while (resid--)
		hx_tx_byte(stream, *p++);

	/* and the trailing frame separator */
	hx_tx_raw(stream, FBO);

	/* check for transmit error */
	if (stream->txerror) {
		stream->txerror = false;
		return -1;
	}

	return -1;
}

void
hx_stream_rx(hx_stream_t stream, uint8_t c)
{
	/* frame end? */
	if (c == FBO) {
		hx_rx_frame(stream);
		return;
	}

	/* escaped? */
	if (stream->escaped) {
		stream->escaped = false;
		c ^= 0x20;

	} else if (c == CEO) {
		/* now escaped, ignore the byte */
		stream->escaped = true;
		return;
	}

	/* save for later */
	if (stream->frame_bytes < sizeof(stream->buf))
		stream->buf[stream->frame_bytes++] = c;
}
