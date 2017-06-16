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
	/* RX state */
	uint8_t			rx_buf[HX_STREAM_MAX_FRAME + 4];
	unsigned		rx_frame_bytes;
	bool			rx_escaped;
	hx_stream_rx_callback	rx_callback;
	void			*rx_callback_arg;

	/* TX state */
	int			fd;
	bool			tx_error;
	const uint8_t		*tx_buf;
	unsigned		tx_resid;
	uint32_t		tx_crc;
	enum {
		TX_IDLE = 0,
		TX_SEND_START,
		TX_SEND_DATA,
		TX_SENT_ESCAPE,
		TX_SEND_END
	}			tx_state;

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
	if (write(stream->fd, &c, 1) != 1) {
		stream->tx_error = true;
	}
}

static int
hx_rx_frame(hx_stream_t stream)
{
	union {
		uint8_t	b[4];
		uint32_t w;
	} u;
	unsigned length = stream->rx_frame_bytes;

	/* reset the stream */
	stream->rx_frame_bytes = 0;
	stream->rx_escaped = false;

	/* not a real frame - too short */
	if (length < 4) {
		if (length > 1) {
			perf_count(stream->pc_rx_errors);
		}

		return 0;
	}

	length -= 4;

	/* compute expected CRC */
	u.w = crc32(&stream->rx_buf[0], length);

	/* compare computed and actual CRC */
	for (unsigned i = 0; i < 4; i++) {
		if (u.b[i] != stream->rx_buf[length + i]) {
			perf_count(stream->pc_rx_errors);
			return 0;
		}
	}

	/* frame is good */
	perf_count(stream->pc_rx_frames);
	stream->rx_callback(stream->rx_callback_arg, &stream->rx_buf[0], length);
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
		stream->rx_callback = callback;
		stream->rx_callback_arg = arg;
	}

	return stream;
}

void
hx_stream_free(hx_stream_t stream)
{
	/* free perf counters (OK if they are NULL) */
	perf_free(stream->pc_tx_frames);
	perf_free(stream->pc_rx_frames);
	perf_free(stream->pc_rx_errors);

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

void
hx_stream_reset(hx_stream_t stream)
{
	stream->rx_frame_bytes = 0;
	stream->rx_escaped = false;

	stream->tx_buf = NULL;
	stream->tx_resid = 0;
	stream->tx_state = TX_IDLE;
}

int
hx_stream_start(hx_stream_t stream,
		const void *data,
		size_t count)
{
	if (count > HX_STREAM_MAX_FRAME) {
		return -EINVAL;
	}

	stream->tx_buf = data;
	stream->tx_resid = count;
	stream->tx_state = TX_SEND_START;
	stream->tx_crc = crc32(data, count);
	return OK;
}

int
hx_stream_send_next(hx_stream_t stream)
{
	int c;

	/* sort out what we're going to send */
	switch (stream->tx_state) {

	case TX_SEND_START:
		stream->tx_state = TX_SEND_DATA;
		return FBO;

	case TX_SEND_DATA:
		c = *stream->tx_buf;

		switch (c) {
		case FBO:
		case CEO:
			stream->tx_state = TX_SENT_ESCAPE;
			return CEO;
		}

		break;

	case TX_SENT_ESCAPE:
		c = *stream->tx_buf ^ 0x20;
		stream->tx_state = TX_SEND_DATA;
		break;

	case TX_SEND_END:
		stream->tx_state = TX_IDLE;
		return FBO;

	case TX_IDLE:
	default:
		return -1;
	}

	/* if we are here, we have consumed a byte from the buffer */
	stream->tx_resid--;
	stream->tx_buf++;

	/* buffer exhausted */
	if (stream->tx_resid == 0) {
		uint8_t *pcrc = (uint8_t *)&stream->tx_crc;

		/* was the buffer the frame CRC? */
		if (stream->tx_buf == (pcrc + sizeof(stream->tx_crc))) {
			stream->tx_state = TX_SEND_END;

		} else {
			/* no, it was the payload - switch to sending the CRC */
			stream->tx_buf = pcrc;
			stream->tx_resid = sizeof(stream->tx_crc);
		}
	}

	return c;
}

int
hx_stream_send(hx_stream_t stream,
	       const void *data,
	       size_t count)
{
	int result;

	result = hx_stream_start(stream, data, count);

	if (result != OK) {
		return result;
	}

	int c;

	while ((c = hx_stream_send_next(stream)) >= 0) {
		hx_tx_raw(stream, c);
	}

	/* check for transmit error */
	if (stream->tx_error) {
		stream->tx_error = false;
		return -EIO;
	}

	perf_count(stream->pc_tx_frames);
	return OK;
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
	if (stream->rx_escaped) {
		stream->rx_escaped = false;
		c ^= 0x20;

	} else if (c == CEO) {
		/* now rx_escaped, ignore the byte */
		stream->rx_escaped = true;
		return;
	}

	/* save for later */
	if (stream->rx_frame_bytes < sizeof(stream->rx_buf)) {
		stream->rx_buf[stream->rx_frame_bytes++] = c;
	}
}
