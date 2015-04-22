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
 * @file hx_stream.h
 *
 * A simple serial line framing protocol based on HDLC
 * with 32-bit CRC protection.
 */

#ifndef _SYSTEMLIB_HX_STREAM_H
#define _SYSTEMLIB_HX_STREAM_H

#include <sys/types.h>

#include "perf_counter.h"

struct hx_stream;
typedef struct hx_stream *hx_stream_t;

#define HX_STREAM_MAX_FRAME	64

typedef void (* hx_stream_rx_callback)(void *arg, const void *data, size_t length);

__BEGIN_DECLS

/**
 * Allocate a new hx_stream object.
 *
 * @param fd		The file handle over which the protocol will
 *			communicate, or -1 if the protocol will use
 *			hx_stream_start/hx_stream_send_next.
 * @param callback	Called when a frame is received.
 * @param callback_arg	Passed to the callback.
 * @return		A handle to the stream, or NULL if memory could
 *			not be allocated.
 */
__EXPORT extern hx_stream_t	hx_stream_init(int fd,
		hx_stream_rx_callback callback,
		void *arg);

/**
 * Free a hx_stream object.
 *
 * @param stream	A handle returned from hx_stream_init.
 */
__EXPORT extern void		hx_stream_free(hx_stream_t stream);

/**
 * Set performance counters for the stream.
 *
 * Any counter may be set to NULL to disable counting that datum.
 *
 * @param stream	A handle returned from hx_stream_init.
 * @param tx_frames	Counter for transmitted frames.
 * @param rx_frames	Counter for received frames.
 * @param rx_errors	Counter for short and corrupt received frames.
 */
__EXPORT extern void		hx_stream_set_counters(hx_stream_t stream,
		perf_counter_t tx_frames,
		perf_counter_t rx_frames,
		perf_counter_t rx_errors);

/**
 * Reset a stream.
 *
 * Forces the local stream state to idle.
 *
 * @param stream	A handle returned from hx_stream_init.
 */
__EXPORT extern void		hx_stream_reset(hx_stream_t stream);

/**
 * Prepare to send a frame.
 *
 * Use this in conjunction with hx_stream_send_next to
 * set the frame to be transmitted.
 *
 * Use hx_stream_send() to write to the stream fd directly.
 *
 * @param stream	A handle returned from hx_stream_init.
 * @param data		Pointer to the data to send.
 * @param count		The number of bytes to send.
 * @return		Zero on success, -errno on error.
 */
__EXPORT extern int		hx_stream_start(hx_stream_t stream,
		const void *data,
		size_t count);

/**
 * Get the next byte to send for a stream.
 *
 * This requires that the stream be prepared for sending by
 * calling hx_stream_start first.
 *
 * @param stream	A handle returned from hx_stream_init.
 * @return		The byte to send, or -1 if there is
 *			nothing left to send.
 */
__EXPORT extern int		hx_stream_send_next(hx_stream_t stream);

/**
 * Send a frame.
 *
 * This function will block until all frame bytes are sent if
 * the descriptor passed to hx_stream_init is marked blocking,
 * otherwise it will return -1 (but may transmit a
 * runt frame at the same time).
 *
 * @todo Handling of non-blocking streams needs to be better.
 *
 * @param stream	A handle returned from hx_stream_init.
 * @param data		Pointer to the data to send.
 * @param count		The number of bytes to send.
 * @return		Zero on success, -errno on error.
 */
__EXPORT extern int		hx_stream_send(hx_stream_t stream,
		const void *data,
		size_t count);

/**
 * Handle a byte from the stream.
 *
 * @param stream	A handle returned from hx_stream_init.
 * @param c		The character to process.
 */
__EXPORT extern void		hx_stream_rx(hx_stream_t stream,
		uint8_t c);

__END_DECLS

#endif
