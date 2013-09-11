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
 * @file sbus.c
 *
 * Serial protocol decoder for the Futaba S.bus protocol.
 */

#include <nuttx/config.h>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include <systemlib/ppm_decode.h>

#include <drivers/drv_hrt.h>

#define DEBUG
#include "px4io.h"
#include "protocol.h"
#include "debug.h"

#define SBUS_FRAME_SIZE		25
#define SBUS_INPUT_CHANNELS	16

static int sbus_fd = -1;

static hrt_abstime last_rx_time;
static hrt_abstime last_frame_time;

static uint8_t	frame[SBUS_FRAME_SIZE];

static unsigned partial_frame_count;

unsigned sbus_frame_drops;

static bool sbus_decode(hrt_abstime frame_time, uint16_t *values, uint16_t *num_values, uint16_t max_channels);

int
sbus_init(const char *device)
{
	if (sbus_fd < 0)
		sbus_fd = open(device, O_RDONLY | O_NONBLOCK);

	if (sbus_fd >= 0) {
		struct termios t;

		/* 100000bps, even parity, two stop bits */
		tcgetattr(sbus_fd, &t);
		cfsetspeed(&t, 100000);
		t.c_cflag |= (CSTOPB | PARENB);
		tcsetattr(sbus_fd, TCSANOW, &t);

		/* initialise the decoder */
		partial_frame_count = 0;
		last_rx_time = hrt_absolute_time();

		debug("S.Bus: ready");

	} else {
		debug("S.Bus: open failed");
	}

	return sbus_fd;
}

bool
sbus_input(uint16_t *values, uint16_t *num_values, uint16_t max_channels)
{
	ssize_t		ret;
	hrt_abstime	now;

	/*
	 * The S.bus protocol doesn't provide reliable framing,
	 * so we detect frame boundaries by the inter-frame delay.
	 *
	 * The minimum frame spacing is 7ms; with 25 bytes at 100000bps
	 * frame transmission time is ~2ms.
	 *
	 * We expect to only be called when bytes arrive for processing,
	 * and if an interval of more than 3ms passes between calls,
	 * the first byte we read will be the first byte of a frame.
	 *
	 * In the case where byte(s) are dropped from a frame, this also
	 * provides a degree of protection. Of course, it would be better
	 * if we didn't drop bytes...
	 */
	now = hrt_absolute_time();

	if ((now - last_rx_time) > 3000) {
		if (partial_frame_count > 0) {
			sbus_frame_drops++;
			partial_frame_count = 0;
		}
	}

	/*
	 * Fetch bytes, but no more than we would need to complete
	 * the current frame.
	 */
	ret = read(sbus_fd, &frame[partial_frame_count], SBUS_FRAME_SIZE - partial_frame_count);

	/* if the read failed for any reason, just give up here */
	if (ret < 1)
		return false;

	last_rx_time = now;

	/*
	 * Add bytes to the current frame
	 */
	partial_frame_count += ret;

	/*
	 * If we don't have a full frame, return
	 */
	if (partial_frame_count < SBUS_FRAME_SIZE)
		return false;

	/*
	 * Great, it looks like we might have a frame.  Go ahead and
	 * decode it.
	 */
	partial_frame_count = 0;
	return sbus_decode(now, values, num_values, max_channels);
}

/*
 * S.bus decoder matrix.
 *
 * Each channel value can come from up to 3 input bytes. Each row in the
 * matrix describes up to three bytes, and each entry gives:
 *
 * - byte offset in the data portion of the frame
 * - right shift applied to the data byte
 * - mask for the data byte
 * - left shift applied to the result into the channel value
 */
struct sbus_bit_pick {
	uint8_t byte;
	uint8_t rshift;
	uint8_t mask;
	uint8_t lshift;
};
static const struct sbus_bit_pick sbus_decoder[SBUS_INPUT_CHANNELS][3] = {
	/*  0 */ { { 0, 0, 0xff, 0}, { 1, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
	/*  1 */ { { 1, 3, 0x1f, 0}, { 2, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
	/*  2 */ { { 2, 6, 0x03, 0}, { 3, 0, 0xff, 2}, { 4, 0, 0x01, 10} },
	/*  3 */ { { 4, 1, 0x7f, 0}, { 5, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
	/*  4 */ { { 5, 4, 0x0f, 0}, { 6, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
	/*  5 */ { { 6, 7, 0x01, 0}, { 7, 0, 0xff, 1}, { 8, 0, 0x03,  9} },
	/*  6 */ { { 8, 2, 0x3f, 0}, { 9, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
	/*  7 */ { { 9, 5, 0x07, 0}, {10, 0, 0xff, 3}, { 0, 0, 0x00,  0} },
	/*  8 */ { {11, 0, 0xff, 0}, {12, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
	/*  9 */ { {12, 3, 0x1f, 0}, {13, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
	/* 10 */ { {13, 6, 0x03, 0}, {14, 0, 0xff, 2}, {15, 0, 0x01, 10} },
	/* 11 */ { {15, 1, 0x7f, 0}, {16, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
	/* 12 */ { {16, 4, 0x0f, 0}, {17, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
	/* 13 */ { {17, 7, 0x01, 0}, {18, 0, 0xff, 1}, {19, 0, 0x03,  9} },
	/* 14 */ { {19, 2, 0x3f, 0}, {20, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
	/* 15 */ { {20, 5, 0x07, 0}, {21, 0, 0xff, 3}, { 0, 0, 0x00,  0} }
};

static bool
sbus_decode(hrt_abstime frame_time, uint16_t *values, uint16_t *num_values, uint16_t max_values)
{
	/* check frame boundary markers to avoid out-of-sync cases */
	if ((frame[0] != 0x0f) || (frame[24] != 0x00)) {
		sbus_frame_drops++;
		return false;
	}

	/* if the failsafe or connection lost bit is set, we consider the frame invalid */
	if ((frame[23] & (1 << 2)) && /* signal lost */
	    (frame[23] & (1 << 3))) { /* failsafe */

		/* actively announce signal loss */
		*values = 0;
		return false;
	}

	/* we have received something we think is a frame */
	last_frame_time = frame_time;

	unsigned chancount = (max_values > SBUS_INPUT_CHANNELS) ?
			     SBUS_INPUT_CHANNELS : max_values;

	/* use the decoder matrix to extract channel data */
	for (unsigned channel = 0; channel < chancount; channel++) {
		unsigned value = 0;

		for (unsigned pick = 0; pick < 3; pick++) {
			const struct sbus_bit_pick *decode = &sbus_decoder[channel][pick];

			if (decode->mask != 0) {
				unsigned piece = frame[1 + decode->byte];
				piece >>= decode->rshift;
				piece &= decode->mask;
				piece <<= decode->lshift;

				value |= piece;
			}
		}

		/* convert 0-2048 values to 1000-2000 ppm encoding in a very sloppy fashion */
		values[channel] = (value / 2) + 998;
	}

	/* decode switch channels if data fields are wide enough */
	if (PX4IO_INPUT_CHANNELS > 17 && chancount > 15) {
		chancount = 18;

		/* channel 17 (index 16) */
		values[16] = (frame[23] & (1 << 0)) * 1000 + 998;
		/* channel 18 (index 17) */
		values[17] = (frame[23] & (1 << 1)) * 1000 + 998;
	}

	/* note the number of channels decoded */
	*num_values = chancount;

	return true;
}
