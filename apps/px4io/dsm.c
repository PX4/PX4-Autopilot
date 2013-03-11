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
 * @file dsm.c
 *
 * Serial protocol decoder for the Spektrum DSM* family of protocols.
 *
 * Decodes into the global PPM buffer and updates accordingly.
 */

#include <nuttx/config.h>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include <drivers/drv_hrt.h>

#define DEBUG

#include "px4io.h"

#define DSM_FRAME_SIZE		16
#define DSM_FRAME_CHANNELS	7

static int dsm_fd = -1;

static hrt_abstime last_rx_time;
static hrt_abstime last_frame_time;

static uint8_t	frame[DSM_FRAME_SIZE];

static unsigned partial_frame_count;
static unsigned	channel_shift;

unsigned dsm_frame_drops;

static bool dsm_decode_channel(uint16_t raw, unsigned shift, unsigned *channel, unsigned *value);
static void dsm_guess_format(bool reset);
static bool dsm_decode(hrt_abstime now, uint16_t *values, uint16_t *num_values);

int
dsm_init(const char *device)
{
	if (dsm_fd < 0)
		dsm_fd = open(device, O_RDONLY | O_NONBLOCK);

	if (dsm_fd >= 0) {
		struct termios t;

		/* 115200bps, no parity, one stop bit */
		tcgetattr(dsm_fd, &t);
		cfsetspeed(&t, 115200);
		t.c_cflag &= ~(CSTOPB | PARENB);
		tcsetattr(dsm_fd, TCSANOW, &t);

		/* initialise the decoder */
		partial_frame_count = 0;
		last_rx_time = hrt_absolute_time();

		/* reset the format detector */
		dsm_guess_format(true);

		debug("DSM: ready");

	} else {
		debug("DSM: open failed");
	}

	return dsm_fd;
}

bool
dsm_input(uint16_t *values, uint16_t *num_values)
{
	ssize_t		ret;
	hrt_abstime	now;

	/*
	 * The DSM* protocol doesn't provide any explicit framing,
	 * so we detect frame boundaries by the inter-frame delay.
	 *
	 * The minimum frame spacing is 11ms; with 16 bytes at 115200bps
	 * frame transmission time is ~1.4ms.
	 *
	 * We expect to only be called when bytes arrive for processing,
	 * and if an interval of more than 5ms passes between calls,
	 * the first byte we read will be the first byte of a frame.
	 *
	 * In the case where byte(s) are dropped from a frame, this also
	 * provides a degree of protection. Of course, it would be better
	 * if we didn't drop bytes...
	 */
	now = hrt_absolute_time();

	if ((now - last_rx_time) > 5000) {
		if (partial_frame_count > 0) {
			dsm_frame_drops++;
			partial_frame_count = 0;
		}
	}

	/*
	 * Fetch bytes, but no more than we would need to complete
	 * the current frame.
	 */
	ret = read(dsm_fd, &frame[partial_frame_count], DSM_FRAME_SIZE - partial_frame_count);

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
	if (partial_frame_count < DSM_FRAME_SIZE)
		return false;

	/*
	 * Great, it looks like we might have a frame.  Go ahead and
	 * decode it.
	 */
	partial_frame_count = 0;
	return dsm_decode(now, values, num_values);
}

static bool
dsm_decode_channel(uint16_t raw, unsigned shift, unsigned *channel, unsigned *value)
{

	if (raw == 0xffff)
		return false;

	*channel = (raw >> shift) & 0xf;

	uint16_t data_mask = (1 << shift) - 1;
	*value = raw & data_mask;

	//debug("DSM: %d 0x%04x -> %d %d", shift, raw, *channel, *value);

	return true;
}

static void
dsm_guess_format(bool reset)
{
	static uint32_t	cs10;
	static uint32_t	cs11;
	static unsigned samples;

	/* reset the 10/11 bit sniffed channel masks */
	if (reset) {
		cs10 = 0;
		cs11 = 0;
		samples = 0;
		channel_shift = 0;
		return;
	}

	/* scan the channels in the current frame in both 10- and 11-bit mode */
	for (unsigned i = 0; i < DSM_FRAME_CHANNELS; i++) {

		uint8_t *dp = &frame[2 + (2 * i)];
		uint16_t raw = (dp[0] << 8) | dp[1];
		unsigned channel, value;

		/* if the channel decodes, remember the assigned number */
		if (dsm_decode_channel(raw, 10, &channel, &value) && (channel < 31))
			cs10 |= (1 << channel);

		if (dsm_decode_channel(raw, 11, &channel, &value) && (channel < 31))
			cs11 |= (1 << channel);

		/* XXX if we cared, we could look for the phase bit here to decide 1 vs. 2-frame format */
	}

	/* wait until we have seen plenty of frames - 2 should normally be enough */
	if (samples++ < 5)
		return;

	/*
	 * Iterate the set of sensible sniffed channel sets and see whether
	 * decoding in 10 or 11-bit mode has yielded anything we recognise.
	 *
	 * XXX Note that due to what seem to be bugs in the DSM2 high-resolution
	 *     stream, we may want to sniff for longer in some cases when we think we
	 *     are talking to a DSM2 receiver in high-resolution mode (so that we can
	 *     reject it, ideally).
	 *     See e.g. http://git.openpilot.org/cru/OPReview-116 for a discussion
	 *     of this issue.
	 */
	static uint32_t masks[] = {
		0x3f,	/* 6 channels (DX6) */
		0x7f,	/* 7 channels (DX7) */
		0xff,	/* 8 channels (DX8) */
		0x1ff,	/* 9 channels (DX9, etc.) */
		0x3ff,	/* 10 channels (DX10) */
		0x3fff	/* 18 channels (DX10) */
	};
	unsigned votes10 = 0;
	unsigned votes11 = 0;

	for (unsigned i = 0; i < (sizeof(masks) / sizeof(masks[0])); i++) {

		if (cs10 == masks[i])
			votes10++;

		if (cs11 == masks[i])
			votes11++;
	}

	if ((votes11 == 1) && (votes10 == 0)) {
		channel_shift = 11;
		debug("DSM: 11-bit format");
		return;
	}

	if ((votes10 == 1) && (votes11 == 0)) {
		channel_shift = 10;
		debug("DSM: 10-bit format");
		return;
	}

	/* call ourselves to reset our state ... we have to try again */
	debug("DSM: format detect fail, 10: 0x%08x %d 11: 0x%08x %d", cs10, votes10, cs11, votes11);
	dsm_guess_format(true);
}

static bool
dsm_decode(hrt_abstime frame_time, uint16_t *values, uint16_t *num_values)
{

	/*
		debug("DSM frame %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x",
		      frame[0], frame[1], frame[2], frame[3], frame[4], frame[5], frame[6], frame[7],
		      frame[8], frame[9], frame[10], frame[11], frame[12], frame[13], frame[14], frame[15]);
	*/
	/*
	 * If we have lost signal for at least a second, reset the
	 * format guessing heuristic.
	 */
	if (((frame_time - last_frame_time) > 1000000) && (channel_shift != 0))
		dsm_guess_format(true);

	/* we have received something we think is a frame */
	last_frame_time = frame_time;

	/* if we don't know the frame format, update the guessing state machine */
	if (channel_shift == 0) {
		dsm_guess_format(false);
		return false;
	}

	/*
	 * The encoding of the first two bytes is uncertain, so we're
	 * going to ignore them for now.
	 *
	 * Each channel is a 16-bit unsigned value containing either a 10-
	 * or 11-bit channel value and a 4-bit channel number, shifted
	 * either 10 or 11 bits. The MSB may also be set to indicate the
	 * second frame in variants of the protocol where more than
	 * seven channels are being transmitted.
	 */

	for (unsigned i = 0; i < DSM_FRAME_CHANNELS; i++) {

		uint8_t *dp = &frame[2 + (2 * i)];
		uint16_t raw = (dp[0] << 8) | dp[1];
		unsigned channel, value;

		if (!dsm_decode_channel(raw, channel_shift, &channel, &value))
			continue;

		/* ignore channels out of range */
		if (channel >= PX4IO_INPUT_CHANNELS)
			continue;

		/* update the decoded channel count */
		if (channel >= *num_values)
			*num_values = channel + 1;

		/* convert 0-1024 / 0-2048 values to 1000-2000 ppm encoding in a very sloppy fashion */
		if (channel_shift == 11)
			value /= 2;

		value += 998;

		/*
		 * Store the decoded channel into the R/C input buffer, taking into
		 * account the different ideas about channel assignement that we have.
		 *
		 * Specifically, the first four channels in rc_channel_data are roll, pitch, thrust, yaw,
		 * but the first four channels from the DSM receiver are thrust, roll, pitch, yaw.
		 */
		switch (channel) {
		case 0:
			channel = 2;
			break;

		case 1:
			channel = 0;
			break;

		case 2:
			channel = 1;

		default:
			break;
		}

		values[channel] = value;
	}

	/*
	 * XXX Note that we may be in failsafe here; we need to work out how to detect that.
	 */
	return true;
}
