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
#include <string.h>

#include <systemlib/ppm_decode.h>
 
#include <drivers/drv_hrt.h>

#include "px4io.h"
#include "protocol.h"

#define DSM_FRAME_SIZE		16
#define DSM_FRAME_CHANNELS	7

static hrt_abstime last_frame_time;

static uint8_t	frame[DSM_FRAME_SIZE];

static unsigned partial_frame_count;
static bool	insync;
static unsigned	channel_shift;

static void dsm_decode(void);

void
dsm_init(unsigned mode)
{
	insync = false;
	partial_frame_count = 0;

	if (mode == RX_MODE_DSM_10BIT) {
		channel_shift = 10;
	} else {
		channel_shift = 11;
	}

	last_frame_time = hrt_absolute_time();
}

void
dsm_input(int fd)
{
	uint8_t		buf[DSM_FRAME_SIZE];
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
	 */
	now = hrt_absolute_time();
	if ((now - last_frame_time) > 5000)
		partial_frame_count = 0;

	/*
	 * Fetch bytes, but no more than we would need to complete
	 * the current frame.
	 */
	ret = read(fd, buf, DSM_FRAME_SIZE - partial_frame_count);

	/* if the read failed for any reason, just give up here */
	if (ret < 1)
		return;

	/*
	 * Add bytes to the current frame
	 */
	memcpy(&frame[partial_frame_count], buf, ret);
	partial_frame_count += ret;

	/*
	 * If we don't have a full frame, return
	 */
	if (partial_frame_count < DSM_FRAME_SIZE)
	 	return;
	last_frame_time = now;

	/*
	 * Great, it looks like we might have a frame.  Go ahead and
	 * decode it.
	 */
	dsm_decode();
	partial_frame_count = 0;
}

static void
dsm_decode(void)
{
	uint16_t	data_mask = (1 << channel_shift) - 1;

	/*
	 * The encoding of the first byte is uncertain, so we're going
	 * to ignore it for now.
	 *
	 * The second byte may tell us about the protocol, but it's not
	 * actually very interesting since what we really want to know
	 * is how the channel data is formatted, and there doesn't seem
	 * to be a reliable way to determine this from the protocol ID
	 * alone.
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

		/* ignore pad channels */
		if (raw == 0xffff)
			continue;

		unsigned channel = (raw >> channel_shift) & 0xf;

		/* ignore channels out of range */
		if (channel >= PX4IO_INPUT_CHANNELS)
			continue;

		if (channel > ppm_decoded_channels)
			ppm_decoded_channels = channel;

		/* convert 0-1024 / 0-2048 values to 1000-2000 ppm encoding in a very sloppy fashion */
		unsigned data = raw & data_mask;
		if (channel_shift == 11)
			data /= 2;
		ppm_buffer[channel] = 988 + data;

	}
	ppm_last_valid_decode = hrt_absolute_time();
}
