/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file safelink.c
 *
 * Serial protocol encoder / decoder for the <put a good name here>
 * simple redundancy protocol (SRP)
 */

#include <nuttx/config.h>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdbool.h>

#include <drivers/drv_hrt.h>
#include "px4io.h"

#define SAFELINK_FRAME_SIZE	(2 + 2 * PX4IO_SERVO_COUNT + 2)
#define SAFELINK_STX1		0xFA
#define SAFELINK_STX2		0x01					/**< version */

static int safelink_fd = -1;

static hrt_abstime last_rx_time;
static hrt_abstime last_frame_time;

static uint8_t	frame[SAFELINK_FRAME_SIZE];

static unsigned partial_frame_count;

unsigned safelink_frame_drops;

static bool safelink_decode(hrt_abstime frame_time, uint16_t *values, uint16_t *num_values, bool *failsafe, uint16_t max_channels);

int
safelink_init(int fd)
{
	safelink_fd = fd;
	return OK;

	r_safelink_count = 0;
}

void
safelink_output(uint16_t *values, uint16_t num_values)
{
	uint8_t buf[SAFELINK_FRAME_SIZE];
	buf[0] = SAFELINK_STX1;
	buf[1] = SAFELINK_STX2;

	for (unsigned i = 0; (i < num_values) && (i < PX4IO_SERVO_COUNT); i++) {
		uint8_t *dp = &buf[2 + (2 * i)];
		dp[0] = values[i] >> 8;
		dp[1] = values[i];
	}

	buf[SAFELINK_FRAME_SIZE - 2] = 0x11;
	buf[SAFELINK_FRAME_SIZE - 1] = 0xFF;
}

bool
safelink_input(uint16_t *values, uint16_t *num_values, bool *failsafe, uint16_t max_channels)
{
	ssize_t		ret;
	hrt_abstime	now;
	bool		decode_ret = false;

	/*
	 * The S.BUS protocol doesn't provide reliable framing,
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
			safelink_frame_drops++;
			partial_frame_count = 0;
		}
	}

	/*
	 * Fetch bytes, but no more than we would need to complete
	 * the current frame.
	 */
	ret = read(safelink_fd, &frame[partial_frame_count], SAFELINK_FRAME_SIZE - partial_frame_count);

	/* if the read failed for any reason, just give up here */
	if (ret < 1)
		return false;

	if (partial_frame_count < SAFELINK_FRAME_SIZE)
		decode_ret = false;

	safelink_decode(now, values, num_values, failsafe, max_channels);

	/* return false as default */
	return decode_ret;
}

static bool safelink_decode(hrt_abstime frame_time, uint16_t *values, uint16_t *num_values, bool *failsafe, uint16_t max_channels)
{
	/* check frame boundary markers to avoid out-of-sync cases */
	if ((frame[0] != SAFELINK_STX1) || frame[1] != SAFELINK_STX2) {
		safelink_frame_drops++;
		return false;
	}

	/* XXX check checksum here */


	/* if we make it down here, we have valid data */
	last_frame_time = frame_time;

	for (unsigned i = 0; (i < max_channels) && (i < PX4IO_SERVO_COUNT); i++) {
		uint8_t *dp = &frame[2 + (2 * i)];
		values[i] = (dp[0] << 8) | dp[1];
	}
}

