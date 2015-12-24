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
 * @file ppm_decode.c
 *
 * PPM input decoder.
 */

#include <px4_config.h>

#include <stdint.h>
#include <string.h>

#include <drivers/drv_hrt.h>

#include "ppm_decode.h"

/*
 * PPM decoder tuning parameters.
 *
 * The PPM decoder works as follows.
 *
 * Initially, the decoder waits in the UNSYNCH state for two edges
 * separated by PPM_MIN_START.  Once the second edge is detected,
 * the decoder moves to the ARM state.
 *
 * The ARM state expects an edge within PPM_MAX_PULSE_WIDTH, being the
 * timing mark for the first channel.  If this is detected, it moves to
 * the INACTIVE state.
 *
 * The INACTIVE phase waits for and discards the next edge, as it is not
 * significant.  Once the edge is detected, it moves to the ACTIVE stae.
 *
 * The ACTIVE state expects an edge within PPM_MAX_PULSE_WIDTH, and when
 * received calculates the time from the previous mark and records
 * this time as the value for the next channel.
 *
 * If at any time waiting for an edge, the delay from the previous edge
 * exceeds PPM_MIN_START the frame is deemed to have ended and the recorded
 * values are advertised to clients.
 */
#define PPM_MAX_PULSE_WIDTH	500		/* maximum width of a pulse */
#define PPM_MIN_CHANNEL_VALUE	800		/* shortest valid channel signal */
#define PPM_MAX_CHANNEL_VALUE	2200		/* longest valid channel signal */
#define PPM_MIN_START		2500		/* shortest valid start gap */

/* Input timeout - after this interval we assume signal is lost */
#define PPM_INPUT_TIMEOUT	100 * 1000	/* 100ms */

/* Number of same-sized frames required to 'lock' */
#define PPM_CHANNEL_LOCK	3		/* should be less than the input timeout */

/* decoded PPM buffer */
#define PPM_MIN_CHANNELS	4
#define PPM_MAX_CHANNELS	12

/*
 * Public decoder state
 */
uint16_t	ppm_buffer[PPM_MAX_CHANNELS];
unsigned	ppm_decoded_channels;
hrt_abstime	ppm_last_valid_decode;

static uint16_t ppm_temp_buffer[PPM_MAX_CHANNELS];

/* PPM decoder state machine */
static struct {
	uint16_t	last_edge;	/* last capture time */
	uint16_t	last_mark;	/* last significant edge */
	unsigned	next_channel;
	unsigned	count_max;
	enum {
		UNSYNCH = 0,
		ARM,
		ACTIVE,
		INACTIVE
	} phase;
} ppm;


void
ppm_input_init(unsigned count_max)
{
	ppm_decoded_channels = 0;
	ppm_last_valid_decode = 0;

	memset(&ppm, 0, sizeof(ppm));
	ppm.count_max = count_max;
}

void
ppm_input_decode(bool reset, unsigned count)
{
	uint16_t width;
	uint16_t interval;
	unsigned i;

	/* if we missed an edge, we have to give up */
	if (reset) {
		goto error;
	}

	/* how long since the last edge? */
	width = count - ppm.last_edge;

	if (count < ppm.last_edge) {
		width += ppm.count_max;        /* handle wrapped count */
	}

	ppm.last_edge = count;

	/*
	 * If this looks like a start pulse, then push the last set of values
	 * and reset the state machine.
	 *
	 * Note that this is not a "high performance" design; it implies a whole
	 * frame of latency between the pulses being received and their being
	 * considered valid.
	 */
	if (width >= PPM_MIN_START) {

		/*
		 * If the number of channels changes unexpectedly, we don't want
		 * to just immediately jump on the new count as it may be a result
		 * of noise or dropped edges.  Instead, take a few frames to settle.
		 */
		if (ppm.next_channel != ppm_decoded_channels) {
			static unsigned new_channel_count;
			static unsigned new_channel_holdoff;

			if (new_channel_count != ppm.next_channel) {
				/* start the lock counter for the new channel count */
				new_channel_count = ppm.next_channel;
				new_channel_holdoff = PPM_CHANNEL_LOCK;

			} else if (new_channel_holdoff > 0) {
				/* this frame matched the last one, decrement the lock counter */
				new_channel_holdoff--;

			} else {
				/* we have seen PPM_CHANNEL_LOCK frames with the new count, accept it */
				ppm_decoded_channels = new_channel_count;
				new_channel_count = 0;
			}

		} else {
			/* frame channel count matches expected, let's use it */
			if (ppm.next_channel > PPM_MIN_CHANNELS) {
				for (i = 0; i < ppm.next_channel; i++) {
					ppm_buffer[i] = ppm_temp_buffer[i];
				}

				ppm_last_valid_decode = hrt_absolute_time();
			}
		}

		/* reset for the next frame */
		ppm.next_channel = 0;

		/* next edge is the reference for the first channel */
		ppm.phase = ARM;

		return;
	}

	switch (ppm.phase) {
	case UNSYNCH:
		/* we are waiting for a start pulse - nothing useful to do here */
		return;

	case ARM:

		/* we expect a pulse giving us the first mark */
		if (width > PPM_MAX_PULSE_WIDTH) {
			goto error;        /* pulse was too long */
		}

		/* record the mark timing, expect an inactive edge */
		ppm.last_mark = count;
		ppm.phase = INACTIVE;
		return;

	case INACTIVE:
		/* this edge is not interesting, but now we are ready for the next mark */
		ppm.phase = ACTIVE;

		/* note that we don't bother looking at the timing of this edge */

		return;

	case ACTIVE:

		/* we expect a well-formed pulse */
		if (width > PPM_MAX_PULSE_WIDTH) {
			goto error;        /* pulse was too long */
		}

		/* determine the interval from the last mark */
		interval = count - ppm.last_mark;
		ppm.last_mark = count;

		/* if the mark-mark timing is out of bounds, abandon the frame */
		if ((interval < PPM_MIN_CHANNEL_VALUE) || (interval > PPM_MAX_CHANNEL_VALUE)) {
			goto error;
		}

		/* if we have room to store the value, do so */
		if (ppm.next_channel < PPM_MAX_CHANNELS) {
			ppm_temp_buffer[ppm.next_channel++] = interval;
		}

		ppm.phase = INACTIVE;
		return;

	}

	/* the state machine is corrupted; reset it */

error:
	/* we don't like the state of the decoder, reset it and try again */
	ppm.phase = UNSYNCH;
	ppm_decoded_channels = 0;
}

