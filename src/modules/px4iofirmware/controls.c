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
 * @file controls.c
 *
 * R/C inputs and servo outputs.
 */

#include <nuttx/config.h>
#include <stdbool.h>

#include <drivers/drv_hrt.h>
#include <systemlib/perf_counter.h>
#include <systemlib/ppm_decode.h>

#include "px4io.h"

#define RC_FAILSAFE_TIMEOUT		2000000		/**< two seconds failsafe timeout */
#define RC_CHANNEL_HIGH_THRESH		5000
#define RC_CHANNEL_LOW_THRESH		-5000

static bool	ppm_input(uint16_t *values, uint16_t *num_values);

static perf_counter_t c_gather_dsm;
static perf_counter_t c_gather_sbus;
static perf_counter_t c_gather_ppm;

void
controls_init(void)
{
	/* DSM input */
	dsm_init("/dev/ttyS0");

	/* S.bus input */
	sbus_init("/dev/ttyS2");

	/* default to a 1:1 input map, all enabled */
	for (unsigned i = 0; i < MAX_CONTROL_CHANNELS; i++) {
		unsigned base = PX4IO_P_RC_CONFIG_STRIDE * i;

		r_page_rc_input_config[base + PX4IO_P_RC_CONFIG_OPTIONS]    = 0;
		r_page_rc_input_config[base + PX4IO_P_RC_CONFIG_MIN]        = 1000;
		r_page_rc_input_config[base + PX4IO_P_RC_CONFIG_CENTER]     = 1500;
		r_page_rc_input_config[base + PX4IO_P_RC_CONFIG_MAX]        = 2000;
		r_page_rc_input_config[base + PX4IO_P_RC_CONFIG_DEADZONE]   = 30;
		r_page_rc_input_config[base + PX4IO_P_RC_CONFIG_ASSIGNMENT] = i;
		r_page_rc_input_config[base + PX4IO_P_RC_CONFIG_OPTIONS]    = PX4IO_P_RC_CONFIG_OPTIONS_ENABLED;
	}

	c_gather_dsm = perf_alloc(PC_ELAPSED, "c_gather_dsm");
	c_gather_sbus = perf_alloc(PC_ELAPSED, "c_gather_sbus");
	c_gather_ppm = perf_alloc(PC_ELAPSED, "c_gather_ppm");
}

void
controls_tick() {

	/*
	 * Gather R/C control inputs from supported sources.
	 *
	 * Note that if you're silly enough to connect more than
	 * one control input source, they're going to fight each
	 * other.  Don't do that.
	 */

	perf_begin(c_gather_dsm);
	bool dsm_updated = dsm_input(r_raw_rc_values, &r_raw_rc_count);
	if (dsm_updated)
		r_status_flags |= PX4IO_P_STATUS_FLAGS_RC_DSM;
	perf_end(c_gather_dsm);

	perf_begin(c_gather_sbus);
	bool sbus_updated = sbus_input(r_raw_rc_values, &r_raw_rc_count);
	if (sbus_updated)
		r_status_flags |= PX4IO_P_STATUS_FLAGS_RC_SBUS;
	perf_end(c_gather_sbus);

	/*
	 * XXX each S.bus frame will cause a PPM decoder interrupt
	 * storm (lots of edges).  It might be sensible to actually
	 * disable the PPM decoder completely if we have S.bus signal.
	 */
	perf_begin(c_gather_ppm);
	bool ppm_updated = ppm_input(r_raw_rc_values, &r_raw_rc_count);
	if (ppm_updated)
		r_status_flags |= PX4IO_P_STATUS_FLAGS_RC_PPM;
	perf_end(c_gather_ppm);

	ASSERT(r_raw_rc_count <= MAX_CONTROL_CHANNELS);

	/*
	 * In some cases we may have received a frame, but input has still
	 * been lost.
	 */
	bool rc_input_lost = false;

	/*
	 * If we received a new frame from any of the RC sources, process it.
	 */
	if (dsm_updated || sbus_updated || ppm_updated) {

		/* update RC-received timestamp */
		system_state.rc_channels_timestamp = hrt_absolute_time();

		/* record a bitmask of channels assigned */
		unsigned assigned_channels = 0;

		/* map raw inputs to mapped inputs */
		/* XXX mapping should be atomic relative to protocol */
		for (unsigned i = 0; i < r_raw_rc_count; i++) {

			/* map the input channel */
			uint16_t *conf = &r_page_rc_input_config[i * PX4IO_P_RC_CONFIG_STRIDE];

			if (conf[PX4IO_P_RC_CONFIG_OPTIONS] & PX4IO_P_RC_CONFIG_OPTIONS_ENABLED) {

				uint16_t raw = r_raw_rc_values[i];

				int16_t scaled;

				/*
				 * 1) Constrain to min/max values, as later processing depends on bounds.
				 */
				if (raw < conf[PX4IO_P_RC_CONFIG_MIN])
					raw = conf[PX4IO_P_RC_CONFIG_MIN];
				if (raw > conf[PX4IO_P_RC_CONFIG_MAX])
					raw = conf[PX4IO_P_RC_CONFIG_MAX];

				/*
				 * 2) Scale around the mid point differently for lower and upper range.
				 *
				 * This is necessary as they don't share the same endpoints and slope.
				 *
				 * First normalize to 0..1 range with correct sign (below or above center),
				 * then scale to 20000 range (if center is an actual center, -10000..10000,
				 * if parameters only support half range, scale to 10000 range, e.g. if
				 * center == min 0..10000, if center == max -10000..0).
				 *
				 * As the min and max bounds were enforced in step 1), division by zero
				 * cannot occur, as for the case of center == min or center == max the if
				 * statement is mutually exclusive with the arithmetic NaN case.
				 *
				 * DO NOT REMOVE OR ALTER STEP 1!
				 */
				if (raw > (conf[PX4IO_P_RC_CONFIG_CENTER] + conf[PX4IO_P_RC_CONFIG_DEADZONE])) {
					scaled = 10000.0f * ((raw - conf[PX4IO_P_RC_CONFIG_CENTER] - conf[PX4IO_P_RC_CONFIG_DEADZONE]) / (float)(conf[PX4IO_P_RC_CONFIG_MAX] - conf[PX4IO_P_RC_CONFIG_CENTER] - conf[PX4IO_P_RC_CONFIG_DEADZONE]));

				} else if (raw < (conf[PX4IO_P_RC_CONFIG_CENTER] - conf[PX4IO_P_RC_CONFIG_DEADZONE])) {
					scaled = 10000.0f * ((raw - conf[PX4IO_P_RC_CONFIG_CENTER] + conf[PX4IO_P_RC_CONFIG_DEADZONE]) / (float)(conf[PX4IO_P_RC_CONFIG_CENTER] - conf[PX4IO_P_RC_CONFIG_DEADZONE] - conf[PX4IO_P_RC_CONFIG_MIN]));

				} else {
					/* in the configured dead zone, output zero */
					scaled = 0;
				}

				/* invert channel if requested */
				if (conf[PX4IO_P_RC_CONFIG_OPTIONS] & PX4IO_P_RC_CONFIG_OPTIONS_REVERSE)
					scaled = -scaled;

				/* and update the scaled/mapped version */
				unsigned mapped = conf[PX4IO_P_RC_CONFIG_ASSIGNMENT];
				ASSERT(mapped < MAX_CONTROL_CHANNELS);

				/* invert channel if pitch - pulling the lever down means pitching up by convention */
				if (mapped == 1) /* roll, pitch, yaw, throttle, override is the standard order */
					scaled = -scaled;

				r_rc_values[mapped] = SIGNED_TO_REG(scaled);
				assigned_channels |= (1 << mapped);
			}
		}

		/* set un-assigned controls to zero */
		for (unsigned i = 0; i < MAX_CONTROL_CHANNELS; i++) {
			if (!(assigned_channels & (1 << i)))
				r_rc_values[i] = 0;
		}

		/*
		 * If we got an update with zero channels, treat it as 
		 * a loss of input.
		 *
		 * This might happen if a protocol-based receiver returns an update
		 * that contains no channels that we have mapped.
		 */
		if (assigned_channels == 0) {
			rc_input_lost = true;
		} else {
			/* set RC OK flag and clear RC lost alarm */
			r_status_flags |= PX4IO_P_STATUS_FLAGS_RC_OK;
			r_status_alarms &= ~PX4IO_P_STATUS_ALARMS_RC_LOST;
		}

		/*
		 * Export the valid channel bitmap
		 */
		r_rc_valid = assigned_channels;
	}

	/*
	 * If we haven't seen any new control data in 200ms, assume we
	 * have lost input.
	 */
	if (hrt_elapsed_time(&system_state.rc_channels_timestamp) > 200000) {
		rc_input_lost = true;

		/* clear the input-kind flags here */
		r_status_flags &= ~(
			PX4IO_P_STATUS_FLAGS_RC_PPM |
			PX4IO_P_STATUS_FLAGS_RC_DSM |
			PX4IO_P_STATUS_FLAGS_RC_SBUS);
	}

	/*
	 * Handle losing RC input
	 */
	if (rc_input_lost) {

		/* Clear the RC input status flag, clear manual override flag */
		r_status_flags &= ~(
			PX4IO_P_STATUS_FLAGS_OVERRIDE |
			PX4IO_P_STATUS_FLAGS_RC_OK);

		/* Set the RC_LOST alarm */
		r_status_alarms |= PX4IO_P_STATUS_ALARMS_RC_LOST;

		/* Mark the arrays as empty */
		r_raw_rc_count = 0;
		r_rc_valid = 0;
	}

	/*
	 * Check for manual override.
	 *
	 * The PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE_OK flag must be set, and we
	 * must have R/C input.
	 * Override is enabled if either the hardcoded channel / value combination
	 * is selected, or the AP has requested it.
	 */
	if ((r_setup_arming & PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE_OK) && 
		(r_status_flags & PX4IO_P_STATUS_FLAGS_RC_OK)) {

		bool override = false;

		/*
		 * Check mapped channel 5 (can be any remote channel,
		 * depends on RC_MAP_OVER parameter);
		 * If the value is 'high' then the pilot has
		 * requested override.
		 *
		 */
		if ((r_status_flags & PX4IO_P_STATUS_FLAGS_RC_OK) && (REG_TO_SIGNED(r_rc_values[4]) > RC_CHANNEL_HIGH_THRESH))
			override = true;

		if (override) {

			r_status_flags |= PX4IO_P_STATUS_FLAGS_OVERRIDE;

			/* mix new RC input control values to servos */
			if (dsm_updated || sbus_updated || ppm_updated)
				mixer_tick();

		} else {
			r_status_flags &= ~PX4IO_P_STATUS_FLAGS_OVERRIDE;
		}
	}
}

static bool
ppm_input(uint16_t *values, uint16_t *num_values)
{
	bool result = false;

	/* avoid racing with PPM updates */
	irqstate_t state = irqsave();

	/*
	 * If we have received a new PPM frame within the last 200ms, accept it
	 * and then invalidate it.
	 */
	if (hrt_elapsed_time(&ppm_last_valid_decode) < 200000) {

		/* PPM data exists, copy it */
		*num_values = ppm_decoded_channels;
		if (*num_values > MAX_CONTROL_CHANNELS)
			*num_values = MAX_CONTROL_CHANNELS;

		for (unsigned i = 0; i < *num_values; i++)
			values[i] = ppm_buffer[i];

		/* clear validity */
		ppm_last_valid_decode = 0;

		/* good if we got any channels */
		result = (*num_values > 0);
	}

	irqrestore(state);

	return result;
}
