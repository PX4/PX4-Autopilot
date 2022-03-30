/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <px4_platform_common/px4_config.h>
#include <stdbool.h>

#include <drivers/drv_hrt.h>
#include <systemlib/ppm_decode.h>
#include <rc/st24.h>
#include <rc/sumd.h>
#include <rc/sbus.h>
#include <rc/dsm.h>
#include <uORB/topics/input_rc.h>

#if defined(PX4IO_PERF)
# include <perf/perf_counter.h>
#endif

#include "px4io.h"

static bool	ppm_input(uint16_t *values, uint16_t *num_values);
static bool	dsm_port_input(uint16_t *rssi, bool *dsm_updated, bool *st24_updated, bool *sumd_updated);

#if defined(PX4IO_PERF)
static perf_counter_t c_gather_dsm;
static perf_counter_t c_gather_sbus;
static perf_counter_t c_gather_ppm;
#endif

static int _dsm_fd = -1;
int _sbus_fd = -1;

#ifdef ADC_RSSI
static unsigned _rssi_adc_counts = 0;
#endif

bool dsm_port_input(uint16_t *rssi, bool *dsm_updated, bool *st24_updated, bool *sumd_updated)
{
	uint8_t n_bytes = 0;
	uint8_t *bytes;
	bool dsm_11_bit;
	int8_t spektrum_rssi;
	unsigned frame_drops = 0;
	*dsm_updated = dsm_input(_dsm_fd, r_raw_rc_values, &r_raw_rc_count, &dsm_11_bit, &n_bytes, &bytes,
				 &spektrum_rssi, &frame_drops, PX4IO_RC_INPUT_CHANNELS);

	if (*dsm_updated) {

		r_raw_rc_flags |= PX4IO_P_RAW_RC_FLAGS_RC_DSM;
		r_raw_rc_flags &= ~(PX4IO_P_RAW_RC_FLAGS_FAILSAFE);

		if (spektrum_rssi >= 0 && spektrum_rssi <= 100) {
			/* ensure ADC RSSI is disabled */
			r_setup_features &= ~(PX4IO_P_SETUP_FEATURES_ADC_RSSI);

			*rssi = spektrum_rssi;
		}

		r_page_raw_rc_input[PX4IO_P_RAW_LOST_FRAME_COUNT] = frame_drops;
	}


	/* get data from FD and attempt to parse with DSM and ST24 libs */
	*st24_updated = false;

	if (!(r_raw_rc_flags & (PX4IO_P_RAW_RC_FLAGS_RC_DSM | PX4IO_P_RAW_RC_FLAGS_RC_SUMD))) {
		uint8_t st24_rssi = 0;
		uint8_t lost_count = 0;
		uint16_t st24_channel_count = 0;

		for (unsigned i = 0; i < n_bytes; i++) {
			/* set updated flag if one complete packet was parsed */
			st24_rssi = INPUT_RC_RSSI_MAX; // input_rc_s::RSSI_MAX;
			*st24_updated |= (OK == st24_decode(bytes[i], &st24_rssi, &lost_count,
							    &st24_channel_count, r_raw_rc_values, PX4IO_RC_INPUT_CHANNELS));
		}

		if (*st24_updated && lost_count == 0) {
			/* ensure ADC RSSI is disabled */
			r_setup_features &= ~(PX4IO_P_SETUP_FEATURES_ADC_RSSI);

			*rssi = st24_rssi;
			r_raw_rc_count = st24_channel_count;

			r_raw_rc_flags |= PX4IO_P_RAW_RC_FLAGS_RC_ST24;
			r_raw_rc_flags &= ~(PX4IO_P_RAW_RC_FLAGS_FAILSAFE);
		}
	}


	/* get data from FD and attempt to parse with SUMD libs */
	*sumd_updated = false;

	if (!(r_raw_rc_flags & (PX4IO_P_RAW_RC_FLAGS_RC_DSM | PX4IO_P_RAW_RC_FLAGS_RC_ST24))) {
		uint8_t sumd_rssi = 0;
		uint8_t sumd_rx_count = 0;
		uint16_t sumd_channel_count = 0;
		bool sumd_failsafe_state = false;

		for (unsigned i = 0; i < n_bytes; i++) {
			/* set updated flag if one complete packet was parsed */
			sumd_rssi = INPUT_RC_RSSI_MAX; // input_rc_s::RSSI_MAX;
			*sumd_updated |= (OK == sumd_decode(bytes[i], &sumd_rssi, &sumd_rx_count,
							    &sumd_channel_count, r_raw_rc_values, PX4IO_RC_INPUT_CHANNELS, &sumd_failsafe_state));
		}

		if (*sumd_updated) {
			/* not setting RSSI since SUMD does not provide one */
			r_raw_rc_count = sumd_channel_count;

			r_raw_rc_flags |= PX4IO_P_RAW_RC_FLAGS_RC_SUMD;

			if (sumd_failsafe_state) {
				r_raw_rc_flags |= (PX4IO_P_RAW_RC_FLAGS_FAILSAFE);

			} else {
				r_raw_rc_flags &= ~(PX4IO_P_RAW_RC_FLAGS_FAILSAFE);
			}
		}
	}


	return (*dsm_updated | *st24_updated | *sumd_updated);
}

void
controls_init(void)
{
	/* no channels */
	r_raw_rc_count = 0;
	system_state.rc_channels_timestamp_received = 0;

	/* DSM input (USART1) */
	_dsm_fd = dsm_init("/dev/ttyS0");

	/* S.bus input (USART3) */
	_sbus_fd = sbus_init("/dev/ttyS2", false);

#if defined(PX4IO_PERF)
	c_gather_dsm = perf_alloc(PC_ELAPSED, "c_gather_dsm");
	c_gather_sbus = perf_alloc(PC_ELAPSED, "c_gather_sbus");
	c_gather_ppm = perf_alloc(PC_ELAPSED, "c_gather_ppm");
#endif
}

void
controls_tick()
{

	/*
	 * Gather R/C control inputs from supported sources.
	 *
	 * Note that if you're silly enough to connect more than
	 * one control input source, they're going to fight each
	 * other.  Don't do that.
	 */

	uint16_t rssi = 0;

#ifdef ADC_RSSI

	if (r_setup_features & PX4IO_P_SETUP_FEATURES_ADC_RSSI) {
		unsigned counts = adc_measure(ADC_RSSI);

		if (counts != 0xffff) {
			/* low pass*/
			_rssi_adc_counts = (_rssi_adc_counts * 0.998f) + (counts * 0.002f);
			/* use 1:1 scaling on 3.3V, 12-Bit ADC input */
			unsigned mV = _rssi_adc_counts * 3300 / 4095;
			/* scale to 0..100 (input_rc_s::RSSI_MAX == 100) */
			rssi = (mV * INPUT_RC_RSSI_MAX / 3300);

			if (rssi > INPUT_RC_RSSI_MAX) {
				rssi = INPUT_RC_RSSI_MAX;
			}
		}
	}

#endif

	bool sbus_updated = false;

	if (!(r_raw_rc_flags & (PX4IO_P_RAW_RC_FLAGS_RC_PPM | PX4IO_P_RAW_RC_FLAGS_RC_DSM | PX4IO_P_RAW_RC_FLAGS_RC_ST24 |
				PX4IO_P_RAW_RC_FLAGS_RC_SUMD))) {
#if defined(PX4IO_PERF)
		perf_begin(c_gather_sbus);
#endif
		bool sbus_failsafe = false;
		bool sbus_frame_drop = false;
		sbus_updated = sbus_input(_sbus_fd, r_raw_rc_values, &r_raw_rc_count, &sbus_failsafe, &sbus_frame_drop,
					  PX4IO_RC_INPUT_CHANNELS);

		if (sbus_updated) {
			r_raw_rc_flags |= PX4IO_P_RAW_RC_FLAGS_RC_SBUS;

			unsigned sbus_rssi = INPUT_RC_RSSI_MAX; // input_rc_s::RSSI_MAX

			if (sbus_frame_drop) {
				sbus_rssi = INPUT_RC_RSSI_MAX / 2;
			}

			r_page_raw_rc_input[PX4IO_P_RAW_LOST_FRAME_COUNT] = sbus_dropped_frames();

			if (sbus_failsafe) {
				r_raw_rc_flags |= PX4IO_P_RAW_RC_FLAGS_FAILSAFE;

			} else {
				r_raw_rc_flags &= ~(PX4IO_P_RAW_RC_FLAGS_FAILSAFE);
			}

			/* set RSSI to an emulated value if ADC RSSI is off */
			if (!(r_setup_features & PX4IO_P_SETUP_FEATURES_ADC_RSSI)) {
				rssi = sbus_rssi;
			}
		}

#if defined(PX4IO_PERF)
		perf_end(c_gather_sbus);
#endif
	}


	/*
	 * XXX each S.bus frame will cause a PPM decoder interrupt
	 * storm (lots of edges).  It might be sensible to actually
	 * disable the PPM decoder completely if we have S.bus signal.
	 */
	bool ppm_updated = false;

	if (!(r_raw_rc_flags & (PX4IO_P_RAW_RC_FLAGS_RC_SBUS | PX4IO_P_RAW_RC_FLAGS_RC_DSM | PX4IO_P_RAW_RC_FLAGS_RC_ST24 |
				PX4IO_P_RAW_RC_FLAGS_RC_SUMD))) {
#if defined(PX4IO_PERF)
		perf_begin(c_gather_ppm);
#endif
		ppm_updated = ppm_input(r_raw_rc_values, &r_raw_rc_count);

		if (ppm_updated) {
			r_raw_rc_flags |= PX4IO_P_RAW_RC_FLAGS_RC_PPM;
			r_raw_rc_flags &= ~PX4IO_P_RAW_RC_FLAGS_FAILSAFE;
		}

#if defined(PX4IO_PERF)
		perf_end(c_gather_ppm);
#endif
	}


	bool dsm_updated = false;
	bool st24_updated = false;
	bool sumd_updated = false;

	if (!(r_raw_rc_flags & (PX4IO_P_RAW_RC_FLAGS_RC_SBUS | PX4IO_P_RAW_RC_FLAGS_RC_PPM))) {
#if defined(PX4IO_PERF)
		perf_begin(c_gather_dsm);
#endif

		(void)dsm_port_input(&rssi, &dsm_updated, &st24_updated, &sumd_updated);

#if defined(PX4IO_PERF)
		perf_end(c_gather_dsm);
#endif
	}

	/* limit number of channels to allowable data size */
	if (r_raw_rc_count > PX4IO_RC_INPUT_CHANNELS) {
		r_raw_rc_count = PX4IO_RC_INPUT_CHANNELS;
	}

	/* store RSSI */
	r_page_raw_rc_input[PX4IO_P_RAW_RC_NRSSI] = rssi;

	/*
	 * If we received a new frame from any of the RC sources, process it.
	 */
	if (dsm_updated || sbus_updated || ppm_updated || st24_updated || sumd_updated) {

		/* update RC-received timestamp */
		system_state.rc_channels_timestamp_received = hrt_absolute_time();

		r_page_raw_rc_input[PX4IO_P_RAW_FRAME_COUNT]++;

	} else if (hrt_elapsed_time(&system_state.rc_channels_timestamp_received) > 200000) {
		/* clear the input-kind flags here, but only when disarmed */
		if (!(r_setup_arming & PX4IO_P_SETUP_ARMING_FMU_ARMED)) {

			r_raw_rc_flags &= ~PX4IO_P_RAW_RC_FLAGS_RC_PPM;
			r_raw_rc_flags &= ~PX4IO_P_RAW_RC_FLAGS_RC_DSM;
			r_raw_rc_flags &= ~PX4IO_P_RAW_RC_FLAGS_RC_SBUS;
			r_raw_rc_flags &= ~PX4IO_P_RAW_RC_FLAGS_RC_ST24;
			r_raw_rc_flags &= ~PX4IO_P_RAW_RC_FLAGS_RC_SUMD;
		}

		/* Set raw channel count to zero */
		r_raw_rc_count = 0;
	}
}

static bool
ppm_input(uint16_t *values, uint16_t *num_values)
{
	bool result = false;

	if (!(num_values) || !(values)) {
		return result;
	}

	/* avoid racing with PPM updates */
	irqstate_t state = px4_enter_critical_section();

	/*
	 * If we have received a new PPM frame within the last 200ms, accept it
	 * and then invalidate it.
	 */
	if (hrt_elapsed_time(&ppm_last_valid_decode) < 200000) {

		/* PPM data exists, copy it */
		*num_values = ppm_decoded_channels;

		if (*num_values > PX4IO_RC_INPUT_CHANNELS) {
			*num_values = PX4IO_RC_INPUT_CHANNELS;
		}

		for (unsigned i = 0; ((i < *num_values) && (i < PPM_MAX_CHANNELS)); i++) {
			values[i] = ppm_buffer[i];
		}

		/* clear validity */
		ppm_last_valid_decode = 0;

		/* store PPM frame length */
		//*frame_len = ppm_frame_length;

		/* good if we got any channels */
		result = (*num_values > 0);
	}

	px4_leave_critical_section(state);

	return result;
}
