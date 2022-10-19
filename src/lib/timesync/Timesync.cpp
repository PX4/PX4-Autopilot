/****************************************************************************
 *
 *   Copyright (c) 2018-2022 PX4 Development Team. All rights reserved.
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
 * @file timesync.cpp
 * timesync implementation.
 *
 * @author Mohammed Kabir <mhkabir98@gmail.com>
 */

#include "Timesync.hpp"

#include <stdlib.h>

void Timesync::update(const uint64_t now_us, const int64_t remote_timestamp_ns, const int64_t originate_timestamp_ns)
{
	// Message originating from this system, compute time offset from it
	if (remote_timestamp_ns > 0) {
		// Calculate time offset between this system and the remote system, assuming RTT for
		// the timesync packet is roughly equal both ways.
		int64_t offset_us = (int64_t)((originate_timestamp_ns / 1000ULL) + now_us - (remote_timestamp_ns / 1000ULL) * 2) / 2 ;

		// Calculate the round trip time (RTT) it took the timesync packet to bounce back to us from remote system
		uint64_t rtt_us = now_us - (originate_timestamp_ns / 1000ULL);

		// Calculate the difference of this sample from the current estimate
		uint64_t deviation = llabs((int64_t)_time_offset - offset_us);

		if (rtt_us < MAX_RTT_SAMPLE) {	// Only use samples with low RTT

			if (sync_converged() && (deviation > MAX_DEVIATION_SAMPLE)) {

				// Increment the counter if we have a good estimate and are getting samples far from the estimate
				_high_deviation_count++;

				// We reset the filter if we received 5 consecutive samples which violate our present estimate.
				// This is most likely due to a time jump on the offboard system.
				if (_high_deviation_count > MAX_CONSECUTIVE_HIGH_DEVIATION) {
					PX4_ERR("Time jump detected. Resetting time synchroniser.");
					// Reset the filter
					reset_filter();
				}

			} else {

				// Filter gain scheduling
				if (!sync_converged()) {
					// Interpolate with a sigmoid function
					double progress = (double)_sequence / (double)CONVERGENCE_WINDOW;
					double p = 1.0 - exp(0.5 * (1.0 - 1.0 / (1.0 - progress)));
					_filter_alpha = p * ALPHA_GAIN_FINAL + (1.0 - p) * ALPHA_GAIN_INITIAL;
					_filter_beta = p * BETA_GAIN_FINAL + (1.0 - p) * BETA_GAIN_INITIAL;

				} else {
					_filter_alpha = ALPHA_GAIN_FINAL;
					_filter_beta = BETA_GAIN_FINAL;
				}

				// Perform filter update
				add_sample(offset_us);

				// Increment sequence counter after filter update
				_sequence++;

				// Reset high deviation count after filter update
				_high_deviation_count = 0;

				// Reset high RTT count after filter update
				_high_rtt_count = 0;
			}

		} else {
			// Increment counter if round trip time is too high for accurate timesync
			_high_rtt_count++;

			if (_high_rtt_count > MAX_CONSECUTIVE_HIGH_RTT) {
				PX4_WARN("RTT too high for timesync: %llu ms", rtt_us / 1000ULL);
				// Reset counter to rate-limit warnings
				_high_rtt_count = 0;
			}

		}

		// Publish status message
		timesync_status_s tsync_status{};
		tsync_status.source_protocol = _source;
		tsync_status.remote_timestamp = remote_timestamp_ns / 1000ULL;
		tsync_status.observed_offset = offset_us;
		tsync_status.estimated_offset = (int64_t)_time_offset;
		tsync_status.round_trip_time = rtt_us;
		tsync_status.timestamp = hrt_absolute_time();

		_timesync_status_pub.publish(tsync_status);
	}
}

uint64_t Timesync::sync_stamp(uint64_t usec)
{
	// Only return synchronised stamp if we have converged to a good value
	if (sync_converged()) {
		return usec + (int64_t)_time_offset;

	} else {
		return hrt_absolute_time();
	}
}

void Timesync::add_sample(int64_t offset_us)
{
	// Online exponential smoothing filter. The derivative of the estimate is also
	// estimated in order to produce an estimate without steady state lag:
	// https://en.wikipedia.org/wiki/Exponential_smoothing#Double_exponential_smoothing
	double time_offset_prev = _time_offset;

	if (_sequence == 0) {
		// First offset sample
		_time_offset = offset_us;

	} else {
		// Update the clock offset estimate
		_time_offset = _filter_alpha * offset_us + (1.0 - _filter_alpha) * (_time_offset + _time_skew);

		// Update the clock skew estimate
		_time_skew = _filter_beta * (_time_offset - time_offset_prev) + (1.0 - _filter_beta) * _time_skew;
	}
}

void Timesync::reset_filter()
{
	// Do a full reset of all statistics and parameters
	_sequence = 0;
	_time_offset = 0.0;
	_time_skew = 0.0;
	_filter_alpha = ALPHA_GAIN_INITIAL;
	_filter_beta = BETA_GAIN_INITIAL;
	_high_deviation_count = 0;
	_high_rtt_count = 0;
}
