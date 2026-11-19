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
 * @file Timesync.cpp
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
					PX4_DEBUG("time jump detected. Resetting time synchroniser.");
					// Reset the filter
					reset_filter();
				}

			} else {

				// Perform filter update
				add_sample(offset_us, now_us);

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

			if (_high_rtt_count == MAX_CONSECUTIVE_HIGH_RTT) {
				PX4_DEBUG("RTT too high for timesync: %llu ms", rtt_us / 1000ULL);
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

void Timesync::add_sample(int64_t offset_us, uint64_t now_us)
{
	// Kalman filter with state [offset, skew]. The skew is a rate [us/s], so the prediction
	// stays consistent when the interval between samples changes (e.g. fast initial sync, then 1 Hz).
	static constexpr double R = MEASUREMENT_NOISE_STD * MEASUREMENT_NOISE_STD;

	if (_sequence == 0) {
		// First offset sample
		_time_offset = offset_us;
		_time_skew = 0.0;
		_var_offset = R;
		_cov_offset_skew = 0.0;
		_var_skew = INITIAL_SKEW_STD * INITIAL_SKEW_STD;

	} else {
		const double dt = (now_us - _last_sample_us) * 1e-6;

		// Predict
		_time_offset += _time_skew * dt;
		_var_offset += dt * (2.0 * _cov_offset_skew + dt * _var_skew) + SKEW_RANDOM_WALK * dt * dt * dt / 3.0;
		_cov_offset_skew += dt * _var_skew + SKEW_RANDOM_WALK * dt * dt / 2.0;
		_var_skew += SKEW_RANDOM_WALK * dt;

		// Update
		const double innovation = offset_us - _time_offset;
		const double k_offset = _var_offset / (_var_offset + R);
		const double k_skew = _cov_offset_skew / (_var_offset + R);

		_time_offset += k_offset * innovation;
		_time_skew += k_skew * innovation;

		_var_skew -= k_skew * _cov_offset_skew;
		_var_offset *= (1.0 - k_offset);
		_cov_offset_skew *= (1.0 - k_offset);
	}

	_last_sample_us = now_us;
}

void Timesync::reset_filter()
{
	// Do a full reset of all statistics and parameters
	_sequence = 0;
	_time_offset = 0.0;
	_time_skew = 0.0;
	_var_offset = 0.0;
	_cov_offset_skew = 0.0;
	_var_skew = 0.0;
	_last_sample_us = 0;
	_high_deviation_count = 0;
	_high_rtt_count = 0;
}
