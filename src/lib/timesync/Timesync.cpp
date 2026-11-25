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
		uint64_t deviation = llabs(offset() - offset_us);

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
		tsync_status.estimated_offset = offset();
		tsync_status.round_trip_time = rtt_us;
		tsync_status.timestamp = hrt_absolute_time();

		_timesync_status_pub.publish(tsync_status);
	}
}

uint64_t Timesync::sync_stamp(uint64_t usec)
{
	// Only return synchronised stamp if we have converged to a good value
	if (sync_converged()) {
		return usec + offset();

	} else {
		return hrt_absolute_time();
	}
}

/*
 * Hybrid Kalman filter:
 * https://en.wikipedia.org/wiki/Kalman_filter#Hybrid_Kalman_filter
 *
 * State:
 *   x = [theta; gamma]
 *   theta = t_local - t_remote : clock offset [s]
 *   gamma = d(theta)/dt        : clock skew [s/s], the relative rate error of the two clocks
 *
 * Model:
 *   dx/dt = F x + w(t),  F = [0 1; 0 0],  w(t) ~ N(0, Q),  Q = diag(q_o, q_s)
 *   z_k   = H x_k + v_k, H = [1 0],       v_k  ~ N(0, R)
 * where Q is the continuous-time noise spectral density, q_o = OFFSET_RANDOM_WALK,
 * q_s = SKEW_RANDOM_WALK, R = MEASUREMENT_NOISE_STD^2, and z_k is the offset observed
 * from one timesync round trip.
 *
 * Prediction: the hybrid filter integrates, over the time dt since the last sample,
 *   dx/dt = F x
 *   dP/dt = F P + P F^T + Q
 * For this F and Q both have an exact closed-form solution:
 *   x <- A x,            A   = [1 dt; 0 1]   (offset += skew * dt, skew unchanged)
 *   P <- A P A^T + Q_d,  Q_d = [q_o dt + q_s dt^3/3   q_s dt^2/2;
 *                               q_s dt^2/2            q_s dt    ]
 *
 * Q_d is the noise added over dt. Each random walk adds q dt to its own variance.
 * The skew noise also leaks into the offset through offset += skew * dt, which
 * gives the dt^2/2 and dt^3/3 terms. A skew kick w at s seconds before the end of
 * dt changes the skew by w and the offset by s w. Summing over the interval:
 *   cov(offset, skew) += integral_0^dt q_s s   ds = q_s dt^2/2
 *   var(offset)       += integral_0^dt q_s s^2 ds = q_s dt^3/3
 *
 * https://en.wikipedia.org/wiki/Discretization#Discretization_of_process_noise
 */
void Timesync::add_sample(int64_t offset_us, uint64_t now_us)
{
	static constexpr double R = MEASUREMENT_NOISE_STD * MEASUREMENT_NOISE_STD;
	const double offset_s = offset_us * 1e-6;

	if (_sequence == 0) {
		// First offset sample
		_time_offset = offset_s;
		_time_skew = 0.0;
		_var_offset = R;
		_cov_offset_skew = 0.0;
		_var_skew = INITIAL_SKEW_STD * INITIAL_SKEW_STD;

	} else {
		const double dt = (now_us - _last_sample_us) * 1e-6;

		// Predict: x <- A x, P <- A P A^T + Q_d, written out in Horner form.
		_time_offset += _time_skew * dt;
		_var_offset += dt * (OFFSET_RANDOM_WALK + 2.0 * _cov_offset_skew + dt * (_var_skew + dt * SKEW_RANDOM_WALK / 3.0));
		_cov_offset_skew += dt * (_var_skew + dt * SKEW_RANDOM_WALK / 2.0);
		_var_skew += dt * SKEW_RANDOM_WALK;

		// Update: S = H P H^T + R, K = P H^T / S, x <- x + K (z - H x), P <- (I - K H) P
		const double innovation = offset_s - _time_offset;
		const double innovation_var = _var_offset + R;
		const double k_offset = _var_offset / innovation_var;
		const double k_skew = _cov_offset_skew / innovation_var;

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
	// The filter state is initialized by the first sample after the reset (_sequence == 0).
	// Clear the offset because it is reported before that sample arrives.
	_sequence = 0;
	_time_offset = 0.0;
	_high_deviation_count = 0;
	_high_rtt_count = 0;
}
