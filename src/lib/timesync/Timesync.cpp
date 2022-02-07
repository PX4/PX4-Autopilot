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

#include "Timesync.hpp"

#include <stdlib.h>

void Timesync::update(int64_t offset_us)
{
	const bool converged = sync_converged();

	// Calculate the difference of this sample from the current estimate
	uint64_t deviation = llabs((int64_t)_time_offset - offset_us);

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
			double p = 1. - exp(0.5 * (1. - 1. / (1. - progress)));
			_filter_alpha = p * ALPHA_GAIN_FINAL + (1. - p) * ALPHA_GAIN_INITIAL;
			_filter_beta = p * BETA_GAIN_FINAL + (1. - p) * BETA_GAIN_INITIAL;

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
	}

	// Publish status message
	timesync_status_s timesync_status{};
	timesync_status.observed_offset = offset_us;
	timesync_status.estimated_offset = (int64_t)round(_time_offset);
	timesync_status.deviation = deviation;
	timesync_status.sequence = _sequence;
	timesync_status.source = _source;
	timesync_status.converged = sync_converged();
	timesync_status.timestamp = hrt_absolute_time();
	_timesync_status_pub.publish(timesync_status);

	if (!converged && sync_converged()) {
		PX4_DEBUG("new time offset: %.1f", _time_offset);
	}
}

void Timesync::add_sample(int64_t offset_us)
{
	/* Online exponential smoothing filter. The derivative of the estimate is also
	 * estimated in order to produce an estimate without steady state lag:
	 * https://en.wikipedia.org/wiki/Exponential_smoothing#Double_exponential_smoothing
	 */
	double time_offset_prev = _time_offset;

	if (_sequence == 0) { // First offset sample
		_time_offset = offset_us;

	} else {
		// Update the clock offset estimate
		_time_offset = _filter_alpha * offset_us + (1. - _filter_alpha) * (_time_offset + _time_skew);

		// Update the clock skew estimate
		_time_skew = _filter_beta * (_time_offset - time_offset_prev) + (1. - _filter_beta) * _time_skew;
	}
}

void Timesync::reset_filter()
{
	PX4_DEBUG("reset filter");

	// Do a full reset of all statistics and parameters
	_sequence = 0;
	_time_offset = 0.0;
	_time_skew = 0.0;
	_filter_alpha = ALPHA_GAIN_INITIAL;
	_filter_beta = BETA_GAIN_INITIAL;
	_high_deviation_count = 0;
}
