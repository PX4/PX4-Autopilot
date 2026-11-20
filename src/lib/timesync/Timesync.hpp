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
 * @file Timesync.hpp
 * time synchroniser definition.
 *
 * @author Mohammed Kabir <mhkabir98@gmail.com>
 */

#pragma once

#include <drivers/drv_hrt.h>

#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/timesync_status.h>

#include <math.h>
#include <float.h>

using namespace time_literals;

static constexpr time_t PX4_EPOCH_SECS = 1234567890ULL;

// Kalman filter tuning. State: offset [s] and skew [s/s], predicted with the actual time
// between samples, so the response does not depend on the timesync rate.
// MEASUREMENT_NOISE_STD : std of an observed offset [s], mostly asymmetric link delay
// OFFSET_RANDOM_WALK : offset random walk density [s^2/s], damps the response to offset steps
// SKEW_RANDOM_WALK : skew random walk density [1/s], higher follows clock corrections faster
// INITIAL_SKEW_STD : prior skew std [s/s], i.e. crystal tolerance
static constexpr double MEASUREMENT_NOISE_STD = 1e-3;
static constexpr double OFFSET_RANDOM_WALK = 1e-8;
static constexpr double SKEW_RANDOM_WALK = 7e-13;
static constexpr double INITIAL_SKEW_STD = 100e-6;

// Number of exchanged timesync packets before the estimate is considered converged.
static constexpr uint32_t CONVERGENCE_WINDOW = 500;

// Outlier rejection and filter reset
//
// Samples with round-trip time higher than MAX_RTT_SAMPLE are not used to update the filter.
// More than MAX_CONSECUTIVE_HIGH_RTT number of such events in a row will throw a debug message
// but not reset the filter.
// Samples whose calculated clock offset is more than MAX_DEVIATION_SAMPLE off from the current
// estimate are not used to update the filter. More than MAX_CONSECUTIVE_HIGH_DEVIATION number
// of such events in a row will reset the filter. This usually happens only due to a time jump
// on the remote system.
// TODO : automatically determine these using ping statistics?
static constexpr uint64_t MAX_RTT_SAMPLE = 10_ms;
static constexpr uint64_t MAX_DEVIATION_SAMPLE = 100_ms;
static constexpr uint32_t MAX_CONSECUTIVE_HIGH_RTT = 10;
static constexpr uint32_t MAX_CONSECUTIVE_HIGH_DEVIATION = 10;

class Timesync
{
public:
	Timesync(uint8_t source = timesync_status_s::SOURCE_PROTOCOL_UNKNOWN) : _source(source) {};
	~Timesync() = default;

	void update(const uint64_t now_us, const int64_t remote_timestamp_ns, int64_t originate_timestamp_ns);

	/**
	 * Convert remote timestamp to local hrt time (usec)
	 * Use synchronised time if available, monotonic boot time otherwise
	 */
	uint64_t sync_stamp(uint64_t usec);

	int64_t offset() const { return (int64_t)(_time_offset * 1e6); } // [us]

	/**
	 * Return true if the timesync algorithm converged to a good estimate,
	 * return false otherwise
	 */
	bool sync_converged() const { return _sequence >= CONVERGENCE_WINDOW; }

	/**
	 * Reset the exponential filter and its states
	 */
	void reset_filter();


private:

	/**
	 * Kalman filter update of time offset and skew
	 */
	void add_sample(int64_t offset_us, uint64_t now_us);
	uORB::PublicationMulti<timesync_status_s>  _timesync_status_pub{ORB_ID(timesync_status)};

	uint32_t _sequence{0};

	// Timesync statistics
	double _time_offset{0}; // [s]
	double _time_skew{0};   // [s/s]

	// Filter covariance
	double _var_offset{0};      // [s^2]
	double _cov_offset_skew{0}; // [s]
	double _var_skew{0};        // [-]
	uint64_t _last_sample_us{0};

	// Outlier rejection and filter reset
	uint32_t _high_deviation_count{0};
	uint32_t _high_rtt_count{0};

	uint8_t _source{};
};
