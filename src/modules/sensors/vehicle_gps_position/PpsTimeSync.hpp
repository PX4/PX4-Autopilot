/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file PpsTimeSync.hpp
 *
 * PPS-based time synchronization for GPS timestamp correction
 */

#pragma once

#include <stdint.h>
#include <uORB/topics/pps_capture.h>
#include <drivers/drv_hrt.h>

class PpsTimeSync
{
public:
	PpsTimeSync() = default;
	~PpsTimeSync() = default;

	void process_pps(const pps_capture_s &pps);
	uint64_t correct_gps_timestamp(uint64_t gps_fc_timestamp, uint64_t gps_utc_timestamp);

private:
	bool is_valid() const;

	uint64_t _pps_hrt_timestamp{0};	// FC time when PPS pulse arrived (usec since boot)
	uint64_t _pps_rtc_timestamp{0};	// GPS UTC time at PPS pulse (usec since Unix epoch)
	int64_t _time_offset{0};

	static constexpr uint64_t kPpsStaleTimeoutUs = 5'000'000;
	static constexpr int64_t kPpsMaxCorrectionUs = 300'000;	// max delay (max of EKF2_GPS_DELAY)

	bool _initialized{false};
	bool _updated{false};
};
