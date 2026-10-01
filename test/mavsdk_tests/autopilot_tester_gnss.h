/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#pragma once

#include "autopilot_tester.h"
#include "gnss_failover.h"

#include <array>
#include <memory>
#include <mutex>

/**
 * SIH failover between two simulated GNSS receivers. The receivers differ by their SIM_GNSSn_BIAS_N/E/D, so a switch
 * shows as an estimator reset by the difference of the biases.
 */
class AutopilotTesterGnss : public AutopilotTester
{
public:
	AutopilotTesterGnss() = default;
	~AutopilotTesterGnss() = default;

	void connect(const std::string uri);

	// SENS_GNSS_PRIME: preferred receiver, 0-based, -1 for none
	void set_preferred_receiver(int instance);

	void takeoff_and_hold(float altitude_m);

	// Starts a check window: the estimator reset count, the ground truth position, the mode and the time
	void mark();

	void inject(const GnssFailover::Injection &injection);
	void inject_off(int instance);
	void inject_without_ack(mavsdk::Failure::FailureType type, int instance);
	void clear(int instance = 0);

	// Waits until the estimator reset counter rose by at least this much since mark()
	void wait_for_resets(unsigned count, std::chrono::seconds timeout);

	// Reset counter increase since mark()
	unsigned resets_since_mark() const;

	// The first reset since mark() moved the position by the bias of receiver to minus the bias of receiver from
	// (0-based); vertical only when GNSS is the height reference
	void check_switch_reset(int from, int to, bool vertical);

	// The vehicle stayed where it was at mark(), by SIH ground truth
	void check_held_horizontally(float tolerance_m);
	void check_held_vertically(float tolerance_m);

	// Since mark(): position control stayed available and the vehicle stayed in this mode
	void check_position_and_mode_kept(mavsdk::Telemetry::FlightMode mode);

	void check_no_yaw_reset_since_mark();

	mavsdk::Telemetry::FlightMode flight_mode() { return getTelemetry()->flight_mode(); }

	void wait_for_position_ok(bool ok, std::chrono::seconds timeout);

	void wait_for_mode_other_than(mavsdk::Telemetry::FlightMode mode, std::chrono::seconds timeout);

	void start_mission_leg(double leg_length_m, float altitude_m);
	void wait_for_mission_finished(std::chrono::seconds timeout);

	void set_param_float(const std::string &param, float value);
	bool get_param_float(const std::string &param, float &value);

	GnssFailover &gnss() { return *_gnss; }

private:
	std::array<float, 3> receiver_bias(int instance);

	std::unique_ptr<GnssFailover> _gnss;
	std::mutex _gnss_mutex; ///< events arrive from a MAVSDK thread, possibly before _gnss exists

	struct Mark {
		unsigned reset_count{0};
		size_t reset_index{0};
		int64_t vehicle_time_us{0};
		Telemetry::GroundTruth ground_truth{};
	} _mark{};
};
