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

#include "autopilot_tester_gnss.h"

#include "math_helpers.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
#include <unistd.h>

namespace
{

// The reset moves the estimate from one receiver's error to the other's; what is left is the receivers' noise and
// the estimate's own error against the old receiver
constexpr float RESET_JUMP_TOLERANCE_M = 0.5f;

// A jump smaller than this isn't a reset of that axis
constexpr float NO_JUMP_M = 0.3f;

constexpr float NO_YAW_JUMP_RAD = 0.035f; // 2 deg

constexpr double LOG_DOWNLOAD_TIMEOUT_S = 120.;

// The logger closes the file shortly after the disarm; a size that holds this long means it is done
constexpr auto LOG_CLOSED_STABLE_TIME = std::chrono::seconds(2);
constexpr auto LOG_CLOSED_TIMEOUT = std::chrono::seconds(60);

// ULog header: 7 magic bytes, a version byte, then the log start time [us] since boot
bool ulog_start_time(const std::filesystem::path &path, uint64_t &start_us)
{
	static constexpr char MAGIC[7] = {'U', 'L', 'o', 'g', 0x01, 0x12, 0x35};
	char header[16];
	std::ifstream file(path, std::ios::binary);

	if (!file.read(header, sizeof(header)) || (memcmp(header, MAGIC, sizeof(MAGIC)) != 0)) {
		return false;
	}

	memcpy(&start_us, header + 8, sizeof(start_us));
	return true;
}

} // namespace

void AutopilotTesterGnss::connect(const std::string uri)
{
	set_event_callback([this](const Events::Event & event) {
		std::lock_guard<std::mutex> lock(_gnss_mutex);

		if (_gnss) {
			_gnss->on_event(event);
		}
	});

	_connected_at = std::filesystem::file_time_type::clock::now();
	AutopilotTester::connect(uri);

	{
		std::lock_guard<std::mutex> lock(_gnss_mutex);
		_gnss.reset(new GnssFailover(get_system(), false));
	}

	_gnss->set_record(&std::cout);

	REQUIRE(_gnss->request_streams());
	request_ground_truth();
}

void AutopilotTesterGnss::set_preferred_receiver(int instance)
{
	set_param_int("SENS_GNSS_PRIME", instance);
}

void AutopilotTesterGnss::set_param_float(const std::string &param, float value)
{
	CHECK(getParams()->set_param_float(param, value) == Param::Result::Success);
}

bool AutopilotTesterGnss::get_param_float(const std::string &param, float &value)
{
	const std::pair<Param::Result, float> result = getParams()->get_param_float(param);
	value = result.second;
	return result.first == Param::Result::Success;
}

void AutopilotTesterGnss::takeoff_and_hold(float altitude_m)
{
	set_takeoff_altitude(altitude_m);
	sleep_for(std::chrono::seconds(1)); // for the takeoff altitude to apply
	arm();
	takeoff();
	wait_until_hovering();
	wait_until_altitude(altitude_m, std::chrono::seconds(30));

	REQUIRE(_gnss->wait_until([this]() { return getTelemetry()->flight_mode() == Telemetry::FlightMode::Hold; }, 30.));

	// The in-flight magnetic yaw alignment resets the estimator shortly after takeoff
	sleep_for(std::chrono::seconds(5));
}

void AutopilotTesterGnss::mark()
{
	_mark.reset_count = _gnss->reset_count();
	_mark.reset_index = _gnss->resets().size();
	_mark.vehicle_time_us = _gnss->vehicle_time_us();
	_mark.ground_truth = getTelemetry()->ground_truth();

	if (_first_mark_us == 0) {
		_first_mark_us = _mark.vehicle_time_us;
	}

	REQUIRE(std::isfinite(_mark.ground_truth.latitude_deg));

	_gnss->record("mark");
}

void AutopilotTesterGnss::inject(const GnssFailover::Injection &injection)
{
	std::cout << time_str() << "Injecting " << injection << std::endl;
	CHECK(_gnss->inject(injection));
}

void AutopilotTesterGnss::inject_off(int instance)
{
	GnssFailover::Injection injection{};
	injection.type = Failure::FailureType::Off;
	injection.instance = instance;
	inject(injection);
}

void AutopilotTesterGnss::inject_without_ack(Failure::FailureType type, int instance)
{
	_gnss->inject_without_ack(type, instance);
}

void AutopilotTesterGnss::clear(int instance)
{
	std::cout << time_str() << "Clearing GNSS injection, instance " << instance << std::endl;
	CHECK(_gnss->clear(instance));
	CHECK(_gnss->restore_parameters());
}

unsigned AutopilotTesterGnss::resets_since_mark() const
{
	return _gnss->reset_count() - _mark.reset_count;
}

void AutopilotTesterGnss::wait_for_resets(unsigned count, std::chrono::seconds timeout)
{
	const bool reset = _gnss->wait_until([this, count]() { return resets_since_mark() >= count; },
	static_cast<double>(timeout.count()));

	if (!reset) {
		std::cout << time_str() << "Expected " << count << " estimator resets, saw " << resets_since_mark() << std::endl;
	}

	REQUIRE(reset);
}

std::array<float, 3> AutopilotTesterGnss::receiver_bias(int instance)
{
	std::array<float, 3> bias{};
	const char axes[3] = {'N', 'E', 'D'};

	for (int i = 0; i < 3; i++) {
		const std::string name = "SIM_GNSS" + std::to_string(instance) + "_BIAS_" + axes[i];
		const std::pair<Param::Result, float> value = getParams()->get_param_float(name);
		REQUIRE(value.first == Param::Result::Success);
		bias[i] = value.second;
	}

	return bias;
}

void AutopilotTesterGnss::check_switch_reset(int from, int to, bool vertical)
{
	const std::vector<GnssFailover::Reset> resets = _gnss->resets();
	REQUIRE(resets.size() > _mark.reset_index);

	// Horizontal and vertical resets can land in different estimator outputs
	std::array<float, 3> jump{};

	for (size_t i = _mark.reset_index; i < resets.size(); i++) {
		for (int axis = 0; axis < 3; axis++) {
			jump[axis] += resets[i].position_jump[axis];
		}
	}

	const std::array<float, 3> bias_from = receiver_bias(from);
	const std::array<float, 3> bias_to = receiver_bias(to);

	std::cout << time_str() << "Reset jump N " << jump[0] << " E " << jump[1] << " D " << jump[2]
		  << ", receiver offset N " << bias_to[0] - bias_from[0] << " E " << bias_to[1] - bias_from[1]
		  << " D " << bias_to[2] - bias_from[2] << std::endl;

	CHECK(std::hypot(jump[0] - (bias_to[0] - bias_from[0]), jump[1] - (bias_to[1] - bias_from[1]))
	      < RESET_JUMP_TOLERANCE_M);

	if (vertical) {
		CHECK(std::fabs(jump[2] - (bias_to[2] - bias_from[2])) < RESET_JUMP_TOLERANCE_M);

	} else {
		CHECK(std::fabs(jump[2]) < NO_JUMP_M);
	}
}

void AutopilotTesterGnss::check_held_horizontally(float tolerance_m)
{
	CHECK(ground_truth_horizontal_position_close_to(_mark.ground_truth, tolerance_m));
}

void AutopilotTesterGnss::check_held_vertically(float tolerance_m)
{
	const float climb = getTelemetry()->ground_truth().absolute_altitude_m - _mark.ground_truth.absolute_altitude_m;

	if (std::fabs(climb) >= tolerance_m) {
		std::cout << time_str() << "Altitude changed by " << climb << " m" << std::endl;
	}

	CHECK(std::fabs(climb) < tolerance_m);
}

void AutopilotTesterGnss::check_position_and_mode_kept(Telemetry::FlightMode mode)
{
	CHECK(_gnss->position_ok_since(_mark.vehicle_time_us));

	for (const Telemetry::FlightMode visited : _gnss->modes_since(_mark.vehicle_time_us)) {
		CHECK(visited == mode);
	}
}

void AutopilotTesterGnss::check_no_yaw_reset_since_mark()
{
	const std::vector<GnssFailover::Reset> resets = _gnss->resets();

	for (size_t i = _mark.reset_index; i < resets.size(); i++) {
		CHECK(std::fabs(resets[i].yaw_jump) < NO_YAW_JUMP_RAD);
	}
}

void AutopilotTesterGnss::wait_for_position_ok(bool ok, std::chrono::seconds timeout)
{
	REQUIRE(_gnss->wait_until([this, ok]() { return _gnss->position_ok() == ok; }, static_cast<double>(timeout.count())));
}

void AutopilotTesterGnss::wait_for_mode_other_than(Telemetry::FlightMode mode, std::chrono::seconds timeout)
{
	REQUIRE(_gnss->wait_until([this, mode]() { return getTelemetry()->flight_mode() != mode; },
	static_cast<double>(timeout.count())));
}

void AutopilotTesterGnss::check_switch_events(unsigned count)
{
	unsigned switches = 0;

	for (const GnssFailover::EventRecord &event : _gnss->events()) {
		if ((event.vehicle_time_us >= _mark.vehicle_time_us) && (event.name == "px4/gnss_receiver_switched")) {
			// The event names the new receiver and the reason
			CHECK(!event.message.empty());
			switches++;
		}
	}

	CHECK(switches == count);
}

std::string AutopilotTesterGnss::rootfs_flight_log()
{
	namespace fs = std::filesystem;

	// A case that arms again starts another log, so the newest file isn't necessarily the flight under test: take the
	// latest log that started before the first mark
	fs::path best;
	uint64_t best_start_us = 0;
	std::error_code error;

	for (const fs::directory_entry &entry : fs::recursive_directory_iterator(px4_rootfs,
			fs::directory_options::skip_permission_denied, error)) {
		uint64_t start_us = 0;

		if (entry.is_regular_file() && (entry.path().extension() == ".ulg")
		    && (entry.last_write_time() >= _connected_at) && ulog_start_time(entry.path(), start_us)
		    && ((_first_mark_us == 0) || (start_us <= static_cast<uint64_t>(_first_mark_us)))
		    && (best.empty() || (start_us > best_start_us))) {
			best = entry.path();
			best_start_us = start_us;
		}
	}

	if (best.empty()) {
		std::cout << time_str() << "No log of this flight in " << px4_rootfs << std::endl;
		return {};
	}

	// The logger closes the file after the disarm; until then its size keeps changing
	const auto deadline = std::chrono::steady_clock::now() + LOG_CLOSED_TIMEOUT;
	auto size = fs::file_size(best, error);
	auto stable_since = std::chrono::steady_clock::now();

	while (std::chrono::steady_clock::now() - stable_since < LOG_CLOSED_STABLE_TIME) {
		if (std::chrono::steady_clock::now() > deadline) {
			std::cout << time_str() << "Log " << best << " still grows " << LOG_CLOSED_TIMEOUT.count()
				  << " s after landing" << std::endl;
			return {};
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(250));
		const auto current = fs::file_size(best, error);

		if (current != size) {
			size = current;
			stable_since = std::chrono::steady_clock::now();
		}
	}

	return best.string();
}

void AutopilotTesterGnss::check_log(const std::string &checks, LogSource source)
{
	// The tests run from the source tree root, where the report lives
	const std::string report = "Tools/gnss_failover_report.py";
	REQUIRE(std::filesystem::exists(report));

	std::string path;
	bool downloaded = false;

	if ((source == LogSource::Rootfs) && !px4_rootfs.empty()) {
		path = rootfs_flight_log();
		REQUIRE(!path.empty());

	} else {
		path = (std::filesystem::temp_directory_path() / ("gnss_failover_" + std::to_string(getpid()) + ".ulg")).string();
		REQUIRE(_gnss->download_last_log(path, LOG_DOWNLOAD_TIMEOUT_S));
		downloaded = true;
	}

	const std::string command = "python3 " + report + " --checks " + checks + " " + path;
	std::cout << time_str() << command << std::endl;
	CHECK(std::system(command.c_str()) == 0);

	if (downloaded) {
		std::filesystem::remove(path);
	}
}

void AutopilotTesterGnss::start_mission_leg(double leg_length_m, float altitude_m)
{
	MissionOptions mission_options;
	mission_options.leg_length_m = leg_length_m;
	mission_options.relative_altitude_m = altitude_m;
	mission_options.rtl_at_end = false;
	prepare_straight_mission(mission_options);

	REQUIRE(_gnss->wait_until([this]() { return getMission()->start_mission() == Mission::Result::Success; }, 3.));
}

void AutopilotTesterGnss::wait_for_mission_finished(std::chrono::seconds timeout)
{
	REQUIRE(_gnss->wait_until([this]() {
		const std::pair<Mission::Result, bool> finished = getMission()->is_mission_finished();
		return finished.first == Mission::Result::Success && finished.second;
	}, static_cast<double>(timeout.count())));
}
