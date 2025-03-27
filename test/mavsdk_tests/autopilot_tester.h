/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include <mavsdk/mavsdk.h>
#include <mavsdk/geometry.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/failure/failure.h>
#include <mavsdk/plugins/info/info.h>
#include <mavsdk/plugins/manual_control/manual_control.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/mission_raw/mission_raw.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/param/param.h>
#include "catch2/catch.hpp"
#include <atomic>
#include <chrono>
#include <ctime>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <thread>

extern std::string connection_url;
extern std::optional<float> speed_factor;

using namespace mavsdk;
using namespace mavsdk::geometry;


inline std::string time_str()
{
	time_t rawtime;
	time(&rawtime);
	struct tm *timeinfo = localtime(&rawtime);
	char time_buffer[18];
	strftime(time_buffer, 18, "[%I:%M:%S|Info ] ", timeinfo);
	return time_buffer;
}

class AutopilotTester
{
public:
	struct MissionOptions {
		double leg_length_m {20.0};
		double relative_altitude_m {10.0};
		bool rtl_at_end {false};
		bool fly_through {false};
	};

	enum class HeightSource {
		Baro,
		Gps
	};

	enum class RcLossException {
		Mission = 0,
		Hold = 1,
		Offboard = 2
	};

	AutopilotTester();
	~AutopilotTester();

	void connect(const std::string uri);

	/**
	 * @brief Wait until vehicle's system status is healthy & is able to arm
	 */
	void wait_until_ready();

	void store_home();
	void check_home_within(float acceptance_radius_m);
	void check_home_not_within(float min_distance_m);
	void set_takeoff_altitude(const float altitude_m);
	void set_rtl_altitude(const float altitude_m);
	void set_height_source(HeightSource height_source);
	void set_rc_loss_exception(RcLossException mask);
	void set_param_vt_fwd_thrust_en(int value);
	void arm();
	void takeoff();
	void land();
	void transition_to_fixedwing();
	void transition_to_multicopter();
	void wait_until_disarmed(std::chrono::seconds timeout_duration = std::chrono::seconds(60));
	void wait_until_hovering(); // TODO: name suggests, that function waits for drone velocity to be zero and not just drone in the air
	void wait_until_altitude(float rel_altitude_m, std::chrono::seconds timeout, float delta = 0.5f);
	void wait_until_fixedwing(std::chrono::seconds timeout);
	void wait_until_speed_lower_than(float speed, std::chrono::seconds timeout);
	void prepare_square_mission(MissionOptions mission_options);
	void prepare_straight_mission(MissionOptions mission_options);
	void execute_mission();
	void execute_mission_and_lose_gps();
	void execute_mission_and_lose_mag();
	void execute_mission_and_get_mag_stuck();
	void execute_mission_and_lose_baro();
	void execute_mission_and_get_baro_stuck();
	void load_qgc_mission_raw_and_move_here(const std::string &plan_file);
	void execute_mission_raw();
	void execute_rtl();
	void execute_land();
	void offboard_goto(const Offboard::PositionNedYaw &target, float acceptance_radius_m = 0.3f,
			   std::chrono::seconds timeout_duration = std::chrono::seconds(60));
	void offboard_land();
	void fly_forward_in_posctl();
	void fly_forward_in_altctl();
	void fly_forward_in_offboard_attitude();
	void request_ground_truth();
	void check_mission_item_speed_above(int item_index, float min_speed_m_s);
	void check_tracks_mission(float corridor_radius_m = 1.5f);
	void check_tracks_mission_raw(float corridor_radius_m = 1.f, bool reverse = false);
	void check_mission_land_within(float acceptance_radius_m);
	void start_checking_altitude(const float max_deviation_m);
	void stop_checking_altitude();
	void check_current_altitude(float target_rel_altitude_m, float max_distance_m = 1.5f);
	void execute_rtl_when_reaching_mission_sequence(int sequence_number);
	void send_custom_mavlink_command(const MavlinkPassthrough::CommandInt &command);
	void send_custom_mavlink_message(mavlink_message_t &message);
	void add_mavlink_message_callback(uint16_t message_id, std::function< void(const mavlink_message_t &)> callback);

	void enable_fixedwing_mectrics();
	void check_airspeed_is_valid();
	void check_airspeed_is_invalid();

	// Blocking call to get the drone's current position in NED frame
	std::array<float, 3> get_current_position_ned();

	void set_param_int(const std::string &param, int32_t value)
	{
		CHECK(_param->set_param_int(param, value) == Param::Result::Success);
	}

	template<typename Rep, typename Period>
	void sleep_for(std::chrono::duration<Rep, Period> duration)
	{
		const std::chrono::microseconds duration_us(duration);

		if (_telemetry && _telemetry->attitude_quaternion().timestamp_us != 0) {

			const int64_t start_time_us = _telemetry->attitude_quaternion().timestamp_us;

			while (true) {
				// Hopefully this is often enough not to have PX4 time out on us.
				std::this_thread::sleep_for(std::chrono::milliseconds(1));

				const int64_t elapsed_time_us = _telemetry->attitude_quaternion().timestamp_us - start_time_us;

				if (elapsed_time_us > duration_us.count()) {
					return;
				}
			}

		} else {
			std::this_thread::sleep_for(duration);
		}
	}

protected:
	mavsdk::Param *getParams() const { return _param.get();}
	mavsdk::Telemetry *getTelemetry() const { return _telemetry.get();}
	mavsdk::ManualControl *getManualControl() const { return _manual_control.get();}
	MavlinkPassthrough *getMavlinkPassthrough() const { return _mavlink_passthrough.get();}
	std::shared_ptr<System> get_system() { return _mavsdk.systems().at(0);}
	mavsdk::geometry::CoordinateTransformation get_coordinate_transformation();
	bool ground_truth_horizontal_position_close_to(const Telemetry::GroundTruth &target_pos, float acceptance_radius_m);

	const Telemetry::GroundTruth &getHome()
	{
		// Check if home was stored before it is accessed
		CHECK(_home.absolute_altitude_m != NAN);
		CHECK(_home.latitude_deg != NAN);
		CHECK(_home.longitude_deg != NAN);
		return _home;
	}

private:
	mavsdk::Mission::MissionItem create_mission_item(
		const mavsdk::geometry::CoordinateTransformation::LocalCoordinate &local_coordinate,
		const MissionOptions &mission_options,
		const mavsdk::geometry::CoordinateTransformation &ct);

	bool ground_truth_horizontal_position_far_from(const Telemetry::GroundTruth &target_pos, float min_distance_m);
	bool estimated_position_close_to(const Offboard::PositionNedYaw &target_pos, float acceptance_radius_m);
	bool estimated_horizontal_position_close_to(const Offboard::PositionNedYaw &target_pos, float acceptance_radius_m);
	void start_and_wait_for_mission_sequence(int sequence_number);
	void start_and_wait_for_mission_sequence_raw(int sequence_number);
	void wait_for_flight_mode(Telemetry::FlightMode flight_mode, std::chrono::seconds timeout);
	void wait_for_landed_state(Telemetry::LandedState landed_state, std::chrono::seconds timeout);
	void wait_for_mission_finished(std::chrono::seconds timeout);
	void wait_for_mission_raw_finished(std::chrono::seconds timeout);
	void move_mission_raw_here(std::vector<mavsdk::MissionRaw::MissionItem> &mission_items);

	void report_speed_factor();

	/**
	 * @brief Continue polling until condition returns true or we have a timeout
	 *
	 * @param fun Boolean returning function. When true, the polling terminates.
	 * @param duration Timeout for polling in `std::chrono::` time unit
	 */
	template<typename Rep, typename Period>
	bool poll_condition_with_timeout(
		std::function<bool()> fun, std::chrono::duration<Rep, Period> duration)
	{
		static constexpr unsigned check_resolution = 100;

		const std::chrono::microseconds duration_us(duration);

		if (_telemetry && _telemetry->attitude_quaternion().timestamp_us != 0) {
			// A system is connected. We can base the timeouts on the autopilot time.
			const int64_t start_time_us = _telemetry->attitude_quaternion().timestamp_us;

			while (!fun()) {
				std::this_thread::sleep_for(duration_us / check_resolution);

				// This might potentially loop forever and the test needs to be killed by a watchdog outside.
				// The reason not to include an absolute timeout here is that it can happen if the host is
				// busy and PX4 doesn't run fast enough.
				const int64_t elapsed_time_us = _telemetry->attitude_quaternion().timestamp_us - start_time_us;

				if (elapsed_time_us > duration_us.count()) {
					std::cout << time_str() << "Timeout, connected to vehicle but waiting for test for " << static_cast<double>
						  (elapsed_time_us) / 1e6 << " seconds\n";
					return false;
				}
			}

		} else {
			// Nothing is connected yet. Use the host time.
			const auto start_time = std::chrono::steady_clock::now();

			while (!fun()) {
				std::this_thread::sleep_for(duration_us / check_resolution);
				const auto elapsed_time_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() -
							     start_time);

				if (elapsed_time_us > duration_us) {
					std::cout << time_str() << "Timeout, waiting for the vehicle for "
						  << elapsed_time_us.count() * std::chrono::steady_clock::period::num
						  / static_cast<double>(std::chrono::steady_clock::period::den)
						  << " seconds\n";
					return false;
				}
			}
		}

		return true;
	}


	mavsdk::Mavsdk _mavsdk{};
	std::unique_ptr<mavsdk::Action> _action{};
	std::unique_ptr<mavsdk::Failure> _failure{};
	std::unique_ptr<mavsdk::Info> _info{};
	std::unique_ptr<mavsdk::ManualControl> _manual_control{};
	std::unique_ptr<MavlinkPassthrough> _mavlink_passthrough;
	std::unique_ptr<mavsdk::Mission> _mission{};
	std::unique_ptr<mavsdk::MissionRaw> _mission_raw{};
	std::unique_ptr<mavsdk::Offboard> _offboard{};
	std::unique_ptr<mavsdk::Param> _param{};
	std::unique_ptr<mavsdk::Telemetry> _telemetry{};

	Telemetry::GroundTruth _home{NAN, NAN, NAN};

	std::atomic<bool> _should_exit {false};
	std::thread _real_time_report_thread {};
};
