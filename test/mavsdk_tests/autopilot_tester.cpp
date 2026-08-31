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

#include "autopilot_tester.h"
#include "math_helpers.h"
#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <iostream>
#include <future>
#include <mutex>
#include <thread>
#include <unistd.h>
#include <cmath>

std::string connection_url {"udp://"};
std::optional<float> speed_factor {std::nullopt};

AutopilotTester::AutopilotTester() :
	_real_time_report_thread([this]()
{
	report_speed_factor();
})
{
}

AutopilotTester::~AutopilotTester()
{
	// connect() can fail before the plugins exist, and a destructor that dereferences them
	// turns the reported timeout into a segfault that hides it
	if (_events) {
		_events->unsubscribe_events(_events_handle);
	}

	_should_exit = true;
	_real_time_report_thread.join();
}

void AutopilotTester::connect(const std::string uri)
{
	ConnectionResult ret = _mavsdk.add_any_connection(uri);
	REQUIRE(ret == ConnectionResult::Success);

	std::cout << time_str() << "Waiting for system connect" << std::endl;
	REQUIRE(poll_condition_with_timeout(
	[this]() { return _mavsdk.systems().size() > 0; }, std::chrono::seconds(25)));

	auto system = get_system();

	_action.reset(new Action(system));
	_failure.reset(new Failure(system));
	_info.reset(new Info(system));
	_manual_control.reset(new ManualControl(system));
	_mission.reset(new Mission(system));
	_mission_raw.reset(new MissionRaw(system));
	_offboard.reset(new Offboard(system));
	_param.reset(new Param(system));
	_telemetry.reset(new Telemetry(system));
	_events.reset(new Events(system));
	_shell.reset(new Shell(system));
	_mavlink_passthrough.reset(new MavlinkPassthrough(system));

	_events_handle = _events->subscribe_events([](const Events::Event & event) {
		std::cout << "[" << event.log_level << "] " << event.message << std::endl;

		if (!event.description.empty()) {
			std::cout << "    Description: " << event.description << std::endl;
		}

		std::cout << "    Event name: " << event.event_namespace << "/" << event.event_name
			  << std::endl;
	});
}

void AutopilotTester::wait_until_ready()
{
	std::cout << time_str() << "Waiting for system to be ready (system health ok & able to arm)" << std::endl;

	// Wait until the system is healthy
	CHECK(poll_condition_with_timeout(
	[this]() { return _telemetry->health_all_ok(); }, std::chrono::seconds(30)));

	// Note: There is a known bug in MAVSDK (https://github.com/mavlink/MAVSDK/issues/1852),
	// where `health_all_ok()` returning true doesn't actually mean vehicle is ready to accept
	// global position estimate as valid (due to hysteresis). This needs to be fixed properly.

	// However, this is mitigated by the `is_armable` check below as a side effect, since
	// when the vehicle considers global position to be valid, it will then allow arming

	// Wait until we can arm
	CHECK(poll_condition_with_timeout(
	[this]() {	return _telemetry->health().is_armable;	}, std::chrono::seconds(45)));
}

void AutopilotTester::store_home()
{
	request_ground_truth();
	std::cout << time_str() << "Waiting to get home position" << std::endl;
	CHECK(poll_condition_with_timeout(
	[this]() {
		_home = _telemetry->ground_truth();
		return std::isfinite(_home.latitude_deg) && std::isfinite(_home.longitude_deg);
	}, std::chrono::seconds(10)));
}

void AutopilotTester::check_home_within(float acceptance_radius_m)
{
	CHECK(ground_truth_horizontal_position_close_to(_home, acceptance_radius_m));
}

void AutopilotTester::check_home_not_within(float min_distance_m)
{
	CHECK(ground_truth_horizontal_position_far_from(_home, min_distance_m));
}

void AutopilotTester::set_takeoff_altitude(const float altitude_m)
{
	CHECK(Action::Result::Success == _action->set_takeoff_altitude(altitude_m));
	const auto result = _action->get_takeoff_altitude();
	CHECK(result.first == Action::Result::Success);
	CHECK(result.second == Approx(altitude_m));
}

void AutopilotTester::set_rtl_altitude(const float altitude_m)
{
	CHECK(Action::Result::Success == _action->set_return_to_launch_altitude(altitude_m));
	const auto result = _action->get_return_to_launch_altitude();
	CHECK(result.first == Action::Result::Success);
	CHECK(result.second == Approx(altitude_m));
}

void AutopilotTester::set_height_source(AutopilotTester::HeightSource height_source)
{
	switch (height_source) {
	case HeightSource::Baro:
		CHECK(_param->set_param_int("EKF2_HGT_REF", 0) == Param::Result::Success);
		break;

	case HeightSource::Gps:
		CHECK(_param->set_param_int("EKF2_HGT_REF", 1) == Param::Result::Success);
	}
}

void AutopilotTester::set_rc_loss_exception(AutopilotTester::RcLossException mask)
{
	switch (mask) {
	case RcLossException::Mission:
		CHECK(_param->set_param_int("COM_RCL_EXCEPT", 1 << 0) == Param::Result::Success);
		break;

	case RcLossException::Hold:
		CHECK(_param->set_param_int("COM_RCL_EXCEPT", 1 << 1) == Param::Result::Success);
		break;

	case RcLossException::Offboard:
		CHECK(_param->set_param_int("COM_RCL_EXCEPT", 1 << 2) == Param::Result::Success);
	}
}

void AutopilotTester::set_param_vt_fwd_thrust_en(int value)
{
	CHECK(_param->set_param_int("VT_FWD_THRUST_EN", value) == Param::Result::Success);
}

void AutopilotTester::arm()
{
	// Even after wait_until_ready() passes, the is_armable health flag can
	// transiently flicker (e.g. the estimator's heading reference briefly
	// dropping out during startup), causing the arm command to be denied.
	// Retry until the autopilot actually accepts it.
	REQUIRE(poll_condition_with_timeout(
	[this]() { return _action->arm() == Action::Result::Success; }, std::chrono::seconds(30)));
}

void AutopilotTester::takeoff()
{
	const auto result = _action->takeoff();
	REQUIRE(result == Action::Result::Success);
}

void AutopilotTester::land()
{
	const auto result = _action->land();
	REQUIRE(result == Action::Result::Success);
}

void AutopilotTester::transition_to_fixedwing()
{
	const auto result = _action->transition_to_fixedwing();
	REQUIRE(result == Action::Result::Success);
}

void AutopilotTester::transition_to_multicopter()
{
	const auto result = _action->transition_to_multicopter();
	REQUIRE(result == Action::Result::Success);
}

void AutopilotTester::wait_until_multicopter(std::chrono::seconds timeout)
{
	REQUIRE(poll_condition_with_timeout(
	[this]() { return _telemetry->vtol_state() == Telemetry::VtolState::Mc; }, timeout));
}

void AutopilotTester::wait_until_disarmed(std::chrono::seconds timeout_duration)
{
	REQUIRE(poll_condition_with_timeout(
	[this]() { return !_telemetry->armed(); }, timeout_duration));
}

void AutopilotTester::wait_until_hovering()
{
	wait_for_landed_state(Telemetry::LandedState::InAir, std::chrono::seconds(45));
}

void AutopilotTester::wait_until_altitude(float rel_altitude_m, std::chrono::seconds timeout, float delta)
{
	REQUIRE(poll_condition_with_timeout(
	[this, rel_altitude_m, delta]() {
		return fabs(rel_altitude_m + _telemetry->position_velocity_ned().position.down_m) <= delta;
	}, timeout));
}

void AutopilotTester::wait_until_fixedwing(std::chrono::seconds timeout)
{
	REQUIRE(poll_condition_with_timeout(
	[this]() { return _telemetry->vtol_state() == Telemetry::VtolState::Fw; }, timeout));
}

void AutopilotTester::prepare_square_mission(MissionOptions mission_options)
{
	const auto ct = get_coordinate_transformation();

	Mission::MissionPlan mission_plan {};
	mission_plan.mission_items.push_back(create_mission_item({mission_options.leg_length_m, 0.}, mission_options, ct));
	mission_plan.mission_items.push_back(create_mission_item({mission_options.leg_length_m, mission_options.leg_length_m},
					     mission_options, ct));
	mission_plan.mission_items.push_back(create_mission_item({0., mission_options.leg_length_m}, mission_options, ct));

	_mission->set_return_to_launch_after_mission(mission_options.rtl_at_end);

	REQUIRE(_mission->upload_mission(mission_plan) == Mission::Result::Success);
	// PX4 needs time to realize that it now has a mission available, so we need to wait a bit here.
	sleep_for(std::chrono::seconds(1));
}

void AutopilotTester::prepare_straight_mission(MissionOptions mission_options)
{
	const auto ct = get_coordinate_transformation();

	Mission::MissionPlan mission_plan {};
	mission_plan.mission_items.push_back(create_mission_item({0, 0.}, mission_options, ct));
	mission_plan.mission_items.push_back(create_mission_item({mission_options.leg_length_m, 0}, mission_options, ct));
	mission_plan.mission_items.push_back(create_mission_item({2 * mission_options.leg_length_m, 0}, mission_options, ct));
	mission_plan.mission_items.push_back(create_mission_item({3 * mission_options.leg_length_m, 0}, mission_options, ct));
	mission_plan.mission_items.push_back(create_mission_item({4 * mission_options.leg_length_m, 0}, mission_options, ct));

	_mission->set_return_to_launch_after_mission(mission_options.rtl_at_end);

	REQUIRE(_mission->upload_mission(mission_plan) == Mission::Result::Success);
	// PX4 needs time to realize that it now has a mission available, so we need to wait a bit here.
	sleep_for(std::chrono::seconds(1));
}

void AutopilotTester::execute_mission()
{
	std::promise<void> prom;
	auto fut = prom.get_future();


	REQUIRE(poll_condition_with_timeout(
	[this]() { return _mission->start_mission() == Mission::Result::Success; }, std::chrono::seconds(3)));

	// poll_condition_with_timeout uses autopilot sim time, so pass sim-time timeout directly
	wait_for_mission_finished(std::chrono::seconds(500));
}

void AutopilotTester::execute_mission_and_lose_gps()
{
	CHECK(_param->set_param_int("SYS_FAILURE_EN", 1) == Param::Result::Success);

	start_and_wait_for_mission_sequence(1);

	CHECK(_failure->inject(Failure::FailureUnit::SensorGps, Failure::FailureType::Off, 0) == Failure::Result::Success);

	// With no position aiding left, a blind descend is performed. MAVSDK has no
	// Descend flight mode, so the caller just waits for the disarm after landing.
}

void AutopilotTester::execute_mission_and_lose_mag()
{
	CHECK(_param->set_param_int("SYS_FAILURE_EN", 1) == Param::Result::Success);

	start_and_wait_for_mission_sequence(1);

	CHECK(_failure->inject(Failure::FailureUnit::SensorMag, Failure::FailureType::Off, 0) == Failure::Result::Success);

	// We except the mission to continue without mag just fine.
	REQUIRE(poll_condition_with_timeout(
	[this]() {
		auto result = _mission->is_mission_finished();
		return result.first == Mission::Result::Success && result.second;
	}, std::chrono::seconds(90)));
}

void AutopilotTester::execute_mission_and_lose_baro()
{
	CHECK(_param->set_param_int("SYS_FAILURE_EN", 1) == Param::Result::Success);

	start_and_wait_for_mission_sequence(1);

	CHECK(_failure->inject(Failure::FailureUnit::SensorBaro, Failure::FailureType::Off, 0) == Failure::Result::Success);

	// We except the mission to continue without baro just fine.
	REQUIRE(poll_condition_with_timeout(
	[this]() {
		auto result = _mission->is_mission_finished();
		return result.first == Mission::Result::Success && result.second;
	}, std::chrono::seconds(90)));
}

// How far the vehicle is allowed to leave its altitude while an IMU fault is active,
// measured against the simulator rather than the estimator being disturbed.
static constexpr float kFailoverGroundTruthBandM = 5.f;

// How far the hovering vehicle may drop below the altitude it was holding while an IMU is
// clipping. It hovers at 20 m, so this keeps it well clear of the ground.
static constexpr float kSelectorAltitudeFloorM = 12.f;

// And how far it may climb. A railed accelerometer reads as falling, so the controller answers
// with thrust and the vehicle goes up rather than down. That transient reaches 25 m in the
// worst run measured here, while losing the second estimator instead sends it past 50 m and it
// never comes back, so this sits between the two.
static constexpr float kSelectorAltitudeCeilingM = 40.f;

// How far it may still be from where it started once the faults are cleared and it has
// settled again.
static constexpr float kSelectorRecoveryToleranceM = 3.f;

// SIH_FAULT_VIBE amplitude used to clip an accelerometer. It is the standard deviation of the
// injected noise, and the driver reports a sample as clipped once it reaches the 16 g
// measurement range, about 157 m/s2. At this amplitude about three quarters of raw samples
// reach the rail, which keeps EKF2's clipping counter climbing rather than sitting near the
// break even point where it steps back down.
static constexpr float kImuClippingFaultAmplitude = 500.f;

// SCALED_IMU reports the integrated vehicle_imu acceleration in milli g, not raw samples, so
// the railed samples are averaged with the clean ones over each integration window. A hovering
// quad sits near 1000 and cannot exceed its thrust to weight ceiling of about 2000 whatever it
// is commanded, while a clipped IMU still reports many thousands. The bands do not overlap.
static constexpr int kClippedImuPeakMg = 4000;
static constexpr int kCleanImuPeakMg = 3000;

// Clears the injected fault however the scope is left. A failing REQUIRE throws, and without
// this the simulator would keep clipping an accelerometer for the rest of the process.
class ImuFaultGuard
{
public:
	explicit ImuFaultGuard(mavsdk::Param *param) : _param(param) {}
	~ImuFaultGuard()
	{
		if (_param != nullptr) {
			_param->set_param_int("SIH_FAULT_IMU", 0);
			_param->set_param_float("SIH_FAULT_VIBE", 0.f);
		}
	}

	ImuFaultGuard(const ImuFaultGuard &) = delete;
	ImuFaultGuard &operator=(const ImuFaultGuard &) = delete;

private:
	mavsdk::Param *_param;
};

// A parameter write has to reach SIH and the samples already in flight have to drain before
// the next phase starts measuring.
static constexpr std::chrono::seconds kFaultSettleTime{3};

// Long enough to collect many SCALED_IMU samples at 50 Hz and to outlast the 3 second
// BADACC_PROBATION that EKF2 holds after clipping stops, so one phase does not bleed into
// the next.
static constexpr std::chrono::seconds kFaultMeasureTime{17};

// Ground truth arrives as HIL_STATE_QUATERNION, about 15 Hz of simulated time here. Requiring
// a third of that over a window catches a stream that stalled or was starved under the speed
// factor, which would otherwise leave every maximum at its starting value and read as a pass.
static constexpr int kMinGroundTruthRateHz = 5;

// How long the selector may take to leave the instance behind a clipped IMU. Measured here
// it takes a few seconds, most of that the health debounce.
static constexpr std::chrono::seconds kHandoverTimeout{30};

int AutopilotTester::primary_estimator_instance()
{
	// The reply arrives in chunks on the receive thread, so collect until the field and the
	// end of its line are both there.
	auto output = std::make_shared<std::string>();
	auto output_mutex = std::make_shared<std::mutex>();
	Shell::ReceiveHandle handle = _shell->subscribe_receive([output, output_mutex](std::string chunk) {
		std::lock_guard<std::mutex> lock(*output_mutex);
		*output += chunk;
	});

	const Shell::Result send_result = _shell->send("listener estimator_selector_status -n 1\n");
	const std::string field = "primary_instance:";
	int instance = -1;

	const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);

	while (std::chrono::steady_clock::now() < deadline) {
		{
			std::lock_guard<std::mutex> lock(*output_mutex);
			const size_t begin = output->find(field);

			if ((begin != std::string::npos) && (output->find('\n', begin) != std::string::npos)) {
				instance = std::atoi(output->c_str() + begin + field.size());
				break;
			}
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}

	_shell->unsubscribe_receive(handle);

	if (instance < 0) {
		std::lock_guard<std::mutex> lock(*output_mutex);
		std::cout << time_str() << "no primary_instance in the shell reply, send result " << send_result
			  << ", reply: " << *output << std::endl;
	}

	return instance;
}

void AutopilotTester::execute_alternating_imu_faults()
{
	// SCALED_IMU and SCALED_IMU2 carry vehicle_imu instance 0 and 1. Clipping one
	// accelerometer shows up there as a large swing on that stream and on no other, so
	// watching both is what tells this test the fault landed on the IMU it asked for.
	// SIH_FAULT_IMU is 1 based, so 1 is the stream behind SCALED_IMU and 2 is SCALED_IMU2.
	auto peak_imu1_mg = std::make_shared<std::atomic<int>>(0);
	auto peak_imu2_mg = std::make_shared<std::atomic<int>>(0);

	request_message_interval(MAVLINK_MSG_ID_SCALED_IMU, 50.f);
	request_message_interval(MAVLINK_MSG_ID_SCALED_IMU2, 50.f);

	auto samples_imu1 = std::make_shared<std::atomic<int>>(0);
	auto samples_imu2 = std::make_shared<std::atomic<int>>(0);

	auto track_peak = [](std::shared_ptr<std::atomic<int>> peak, std::shared_ptr<std::atomic<int>> samples,
	int16_t zacc) {
		const int magnitude = std::abs((int)zacc);

		// A load followed by a store would let a callback that read the old peak write it back
		// after the phase reset, carrying a faulted value into a clean window.
		int previous = peak->load();

		while ((magnitude > previous) && !peak->compare_exchange_weak(previous, magnitude)) {
		}

		samples->fetch_add(1);
	};

	add_mavlink_message_callback(MAVLINK_MSG_ID_SCALED_IMU, [peak_imu1_mg, samples_imu1, track_peak](
	const mavlink_message_t &message) {
		mavlink_scaled_imu_t imu;
		mavlink_msg_scaled_imu_decode(&message, &imu);
		track_peak(peak_imu1_mg, samples_imu1, imu.zacc);
	});

	add_mavlink_message_callback(MAVLINK_MSG_ID_SCALED_IMU2, [peak_imu2_mg, samples_imu2, track_peak](
	const mavlink_message_t &message) {
		mavlink_scaled_imu2_t imu;
		mavlink_msg_scaled_imu2_decode(&message, &imu);
		track_peak(peak_imu2_mg, samples_imu2, imu.zacc);
	});

	// The recovery is judged against the simulator rather than the estimator, which is the
	// thing these faults are disturbing.
	auto truth_altitude_m = std::make_shared<std::atomic<float>>(NAN);
	auto truth_altitude_start_m = std::make_shared<std::atomic<float>>(NAN);
	auto truth_lowest_m = std::make_shared<std::atomic<float>>(0.f);
	auto truth_highest_m = std::make_shared<std::atomic<float>>(0.f);
	auto truth_samples = std::make_shared<std::atomic<int>>(0);
	auto truth_handle = _telemetry->subscribe_ground_truth([truth_altitude_m, truth_altitude_start_m,
			  truth_lowest_m, truth_highest_m, truth_samples](Telemetry::GroundTruth ground_truth) {
		const float altitude_m = ground_truth.absolute_altitude_m;

		if (!std::isfinite(altitude_m)) {
			return;
		}

		truth_altitude_m->store(altitude_m);

		const float start_m = truth_altitude_start_m->load();

		if (std::isfinite(start_m)) {
			// Tracked signed so the two directions can be bounded separately
			const float offset_m = altitude_m - start_m;

			float lowest = truth_lowest_m->load();

			while ((offset_m < lowest) && !truth_lowest_m->compare_exchange_weak(lowest, offset_m)) {
			}

			float highest = truth_highest_m->load();

			while ((offset_m > highest) && !truth_highest_m->compare_exchange_weak(highest, offset_m)) {
			}

			truth_samples->fetch_add(1);
		}
	});

	REQUIRE(poll_condition_with_timeout(
	[peak_imu1_mg, peak_imu2_mg, truth_altitude_m]() {
		return (peak_imu1_mg->load() > 0) && (peak_imu2_mg->load() > 0)
		       && std::isfinite(truth_altitude_m->load());
	}, std::chrono::seconds(20)));

	// From here on every ground truth sample feeds the running maximum, so a dip and a
	// recovery inside one phase cannot hide between two samples.
	truth_altitude_start_m->store(truth_altitude_m->load());

	// Where the selector starts. SIH brings both instances up together and the selector
	// takes the first one, so the handovers below are changes rather than coincidences.
	const int initial_instance = primary_estimator_instance();
	std::cout << time_str() << "primary estimator instance before the faults " << initial_instance << std::endl;
	CHECK(initial_instance == 0);

	// Each measurement window has to have been observed at a sane rate, otherwise a stalled
	// ground truth stream leaves the extrema at zero and every altitude check passes.
	const int min_truth_samples = kFaultMeasureTime.count() * kMinGroundTruthRateHz;

	ImuFaultGuard fault_guard(_param.get());

	// Clip the accelerometer behind the first instance. If the parameters are missing there is
	// nothing left to test, so stop rather than fly the rest of the sequence. Each phase gives
	// the write time to reach SIH and lets the samples already in flight drain, then zeroes
	// the peaks and measures, otherwise the previous phase's clipping lands in this one.
	REQUIRE(_param->set_param_float("SIH_FAULT_VIBE", kImuClippingFaultAmplitude) == Param::Result::Success);
	REQUIRE(_param->set_param_int("SIH_FAULT_IMU", 1) == Param::Result::Success);
	sleep_for(kFaultSettleTime);
	peak_imu1_mg->store(0);
	peak_imu2_mg->store(0);
	samples_imu1->store(0);
	samples_imu2->store(0);
	truth_samples->store(0);
	sleep_for(kFaultMeasureTime);

	// A stream that stopped arriving would leave its peak at zero and satisfy every clean
	// check, so each phase proves it measured something first.
	REQUIRE(samples_imu1->load() > 0);
	REQUIRE(samples_imu2->load() > 0);
	REQUIRE(truth_samples->load() >= min_truth_samples);

	// The first IMU is being clipped and the second one is not.
	CHECK(peak_imu1_mg->load() > kClippedImuPeakMg);
	CHECK(peak_imu2_mg->load() < kCleanImuPeakMg);

	// The selector has to have left the instance behind the clipped IMU.
	const int instance_with_first_imu_clipped = primary_estimator_instance();
	std::cout << time_str() << "primary estimator instance with the first IMU clipped "
		  << instance_with_first_imu_clipped << ", ground truth samples " << truth_samples->load() << std::endl;
	CHECK(instance_with_first_imu_clipped == 1);

	// Move the clipping to the other IMU. The fault has to follow the parameter, which is
	// what says the injection is per instance rather than shared.
	REQUIRE(_param->set_param_int("SIH_FAULT_IMU", 2) == Param::Result::Success);
	sleep_for(kFaultSettleTime);
	peak_imu1_mg->store(0);
	peak_imu2_mg->store(0);
	samples_imu1->store(0);
	samples_imu2->store(0);
	truth_samples->store(0);
	sleep_for(kFaultMeasureTime);

	REQUIRE(samples_imu1->load() > 0);
	REQUIRE(samples_imu2->load() > 0);
	REQUIRE(truth_samples->load() >= min_truth_samples);

	CHECK(peak_imu2_mg->load() > kClippedImuPeakMg);
	CHECK(peak_imu1_mg->load() < kCleanImuPeakMg);

	// And back again, now that the clipping has moved to the other one.
	const int instance_with_second_imu_clipped = primary_estimator_instance();
	std::cout << time_str() << "primary estimator instance with the second IMU clipped "
		  << instance_with_second_imu_clipped << ", ground truth samples " << truth_samples->load() << std::endl;
	CHECK(instance_with_second_imu_clipped == 0);

	// Clear it and both have to settle back down.
	REQUIRE(_param->set_param_int("SIH_FAULT_IMU", 0) == Param::Result::Success);
	REQUIRE(_param->set_param_float("SIH_FAULT_VIBE", 0.f) == Param::Result::Success);
	sleep_for(kFaultSettleTime);
	peak_imu1_mg->store(0);
	peak_imu2_mg->store(0);
	samples_imu1->store(0);
	samples_imu2->store(0);
	truth_samples->store(0);
	sleep_for(kFaultMeasureTime);

	REQUIRE(samples_imu1->load() > 0);
	REQUIRE(samples_imu2->load() > 0);
	REQUIRE(truth_samples->load() >= min_truth_samples);

	CHECK(peak_imu1_mg->load() < kCleanImuPeakMg);
	CHECK(peak_imu2_mg->load() < kCleanImuPeakMg);

	// Either instance is a legitimate choice once both are clean, so this is recorded rather
	// than asserted.
	std::cout << time_str() << "primary estimator instance after the faults cleared "
		  << primary_estimator_instance() << ", ground truth samples " << truth_samples->load() << std::endl;

	// Judged against the simulator, not the estimator the faults are disturbing. Clipping an
	// accelerometer does push a hovering vehicle around, so what this asserts is that it comes
	// back once the fault clears, not that it was never moved. The excursion while the fault
	// is active is reported for the record rather than bounded.
	_telemetry->unsubscribe_ground_truth(truth_handle);

	const float recovered_deviation_m = fabsf(truth_altitude_m->load() - truth_altitude_start_m->load());
	std::cout << time_str() << "ground truth altitude relative to the hover, lowest "
		  << truth_lowest_m->load() << " m, highest " << truth_highest_m->load()
		  << " m, after recovery " << recovered_deviation_m << " m" << std::endl;

	// Losing height is the outcome that ends on the ground, so that is the one with a hard
	// floor. The caller hovers at 20 m, so this keeps it well clear of the ground rather than
	// only requiring that it came back afterwards.
	CHECK(truth_lowest_m->load() > -kSelectorAltitudeFloorM);
	CHECK(truth_highest_m->load() < kSelectorAltitudeCeilingM);
	CHECK(recovered_deviation_m < kSelectorRecoveryToleranceM);
}

void AutopilotTester::check_tracks_mission_ground_truth(float corridor_radius_m)
{
	auto mission = _mission->download_mission();
	REQUIRE(mission.first == Mission::Result::Success);

	std::vector<Mission::MissionItem> mission_items = mission.second.mission_items;
	REQUIRE(mission_items.size() >= 2);
	auto ct = get_coordinate_transformation();

	// Horizontal only. The vertical behaviour is asserted separately, and mixing an AMSL
	// ground truth altitude with a mission relative altitude would compare two datums.
	_truth_corridor_radius_m = corridor_radius_m;
	_truth_corridor_worst_m.store(0.f);
	_truth_corridor_leg_count = mission_items.size();
	_truth_corridor_leg_samples = std::make_unique<std::atomic<int>[]>(_truth_corridor_leg_count);

	_truth_corridor_handle = _telemetry->subscribe_ground_truth([ct, mission_items,
	    this](Telemetry::GroundTruth ground_truth) {
		// The item being flown to is read here rather than delivered with the sample, so a
		// sample queued just before a waypoint switch can arrive with the next item current.
		// Measuring against the leg being flown and the one before it, as finite segments,
		// keeps such a sample on the leg it belongs to rather than on the extension of the
		// next one.
		auto progress = _mission->mission_progress();

		if ((progress.current <= 0) || (progress.current >= (int)mission_items.size())) {
			return;
		}

		if (!std::isfinite(ground_truth.latitude_deg) || !std::isfinite(ground_truth.longitude_deg)) {
			return;
		}

		using GlobalCoordinate = CoordinateTransformation::GlobalCoordinate;
		const auto local = ct.local_from_global(GlobalCoordinate{ground_truth.latitude_deg, ground_truth.longitude_deg});

		const std::array<float, 3> current { (float)local.north_m, (float)local.east_m, 0.f };

		auto leg_distance = [&](int item) {
			std::array<float, 3> start = get_local_mission_item<float>(mission_items[item - 1], ct);
			std::array<float, 3> end = get_local_mission_item<float>(mission_items[item], ct);
			start[2] = 0.f;
			end[2] = 0.f;
			return point_to_segment_distance(current, start, end);
		};

		float distance_m = leg_distance(progress.current);

		if (progress.current >= 2) {
			distance_m = std::min(distance_m, leg_distance(progress.current - 1));
		}

		float worst = _truth_corridor_worst_m.load();

		while ((distance_m > worst) && !_truth_corridor_worst_m.compare_exchange_weak(worst, distance_m)) {
		}

		_truth_corridor_leg_samples[progress.current].fetch_add(1);
	});
}

void AutopilotTester::stop_tracking_mission_ground_truth()
{
	_telemetry->unsubscribe_ground_truth(_truth_corridor_handle);

	std::cout << time_str() << "worst ground truth distance from the mission legs "
		  << _truth_corridor_worst_m.load() << " m, samples per leg";

	for (size_t item = 1; item < _truth_corridor_leg_count; item++) {
		std::cout << " " << _truth_corridor_leg_samples[item].load();
	}

	std::cout << std::endl;

	// A maximum of zero because a leg was never measured would read as a pass, so every leg
	// has to have been seen.
	for (size_t item = 1; item < _truth_corridor_leg_count; item++) {
		REQUIRE(_truth_corridor_leg_samples[item].load() > 0);
	}

	CHECK(_truth_corridor_worst_m.load() < _truth_corridor_radius_m);
}

void AutopilotTester::execute_mission_and_degrade_primary_imu()
{
	// SCALED_IMU carries vehicle_imu instance 0, the one this test clips. Watching its peak
	// vertical acceleration is what says the fault was really injected. Without it the test
	// would pass on a build where the injection silently does nothing.
	auto peak_imu1_mg = std::make_shared<std::atomic<int>>(0);
	auto imu_samples = std::make_shared<std::atomic<int>>(0);
	request_message_interval(MAVLINK_MSG_ID_SCALED_IMU, 50.f);
	add_mavlink_message_callback(MAVLINK_MSG_ID_SCALED_IMU, [peak_imu1_mg,
	imu_samples](const mavlink_message_t &message) {
		mavlink_scaled_imu_t imu;
		mavlink_msg_scaled_imu_decode(&message, &imu);
		const int magnitude = std::abs((int)imu.zacc);
		int previous = peak_imu1_mg->load();

		while ((magnitude > previous) && !peak_imu1_mg->compare_exchange_weak(previous, magnitude)) {
		}

		imu_samples->fetch_add(1);
	});

	// Inject on a progress event rather than on a poll. Polling samples on a wall clock grid
	// while the mission advances on the simulator clock, so under a speed factor the mission
	// can pass the whole window between two samples. Waiting on the event instead means the
	// injection starts from a point where the mission was still running.
	// The callback owns its state through shared pointers rather than capturing this
	// scope by reference, so if a REQUIRE below aborts the test the subscription that
	// outlives it still has nothing dangling to touch.
	auto injection_window = std::make_shared<std::promise<void>>();
	auto window_signalled = std::make_shared<std::atomic<bool>>(false);
	auto injection_window_reached = injection_window->get_future();

	Mission::MissionProgressHandle progress_handle = _mission->subscribe_mission_progress(
	[injection_window, window_signalled](Mission::MissionProgress progress) {
		if ((progress.current >= 1) && (progress.current < progress.total) && !window_signalled->exchange(true)) {
			injection_window->set_value();
		}
	});

	REQUIRE(_mission->start_mission() == Mission::Result::Success);
	REQUIRE(injection_window_reached.wait_for(std::chrono::seconds(120)) == std::future_status::ready);
	_mission->unsubscribe_mission_progress(progress_handle);

	// The vehicle is judged against the simulator, not against the estimator these faults
	// are deliberately disturbing. An estimator based band cannot be used here, because a
	// handover steps the estimate on purpose and the same band would have to both allow
	// that step and catch a real excursion.
	auto truth_reference_m = std::make_shared<std::atomic<float>>(NAN);
	auto truth_max_deviation_m = std::make_shared<std::atomic<float>>(0.f);
	auto truth_samples = std::make_shared<std::atomic<int>>(0);
	auto truth_handle = _telemetry->subscribe_ground_truth([truth_reference_m, truth_max_deviation_m,
			   truth_samples](Telemetry::GroundTruth ground_truth) {
		const float reference = truth_reference_m->load();

		if (!std::isfinite(ground_truth.absolute_altitude_m) || !std::isfinite(reference)) {
			return;
		}

		const float deviation = fabsf(ground_truth.absolute_altitude_m - reference);
		float worst = truth_max_deviation_m->load();

		while ((deviation > worst) && !truth_max_deviation_m->compare_exchange_weak(worst, deviation)) {
		}

		truth_samples->fetch_add(1);
	});

	REQUIRE(poll_condition_with_timeout(
	[peak_imu1_mg]() { return peak_imu1_mg->load() > 0; }, std::chrono::seconds(20)));

	const float truth_altitude_at_injection_m = _telemetry->ground_truth().absolute_altitude_m;
	REQUIRE(std::isfinite(truth_altitude_at_injection_m));
	truth_reference_m->store(truth_altitude_at_injection_m);

	// Where the selector starts, so the handover below is a change rather than a coincidence.
	const int initial_instance = primary_estimator_instance();
	std::cout << time_str() << "primary estimator instance before the fault " << initial_instance << std::endl;
	CHECK(initial_instance == 0);

	ImuFaultGuard fault_guard(_param.get());

	// Sustained clipping degrades the estimator instance behind this IMU without silencing
	// the sensor. A silenced sensor is a different failure mode and is already covered by
	// failure injection.
	REQUIRE(_param->set_param_float("SIH_FAULT_VIBE", kImuClippingFaultAmplitude) == Param::Result::Success);
	REQUIRE(_param->set_param_int("SIH_FAULT_IMU", 1) == Param::Result::Success);

	// Measure only what arrives after the fault is in.
	peak_imu1_mg->store(0);
	imu_samples->store(0);
	truth_samples->store(0);
	const double fault_start_s = autopilot_time_s();

	auto mission_still_running = [this]() {
		auto result = _mission->is_mission_finished();
		return result.first == Mission::Result::Success && !result.second;
	};

	// The clipping has to be visible while the mission is still running. Without this the
	// test would pass when the injection silently did nothing, and a progress event that
	// arrived late could put the whole fault after the mission had already ended.
	REQUIRE(poll_condition_with_timeout(
	[peak_imu1_mg]() { return peak_imu1_mg->load() > kClippedImuPeakMg; }, std::chrono::seconds(20)));
	REQUIRE(mission_still_running());

	// The selector has to leave the instance behind the clipped IMU, and do so before the
	// mission ends. That is the handover the rest of the mission then rides through.
	REQUIRE(poll_condition_with_timeout(
	[this]() { return primary_estimator_instance() == 1; }, kHandoverTimeout));
	REQUIRE(mission_still_running());
	std::cout << time_str() << "primary estimator instance moved to 1 with the mission still running" << std::endl;

	// The mission has to run to the end through the handover.
	REQUIRE(poll_condition_with_timeout(
	[this]() {
		auto result = _mission->is_mission_finished();
		return result.first == Mission::Result::Success && result.second;
	}, std::chrono::seconds(180)));

	const double fault_window_s = autopilot_time_s() - fault_start_s;

	// Clear the fault before the caller flies home. Landing on a clipping accelerometer is a
	// different test, and leaving it on would quietly make the disarm timeout load bearing.
	REQUIRE(_param->set_param_int("SIH_FAULT_IMU", 0) == Param::Result::Success);
	REQUIRE(_param->set_param_float("SIH_FAULT_VIBE", 0.f) == Param::Result::Success);

	// The vehicle itself has to hold its altitude through the fault, measured against the
	// simulator rather than against the estimate. A maximum of zero because the stream
	// stalled would otherwise read as a pass, so the window has to have been sampled at a
	// sane rate first.
	_telemetry->unsubscribe_ground_truth(truth_handle);
	std::cout << time_str() << "ground truth samples through the fault " << truth_samples->load()
		  << " over " << fault_window_s << " s, worst altitude deviation " << truth_max_deviation_m->load()
		  << " m" << std::endl;
	REQUIRE(truth_samples->load() >= fault_window_s * kMinGroundTruthRateHz);
	CHECK(truth_max_deviation_m->load() < kFailoverGroundTruthBandM);
}

void AutopilotTester::execute_mission_and_get_baro_stuck()
{
	CHECK(_param->set_param_int("SYS_FAILURE_EN", 1) == Param::Result::Success);

	start_and_wait_for_mission_sequence(1);

	CHECK(_failure->inject(Failure::FailureUnit::SensorBaro, Failure::FailureType::Stuck, 0) == Failure::Result::Success);

	// We except the mission to continue with a stuck baro just fine.
	REQUIRE(poll_condition_with_timeout(
	[this]() {
		auto result = _mission->is_mission_finished();
		return result.first == Mission::Result::Success && result.second;
	}, std::chrono::seconds(90)));
}

void AutopilotTester::execute_mission_and_get_mag_stuck()
{
	CHECK(_param->set_param_int("SYS_FAILURE_EN", 1) == Param::Result::Success);

	start_and_wait_for_mission_sequence(1);

	CHECK(_failure->inject(Failure::FailureUnit::SensorMag, Failure::FailureType::Stuck, 0) == Failure::Result::Success);

	// We except the mission to continue with a stuck mag just fine.
	REQUIRE(poll_condition_with_timeout(
	[this]() {
		auto result = _mission->is_mission_finished();
		return result.first == Mission::Result::Success && result.second;
	}, std::chrono::seconds(120)));
}

CoordinateTransformation AutopilotTester::get_coordinate_transformation()
{
	const auto home = _telemetry->home();
	CHECK(std::isfinite(home.latitude_deg));
	CHECK(std::isfinite(home.longitude_deg));
	return CoordinateTransformation({home.latitude_deg, home.longitude_deg});
}

Mission::MissionItem  AutopilotTester::create_mission_item(
	const CoordinateTransformation::LocalCoordinate &local_coordinate,
	const MissionOptions &mission_options,
	const CoordinateTransformation &ct)
{
	auto mission_item = Mission::MissionItem{};
	const auto pos_north = ct.global_from_local(local_coordinate);
	mission_item.latitude_deg = pos_north.latitude_deg;
	mission_item.longitude_deg = pos_north.longitude_deg;
	mission_item.relative_altitude_m = mission_options.relative_altitude_m;
	mission_item.is_fly_through = mission_options.fly_through;
	return mission_item;
}

void AutopilotTester::load_qgc_mission_raw_and_move_here(const std::string &plan_file)
{
	auto import_result = _mission_raw->import_qgroundcontrol_mission(plan_file);
	REQUIRE(import_result.first == MissionRaw::Result::Success);

	move_mission_raw_here(import_result.second.mission_items);

	REQUIRE(_mission_raw->upload_mission(import_result.second.mission_items) == MissionRaw::Result::Success);
	// PX4 needs time to realize that it now has a mission available, so we need to wait a bit here.
	sleep_for(std::chrono::seconds(1));
}

void AutopilotTester::execute_mission_raw()
{
	REQUIRE(_mission->start_mission() == Mission::Result::Success);

	wait_for_mission_raw_finished(std::chrono::seconds(300));
}

namespace
{
MissionRaw::MissionItem make_raw_mission_item(uint32_t seq, uint32_t command, uint32_t frame,
		float param1, float param2, int32_t x, int32_t y, float z, uint32_t current)
{
	MissionRaw::MissionItem item{};
	item.seq = seq;
	item.frame = frame;
	item.command = command;
	item.current = current;
	item.autocontinue = 1;
	item.param1 = param1;
	item.param2 = param2;
	item.param3 = 0.f;
	item.param4 = 0.f;
	item.x = x;
	item.y = y;
	item.z = z;
	item.mission_type = MAV_MISSION_TYPE_MISSION;
	return item;
}
} // namespace

int AutopilotTester::prepare_multicopter_mission_with_do_jump(const MissionOptions &mission_options, int jump_repeats)
{
	const auto ct = get_coordinate_transformation();
	const float altitude_m = static_cast<float>(mission_options.relative_altitude_m);
	const double leg_m = mission_options.leg_length_m;

	// Convert a local (north, east) offset from home into a GLOBAL_RELATIVE_ALT mission item.
	auto position_item = [&](uint32_t seq, uint32_t command, double north_m, double east_m, float z,
	uint32_t current) {
		const auto global = ct.global_from_local({north_m, east_m});
		return make_raw_mission_item(seq, command, MAV_FRAME_GLOBAL_RELATIVE_ALT, 0.f, 0.f,
					     static_cast<int32_t>(std::lround(global.latitude_deg * 1e7)),
					     static_cast<int32_t>(std::lround(global.longitude_deg * 1e7)), z, current);
	};

	// A simple square-ish path with a DO_JUMP that loops back to waypoint 1. The DO_JUMP is at seq 3.
	const int jump_index = 3;
	std::vector<MissionRaw::MissionItem> items;
	items.push_back(position_item(0, MAV_CMD_NAV_TAKEOFF, 0., 0., altitude_m, /*current*/ 1));
	items.push_back(position_item(1, MAV_CMD_NAV_WAYPOINT, leg_m, 0., altitude_m, 0));
	items.push_back(position_item(2, MAV_CMD_NAV_WAYPOINT, leg_m, leg_m, altitude_m, 0));
	items.push_back(make_raw_mission_item(jump_index, MAV_CMD_DO_JUMP, MAV_FRAME_MISSION,
					      /*param1: target seq*/ 1.f, /*param2: repeats*/ static_cast<float>(jump_repeats),
					      0, 0, 0.f, 0));
	items.push_back(position_item(4, MAV_CMD_NAV_WAYPOINT, 0., leg_m, altitude_m, 0));
	items.push_back(position_item(5, MAV_CMD_NAV_LAND, 0., 0., 0.f, 0));

	REQUIRE(_mission_raw->upload_mission(items) == MissionRaw::Result::Success);
	// PX4 needs time to realize that it now has a mission available, so we need to wait a bit here.
	sleep_for(std::chrono::seconds(1));

	return jump_index;
}

void AutopilotTester::start_mission_raw_and_wait_for_sequence(int sequence_number)
{
	start_and_wait_for_mission_sequence_raw(sequence_number);
}

void AutopilotTester::send_set_current_mission_item(int index)
{
	// Send a single raw MISSION_SET_CURRENT, fire-and-forget - this mimics a one-shot operator/GCS
	// action. We deliberately do NOT use MissionRaw::set_current_mission_item(): that call keeps
	// retransmitting MISSION_SET_CURRENT until the autopilot echoes back MISSION_CURRENT with the
	// exact requested seq. When the requested item is a DO_JUMP the navigator resolves it to the
	// jump target and reports a different seq, so MAVSDK would re-send indefinitely and continuously
	// reset the mission, which is not the scenario we want to exercise here.
	const uint8_t target_sysid = _mavlink_passthrough->get_target_sysid();
	const uint16_t seq = static_cast<uint16_t>(index);

	const MavlinkPassthrough::Result result = _mavlink_passthrough->queue_message(
	[target_sysid, seq](MavlinkAddress mavlink_address, uint8_t channel) {
		mavlink_message_t message;
		mavlink_msg_mission_set_current_pack_chan(
			mavlink_address.system_id,
			mavlink_address.component_id,
			channel,
			&message,
			target_sysid,
			MAV_COMP_ID_AUTOPILOT1,
			seq);
		return message;
	});

	// Extra parentheses stop Catch2 from decomposing the expression: stringifying
	// MavlinkPassthrough::Result pulls in an operator<< that the MAVSDK lib does not export.
	REQUIRE((result == MavlinkPassthrough::Result::Success));
}

void AutopilotTester::execute_rtl()
{
	REQUIRE(Action::Result::Success == _action->return_to_launch());
}

void AutopilotTester::execute_land()
{
	REQUIRE(Action::Result::Success == _action->land());
}

void AutopilotTester::start_offboard_with_retry(const std::function<void()> &resend_setpoint)
{
	// MAVSDK's offboard watchdog resets the setpoint state to NotActive when a
	// heartbeat without offboard mode arrives more than 3 s after _last_started
	// — which is only ever set by start(), so before the first start() it is
	// epoch zero and the guard is always true. At high sim speed factors
	// heartbeats arrive every few milliseconds of wall time, so one can slip in
	// between set_*() and start(), making start() fail with NoSetpointSet.
	// Re-send the setpoint and retry until the command goes out.
	Offboard::Result result{};

	for (int i = 0; i < 10; ++i) {
		result = _offboard->start();

		if (result == Offboard::Result::Success) {
			return;
		}

		resend_setpoint();
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}

	REQUIRE(result == Offboard::Result::Success);
}

void AutopilotTester::offboard_goto(const Offboard::PositionNedYaw &target, float acceptance_radius_m,
				    std::chrono::seconds timeout_duration)
{
	_offboard->set_position_ned(target);
	start_offboard_with_retry([this, target]() { _offboard->set_position_ned(target); });
	CHECK(poll_condition_with_timeout(
	[ = ]() { return estimated_position_close_to(target, acceptance_radius_m); }, timeout_duration));
	std::cout << time_str() << "Target position reached" << std::endl;
}

void AutopilotTester::check_mission_item_speed_above(int item_index, float min_speed_m_s)
{

	_telemetry->set_rate_velocity_ned(10);
	_telemetry->subscribe_velocity_ned([item_index, min_speed_m_s, this](Telemetry::VelocityNed velocity) {
		float horizontal = std::hypot(velocity.north_m_s, velocity.east_m_s);
		auto progress = _mission->mission_progress();

		if (progress.current == item_index) {
			CHECK(horizontal > min_speed_m_s);
		}
	});
}

void AutopilotTester::fly_forward_in_posctl()
{
	const unsigned manual_control_rate_hz = 50;

	// Send something to make sure RC is available.
	for (unsigned i = 0; i < 1 * manual_control_rate_hz; ++i) {
		CHECK(_manual_control->set_manual_control_input(0.f, 0.f, 0.5f, 0.f) == ManualControl::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / manual_control_rate_hz));
	}

	CHECK(_manual_control->start_position_control() == ManualControl::Result::Success);

	// Send something to make sure RC is available.
	for (unsigned i = 0; i < 1 * manual_control_rate_hz; ++i) {
		CHECK(_manual_control->set_manual_control_input(0.f, 0.f, 0.5f, 0.f) == ManualControl::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / manual_control_rate_hz));
	}

	store_home();

	// Send something to make sure RC is available.
	for (unsigned i = 0; i < 1 * manual_control_rate_hz; ++i) {
		CHECK(_manual_control->set_manual_control_input(0.f, 0.f, 0.5f, 0.f) == ManualControl::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / manual_control_rate_hz));
	}

	wait_until_ready();

	// Send something to make sure RC is available.
	for (unsigned i = 0; i < 1 * manual_control_rate_hz; ++i) {
		CHECK(_manual_control->set_manual_control_input(0.f, 0.f, 0.5f, 0.f) == ManualControl::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / manual_control_rate_hz));
	}

	arm();

	// Climb up for 5 seconds
	for (unsigned i = 0; i < 5 * manual_control_rate_hz; ++i) {
		CHECK(_manual_control->set_manual_control_input(0.f, 0.f, 1.f, 0.f) == ManualControl::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / manual_control_rate_hz));
	}

	// Fly forward for 10 seconds
	for (unsigned i = 0; i < 10 * manual_control_rate_hz; ++i) {
		CHECK(_manual_control->set_manual_control_input(0.5f, 0.f, 0.5f, 0.f) == ManualControl::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / manual_control_rate_hz));
	}

	// Descend until disarmed
	for (unsigned i = 0; i < 60 * manual_control_rate_hz; ++i) {
		CHECK(_manual_control->set_manual_control_input(0.f, 0.f, 0.0f, 0.f) == ManualControl::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / manual_control_rate_hz));

		if (!_telemetry->in_air()) {
			break;
		}
	}
}

void AutopilotTester::fly_forward_in_altctl()
{
	const unsigned manual_control_rate_hz = 50;

	// Send something to make sure RC is available.
	for (unsigned i = 0; i < 1 * manual_control_rate_hz; ++i) {
		CHECK(_manual_control->set_manual_control_input(0.f, 0.f, 0.5f, 0.f) == ManualControl::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / manual_control_rate_hz));
	}

	CHECK(_manual_control->start_altitude_control() == ManualControl::Result::Success);

	// Send something to make sure RC is available.
	for (unsigned i = 0; i < 1 * manual_control_rate_hz; ++i) {
		CHECK(_manual_control->set_manual_control_input(0.f, 0.f, 0.5f, 0.f) == ManualControl::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / manual_control_rate_hz));
	}

	store_home();

	// Send something to make sure RC is available.
	for (unsigned i = 0; i < 1 * manual_control_rate_hz; ++i) {
		CHECK(_manual_control->set_manual_control_input(0.f, 0.f, 0.5f, 0.f) == ManualControl::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / manual_control_rate_hz));
	}

	wait_until_ready();

	// Send something to make sure RC is available.
	for (unsigned i = 0; i < 1 * manual_control_rate_hz; ++i) {
		CHECK(_manual_control->set_manual_control_input(0.f, 0.f, 0.5f, 0.f) == ManualControl::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / manual_control_rate_hz));
	}

	arm();

	// Send something to make sure RC is available.
	for (unsigned i = 0; i < 1 * manual_control_rate_hz; ++i) {
		CHECK(_manual_control->set_manual_control_input(0.f, 0.f, 0.5f, 0.f) == ManualControl::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / manual_control_rate_hz));
	}

	// Climb up for 5 seconds
	for (unsigned i = 0; i < 5 * manual_control_rate_hz; ++i) {
		CHECK(_manual_control->set_manual_control_input(0.f, 0.f, 1.f, 0.f) == ManualControl::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / manual_control_rate_hz));
	}

	// Fly forward for 10 seconds
	for (unsigned i = 0; i < 10 * manual_control_rate_hz; ++i) {
		CHECK(_manual_control->set_manual_control_input(0.5f, 0.f, 0.5f, 0.f) == ManualControl::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / manual_control_rate_hz));
	}

	// Descend until disarmed
	for (unsigned i = 0; i < 60 * manual_control_rate_hz; ++i) {
		CHECK(_manual_control->set_manual_control_input(0.f, 0.f, 0.0f, 0.f) == ManualControl::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / manual_control_rate_hz));

		if (!_telemetry->in_air()) {
			break;
		}
	}
}
void AutopilotTester::fly_forward_in_offboard_attitude()
{
	// This test does not depend on valid position estimate.
	// Wait for raw gps & stable attitude estimate
	CHECK(poll_condition_with_timeout(
	[this]() {
		auto attitude = _telemetry->attitude_euler();
		return _telemetry->raw_gps().altitude_ellipsoid_m > 0.f && fabsf(attitude.roll_deg) < 5.f
		       && fabsf(attitude.pitch_deg) < 5.f;
	}, std::chrono::seconds(20)));

	const float start_altitude_ellipsoid_m = _telemetry->raw_gps().altitude_ellipsoid_m;

	Offboard::Attitude attitude{};
	_offboard->set_attitude(attitude);
	start_offboard_with_retry([this, attitude]() { _offboard->set_attitude(attitude); });

	// Wait until we can arm
	CHECK(poll_condition_with_timeout(
	[this]() {	return _telemetry->health().is_armable;	}, std::chrono::seconds(20)));
	arm();

	const unsigned offboard_rate_hz = 50;

	// Climb
	const float climb_altitude_m = 10.f;
	attitude.thrust_value = 0.8f;

	while (_telemetry->raw_gps().altitude_ellipsoid_m - start_altitude_ellipsoid_m < climb_altitude_m) {
		CHECK(_offboard->set_attitude(attitude) == Offboard::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / offboard_rate_hz));
	}

	// Fly forward for 3s
	attitude.thrust_value = 0.8f;
	attitude.pitch_deg = -20.f;

	for (unsigned i = 0; i < 3 * offboard_rate_hz; ++i) {
		CHECK(_offboard->set_attitude(attitude) == Offboard::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / offboard_rate_hz));
	}

	// Check attitude
	auto attitude_estimate = _telemetry->attitude_euler();
	CHECK(fabsf(attitude.roll_deg - attitude_estimate.roll_deg) < 5.f);
	CHECK(fabsf(attitude.pitch_deg - attitude_estimate.pitch_deg) < 5.f);

	// Descend
	attitude.thrust_value = 0.4f;
	attitude.pitch_deg = 0.f;

	for (unsigned i = 0; i < 6 * offboard_rate_hz; ++i) {
		CHECK(_offboard->set_attitude(attitude) == Offboard::Result::Success);
		sleep_for(std::chrono::milliseconds(1000 / offboard_rate_hz));
	}

	attitude.thrust_value = 0.0f;
	CHECK(_offboard->set_attitude(attitude) == Offboard::Result::Success);
}

void AutopilotTester::start_checking_altitude(const float max_deviation_m)
{
	std::array<float, 3> initial_position = get_current_position_ned();
	float target_altitude = initial_position[2];

	_check_altitude_handle = _telemetry->subscribe_position_velocity_ned([target_altitude, max_deviation_m,
			 this](Telemetry::PositionVelocityNed new_position) {
		const float current_deviation = fabs(target_altitude - new_position.position.down_m);
		CHECK(current_deviation <= max_deviation_m);
	});
}

void AutopilotTester::stop_checking_altitude()
{
	_telemetry->unsubscribe_position_velocity_ned(_check_altitude_handle);
}

void AutopilotTester::check_tracks_mission_raw(float corridor_radius_m, bool reverse)
{
	auto mission_raw = _mission_raw->download_mission();
	CHECK(mission_raw.first == MissionRaw::Result::Success);

	auto mission_items = mission_raw.second;
	auto ct = get_coordinate_transformation();

	_telemetry->set_rate_position_velocity_ned(5);
	_telemetry->subscribe_position_velocity_ned([ct, mission_items, corridor_radius_m, reverse,
	    this](Telemetry::PositionVelocityNed position_velocity_ned) {
		auto progress = _mission_raw->mission_progress();


		std::function<std::array<float, 3>(std::vector<mavsdk::MissionRaw::MissionItem>, unsigned, mavsdk::geometry::CoordinateTransformation)>
		get_waypoint_for_sequence = [](std::vector<mavsdk::MissionRaw::MissionItem> mission_items, int sequence, auto ct) {
			for (auto waypoint : mission_items) {

				if (waypoint.seq == (uint32_t)sequence) {
					return get_local_mission_item_from_raw_item<float>(waypoint, ct);
				}
			}

			return  std::array<float, 3>({0.0f, 0.0f, 0.0f});
		};

		if (progress.current > 0 && progress.current < progress.total) {
			// Get shortest distance of current position to 3D line between previous and next waypoint

			std::array<float, 3> current { position_velocity_ned.position.north_m,
						       position_velocity_ned.position.east_m,
						       position_velocity_ned.position.down_m };
			std::array<float, 3> wp_prev = get_waypoint_for_sequence(mission_items,
						       reverse ? progress.current + 1 : progress.current - 1, ct);
			std::array<float, 3> wp_next = get_waypoint_for_sequence(mission_items, progress.current, ct);

			float distance_to_trajectory = point_to_line_distance(current, wp_prev, wp_next);

			CHECK(distance_to_trajectory < corridor_radius_m);
		}
	});
}

void AutopilotTester::check_mission_land_within(float acceptance_radius_m)
{
	auto mission_raw = _mission_raw->download_mission();
	CHECK(mission_raw.first == MissionRaw::Result::Success);

	// Get last mission item
	MissionRaw::MissionItem land_mission_item = mission_raw.second.back();
	bool is_landing_item = (land_mission_item.command == 85) || (land_mission_item.command == 21);
	CHECK(is_landing_item);
	Telemetry::GroundTruth land_coord{};
	land_coord.latitude_deg = static_cast<double>(land_mission_item.x) / 1E7;
	land_coord.longitude_deg = static_cast<double>(land_mission_item.y) / 1E7;

	CHECK(ground_truth_horizontal_position_close_to(land_coord, acceptance_radius_m));
}

void AutopilotTester::check_tracks_mission(float corridor_radius_m)
{
	auto mission = _mission->download_mission();
	CHECK(mission.first == Mission::Result::Success);

	std::vector<Mission::MissionItem> mission_items = mission.second.mission_items;
	auto ct = get_coordinate_transformation();

	_telemetry->set_rate_position_velocity_ned(5);
	_telemetry->subscribe_position_velocity_ned([ct, mission_items, corridor_radius_m,
	    this](Telemetry::PositionVelocityNed position_velocity_ned) {
		auto progress = _mission->mission_progress();

		if (progress.current > 0 && progress.current < progress.total) {
			// Get shortest distance of current position to 3D line between previous and next waypoint

			std::array<float, 3> current { position_velocity_ned.position.north_m,
						       position_velocity_ned.position.east_m,
						       position_velocity_ned.position.down_m };
			std::array<float, 3> wp_prev = get_local_mission_item<float>(mission_items[progress.current - 1], ct);
			std::array<float, 3> wp_next = get_local_mission_item<float>(mission_items[progress.current], ct);

			float distance_to_trajectory = point_to_line_distance(current, wp_prev, wp_next);

			CHECK(distance_to_trajectory < corridor_radius_m);
		}
	});
}

void AutopilotTester::check_current_altitude(float target_rel_altitude_m, float max_distance_m)
{
	CHECK(std::abs(_telemetry->position().relative_altitude_m - target_rel_altitude_m) <= max_distance_m);
}

void AutopilotTester::execute_rtl_when_reaching_mission_sequence(int sequence_number)
{
	start_and_wait_for_mission_sequence_raw(sequence_number);
	execute_rtl();
}

void AutopilotTester::send_custom_mavlink_command(const MavlinkPassthrough::CommandInt &command)
{
	_mavlink_passthrough->send_command_int(command);
}

void AutopilotTester::request_message_interval(uint16_t message_id, float rate_hz)
{
	// The onboard mode this test connects to does not stream the IMU messages by default,
	// so ask for them. PX4 resolves the message id to a stream and enables it.
	MavlinkPassthrough::CommandInt command{};
	command.target_sysid = _mavlink_passthrough->get_target_sysid();
	command.target_compid = _mavlink_passthrough->get_target_compid();
	command.command = MAV_CMD_SET_MESSAGE_INTERVAL;
	command.frame = MAV_FRAME_GLOBAL;
	command.param1 = (float)message_id;
	command.param2 = 1e6f / rate_hz;
	REQUIRE((_mavlink_passthrough->send_command_int(command) == MavlinkPassthrough::Result::Success));
}

void AutopilotTester::add_mavlink_message_callback(uint16_t message_id,
		std::function< void(const mavlink_message_t &)> callback)
{
	_mavlink_passthrough->subscribe_message(message_id, std::move(callback));
}

mavlink_home_position_t AutopilotTester::get_home_position(std::chrono::seconds timeout)
{
	auto home_position = std::make_shared<mavlink_home_position_t>();
	auto received = std::make_shared<std::atomic<bool>>(false);

	auto handle = _mavlink_passthrough->subscribe_message(
			      MAVLINK_MSG_ID_HOME_POSITION,
	[home_position, received](const mavlink_message_t &message) {
		mavlink_msg_home_position_decode(&message, home_position.get());
		received->store(true);
	});

	// Ask for a fresh HOME_POSITION so we don't rely on the periodic stream timing.
	MavlinkPassthrough::CommandLong request{};
	request.target_sysid = _mavlink_passthrough->get_target_sysid();
	request.target_compid = _mavlink_passthrough->get_target_compid();
	request.command = MAV_CMD_REQUEST_MESSAGE;
	request.param1 = static_cast<float>(MAVLINK_MSG_ID_HOME_POSITION);
	_mavlink_passthrough->send_command_long(request);

	const bool got_it = poll_condition_with_timeout([received]() { return received->load(); }, timeout);
	_mavlink_passthrough->unsubscribe_message(MAVLINK_MSG_ID_HOME_POSITION, handle);
	REQUIRE(got_it);
	return *home_position;
}

Telemetry::EulerAngle AutopilotTester::get_attitude_euler()
{
	return _telemetry->attitude_euler();
}

std::array<float, 3> AutopilotTester::get_current_position_ned()
{
	mavsdk::Telemetry::PositionVelocityNed position_velocity_ned = _telemetry->position_velocity_ned();
	std::array<float, 3> position_ned{position_velocity_ned.position.north_m, position_velocity_ned.position.east_m, position_velocity_ned.position.down_m};
	return position_ned;
}

void AutopilotTester::offboard_land()
{
	Offboard::VelocityNedYaw land_velocity;
	land_velocity.north_m_s = 0.0f;
	land_velocity.east_m_s = 0.0f;
	land_velocity.down_m_s = 1.0f;
	land_velocity.yaw_deg = 0.0f;
	_offboard->set_velocity_ned(land_velocity);
}

bool AutopilotTester::estimated_position_close_to(const Offboard::PositionNedYaw &target_pos, float acceptance_radius_m)
{
	Telemetry::PositionNed est_pos = _telemetry->position_velocity_ned().position;
	const float distance_m = std::sqrt(sq(est_pos.north_m - target_pos.north_m) +
					   sq(est_pos.east_m - target_pos.east_m) +
					   sq(est_pos.down_m - target_pos.down_m));
	const bool pass = distance_m < acceptance_radius_m;

	if (!pass) {
		std::cout << time_str() << "distance: " << distance_m << ", " << "acceptance: " << acceptance_radius_m << std::endl;
	}

	return  pass;
}

bool AutopilotTester::estimated_horizontal_position_close_to(const Offboard::PositionNedYaw &target_pos,
		float acceptance_radius_m)
{
	Telemetry::PositionNed est_pos = _telemetry->position_velocity_ned().position;
	return sq(est_pos.north_m - target_pos.north_m) +
	       sq(est_pos.east_m - target_pos.east_m) < sq(acceptance_radius_m);
}

void AutopilotTester::request_ground_truth()
{
	CHECK(_telemetry->set_rate_ground_truth(15) == Telemetry::Result::Success);
}

bool AutopilotTester::ground_truth_horizontal_position_close_to(const Telemetry::GroundTruth &target_pos,
		float acceptance_radius_m)
{
	CHECK(std::isfinite(target_pos.latitude_deg));
	CHECK(std::isfinite(target_pos.longitude_deg));
	using GlobalCoordinate = CoordinateTransformation::GlobalCoordinate;
	using LocalCoordinate = CoordinateTransformation::LocalCoordinate;
	CoordinateTransformation ct(GlobalCoordinate{target_pos.latitude_deg, target_pos.longitude_deg});

	Telemetry::GroundTruth current_pos = _telemetry->ground_truth();
	CHECK(std::isfinite(current_pos.latitude_deg));
	CHECK(std::isfinite(current_pos.longitude_deg));
	GlobalCoordinate global_current;
	global_current.latitude_deg = current_pos.latitude_deg;
	global_current.longitude_deg = current_pos.longitude_deg;
	LocalCoordinate local_pos = ct.local_from_global(global_current);
	const double distance_m = sqrt(sq(local_pos.north_m) + sq(local_pos.east_m));
	const bool pass = distance_m < acceptance_radius_m;

	if (!pass) {
		std::cout << time_str() << "target_pos.lat: " << target_pos.latitude_deg << std::endl;
		std::cout << time_str() << "target_pos.lon: " << target_pos.longitude_deg << std::endl;
		std::cout << time_str() << "current.lat: " << current_pos.latitude_deg << std::endl;
		std::cout << time_str() << "current.lon: " << current_pos.longitude_deg << std::endl;
		std::cout << time_str() << "Distance: " << distance_m << std::endl;
		std::cout << time_str() << "Acceptance radius: " << acceptance_radius_m << std::endl;
	}

	return pass;
}

bool AutopilotTester::ground_truth_horizontal_position_far_from(const Telemetry::GroundTruth &target_pos,
		float min_distance_m)
{
	CHECK(std::isfinite(target_pos.latitude_deg));
	CHECK(std::isfinite(target_pos.longitude_deg));
	using GlobalCoordinate = CoordinateTransformation::GlobalCoordinate;
	using LocalCoordinate = CoordinateTransformation::LocalCoordinate;
	CoordinateTransformation ct(GlobalCoordinate{target_pos.latitude_deg, target_pos.longitude_deg});

	Telemetry::GroundTruth current_pos = _telemetry->ground_truth();
	CHECK(std::isfinite(current_pos.latitude_deg));
	CHECK(std::isfinite(current_pos.longitude_deg));
	GlobalCoordinate global_current;
	global_current.latitude_deg = current_pos.latitude_deg;
	global_current.longitude_deg = current_pos.longitude_deg;
	LocalCoordinate local_pos = ct.local_from_global(global_current);
	const double distance_m = sqrt(sq(local_pos.north_m) + sq(local_pos.east_m));
	const bool pass = distance_m > min_distance_m;

	if (!pass) {
		std::cout << time_str() << "target_pos.lat: " << target_pos.latitude_deg << std::endl;
		std::cout << time_str() << "target_pos.lon: " << target_pos.longitude_deg << std::endl;
		std::cout << time_str() << "current.lat: " << current_pos.latitude_deg << std::endl;
		std::cout << time_str() << "current.lon: " << current_pos.longitude_deg << std::endl;
		std::cout << time_str() << "Distance: " << distance_m << std::endl;
		std::cout << time_str() << "Min distance: " << min_distance_m << std::endl;
	}

	return pass;
}

void AutopilotTester::start_and_wait_for_mission_sequence(int sequence_number)
{
	REQUIRE(_mission->start_mission() == Mission::Result::Success);
	REQUIRE(poll_condition_with_timeout(
	[this, sequence_number]() {
		return _mission->mission_progress().current >= sequence_number;
	}, std::chrono::seconds(60)));
}

void AutopilotTester::start_and_wait_for_mission_sequence_raw(int sequence_number)
{
	auto prom = std::promise<void> {};
	auto fut = prom.get_future();
	// Guards against bunched progress events (e.g. under high sim speed factor) firing the
	// callback twice before unsubscribe takes effect, which would set the promise twice.
	std::atomic<bool> done{false};

	MissionRaw::MissionProgressHandle handle = _mission_raw->subscribe_mission_progress(
	[&prom, &handle, &done, this, sequence_number](MissionRaw::MissionProgress progress) {
		std::cout << time_str() << "Progress: " << progress.current << "/" << progress.total << std::endl;

		if (progress.current >= sequence_number && !done.exchange(true)) {
			_mission_raw->unsubscribe_mission_progress(handle);
			prom.set_value();
		}
	});

	REQUIRE(_mission_raw->start_mission() == MissionRaw::Result::Success);

	REQUIRE(fut.wait_for(std::chrono::seconds(60)) == std::future_status::ready);
}

void AutopilotTester::wait_for_flight_mode(Telemetry::FlightMode flight_mode, std::chrono::seconds timeout)
{
	REQUIRE(poll_condition_with_timeout(
	[this, flight_mode]() {
		return _telemetry->flight_mode() == flight_mode;
	}, timeout));
}

void AutopilotTester::wait_for_landed_state(Telemetry::LandedState landed_state, std::chrono::seconds timeout)
{
	REQUIRE(poll_condition_with_timeout(
	[this, landed_state]() {
		return _telemetry->landed_state() == landed_state;
	}, timeout));
}

void AutopilotTester::wait_until_speed_lower_than(float speed, std::chrono::seconds timeout)
{
	REQUIRE(poll_condition_with_timeout(
	[this, speed]() {
		auto vel = _telemetry->position_velocity_ned().velocity;
		return std::sqrt(vel.north_m_s * vel.north_m_s +
				 vel.east_m_s * vel.east_m_s +
				 vel.down_m_s * vel.down_m_s) < speed;
	}, timeout));
}

void AutopilotTester::wait_for_mission_finished(std::chrono::seconds timeout)
{
	REQUIRE(poll_condition_with_timeout(
	[ = ]() {
		auto result = _mission->is_mission_finished();
		return result.first == Mission::Result::Success && result.second;
	}, timeout));
}

void AutopilotTester::wait_for_mission_raw_finished(std::chrono::seconds timeout)
{
	REQUIRE(poll_condition_with_timeout(
	[ = ]() {
		auto result = _mission_raw->is_mission_finished();
		return result.first == MissionRaw::Result::Success && result.second;
	}, timeout));
}

void AutopilotTester::move_mission_raw_here(std::vector<MissionRaw::MissionItem> &mission_items)
{
	const auto position = _telemetry->position();
	REQUIRE(std::isfinite(position.latitude_deg));
	REQUIRE(std::isfinite(position.longitude_deg));

	auto offset_x = mission_items[0].x - static_cast<int32_t>(1e7 * position.latitude_deg);
	auto offset_y = mission_items[0].y - static_cast<int32_t>(1e7 * position.longitude_deg);

	for (auto &item : mission_items) {
		if (item.frame == 3) { // MAV_FRAME_GLOBAL_RELATIVE_ALT
			item.x -= offset_x;
		}

		item.y -= offset_y;
	}
}

void AutopilotTester::report_speed_factor()
{
	// We check the exit flag more often than the speed factor.
	unsigned counter = 0;

	while (!_should_exit) {
		if (counter++ % 10 == 0) {
			if (_info != nullptr) {
				std::cout << "Current speed factor: " << _info->get_speed_factor().second ;

				if (speed_factor.has_value()) {
					std::cout << " (set: " << speed_factor.value() << ')';
				}

				std::cout << std::endl;
			}
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}

void AutopilotTester::enable_fixedwing_mectrics()
{
	CHECK(getTelemetry()->set_rate_fixedwing_metrics(10.f) == Telemetry::Result::Success);
}

void AutopilotTester::check_airspeed_is_valid()
{
	// If the airspeed was invalidated during the flight, the airspeed is sent in the
	// telemetry is NAN and stays so with the default parameter settings.
	const Telemetry::FixedwingMetrics &metrics = getTelemetry()->fixedwing_metrics();
	REQUIRE(std::isfinite(metrics.airspeed_m_s));
}

void AutopilotTester::check_airspeed_is_invalid()
{
	// If the airspeed was invalidated during the flight, the airspeed is sent in the
	// telemetry is NAN and stays so with the default parameter settings.
	const Telemetry::FixedwingMetrics &metrics = getTelemetry()->fixedwing_metrics();
	std::cout << "Reported airspeed after failure: " << metrics.airspeed_m_s ;
	REQUIRE(!std::isfinite(metrics.airspeed_m_s));
}
