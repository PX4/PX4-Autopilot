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

#include <gtest/gtest.h>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "detect_and_avoid.h"
#include "../navigator.h"
#include "../mission.h"
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>
#include <uORB/topics/mavlink_log.h>

using namespace time_literals;

extern "C" __EXPORT int dataman_main(int argc, char *argv[]);

/* Set simple bounds */
static constexpr float kDfltF34LvlCriticalRad = 150.f;
static constexpr float kDfltF34LvlCriticalHgt = 30.f;
static constexpr float kDfltF34LvlHighRad = 600.f;
static constexpr float kDfltF34LvlHighHgt = 80.f;
static constexpr int kDfltF34LvlMediumTime = 30.f;
static constexpr int kDfltF34LvlLowTime = 30.f;

static constexpr float kDfltDaaDfltVel = 5.f;
static constexpr int kDfltDaaEnDfltVel = 1;
static constexpr int kDfltF3442Standard = detect_and_avoid_s::DAA_STANDARD_F3442;
static constexpr uint8_t kDaaConflictLvlNone = 0;
static constexpr uint8_t kDaaConflictLvlLow = 1;
static constexpr uint8_t kDaaConflictLvlMedium = 2;
static constexpr uint8_t kDaaConflictLvlHigh = 3;
static constexpr uint8_t kDaaConflictLvlCritical = 4;

static constexpr int action_param_value(const DaaAction action)
{
	switch (action) {
	case DaaAction::kDisabled:
		return 0;

	case DaaAction::kWarnOnly:
		return 1;

	case DaaAction::kReturnMode:
		return 2;

	case DaaAction::kLandMode:
		return 3;

	case DaaAction::kPositionHoldMode:
		return 4;

	case DaaAction::kTerminate:
		return 5;

	default:
		return 0;
	}
}

/* Default drone position. Not important as traffic is defined relatively to uav */
static constexpr float kDfltLatUav = 150.f;
static constexpr float kDfltLonUav = 30.f;
static constexpr float kDfltAltUav = 600.f;
static constexpr hrt_abstime kOrbWaitTimeoutUs = 200000;
static constexpr useconds_t kOrbWaitPollIntervalUs = 1000;

// Start dataman once for the lifetime of the test binary. Each Navigator constructs
// multiple DatamanClient instances and each one waits up to 1s for a client ID, so
// restarting dataman between test suites adds ~4 s of timeout per parameterized case.
// Same pattern as in src/modules/navigator/test/test_mission_base.cpp.
class NavigatorDatamanRuntime
{
public:
	NavigatorDatamanRuntime()
	{
		param_control_autosave(false);
		px4::WorkQueueManagerStart();

		char name[] = "dataman";
		char start[] = "start";
		char ram[] = "-r";
		char *argv[] = {name, start, ram};
		dataman_main(3, argv);
	}

	~NavigatorDatamanRuntime()
	{
		param_control_autosave(true);

		char name[] = "dataman";
		char stop[] = "stop";
		char *argv[] = {name, stop};
		dataman_main(2, argv);

		px4::WorkQueueManagerStop();
	}
};

static NavigatorDatamanRuntime &navigatorDatamanRuntime()
{
	static NavigatorDatamanRuntime runtime{};
	return runtime;
}

// To run: make tests TESTFILTER=detect_and_avoid
class DetectAndAvoidTest : public ::testing::Test
{

protected:

	DetectAndAvoidTest() {}
	~DetectAndAvoidTest() override {}

	void SetUp() override
	{
		navigatorDatamanRuntime();
		param_reset_all();
		set_DFLT_daa_params(); // NMAC: (150, 30), WC: (600, 80), Aug-WC: 30s, Aug-NMAC: 30s
		navigator = std::make_unique<Navigator>();
	}

	void TearDown() override
	{
		navigator.reset();
		transponder_report_s tr{};
		_traffic_pub.publish(tr);
	}

public:

	uORB::SubscriptionData<detect_and_avoid_s> _detect_and_avoid_sub{ORB_ID(detect_and_avoid)};
	uORB::SubscriptionData<detect_and_avoid_most_urgent_s> _detect_and_avoid_most_urgent_sub{ORB_ID(detect_and_avoid_most_urgent)};
	uORB::SubscriptionData<vehicle_command_s> _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::SubscriptionData<transponder_report_s> _traffic_sub{ORB_ID(transponder_report)};
	uORB::Subscription _mavlink_log_sub{ORB_ID(mavlink_log)};
	uORB::Subscription _global_pos_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _vehicle_status_state_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};

	uORB::Publication<transponder_report_s> _traffic_pub{ORB_ID(transponder_report)};
	uORB::Publication<vehicle_global_position_s>		_global_pos_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<vehicle_status_s>		_vehicle_status_pub{ORB_ID(vehicle_status)};
	uORB::Publication<vehicle_local_position_s>		_vehicle_local_position_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<vehicle_land_detected_s>		_vehicle_land_detected_pub{ORB_ID(vehicle_land_detected)};
	uORB::Publication<parameter_update_s>		_parameter_update_pub{ORB_ID(parameter_update)};
	std::unique_ptr<Navigator> navigator{};

	struct breach_distances_s {
		float nmac_hor;
		float nmac_vert;
		float wc_hor;
		float wc_vert;
		float aug_nmac_hor;
		float aug_nmac_vert;
		float aug_wc_hor;
		float aug_wc_vert;
	};

	void get_DFLT_breach_distances(const matrix::Vector3f uav_vel, const float traffic_hor_vel,
				       breach_distances_s &breach_distance)
	{

		const float uav_hor_vel = uav_vel.xy().norm();

		/* NMAC */
		breach_distance.nmac_hor = 2 * kDfltF34LvlCriticalRad;
		breach_distance.nmac_vert = 2 * kDfltF34LvlCriticalHgt;

		/* WC */
		breach_distance.wc_hor = 2 * kDfltF34LvlHighRad;
		breach_distance.wc_vert = 2 * kDfltF34LvlHighHgt;

		/* Aug NMAC */
		breach_distance.aug_nmac_hor = 2 * kDfltF34LvlCriticalRad + kDfltF34LvlMediumTime * (fabsf(
						       traffic_hor_vel) + fabsf(uav_hor_vel));
		breach_distance.aug_nmac_vert = 2 * kDfltF34LvlCriticalHgt + kDfltF34LvlMediumTime * (fabsf(
							kDfltDaaDfltVel) + fabsf(uav_vel(2)));

		/* Aug WC */
		breach_distance.aug_wc_hor = 2 * kDfltF34LvlHighRad + kDfltF34LvlLowTime * (fabsf(
						     traffic_hor_vel) + fabsf(uav_hor_vel));
		breach_distance.aug_wc_vert = 2 * kDfltF34LvlHighHgt + kDfltF34LvlLowTime * (fabsf(
						      kDfltDaaDfltVel) + fabsf(uav_vel(2)));
	}

	void publish_local_pos_vel(const matrix::Vector3f local_pos_vel, const hrt_abstime timestamp = hrt_absolute_time(),
				   const float heading = 0.f)
	{
		vehicle_local_position_s l_pos{};
		l_pos.timestamp_sample = timestamp;
		l_pos.timestamp = timestamp;
		l_pos.vx = local_pos_vel(0);
		l_pos.vy = local_pos_vel(1);
		l_pos.vz = local_pos_vel(2);
		l_pos.heading = heading;
		_vehicle_local_position_pub.publish(l_pos);
	}

	void publish_global_pos(const double lat, const double lon, const double alt, const hrt_abstime timestamp = hrt_absolute_time())
	{
		vehicle_global_position_s g_pos{};
		g_pos.timestamp_sample = timestamp;
		g_pos.lat = lat;
		g_pos.lon = lon;
		g_pos.alt = alt;
		g_pos.timestamp = timestamp;
		_global_pos_pub.publish(g_pos);
	}

	void publish_vehicle_status(const uint8_t nav_state, const uint8_t arming_state)
	{

		vehicle_status_s v_status{};
		v_status.timestamp = hrt_absolute_time();
		v_status.nav_state = nav_state;
		v_status.arming_state = arming_state;
		_vehicle_status_pub.publish(v_status);
	}

	void publish_land_status(const bool has_landed)
	{

		vehicle_land_detected_s vehicle_land{};
		vehicle_land.timestamp = hrt_absolute_time();
		vehicle_land.landed = has_landed;
		_vehicle_land_detected_pub.publish(vehicle_land);

	}

	void publish_parameter_update()
	{
		parameter_update_s update{};
		update.timestamp = hrt_absolute_time();
		_parameter_update_pub.publish(update);
	}

	transponder_report_s create_transponder_report(const uint32_t icao_address, const char *callsign, const double lat,
			const double lon, const float altitude, const float hor_velocity, const float ver_velocity,
			const uint16_t flags)
	{
		transponder_report_s tr{};
		tr.timestamp = hrt_absolute_time();
		tr.icao_address = icao_address;
		tr.lat = lat;
		tr.lon = lon;
		tr.altitude = altitude;
		tr.heading = 0.f;
		tr.hor_velocity = hor_velocity;
		tr.ver_velocity = ver_velocity;
		tr.altitude_type = 0;
		tr.emitter_type = 1;
		tr.tslc = 1;
		tr.flags = flags;
		strncpy(&tr.callsign[0], callsign, sizeof(tr.callsign) - 1);
		tr.callsign[sizeof(tr.callsign) - 1] = '\0';
		return tr;
	}

	void publish_transponder_report(const transponder_report_s &tr)
	{
		_traffic_pub.publish(tr);
	}

	template<typename Predicate>
	void wait_until(const Predicate &predicate, const hrt_abstime timeout_us = kOrbWaitTimeoutUs)
	{
		const hrt_abstime start = hrt_absolute_time();
		bool condition_met = predicate();

		while (!condition_met && hrt_elapsed_time(&start) < timeout_us) {
			px4_usleep(kOrbWaitPollIntervalUs);
			condition_met = predicate();
		}

		ASSERT_TRUE(condition_met);
	}

	void wait_for_topic_update(uORB::Subscription &subscription, const hrt_abstime timeout_us = kOrbWaitTimeoutUs)
	{
		wait_until([&subscription]() {
			return subscription.updated();
		}, timeout_us);
	}

	void drain_transponder_report_topic()
	{
		while (_traffic_sub.update()) {}
	}

	template<typename Publisher>
	void publish_traffic_and_check(const Publisher &publisher)
	{
		drain_transponder_report_topic();
		publisher();
		wait_for_topic_update(_traffic_sub);
		navigator->check_traffic();
	}

	void publish_transponder_report_and_check(const transponder_report_s &tr)
	{
		publish_traffic_and_check([&]() {
			publish_transponder_report(tr);
		});
	}

	void set_DFLT_daa_params()
	{
		param_set(param_handle(px4::params::F34_LVL_CRIT_RAD), &kDfltF34LvlCriticalRad);
		param_set(param_handle(px4::params::F34_LVL_CRIT_HGT), &kDfltF34LvlCriticalHgt);
		param_set(param_handle(px4::params::F34_LVL_HIGH_RAD), &kDfltF34LvlHighRad);
		param_set(param_handle(px4::params::F34_LVL_HIGH_HGT), &kDfltF34LvlHighHgt);
		param_set(param_handle(px4::params::F34_LVL_MED_TIME), &kDfltF34LvlMediumTime);
		param_set(param_handle(px4::params::F34_LVL_LOW_TIME), &kDfltF34LvlLowTime);

		param_set(param_handle(px4::params::DAA_EN_DFLT_VEL), &kDfltDaaEnDfltVel);
		param_set(param_handle(px4::params::DAA_DFLT_VEL), &kDfltDaaDfltVel);
		param_set(param_handle(px4::params::DAA_STANDARD), &kDfltF3442Standard);

	}

	void set_default_uav_state(const double lat_uav, const double lon_uav, const float alt_uav,
				   const matrix::Vector3f uav_vel)
	{

		/* Navigation state */
		const bool has_landed = false;
		const uint8_t arming_state = vehicle_status_s::ARMING_STATE_ARMED;
		const uint8_t nav_state = vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION;

		/* Publish UAV state */
		publish_global_pos(lat_uav, lon_uav, alt_uav);
		publish_land_status(has_landed);
		publish_vehicle_status(nav_state, arming_state);
		publish_local_pos_vel(uav_vel);
	}

	void generate_random_callsign(char *callsign, size_t length = 8)
	{
		const char *chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
		const size_t chars_length = 36; // Length of the chars array.

		for (size_t i = 0; i < length; ++i) {
			const size_t index = rand() % chars_length;
			callsign[i] = chars[index];
		}

		callsign[length] = '\0'; // Null-terminate the string.
	}

	void generate_random_UAS_ID(uint8_t uas_id[PX4_GUID_BYTE_LENGTH])
	{
		for (int i = 0; i < PX4_GUID_BYTE_LENGTH; i++) {
			uas_id[i] = rand() % 256;
		}
	}

	/* Test conflict priorities */
	const uint8_t states_enable_hold [4] {
		vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION,
		vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF,
		vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET,
		vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF,
	};

	const uint8_t states_enable_rtl [2] {
		vehicle_status_s::NAVIGATION_STATE_ORBIT,
		vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER,
	};

	const uint8_t states_enable_land [1] {
		vehicle_status_s::NAVIGATION_STATE_AUTO_RTL
	};

	const uint8_t states_enable_termination [8] {
		vehicle_status_s::NAVIGATION_STATE_AUTO_LAND,
		vehicle_status_s::NAVIGATION_STATE_DESCEND,
		vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND,
		vehicle_status_s::NAVIGATION_STATE_MANUAL,
		vehicle_status_s::NAVIGATION_STATE_ALTCTL,
		vehicle_status_s::NAVIGATION_STATE_POSCTL,
		vehicle_status_s::NAVIGATION_STATE_ACRO,
		vehicle_status_s::NAVIGATION_STATE_STAB
	};

	const uint8_t states_never_enable [2] {
		vehicle_status_s::NAVIGATION_STATE_TERMINATION,
		vehicle_status_s::NAVIGATION_STATE_OFFBOARD,
	};

	struct expected_outcome_s {
		bool hold_allowed;
		bool rtl_allowed;
		bool land_allowed;
		bool terminate_allowed;
	};

	void check_eval_conflict_escalation_action(Navigator *navigator_instance, const expected_outcome_s &expected_outcome,
			DaaAction action)
	{

		const uint8_t arming_state = vehicle_status_s::ARMING_STATE_ARMED;

		for (const uint8_t nav_state : states_enable_hold) {
			publish_vehicle_status(nav_state, arming_state);
			sync_navigator_topics(navigator_instance);
			ASSERT_TRUE(navigator_instance->get_detect_and_avoid()->eval_conflict_escalation_action(action) ==
				    expected_outcome.hold_allowed);
		}

		for (const uint8_t nav_state : states_enable_rtl) {
			publish_vehicle_status(nav_state, arming_state);
			sync_navigator_topics(navigator_instance);
			ASSERT_TRUE(navigator_instance->get_detect_and_avoid()->eval_conflict_escalation_action(action) ==
				    expected_outcome.rtl_allowed);
		}

		for (const uint8_t nav_state : states_enable_land) {
			publish_vehicle_status(nav_state, arming_state);
			sync_navigator_topics(navigator_instance);
			ASSERT_TRUE(navigator_instance->get_detect_and_avoid()->eval_conflict_escalation_action(action) ==
				    expected_outcome.land_allowed);
		}

		for (const uint8_t nav_state : states_enable_termination) {
			publish_vehicle_status(nav_state, arming_state);
			sync_navigator_topics(navigator_instance);
			ASSERT_TRUE(navigator_instance->get_detect_and_avoid()->eval_conflict_escalation_action(action) ==
				    expected_outcome.terminate_allowed);
		}

		for (const uint8_t nav_state : states_never_enable) {
			publish_vehicle_status(nav_state, arming_state);
			sync_navigator_topics(navigator_instance);
			ASSERT_FALSE(navigator_instance->get_detect_and_avoid()->eval_conflict_escalation_action(action));
		}
	};

	void check_highest_conflict(const uint8_t conflict_level, const bool action_required)
	{
		if (_detect_and_avoid_most_urgent_sub.updated()) {
			ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
		}

		const detect_and_avoid_most_urgent_s &daa_status = _detect_and_avoid_most_urgent_sub.get();
		ASSERT_TRUE(daa_status.conflict_level == conflict_level);
		ASSERT_TRUE(daa_status.has_action == action_required);
	}

	void expect_empty_most_urgent_status(const detect_and_avoid_most_urgent_s &status)
	{
		EXPECT_EQ(status.conflict_level, kDaaConflictLvlNone);
		EXPECT_FALSE(status.has_action);
		EXPECT_EQ(status.unique_id, 0ULL);
		EXPECT_EQ(status.unique_id_encoding, detect_and_avoid_most_urgent_s::UNIQUE_ID_ENCODING_ICAO);
		EXPECT_FLOAT_EQ(status.aircraft_dist, 9999.f);
		EXPECT_GT(status.timestamp, 0u);
	}

	std::vector<std::string> drain_mavlink_logs()
	{
		std::vector<std::string> logs;
		mavlink_log_s mavlink_log{};

		while (_mavlink_log_sub.update(&mavlink_log)) {
			logs.emplace_back(mavlink_log.text);
		}

		return logs;
	}

	size_t count_logs_with_prefix(const std::vector<std::string> &logs, const char *prefix)
	{
		size_t count = 0;

		for (const std::string &log : logs) {
			if (log.rfind(prefix, 0) == 0) {
				++count;
			}
		}

		return count;
	}

	bool any_log_contains(const std::vector<std::string> &logs, const char *needle)
	{
		for (const std::string &log : logs) {
			if (log.find(needle) != std::string::npos) {
				return true;
			}
		}

		return false;
	}

	void drain_detect_and_avoid_topic()
	{
		while (_detect_and_avoid_sub.update()) {}
	}

	void drain_detect_and_avoid_most_urgent_topic()
	{
		while (_detect_and_avoid_most_urgent_sub.update()) {}
	}

	void recreate_navigator()
	{
		navigator.reset();
		navigator = std::make_unique<Navigator>();
	}

	void sync_navigator_topics(Navigator *navigator_instance = nullptr)
	{
		Navigator *target = navigator_instance ? navigator_instance : navigator.get();

		if (_global_pos_sub.updated()) {
			vehicle_global_position_s global_position{};

			if (_global_pos_sub.copy(&global_position)) {
				*target->get_global_position() = global_position;
			}
		}

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s land_detected{};

			if (_vehicle_land_detected_sub.copy(&land_detected)) {
				*target->get_land_detected() = land_detected;
			}
		}

		vehicle_local_position_s local_position{};

		if (_vehicle_local_position_sub.copy(&local_position)) {
			*target->get_local_position() = local_position;
		}

		vehicle_status_s vehicle_status{};

		if (_vehicle_status_state_sub.copy(&vehicle_status)) {
			*target->get_vstatus() = vehicle_status;
		}
	}
};

struct action_priority_test_case_s {
	const char *name;
	DaaAction action;
	DetectAndAvoidTest::expected_outcome_s expected_outcome;
};

class DetectAndAvoidActionPriorityTest : public DetectAndAvoidTest, public ::testing::WithParamInterface<action_priority_test_case_s>
{
};

std::string action_priority_test_case_name(const ::testing::TestParamInfo<action_priority_test_case_s> &info)
{
	return info.param.name;
}

// WHY: Activation and deactivation gate all DetectAndAvoid processing and must fail closed on bad configuration.
// WHAT: Verify successful startup with default params, then check disabled and invalid parameter sets leave the module inactive.
TEST_F(DetectAndAvoidTest, OnActivation)
{
	// GIVEN: DetectAndAvoid starts with the default valid parameter set.
	EXPECT_TRUE(navigator->get_detect_and_avoid()->is_activated());

	// WHEN: The module is explicitly deactivated.
	navigator->get_detect_and_avoid()->on_inactivation();

	// THEN: It reports itself inactive.
	EXPECT_FALSE(navigator->get_detect_and_avoid()->is_activated());

	// WHEN: The feature is disabled through parameters before activation.
	const int daa_en = 0;
	param_t param_daa_en = param_handle(px4::params::DAA_EN);
	param_set(param_daa_en, &daa_en);

	navigator->get_detect_and_avoid()->on_activation();

	// THEN: Activation is refused while the feature is disabled.
	EXPECT_FALSE(navigator->get_detect_and_avoid()->is_activated());
	param_reset_all();

	// WHEN: A required F3442 parameter is invalid.
	const float negative_value = -10;
	param_t param_f34_lvl_critical_rad = param_handle(px4::params::F34_LVL_CRIT_RAD);
	param_set(param_f34_lvl_critical_rad, &negative_value);

	navigator->get_detect_and_avoid()->on_activation();

	// THEN: Activation fails closed.
	EXPECT_FALSE(navigator->get_detect_and_avoid()->is_activated());
	param_reset_all();

	param_t param_f34_lvl_high_rad = param_handle(px4::params::F34_LVL_HIGH_RAD);
	param_set(param_f34_lvl_high_rad, &negative_value);

	navigator->get_detect_and_avoid()->on_activation();
	EXPECT_FALSE(navigator->get_detect_and_avoid()->is_activated());
	param_reset_all();

	// WHEN: An unknown DAA standard is selected.
	const uint8_t daa_standard = 5;
	param_t param_daa_standard = param_handle(px4::params::DAA_STANDARD);
	param_set(param_daa_standard, &daa_standard);

	navigator->get_detect_and_avoid()->on_activation();

	// THEN: Unsupported standards are rejected as well.
	EXPECT_FALSE(navigator->get_detect_and_avoid()->is_activated());

	param_reset_all();
}

// WHY: Explicit deactivation should clear the published most-urgent state so downstream consumers never keep stale traffic after DAA stops.
// WHAT: Publish a critical conflict, call on_inactivation(), and verify a cleared detect_and_avoid_most_urgent sample is republished.
TEST_F(DetectAndAvoidTest, OnInactivationRepublishesClearedMostUrgentState)
{
	// GIVEN: Ownship is initialized and DetectAndAvoid has already published a critical most-urgent conflict.
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();
	drain_detect_and_avoid_most_urgent_topic();

	breach_distances_s breach_dist;
	get_DFLT_breach_distances(uav_vel, hor_velocity, breach_dist);

	const float critical_distance = breach_dist.nmac_hor - 1.f;
	const float critical_alt_diff = breach_dist.nmac_vert - 1.f;

	// WHEN: A critical traffic report is processed.
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(14545057, "DDF0A1", critical_distance, 0.0f, 0.0f,
				critical_alt_diff,
				hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});

	// THEN: A kDaaConflictLvlCritical conflict is published.
	ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
	EXPECT_EQ(_detect_and_avoid_most_urgent_sub.get().conflict_level, kDaaConflictLvlCritical);
	drain_detect_and_avoid_most_urgent_topic();

	// WHEN: The module is explicitly inactivated.
	navigator->get_detect_and_avoid()->on_inactivation();

	wait_until([&]() {
		return _detect_and_avoid_most_urgent_sub.updated();
	});

	// THEN: A no-conflict most-urgent sample is republished.
	EXPECT_FALSE(navigator->get_detect_and_avoid()->is_activated());
	ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
	expect_empty_most_urgent_status(_detect_and_avoid_most_urgent_sub.get());
}

// WHY: Failed activation should clear the published most-urgent state so downstream consumers never keep stale traffic after DAA stops.
// WHAT: Start from a published critical conflict, force activation failure with an invalid standard, and verify the replacement most-urgent sample is cleared.
TEST_F(DetectAndAvoidTest, FailedActivationPublishesBenignNoConflictOutput)
{
	// GIVEN: Ownship is initialized and DetectAndAvoid has already published a critical most-urgent conflict.
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();
	drain_detect_and_avoid_most_urgent_topic();

	breach_distances_s breach_dist;
	get_DFLT_breach_distances(uav_vel, hor_velocity, breach_dist);

	const float critical_distance = breach_dist.nmac_hor - 1.f;
	const float critical_alt_diff = breach_dist.nmac_vert - 1.f;

	// WHEN: A critical traffic report is processed.
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(14545057, "DDF0A1", critical_distance, 0.0f, 0.0f,
				critical_alt_diff,
				hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});

	// THEN: A kDaaConflictLvlCritical conflict is published.
	ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
	EXPECT_EQ(_detect_and_avoid_most_urgent_sub.get().conflict_level, kDaaConflictLvlCritical);
	drain_detect_and_avoid_most_urgent_topic();

	const uint8_t invalid_standard = 5;
	param_set(param_handle(px4::params::DAA_STANDARD), &invalid_standard);

	// WHEN: Activation is retried with an invalid DAA standard.
	navigator->get_detect_and_avoid()->on_activation();

	wait_until([&]() {
		return _detect_and_avoid_most_urgent_sub.updated();
	});

	// THEN: Activation fails and republishes a no-conflict most-urgent sample.
	EXPECT_FALSE(navigator->get_detect_and_avoid()->is_activated());
	ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
	expect_empty_most_urgent_status(_detect_and_avoid_most_urgent_sub.get());
}

// WHY: ICAO identifiers are vehicle-specific, so the default configuration must not suppress unrelated traffic.
// WHAT: Read the default primary and secondary ICAO parameters and verify both are disabled by default.
TEST_F(DetectAndAvoidTest, PrimaryAndSecondaryIcaoDefaultsDisabled)
{
	int32_t primary_icao = 0;
	ASSERT_EQ(param_get(param_handle(px4::params::ADSB_ICAO_ID), &primary_icao), 0);
	EXPECT_EQ(primary_icao, -1);

	int32_t secondary_icao = 0;
	ASSERT_EQ(param_get(param_handle(px4::params::ADSB_ICAO_ID_2), &secondary_icao), 0);
	EXPECT_EQ(secondary_icao, -1);
}

// WHY: The optional default vertical traffic speed directly changes F3442 conflict classification.
// WHAT: Run the same traffic scenario with the fallback velocity enabled and disabled and compare the reported conflict level.
TEST_F(DetectAndAvoidTest, DefaultVelocity)
{
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float traffic_hor_vel = 10000000.f;
	const float traffic_ver_vel = 10000000.f;
	const float alt_diff_out_of_conflict = 1000.f; // Outside all conflict zones if default vertical speed is used.
	conflict_info_s conflict;

	// WHEN: Default traffic vertical velocity handling is enabled before activation.
	const int daa_en_dflt_vel = 1;
	param_set(param_handle(px4::params::DAA_EN_DFLT_VEL), &daa_en_dflt_vel);
	recreate_navigator();

	// GIVEN: The vehicle is armed, airborne, and in mission mode.
	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	// THEN: Navigator state reflects the published ownship data.
	EXPECT_EQ(navigator->get_local_position()->vx, uav_vel(0));
	EXPECT_EQ(navigator->get_local_position()->vy, uav_vel(1));
	EXPECT_EQ(navigator->get_local_position()->vz, uav_vel(2));
	EXPECT_FALSE(navigator->get_land_detected()->landed);
	EXPECT_TRUE(navigator->get_vstatus()->arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	EXPECT_TRUE(navigator->get_vstatus()->nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
	EXPECT_EQ(navigator->get_global_position()->lat, lat_uav);
	EXPECT_EQ(navigator->get_global_position()->lon, lon_uav);
	EXPECT_EQ(navigator->get_global_position()->alt, alt_uav);

	PX4_DEBUG("F_TEST DAA: Default vel enabled.");
	navigator->get_detect_and_avoid()->fake_traffic(14545057, "DDF0A1", 10, 0.0f, 0.0f, alt_diff_out_of_conflict,
			traffic_hor_vel,
			traffic_ver_vel,
			1, lat_uav, lon_uav,
			alt_uav);
	navigator->check_traffic();
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);

	// THEN: The encounter stays out of conflict.
	EXPECT_TRUE(conflict.conflict_level == detect_and_avoid_s::DAA_CONFLICT_LVL_NONE);

	// WHEN: Default traffic vertical velocity handling is disabled before re-activation.
	PX4_DEBUG("F_TEST DAA: Default vel disabled.");
	const int daa_disable_dflt_vel = 0;
	param_set(param_handle(px4::params::DAA_EN_DFLT_VEL), &daa_disable_dflt_vel);
	recreate_navigator();
	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	navigator->get_detect_and_avoid()->fake_traffic(14545057, "DDF0A1", 10, 0.0f, 0.0f, alt_diff_out_of_conflict,
			traffic_hor_vel,
			traffic_ver_vel,
			1, lat_uav, lon_uav,
			alt_uav);
	navigator->check_traffic();
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);

	// THEN: The same encounter escalates into the augmented NMAC level.
	EXPECT_TRUE(conflict.conflict_level == detect_and_avoid_s::DAA_CONFLICT_LVL_MEDIUM);
}

// WHY: The nominal flow needs to cover self-detection, conflict prioritization, de-escalation, and final buffer cleanup.
// WHAT: Inject ownship traffic plus multiple external conflicts of different severities and verify the most urgent conflict evolves correctly.
TEST_F(DetectAndAvoidTest, BasicBehavior)
{
	// GIVEN: Ownship is initialized and traffic is expressed relative to it.
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};

	// GIVEN: The vehicle is armed, airborne, and in mission mode.
	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	// THEN: Navigator state reflects the published ownship data.
	EXPECT_EQ(navigator->get_local_position()->vx, uav_vel(0));
	EXPECT_EQ(navigator->get_local_position()->vy, uav_vel(1));
	EXPECT_EQ(navigator->get_local_position()->vz, uav_vel(2));
	EXPECT_FALSE(navigator->get_land_detected()->landed);
	EXPECT_TRUE(navigator->get_vstatus()->arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	EXPECT_TRUE(navigator->get_vstatus()->nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
	EXPECT_EQ(navigator->get_global_position()->lat, lat_uav);
	EXPECT_EQ(navigator->get_global_position()->lon, lon_uav);
	EXPECT_EQ(navigator->get_global_position()->alt, alt_uav);

	// GIVEN: Representative traffic parameters for the staged conflict sequence.
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f; // Not used because default velocity handling is enabled.

	conflict_info_s conflict;

	// WHEN: No traffic has been processed yet.
	navigator->check_traffic();
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);

	// THEN: The most urgent conflict starts at none.
	EXPECT_TRUE(conflict.conflict_level == detect_and_avoid_s::DAA_CONFLICT_LVL_NONE);

	// GIVEN: Self identifiers are configured for both primary and secondary ICAO encodings.
	const uint32_t own_icao = 10436515;
	param_set(param_handle(px4::params::ADSB_ICAO_ID), &own_icao);

	const uint32_t own_icao_2 = 9882899;
	param_set(param_handle(px4::params::ADSB_ICAO_ID_2), &own_icao_2);
	recreate_navigator();
	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	breach_distances_s breach_dist;
	get_DFLT_breach_distances(uav_vel, hor_velocity, breach_dist);

	// NMAC breach:
	const float in_nmac_distance = breach_dist.nmac_hor - 1;
	const float in_nmac_alt_diff = breach_dist.nmac_vert - 1;
	const float in_nmac_overall_dist = sqrtf(in_nmac_distance * in_nmac_distance +
					   in_nmac_alt_diff * in_nmac_alt_diff);

	// WC breach:
	const float in_wc_distance = breach_dist.nmac_hor + 1;
	const float in_wc_alt_diff =  breach_dist.nmac_vert + 1;
	const float in_wc_overall_dist = sqrtf(in_wc_distance * in_wc_distance +
					       in_wc_alt_diff * in_wc_alt_diff);

	// No more WC breach:
	const float no_more_wc_distance = breach_dist.wc_hor + 1;
	const float no_more_wc_alt_diff = breach_dist.wc_vert + 1;
	const float no_more_wc_overall_dist = sqrtf(no_more_wc_distance * no_more_wc_distance +
					      no_more_wc_alt_diff * no_more_wc_alt_diff);

	// No more Aug NMAC breach:
	const float no_more_aug_nmac_distance = breach_dist.aug_nmac_hor + 1;
	const float no_more_aug_nmac_alt_diff = breach_dist.aug_nmac_vert + 1;
	const float no_more_aug_nmac_overall_dist = sqrtf(no_more_aug_nmac_distance * no_more_aug_nmac_distance +
			no_more_aug_nmac_alt_diff * no_more_aug_nmac_alt_diff);

	// No more Aug WC breach:
	const float no_conflict_distance = breach_dist.aug_wc_hor + 1;
	const float no_conflict_alt_diff = breach_dist.aug_wc_vert + 1;

	// WHEN: Traffic matches the configured ownship identities.
	PX4_DEBUG("---- F_TEST DAA: own icao");
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(own_icao, "9F3FA3", in_nmac_distance, 0.0f, 0.0f, in_nmac_alt_diff,
				hor_velocity,
				ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);

	// THEN: Self detections do not create conflicts.
	EXPECT_TRUE(conflict.conflict_level == detect_and_avoid_s::DAA_CONFLICT_LVL_NONE);

	PX4_DEBUG("---- F_TEST DAA: own secondary icao");
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(own_icao_2, "96CD13", in_nmac_distance, 0.0f, 0.0f, in_nmac_alt_diff,
				hor_velocity,
				ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);
	EXPECT_TRUE(conflict.conflict_level == detect_and_avoid_s::DAA_CONFLICT_LVL_NONE);

	// WHEN: A critical NMAC breach is introduced.
	const uint32_t icao_ddfa0a1 = 14545057;
	PX4_DEBUG("---- F_TEST DAA: DDF0A1 NMAC breach");
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(icao_ddfa0a1, "DDF0A1", in_nmac_distance, 0.0f, 0.0f,
				in_nmac_alt_diff, hor_velocity,
				ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);
	EXPECT_TRUE(conflict.conflict_level == detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL);
	EXPECT_EQ(conflict.aircraft_dist, in_nmac_overall_dist);
	EXPECT_EQ(conflict.unique_id.id, icao_ddfa0a1);

	// WHEN: A weaker WC breach is added while the critical conflict is still active.
	const uint32_t icao_6e9f7b = 7249787;
	PX4_DEBUG("---- F_TEST DAA: 6E9F7B WC conflict, not most important.");
	navigator->get_detect_and_avoid()->fake_traffic(icao_6e9f7b, "6E9F7B", in_wc_distance, 0.0f, 0.0f, in_wc_alt_diff,
			hor_velocity,
			ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	navigator->check_traffic();
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);
	EXPECT_TRUE(conflict.conflict_level ==
		    detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL); // DDF0A1 conflict is still the most important
	EXPECT_EQ(conflict.aircraft_dist, in_nmac_overall_dist);
	EXPECT_EQ(conflict.unique_id.id, icao_ddfa0a1);

	// WHEN: The critical conflict is resolved while the WC breach remains.
	PX4_DEBUG("---- F_TEST DAA: DDF0A1 No more conflict, 6E9F7B WC conflict");
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(icao_ddfa0a1, "DDF0A1", no_conflict_distance, 0.0f, 0.0f,
				no_conflict_alt_diff,
				hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);
	EXPECT_TRUE(conflict.conflict_level == detect_and_avoid_s::DAA_CONFLICT_LVL_HIGH);
	EXPECT_EQ(conflict.aircraft_dist, in_wc_overall_dist);
	EXPECT_EQ(conflict.unique_id.id, icao_6e9f7b);

	// WHEN: The remaining traffic de-escalates from WC to augmented NMAC.
	PX4_DEBUG("---- F_TEST DAA: 6E9F7B no more WC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(icao_6e9f7b, "6E9F7B", no_more_wc_distance, 0.0f, 0.0f,
			no_more_wc_alt_diff,
			hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	navigator->check_traffic();
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);
	EXPECT_TRUE(conflict.conflict_level == detect_and_avoid_s::DAA_CONFLICT_LVL_MEDIUM);
	EXPECT_EQ(conflict.aircraft_dist, no_more_wc_overall_dist);
	EXPECT_EQ(conflict.unique_id.id, icao_6e9f7b);

	// WHEN: The traffic de-escalates again into augmented WC only.
	PX4_DEBUG("---- F_TEST DAA: 6E9F7B no more Aug NMAC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(icao_6e9f7b, "6E9F7B", no_more_aug_nmac_distance, 0.0f, 0.0f,
			no_more_aug_nmac_alt_diff,
			hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	navigator->check_traffic();
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);
	EXPECT_TRUE(conflict.conflict_level == detect_and_avoid_s::DAA_CONFLICT_LVL_LOW);
	EXPECT_EQ(conflict.aircraft_dist, no_more_aug_nmac_overall_dist);
	EXPECT_EQ(conflict.unique_id.id, icao_6e9f7b);

	// WHEN: The last remaining traffic exits all conflict zones.
	PX4_DEBUG("---- F_TEST DAA: 6E9F7B no more conflict");
	navigator->get_detect_and_avoid()->fake_traffic(icao_6e9f7b, "6E9F7B", no_conflict_distance, 0.0f, 0.0f,
			no_conflict_alt_diff,
			hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	navigator->check_traffic();
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);

	// THEN: The conflict buffer fully de-escalates back to none.
	EXPECT_TRUE(conflict.conflict_level == detect_and_avoid_s::DAA_CONFLICT_LVL_NONE);
}

// WHY: F3442 conflict zones are nested, so a disabled inner-zone action must fall back to the next enabled breached zone.
// WHAT: Disable the critical-zone action, keep the high-zone action enabled, trigger a critical conflict, and check that land is commanded.
TEST_F(DetectAndAvoidTest, DisabledHigherPriorityConflictFallsBackToEnabledZone)
{
	// GIVEN: Critical conflicts are configured to take no action while high conflicts still command land.
	const int critical_action = action_param_value(DaaAction::kDisabled);
	const int high_action = action_param_value(DaaAction::kLandMode);
	param_set(param_handle(px4::params::DAA_LVL_CRIT_ACT), &critical_action);
	param_set(param_handle(px4::params::DAA_LVL_HIGH_ACT), &high_action);
	recreate_navigator();

	// GIVEN: Ownship is initialized in a mission state with representative motion.
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	breach_distances_s breach_dist;
	get_DFLT_breach_distances(uav_vel, hor_velocity, breach_dist);

	const float in_nmac_distance = breach_dist.nmac_hor - 1.f;
	const float in_nmac_alt_diff = breach_dist.nmac_vert - 1.f;

	// WHEN: Traffic breaches the critical NMAC volume.
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(14545057, "DDF0A1", in_nmac_distance, 0.0f, 0.0f, in_nmac_alt_diff,
				hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});

	conflict_info_s conflict;
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);
	EXPECT_EQ(conflict.conflict_level, kDaaConflictLvlCritical);
	check_highest_conflict(kDaaConflictLvlCritical, true);

	// THEN: The module falls back to the next enabled breached action and commands land.
	ASSERT_TRUE(_vehicle_command_sub.update());
	EXPECT_EQ(_vehicle_command_sub.get().command, vehicle_command_s::VEHICLE_CMD_NAV_LAND);
}

// WHY: Resetting the module must clear buffered conflicts so stale traffic cannot survive a reactivation.
// WHAT: Create a critical conflict, reactivate the module, then verify a later low-priority conflict is treated as the only remaining traffic.
TEST_F(DetectAndAvoidTest, ResetClearsTrafficBuffer)
{
	// GIVEN: Ownship is initialized with a traffic encounter that becomes critically urgent.
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	breach_distances_s breach_dist;
	get_DFLT_breach_distances(uav_vel, hor_velocity, breach_dist);

	const uint32_t critical_traffic_icao = 14545057;
	const float in_nmac_distance = breach_dist.nmac_hor - 1.f;
	const float in_nmac_alt_diff = breach_dist.nmac_vert - 1.f;

	// WHEN: The critical conflict is processed.
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(critical_traffic_icao, "DDF0A1", in_nmac_distance, 0.0f, 0.0f,
				in_nmac_alt_diff, hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});

	conflict_info_s conflict;
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);
	ASSERT_EQ(conflict.unique_id.id, critical_traffic_icao);
	ASSERT_EQ(conflict.conflict_level, kDaaConflictLvlCritical);

	// WHEN: The module is deactivated and activated again.
	navigator->get_detect_and_avoid()->on_inactivation();
	navigator->get_detect_and_avoid()->on_activation();
	ASSERT_TRUE(navigator->get_detect_and_avoid()->is_activated());
	sync_navigator_topics();

	const uint32_t low_traffic_icao = 7249787;
	const float low_conflict_distance = breach_dist.aug_nmac_hor + 1.f;
	const float low_conflict_alt_diff = breach_dist.aug_nmac_vert + 1.f;

	// WHEN: A new low-priority conflict is published after the reset.
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(low_traffic_icao, "6E9F7B", low_conflict_distance, 0.0f, 0.0f,
				low_conflict_alt_diff, hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});

	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);

	// THEN: The old critical conflict is gone and only the new traffic remains.
	EXPECT_EQ(conflict.unique_id.id, low_traffic_icao);
	EXPECT_EQ(conflict.conflict_level, kDaaConflictLvlLow);
}

// WHY: The traffic subscription is queued, and dropping earlier reports would let a later low-priority message hide a real hazard.
// WHAT: Publish two reports back to back in one cycle and verify the critical one is still selected as most urgent.
TEST_F(DetectAndAvoidTest, ProcessesAllQueuedTrafficReports)
{
	// GIVEN: Ownship is initialized and two traffic reports will arrive in the same processing cycle.
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	breach_distances_s breach_dist;
	get_DFLT_breach_distances(uav_vel, hor_velocity, breach_dist);

	const uint32_t critical_traffic_icao = 14545057;
	const uint32_t low_traffic_icao = 7249787;
	const float in_nmac_distance = breach_dist.nmac_hor - 1.f;
	const float in_nmac_alt_diff = breach_dist.nmac_vert - 1.f;
	const float low_conflict_distance = breach_dist.aug_nmac_hor + 1.f;
	const float low_conflict_alt_diff = breach_dist.aug_nmac_vert + 1.f;

	// WHEN: The highest-priority conflict is queued first and a lower-priority one second.
	navigator->get_detect_and_avoid()->fake_traffic(critical_traffic_icao, "DDF0A1", in_nmac_distance, 0.0f, 0.0f,
			in_nmac_alt_diff, hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(low_traffic_icao, "6E9F7B", low_conflict_distance, 0.0f, 0.0f,
				low_conflict_alt_diff, hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});

	conflict_info_s conflict;
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);

	// THEN: The queue is drained completely and the critical conflict still wins.
	EXPECT_EQ(conflict.unique_id.id, critical_traffic_icao);
	EXPECT_EQ(conflict.conflict_level, kDaaConflictLvlCritical);
}

// WHY: Operators need a way to cancel a queued fake-traffic script before it publishes more synthetic traffic.
// WHAT: Arm a fake-traffic mode, stop it before the next navigator cycle, and verify no synthetic conflicts are emitted.
TEST_F(DetectAndAvoidTest, StopFakeTrafficCancelsPendingScript)
{
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	navigator->run_fake_traffic(DetectAndAvoid::FakeTraffMode::kQueueFill);
	navigator->stop_fake_traffic();
	drain_mavlink_logs();

	navigator->check_traffic();

	conflict_info_s conflict;
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);

	EXPECT_EQ(conflict.conflict_level, kDaaConflictLvlNone);

	const std::vector<std::string> logs = drain_mavlink_logs();
	EXPECT_EQ(count_logs_with_prefix(logs, "DAA New and Main:"), 0u);
	EXPECT_EQ(count_logs_with_prefix(logs, "DAA New "), 0u);
}

// WHY: Zero latitude is a valid coordinate and must not be rejected as an uninitialized value.
// WHAT: Publish traffic on the equator and verify it still produces a conflict.
TEST_F(DetectAndAvoidTest, AcceptsTrafficOnZeroLatitude)
{
	// GIVEN: Ownship is positioned on the equator, where latitude is exactly zero.
	const double lat_uav = 0.0;
	const double lon_uav = 1.0;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	const uint32_t traffic_icao = 14545057;
	const uint16_t flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS |
			       transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE;
	transponder_report_s tr = create_transponder_report(traffic_icao, "DDF0A1", 0.0, lon_uav, alt_uav + 1.f, 0.f, 0.f,
				  flags);

	// WHEN: Traffic is published with that zero-latitude coordinate.
	publish_transponder_report_and_check(tr);

	conflict_info_s conflict;
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);

	// THEN: The valid equatorial coordinate is processed as a real conflict.
	EXPECT_EQ(conflict.unique_id.id, traffic_icao);
	EXPECT_EQ(conflict.conflict_level, kDaaConflictLvlCritical);
}

// WHY: Runtime parameter changes must stop and restart DAA cleanly so operators never see stale outputs and re-enabling works without a reboot.
// WHAT: Create a conflict, disable the module and verify the cleared publication, then re-enable it and confirm traffic processing resumes.
TEST_F(DetectAndAvoidTest, RuntimeDisableAndReenableUpdatesState)
{
	// GIVEN: Ownship is initialized with an active critical conflict.
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	breach_distances_s breach_dist;
	get_DFLT_breach_distances(uav_vel, hor_velocity, breach_dist);

	const float critical_distance = breach_dist.nmac_hor - 1.f;
	const float critical_alt_diff = breach_dist.nmac_vert - 1.f;

	// WHEN: The critical traffic report is processed.
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(14545057, "DDF0A1", critical_distance, 0.0f, 0.0f,
				critical_alt_diff,
				hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});

	conflict_info_s conflict;
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);
	ASSERT_EQ(conflict.conflict_level, kDaaConflictLvlCritical);

	// WHEN: The feature is disabled at runtime and a parameter update is published.
	const int daa_en = 0;
	param_set(param_handle(px4::params::DAA_EN), &daa_en);
	publish_parameter_update();

	ASSERT_TRUE(navigator->get_detect_and_avoid()->is_activated());
	navigator->check_traffic();
	EXPECT_FALSE(navigator->get_detect_and_avoid()->is_activated());

	ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
	const detect_and_avoid_most_urgent_s &status = _detect_and_avoid_most_urgent_sub.get();
	EXPECT_EQ(status.conflict_level, kDaaConflictLvlNone);
	EXPECT_FALSE(status.has_action);
	EXPECT_GT(status.timestamp, 0u);

	// WHEN: The feature is re-enabled and ownship plus traffic are published again.
	// The parameter update subscription is throttled to 1 s, so we wait past that
	// window to make sure the second update is actually picked up.
	px4_usleep(1100000);

	const int daa_reenable = 1;
	param_set(param_handle(px4::params::DAA_EN), &daa_reenable);
	publish_parameter_update();
	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();
	navigator->check_traffic();
	EXPECT_TRUE(navigator->get_detect_and_avoid()->is_activated());

	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(14545057, "DDF0A1", critical_distance, 0.0f, 0.0f,
				critical_alt_diff,
				hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});

	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);

	// THEN: Disabling clears the published state and re-enabling restores normal processing.
	EXPECT_EQ(conflict.conflict_level, kDaaConflictLvlCritical);
}

// WHY: Runtime notification throttling must fail safe even if parameter bounds are bypassed, otherwise a negative interval can silence all on-ground DAA warnings.
// WHAT: Force a negative notification interval before activation, trigger a landed critical conflict that requires action, and verify the warning is still emitted immediately.
TEST_F(DetectAndAvoidTest, NegativeNotificationIntervalIsClampedToZero)
{
	const int negative_notification_interval = -1;
	const int critical_action = action_param_value(DaaAction::kLandMode);
	param_set(param_handle(px4::params::DAA_NOTIF_STATE), &negative_notification_interval);
	param_set(param_handle(px4::params::DAA_LVL_CRIT_ACT), &critical_action);
	recreate_navigator();

	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	publish_global_pos(lat_uav, lon_uav, alt_uav);
	publish_land_status(true);
	publish_vehicle_status(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION, vehicle_status_s::ARMING_STATE_ARMED);
	publish_local_pos_vel(uav_vel);
	sync_navigator_topics();
	drain_mavlink_logs();

	breach_distances_s breach_dist;
	get_DFLT_breach_distances(uav_vel, hor_velocity, breach_dist);
	const float critical_distance = breach_dist.nmac_hor - 1.f;
	const float critical_alt_diff = breach_dist.nmac_vert - 1.f;

	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(14545057, "DDF0A1", critical_distance, 0.0f, 0.0f,
				critical_alt_diff,
				hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});

	wait_for_topic_update(_mavlink_log_sub);
	const std::vector<std::string> logs = drain_mavlink_logs();
	EXPECT_TRUE(any_log_contains(logs, "do not takeoff"));
}

// WHY: The first landed warning should not depend on system uptime; otherwise large positive notification intervals can suppress the initial safety message.
// WHAT: Configure a long notification interval, trigger a landed critical conflict that requires action, and verify the first warning is emitted immediately.
TEST_F(DetectAndAvoidTest, FirstLandedWarningIsImmediateWithPositiveInterval)
{
	// GIVEN: A long positive notification interval is configured before activation and ownship is landed and armed.
	const int long_notification_interval = 24 * 60 * 60;
	const int critical_action = action_param_value(DaaAction::kLandMode);
	param_set(param_handle(px4::params::DAA_NOTIF_STATE), &long_notification_interval);
	param_set(param_handle(px4::params::DAA_LVL_CRIT_ACT), &critical_action);
	recreate_navigator();

	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	publish_global_pos(lat_uav, lon_uav, alt_uav);
	publish_land_status(true);
	publish_vehicle_status(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION, vehicle_status_s::ARMING_STATE_ARMED);
	publish_local_pos_vel(uav_vel);
	sync_navigator_topics();
	drain_mavlink_logs();

	breach_distances_s breach_dist;
	get_DFLT_breach_distances(uav_vel, hor_velocity, breach_dist);
	const float critical_distance = breach_dist.nmac_hor - 1.f;
	const float critical_alt_diff = breach_dist.nmac_vert - 1.f;

	// WHEN: A landed critical conflict that requires action is processed.
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(14545057, "DDF0A1", critical_distance, 0.0f, 0.0f,
				critical_alt_diff,
				hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});

	wait_for_topic_update(_mavlink_log_sub);
	const std::vector<std::string> logs = drain_mavlink_logs();

	// THEN: The first on-ground warning is emitted immediately instead of waiting for the interval to elapse.
	EXPECT_TRUE(any_log_contains(logs, "do not takeoff"));
}

// WHY: Runtime action updates must not re-evaluate the current buffer, but later conflict escalations must use the refreshed action value.
// WHAT: Start with a medium conflict, change the high-level action to hold at runtime, verify the parameter update alone publishes no command, then escalate to high and confirm hold is commanded.
TEST_F(DetectAndAvoidTest, RuntimeF3442ActionUpdateAppliesOnNextEscalationOnly)
{
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();
	drain_detect_and_avoid_most_urgent_topic();

	while (_vehicle_command_sub.update()) {}

	breach_distances_s breach_dist;
	get_DFLT_breach_distances(uav_vel, hor_velocity, breach_dist);

	const float medium_distance = breach_dist.aug_nmac_hor - 1.f;
	const float medium_alt_diff = breach_dist.aug_nmac_vert - 1.f;
	const float high_distance = breach_dist.wc_hor - 1.f;
	const float high_alt_diff = breach_dist.wc_vert - 1.f;

	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(14545057, "DDF0A1", medium_distance, 0.0f, 0.0f,
				medium_alt_diff,
				hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});

	ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
	EXPECT_EQ(_detect_and_avoid_most_urgent_sub.get().conflict_level, kDaaConflictLvlMedium);
	EXPECT_FALSE(_detect_and_avoid_most_urgent_sub.get().has_action);
	EXPECT_FALSE(_vehicle_command_sub.update());

	const int high_action_hold = action_param_value(DaaAction::kPositionHoldMode);
	param_set(param_handle(px4::params::DAA_LVL_HIGH_ACT), &high_action_hold);
	publish_parameter_update();
	drain_detect_and_avoid_most_urgent_topic();

	while (_vehicle_command_sub.update()) {}

	navigator->check_traffic();
	EXPECT_FALSE(_detect_and_avoid_most_urgent_sub.update());
	EXPECT_FALSE(_vehicle_command_sub.update());

	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(14545057, "DDF0A1", high_distance, 0.0f, 0.0f,
				high_alt_diff,
				hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});

	ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
	EXPECT_EQ(_detect_and_avoid_most_urgent_sub.get().conflict_level, kDaaConflictLvlHigh);
	EXPECT_TRUE(_detect_and_avoid_most_urgent_sub.get().has_action);
	ASSERT_TRUE(_vehicle_command_sub.update());
	EXPECT_EQ(_vehicle_command_sub.get().command, vehicle_command_s::VEHICLE_CMD_DO_SET_MODE);
}

// WHY: Runtime NAV_TRAFF_AVOID action updates must not re-evaluate the current buffer, but later conflict escalations must use the refreshed action value.
// WHAT: Start with a crosstrack conflict in warn-only mode, switch NAV_TRAFF_AVOID to terminate at runtime, verify no immediate command is sent, then clear and re-trigger the conflict and confirm termination is commanded.
TEST_F(DetectAndAvoidTest, RuntimeCrosstrackTerminateUpdateAppliesOnNextConflict)
{
	const int crosstrack_standard = detect_and_avoid_s::DAA_STANDARD_CROSSTRACK;
	const int warn_only = action_param_value(DaaAction::kWarnOnly);
	const int terminate_action = action_param_value(DaaAction::kTerminate);
	param_set(param_handle(px4::params::DAA_STANDARD), &crosstrack_standard);
	param_set(param_handle(px4::params::NAV_TRAFF_AVOID), &warn_only);
	recreate_navigator();

	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{0.f, 0.f, 0.f};
	const float conflict_distance = 200.f;
	const float resolve_distance = 5000.f;
	const float traffic_heading = 3.f * M_PI_2_F;
	const float traffic_direction = M_PI_2_F;
	const float hor_velocity = 30.f;

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();
	drain_detect_and_avoid_most_urgent_topic();

	while (_vehicle_command_sub.update()) {}

	const auto publish_crosstrack_sample = [&](const float distance) {
		publish_traffic_and_check([&]() {
			navigator->get_detect_and_avoid()->fake_traffic(14545057, "DDF0A1", distance, traffic_direction, traffic_heading,
					0.f, hor_velocity, 0.f, 1, lat_uav, lon_uav, alt_uav);
		});
	};

	publish_crosstrack_sample(conflict_distance);

	ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
	EXPECT_EQ(_detect_and_avoid_most_urgent_sub.get().conflict_level, kDaaConflictLvlHigh);
	EXPECT_FALSE(_detect_and_avoid_most_urgent_sub.get().has_action);
	EXPECT_FALSE(_vehicle_command_sub.update());

	param_set(param_handle(px4::params::NAV_TRAFF_AVOID), &terminate_action);
	publish_parameter_update();
	drain_detect_and_avoid_most_urgent_topic();

	while (_vehicle_command_sub.update()) {}

	navigator->check_traffic();
	EXPECT_FALSE(_detect_and_avoid_most_urgent_sub.update());
	EXPECT_FALSE(_vehicle_command_sub.update());

	publish_crosstrack_sample(resolve_distance);

	ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
	EXPECT_EQ(_detect_and_avoid_most_urgent_sub.get().conflict_level, kDaaConflictLvlNone);
	EXPECT_FALSE(_detect_and_avoid_most_urgent_sub.get().has_action);

	publish_crosstrack_sample(conflict_distance);

	ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
	EXPECT_EQ(_detect_and_avoid_most_urgent_sub.get().conflict_level, kDaaConflictLvlHigh);
	EXPECT_TRUE(_detect_and_avoid_most_urgent_sub.get().has_action);
	ASSERT_TRUE(_vehicle_command_sub.update());
	EXPECT_EQ(_vehicle_command_sub.get().command, vehicle_command_s::VEHICLE_CMD_DO_FLIGHTTERMINATION);
}

// WHY: Invalid traffic altitude should be rejected before any DAA math runs to avoid propagating NaNs through conflict evaluation.
// WHAT: Publish a report with a NaN altitude and verify that no detect_and_avoid sample is published and no conflict is buffered.
TEST_F(DetectAndAvoidTest, RejectsNonFiniteTrafficAltitude)
{
	// GIVEN: Ownship is initialized and the detect_and_avoid topic is drained.
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();
	drain_detect_and_avoid_topic();

	const uint16_t flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS |
			       transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE;
	transponder_report_s tr = create_transponder_report(14545057, "DDF0A1", lat_uav, lon_uav,
				  std::numeric_limits<float>::quiet_NaN(), 0.f, 0.f, flags);

	// WHEN: Traffic is published with a NaN altitude.
	publish_transponder_report_and_check(tr);

	// THEN: No output is published and no conflict is buffered.
	EXPECT_FALSE(_detect_and_avoid_sub.update());

	conflict_info_s conflict;
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);
	EXPECT_EQ(conflict.conflict_level, kDaaConflictLvlNone);
}

// WHY: Crosstrack mode depends on a finite traffic heading and must fail closed if the heading value is NaN or Inf.
// WHAT: Switch to crosstrack mode, publish invalid heading values with the VALID_HEADING flag set, and verify no output or buffered conflict is produced.
TEST_F(DetectAndAvoidTest, CrosstrackRejectsNonFiniteTrafficHeading)
{
	// GIVEN: DetectAndAvoid is recreated in crosstrack mode.
	const int crosstrack_standard = detect_and_avoid_s::DAA_STANDARD_CROSSTRACK;
	param_set(param_handle(px4::params::DAA_STANDARD), &crosstrack_standard);
	recreate_navigator();

	// GIVEN: Ownship is initialized and traffic headings are marked as valid.
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	const uint16_t flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS |
			       transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE |
			       transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING |
			       transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY;

	// WHEN: Traffic is published with non-finite heading values.
	for (const float heading : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
		drain_detect_and_avoid_topic();

		transponder_report_s tr = create_transponder_report(14545057, "DDF0A1", lat_uav, lon_uav,
					  alt_uav, 30.f, 0.f, flags);
		tr.heading = heading;
		publish_transponder_report_and_check(tr);

		// THEN: No detect_and_avoid output is published and no conflict is buffered.
		EXPECT_FALSE(_detect_and_avoid_sub.update());

		conflict_info_s conflict;
		navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);
		EXPECT_EQ(conflict.conflict_level, kDaaConflictLvlNone);
	}
}

// WHY: A hovering ownship has no meaningful course-over-ground, so crosstrack mode must use the local-position yaw instead of atan2f(0, 0).
// WHAT: Publish a hovering ownship with unavailable local yaw and verify crosstrack processing fails instead of inventing a north heading.
TEST_F(DetectAndAvoidTest, HoveringCrosstrackRejectsNonFiniteOwnshipHeading)
{
	// GIVEN: DetectAndAvoid is recreated in crosstrack mode.
	const int crosstrack_standard = detect_and_avoid_s::DAA_STANDARD_CROSSTRACK;
	param_set(param_handle(px4::params::DAA_STANDARD), &crosstrack_standard);
	recreate_navigator();

	// GIVEN: Ownship is hovering and local-position yaw is unavailable.
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{0.f, 0.f, 0.f};

	publish_global_pos(lat_uav, lon_uav, alt_uav);
	publish_land_status(false);
	publish_vehicle_status(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION, vehicle_status_s::ARMING_STATE_ARMED);
	publish_local_pos_vel(uav_vel, hrt_absolute_time(), std::numeric_limits<float>::quiet_NaN());
	sync_navigator_topics();
	drain_detect_and_avoid_topic();

	const uint16_t flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS |
			       transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE |
			       transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING |
			       transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY;
	transponder_report_s tr = create_transponder_report(14545057, "DDF0A1", lat_uav, lon_uav,
				  alt_uav, 30.f, 0.f, flags);
	tr.heading = 3.f * M_PI_2_F;

	// WHEN: A traffic report that would otherwise be processed by crosstrack mode is received.
	publish_transponder_report_and_check(tr);

	// THEN: No output is published and no conflict is buffered because ownship yaw is not finite.
	EXPECT_FALSE(_detect_and_avoid_sub.update());

	conflict_info_s conflict;
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);
	EXPECT_EQ(conflict.conflict_level, kDaaConflictLvlNone);
}

// WHY: DAA must stop exposing buffered conflicts once the uav's position or velocity timestamps go stale.
// WHAT: Create an active conflict, then republish uav's state with old timestamps and verify the conflict buffer and most-urgent topic reset to no-conflict.
TEST_F(DetectAndAvoidTest, ClearsConflictStateWhenOwnshipDataGoesStale)
{
	// GIVEN: Ownship is initialized with an active critical conflict.
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	breach_distances_s breach_dist;
	get_DFLT_breach_distances(uav_vel, hor_velocity, breach_dist);
	const float critical_distance = breach_dist.nmac_hor - 1.f;
	const float critical_alt_diff = breach_dist.nmac_vert - 1.f;

	// WHEN: The critical traffic report is processed.
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(14545057, "DDF0A1", critical_distance, 0.0f, 0.0f,
				critical_alt_diff, hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});

	conflict_info_s conflict;
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);
	ASSERT_EQ(conflict.conflict_level, kDaaConflictLvlCritical);

	drain_detect_and_avoid_most_urgent_topic();

	// WHEN: Ownship position and velocity are republished with stale timestamps.
	const hrt_abstime stale_timestamp = hrt_absolute_time() - 2_s;
	publish_global_pos(lat_uav, lon_uav, alt_uav, stale_timestamp);
	publish_local_pos_vel(uav_vel, stale_timestamp);
	wait_until([&]() {
		sync_navigator_topics();
		return navigator->get_global_position()->timestamp == stale_timestamp &&
		       navigator->get_local_position()->timestamp == stale_timestamp;
	});

	wait_until([&]() {
		navigator->check_traffic();
		navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);
		return conflict.conflict_level == kDaaConflictLvlNone && _detect_and_avoid_most_urgent_sub.updated();
	});

	// THEN: The buffered conflict and the most-urgent publication are both cleared.
	EXPECT_EQ(conflict.conflict_level, kDaaConflictLvlNone);

	ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
	const detect_and_avoid_most_urgent_s &status = _detect_and_avoid_most_urgent_sub.get();
	EXPECT_FALSE(status.has_action);
	EXPECT_EQ(status.conflict_level, kDaaConflictLvlNone);
}

// WHY: Numeric collisions between ID encodings must not merge unrelated aircraft into one buffer slot.
// WHAT: Use an ICAO and a callsign that encode to the same integer and verify their conflicts remain independent across updates and removals.
TEST_F(DetectAndAvoidTest, DifferentEncodingsDoNotShareBufferSlot)
{
	// GIVEN: Ownship is initialized and the DAA publications are drained.
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();
	drain_detect_and_avoid_topic();
	drain_detect_and_avoid_most_urgent_topic();

	breach_distances_s breach_dist;
	get_DFLT_breach_distances(uav_vel, hor_velocity, breach_dist);

	const char colliding_callsign[] = "ABC";
	const uint64_t colliding_unique_id = navigator->get_detect_and_avoid()->callsign_to_uint64(colliding_callsign);
	ASSERT_GT(colliding_unique_id, 0u);
	ASSERT_LE(colliding_unique_id, static_cast<uint64_t>(UINT32_MAX));

	const uint32_t colliding_icao = static_cast<uint32_t>(colliding_unique_id);
	const float critical_distance = breach_dist.nmac_hor - 1.f;
	const float critical_alt_diff = breach_dist.nmac_vert - 1.f;
	const float low_distance = breach_dist.aug_nmac_hor + 1.f;
	const float low_alt_diff = breach_dist.aug_nmac_vert + 1.f;
	const float no_conflict_distance = breach_dist.aug_wc_hor + 1.f;
	const float no_conflict_alt_diff = breach_dist.aug_wc_vert + 1.f;

	// WHEN: ICAO traffic occupies the colliding numeric slot first.
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(colliding_icao, "DDF0A1", critical_distance, 0.0f, 0.0f,
				critical_alt_diff,
				hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});

	conflict_info_s conflict;
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);
	ASSERT_EQ(conflict.unique_id.encoding, detect_and_avoid_s::UNIQUE_ID_ENCODING_ICAO);
	ASSERT_EQ(conflict.unique_id.id, colliding_icao);
	ASSERT_EQ(conflict.conflict_level, kDaaConflictLvlCritical);
	ASSERT_TRUE(_detect_and_avoid_sub.update());
	const detect_and_avoid_s &first_report = _detect_and_avoid_sub.get();
	EXPECT_EQ(first_report.unique_id, colliding_icao);
	EXPECT_EQ(first_report.unique_id_encoding, detect_and_avoid_s::UNIQUE_ID_ENCODING_ICAO);
	ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
	const detect_and_avoid_most_urgent_s &first_status = _detect_and_avoid_most_urgent_sub.get();
	EXPECT_EQ(first_status.unique_id, colliding_icao);
	EXPECT_EQ(first_status.unique_id_encoding, detect_and_avoid_most_urgent_s::UNIQUE_ID_ENCODING_ICAO);

	// WHEN: Callsign-based traffic that hashes to the same numeric value is added.
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(0, colliding_callsign, low_distance, 0.0f, 0.0f, low_alt_diff,
				hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});

	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);
	ASSERT_EQ(conflict.unique_id.encoding, detect_and_avoid_s::UNIQUE_ID_ENCODING_ICAO);
	ASSERT_EQ(conflict.unique_id.id, colliding_icao);
	ASSERT_EQ(conflict.conflict_level, kDaaConflictLvlCritical);

	// WHEN: The ICAO conflict resolves while the callsign-based traffic remains active.
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(colliding_icao, "DDF0A1", no_conflict_distance, 0.0f, 0.0f,
				no_conflict_alt_diff,
				hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});

	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);

	// THEN: The callsign traffic remains in its own slot and becomes the most urgent entry.
	EXPECT_EQ(conflict.unique_id.encoding, detect_and_avoid_s::UNIQUE_ID_ENCODING_ADSB_CALLSIGN);
	EXPECT_EQ(conflict.unique_id.id, colliding_unique_id);
	EXPECT_EQ(conflict.conflict_level, kDaaConflictLvlLow);
	ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
	const detect_and_avoid_most_urgent_s &callsign_status = _detect_and_avoid_most_urgent_sub.get();
	EXPECT_EQ(callsign_status.unique_id, colliding_unique_id);
	EXPECT_EQ(callsign_status.unique_id_encoding, detect_and_avoid_most_urgent_s::UNIQUE_ID_ENCODING_ADSB_CALLSIGN);
}

// WHY: A new conflict that is immediately the most urgent one should notify the operator only once.
// WHAT: Inject a single warning-level conflict and verify it emits one combined "new and main" message instead of separate new and main logs.
TEST_F(DetectAndAvoidTest, NewMostUrgentConflictUsesCombinedNotification)
{
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();
	drain_mavlink_logs();

	breach_distances_s breach_dist;
	get_DFLT_breach_distances(uav_vel, hor_velocity, breach_dist);

	const float critical_distance = breach_dist.nmac_hor - 1.f;
	const float critical_alt_diff = breach_dist.nmac_vert - 1.f;

	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(14545057, "DDF0A1", critical_distance, 0.0f, 0.0f,
				critical_alt_diff,
				hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});

	const std::vector<std::string> logs = drain_mavlink_logs();
	EXPECT_EQ(count_logs_with_prefix(logs, "DAA New and Main:"), 1u);
	EXPECT_EQ(count_logs_with_prefix(logs, "DAA Main:"), 0u);
	EXPECT_FALSE(any_log_contains(logs, "DAA New DDF0A1"));
}

// WHY: If a new conflict becomes the most urgent one without changing the highest conflict level, the operator still needs a combined main notification.
// WHAT: Add a closer conflict at the same level as the current main conflict and verify it emits the combined "new and main" message immediately.
TEST_F(DetectAndAvoidTest, NewMostUrgentConflictSameLevelStillNotifiesAsMain)
{
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	breach_distances_s breach_dist;
	get_DFLT_breach_distances(uav_vel, hor_velocity, breach_dist);

	const float far_critical_distance = breach_dist.nmac_hor - 1.f;
	const float close_critical_distance = breach_dist.nmac_hor - 10.f;
	const float critical_alt_diff = breach_dist.nmac_vert - 1.f;

	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(14545057, "DDF0A1", far_critical_distance, 0.0f, 0.0f,
				critical_alt_diff,
				hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});

	drain_mavlink_logs();

	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(7249787, "6E9F7B", close_critical_distance, 0.0f, 0.0f,
				critical_alt_diff,
				hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});

	conflict_info_s conflict;
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);
	ASSERT_EQ(conflict.unique_id.id, 7249787u);
	ASSERT_EQ(conflict.conflict_level, kDaaConflictLvlCritical);

	const std::vector<std::string> logs = drain_mavlink_logs();
	EXPECT_EQ(count_logs_with_prefix(logs, "DAA New and Main:"), 1u);
	EXPECT_EQ(count_logs_with_prefix(logs, "DAA Main:"), 0u);
	EXPECT_FALSE(any_log_contains(logs, "DAA New 6E9F7B"));
	EXPECT_TRUE(any_log_contains(logs, "DAA New and Main: 6E9F7B lvl 4."));
}

// WHY: Escalating the primary conflict should not also emit a secondary-traffic notification for the same update.
// WHAT: Raise the most urgent conflict while another traffic item remains active and verify only the main-status log is emitted.
TEST_F(DetectAndAvoidTest, MostUrgentEscalationDoesNotSendSecondaryNotification)
{
	// GIVEN: One primary medium conflict and one secondary low conflict are active.
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	breach_distances_s breach_dist;
	get_DFLT_breach_distances(uav_vel, hor_velocity, breach_dist);

	const float medium_distance = breach_dist.wc_hor + 1.f;
	const float medium_alt_diff = breach_dist.wc_vert + 1.f;
	const float high_distance = breach_dist.nmac_hor + 1.f;
	const float high_alt_diff = breach_dist.nmac_vert + 1.f;
	const float low_distance = breach_dist.aug_nmac_hor + 1.f;
	const float low_alt_diff = breach_dist.aug_nmac_vert + 1.f;

	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(14545057, "DDF0A1", medium_distance, 0.0f, 0.0f,
				medium_alt_diff,
				hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});

	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(7249787, "6E9F7B", low_distance, 0.0f, 0.0f, low_alt_diff,
				hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});

	drain_mavlink_logs();

	// WHEN: The most urgent conflict escalates but stays the most urgent entry.
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(14545057, "DDF0A1", high_distance, 0.0f, 0.0f, high_alt_diff,
				hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});

	conflict_info_s conflict;
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);
	ASSERT_EQ(conflict.unique_id.id, 14545057u);
	ASSERT_EQ(conflict.conflict_level, kDaaConflictLvlHigh);

	// THEN: Only the most-urgent notification is emitted for that update.
	const std::vector<std::string> logs = drain_mavlink_logs();
	EXPECT_EQ(count_logs_with_prefix(logs, "DAA Main:"), 1u);
	EXPECT_EQ(count_logs_with_prefix(logs, "DAA SEC:"), 0u);
}

// WHY: De-escalating the primary conflict should not also emit a secondary-traffic notification for the same update.
// WHAT: Lower the most urgent conflict while another traffic item remains active and verify only the main-status log is emitted.
TEST_F(DetectAndAvoidTest, MostUrgentDeescalationDoesNotSendSecondaryNotification)
{
	// GIVEN: One primary critical conflict and one secondary low conflict are active.
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	breach_distances_s breach_dist;
	get_DFLT_breach_distances(uav_vel, hor_velocity, breach_dist);

	const float critical_distance = breach_dist.nmac_hor - 1.f;
	const float critical_alt_diff = breach_dist.nmac_vert - 1.f;
	const float high_distance = breach_dist.nmac_hor + 1.f;
	const float high_alt_diff = breach_dist.nmac_vert + 1.f;
	const float low_distance = breach_dist.aug_nmac_hor + 1.f;
	const float low_alt_diff = breach_dist.aug_nmac_vert + 1.f;

	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(14545057, "DDF0A1", critical_distance, 0.0f, 0.0f,
				critical_alt_diff,
				hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});

	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(7249787, "6E9F7B", low_distance, 0.0f, 0.0f, low_alt_diff,
				hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});

	drain_mavlink_logs();

	// WHEN: The most urgent conflict de-escalates but stays the most urgent entry.
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(14545057, "DDF0A1", high_distance, 0.0f, 0.0f, high_alt_diff,
				hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});

	conflict_info_s conflict;
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);
	ASSERT_EQ(conflict.unique_id.id, 14545057u);
	ASSERT_EQ(conflict.conflict_level, kDaaConflictLvlHigh);

	// THEN: Only the most-urgent notification is emitted for that update.
	const std::vector<std::string> logs = drain_mavlink_logs();
	EXPECT_EQ(count_logs_with_prefix(logs, "DAA Main:"), 1u);
	EXPECT_EQ(count_logs_with_prefix(logs, "DAA SEC:"), 0u);
}

// WHY: Secondary traffic still needs explicit operator feedback when its conflict resolves.
// WHAT: Resolve a non-primary conflict and verify the secondary solved notification is emitted while the primary conflict remains active.
TEST_F(DetectAndAvoidTest, SecondaryConflictResolutionSendsSecondaryNotification)
{
	// GIVEN: One primary critical conflict and one secondary high conflict are active.
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	breach_distances_s breach_dist;
	get_DFLT_breach_distances(uav_vel, hor_velocity, breach_dist);

	const float critical_distance = breach_dist.nmac_hor - 1.f;
	const float critical_alt_diff = breach_dist.nmac_vert - 1.f;
	const float high_distance = breach_dist.nmac_hor + 1.f;
	const float high_alt_diff = breach_dist.nmac_vert + 1.f;
	const float no_conflict_distance = breach_dist.aug_wc_hor + 1.f;
	const float no_conflict_alt_diff = breach_dist.aug_wc_vert + 1.f;

	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(14545057, "DDF0A1", critical_distance, 0.0f, 0.0f,
				critical_alt_diff,
				hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});

	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(7249787, "6E9F7B", high_distance, 0.0f, 0.0f, high_alt_diff,
				hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});

	drain_mavlink_logs();

	// WHEN: The secondary conflict resolves while the primary conflict remains active.
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(7249787, "6E9F7B", no_conflict_distance, 0.0f, 0.0f,
				no_conflict_alt_diff,
				hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});

	conflict_info_s conflict;
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);
	ASSERT_EQ(conflict.unique_id.id, 14545057u);
	ASSERT_EQ(conflict.conflict_level, kDaaConflictLvlCritical);

	// THEN: The operator receives a single secondary solved notification.
	const std::vector<std::string> logs = drain_mavlink_logs();
	EXPECT_EQ(count_logs_with_prefix(logs, "DAA SEC:"), 1u);
	EXPECT_TRUE(any_log_contains(logs, "solved"));
}

// WHY: Buffer-full conditions can repeat quickly, so ignored-traffic warnings must be rate-limited to stay actionable.
// WHAT: Fill the buffer, inject the same ignored traffic twice in quick succession, and verify only the first attempt logs an ignore warning.
TEST_F(DetectAndAvoidTest, IgnoredTrafficNotificationIsThrottled)
{
	// GIVEN: The conflict buffer is filled with active traffic.
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};
	const float hor_velocity = 20.f;
	const float ver_velocity = 0.f;

	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	breach_distances_s breach_dist;
	get_DFLT_breach_distances(uav_vel, hor_velocity, breach_dist);

	const float buffer_distance = breach_dist.aug_nmac_hor + 1.f;
	const float buffer_alt_diff = breach_dist.aug_nmac_vert + 1.f;
	const float ignored_distance = breach_dist.aug_wc_hor - 1.f;
	const float ignored_alt_diff = breach_dist.aug_wc_vert - 1.f;
	const uint32_t traffic_ids[kDaaMaxTraffic] {14545057, 7249787, 6593425, 3318901, 5207278};

	for (uint8_t i = 0; i < kDaaMaxTraffic; ++i) {
		publish_traffic_and_check([&]() {
			navigator->get_detect_and_avoid()->fake_traffic(traffic_ids[i], "DDF0A1", buffer_distance,
					static_cast<float>(i),
					0.0f, buffer_alt_diff, hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
		});
	}

	drain_mavlink_logs();

	// WHEN: The same ignored traffic is injected twice back to back.
	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(0x123456, "123456", ignored_distance, 0.0f, 0.0f, ignored_alt_diff,
				hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});

	const std::vector<std::string> first_logs = drain_mavlink_logs();
	EXPECT_TRUE(any_log_contains(first_logs, "ignored"));

	publish_traffic_and_check([&]() {
		navigator->get_detect_and_avoid()->fake_traffic(0x123456, "123456", ignored_distance, 0.0f, 0.0f, ignored_alt_diff,
				hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	});

	const std::vector<std::string> second_logs = drain_mavlink_logs();

	// THEN: Only the first ignored insertion emits a warning.
	EXPECT_FALSE(any_log_contains(second_logs, "ignored"));
}

// WHY: The bounded conflict buffer must keep the most important traffic and evict less urgent entries deterministically.
// WHAT: Fill the buffer with ordered priorities, insert better and worse conflicts, then resolve them step by step while checking the chosen top conflict.
TEST_F(DetectAndAvoidTest, BufferFull)
{
	/*
	Test behavior when buffer is full ( max 5 traffic).
	Define traffic with priorities (1,2,...,7).
	The lower the priority the more important the traffic (the closest)

	1. Fill buffer with [6, 5, 4, 3, 2]
	2. New traffic:
		Add traff 7:	--> State remains [6, 5, 4, 3, 2]
		Add traff 0:	--> New state [5, 4, 3, 2, 0]
		Add traff 1:	--> New state [4, 3, 2, 1, 0]
		Resolve traff 0: --> New state: [4, 3, 2, 1]
		Add traff 7:	--> New state: [7, 4, 3, 2, 1]
		Resolve traff 3: --> New state: [7, 4, 2, 1]
		Resolve traff 2: --> New state: [7, 4, 1]
		Resolve traff 1: --> New state: [7, 4]
		Resolve traff 4: --> New state: [7]
		Resolve traff 7: --> New state: []
	*/

	// GIVEN: Ownship is initialized and traffic will be expressed relative to it.
	const double lat_uav = 46.52342;
	const double lon_uav = 6.524234;
	const float alt_uav = 400.f;
	const matrix::Vector3f uav_vel{5.f, 10.f, 2.f};

	// GIVEN: The vehicle is armed, airborne, and in mission mode.
	set_default_uav_state(lat_uav, lon_uav, alt_uav, uav_vel);
	sync_navigator_topics();

	// THEN: Navigator state reflects the published ownship data.
	EXPECT_EQ(navigator->get_local_position()->vx, uav_vel(0));
	EXPECT_EQ(navigator->get_local_position()->vy, uav_vel(1));
	EXPECT_EQ(navigator->get_local_position()->vz, uav_vel(2));
	EXPECT_FALSE(navigator->get_land_detected()->landed);
	EXPECT_TRUE(navigator->get_vstatus()->arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	EXPECT_TRUE(navigator->get_vstatus()->nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
	EXPECT_EQ(navigator->get_global_position()->lat, lat_uav);
	EXPECT_EQ(navigator->get_global_position()->lon, lon_uav);
	EXPECT_EQ(navigator->get_global_position()->alt, alt_uav);

	// GIVEN: Representative traffic parameters for a full buffer-priority exercise.
	const float hor_velocity = 20.f;
	const float ver_velocity = 10000000.f; // Not used because DAA_EN_DFLT_VEL is enabled.

	conflict_info_s conflict;

	// WHEN: No traffic has been processed yet.
	navigator->check_traffic();
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);

	// THEN: The most urgent conflict starts at none.
	EXPECT_TRUE(conflict.conflict_level == detect_and_avoid_s::DAA_CONFLICT_LVL_NONE);

	breach_distances_s breach_dist;
	get_DFLT_breach_distances(uav_vel, hor_velocity, breach_dist);

	// Priority 0, NMAC breach:
	const uint32_t prio_0_icao_96CD13 = 9882899;
	const uint8_t prio_0_conflict_level = detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL;
	const float prio_0_in_nmac_distance = breach_dist.nmac_hor - 5;
	const float prio_0_in_nmac_alt_diff = breach_dist.nmac_vert - 5;
	const float prio_0_in_nmac_overall_dist = sqrtf(prio_0_in_nmac_distance * prio_0_in_nmac_distance +
			prio_0_in_nmac_alt_diff * prio_0_in_nmac_alt_diff);

	// Priority 1, NMAC breach:
	const uint32_t prio_1_icao_A72BC8 = 10955720;
	const uint8_t prio_1_conflict_level = detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL;
	const float prio_1_in_nmac_distance = breach_dist.nmac_hor - 1;
	const float prio_1_in_nmac_alt_diff = breach_dist.nmac_vert - 1;
	const float prio_1_in_nmac_overall_dist = sqrtf(prio_1_in_nmac_distance * prio_1_in_nmac_distance +
			prio_1_in_nmac_alt_diff * prio_1_in_nmac_alt_diff);

	// Priority 2, WC breach at NMAC limit
	const uint32_t prio_2_icao_36A887 = 3582087;
	const uint8_t prio_2_conflict_level = detect_and_avoid_s::DAA_CONFLICT_LVL_HIGH;
	const float prio_2_in_wc_distance = breach_dist.nmac_hor + 1;
	const float prio_2_in_wc_alt_diff =  breach_dist.nmac_vert + 1;
	const float prio_2_in_wc_overall_dist = sqrtf(prio_2_in_wc_distance * prio_2_in_wc_distance +
						prio_2_in_wc_alt_diff * prio_2_in_wc_alt_diff);

	// Priority 3, WC breach at WC limit
	const uint32_t prio_3_icao_32A475 = 3318901;
	const uint8_t prio_3_conflict_level = detect_and_avoid_s::DAA_CONFLICT_LVL_HIGH;
	const float prio_3_in_wc_distance = breach_dist.wc_hor - 1;
	const float prio_3_in_wc_alt_diff = breach_dist.wc_vert - 1;
	const float prio_3_in_wc_overall_dist = sqrtf(prio_3_in_wc_distance * prio_3_in_wc_distance +
						prio_3_in_wc_alt_diff * prio_3_in_wc_alt_diff);

	// Priority 4, Aug NMAC breach at WC limit
	const uint32_t prio_4_icao_BAF2B4 = 12251828;
	const uint8_t prio_4_conflict_level = detect_and_avoid_s::DAA_CONFLICT_LVL_MEDIUM;
	const float prio_4_in_aug_nmac_distance = breach_dist.wc_hor + 1;
	const float prio_4_in_aug_nmac_alt_diff = breach_dist.wc_vert + 1;
	const float prio_4_in_aug_nmac_overall_dist = sqrtf(prio_4_in_aug_nmac_distance * prio_4_in_aug_nmac_distance +
			prio_4_in_aug_nmac_alt_diff * prio_4_in_aug_nmac_alt_diff);

	// Priority 5, Aug NMAC breach at Aug WC limit
	const uint32_t prio_5_icao_4F74EE = 5207278;
	const uint8_t prio_5_conflict_level = detect_and_avoid_s::DAA_CONFLICT_LVL_MEDIUM;
	const float prio_5_in_aug_nmac_distance = breach_dist.aug_nmac_hor - 1;
	const float prio_5_in_aug_nmac_alt_diff = breach_dist.aug_nmac_vert - 1;
	const float prio_5_in_aug_nmac_overall_dist = sqrtf(prio_5_in_aug_nmac_distance * prio_5_in_aug_nmac_distance +
			prio_5_in_aug_nmac_alt_diff * prio_5_in_aug_nmac_alt_diff);

	// Priority 6, Aug WC breach at Aug NMAC limit
	const uint32_t prio_6_icao_2F1BF1 = 3087345;
	const uint8_t prio_6_conflict_level = detect_and_avoid_s::DAA_CONFLICT_LVL_LOW;
	const float prio_6_in_aug_wc_distance = breach_dist.aug_nmac_hor + 1;
	const float prio_6_in_aug_wc_alt_diff = breach_dist.aug_nmac_vert + 1;
	const float prio_6_in_aug_wc_overall_dist = sqrtf(prio_6_in_aug_wc_distance * prio_6_in_aug_wc_distance +
			prio_6_in_aug_wc_alt_diff * prio_6_in_aug_wc_alt_diff);

	// Priority 7, Aug WC breach at No conflict limit
	const uint32_t prio_7_icao_6F0C1B = 7277595;
	const uint8_t prio_7_conflict_level = detect_and_avoid_s::DAA_CONFLICT_LVL_LOW;
	const float prio_7_in_aug_wc_distance = breach_dist.aug_wc_hor - 1;
	const float prio_7_in_aug_wc_alt_diff = breach_dist.aug_wc_vert - 1;
	const float prio_7_in_aug_wc_overall_dist = sqrtf(prio_7_in_aug_wc_distance * prio_7_in_aug_wc_distance +
			prio_7_in_aug_wc_alt_diff * prio_7_in_aug_wc_alt_diff);

	// Priority 8, No conflict
	// const uint8_t prio_8_conflict_level = detect_and_avoid_s::DAA_CONFLICT_LVL_NONE;
	const float prio_8_no_conflict_distance = breach_dist.aug_wc_hor + 1;
	const float prio_8_no_conflict_alt_diff = breach_dist.aug_wc_vert + 1;

	// WHEN: The buffer is filled with priorities 6, 5, 4, 3, 2.
	PX4_DEBUG("---- F_TEST DAA: Prio 6, 2F1BF1 Aug WC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(prio_6_icao_2F1BF1, "2F1BF1", prio_6_in_aug_wc_distance, 0.0f, 0.0f,
			prio_6_in_aug_wc_alt_diff,
			hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	navigator->check_traffic();
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);
	EXPECT_TRUE(conflict.conflict_level == prio_6_conflict_level);
	EXPECT_EQ(conflict.aircraft_dist, prio_6_in_aug_wc_overall_dist);
	EXPECT_EQ(conflict.unique_id.id, prio_6_icao_2F1BF1);

	check_highest_conflict(conflict.conflict_level,
			       navigator->get_detect_and_avoid()->conflict_lvl_requires_action(conflict.conflict_level));

	PX4_DEBUG("---- F_TEST DAA: Prio 5, 4F74EE Aug NMAC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(prio_5_icao_4F74EE, "4F74EE", prio_5_in_aug_nmac_distance, 0.0f, 0.0f,
			prio_5_in_aug_nmac_alt_diff,
			hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	navigator->check_traffic();
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);
	EXPECT_TRUE(conflict.conflict_level == prio_5_conflict_level);
	EXPECT_EQ(conflict.aircraft_dist, prio_5_in_aug_nmac_overall_dist);
	EXPECT_EQ(conflict.unique_id.id, prio_5_icao_4F74EE);

	check_highest_conflict(conflict.conflict_level,
			       navigator->get_detect_and_avoid()->conflict_lvl_requires_action(conflict.conflict_level));

	PX4_DEBUG("---- F_TEST DAA: Prio 4, BAF2B4 Aug NMAC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(prio_4_icao_BAF2B4, "BAF2B4", prio_4_in_aug_nmac_distance, 0.0f, 0.0f,
			prio_4_in_aug_nmac_alt_diff,
			hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	navigator->check_traffic();
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);
	EXPECT_TRUE(conflict.conflict_level == prio_4_conflict_level);
	EXPECT_EQ(conflict.aircraft_dist, prio_4_in_aug_nmac_overall_dist);
	EXPECT_EQ(conflict.unique_id.id, prio_4_icao_BAF2B4);

	check_highest_conflict(conflict.conflict_level,
			       navigator->get_detect_and_avoid()->conflict_lvl_requires_action(conflict.conflict_level));

	PX4_DEBUG("---- F_TEST DAA: Prio 3, 32A475 WC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(prio_3_icao_32A475, "32A475", prio_3_in_wc_distance, 0.0f, 0.0f,
			prio_3_in_wc_alt_diff,
			hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	navigator->check_traffic();
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);
	EXPECT_TRUE(conflict.conflict_level == prio_3_conflict_level);
	EXPECT_EQ(conflict.aircraft_dist, prio_3_in_wc_overall_dist);
	EXPECT_EQ(conflict.unique_id.id, prio_3_icao_32A475);

	check_highest_conflict(conflict.conflict_level,
			       navigator->get_detect_and_avoid()->conflict_lvl_requires_action(conflict.conflict_level));

	PX4_DEBUG("---- F_TEST DAA: Prio 2, 36A887 WC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(prio_2_icao_36A887, "36A887", prio_2_in_wc_distance, 0.0f, 0.0f,
			prio_2_in_wc_alt_diff,
			hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	navigator->check_traffic();
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);
	EXPECT_TRUE(conflict.conflict_level == prio_2_conflict_level);
	EXPECT_EQ(conflict.aircraft_dist, prio_2_in_wc_overall_dist);
	EXPECT_EQ(conflict.unique_id.id, prio_2_icao_36A887);

	check_highest_conflict(conflict.conflict_level,
			       navigator->get_detect_and_avoid()->conflict_lvl_requires_action(conflict.conflict_level));

	// THEN: Priority 2 is the most urgent conflict before overflow cases are exercised.

	// WHEN: A lower-priority conflict is inserted into the full buffer.
	PX4_DEBUG("---- F_TEST DAA: Prio 7, 6F0C1B Aug WC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(prio_7_icao_6F0C1B, "6F0C1B", prio_7_in_aug_wc_distance, 0.0f, 0.0f,
			prio_7_in_aug_wc_alt_diff,
			hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	navigator->check_traffic();
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);

	// THEN: The existing buffer contents are preserved because the new traffic is less urgent.
	// Buffer [6, 5, 4, 3, 2]
	EXPECT_TRUE(conflict.conflict_level == prio_2_conflict_level);
	EXPECT_EQ(conflict.aircraft_dist, prio_2_in_wc_overall_dist);
	EXPECT_EQ(conflict.unique_id.id, prio_2_icao_36A887);

	check_highest_conflict(conflict.conflict_level,
			       navigator->get_detect_and_avoid()->conflict_lvl_requires_action(conflict.conflict_level));

	// WHEN: A new highest-priority conflict arrives.
	PX4_DEBUG("---- F_TEST DAA: Prio 0, 96CD13 NMAC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(prio_0_icao_96CD13, "96CD13", prio_0_in_nmac_distance, 0.0f, 0.0f,
			prio_0_in_nmac_alt_diff,
			hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	navigator->check_traffic();
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);

	// THEN: The weakest buffered conflict is removed and priority 0 becomes most urgent.
	// Buffer: [5, 4, 3, 2, 0]
	EXPECT_TRUE(conflict.conflict_level == prio_0_conflict_level);
	EXPECT_EQ(conflict.aircraft_dist, prio_0_in_nmac_overall_dist);
	EXPECT_EQ(conflict.unique_id.id, prio_0_icao_96CD13);

	check_highest_conflict(conflict.conflict_level,
			       navigator->get_detect_and_avoid()->conflict_lvl_requires_action(conflict.conflict_level));

	// WHEN: A second high-priority conflict is inserted.
	PX4_DEBUG("---- F_TEST DAA: Prio 1, A72BC8 NMAC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(prio_1_icao_A72BC8, "A72BC8", prio_1_in_nmac_distance, 0.0f, 0.0f,
			prio_1_in_nmac_alt_diff,
			hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	navigator->check_traffic();
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);

	// THEN: Priority 0 remains most urgent while the buffer reorders around it.
	// Buffer: [4, 3, 2, 1, 0]
	EXPECT_TRUE(conflict.conflict_level == prio_0_conflict_level);
	EXPECT_EQ(conflict.aircraft_dist, prio_0_in_nmac_overall_dist);
	EXPECT_EQ(conflict.unique_id.id, prio_0_icao_96CD13);

	check_highest_conflict(conflict.conflict_level,
			       navigator->get_detect_and_avoid()->conflict_lvl_requires_action(conflict.conflict_level));

	// WHEN: The top-priority conflict resolves.
	PX4_DEBUG("---- F_TEST DAA: Prio 0, 96CD13 resolve");
	navigator->get_detect_and_avoid()->fake_traffic(prio_0_icao_96CD13, "96CD13", prio_8_no_conflict_distance, 0.0f, 0.0f,
			prio_8_no_conflict_alt_diff,
			hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	navigator->check_traffic();
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);

	// THEN: Priority 1 becomes the new most urgent conflict.
	// Buffer: [4, 3, 2, 1]
	EXPECT_TRUE(conflict.conflict_level == prio_1_conflict_level);
	EXPECT_EQ(conflict.aircraft_dist, prio_1_in_nmac_overall_dist);
	EXPECT_EQ(conflict.unique_id.id, prio_1_icao_A72BC8);

	check_highest_conflict(conflict.conflict_level,
			       navigator->get_detect_and_avoid()->conflict_lvl_requires_action(conflict.conflict_level));

	// WHEN: The buffer has room again and a low-priority conflict is added back.
	PX4_DEBUG("---- F_TEST DAA: Prio 7, 6F0C1B Aug WC conflict");
	navigator->get_detect_and_avoid()->fake_traffic(prio_7_icao_6F0C1B, "6F0C1B", prio_7_in_aug_wc_distance, 0.0f, 0.0f,
			prio_7_in_aug_wc_alt_diff,
			hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	navigator->check_traffic();
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);

	// THEN: The added low-priority traffic does not affect the top conflict.
	// Buffer: [7, 4, 3, 2, 1]
	EXPECT_TRUE(conflict.conflict_level == prio_1_conflict_level);
	EXPECT_EQ(conflict.aircraft_dist, prio_1_in_nmac_overall_dist);
	EXPECT_EQ(conflict.unique_id.id, prio_1_icao_A72BC8);

	check_highest_conflict(conflict.conflict_level,
			       navigator->get_detect_and_avoid()->conflict_lvl_requires_action(conflict.conflict_level));

	// WHEN: Mid-priority conflicts resolve one by one.
	PX4_DEBUG("---- F_TEST DAA: Prio 3, 32A475 resolve");
	navigator->get_detect_and_avoid()->fake_traffic(prio_3_icao_32A475, "32A475", prio_8_no_conflict_distance, 0.0f, 0.0f,
			prio_8_no_conflict_alt_diff,
			hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	navigator->check_traffic();
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);
	// Buffer: [7, 4, 2, 1]
	EXPECT_TRUE(conflict.conflict_level == prio_1_conflict_level);
	EXPECT_EQ(conflict.aircraft_dist, prio_1_in_nmac_overall_dist);
	EXPECT_EQ(conflict.unique_id.id, prio_1_icao_A72BC8);

	check_highest_conflict(conflict.conflict_level,
			       navigator->get_detect_and_avoid()->conflict_lvl_requires_action(conflict.conflict_level));

	PX4_DEBUG("---- F_TEST DAA: Prio 2, 36A887 resolve");
	navigator->get_detect_and_avoid()->fake_traffic(prio_2_icao_36A887, "36A887", prio_8_no_conflict_distance, 0.0f, 0.0f,
			prio_8_no_conflict_alt_diff,
			hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	navigator->check_traffic();
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);
	// Buffer: [7, 4, 1]
	EXPECT_TRUE(conflict.conflict_level == prio_1_conflict_level);
	EXPECT_EQ(conflict.aircraft_dist, prio_1_in_nmac_overall_dist);
	EXPECT_EQ(conflict.unique_id.id, prio_1_icao_A72BC8);

	check_highest_conflict(conflict.conflict_level,
			       navigator->get_detect_and_avoid()->conflict_lvl_requires_action(conflict.conflict_level));

	PX4_DEBUG("---- F_TEST DAA: Prio 1, A72BC8 resolve");
	navigator->get_detect_and_avoid()->fake_traffic(prio_1_icao_A72BC8, "A72BC8", prio_8_no_conflict_distance, 0.0f, 0.0f,
			prio_8_no_conflict_alt_diff,
			hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	navigator->check_traffic();
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);
	// Buffer: [7, 4]
	EXPECT_TRUE(conflict.conflict_level == prio_4_conflict_level);
	EXPECT_EQ(conflict.aircraft_dist, prio_4_in_aug_nmac_overall_dist);
	EXPECT_EQ(conflict.unique_id.id, prio_4_icao_BAF2B4);

	check_highest_conflict(conflict.conflict_level,
			       navigator->get_detect_and_avoid()->conflict_lvl_requires_action(conflict.conflict_level));

	PX4_DEBUG("---- F_TEST DAA: Prio 4, BAF2B4 resolve");
	navigator->get_detect_and_avoid()->fake_traffic(prio_4_icao_BAF2B4, "BAF2B4", prio_8_no_conflict_distance, 0.0f, 0.0f,
			prio_8_no_conflict_alt_diff,
			hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	navigator->check_traffic();
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);
	// Buffer: [7]
	EXPECT_TRUE(conflict.conflict_level == prio_7_conflict_level);
	EXPECT_EQ(conflict.aircraft_dist, prio_7_in_aug_wc_overall_dist);
	EXPECT_EQ(conflict.unique_id.id, prio_7_icao_6F0C1B);

	check_highest_conflict(conflict.conflict_level,
			       navigator->get_detect_and_avoid()->conflict_lvl_requires_action(conflict.conflict_level));

	PX4_DEBUG("---- F_TEST DAA: Prio 7, 6F0C1B resolve");
	navigator->get_detect_and_avoid()->fake_traffic(prio_7_icao_6F0C1B, "6F0C1B", prio_8_no_conflict_distance, 0.0f, 0.0f,
			prio_8_no_conflict_alt_diff,
			hor_velocity, ver_velocity, 1, lat_uav, lon_uav, alt_uav);
	navigator->check_traffic();
	navigator->get_detect_and_avoid()->get_most_urgent_conflict(conflict);

	// THEN: The buffer returns to an empty no-conflict state.
	// Buffer: []
	EXPECT_TRUE(conflict.conflict_level == detect_and_avoid_s::DAA_CONFLICT_LVL_NONE);

	check_highest_conflict(conflict.conflict_level,
			       navigator->get_detect_and_avoid()->conflict_lvl_requires_action(conflict.conflict_level));
}

// WHY: Ownship reports must never enter the avoidance buffer regardless of whether identity comes from ICAO, callsign, or UAS ID.
// WHAT: Configure each self identifier format and verify DetectAndAvoid recognizes and suppresses matching traffic.
TEST_F(DetectAndAvoidTest, SelfDetection)
{
	/*
		Method used to set call sign parameters:
			1. Start with callsign
			2. Convert to uint64:
				callsign_to_uint64(const char callsign[kCallsignLength]);
			3. Convert to two uint32
				const uint32_t lower = static_cast<uint32_t>(unique_id.id & 0xFFFFFFFF);
				const uint32_t upper = static_cast<uint32_t>((unique_id.id >> 32) & 0xFFFFFFFF);

		Method used to set ICAO parameters:
			uint32_t icao_uint = static_cast<uint32_t>(std::stoul(icao_str, nullptr, 16));

	*/

	// GIVEN: DetectAndAvoid is active with default parameters.
	EXPECT_TRUE(navigator->get_detect_and_avoid()->is_activated());

	// GIVEN: Random ADS-B callsigns will be configured as ownship identifiers.
	char callsign[9];
	transponder_report_s tr{};
	unique_id_s unique_id;

	// WHEN: Callsign identifiers are written into the self-identification parameters.
	for (int i = 0; i < 30; ++i) {
		generate_random_callsign(callsign, 8);
		const uint64_t callsign_64 = navigator->get_detect_and_avoid()->callsign_to_uint64(callsign);
		const uint32_t own_adsb_callsign1 = static_cast<uint32_t>(callsign_64 & 0xFFFFFFFF);
		const uint32_t own_adsb_callsign2 = static_cast<uint32_t>((callsign_64 >> 32) & 0xFFFFFFFF);
		param_set(param_handle(px4::params::ADSB_CALLSIGN_1), &own_adsb_callsign1);
		param_set(param_handle(px4::params::ADSB_CALLSIGN_2), &own_adsb_callsign2);

		navigator->get_detect_and_avoid()->updateParams();

		tr.icao_address = 0; // Invalid ICAO
		strncpy(&tr.callsign[0], callsign, sizeof(tr.callsign) - 1);
		tr.callsign[sizeof(tr.callsign) - 1] = 0;
		tr.flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN;

		navigator->get_detect_and_avoid()->get_unique_id(tr, unique_id);

		// THEN: Matching callsigns are recognized as self-detections.
		EXPECT_TRUE(unique_id.encoding == detect_and_avoid_s::UNIQUE_ID_ENCODING_ADSB_CALLSIGN);
		EXPECT_TRUE(unique_id.id == callsign_64);
		EXPECT_TRUE(navigator->get_detect_and_avoid()->is_self_detection(unique_id));
	}

	// WHEN: The primary ICAO address is configured as ownship.
	const uint32_t own_icao = 6593425;
	tr.icao_address = own_icao;
	navigator->get_detect_and_avoid()->get_unique_id(tr, unique_id);

	// THEN: With the default disabled primary ICAO parameter, real traffic is not treated as ownship.
	EXPECT_FALSE(navigator->get_detect_and_avoid()->is_self_detection(unique_id));

	param_set(param_handle(px4::params::ADSB_ICAO_ID), &own_icao);
	navigator->get_detect_and_avoid()->updateParams();

	tr.icao_address = own_icao;
	strncpy(&tr.callsign[0], callsign, sizeof(tr.callsign) - 1);
	tr.callsign[sizeof(tr.callsign) - 1] = 0;
	tr.flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN;

	navigator->get_detect_and_avoid()->get_unique_id(tr, unique_id);

	// THEN: Matching primary ICAO traffic is treated as self-detection.
	EXPECT_TRUE(unique_id.encoding == detect_and_avoid_s::UNIQUE_ID_ENCODING_ICAO);
	EXPECT_TRUE(unique_id.id == own_icao);
	EXPECT_TRUE(navigator->get_detect_and_avoid()->is_self_detection(unique_id));

	// WHEN: The secondary ICAO address is configured as ownship.
	const uint32_t own_icao_2 = 3318901;
	param_set(param_handle(px4::params::ADSB_ICAO_ID_2), &own_icao_2);
	navigator->get_detect_and_avoid()->updateParams();

	tr.icao_address = own_icao_2;
	strncpy(&tr.callsign[0], callsign, sizeof(tr.callsign) - 1);
	tr.callsign[sizeof(tr.callsign) - 1] = 0;
	tr.flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN;

	navigator->get_detect_and_avoid()->get_unique_id(tr, unique_id);

	// THEN: Matching secondary ICAO traffic is also treated as self-detection.
	EXPECT_TRUE(unique_id.encoding == detect_and_avoid_s::UNIQUE_ID_ENCODING_ICAO);
	EXPECT_TRUE(unique_id.id == own_icao_2);
	EXPECT_TRUE(navigator->get_detect_and_avoid()->is_self_detection(unique_id));

	// WHEN: The traffic report carries only a UAS ID.
	transponder_report_s tr_uas_id{};
	tr_uas_id.icao_address = 0;
	memset(tr_uas_id.callsign, 0, sizeof(tr_uas_id.callsign));

#ifndef BOARD_HAS_NO_UUID
	px4_guid_t px4_guid {};
	board_get_px4_guid(px4_guid);
	memcpy(tr_uas_id.uas_id, px4_guid, sizeof(px4_guid));
	EXPECT_TRUE(navigator->get_detect_and_avoid()->get_unique_id(tr_uas_id, unique_id));

	// THEN: On boards with a UUID, the local UAS ID is recognized as self.
	EXPECT_TRUE(unique_id.encoding == detect_and_avoid_s::UNIQUE_ID_ENCODING_UAS_ID);
	EXPECT_TRUE(unique_id.id == navigator->get_detect_and_avoid()->last_uas_id_bytes_to_uint64(tr_uas_id.uas_id));
	EXPECT_TRUE(navigator->get_detect_and_avoid()->is_self_detection(unique_id));
#else

	for (int i = 0; i < PX4_GUID_BYTE_LENGTH; ++i) {
		tr_uas_id.uas_id[i] = 0xe0 + i;
	}

	EXPECT_TRUE(navigator->get_detect_and_avoid()->get_unique_id(tr_uas_id, unique_id));

	// THEN: Without a board UUID, an arbitrary UAS ID is not considered self.
	EXPECT_TRUE(unique_id.encoding == detect_and_avoid_s::UNIQUE_ID_ENCODING_UAS_ID);
	EXPECT_FALSE(navigator->get_detect_and_avoid()->is_self_detection(unique_id));
#endif

	// WHEN: Traffic provides no usable identity fields.
	transponder_report_s tr_no_id{};
	tr_no_id.icao_address = 0;
	strncpy(&tr_no_id.callsign[0], callsign, sizeof(tr_no_id.callsign) - 1);
	tr_no_id.callsign[sizeof(tr.callsign) - 1] = 0;
	tr_no_id.flags = 0;

	// THEN: No unique identifier can be extracted.
	EXPECT_FALSE(navigator->get_detect_and_avoid()->get_unique_id(tr_no_id, unique_id));
}

// WHY: Callsign packing and unpacking must round-trip cleanly so buffer keys and operator-facing messages stay consistent.
// WHAT: Convert random callsigns to uint64 and back, then verify the recovered string produces the same encoded value and reject unterminated input.
TEST_F(DetectAndAvoidTest, ReversibleCallsign)
{
	/*
		Ensure conversion is reversible
			Callsign:
				- uint = callsign_to_uint64
				- recovered = convert_uint64_callsign_to_str(uint)
				- back_to_uint = callsign_to_uint64(recovered)

				--> uint == back_to_uint
	*/

	// GIVEN: DetectAndAvoid is active and the callsign helpers are available.
	EXPECT_TRUE(navigator->get_detect_and_avoid()->is_activated());

	char callsign[9];

	// WHEN: Random null-terminated callsigns are converted to uint64 and back.
	for (int i = 0; i < 30; ++i) {
		generate_random_callsign(callsign, 8);
		const uint64_t callsign_uint = navigator->get_detect_and_avoid()->callsign_to_uint64(callsign);
		char recovered[9];
		navigator->get_detect_and_avoid()->convert_uint64_callsign_to_str(callsign_uint, recovered);
		const uint64_t final_uint = navigator->get_detect_and_avoid()->callsign_to_uint64(recovered);

		// THEN: The encoding round-trips without changing the packed value.
		EXPECT_TRUE(callsign_uint == final_uint);

		// Only debug print once
		if (i == 0) {

			PX4_DEBUG("TEST_F: UniqueID, init: %s (int %lu), recovered: %s (int %lu)", callsign, callsign_uint, recovered,
				  final_uint);

			char part1_recovered[9];
			char part2_recovered[9];
			uint32_t lower = static_cast<uint32_t>(callsign_uint & 0xFFFFFFFF);
			uint32_t upper = static_cast<uint32_t>((callsign_uint >> 32) & 0xFFFFFFFF);
			navigator->get_detect_and_avoid()->convert_uint64_callsign_to_str(uint64_t(lower), part1_recovered);
			navigator->get_detect_and_avoid()->convert_uint64_callsign_to_str(uint64_t(upper), part2_recovered);
			PX4_DEBUG("TEST_F: full: %s =? %s---%s == PART1---PART2", recovered, part1_recovered, part2_recovered);
		}
	}

	// WHEN: A callsign is not null terminated.
	char callsign_non_null[9];
	generate_random_callsign(callsign_non_null, 8);
	callsign_non_null[8] = 1;

	// THEN: The helper rejects the malformed input.
	EXPECT_TRUE(navigator->get_detect_and_avoid()->callsign_to_uint64(callsign_non_null) == 0);
}

// WHY: Callsigns are stored as bytes, so high-bit characters must only affect their own byte slot and never sign-extend across the uint64_t key.
// WHAT: Encode a callsign containing bytes with the MSB set and verify the packed value preserves only those byte values.
TEST_F(DetectAndAvoidTest, CallsignPackingDoesNotSignExtend)
{
	const char callsign[kCallsignLength] {static_cast<char>(0x80), static_cast<char>(0xFF), '\0'};
	const uint64_t callsign_uint = navigator->get_detect_and_avoid()->callsign_to_uint64(callsign);

	EXPECT_EQ(callsign_uint, static_cast<uint64_t>(0xFF80u));
}

// WHY: MAVLink fixed-length string fields copy every byte, so unused callsign bytes must be deterministic after decoding.
// WHAT: Decode a short callsign into a pre-filled buffer and verify all bytes after the terminator are cleared.
TEST_F(DetectAndAvoidTest, CallsignDecodeClearsUnusedBytes)
{
	const char callsign[kCallsignLength] {'A', 'B', '\0'};
	const uint64_t callsign_uint = navigator->get_detect_and_avoid()->callsign_to_uint64(callsign);
	char recovered[kCallsignLength];
	memset(recovered, 0x7F, sizeof(recovered));

	navigator->get_detect_and_avoid()->convert_uint64_callsign_to_str(callsign_uint, recovered);

	EXPECT_STREQ(recovered, "AB");

	for (int i = 2; i < kCallsignLength; ++i) {
		EXPECT_EQ(recovered[i], '\0');
	}
}

// WHY: ICAO identifiers are 24-bit values, so operator-facing formatting must stay fixed-width and avoid platform-specific truncation.
// WHAT: Format representative ICAO values and verify the output is uppercase, zero-padded, and derived from the low 24 bits.
TEST_F(DetectAndAvoidTest, FormatsIcaoAsFixedWidthHex)
{
	// GIVEN: A buffer sized for a 6-character ICAO address plus the null terminator.
	char icao_buffer[kIcaoLength] {};

	// WHEN: A short ICAO value is formatted.
	DetectAndAvoid::convert_icao_uint32_to_hex_str(0xABCu, icao_buffer, sizeof(icao_buffer));

	// THEN: The helper zero-pads and uppercases the address.
	EXPECT_STREQ(icao_buffer, "000ABC");

	// WHEN: High bits are present in the input value.
	DetectAndAvoid::convert_icao_uint32_to_hex_str(0x12ABCDEFu, icao_buffer, sizeof(icao_buffer));

	// THEN: Only the ICAO address bits are rendered.
	EXPECT_STREQ(icao_buffer, "ABCDEF");
}

// WHY: Reduced UAS-ID packing must preserve the exact GUID tail bytes used as the unique identifier.
// WHAT: Round-trip random and zero-valued GUID tails through the uint64 conversion helpers and compare every byte.
TEST_F(DetectAndAvoidTest, ReversibleUasId)
{
	/*
		Ensure conversion is reversible
			UAS_ID:
				- uint = last_uas_id_bytes_to_uint64(uas_id);
				- recovered = uint64_to_last_uas_id_bytes(uint);

				--> last_uas_id_bytes == recovered_bytes
	*/

	// GIVEN: DetectAndAvoid is active and reduced UAS-ID packing is available.
	EXPECT_TRUE(navigator->get_detect_and_avoid()->is_activated());

	uint8_t uas_id[PX4_GUID_BYTE_LENGTH];

	// WHEN: Random UAS IDs are packed into uint64 and unpacked again.
	for (int i = 0; i < 30; ++i) {
		generate_random_UAS_ID(uas_id);
		const uint64_t uas_id_uint = navigator->get_detect_and_avoid()->last_uas_id_bytes_to_uint64(
						     uas_id); // Converts last kIdEncodingNbBytes bytes
		uint8_t recovered[kIdEncodingNbBytes]; // Gets the last kIdEncodingNbBytes back
		navigator->get_detect_and_avoid()->uint64_to_last_uas_id_bytes(uas_id_uint, recovered);

		// THEN: The packed representation preserves the trailing GUID bytes exactly.
		for (int k = 0; k < kIdEncodingNbBytes; ++k) {
			EXPECT_TRUE(uas_id[PX4_GUID_BYTE_LENGTH - kIdEncodingNbBytes + k] == recovered[k]);

			if (i == 0) {
				PX4_DEBUG(" Byte %d/%d: uas == %d == %d == recovered", PX4_GUID_BYTE_LENGTH - kIdEncodingNbBytes + k + 1,
					  PX4_GUID_BYTE_LENGTH, uas_id[PX4_GUID_BYTE_LENGTH - kIdEncodingNbBytes + k], recovered[k]);
			}

		}
	}

	// WHEN: The UAS ID is all zeros.
	uint8_t zero_uas_id[PX4_GUID_BYTE_LENGTH];

	for (int i = 0; i < PX4_GUID_BYTE_LENGTH; i++) {
		zero_uas_id[i] = 0;
	}

	const uint64_t zero_uas_id_uint = navigator->get_detect_and_avoid()->last_uas_id_bytes_to_uint64(
			zero_uas_id); // Converts last kIdEncodingNbBytes bytes

	ASSERT_TRUE(zero_uas_id_uint == 0);

	// WHEN: The zero value is unpacked again.
	uint8_t zero_recovered[kIdEncodingNbBytes];
	navigator->get_detect_and_avoid()->uint64_to_last_uas_id_bytes(zero_uas_id_uint, zero_recovered);

	// THEN: The unpacked trailing bytes remain zeroed.
	for (int k = 0; k < kIdEncodingNbBytes; ++k) {
		EXPECT_TRUE(0 == zero_recovered[k]);
	}
}

// WHY: Automatic DAA actions must only escalate when they are stronger than the current navigator state permits.
// WHAT: Evaluate each requested action across representative navigation states and verify the expected escalation decisions.
TEST_P(DetectAndAvoidActionPriorityTest, ActionPriorities)
{
	EXPECT_TRUE(navigator->get_detect_and_avoid()->is_activated());

	const action_priority_test_case_s &test_case = GetParam();
	check_eval_conflict_escalation_action(navigator.get(), test_case.expected_outcome, test_case.action);
}

INSTANTIATE_TEST_SUITE_P(
	AllActions,
	DetectAndAvoidActionPriorityTest,
	::testing::Values(
		action_priority_test_case_s{"Disabled", DaaAction::kDisabled, {false, false, false, false}},
		action_priority_test_case_s{"WarnOnly", DaaAction::kWarnOnly, {false, false, false, false}},
		action_priority_test_case_s{"Hold", DaaAction::kPositionHoldMode, {true, false, false, false}},
		action_priority_test_case_s{"ReturnMode", DaaAction::kReturnMode, {true, true, false, false}},
		action_priority_test_case_s{"LandMode", DaaAction::kLandMode, {true, true, true, false}},
		action_priority_test_case_s{"Terminate", DaaAction::kTerminate, {true, true, true, true}}),
	action_priority_test_case_name);
