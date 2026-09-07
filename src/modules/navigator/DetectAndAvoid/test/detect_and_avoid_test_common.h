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

/**
 * @file detect_and_avoid_test_common.h
 * @brief Shared fixture and helpers for the DetectAndAvoid functional tests.
 *
 * @author Jonas Perolini <jonspero@me.com>
 *
 */

#pragma once

#include <gtest/gtest.h>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "detect_and_avoid.h"
#include "navigator.h"
#include "mission.h"
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>
#include <uORB/topics/mavlink_log.h>

using namespace time_literals;

extern "C" __EXPORT int dataman_main(int argc, char *argv[]);

static constexpr uint8_t kDaaConflictLvlNone = 0;
static constexpr uint8_t kDaaConflictLvlLow = 1;
static constexpr uint8_t kDaaConflictLvlMedium = 2;
static constexpr uint8_t kDaaConflictLvlHigh = 3;
static constexpr uint8_t kDaaConflictLvlCritical = 4;

#if defined(CONFIG_NAVIGATOR_ADSB_F3442) && CONFIG_NAVIGATOR_ADSB_F3442
/* F3442 default volumes. Set simple bounds */
static constexpr float kDfltDaaLvlCriticalRad = 150.f;
static constexpr float kDfltDaaLvlCriticalHgt = 30.f;
static constexpr float kDfltDaaLvlHighRad = 600.f;
static constexpr float kDfltDaaLvlHighHgt = 80.f;
static constexpr int kDfltDaaLvlMediumTime = 30;
static constexpr int kDfltDaaLvlLowTime = 30;

static constexpr float kDfltDaaDfltVel = 5.f;
static constexpr int kDfltDaaEnDfltVel = 1;
#else
/* Crosstrack default gates. These match the NAV_TRAFF_* parameter defaults. */
static constexpr float kDfltNavTraffHorSep = 500.f;
static constexpr float kDfltNavTraffVerSep = 500.f;
static constexpr int kDfltNavTraffCollTime = 60;
#endif // CONFIG_NAVIGATOR_ADSB_F3442

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

static inline NavigatorDatamanRuntime &navigatorDatamanRuntime()
{
	static NavigatorDatamanRuntime runtime{};
	return runtime;
}

// To run: make tests TESTFILTER=detect_and_avoid
class DetectAndAvoidTest : public ::testing::Test
{

protected:

	void SetUp() override
	{
		navigatorDatamanRuntime();
		param_reset_all();
		set_DFLT_daa_params();
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

#if defined(CONFIG_NAVIGATOR_ADSB_F3442) && CONFIG_NAVIGATOR_ADSB_F3442
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

	breach_distances_s get_DFLT_breach_distances(const matrix::Vector3f uav_vel, const float traffic_hor_vel)
	{
		breach_distances_s breach_distance{};

		const float uav_hor_vel = uav_vel.xy().norm();

		/* NMAC */
		breach_distance.nmac_hor = 2 * kDfltDaaLvlCriticalRad;
		breach_distance.nmac_vert = 2 * kDfltDaaLvlCriticalHgt;

		/* WC */
		breach_distance.wc_hor = 2 * kDfltDaaLvlHighRad;
		breach_distance.wc_vert = 2 * kDfltDaaLvlHighHgt;

		/* Aug NMAC */
		breach_distance.aug_nmac_hor = 2 * kDfltDaaLvlCriticalRad + kDfltDaaLvlMediumTime * (fabsf(
						       traffic_hor_vel) + fabsf(uav_hor_vel));
		breach_distance.aug_nmac_vert = 2 * kDfltDaaLvlCriticalHgt + kDfltDaaLvlMediumTime * (fabsf(
							kDfltDaaDfltVel) + fabsf(uav_vel(2)));

		/* Aug WC */
		breach_distance.aug_wc_hor = 2 * kDfltDaaLvlHighRad + kDfltDaaLvlLowTime * (fabsf(
						     traffic_hor_vel) + fabsf(uav_hor_vel));
		breach_distance.aug_wc_vert = 2 * kDfltDaaLvlHighHgt + kDfltDaaLvlLowTime * (fabsf(
						      kDfltDaaDfltVel) + fabsf(uav_vel(2)));
		return breach_distance;
	}
#endif // CONFIG_NAVIGATOR_ADSB_F3442

	void publish_local_pos_vel(const matrix::Vector3f local_pos_vel, const hrt_abstime timestamp = hrt_absolute_time(),
				   const float heading = 0.f, const bool v_xy_valid = true, const bool v_z_valid = true)
	{
		vehicle_local_position_s l_pos{};
		l_pos.timestamp_sample = timestamp;
		l_pos.timestamp = timestamp;
		l_pos.v_xy_valid = v_xy_valid;
		l_pos.v_z_valid = v_z_valid;
		l_pos.vx = local_pos_vel(0);
		l_pos.vy = local_pos_vel(1);
		l_pos.vz = local_pos_vel(2);
		l_pos.heading = heading;
		_vehicle_local_position_pub.publish(l_pos);
	}

	void publish_global_pos(const double lat, const double lon, const double alt,
				const hrt_abstime timestamp = hrt_absolute_time(), const bool lat_lon_valid = true,
				const bool alt_valid = true)
	{
		vehicle_global_position_s g_pos{};
		g_pos.timestamp_sample = timestamp;
		g_pos.lat_lon_valid = lat_lon_valid;
		g_pos.alt_valid = alt_valid;
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

	// Force the DAA to reload its cached parameters immediately.
	// on_activation() calls ModuleParams::updateParams()
	void reload_daa_parameters()
	{
		navigator->get_detect_and_avoid()->on_activation();
	}

	conflict_info_s expect_most_urgent_conflict(uint8_t expected_level, uint64_t expected_id)
	{
		const conflict_info_s conflict = navigator->get_detect_and_avoid()->get_most_urgent_conflict();
		SCOPED_TRACE(testing::Message() << "most-urgent conflict: expected level " << static_cast<int>(expected_level)
			     << ", id " << expected_id);
		EXPECT_EQ(conflict.conflict_level, expected_level);
		EXPECT_EQ(conflict.encoded_id.id, expected_id);
		return conflict;
	}

	struct TestSyntheticTrafficReport : public DetectAndAvoid::SyntheticTrafficReport {
		TestSyntheticTrafficReport(uint32_t icao, const char *report_callsign, float relative_distance)
		{
			icao_address = icao;
			callsign = report_callsign;
			distance = relative_distance;
		}

		TestSyntheticTrafficReport with_direction(float relative_direction) const
		{
			TestSyntheticTrafficReport report{*this};
			report.direction = relative_direction;
			return report;
		}

		TestSyntheticTrafficReport with_heading(float heading) const
		{
			TestSyntheticTrafficReport report{*this};
			report.traffic_heading = heading;
			return report;
		}

		TestSyntheticTrafficReport with_altitude_diff(float relative_altitude) const
		{
			TestSyntheticTrafficReport report{*this};
			report.altitude_diff = relative_altitude;
			return report;
		}

		TestSyntheticTrafficReport with_velocity(float horizontal_velocity, float vertical_velocity) const
		{
			TestSyntheticTrafficReport report{*this};
			report.hor_velocity = horizontal_velocity;
			report.ver_velocity = vertical_velocity;
			return report;
		}

		TestSyntheticTrafficReport from_ownship(double lat, double lon, float alt) const
		{
			TestSyntheticTrafficReport report{*this};
			report.lat_uav = lat;
			report.lon_uav = lon;
			report.alt_uav = alt;
			return report;
		}
	};

	TestSyntheticTrafficReport fake_traffic_report(uint32_t icao_address, const char *callsign, float distance)
	{
		return TestSyntheticTrafficReport{icao_address, callsign, distance};
	}

	// Copy a callsign into a transponder report, always leaving the field null-terminated.
	// Centralizes the strncpy + terminator pattern so individual tests cannot get the bounds wrong.
	void set_report_callsign(transponder_report_s &tr, const char *callsign)
	{
		strncpy(tr.callsign, callsign, sizeof(tr.callsign) - 1);
		tr.callsign[sizeof(tr.callsign) - 1] = '\0';
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
		set_report_callsign(tr, callsign);
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

	// Install valid default parameters for the standard built into this firmware so the
	// module activates cleanly. The standard-specific tests still override individual
	// parameters to exercise their own boundaries.
	void set_DFLT_daa_params()
	{
#if defined(CONFIG_NAVIGATOR_ADSB_F3442) && CONFIG_NAVIGATOR_ADSB_F3442
		// NMAC: (150, 30), WC: (600, 80), Aug-WC: 30s, Aug-NMAC: 30s
		param_set(param_handle(px4::params::DAA_LVL_CRIT_RAD), &kDfltDaaLvlCriticalRad);
		param_set(param_handle(px4::params::DAA_LVL_CRIT_HGT), &kDfltDaaLvlCriticalHgt);
		param_set(param_handle(px4::params::DAA_LVL_HIGH_RAD), &kDfltDaaLvlHighRad);
		param_set(param_handle(px4::params::DAA_LVL_HIGH_HGT), &kDfltDaaLvlHighHgt);
		param_set(param_handle(px4::params::DAA_LVL_MED_TIME), &kDfltDaaLvlMediumTime);
		param_set(param_handle(px4::params::DAA_LVL_LOW_TIME), &kDfltDaaLvlLowTime);

		param_set(param_handle(px4::params::DAA_EN_DFLT_VEL), &kDfltDaaEnDfltVel);
		param_set(param_handle(px4::params::DAA_DFLT_VEL), &kDfltDaaDfltVel);
#else
		param_set(param_handle(px4::params::NAV_TRAFF_A_HOR), &kDfltNavTraffHorSep);
		param_set(param_handle(px4::params::NAV_TRAFF_A_VER), &kDfltNavTraffVerSep);
		param_set(param_handle(px4::params::NAV_TRAFF_COLL_T), &kDfltNavTraffCollTime);
#endif // CONFIG_NAVIGATOR_ADSB_F3442
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



	void check_highest_conflict(const uint8_t conflict_level)
	{

		const bool action_required = navigator->get_detect_and_avoid()->get_action_from_conflict_level(conflict_level) > DaaAction::kWarnOnly;

		if (_detect_and_avoid_most_urgent_sub.updated()) {
			ASSERT_TRUE(_detect_and_avoid_most_urgent_sub.update());
		}

		const detect_and_avoid_most_urgent_s &daa_status = _detect_and_avoid_most_urgent_sub.get();
		ASSERT_EQ(daa_status.conflict_level, conflict_level);
		ASSERT_EQ(daa_status.has_action, action_required);
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
