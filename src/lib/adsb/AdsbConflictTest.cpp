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
#include "AdsbConflict.h"

#include <parameters/param.h>

#include <lib/mathlib/mathlib.h>
#include <limits>

struct RelativeTrafficScenario {
	float distance_m;
	float bearing_rad;
	float altitude_offset_m;
	float heading_rad;
	float hor_speed_m_s;
	float ver_speed_up_m_s;
};

class AdsbConflictTest : public ::testing::Test
{
protected:
	const matrix::Vector2d uav_lat_lon_{32.617013, -96.490564};
	const float uav_alt_{1000.f};
	const float uav_heading_{0.f};
	const matrix::Vector3f stationary_uav_vel_ned_{0.f, 0.f, 0.f};

	void SetUp() override
	{
		param_control_autosave(false);
		param_reset_all();
#if defined(CONFIG_NAVIGATOR_ADSB_F3442) && CONFIG_NAVIGATOR_ADSB_F3442
		set_f3442_params(20.f, 10.f, 100.f, 50.f, 30, 40);
#else
		set_crosstrack_params(500.f, 500.f, 60);
#endif // CONFIG_NAVIGATOR_ADSB_F3442
	}

#if defined(CONFIG_NAVIGATOR_ADSB_F3442) && CONFIG_NAVIGATOR_ADSB_F3442
	void set_f3442_params(float nmac_radius, float nmac_height, float wc_radius,
			      float wc_height, int nmac_latency_s, int wc_latency_s)
	{
		param_set(param_handle(px4::params::F34_LVL_CRIT_RAD), &nmac_radius);
		param_set(param_handle(px4::params::F34_LVL_CRIT_HGT), &nmac_height);
		param_set(param_handle(px4::params::F34_LVL_HIGH_RAD), &wc_radius);
		param_set(param_handle(px4::params::F34_LVL_HIGH_HGT), &wc_height);
		param_set(param_handle(px4::params::F34_LVL_MED_TIME), &nmac_latency_s);
		param_set(param_handle(px4::params::F34_LVL_LOW_TIME), &wc_latency_s);
	}
#endif // CONFIG_NAVIGATOR_ADSB_F3442

	void set_crosstrack_params(float horizontal_separation_m,
				   float vertical_separation_m, int collision_time_s)
	{
		param_set(param_handle(px4::params::NAV_TRAFF_A_HOR), &horizontal_separation_m);
		param_set(param_handle(px4::params::NAV_TRAFF_A_VER), &vertical_separation_m);
		param_set(param_handle(px4::params::NAV_TRAFF_COLL_T), &collision_time_s);
	}

	/**
	 * @brief Build a traffic report from a relative encounter with the fixed
	 * ownship state used by these tests.
	 */
	transponder_report_s create_relative_report(const RelativeTrafficScenario &scenario)
	{
		transponder_report_s report{};
		report.timestamp = hrt_absolute_time();
		waypoint_from_heading_and_distance(uav_lat_lon_(0), uav_lat_lon_(1),
						   scenario.bearing_rad, scenario.distance_m, &report.lat, &report.lon);
		report.altitude = uav_alt_ + scenario.altitude_offset_m;
		report.heading = scenario.heading_rad;
		report.hor_velocity = scenario.hor_speed_m_s;
		report.ver_velocity = scenario.ver_speed_up_m_s;
		return report;
	}

	daa_input_s create_daa_input(const matrix::Vector2d &uav_lat_lon, const float uav_alt, const float uav_heading,
				     const matrix::Vector3f &uav_vel_ned, const transponder_report_s &report) const
	{
		daa_input_s daa_input{};
		daa_input.uav_lat_lon = uav_lat_lon;
		daa_input.uav_alt = uav_alt;
		daa_input.uav_heading = uav_heading;
		daa_input.uav_vel_ned = uav_vel_ned;
		daa_input.transponder_report = report;
		return daa_input;
	}
};

// WHY: Input validation must fail closed so NaNs and infinities never reach the conflict algorithms.
// WHAT: Feed non-finite coordinates and velocities into `calculate_daa_output()` and verify the library rejects them.
TEST_F(AdsbConflictTest, RejectsNonFiniteCoordinatesAndVelocities)
{
	// GIVEN: The built DAA standard with an otherwise valid traffic encounter.
	AdsbConflict adsb_conflict;
	const RelativeTrafficScenario valid_traffic{200.f, math::radians(90.f), 0.f, uav_heading_, 30.f, 0.f};

	detect_and_avoid_s daa_output{};
	transponder_report_s report = create_relative_report(valid_traffic);
	const float nan = std::numeric_limits<float>::quiet_NaN();
	const float inf = std::numeric_limits<float>::infinity();

	// WHEN: Any ownship coordinate becomes non-finite.
	EXPECT_FALSE(adsb_conflict.calculate_daa_output(create_daa_input(matrix::Vector2d(nan, uav_lat_lon_(1)), uav_alt_,
			uav_heading_, stationary_uav_vel_ned_, report), daa_output));
	EXPECT_FALSE(adsb_conflict.calculate_daa_output(create_daa_input(matrix::Vector2d(uav_lat_lon_(0), nan), uav_alt_,
			uav_heading_, stationary_uav_vel_ned_, report), daa_output));

	// WHEN: The traffic position becomes non-finite.
	report.lat = nan;
	EXPECT_FALSE(adsb_conflict.calculate_daa_output(create_daa_input(uav_lat_lon_, uav_alt_, uav_heading_,
			stationary_uav_vel_ned_, report), daa_output));

	// WHEN: Either aircraft advertises a non-finite velocity.
	report = create_relative_report(valid_traffic);
	report.hor_velocity = inf;
	EXPECT_FALSE(adsb_conflict.calculate_daa_output(create_daa_input(uav_lat_lon_, uav_alt_, uav_heading_,
			stationary_uav_vel_ned_, report), daa_output));

	report = create_relative_report(valid_traffic);

	// THEN: The wrapper fails and never publishes conflict output.
	EXPECT_FALSE(adsb_conflict.calculate_daa_output(create_daa_input(uav_lat_lon_, uav_alt_, uav_heading_,
			matrix::Vector3f(0.f, nan, 0.f), report), daa_output));
}

#if !defined(CONFIG_NAVIGATOR_ADSB_F3442) || !CONFIG_NAVIGATOR_ADSB_F3442
// WHY: Crosstrack mode depends on finite headings, and the wrapper must block invalid values before delegating.
// WHAT: Select crosstrack mode, inject NaN headings on both the traffic and ownship sides, and verify
// `calculate_daa_output()` fails.
TEST_F(AdsbConflictTest, CrosstrackRejectsNonFiniteHeadings)
{
	// GIVEN: Crosstrack mode with a valid approaching encounter.
	AdsbConflict adsb_conflict;
	const RelativeTrafficScenario approaching_traffic{200.f, math::radians(90.f), 0.f, math::radians(270.f), 30.f, 0.f};

	detect_and_avoid_s daa_output{};
	transponder_report_s report = create_relative_report(approaching_traffic);
	const float nan = std::numeric_limits<float>::quiet_NaN();

	// WHEN: The ownship heading is non-finite.
	// THEN: Crosstrack processing rejects the encounter.
	EXPECT_FALSE(adsb_conflict.calculate_daa_output(create_daa_input(uav_lat_lon_, uav_alt_, nan,
			stationary_uav_vel_ned_, report), daa_output));

	// WHEN: The traffic heading is non-finite.
	report.heading = nan;
	// THEN: Crosstrack processing rejects the encounter.
	EXPECT_FALSE(adsb_conflict.calculate_daa_output(create_daa_input(uav_lat_lon_, uav_alt_, uav_heading_,
			stationary_uav_vel_ned_, report), daa_output));
}
#endif // !CONFIG_NAVIGATOR_ADSB_F3442

// WHY: `try_updating_params()` is the last line of defense against invalid runtime tuning.
// WHAT: Validate a good parameter set for the built standard, then break a required parameter and confirm the update is rejected.
TEST_F(AdsbConflictTest, UsesBuiltStandardForParamValidation)
{
	// GIVEN: A wrapper using the standard selected at build time.
	AdsbConflict adsb_conflict;
	constexpr float invalid_bound{-1.f};

	// WHEN: The built standard starts with valid parameters.
	EXPECT_TRUE(adsb_conflict.try_updating_params());

#if defined(CONFIG_NAVIGATOR_ADSB_F3442) && CONFIG_NAVIGATOR_ADSB_F3442
	set_f3442_params(invalid_bound, 10.f, 100.f, 50.f, 30, 40);
#else
	set_crosstrack_params(invalid_bound, 500.f, 60);
#endif // CONFIG_NAVIGATOR_ADSB_F3442

	// THEN: The built standard validates against its own parameter set.
	EXPECT_FALSE(adsb_conflict.try_updating_params());
}

// WHY: `AdsbConflict` owns output forwarding, not the detailed math inside the built standard.
// WHAT: Evaluate one representative encounter and verify the wrapper publishes the built standard's result.
TEST_F(AdsbConflictTest, DelegatesToBuiltStandardAndForwardsOutput)
{
	// GIVEN: One representative encounter for the built standard.
	AdsbConflict adsb_conflict;
	const RelativeTrafficScenario approaching_traffic{19.f, math::radians(90.f), 9.f, math::radians(270.f), 5.f, 0.f};
	const transponder_report_s report = create_relative_report(approaching_traffic);

	detect_and_avoid_s daa_output{};

	// WHEN: The encounter is processed.
	ASSERT_TRUE(adsb_conflict.try_updating_params());
	ASSERT_TRUE(adsb_conflict.calculate_daa_output(create_daa_input(uav_lat_lon_, uav_alt_, uav_heading_,
			stationary_uav_vel_ned_, report), daa_output));

#if defined(CONFIG_NAVIGATOR_ADSB_F3442) && CONFIG_NAVIGATOR_ADSB_F3442
	// THEN: The wrapper publishes the delegated F3442 result.
	EXPECT_EQ(daa_output.conflict_level, detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL);
	EXPECT_NEAR(daa_output.aircraft_dist_hor, approaching_traffic.distance_m, 0.1f);
	EXPECT_NEAR(daa_output.aircraft_dist_vert, approaching_traffic.altitude_offset_m, 0.1f);
#else
	// THEN: The wrapper publishes the delegated crosstrack result instead.
	EXPECT_EQ(daa_output.conflict_level, detect_and_avoid_s::DAA_CONFLICT_LVL_HIGH);
	EXPECT_NEAR(daa_output.aircraft_dist_vert, approaching_traffic.altitude_offset_m, 0.1f);
	EXPECT_LT(fabsf(daa_output.aircraft_dist_hor), 500.f);
#endif // CONFIG_NAVIGATOR_ADSB_F3442
}
