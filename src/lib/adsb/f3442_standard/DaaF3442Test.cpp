/****************************************************************************
 *
 *   Copyright (C) 2024 PX4 Development Team. All rights reserved.
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
 * @file DaaF3442Test.cpp
 *
 * Test file for the F3442 DAA standard
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include <gtest/gtest.h>
#include <parameters/param.h>

#include <array>
#include <lib/mathlib/mathlib.h>
#include <limits>

#include "DaaF3442.h"

class DaaF3442Test : public ::testing::Test
{
protected:
	DaaF3442 daa;

	void SetUp() override
	{
		param_control_autosave(false);
		param_reset_all();
	}

	void set_f3442_params(float nmac_radius, float nmac_height, float wc_radius, float wc_height,
			      int nmac_latency_s, int wc_latency_s)
	{
		param_set(param_handle(px4::params::F34_LVL_CRIT_RAD), &nmac_radius);
		param_set(param_handle(px4::params::F34_LVL_CRIT_HGT), &nmac_height);
		param_set(param_handle(px4::params::F34_LVL_HIGH_RAD), &wc_radius);
		param_set(param_handle(px4::params::F34_LVL_HIGH_HGT), &wc_height);
		param_set(param_handle(px4::params::F34_LVL_MED_TIME), &nmac_latency_s);
		param_set(param_handle(px4::params::F34_LVL_LOW_TIME), &wc_latency_s);
	}
};

// WHY: Bad F3442 tuning silently changes conflict severity, so invalid runtime parameters must be rejected.
// WHAT: Exercise representative valid, negative, and non-finite parameter sets and verify `try_setting_params()` only accepts the safe configuration.
TEST_F(DaaF3442Test, RejectsNegativeAndNonFiniteParams)
{
	// GIVEN: Representative valid, negative, and non-finite F3442 parameter sets.
	struct ParamCase {
		float nmac_radius;
		float nmac_height;
		float wc_radius;
		float wc_height;
		int nmac_latency_s;
		int wc_latency_s;
		bool expected_result;
	};

	constexpr float valid_nmac_radius_m = 10.f;
	constexpr float valid_nmac_height_m = 10.f;
	constexpr float valid_wc_radius_m = 20.f;
	constexpr float valid_wc_height_m = 20.f;
	constexpr int valid_nmac_latency_s = 5;
	constexpr int valid_wc_latency_s = 5;
	const float nan = std::numeric_limits<float>::quiet_NaN();
	const float inf = std::numeric_limits<float>::infinity();

	const std::array<ParamCase, 11> cases{{
			{valid_nmac_radius_m, valid_nmac_height_m, valid_wc_radius_m, valid_wc_height_m, valid_nmac_latency_s, valid_wc_latency_s, true},
			{-valid_nmac_radius_m, valid_nmac_height_m, valid_wc_radius_m, valid_wc_height_m, valid_nmac_latency_s, valid_wc_latency_s, false},
			{valid_nmac_radius_m, -valid_nmac_height_m, valid_wc_radius_m, valid_wc_height_m, valid_nmac_latency_s, valid_wc_latency_s, false},
			{valid_nmac_radius_m, valid_nmac_height_m, -valid_wc_radius_m, valid_wc_height_m, valid_nmac_latency_s, valid_wc_latency_s, false},
			{valid_nmac_radius_m, valid_nmac_height_m, valid_wc_radius_m, -valid_wc_height_m, valid_nmac_latency_s, valid_wc_latency_s, false},
			{nan, valid_nmac_height_m, valid_wc_radius_m, valid_wc_height_m, valid_nmac_latency_s, valid_wc_latency_s, false},
			{valid_nmac_radius_m, inf, valid_wc_radius_m, valid_wc_height_m, valid_nmac_latency_s, valid_wc_latency_s, false},
			{valid_nmac_radius_m, valid_nmac_height_m, nan, valid_wc_height_m, valid_nmac_latency_s, valid_wc_latency_s, false},
			{valid_nmac_radius_m, valid_nmac_height_m, valid_wc_radius_m, inf, valid_nmac_latency_s, valid_wc_latency_s, false},
			{valid_nmac_radius_m, valid_nmac_height_m, valid_wc_radius_m, valid_wc_height_m, -valid_nmac_latency_s, valid_wc_latency_s, false},
			{valid_nmac_radius_m, valid_nmac_height_m, valid_wc_radius_m, valid_wc_height_m, valid_nmac_latency_s, -valid_wc_latency_s, false},
		}};

	// WHEN: Each parameter set is loaded into the DAA object.
	for (size_t i = 0; i < cases.size(); ++i) {
		const ParamCase &test_case = cases[i];
		SCOPED_TRACE(i);

		set_f3442_params(test_case.nmac_radius, test_case.nmac_height, test_case.wc_radius, test_case.wc_height,
				 test_case.nmac_latency_s, test_case.wc_latency_s);
		EXPECT_EQ(daa.try_setting_params(), test_case.expected_result);
	}
}

// WHY: The conflict volumes must remain hierarchical, otherwise a lower-severity zone can become smaller than a higher-severity one.
// WHAT: Configure weaker outer bounds than inner bounds and verify F3442 rejects each non-hierarchical configuration.
TEST_F(DaaF3442Test, RejectsNonHierarchicalBounds)
{
	// GIVEN: Parameter sets where an outer protection volume is not larger than the inner one.
	struct HierarchyCase {
		float nmac_radius;
		float nmac_height;
		float wc_radius;
		float wc_height;
	};

	constexpr int nmac_latency_s = 30;
	constexpr int wc_latency_s = 30;
	const std::array<HierarchyCase, 3> cases{{
			{20.f, 10.f, 19.f, 10.f},
			{20.f, 10.f, 20.f, 9.f},
			{200.f, 100.f, 100.f, 50.f},
		}};

	// WHEN: The hierarchy is validated.
	for (size_t i = 0; i < cases.size(); ++i) {
		const HierarchyCase &test_case = cases[i];
		SCOPED_TRACE(i);

		set_f3442_params(test_case.nmac_radius, test_case.nmac_height, test_case.wc_radius, test_case.wc_height,
				 nmac_latency_s, wc_latency_s);

		// THEN: F3442 rejects every non-hierarchical configuration.
		EXPECT_FALSE(daa.try_setting_params());
	}
}

// WHY: `is_in_bounds()` is the base primitive for every F3442 zone comparison.
// WHAT: Check inclusive bounds handling and confirm invalid negative or non-finite inputs fail closed.
TEST_F(DaaF3442Test, IsInBoundsRejectsInvalidInputs)
{
	// GIVEN: A symmetric bound and representative in-range, out-of-range, and invalid inputs.
	const matrix::Vector2f bounds(10.f, 10.f);
	const matrix::Vector2f within_bounds(5.f, 5.f);
	const matrix::Vector2f on_boundary(10.f, 10.f);
	const matrix::Vector2f outside_both_bounds(15.f, 15.f);
	const matrix::Vector2f outside_horizontal_bounds(15.f, 4.f);
	const matrix::Vector2f outside_vertical_bounds(4.f, 15.f);
	const matrix::Vector2f negative_bounds(-10.f, 10.f);
	const matrix::Vector2f negative_distance(-5.f, 5.f);
	const matrix::Vector2f non_finite_distance(5.f, std::numeric_limits<float>::quiet_NaN());

	// WHEN: The boundary check is evaluated across those inputs.
	EXPECT_TRUE(daa.is_in_bounds(within_bounds, bounds));
	EXPECT_TRUE(daa.is_in_bounds(on_boundary, bounds));
	EXPECT_FALSE(daa.is_in_bounds(outside_both_bounds, bounds));
	EXPECT_FALSE(daa.is_in_bounds(outside_horizontal_bounds, bounds));
	EXPECT_FALSE(daa.is_in_bounds(outside_vertical_bounds, bounds));
	EXPECT_FALSE(daa.is_in_bounds(within_bounds, negative_bounds));
	EXPECT_FALSE(daa.is_in_bounds(negative_distance, bounds));

	// THEN: Invalid magnitudes and non-finite values fail closed.
	EXPECT_FALSE(daa.is_in_bounds(non_finite_distance, bounds));
}

// WHY: Augmented F3442 volumes are based on absolute speed and latency, so sign errors would shrink predicted conflict regions.
// WHAT: Feed negative velocities and latency into `calculate_aircraft_conflict_volume()` and verify it still expands the volume by absolute magnitude.
TEST_F(DaaF3442Test, CalculateAircraftConflictVolumeUsesAbsoluteInputs)
{
	// GIVEN: A base volume, signed velocity components, and a negative latency value.
	const matrix::Vector2f base_bounds(10.f, 20.f);
	const matrix::Vector2f velocity(-2.f, 4.f);
	const float latency_s = -5.f;
	const float expected_horizontal_bounds_m = base_bounds(0) + fabsf(velocity(0)) * fabsf(latency_s);
	const float expected_vertical_bounds_m = base_bounds(1) + fabsf(velocity(1)) * fabsf(latency_s);

	// WHEN: The traffic conflict volume is augmented.
	matrix::Vector2f augmented_bounds;
	daa.calculate_aircraft_conflict_volume(base_bounds, velocity, latency_s, augmented_bounds);

	// THEN: The augmentation uses absolute magnitudes for both velocity and latency.
	EXPECT_FLOAT_EQ(augmented_bounds(0), expected_horizontal_bounds_m);
	EXPECT_FLOAT_EQ(augmented_bounds(1), expected_vertical_bounds_m);
}

// WHY: F3442 augmented boundaries model both aircraft volumes and must include motion from each side.
// WHAT: Expand a shared base boundary with different ownship and traffic velocities and verify the resulting component-wise sum.
TEST_F(DaaF3442Test, CalculateAugmentedBoundariesAddsBothAircraftVolumes)
{
	// GIVEN: A shared base volume and distinct ownship and traffic velocities.
	const matrix::Vector2f base_bounds(10.f, 20.f);
	const matrix::Vector2f uav_velocity(2.f, 4.f);
	const matrix::Vector2f traffic_velocity(8.f, 10.f);
	const float latency_s = 3.f;
	const float expected_horizontal_bounds_m = 2.f * base_bounds(0)
			+ (fabsf(uav_velocity(0)) + fabsf(traffic_velocity(0))) * latency_s;
	const float expected_vertical_bounds_m = 2.f * base_bounds(1)
			+ (fabsf(uav_velocity(1)) + fabsf(traffic_velocity(1))) * latency_s;

	// WHEN: The augmented F3442 boundaries are calculated.
	matrix::Vector2f augmented_bounds;
	daa.calculate_augmented_boundaries(base_bounds, uav_velocity, traffic_velocity, latency_s, augmented_bounds);

	// THEN: The final volume includes motion from both aircraft.
	EXPECT_FLOAT_EQ(augmented_bounds(0), expected_horizontal_bounds_m);
	EXPECT_FLOAT_EQ(augmented_bounds(1), expected_vertical_bounds_m);
}

// WHY: F3442 severity depends on the first breached zone in priority order, and the class now returns the published level directly.
// WHAT: Configure representative bounds, probe each zone boundary, and verify `calculate_conflict_level()` returns the expected highest-priority level.
TEST_F(DaaF3442Test, CalculateConflictLevelRespectsPriorityOrder)
{
	// GIVEN: Valid F3442 parameters and representative distances for each breach tier.
	constexpr float nmac_radius_m = 20.f;
	constexpr float nmac_height_m = 10.f;
	constexpr float wc_radius_m = 100.f;
	constexpr float wc_height_m = 50.f;
	constexpr int nmac_latency_s = 30;
	constexpr int wc_latency_s = 40;
	set_f3442_params(nmac_radius_m, nmac_height_m, wc_radius_m, wc_height_m, nmac_latency_s, wc_latency_s);
	ASSERT_TRUE(daa.try_setting_params());

	const matrix::Vector2f uav_velocity(10.f, 2.f);
	const matrix::Vector2f traffic_velocity(5.f, 4.f);
	const matrix::Vector2f nmac_bounds(2.f * nmac_radius_m, 2.f * nmac_height_m);
	const matrix::Vector2f wc_bounds(2.f * wc_radius_m, 2.f * wc_height_m);
	const matrix::Vector2f aug_nmac_bounds(
		2.f * nmac_radius_m + (fabsf(uav_velocity(0)) + fabsf(traffic_velocity(0))) * nmac_latency_s,
		2.f * nmac_height_m + (fabsf(uav_velocity(1)) + fabsf(traffic_velocity(1))) * nmac_latency_s);
	const matrix::Vector2f aug_wc_bounds(
		2.f * wc_radius_m + (fabsf(uav_velocity(0)) + fabsf(traffic_velocity(0))) * wc_latency_s,
		2.f * wc_height_m + (fabsf(uav_velocity(1)) + fabsf(traffic_velocity(1))) * wc_latency_s);
	constexpr float inside_margin_m = 1.f;
	const matrix::Vector2f critical_distance(nmac_bounds(0) - inside_margin_m, nmac_bounds(1) - inside_margin_m);
	const matrix::Vector2f high_distance(wc_bounds(0) - inside_margin_m, wc_bounds(1) - inside_margin_m);
	const matrix::Vector2f medium_distance(aug_nmac_bounds(0) - inside_margin_m, aug_nmac_bounds(1) - inside_margin_m);
	const matrix::Vector2f low_distance(aug_wc_bounds(0) - inside_margin_m, aug_wc_bounds(1) - inside_margin_m);
	const matrix::Vector2f outside_low_distance(aug_wc_bounds(0) + inside_margin_m, aug_wc_bounds(1) + inside_margin_m);
	const matrix::Vector2f outside_low_vertical_only(low_distance(0), aug_wc_bounds(1) + inside_margin_m);
	const matrix::Vector2f outside_low_horizontal_only(aug_wc_bounds(0) + inside_margin_m, low_distance(1));

	// WHEN: The same velocity pair is evaluated against progressively weaker breaches.
	EXPECT_EQ(daa.calculate_conflict_level(critical_distance, uav_velocity, traffic_velocity),
		  detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL);
	EXPECT_EQ(daa.calculate_conflict_level(high_distance, uav_velocity, traffic_velocity),
		  detect_and_avoid_s::DAA_CONFLICT_LVL_HIGH);
	EXPECT_EQ(daa.calculate_conflict_level(medium_distance, uav_velocity, traffic_velocity),
		  detect_and_avoid_s::DAA_CONFLICT_LVL_MEDIUM);
	EXPECT_EQ(daa.calculate_conflict_level(low_distance, uav_velocity, traffic_velocity),
		  detect_and_avoid_s::DAA_CONFLICT_LVL_LOW);
	EXPECT_EQ(daa.calculate_conflict_level(outside_low_distance, uav_velocity, traffic_velocity),
		  detect_and_avoid_s::DAA_CONFLICT_LVL_NONE);
	EXPECT_EQ(daa.calculate_conflict_level(outside_low_vertical_only, uav_velocity, traffic_velocity),
		  detect_and_avoid_s::DAA_CONFLICT_LVL_NONE);

	// THEN: The first breached zone in priority order determines the final published level.
	EXPECT_EQ(daa.calculate_conflict_level(outside_low_horizontal_only, uav_velocity, traffic_velocity),
		  detect_and_avoid_s::DAA_CONFLICT_LVL_NONE);
}

// WHY: `calculate_daa_stats()` is the public F3442 entry point and must preserve both measured geometry and derived severity.
// WHAT: Run one near-collision and one no-conflict geometry through the full method and verify distance reporting, head-on collision timing, and final level.
TEST_F(DaaF3442Test, CalculateDaaStatsReportsExpectedGeometryAndLevel)
{
	// GIVEN: Valid F3442 parameters and one near-conflict plus one clear encounter geometry.
	constexpr float nmac_radius_m = 20.f;
	constexpr float nmac_height_m = 10.f;
	constexpr float wc_radius_m = 100.f;
	constexpr float wc_height_m = 50.f;
	constexpr int nmac_latency_s = 30;
	constexpr int wc_latency_s = 40;
	set_f3442_params(nmac_radius_m, nmac_height_m, wc_radius_m, wc_height_m, nmac_latency_s, wc_latency_s);
	ASSERT_TRUE(daa.try_setting_params());

	const matrix::Vector2d uav_lat_lon(46.52342, 6.524234);
	constexpr float uav_altitude_m = 400.f;
	const matrix::Vector3f uav_velocity_ned_m_s(10.f, 0.f, -2.f);
	constexpr float uav_heading_rad = 0.f;
	const aircraft_state_s uav_state{
		uav_lat_lon,
		uav_altitude_m,
		uav_velocity_ned_m_s,
		uav_heading_rad
	};

	aircraft_state_s traffic_state{};
	const matrix::Vector3f traffic_velocity_ned_m_s(-5.f, 0.f, -4.f);
	const float traffic_heading_rad = math::radians(180.f);
	const float east_bearing_rad = math::radians(90.f);
	constexpr float expectation_tolerance = 0.1f;
	double traffic_lat{0.0};
	double traffic_lon{0.0};
	traffic_state.velocity_ned = traffic_velocity_ned_m_s;
	traffic_state.heading = traffic_heading_rad;

	auto place_traffic_due_east = [&](float horizontal_distance_m, float traffic_altitude_m) {
		waypoint_from_heading_and_distance(uav_state.lat_lon(0), uav_state.lat_lon(1), east_bearing_rad, horizontal_distance_m,
						   &traffic_lat, &traffic_lon);
		traffic_state.lat_lon = matrix::Vector2d(traffic_lat, traffic_lon);
		traffic_state.altitude = traffic_altitude_m;
	};

	const float uav_speed_m_s = uav_velocity_ned_m_s.norm();
	const float traffic_speed_m_s = traffic_velocity_ned_m_s.norm();
	const float expected_relative_speed_m_s = uav_speed_m_s + traffic_speed_m_s;
	constexpr float near_horizontal_distance_m = 39.f;
	constexpr float near_vertical_separation_m = 9.f;
	const float near_traffic_altitude_m = uav_altitude_m + near_vertical_separation_m;
	place_traffic_due_east(near_horizontal_distance_m, near_traffic_altitude_m);

	// WHEN: The near-conflict geometry is evaluated through the public stats API.
	daa_stats_s daa_stats{};
	EXPECT_EQ(daa.calculate_daa_stats(uav_state, traffic_state, daa_stats), detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL);
	EXPECT_NEAR(daa_stats.aircraft_dist_hor, near_horizontal_distance_m, expectation_tolerance);
	EXPECT_NEAR(daa_stats.aircraft_dist_vert, near_vertical_separation_m, expectation_tolerance);
	EXPECT_NEAR(daa_stats.expected_min_dist_time_sec, near_horizontal_distance_m / expected_relative_speed_m_s, expectation_tolerance);

	// WHEN: The traffic is moved well outside every protection volume.
	constexpr float far_horizontal_distance_m = 900.f;
	constexpr float far_vertical_separation_m = 400.f;
	const float far_traffic_altitude_m = uav_altitude_m + far_vertical_separation_m;
	place_traffic_due_east(far_horizontal_distance_m, far_traffic_altitude_m);

	// THEN: The API reports no conflict while preserving the measured separation.
	EXPECT_EQ(daa.calculate_daa_stats(uav_state, traffic_state, daa_stats), detect_and_avoid_s::DAA_CONFLICT_LVL_NONE);
	EXPECT_NEAR(daa_stats.aircraft_dist_hor, far_horizontal_distance_m, expectation_tolerance);
	EXPECT_NEAR(daa_stats.aircraft_dist_vert, far_vertical_separation_m, expectation_tolerance);
	EXPECT_NEAR(daa_stats.expected_min_dist_time_sec, far_horizontal_distance_m / expected_relative_speed_m_s, expectation_tolerance);
}
