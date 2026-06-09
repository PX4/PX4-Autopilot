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

#include "DaaCrosstrackTestData.h"
#include "DaaCrosstrack.h"

#include <parameters/param.h>

#include <lib/mathlib/mathlib.h>
#include <limits>

namespace
{
const matrix::Vector2d kUavLatLon{32.617013, -96.490564};
constexpr float kUavAlt{1000.f};

void set_crosstrack_params(float horizontal_separation_m, float vertical_separation_m, int collision_time_s)
{
	param_set(param_handle(px4::params::NAV_TRAFF_A_HOR), &horizontal_separation_m);
	param_set(param_handle(px4::params::NAV_TRAFF_A_VER), &vertical_separation_m);
	param_set(param_handle(px4::params::NAV_TRAFF_COLL_T), &collision_time_s);
}

matrix::Vector3f velocity_ned_from_track(float heading_rad, float horizontal_speed_m_s, float vertical_speed_up_m_s)
{
	return matrix::Vector3f(cosf(heading_rad) * horizontal_speed_m_s, sinf(heading_rad) * horizontal_speed_m_s, -vertical_speed_up_m_s);
}

aircraft_state_s create_aircraft_state(const matrix::Vector2d &reference_lat_lon, float reference_altitude,
				       float distance_m, float bearing_rad, float altitude_offset_m, float heading_rad,
				       const matrix::Vector3f &velocity_ned)
{
	double lat{0.0};
	double lon{0.0};
	waypoint_from_heading_and_distance(reference_lat_lon(0), reference_lat_lon(1), bearing_rad, distance_m, &lat, &lon);

	return aircraft_state_s{
		matrix::Vector2d(lat, lon),
		reference_altitude + altitude_offset_m,
		velocity_ned,
		heading_rad
	};
}

aircraft_state_s create_aircraft_state(const traffic_data_s &traffic)
{
	return aircraft_state_s{
		matrix::Vector2d(traffic.lat_traffic, traffic.lon_traffic),
		traffic.alt_traffic,
		velocity_ned_from_track(traffic.heading_traffic, traffic.vxy_traffic, traffic.vz_traffic),
		traffic.heading_traffic
	};
}
} // namespace

class DaaCrosstrackTest : public ::testing::Test
{
protected:
	DaaCrosstrack daa;

	void SetUp() override
	{
		param_control_autosave(false);
		param_reset_all();
		set_crosstrack_params(500.f, 500.f, 60);
	}
};

// WHY: Crosstrack parameters directly control the conflict gate, so invalid runtime values must be rejected immediately.
// WHAT: Exercise each tuning parameter with NaN, Inf, and out-of-range inputs through try_setting_params() and verify the object fails closed.
TEST_F(DaaCrosstrackTest, RejectsNonFiniteParams)
{
	// GIVEN: Baseline valid params installed by SetUp(), try_setting_params() must accept them.
	ASSERT_TRUE(daa.try_setting_params());

	const float nan = std::numeric_limits<float>::quiet_NaN();
	const float inf = std::numeric_limits<float>::infinity();
	const int negative_time = -1;

	// WHEN: The horizontal separation is invalidated with NaN, Inf, or zero.
	for (const float bad : {nan, inf, 0.f}) {
		set_crosstrack_params(bad, 500.f, 60);
		EXPECT_FALSE(daa.try_setting_params()) << "horizontal separation = " << bad;
	}

	// WHEN: The vertical separation is invalidated with NaN, Inf, or zero.
	for (const float bad : {nan, inf, 0.f}) {
		set_crosstrack_params(500.f, bad, 60);
		EXPECT_FALSE(daa.try_setting_params()) << "vertical separation = " << bad;
	}

	// WHEN: The collision time is set to a negative value.
	set_crosstrack_params(500.f, 500.f, negative_time);
	EXPECT_FALSE(daa.try_setting_params());

	// THEN: Restoring valid values lets try_setting_params() succeed again.
	set_crosstrack_params(500.f, 500.f, 60);
	EXPECT_TRUE(daa.try_setting_params());
}

// WHY: The crosstrack solver relies on a finite traffic heading to build the predicted path line.
// WHAT: Pass a NaN traffic heading directly into `calculate_daa_stats()` and verify it returns no conflict with cleared output stats.
TEST_F(DaaCrosstrackTest, InvalidTrafficHeadingReturnsNoConflict)
{
	// GIVEN: Valid crosstrack parameters and a traffic encounter with a NaN heading.
	ASSERT_TRUE(daa.try_setting_params());

	const aircraft_state_s uav_state{
		kUavLatLon,
		kUavAlt,
		matrix::Vector3f(0.f, 0.f, 0.f),
		0.f
	};
	aircraft_state_s traffic_state = create_aircraft_state(kUavLatLon, kUavAlt, 200.f, math::radians(90.f), 0.f,
					 std::numeric_limits<float>::quiet_NaN(), matrix::Vector3f(0.f, -30.f, 0.f));

	// WHEN: The crosstrack implementation evaluates that encounter.
	daa_stats_s daa_stats{};
	EXPECT_EQ(daa.calculate_daa_stats(uav_state, traffic_state, daa_stats), detect_and_avoid_s::DAA_CONFLICT_LVL_NONE);

	// THEN: It fails closed and clears all reported stats.
	EXPECT_FLOAT_EQ(daa_stats.aircraft_dist_hor, 0.f);
	EXPECT_FLOAT_EQ(daa_stats.aircraft_dist_vert, 0.f);
	EXPECT_FLOAT_EQ(daa_stats.expected_min_dist_time_sec, 0.f);
}

// WHY: A representative approaching encounter should trigger the crosstrack gates without requiring the test to duplicate the full line-distance math.
// WHAT: Run one conflict scenario through `calculate_daa_stats()` and verify severity plus bounded cross-track, vertical, and timing outputs.
TEST_F(DaaCrosstrackTest, DetectsApproachingConflictAndReportsExpectedStats)
{
	// GIVEN: Valid crosstrack parameters and an approaching head-on encounter.
	ASSERT_TRUE(daa.try_setting_params());

	const aircraft_state_s uav_state{
		kUavLatLon,
		kUavAlt,
		matrix::Vector3f(0.f, 0.f, 0.f),
		0.f
	};
	const aircraft_state_s traffic_state = create_aircraft_state(kUavLatLon, kUavAlt, 200.f, math::radians(90.f), 0.f,
					       math::radians(270.f), matrix::Vector3f(0.f, -30.f, 0.f));

	// WHEN: The crosstrack implementation evaluates the encounter.
	daa_stats_s daa_stats{};
	EXPECT_EQ(daa.calculate_daa_stats(uav_state, traffic_state, daa_stats), detect_and_avoid_s::DAA_CONFLICT_LVL_HIGH);

	// THEN: It reports a high conflict with the expected cross-track, vertical, and timing outputs.
	EXPECT_LT(fabsf(daa_stats.aircraft_dist_hor), 500.f);
	EXPECT_NEAR(daa_stats.aircraft_dist_vert, 0.f, 0.1f);
	EXPECT_NEAR(daa_stats.expected_min_dist_time_sec, 200.f / 30.f, 0.1f);
}

// WHY: Ensure that the new DAA work preserves the old crosstrack conflict decisions from the adsb/test_adsb_helper.ipynb dataset.
// WHAT: Replay every legacy traffic sample from `DaaCrosstrackTestData.h` through `DaaCrosstrack` and verify the published conflict level matches the historic data.
TEST_F(DaaCrosstrackTest, MatchesLegacyNotebookDataset)
{
	// GIVEN: The same ownship state and runtime parameters that the original crosstrack test used.
	ASSERT_TRUE(daa.try_setting_params());

	const aircraft_state_s uav_state{
		kUavLatLon,
		kUavAlt,
		matrix::Vector3f(0.f, 0.f, 0.f),
		0.f
	};

	// WHEN: Every frozen traffic sample is replayed through the current crosstrack solver.
	for (uint32_t i = 0; i < sizeof(kTrafficDataset) / sizeof(kTrafficDataset[0]); ++i) {
		const traffic_data_s &traffic = kTrafficDataset[i];
		SCOPED_TRACE(testing::Message() << "kTrafficDataset[" << i << "]");

		daa_stats_s daa_stats{};
		const uint8_t conflict_level = daa.calculate_daa_stats(uav_state, create_aircraft_state(traffic), daa_stats);
		const uint8_t expected_conflict_level = traffic.in_conflict ? detect_and_avoid_s::DAA_CONFLICT_LVL_HIGH :
							detect_and_avoid_s::DAA_CONFLICT_LVL_NONE;

		// THEN: The new implementation keeps the legacy conflict decisions for every historic sample.
		EXPECT_EQ(conflict_level, expected_conflict_level);
	}
}
