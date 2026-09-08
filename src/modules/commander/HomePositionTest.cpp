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

#include "HomePosition.hpp"
#include "commander_helper.h"

#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/home_position.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>

#include <memory>
#include <tuple>

// to run: make tests TESTFILTER=HomePosition

class HomePositionTest : public ::testing::Test
{
protected:
	static constexpr double kLat = 47.397742;
	static constexpr double kLon = 8.545594;
	static constexpr float kRefAlt = 100.f;
	static constexpr float kDrift = 3.f; // GNSS altitude jump, above kAltitudeDifferenceThreshold
	static constexpr float kTolerance = 1e-4f;

	void SetUp() override
	{
		param_control_autosave(false);

		// The constructor caches EKF2_HGT_REF and the parameter objects read their
		// values on construction, so all parameters are set before HomePosition exists
		const char *names[] = {"COM_HOME_EN", "EKF2_GPS_CTRL", "EKF2_HGT_REF"};
		const int32_t values[] = {1, 3, 1};

		for (unsigned i = 0; i < 3; ++i) {
			_saved[i].handle = param_find(names[i]);
			ASSERT_NE(_saved[i].handle, PARAM_INVALID) << names[i];
			ASSERT_EQ(param_get(_saved[i].handle, &_saved[i].value), 0);
			ASSERT_EQ(param_set_no_notification(_saved[i].handle, &values[i]), 0);
		}

		// tune_home_set() publishes LED and tune controls; a missing LED device is not an error here
		(void)led_init();
		ASSERT_EQ(buzzer_init(), 0);
		_helpers_initialized = true;

		_flags.attitude_invalid = true;
		_home = std::make_unique<HomePosition>(_flags);

		publishPosition(0.f, kRefAlt, true, true);
		publishGnss(kRefAlt, false); // overwrite retained GNSS data with an invalid fix
	}

	void TearDown() override
	{
		_home.reset(); // destroy before the cached parameters are restored

		if (_helpers_initialized) {
			buzzer_deinit();
			led_deinit();
		}

		for (const auto &saved : _saved) {
			if (saved.handle != PARAM_INVALID) {
				param_set_no_notification(saved.handle, &saved.value);
			}
		}
	}

	// Publish the local and global position estimates and let HomePosition consume them
	void publishPosition(float z, float alt, bool global_valid, bool global_reference)
	{
		_flags.local_position_invalid = false;
		_flags.global_position_invalid = !global_valid;
		const hrt_abstime now = hrt_absolute_time();

		vehicle_local_position_s lpos{};
		lpos.timestamp = now;
		lpos.z = z;
		lpos.xy_valid = true;
		lpos.z_valid = true;
		lpos.xy_global = global_reference;
		lpos.z_global = global_reference;
		lpos.ref_lat = kLat;
		lpos.ref_lon = kLon;
		lpos.ref_alt = kRefAlt;
		_lpos_pub.publish(lpos);

		vehicle_global_position_s gpos{};
		gpos.timestamp = now;
		gpos.lat = kLat;
		gpos.lon = kLon;
		gpos.alt = alt;
		_gpos_pub.publish(gpos);

		vehicle_land_detected_s land{};
		land.timestamp = now;
		land.landed = false;
		_land_pub.publish(land);

		vehicle_attitude_s attitude{};
		attitude.timestamp = now;
		attitude.q[0] = 1.f;
		_attitude_pub.publish(attitude);

		_home->update(false, false);
	}

	// Publish a constant barometric altitude and a GNSS sample, then run the altitude correction
	void publishGnss(float alt, bool valid = true)
	{
		const hrt_abstime now = hrt_absolute_time();

		vehicle_air_data_s baro{};
		baro.timestamp = now;
		baro.baro_alt_meter = kRefAlt;
		_baro_pub.publish(baro);

		sensor_gps_s gps{};
		gps.timestamp = now;
		gps.latitude_deg = kLat;
		gps.longitude_deg = kLon;
		gps.altitude_msl_m = alt;
		gps.fix_type = valid ? 3 : 0;
		gps.eph = 1.f;
		gps.epv = 1.f;
		gps.s_variance_m_s = 0.1f;
		gps.vel_ned_valid = true;
		gps.vel_d_m_s = 0.f;
		_gps_pub.publish(gps);

		_home->update(false, false);
	}

	home_position_s readHome()
	{
		home_position_s home{};
		EXPECT_TRUE(_home_sub.copy(&home));
		return home;
	}

	// Set an automatic home, take off and let a GNSS altitude jump correct it once
	void startAndCorrectAutomaticHome()
	{
		publishPosition(0.f, kRefAlt, true, true);
		ASSERT_TRUE(_home->setHomePosition());
		_home->setTakeoffTime(hrt_absolute_time());
		publishGnss(kRefAlt); // initialises the velocity integral and the baro/GNSS reference
		const home_position_s before = readHome();
		ASSERT_NEAR(before.alt, kRefAlt, kTolerance);

		publishGnss(kRefAlt + kDrift);
		const home_position_s corrected = readHome();
		ASSERT_NEAR(corrected.alt, kRefAlt + kDrift, kTolerance);
		ASSERT_NEAR(corrected.z, -kDrift, kTolerance);
		ASSERT_EQ(corrected.update_count, before.update_count + 1U);
	}

	struct SavedParam {
		param_t handle{PARAM_INVALID};
		int32_t value{0};
	};

	failsafe_flags_s _flags{}; // must outlive the HomePosition that references it
	std::unique_ptr<HomePosition> _home;
	SavedParam _saved[3] {};
	bool _helpers_initialized{false};

	uORB::Publication<vehicle_local_position_s> _lpos_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<vehicle_global_position_s> _gpos_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<vehicle_attitude_s> _attitude_pub{ORB_ID(vehicle_attitude)};
	uORB::Publication<vehicle_land_detected_s> _land_pub{ORB_ID(vehicle_land_detected)};
	uORB::Publication<vehicle_air_data_s> _baro_pub{ORB_ID(vehicle_air_data)};
	uORB::Publication<sensor_gps_s> _gps_pub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription _home_sub{ORB_ID(home_position)};
};

// Local-only home completed in the air: the global home altitude must be derived from the
// vertical distance between the current position and home, from the fused global position
// as well as from the raw GNSS position. A non-zero home.z separates the sign of the two terms.
class HomePositionInAirCompletionTest : public HomePositionTest,
	public ::testing::WithParamInterface<std::tuple<bool, float>>
{
};

TEST_P(HomePositionInAirCompletionTest, DerivesGlobalHomeAltitudeFromLocalHeightDifference)
{
	const bool use_raw_gnss = std::get<0>(GetParam());
	const float home_z = std::get<1>(GetParam());

	// Home is set while only the local position is valid
	publishPosition(home_z, kRefAlt, false, false);
	ASSERT_TRUE(_home->setHomePosition());
	const home_position_s before = readHome();
	ASSERT_TRUE(before.valid_lpos);
	ASSERT_FALSE(before.valid_hpos);
	ASSERT_FALSE(before.valid_alt);

	// Climb 10 m above the local origin, then a global position becomes available
	publishPosition(-10.f, 110.f, !use_raw_gnss, false);

	if (use_raw_gnss) {
		publishGnss(110.f); // the fused global position stays invalid, the raw GNSS branch is used
	}

	_home->setInAirHomePosition();

	const home_position_s home = readHome();
	EXPECT_TRUE(home.valid_hpos);
	EXPECT_TRUE(home.valid_alt);
	EXPECT_NEAR(home.alt, kRefAlt - home_z, kTolerance);
	EXPECT_FLOAT_EQ(home.z, home_z);
	EXPECT_EQ(home.update_count, before.update_count + 1U);
}

INSTANTIATE_TEST_SUITE_P(FusedAndRawGnss, HomePositionInAirCompletionTest,
			 ::testing::Combine(::testing::Bool(), ::testing::Values(0.f, 4.f)));

// After a correction was applied, a replaced home and a re-initialised velocity integral must
// both start from a clean correction reference: neither the previous drift is applied again
// nor is the correction already stored in the home undone.
enum class CorrectionTransition {
	GnssGapThenNewHome,
	NewHomeWithoutGnssGap,
	GnssGapWithSameHome
};

class HomePositionCorrectionReferenceTest : public HomePositionTest,
	public ::testing::WithParamInterface<CorrectionTransition>
{
};

TEST_P(HomePositionCorrectionReferenceTest, StartsCleanCorrectionReference)
{
	ASSERT_NO_FATAL_FAILURE(startAndCorrectAutomaticHome());

	const bool gnss_gap = GetParam() != CorrectionTransition::NewHomeWithoutGnssGap;
	const bool new_home = GetParam() != CorrectionTransition::GnssGapWithSameHome;

	if (gnss_gap) {
		publishGnss(kRefAlt + kDrift, false); // one invalid sample re-initialises the integral
	}

	if (new_home) {
		publishPosition(-kDrift, kRefAlt + kDrift, true, true);
		ASSERT_TRUE(_home->setHomePosition());
		_home->setTakeoffTime(hrt_absolute_time());
	}

	const home_position_s before = readHome();
	ASSERT_NEAR(before.alt, kRefAlt + kDrift, kTolerance);

	// GNSS altitude is unchanged since the correction: the home must not move
	publishGnss(kRefAlt + kDrift);
	const home_position_s after = readHome();
	EXPECT_FLOAT_EQ(after.alt, before.alt);
	EXPECT_FLOAT_EQ(after.z, before.z);
	EXPECT_EQ(after.update_count, before.update_count);

	// A new GNSS altitude jump is still corrected from the new reference
	publishGnss(kRefAlt + 2.f * kDrift);
	const home_position_s next = readHome();
	EXPECT_NEAR(next.alt, before.alt + kDrift, kTolerance);
	EXPECT_NEAR(next.z, before.z - kDrift, kTolerance);
	EXPECT_EQ(next.update_count, before.update_count + 1U);
}

INSTANTIATE_TEST_SUITE_P(Transitions, HomePositionCorrectionReferenceTest,
			 ::testing::Values(CorrectionTransition::GnssGapThenNewHome,
					 CorrectionTransition::NewHomeWithoutGnssGap,
					 CorrectionTransition::GnssGapWithSameHome));
