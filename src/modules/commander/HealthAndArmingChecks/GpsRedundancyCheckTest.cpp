/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "Common.hpp"
#include "checks/gpsRedundancyCheck.hpp"

#include <uORB/topics/event.h>
#include <uORB/topics/sensor_gps.h>
#include <px4_platform_common/param.h>

using namespace time_literals;

// to run: make tests TESTFILTER=GpsRedundancyCheck

/* EVENT
 * @skip-file
 */

// Base position (PX4 SITL default home)
static constexpr double BASE_LAT = 47.397742;
static constexpr double BASE_LON = 8.545594;

// 1 deg latitude ≈ 111111m
// AGREEING:  ~0.4m north  → below gate of 0.785m (0.70m lever arm + 0.085m eph tolerance)
// DIVERGING: ~2.0m north  → above gate
static constexpr double AGREEING_LAT  = BASE_LAT + 3.6e-6;
static constexpr double DIVERGING_LAT = BASE_LAT + 1.8e-5;

class GpsRedundancyCheckTest : public ::testing::Test
{
public:
	static void SetUpTestSuite()
	{
		// Advertise the event topic so queued events are not lost
		orb_advertise(ORB_ID(event), nullptr);

		// Pre-advertise sensor_gps instances 0 and 1 so they are always
		// available to the check's SubscriptionMultiArray regardless of test order.
		sensor_gps_s gps{};
		_pub0 = orb_advertise(ORB_ID(sensor_gps), &gps);
		int inst;
		_pub1 = orb_advertise_multi(ORB_ID(sensor_gps), &gps, &inst);
	}

	void SetUp() override
	{
		param_control_autosave(false);

		// Lever arm: GPS0 at +35cm forward, GPS1 at -35cm → expected_d = 0.70m
		// With both RTK Fixed (eph=0.02m): gate = 0.70 + 3*√(0.02²+0.02²) = 0.785m
		float f = 0.35f;
		param_set(param_find("SENS_GPS0_OFFX"), &f);
		f = 0.f;
		param_set(param_find("SENS_GPS0_OFFY"), &f);
		f = -0.35f;
		param_set(param_find("SENS_GPS1_OFFX"), &f);
		f = 0.f;
		param_set(param_find("SENS_GPS1_OFFY"), &f);
		int i = 0;
		param_set(param_find("SYS_HAS_NUM_GNSS"), &i);
		param_set(param_find("COM_GPS_LOSS_ACT"), &i);

		// Construct check after params are set so ParamFloat reads correct initial values
		_check = new GpsRedundancyChecks();

		// Reset both instances to "absent" — device_id=0 is filtered by the check
		sensor_gps_s empty{};
		orb_publish(ORB_ID(sensor_gps), _pub0, &empty);
		orb_publish(ORB_ID(sensor_gps), _pub1, &empty);
	}

	void TearDown() override
	{
		delete _check;
		_check = nullptr;
	}

	sensor_gps_s makeGps(double lat, double lon, float eph = 0.02f, uint8_t fix_type = 6)
	{
		sensor_gps_s gps{};
		gps.timestamp         = hrt_absolute_time();
		gps.device_id         = 1;
		gps.latitude_deg      = lat;
		gps.longitude_deg     = lon;
		gps.altitude_ellipsoid_m = 100.0;
		gps.eph               = eph;
		gps.epv               = 0.05f;
		gps.fix_type          = fix_type;
		return gps;
	}

	// Runs one check cycle and returns true if the GPS divergence health warning fired
	bool hasDivergenceWarning(bool armed)
	{
		vehicle_status_s status{};
		status.arming_state = armed
				      ? vehicle_status_s::ARMING_STATE_ARMED
				      : vehicle_status_s::ARMING_STATE_DISARMED;
		Context context{status};
		failsafe_flags_s failsafe_flags{};
		Report reporter{failsafe_flags, 0_s};
		_check->checkAndReport(context, reporter);
		return ((uint64_t)reporter.healthResults().warning
			& (uint64_t)events::px4::enums::health_component_t::gps) != 0;
	}

	// Runs one check cycle and returns true if gnss_lost was set (failsafe would trigger)
	bool hasGnssLost(bool armed)
	{
		vehicle_status_s status{};
		status.arming_state = armed
				      ? vehicle_status_s::ARMING_STATE_ARMED
				      : vehicle_status_s::ARMING_STATE_DISARMED;
		Context context{status};
		failsafe_flags_s failsafe_flags{};
		Report reporter{failsafe_flags, 0_s};
		_check->checkAndReport(context, reporter);
		return failsafe_flags.gnss_lost;
	}

	static orb_advert_t _pub0;
	static orb_advert_t _pub1;
	GpsRedundancyChecks *_check{nullptr};
};

orb_advert_t GpsRedundancyCheckTest::_pub0{nullptr};
orb_advert_t GpsRedundancyCheckTest::_pub1{nullptr};

// No GPS published → active_count=0 → no divergence check
TEST_F(GpsRedundancyCheckTest, NoGnss)
{
	EXPECT_FALSE(hasDivergenceWarning(false));
}

// Only one active receiver → divergence check requires ≥ 2
TEST_F(GpsRedundancyCheckTest, SingleGnss)
{
	sensor_gps_s gps0 = makeGps(BASE_LAT, BASE_LON);
	orb_publish(ORB_ID(sensor_gps), _pub0, &gps0);
	EXPECT_FALSE(hasDivergenceWarning(false));
}

// Two receivers ~0.4m apart (within gate of 0.785m) → no warning
TEST_F(GpsRedundancyCheckTest, TwoGnssAgreeingPrearm)
{
	sensor_gps_s gps0 = makeGps(BASE_LAT, BASE_LON);
	sensor_gps_s gps1 = makeGps(AGREEING_LAT, BASE_LON);
	orb_publish(ORB_ID(sensor_gps), _pub0, &gps0);
	orb_publish(ORB_ID(sensor_gps), _pub1, &gps1);
	EXPECT_FALSE(hasDivergenceWarning(false));
}

// Two receivers ~2.0m apart (beyond gate), pre-arm → immediate warning (sustain=0 pre-arm)
TEST_F(GpsRedundancyCheckTest, TwoGnssDivergingPrearm)
{
	sensor_gps_s gps0 = makeGps(BASE_LAT, BASE_LON);
	sensor_gps_s gps1 = makeGps(DIVERGING_LAT, BASE_LON);
	orb_publish(ORB_ID(sensor_gps), _pub0, &gps0);
	orb_publish(ORB_ID(sensor_gps), _pub1, &gps1);
	EXPECT_TRUE(hasDivergenceWarning(false));
}

// Two receivers ~2.0m apart, in-flight, first call → no warning yet (2s sustain not elapsed)
TEST_F(GpsRedundancyCheckTest, TwoGnssDivergingInflightNotYetSustained)
{
	sensor_gps_s gps0 = makeGps(BASE_LAT, BASE_LON);
	sensor_gps_s gps1 = makeGps(DIVERGING_LAT, BASE_LON);
	orb_publish(ORB_ID(sensor_gps), _pub0, &gps0);
	orb_publish(ORB_ID(sensor_gps), _pub1, &gps1);
	EXPECT_FALSE(hasDivergenceWarning(true));
}

// GPS1 fix_type=2 (2D only) → treated as inactive → only 1 active receiver → no divergence check
TEST_F(GpsRedundancyCheckTest, FixTypeBelow3TreatedAsInactive)
{
	sensor_gps_s gps0 = makeGps(BASE_LAT, BASE_LON);
	sensor_gps_s gps1 = makeGps(DIVERGING_LAT, BASE_LON, 0.02f, 2);
	orb_publish(ORB_ID(sensor_gps), _pub0, &gps0);
	orb_publish(ORB_ID(sensor_gps), _pub1, &gps1);
	EXPECT_FALSE(hasDivergenceWarning(false));
}

// Divergence with SYS_HAS_NUM_GNSS=2 and COM_GPS_LOSS_ACT>0 → gnss_lost set (failsafe fires)
TEST_F(GpsRedundancyCheckTest, DivergenceWithRedundancyRequiredSetsGnssLost)
{
	int i = 2;
	param_set(param_find("SYS_HAS_NUM_GNSS"), &i);
	i = 1; // Return
	param_set(param_find("COM_GPS_LOSS_ACT"), &i);
	delete _check;
	_check = new GpsRedundancyChecks();

	sensor_gps_s gps0 = makeGps(BASE_LAT, BASE_LON);
	sensor_gps_s gps1 = makeGps(DIVERGING_LAT, BASE_LON);
	orb_publish(ORB_ID(sensor_gps), _pub0, &gps0);
	orb_publish(ORB_ID(sensor_gps), _pub1, &gps1);
	EXPECT_TRUE(hasGnssLost(false));
}

// Divergence with SYS_HAS_NUM_GNSS=0 and COM_GPS_LOSS_ACT>0 → warning only, gnss_lost not set
TEST_F(GpsRedundancyCheckTest, DivergenceWithoutRedundancyRequiredWarnsOnly)
{
	int i = 0;
	param_set(param_find("SYS_HAS_NUM_GNSS"), &i);
	i = 1; // Return
	param_set(param_find("COM_GPS_LOSS_ACT"), &i);
	delete _check;
	_check = new GpsRedundancyChecks();

	sensor_gps_s gps0 = makeGps(BASE_LAT, BASE_LON);
	sensor_gps_s gps1 = makeGps(DIVERGING_LAT, BASE_LON);
	orb_publish(ORB_ID(sensor_gps), _pub0, &gps0);
	orb_publish(ORB_ID(sensor_gps), _pub1, &gps1);
	EXPECT_FALSE(hasGnssLost(false));
	EXPECT_TRUE(hasDivergenceWarning(false));
}

// Divergence then recovery: timer resets and warning stops
TEST_F(GpsRedundancyCheckTest, DivergenceClearsAfterRecovery)
{
	// First: diverging pre-arm → warning fires
	sensor_gps_s gps0 = makeGps(BASE_LAT, BASE_LON);
	sensor_gps_s gps1 = makeGps(DIVERGING_LAT, BASE_LON);
	orb_publish(ORB_ID(sensor_gps), _pub0, &gps0);
	orb_publish(ORB_ID(sensor_gps), _pub1, &gps1);
	EXPECT_TRUE(hasDivergenceWarning(false));

	// Then: receivers agree → warning clears
	gps0 = makeGps(BASE_LAT, BASE_LON);
	gps1 = makeGps(AGREEING_LAT, BASE_LON);
	orb_publish(ORB_ID(sensor_gps), _pub0, &gps0);
	orb_publish(ORB_ID(sensor_gps), _pub1, &gps1);
	EXPECT_FALSE(hasDivergenceWarning(false));
}
