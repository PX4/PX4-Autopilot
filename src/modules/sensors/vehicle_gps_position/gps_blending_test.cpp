/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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
 * Test code for the GPS blending logic
 * Run this test only using make tests TESTFILTER=GpsBlending
 *
 * @author Mathieu Bresciani <mathieu@auterion.com>
 */

#include <gtest/gtest.h>
#include <matrix/matrix/math.hpp>

#include "gps_blending.hpp"

using matrix::Vector3f;

class GpsBlendingTest : public ::testing::Test
{
public:
	sensor_gps_s getDefaultGpsData();
	void runSeconds(float duration_s, GpsBlending &gps_blending, sensor_gps_s &gps_data, int instance);
	void runSeconds(float duration_s, GpsBlending &gps_blending, sensor_gps_s &gps_data0, sensor_gps_s &gps_data1);

	uint64_t _time_now_us{1000000};
};

sensor_gps_s GpsBlendingTest::getDefaultGpsData()
{
	sensor_gps_s gps_data{};
	gps_data.timestamp = _time_now_us - 10e3;
	gps_data.time_utc_usec = 0;
	gps_data.latitude_deg = 47.0;
	gps_data.longitude_deg = 9.0;
	gps_data.altitude_msl_m = 800.0;
	gps_data.altitude_ellipsoid_m = 800.0;
	gps_data.s_variance_m_s = 0.2f;
	gps_data.c_variance_rad = 0.5f;
	gps_data.eph = 0.7f;
	gps_data.epv = 1.2f;
	gps_data.hdop = 1.f;
	gps_data.vdop = 1.f;
	gps_data.noise_per_ms = 20;
	gps_data.jamming_indicator = 40;
	gps_data.vel_m_s = 1.f;
	gps_data.vel_n_m_s = 1.f;
	gps_data.vel_e_m_s = 1.f;
	gps_data.vel_d_m_s = 1.f;
	gps_data.cog_rad = 0.f;
	gps_data.timestamp_time_relative = 0;
	gps_data.heading = NAN;
	gps_data.heading_offset = 0.f;
	gps_data.fix_type = 4;
	gps_data.vel_ned_valid = true;
	gps_data.satellites_used = 8;

	return gps_data;
}

void GpsBlendingTest::runSeconds(float duration_s, GpsBlending &gps_blending, sensor_gps_s &gps_data, int instance)
{
	const float dt = 0.1;
	const uint64_t dt_us = static_cast<uint64_t>(dt * 1e6f);

	for (int k = 0; k < static_cast<int>(duration_s / dt); k++) {
		gps_blending.setGpsData(gps_data, instance);
		gps_blending.update(_time_now_us);

		_time_now_us += dt_us;
		gps_data.timestamp += dt_us;
	}
}

void GpsBlendingTest::runSeconds(float duration_s, GpsBlending &gps_blending, sensor_gps_s &gps_data0,
				 sensor_gps_s &gps_data1)
{
	const float dt = 0.1;
	const uint64_t dt_us = static_cast<uint64_t>(dt * 1e6f);

	for (int k = 0; k < static_cast<int>(duration_s / dt); k++) {
		gps_blending.setGpsData(gps_data0, 0);
		gps_blending.setGpsData(gps_data1, 1);

		gps_blending.update(_time_now_us);

		_time_now_us += dt_us;
		gps_data0.timestamp += dt_us;
		gps_data1.timestamp += dt_us;
	}
}

TEST_F(GpsBlendingTest, noData)
{
	GpsBlending gps_blending;

	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
	EXPECT_FALSE(gps_blending.isNewOutputDataAvailable());

	gps_blending.update(_time_now_us);

	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
	EXPECT_FALSE(gps_blending.isNewOutputDataAvailable());
}

TEST_F(GpsBlendingTest, singleReceiver)
{
	GpsBlending gps_blending;

	gps_blending.setPrimaryInstance(-1);
	sensor_gps_s gps_data = getDefaultGpsData();

	gps_blending.setGpsData(gps_data, 1);
	gps_blending.update(_time_now_us);

	_time_now_us += 200e3;
	gps_data.timestamp = _time_now_us - 10e3;
	gps_blending.setGpsData(gps_data, 1);
	gps_blending.update(_time_now_us);

	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_EQ(gps_blending.getNumberOfGpsSuitableForBlending(), 1);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	// BUT IF: a second update is called without data
	gps_blending.update(_time_now_us);

	// THEN: no new data should be available
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_EQ(gps_blending.getNumberOfGpsSuitableForBlending(), 1);
	EXPECT_FALSE(gps_blending.isNewOutputDataAvailable());
}

TEST_F(GpsBlendingTest, dualReceiverNoBlending)
{
	GpsBlending gps_blending;

	// GIVEN: two receivers with the same prioity, gps0 only has a 2D fix
	gps_blending.setPrimaryInstance(-1);
	sensor_gps_s gps_data0 = getDefaultGpsData();
	sensor_gps_s gps_data1 = getDefaultGpsData();

	gps_data0.fix_type = sensor_gps_s::FIX_TYPE_2D;
	gps_blending.setGpsData(gps_data0, 0);
	gps_blending.setGpsData(gps_data1, 1);
	gps_blending.update(_time_now_us);

	// THEN: gps1 should be selected because it is the only one with a 3D fix
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_EQ(gps_blending.getNumberOfGpsSuitableForBlending(), 2);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	_time_now_us += 200e3;
	gps_data0.timestamp = _time_now_us - 10e3;
	gps_data1.timestamp = _time_now_us - 10e3;
	gps_data0.fix_type = sensor_gps_s::FIX_TYPE_3D;
	gps_data0.satellites_used = gps_data1.satellites_used + 2;
	gps_blending.setGpsData(gps_data0, 0);
	gps_blending.setGpsData(gps_data1, 1);
	gps_blending.update(_time_now_us);

	// THEN: gps1 should still be selected, the satellite count isn't used for ranking
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_EQ(gps_blending.getNumberOfGpsSuitableForBlending(), 2);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());
}

TEST_F(GpsBlendingTest, dualReceiverBlendingHPos)
{
	GpsBlending gps_blending;

	sensor_gps_s gps_data0 = getDefaultGpsData();
	sensor_gps_s gps_data1 = getDefaultGpsData();

	gps_blending.setBlendingUseHPosAccuracy(true);

	gps_data1.eph = gps_data0.eph / 2.f;
	gps_blending.setGpsData(gps_data0, 0);
	gps_blending.setGpsData(gps_data1, 1);
	gps_blending.update(_time_now_us);

	// THEN: the blended instance should be selected (2)
	// and the eph should be adjusted
	EXPECT_EQ(gps_blending.getSelectedGps(), 2);
	EXPECT_EQ(gps_blending.getNumberOfGpsSuitableForBlending(), 2);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());
	EXPECT_LT(gps_blending.getOutputGpsData().eph, gps_data0.eph);
	EXPECT_FLOAT_EQ(gps_blending.getOutputGpsData().eph, gps_data1.eph); // TODO: should be greater than
	EXPECT_EQ(gps_blending.getOutputGpsData().timestamp, gps_data0.timestamp);
	EXPECT_EQ(gps_blending.getOutputGpsData().timestamp_sample, gps_data0.timestamp_sample);
	EXPECT_EQ(gps_blending.getOutputGpsData().latitude_deg, gps_data0.latitude_deg);
	EXPECT_EQ(gps_blending.getOutputGpsData().latitude_deg, gps_data0.latitude_deg);
	EXPECT_EQ(gps_blending.getOutputGpsData().altitude_msl_m, gps_data0.altitude_msl_m);
}

TEST_F(GpsBlendingTest, dualReceiverFailover)
{
	GpsBlending gps_blending;

	// GIVEN: a dual GPS setup with the first instance (0)
	// set as primary
	gps_blending.setPrimaryInstance(0);
	gps_blending.setBlendingUseSpeedAccuracy(false);
	gps_blending.setBlendingUseHPosAccuracy(false);
	gps_blending.setBlendingUseVPosAccuracy(false);

	// WHEN: only the secondary receiver is available
	sensor_gps_s gps_data1 = getDefaultGpsData();

	const float duration_s = 10.f;
	runSeconds(duration_s, gps_blending, gps_data1, 1);

	// THEN: the secondary instance as the primary one is not available
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_EQ(gps_blending.getNumberOfGpsSuitableForBlending(), 1);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	// BUT WHEN: the data of the primary receiver is avaialbe
	sensor_gps_s gps_data0 = getDefaultGpsData();
	runSeconds(1.f, gps_blending, gps_data0, gps_data1);

	// THEN: the secondary instance is kept for the hold time
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);

	runSeconds(1.5f, gps_blending, gps_data0, gps_data1);

	// THEN: the primary instance is selected and the data
	// is available
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
	EXPECT_EQ(gps_blending.getNumberOfGpsSuitableForBlending(), 2);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	runSeconds(duration_s, gps_blending, gps_data0, gps_data1);

	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	// BUT WHEN: the primary receiver isn't available anymore
	runSeconds(duration_s, gps_blending, gps_data1, 1);

	// THEN: the data of the secondary receiver can be used
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	// AND IF: the primary receiver is available again
	gps_data0.timestamp = gps_data1.timestamp;

	runSeconds(3.f, gps_blending, gps_data0, gps_data1);

	// THEN: the primary receiver should be used again after the hold time
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	// BUT IF: the secondary receiver has a better fix type and accuracy than the primary one
	gps_data1.fix_type = sensor_gps_s::FIX_TYPE_RTK_FIXED;
	gps_data1.eph = 0.02f;
	gps_data1.epv = 0.03f;

	runSeconds(5.f, gps_blending, gps_data0, gps_data1);

	// THEN: the selector shouldn't switch again as the primary one is available
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	// BUT IF: the primary receiver looses its fix
	gps_data0.fix_type = 1;

	runSeconds(1.f, gps_blending, gps_data0, gps_data1);

	// THEN: the selector should switch as the primary one is unable to provide correct data
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());
}

TEST_F(GpsBlendingTest, preferredFailoverAfterHold)
{
	GpsBlending gps_blending;

	// GIVEN: two receivers, gps0 preferred
	gps_blending.setPrimaryInstance(0);
	gps_blending.setBlendingUseSpeedAccuracy(false);
	gps_blending.setBlendingUseHPosAccuracy(false);
	gps_blending.setBlendingUseVPosAccuracy(false);

	sensor_gps_s gps_data0 = getDefaultGpsData();
	sensor_gps_s gps_data1 = getDefaultGpsData();

	runSeconds(1.f, gps_blending, gps_data0, gps_data1);

	EXPECT_EQ(gps_blending.getSelectedGps(), 0);

	// WHEN: the preferred receiver doesn't meet the EKF2 eph requirement
	gps_data0.eph = 4.f;

	runSeconds(1.5f, gps_blending, gps_data0, gps_data1);

	// THEN: it is kept for the hold time
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);

	// AND WHEN: it meets the requirement again before the hold time elapses, then fails again
	gps_data0.eph = 0.7f;

	runSeconds(0.5f, gps_blending, gps_data0, gps_data1);

	gps_data0.eph = 4.f;

	runSeconds(1.5f, gps_blending, gps_data0, gps_data1);

	// THEN: no switch happens as the hold time restarted
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);

	// BUT WHEN: it doesn't meet the requirement for longer than the hold time
	runSeconds(1.f, gps_blending, gps_data0, gps_data1);

	// THEN: the receiver meeting the requirements is selected
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());
}

TEST_F(GpsBlendingTest, preferredFailoverOnlyToBetterQuality)
{
	GpsBlending gps_blending;

	gps_blending.setPrimaryInstance(0);
	gps_blending.setBlendingUseSpeedAccuracy(false);
	gps_blending.setBlendingUseHPosAccuracy(false);
	gps_blending.setBlendingUseVPosAccuracy(false);

	// GIVEN: two receivers, gps0 preferred, neither meeting the EKF2 eph requirement
	sensor_gps_s gps_data0 = getDefaultGpsData();
	sensor_gps_s gps_data1 = getDefaultGpsData();
	gps_data0.eph = 4.f;
	gps_data1.eph = 3.5f;

	runSeconds(5.f, gps_blending, gps_data0, gps_data1);

	// THEN: the preferred receiver is kept as the other one doesn't meet the requirements either
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);

	// WHEN: the preferred receiver loses its 3D fix
	gps_data0.fix_type = sensor_gps_s::FIX_TYPE_2D;

	runSeconds(0.1f, gps_blending, gps_data0, gps_data1);

	// THEN: the selector switches immediately to the receiver that still has a 3D fix
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());
}

TEST_F(GpsBlendingTest, preferredReturnWithHold)
{
	GpsBlending gps_blending;

	gps_blending.setPrimaryInstance(0);
	gps_blending.setBlendingUseSpeedAccuracy(false);
	gps_blending.setBlendingUseHPosAccuracy(false);
	gps_blending.setBlendingUseVPosAccuracy(false);

	// GIVEN: gps0 preferred but not available
	sensor_gps_s gps_data1 = getDefaultGpsData();

	runSeconds(3.f, gps_blending, gps_data1, 1);

	EXPECT_EQ(gps_blending.getSelectedGps(), 1);

	// WHEN: gps0 publishes but doesn't meet the EKF2 eph requirement
	sensor_gps_s gps_data0 = getDefaultGpsData();
	gps_data0.eph = 4.f;

	runSeconds(5.f, gps_blending, gps_data0, gps_data1);

	// THEN: gps1 is kept
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);

	// WHEN: gps0 meets the requirements
	gps_data0.eph = 0.7f;

	runSeconds(1.f, gps_blending, gps_data0, gps_data1);

	// THEN: gps1 is kept for the hold time
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);

	// AND WHEN: gps0 fails the requirements again before the hold time elapses, then meets them again
	gps_data0.eph = 4.f;

	runSeconds(0.5f, gps_blending, gps_data0, gps_data1);

	gps_data0.eph = 0.7f;

	runSeconds(1.5f, gps_blending, gps_data0, gps_data1);

	// THEN: gps1 is still kept as the hold time restarted
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);

	// BUT WHEN: gps0 meets the requirements for longer than the hold time
	runSeconds(1.f, gps_blending, gps_data0, gps_data1);

	// THEN: the selection returns to the preferred receiver
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());
}

TEST_F(GpsBlendingTest, preferredAfterBlending)
{
	GpsBlending gps_blending;

	// GIVEN: two receivers being blended, gps0 preferred
	gps_blending.setPrimaryInstance(0);
	gps_blending.setBlendingUseHPosAccuracy(true);

	sensor_gps_s gps_data0 = getDefaultGpsData();
	sensor_gps_s gps_data1 = getDefaultGpsData();

	runSeconds(1.f, gps_blending, gps_data0, gps_data1);

	EXPECT_EQ(gps_blending.getSelectedGps(), 2);

	// WHEN: the preferred receiver loses its fix and blending stops
	gps_data0.fix_type = sensor_gps_s::FIX_TYPE_NONE;

	runSeconds(0.1f, gps_blending, gps_data0, gps_data1);

	// THEN: the remaining receiver is selected right away
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());
}

TEST_F(GpsBlendingTest, rankedAccuracyWithHold)
{
	GpsBlending gps_blending;

	// GIVEN: two receivers with the same priority and the same accuracy
	gps_blending.setPrimaryInstance(-1);
	gps_blending.setBlendingUseSpeedAccuracy(false);
	gps_blending.setBlendingUseHPosAccuracy(false);
	gps_blending.setBlendingUseVPosAccuracy(false);

	sensor_gps_s gps_data0 = getDefaultGpsData();
	sensor_gps_s gps_data1 = getDefaultGpsData();

	runSeconds(2.f, gps_blending, gps_data0, gps_data1);

	EXPECT_EQ(gps_blending.getSelectedGps(), 0);

	// WHEN: gps1 reports a slightly better accuracy and a better fix type
	gps_data1.eph = 0.6f;
	gps_data1.epv = 1.0f;
	gps_data1.fix_type = sensor_gps_s::FIX_TYPE_RTK_FIXED;

	runSeconds(5.f, gps_blending, gps_data0, gps_data1);

	// THEN: the selection is kept, the difference is within the switching margin
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);

	// WHEN: gps1 reports a clearly better accuracy
	gps_data1.eph = 0.3f;
	gps_data1.epv = 0.5f;

	runSeconds(1.f, gps_blending, gps_data0, gps_data1);

	// THEN: the selection is held for the hold time
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);

	// AND WHEN: gps1 degrades again before the hold time elapses
	gps_data1.eph = gps_data0.eph;
	gps_data1.epv = gps_data0.epv;

	runSeconds(0.5f, gps_blending, gps_data0, gps_data1);

	// AND: is clearly better again, but not for the full hold time
	gps_data1.eph = 0.3f;
	gps_data1.epv = 0.5f;

	runSeconds(1.5f, gps_blending, gps_data0, gps_data1);

	// THEN: no switch happens as the hold time restarted
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);

	// BUT WHEN: gps1 stays clearly better for longer than the hold time
	runSeconds(1.f, gps_blending, gps_data0, gps_data1);

	// THEN: the more accurate receiver is selected
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	// AND WHEN: gps0 becomes slightly more accurate than gps1
	gps_data0.eph = 0.25f;
	gps_data0.epv = 0.45f;

	runSeconds(5.f, gps_blending, gps_data0, gps_data1);

	// THEN: the selection is kept
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
}

TEST_F(GpsBlendingTest, rankedUpdateRate)
{
	GpsBlending gps_blending;

	// GIVEN: two receivers with the same accuracy, gps0 at 5 Hz and gps1 at 10 Hz
	gps_blending.setPrimaryInstance(-1);
	gps_blending.setBlendingUseSpeedAccuracy(false);
	gps_blending.setBlendingUseHPosAccuracy(false);
	gps_blending.setBlendingUseVPosAccuracy(false);

	sensor_gps_s gps_data0 = getDefaultGpsData();
	sensor_gps_s gps_data1 = getDefaultGpsData();

	const uint64_t dt_us = 100_ms;

	auto run = [&](int steps) {
		for (int k = 0; k < steps; k++) {
			if ((k % 2) == 0) {
				gps_blending.setGpsData(gps_data0, 0);
				gps_data0.timestamp += 2 * dt_us;
			}

			gps_blending.setGpsData(gps_data1, 1);
			gps_data1.timestamp += dt_us;

			gps_blending.update(_time_now_us);
			_time_now_us += dt_us;
		}
	};

	run(10);

	// THEN: the selection is held for the hold time
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);

	run(30);

	// THEN: the receiver with the higher update rate is selected
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);

	// WHEN: the slower receiver is clearly more accurate
	gps_data0.eph = 0.3f;
	gps_data0.epv = 0.5f;

	run(30);

	// THEN: the accuracy has precedence over the update rate
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
}

TEST_F(GpsBlendingTest, rankedSwitchOnFailure)
{
	GpsBlending gps_blending;

	gps_blending.setPrimaryInstance(-1);
	gps_blending.setBlendingUseSpeedAccuracy(false);
	gps_blending.setBlendingUseHPosAccuracy(false);
	gps_blending.setBlendingUseVPosAccuracy(false);

	// GIVEN: two equivalent receivers, gps0 selected
	sensor_gps_s gps_data0 = getDefaultGpsData();
	sensor_gps_s gps_data1 = getDefaultGpsData();

	runSeconds(2.f, gps_blending, gps_data0, gps_data1);

	EXPECT_EQ(gps_blending.getSelectedGps(), 0);

	// WHEN: gps0 loses its fix
	gps_data0.fix_type = sensor_gps_s::FIX_TYPE_NONE;

	runSeconds(0.1f, gps_blending, gps_data0, gps_data1);

	// THEN: the selector switches immediately
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);

	// WHEN: gps0 recovers and gps1 no longer meets the EKF2 requirements
	gps_data0.fix_type = gps_data1.fix_type;
	runSeconds(1.f, gps_blending, gps_data0, gps_data1);

	gps_data1.eph = 4.f;

	runSeconds(1.f, gps_blending, gps_data0, gps_data1);

	// THEN: the selection is held for the hold time
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);

	runSeconds(1.5f, gps_blending, gps_data0, gps_data1);

	// THEN: the receiver meeting the requirements is selected
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);

	// WHEN: gps1 recovers and gps0 stops publishing
	gps_data1.eph = gps_data0.eph;
	runSeconds(1.f, gps_blending, gps_data0, gps_data1);

	EXPECT_EQ(gps_blending.getSelectedGps(), 0);

	runSeconds(3.f, gps_blending, gps_data1, 1);

	// THEN: the selector switches once gps0 timed out
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());
}

TEST_F(GpsBlendingTest, rankedMinimumRequirements)
{
	GpsBlending gps_blending;

	gps_blending.setPrimaryInstance(-1);
	gps_blending.setBlendingUseSpeedAccuracy(false);
	gps_blending.setBlendingUseHPosAccuracy(false);
	gps_blending.setBlendingUseVPosAccuracy(false);
	gps_blending.setMinimumRequirements(sensor_gps_s::FIX_TYPE_3D, 1.f, 3.f);

	// GIVEN: gps0 has a better fix type but doesn't meet the eph requirement,
	// gps1 meets all requirements
	sensor_gps_s gps_data0 = getDefaultGpsData();
	sensor_gps_s gps_data1 = getDefaultGpsData();
	gps_data0.fix_type = sensor_gps_s::FIX_TYPE_RTK_FLOAT;
	gps_data0.eph = 1.5f;
	gps_data1.fix_type = sensor_gps_s::FIX_TYPE_3D;
	gps_data1.eph = 0.8f;

	runSeconds(1.f, gps_blending, gps_data0, gps_data1);

	// THEN: gps0, selected by default, is kept for the hold time
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);

	runSeconds(1.5f, gps_blending, gps_data0, gps_data1);

	// THEN: the receiver meeting the requirements is selected
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);

	// WHEN: gps0 meets the requirements and is clearly more accurate
	gps_data0.eph = 0.3f;
	gps_data0.epv = 0.5f;

	runSeconds(1.f, gps_blending, gps_data0, gps_data1);

	// THEN: the selection is held for the hold time
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);

	runSeconds(1.5f, gps_blending, gps_data0, gps_data1);

	// THEN: the more accurate receiver wins after the hold time
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
}

TEST_F(GpsBlendingTest, rankedExtrapolatedFixNotUsable)
{
	GpsBlending gps_blending;

	gps_blending.setPrimaryInstance(-1);
	gps_blending.setBlendingUseSpeedAccuracy(false);
	gps_blending.setBlendingUseHPosAccuracy(false);
	gps_blending.setBlendingUseVPosAccuracy(false);

	// GIVEN: gps0 extrapolating (dead reckoning), gps1 with a 3D fix
	sensor_gps_s gps_data0 = getDefaultGpsData();
	sensor_gps_s gps_data1 = getDefaultGpsData();
	gps_data0.fix_type = sensor_gps_s::FIX_TYPE_EXTRAPOLATED;
	gps_data0.eph = 0.1f;
	gps_data0.epv = 0.1f;
	gps_data1.fix_type = sensor_gps_s::FIX_TYPE_3D;

	runSeconds(3.f, gps_blending, gps_data0, gps_data1);

	// THEN: the 3D fix is selected even if the extrapolated fix type value and accuracy are better
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);

	// WHEN: gps1 starts extrapolating and gps0 gets a 3D fix
	gps_data0.fix_type = sensor_gps_s::FIX_TYPE_3D;
	gps_data1.fix_type = sensor_gps_s::FIX_TYPE_EXTRAPOLATED;

	runSeconds(0.1f, gps_blending, gps_data0, gps_data1);

	// THEN: the switch happens immediately
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
}

TEST_F(GpsBlendingTest, rankedAfterBlending)
{
	GpsBlending gps_blending;

	// GIVEN: two receivers being blended
	gps_blending.setPrimaryInstance(-1);
	gps_blending.setBlendingUseHPosAccuracy(true);

	sensor_gps_s gps_data0 = getDefaultGpsData();
	sensor_gps_s gps_data1 = getDefaultGpsData();

	runSeconds(1.f, gps_blending, gps_data0, gps_data1);

	EXPECT_EQ(gps_blending.getSelectedGps(), 2);

	// WHEN: gps0 loses its fix and blending stops
	gps_data0.fix_type = sensor_gps_s::FIX_TYPE_NONE;

	runSeconds(0.1f, gps_blending, gps_data0, gps_data1);

	// THEN: the remaining receiver is selected right away
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());
}

TEST_F(GpsBlendingTest, singleReceiverAntennaOffset)
{
	GpsBlending gps_blending;

	gps_blending.setPrimaryInstance(-1);
	sensor_gps_s gps_data = getDefaultGpsData();

	const Vector3f offset0(0.1f, 0.0f, -0.05f);
	gps_blending.setAntennaOffset(offset0, 1);

	gps_blending.setGpsData(gps_data, 1);
	gps_blending.update(_time_now_us);

	_time_now_us += 200e3;
	gps_data.timestamp = _time_now_us - 10e3;
	gps_blending.setGpsData(gps_data, 1);
	gps_blending.update(_time_now_us);

	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	const Vector3f &out = gps_blending.getOutputAntennaOffset();
	EXPECT_FLOAT_EQ(out(0), offset0(0));
	EXPECT_FLOAT_EQ(out(1), offset0(1));
	EXPECT_FLOAT_EQ(out(2), offset0(2));
}

TEST_F(GpsBlendingTest, dualReceiverBlendedAntennaOffset)
{
	GpsBlending gps_blending;

	sensor_gps_s gps_data0 = getDefaultGpsData();
	sensor_gps_s gps_data1 = getDefaultGpsData();

	gps_blending.setBlendingUseHPosAccuracy(true);

	// Equal accuracy → equal weights (0.5 each)
	const Vector3f offset0(0.1f, 0.0f, -0.05f);
	const Vector3f offset1(-0.1f, 0.0f, -0.05f);
	gps_blending.setAntennaOffset(offset0, 0);
	gps_blending.setAntennaOffset(offset1, 1);

	gps_blending.setGpsData(gps_data0, 0);
	gps_blending.setGpsData(gps_data1, 1);
	gps_blending.update(_time_now_us);

	EXPECT_EQ(gps_blending.getSelectedGps(), 2); // blended

	const Vector3f &out = gps_blending.getOutputAntennaOffset();
	// Equal weights → average of offsets
	EXPECT_NEAR(out(0), 0.0f, 1e-5f);
	EXPECT_NEAR(out(1), 0.0f, 1e-5f);
	EXPECT_NEAR(out(2), -0.05f, 1e-5f);
}

TEST_F(GpsBlendingTest, failoverAntennaOffset)
{
	GpsBlending gps_blending;

	gps_blending.setPrimaryInstance(0);
	gps_blending.setBlendingUseSpeedAccuracy(false);
	gps_blending.setBlendingUseHPosAccuracy(false);
	gps_blending.setBlendingUseVPosAccuracy(false);

	const Vector3f offset0(0.1f, 0.0f, 0.0f);
	const Vector3f offset1(-0.1f, 0.0f, 0.0f);
	gps_blending.setAntennaOffset(offset0, 0);
	gps_blending.setAntennaOffset(offset1, 1);

	// Only secondary available
	sensor_gps_s gps_data1 = getDefaultGpsData();
	runSeconds(10.f, gps_blending, gps_data1, 1);

	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_FLOAT_EQ(gps_blending.getOutputAntennaOffset()(0), offset1(0));

	// Now primary becomes available for longer than the hold time
	sensor_gps_s gps_data0 = getDefaultGpsData();
	gps_data0.timestamp = gps_data1.timestamp;
	runSeconds(3.f, gps_blending, gps_data0, gps_data1);

	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
	EXPECT_FLOAT_EQ(gps_blending.getOutputAntennaOffset()(0), offset0(0));
}

TEST_F(GpsBlendingTest, dualReceiverAsymmetricWeightAntennaOffset)
{
	GpsBlending gps_blending;

	sensor_gps_s gps_data0 = getDefaultGpsData();
	sensor_gps_s gps_data1 = getDefaultGpsData();

	gps_blending.setBlendingUseHPosAccuracy(true);

	// gps0 has twice the accuracy of gps1 → higher weight
	gps_data0.eph = 0.5f;
	gps_data1.eph = 1.0f;

	const Vector3f offset0(0.2f, 0.0f, -0.1f);
	const Vector3f offset1(-0.2f, 0.0f, 0.1f);
	gps_blending.setAntennaOffset(offset0, 0);
	gps_blending.setAntennaOffset(offset1, 1);

	gps_blending.setGpsData(gps_data0, 0);
	gps_blending.setGpsData(gps_data1, 1);
	gps_blending.update(_time_now_us);

	EXPECT_EQ(gps_blending.getSelectedGps(), 2); // blended

	// hpos weights: inverse variance → w0 = 1/(0.5^2) = 4, w1 = 1/(1.0^2) = 1
	// normalized: w0 = 0.8, w1 = 0.2
	// blended offset = 0.8 * (0.2, 0, -0.1) + 0.2 * (-0.2, 0, 0.1)
	//                = (0.16, 0, -0.08) + (-0.04, 0, 0.02) = (0.12, 0, -0.06)
	const Vector3f &out = gps_blending.getOutputAntennaOffset();
	EXPECT_NEAR(out(0), 0.12f, 1e-5f);
	EXPECT_NEAR(out(1), 0.0f, 1e-5f);
	EXPECT_NEAR(out(2), -0.06f, 1e-5f);
}

TEST_F(GpsBlendingTest, blendingFallthroughAntennaOffset)
{
	GpsBlending gps_blending;

	// Enable blending, but give one receiver eph=0 so can_do_blending is false
	gps_blending.setPrimaryInstance(-1);
	gps_blending.setBlendingUseHPosAccuracy(true);
	gps_blending.setBlendingUseSpeedAccuracy(false);
	gps_blending.setBlendingUseVPosAccuracy(false);

	const Vector3f offset0(0.15f, -0.05f, 0.0f);
	const Vector3f offset1(-0.15f, 0.05f, 0.0f);
	gps_blending.setAntennaOffset(offset0, 0);
	gps_blending.setAntennaOffset(offset1, 1);

	sensor_gps_s gps_data0 = getDefaultGpsData();
	sensor_gps_s gps_data1 = getDefaultGpsData();

	// eph=0 on both → horizontal_accuracy_sum_sq=0 → can_do_blending=false → fallthrough
	gps_data0.eph = 0.0f;
	gps_data1.eph = 0.0f;

	// gps0 only has a 2D fix → gps1 wins non-blending selection
	gps_data0.fix_type = sensor_gps_s::FIX_TYPE_2D;

	gps_blending.setGpsData(gps_data0, 0);
	gps_blending.setGpsData(gps_data1, 1);
	gps_blending.update(_time_now_us);

	_time_now_us += 200e3;
	gps_data0.timestamp = _time_now_us - 10e3;
	gps_data1.timestamp = _time_now_us - 10e3;
	gps_blending.setGpsData(gps_data0, 0);
	gps_blending.setGpsData(gps_data1, 1);
	gps_blending.update(_time_now_us);

	// Falls through to non-blending path, gps1 selected by the receiver ranking
	EXPECT_LT(gps_blending.getSelectedGps(), 2); // not blended
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);

	const Vector3f &out = gps_blending.getOutputAntennaOffset();
	EXPECT_FLOAT_EQ(out(0), offset1(0));
	EXPECT_FLOAT_EQ(out(1), offset1(1));
	EXPECT_FLOAT_EQ(out(2), offset1(2));
}

TEST_F(GpsBlendingTest, dualReceiverNoBlendingStaleFlag)
{
	GpsBlending gps_blending;

	// GIVEN: two receivers, no blending (no accuracy metrics enabled)
	gps_blending.setPrimaryInstance(-1);
	gps_blending.setBlendingUseSpeedAccuracy(false);
	gps_blending.setBlendingUseHPosAccuracy(false);
	gps_blending.setBlendingUseVPosAccuracy(false);

	sensor_gps_s gps_data0 = getDefaultGpsData();
	sensor_gps_s gps_data1 = getDefaultGpsData();

	gps_data0.fix_type = sensor_gps_s::FIX_TYPE_2D; // gps1 wins selection

	// First update: both receivers provide data, gps1 is selected
	gps_blending.setGpsData(gps_data0, 0);
	gps_blending.setGpsData(gps_data1, 1);
	gps_blending.update(_time_now_us);

	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	// Second update: NO new data from either receiver
	// With the bug (_gps_updated[gps_select_index] instead of [i]),
	// the non-selected instance's flag was never cleared, so this
	// would spuriously report new data available.
	gps_blending.update(_time_now_us);

	EXPECT_FALSE(gps_blending.isNewOutputDataAvailable());
}

TEST_F(GpsBlendingTest, dualReceiverUTCTime)
{
	GpsBlending gps_blending;
	sensor_gps_s gps_data0 = getDefaultGpsData();
	sensor_gps_s gps_data1 = getDefaultGpsData();

	// WHEN: Only GPS1 has a nonzero UTC time
	gps_blending = GpsBlending();
	gps_data1.time_utc_usec = 1700000000000000ULL;
	gps_blending.setGpsData(gps_data0, 0);
	gps_blending.setGpsData(gps_data1, 1);
	gps_blending.setBlendingUseHPosAccuracy(true);
	gps_blending.update(_time_now_us);
	// THEN: GPS 1 time should be used
	EXPECT_EQ(gps_blending.getOutputGpsData().time_utc_usec, gps_data1.time_utc_usec);

	// WHEN: Both GPSes have a nonzero UTC time
	gps_blending = GpsBlending();
	gps_data0.time_utc_usec = 1700000000001000ULL;
	gps_data1.time_utc_usec = 1700000000000000ULL;
	gps_blending.setGpsData(gps_data0, 0);
	gps_blending.setGpsData(gps_data1, 1);
	gps_blending.setBlendingUseHPosAccuracy(true);
	gps_blending.update(_time_now_us);
	// THEN: The average of the two timestamps should be used
	EXPECT_EQ(gps_blending.getOutputGpsData().time_utc_usec, 1700000000000500ULL);
}
