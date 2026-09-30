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
 * Test code for the GPS receiver selection logic
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
	sensor_gnss_s getDefaultGnssData();
	void runSeconds(float duration_s, GpsBlending &gps_blending, sensor_gnss_s &gnss_data, int instance);
	void runSeconds(float duration_s, GpsBlending &gps_blending, sensor_gnss_s &gnss_data0, sensor_gnss_s &gnss_data1);

	uint64_t _time_now_us{1000000};
};

sensor_gnss_s GpsBlendingTest::getDefaultGnssData()
{
	sensor_gnss_s gnss_data{};
	gnss_data.timestamp = _time_now_us - 10e3;
	gnss_data.time_utc_usec = 0;
	gnss_data.latitude = 47.0;
	gnss_data.longitude = 9.0;
	gnss_data.altitude_msl = 800.0;
	gnss_data.altitude_ellipsoid = 800.0;
	gnss_data.speed_accuracy = 0.2f;
	gnss_data.course_accuracy = 0.5f;
	gnss_data.eph = 0.7f;
	gnss_data.epv = 1.2f;
	gnss_data.hdop = 1.f;
	gnss_data.vdop = 1.f;
	gnss_data.noise = 20;
	gnss_data.jamming_indicator = 40;
	gnss_data.ground_speed = 1.f;
	gnss_data.vel_north = 1.f;
	gnss_data.vel_east = 1.f;
	gnss_data.vel_down = 1.f;
	gnss_data.course = 0.f;
	gnss_data.timestamp_time_relative = 0;
	gnss_data.fix_type = 4;
	gnss_data.vel_ned_valid = true;
	gnss_data.satellites_used = 8;

	return gnss_data;
}

void GpsBlendingTest::runSeconds(float duration_s, GpsBlending &gps_blending, sensor_gnss_s &gnss_data, int instance)
{
	const float dt = 0.1;
	const uint64_t dt_us = static_cast<uint64_t>(dt * 1e6f);

	for (int k = 0; k < static_cast<int>(duration_s / dt); k++) {
		gps_blending.setGnssData(gnss_data, instance);
		gps_blending.update(_time_now_us);

		_time_now_us += dt_us;
		gnss_data.timestamp += dt_us;
	}
}

void GpsBlendingTest::runSeconds(float duration_s, GpsBlending &gps_blending, sensor_gnss_s &gnss_data0,
				 sensor_gnss_s &gnss_data1)
{
	const float dt = 0.1;
	const uint64_t dt_us = static_cast<uint64_t>(dt * 1e6f);

	for (int k = 0; k < static_cast<int>(duration_s / dt); k++) {
		gps_blending.setGnssData(gnss_data0, 0);
		gps_blending.setGnssData(gnss_data1, 1);

		gps_blending.update(_time_now_us);

		_time_now_us += dt_us;
		gnss_data0.timestamp += dt_us;
		gnss_data1.timestamp += dt_us;
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
	sensor_gnss_s gnss_data = getDefaultGnssData();

	gps_blending.setGnssData(gnss_data, 1);
	gps_blending.update(_time_now_us);

	_time_now_us += 200e3;
	gnss_data.timestamp = _time_now_us - 10e3;
	gps_blending.setGnssData(gnss_data, 1);
	gps_blending.update(_time_now_us);

	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	// BUT IF: a second update is called without data
	gps_blending.update(_time_now_us);

	// THEN: no new data should be available
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_FALSE(gps_blending.isNewOutputDataAvailable());
}

TEST_F(GpsBlendingTest, dualReceiverNoBlending)
{
	GpsBlending gps_blending;

	// GIVEN: two receivers with the same prioity
	gps_blending.setPrimaryInstance(-1);
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	gnss_data1.satellites_used = gnss_data0.satellites_used + 2; // gps1 has more satellites than gps0
	gps_blending.setGnssData(gnss_data0, 0);
	gps_blending.setGnssData(gnss_data1, 1);
	gps_blending.update(_time_now_us);

	// THEN: gps1 should be selected because it has more satellites
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	gnss_data1.satellites_used = gnss_data0.satellites_used - 2; // gps1 has less satellites than gps0
	gps_blending.setGnssData(gnss_data0, 0);
	gps_blending.setGnssData(gnss_data1, 1);
	gps_blending.update(_time_now_us);

	// THEN: gps0 should be selected because it has more satellites
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());
}

TEST_F(GpsBlendingTest, dualReceiverFailover)
{
	GpsBlending gps_blending;

	// GIVEN: a dual GPS setup with the first instance (0)
	// set as primary
	gps_blending.setPrimaryInstance(0);

	// WHEN: only the secondary receiver is available
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	const float duration_s = 10.f;
	runSeconds(duration_s, gps_blending, gnss_data1, 1);

	// THEN: the secondary instance as the primary one is not available
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	// BUT WHEN: the data of the primary receiver is avaialbe
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	runSeconds(1.f, gps_blending, gnss_data0, gnss_data1);

	// THEN: the primary instance is selected and the data
	// is available
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	runSeconds(duration_s, gps_blending, gnss_data0, gnss_data1);

	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	// BUT WHEN: the primary receiver isn't available anymore
	runSeconds(duration_s, gps_blending, gnss_data1, 1);

	// THEN: the data of the secondary receiver can be used
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	// AND IF: the primary receiver is available again and has
	// better metrics than the secondary one
	gnss_data0.timestamp = gnss_data1.timestamp;
	gnss_data0.satellites_used = gnss_data1.satellites_used + 2;

	runSeconds(1.f, gps_blending, gnss_data0, gnss_data1);

	// THEN: the primary receiver should be used again
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	// BUT IF: the secondary receiver has better metrics than the primary one
	gnss_data1.satellites_used = gnss_data0.satellites_used + 2;

	runSeconds(1.f, gps_blending, gnss_data0, gnss_data1);

	// THEN: the selector shouldn't switch again as the primary one is available
	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	// BUT IF: the primary receiver looses its fix
	gnss_data0.fix_type = 1;

	runSeconds(1.f, gps_blending, gnss_data0, gnss_data1);

	// THEN: the selector should switch as the primary one is unable to provide correct data
	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());
}

TEST_F(GpsBlendingTest, singleReceiverAntennaOffset)
{
	GpsBlending gps_blending;

	gps_blending.setPrimaryInstance(-1);
	sensor_gnss_s gnss_data = getDefaultGnssData();

	const Vector3f offset0(0.1f, 0.0f, -0.05f);
	gps_blending.setAntennaOffset(offset0, 1);

	gps_blending.setGnssData(gnss_data, 1);
	gps_blending.update(_time_now_us);

	_time_now_us += 200e3;
	gnss_data.timestamp = _time_now_us - 10e3;
	gps_blending.setGnssData(gnss_data, 1);
	gps_blending.update(_time_now_us);

	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_TRUE(gps_blending.isNewOutputDataAvailable());

	const Vector3f &out = gps_blending.getOutputAntennaOffset();
	EXPECT_FLOAT_EQ(out(0), offset0(0));
	EXPECT_FLOAT_EQ(out(1), offset0(1));
	EXPECT_FLOAT_EQ(out(2), offset0(2));
}

TEST_F(GpsBlendingTest, failoverAntennaOffset)
{
	GpsBlending gps_blending;

	gps_blending.setPrimaryInstance(0);

	const Vector3f offset0(0.1f, 0.0f, 0.0f);
	const Vector3f offset1(-0.1f, 0.0f, 0.0f);
	gps_blending.setAntennaOffset(offset0, 0);
	gps_blending.setAntennaOffset(offset1, 1);

	// Only secondary available
	sensor_gnss_s gnss_data1 = getDefaultGnssData();
	runSeconds(10.f, gps_blending, gnss_data1, 1);

	EXPECT_EQ(gps_blending.getSelectedGps(), 1);
	EXPECT_FLOAT_EQ(gps_blending.getOutputAntennaOffset()(0), offset1(0));

	// Now primary becomes available
	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	gnss_data0.timestamp = gnss_data1.timestamp;
	runSeconds(1.f, gps_blending, gnss_data0, gnss_data1);

	EXPECT_EQ(gps_blending.getSelectedGps(), 0);
	EXPECT_FLOAT_EQ(gps_blending.getOutputAntennaOffset()(0), offset0(0));
}

TEST_F(GpsBlendingTest, dualReceiverNoBlendingStaleFlag)
{
	GpsBlending gps_blending;

	// GIVEN: two receivers
	gps_blending.setPrimaryInstance(-1);

	sensor_gnss_s gnss_data0 = getDefaultGnssData();
	sensor_gnss_s gnss_data1 = getDefaultGnssData();

	gnss_data1.satellites_used = gnss_data0.satellites_used + 2; // gps1 wins selection

	// First update: both receivers provide data, gps1 is selected
	gps_blending.setGnssData(gnss_data0, 0);
	gps_blending.setGnssData(gnss_data1, 1);
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
