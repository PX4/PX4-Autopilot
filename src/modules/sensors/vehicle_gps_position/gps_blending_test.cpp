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

	GpsBlending<2> _gps_blending;
};

sensor_gps_s GpsBlendingTest::getDefaultGpsData()
{
	sensor_gps_s gps_data{};
	gps_data.timestamp = hrt_absolute_time() - 10e3; // microseconds
	gps_data.time_utc_usec = 0;
	gps_data.lat = 47e7;
	gps_data.lon = 9e7;
	gps_data.alt = 800e3;
	gps_data.alt_ellipsoid = 800e3;
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

TEST_F(GpsBlendingTest, noData)
{
	EXPECT_EQ(_gps_blending.getSelectedGps(), 0);
	EXPECT_FALSE(_gps_blending.isNewOutputDataAvailable());

	_gps_blending.update();

	EXPECT_EQ(_gps_blending.getSelectedGps(), 0);
	EXPECT_FALSE(_gps_blending.isNewOutputDataAvailable());
}

TEST_F(GpsBlendingTest, singleReceiver)
{
	sensor_gps_s gps_data = getDefaultGpsData();

	_gps_blending.setGpsData(gps_data, 1);
	_gps_blending.update();

	gps_data.timestamp = hrt_absolute_time() - 10e3;
	_gps_blending.setGpsData(gps_data, 1);
	_gps_blending.update();

	EXPECT_EQ(_gps_blending.getSelectedGps(), 1);
	EXPECT_EQ(_gps_blending.getNumberOfGpsSuitableForBlending(), 1);
	EXPECT_TRUE(_gps_blending.isNewOutputDataAvailable());
}

TEST_F(GpsBlendingTest, dualReceiverNoBlending)
{
	sensor_gps_s gps_data0 = getDefaultGpsData();
	sensor_gps_s gps_data1 = getDefaultGpsData();

	gps_data1.satellites_used = gps_data0.satellites_used + 2; // gps1 has more satellites than gps0
	_gps_blending.setGpsData(gps_data0, 0);
	_gps_blending.setGpsData(gps_data1, 1);
	_gps_blending.update();

	// THEN: gps1 should be selected because it has more satellites
	EXPECT_EQ(_gps_blending.getSelectedGps(), 1);
	EXPECT_EQ(_gps_blending.getNumberOfGpsSuitableForBlending(), 2);
	EXPECT_TRUE(_gps_blending.isNewOutputDataAvailable());

	gps_data1.satellites_used = gps_data0.satellites_used - 2; // gps1 has less satellites than gps0
	_gps_blending.setGpsData(gps_data0, 0);
	_gps_blending.setGpsData(gps_data1, 1);
	_gps_blending.update();

	// THEN: gps0 should be selected because it has more satellites
	EXPECT_EQ(_gps_blending.getSelectedGps(), 0);
	EXPECT_EQ(_gps_blending.getNumberOfGpsSuitableForBlending(), 2);
	EXPECT_TRUE(_gps_blending.isNewOutputDataAvailable());
}

TEST_F(GpsBlendingTest, dualReceiverBlendingHPos)
{
	sensor_gps_s gps_data0 = getDefaultGpsData();
	sensor_gps_s gps_data1 = getDefaultGpsData();

	_gps_blending.setBlendingUseHPosAccuracy(true);

	gps_data1.eph = gps_data0.eph / 2.f;
	_gps_blending.setGpsData(gps_data0, 0);
	_gps_blending.setGpsData(gps_data1, 1);
	_gps_blending.update();

	// THEN: the blended instance should be selected (2)
	// and the eph should be adjusted
	EXPECT_EQ(_gps_blending.getSelectedGps(), 2);
	EXPECT_EQ(_gps_blending.getNumberOfGpsSuitableForBlending(), 2);
	EXPECT_TRUE(_gps_blending.isNewOutputDataAvailable());
	EXPECT_LT(_gps_blending.getOutputGpsData().eph, gps_data0.eph);
	EXPECT_FLOAT_EQ(_gps_blending.getOutputGpsData().eph, gps_data1.eph); // TODO: should be greater than
}
