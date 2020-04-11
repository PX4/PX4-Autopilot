/****************************************************************************
 *
 *   Copyright (c) 2020 ECL Development Team. All rights reserved.
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
#include <math.h>
#include <mathlib/mathlib.h>
#include <memory>
#include <geo/geo.h>

class GeoTest : public ::testing::Test {
 public:
	void SetUp() override
	{
		origin.timestamp = 0;
		origin.lat_rad = math::radians(473566094 / 1e7);
		origin.lon_rad = math::radians(85190237 / 1e7);
		origin.sin_lat = sin(origin.lat_rad);
		origin.cos_lat = cos(origin.lat_rad);
		origin.init_done = true;
	}

 protected:
	map_projection_reference_s origin;

};


TEST_F(GeoTest, reprojectProject)
{
	// GIVEN: x and y coordinates in the local cartesian frame
	float x = 0.5;
	float y = 1;
	double lat;
	double lon;
	map_projection_reproject(&origin, x, y, &lat, &lon);
	float x_new;
	float y_new;
	map_projection_project(&origin, lat, lon, &x_new, &y_new);
	double lat_new;
	double lon_new;
	map_projection_reproject(&origin, x_new, y_new, &lat_new, &lon_new);
	EXPECT_FLOAT_EQ(x, x_new);
	EXPECT_FLOAT_EQ(y, y_new);
	EXPECT_FLOAT_EQ(lat, lat_new);
	EXPECT_FLOAT_EQ(lon, lon_new);
}

TEST_F(GeoTest, projectReproject)
{
	// GIVEN: x and y coordinates in the local cartesian frame
	double lat = 47.356616973876953;
	double lon = 8.5190505981445313;
	float x;
	float y;
	map_projection_project(&origin, lat, lon, &x, &y);
	double lat_new;
	double lon_new;
	map_projection_reproject(&origin, x, y, &lat_new, &lon_new);
	float x_new;
	float y_new;
	map_projection_project(&origin, lat_new, lon_new, &x_new, &y_new);
	// WHEN: apply the mapping and its inverse the output should stay the same
	EXPECT_FLOAT_EQ(x, x_new);
	EXPECT_FLOAT_EQ(y, y_new);
	EXPECT_FLOAT_EQ(lat, lat_new);
	EXPECT_FLOAT_EQ(lon, lon_new);
}
