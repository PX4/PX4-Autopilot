/****************************************************************************
 *
 *   Copyright (C) 2023 PX4 Development Team. All rights reserved.
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
 * To run this test only use: make tests TESTFILTER=atmosphere
 */

#include <gtest/gtest.h>
#include <lib/atmosphere/atmosphere.h>
using namespace atmosphere;

TEST(TestAtmosphere, pressureFromAltitude)
{
	// GIVEN pressure at seal level in standard atmosphere and sea level altitude
	const float pressureAtSeaLevel = 101325.f; // Pa
	float altitude = 0.0f;

	// WHEN we calculate pressure based on altitude
	float pressure = getPressureFromAltitude(altitude);

	// THEN expect seal level pressure for standard atmosphere
	EXPECT_FLOAT_EQ(pressure, pressureAtSeaLevel);

	// WHEN we are at 3000m altitude
	altitude = 3000.0f;
	pressure = getPressureFromAltitude(altitude);

	// THEN expect standard atmosphere pressure at 3000m (error of 10Pa corresponds to 1m altitude error in standard atmosphere when altitude is between 1000 and 2000m)
	EXPECT_NEAR(pressure, 70120.f, 10.0f);
}

TEST(TestAtmosphere, altitudeFromPressure)
{
	// GIVEN pressure at seal level in standard atmosphere
	const float pressureAtSealevel = 101325.f;
	float pressure = pressureAtSealevel;

	// WHEN we calculate altitude from pressure
	float altitude = getAltitudeFromPressure(pressure, pressureAtSealevel);

	// THEN expect resulting altitude to be near sea level
	EXPECT_FLOAT_EQ(altitude, 0.0f);

	// GIVEN  pressure is standard atmosphere pressure at 3000m
	pressure = 70109.f;

	// WHEN we calculate altitude from pressure
	altitude = getAltitudeFromPressure(pressure, pressureAtSealevel);

	// THEN expect altitude to be near 3000m
	EXPECT_NEAR(altitude, 3000.0f, 0.5f);
}

TEST(TestAtmosphere, DensityFromPressure)
{
// GIVEN standard atmosphere at sea level
	const float pressureAtSealevel = 101325.f;
	const float tempSeaLevel = 15.f;

	// WHEN we calculate density from pressure and temperature
	float density = getDensityFromPressureAndTemp(pressureAtSealevel, tempSeaLevel);

	// THEN expect density at sea level in standard atmosphere
	EXPECT_NEAR(density, 1.225f, 0.001f);

	// GIVEN standard atmosphere at 3000m
	const float pressure = 70109.f;
	const float tempAt3000m = -4.5f;

	// WHEN we calculate density from pressure and temperature
	density = getDensityFromPressureAndTemp(pressure, tempAt3000m);

	// THEN expect density at 3000m in standard atmosphere
	EXPECT_NEAR(density, 0.9091f, 0.001f);
}

TEST(TestAtmosphere, StandardTemperature)
{
	// GIVEN standard atmosphere at sea level
	float altitude = 0.f;

	// WHEN we calculate standard temperature from altitude
	float temperature = getStandardTemperatureAtAltitude(altitude);

	// THEN expect standard temperature at sea level
	EXPECT_NEAR(temperature, 15.f, 0.001f);

	// GIVEN standard atmosphere at 3000m
	altitude = 3000.f;

	// WHEN we calculate standard temperature from altitude
	temperature = getStandardTemperatureAtAltitude(altitude);

	// THEN expect standard temperature at 3000m
	EXPECT_NEAR(temperature, -4.5f, 0.001f);
}
