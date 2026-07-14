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

/**
 * Test for VehicleAirData barometer compensation.
 */

#include <gtest/gtest.h>

#define private public
#include "../VehicleAirData.hpp"
#undef private

#include <drivers/drv_hrt.h>
#include <geo/geo.h>
#include <parameters/param.h>
#include <px4_platform_common/defines.h>
#include <uORB/uORBManager.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/wind.h>

using namespace time_literals;

class VehicleAirDataTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		uORB::Manager::initialize();
		param_control_autosave(false);
		param_reset_all();
	}

	void TearDown() override
	{
		if (_attitude_pub != nullptr) {
			orb_unadvertise(_attitude_pub);
		}

		if (_local_position_pub != nullptr) {
			orb_unadvertise(_local_position_pub);
		}

		if (_wind_pub != nullptr) {
			orb_unadvertise(_wind_pub);
		}

		if (_thrust_pub != nullptr) {
			orb_unadvertise(_thrust_pub);
		}

		uORB::Manager::terminate();
	}

	void setParam(const char *name, float value)
	{
		const param_t handle = param_find(name);
		ASSERT_NE(handle, PARAM_INVALID);
		ASSERT_EQ(param_set(handle, &value), PX4_OK);
	}

	void publishAttitude()
	{
		vehicle_attitude_s attitude{};
		attitude.timestamp = hrt_absolute_time();
		attitude.q[0] = 1.f;

		publish(ORB_ID(vehicle_attitude), _attitude_pub, &attitude);
	}

	void publishLocalPosition(float vx, float vy, float vz = 0.f)
	{
		vehicle_local_position_s local_position{};
		local_position.timestamp = hrt_absolute_time();
		local_position.timestamp_sample = local_position.timestamp;
		local_position.vx = vx;
		local_position.vy = vy;
		local_position.vz = vz;
		local_position.xy_valid = true;
		local_position.v_xy_valid = true;

		publish(ORB_ID(vehicle_local_position), _local_position_pub, &local_position);
	}

	void publishWind(float windspeed_north, float windspeed_east)
	{
		wind_s wind{};
		wind.timestamp = hrt_absolute_time();
		wind.windspeed_north = windspeed_north;
		wind.windspeed_east = windspeed_east;

		publish(ORB_ID(wind), _wind_pub, &wind);
	}

	void publishThrust(hrt_abstime timestamp, float thrust_z)
	{
		vehicle_thrust_setpoint_s thrust{};
		thrust.timestamp = timestamp;
		thrust.xyz[2] = thrust_z;

		publish(ORB_ID(vehicle_thrust_setpoint), _thrust_pub, &thrust);
	}

private:
	template<typename T>
	void publish(const orb_metadata *meta, orb_advert_t &handle, const T *message)
	{
		if (handle == nullptr) {
			handle = orb_advertise(meta, message);
			ASSERT_NE(handle, nullptr);

		} else {
			ASSERT_EQ(orb_publish(meta, handle, message), PX4_OK);
		}
	}

	orb_advert_t _attitude_pub{nullptr};
	orb_advert_t _local_position_pub{nullptr};
	orb_advert_t _wind_pub{nullptr};
	orb_advert_t _thrust_pub{nullptr};
};

TEST_F(VehicleAirDataTest, DynamicPressureCompensationDefaultsToZero)
{
	sensors::VehicleAirData air_data;
	air_data.ParametersUpdate(true);

	EXPECT_FLOAT_EQ(air_data.dynamicPressureCompensation(1.2f), 0.f);
}

TEST_F(VehicleAirDataTest, DynamicPressureCompensation)
{
	setParam("SENS_BARO_K_XP", 1.f);

	publishAttitude();
	publishLocalPosition(10.f, 0.f);
	publishWind(0.f, 0.f);

	sensors::VehicleAirData air_data;
	air_data.ParametersUpdate(true);

	const float expected = 0.5f * 10.f * 10.f / CONSTANTS_ONE_G;
	EXPECT_NEAR(air_data.dynamicPressureCompensation(1.2f), expected, 1e-4f);

	setParam("SENS_BARO_K_XP", 0.f);
	setParam("SENS_BARO_K_XN", -0.5f);

	publishLocalPosition(-6.f, 0.f);
	air_data.ParametersUpdate(true);

	const float expected_negative_x = 0.5f * 6.f * 6.f * -0.5f / CONSTANTS_ONE_G;
	EXPECT_NEAR(air_data.dynamicPressureCompensation(1.2f), expected_negative_x, 1e-4f);

	setParam("SENS_BARO_K_XN", 0.f);
	setParam("SENS_BARO_K_XP", 1.f);
	setParam("SENS_BARO_VMAX", 5.f);

	publishLocalPosition(30.f, 0.f);
	air_data.ParametersUpdate(true);

	const float expected_clamped = 0.5f * 5.f * 5.f / CONSTANTS_ONE_G;
	EXPECT_NEAR(air_data.dynamicPressureCompensation(1.2f), expected_clamped, 1e-4f);
}

TEST_F(VehicleAirDataTest, ThrustCompensationUsesClosestBufferedSample)
{
	setParam("SENS_BARO_K_T", 2.f);

	const hrt_abstime now = hrt_absolute_time();
	publishThrust(now - 100_ms, -0.3f);
	publishThrust(now + 100_ms, -0.6f);

	sensors::VehicleAirData air_data;
	air_data.ParametersUpdate(true);
	air_data.updateThrustBuffer();

	EXPECT_NEAR(air_data.thrustCompensation(now + 90_ms), 1.2f, 1e-5f);
}

TEST_F(VehicleAirDataTest, ThrustCompensationRejectsStaleSample)
{
	setParam("SENS_BARO_K_T", 2.f);

	const hrt_abstime now = hrt_absolute_time();
	publishThrust(now - 1_s, -0.6f);

	sensors::VehicleAirData air_data;
	air_data.ParametersUpdate(true);
	air_data.updateThrustBuffer();

	EXPECT_FLOAT_EQ(air_data.thrustCompensation(now), 0.f);
}
