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
 * Test for VehicleAirData
 */

#include <gtest/gtest.h>

#include "../VehicleAirData.hpp"

#include <drivers/drv_hrt.h>
#include <hrt_work.h>
#include <lib/geo/geo.h>
#include <parameters/param.h>
#include <px4_platform_common/time.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/wind.h>
#include <uORB/uORBManager.hpp>

class VehicleAirDataTest : public ::testing::Test
{
public:
	class VehicleAirDataTestable : public sensors::VehicleAirData
	{
	public:
		// Run() re-arms itself through the hrt callout worker, cleared again so the cycle stays
		// in the test's hands
		void RunPublic()
		{
			Run();
			ScheduleClear();
		}
		int selectedIndex() const { return selectedSensorIndex(); }
		uint8_t priority(int index) const { return sensorPriority(index); }
		bool callbackRegistered(int index) const { return VehicleAirData::callbackRegistered(index); }
	};

	static constexpr int kSensorCount = 1;

	// device i publishes on sensor_baro instance i and is calibrated in slot CAL_BAROi for the whole suite
	static constexpr uint32_t kDeviceIds[kSensorCount] {0x3c4d01};

	// uORB nodes outlive a manager restart, so the suite brings the manager up once and keeps its topics
	static void SetUpTestSuite()
	{
		// the gtest harness does not bring up the hrt: Run() schedules its next cycle through it,
		// and without hrt_init() the callout lock is not recursive, so the worker deadlocks on
		// its own nested lock on Linux the first time a callout is due
		hrt_init();
		hrt_work_queue_init();
		uORB::Manager::initialize();

		for (int i = 0; i < kSensorCount; i++) {
			ASSERT_TRUE(_sensor_baro_pubs[i].advertise());
			ASSERT_EQ(_sensor_baro_pubs[i].get_instance(), i);
		}
	}

	// every test starts with all devices disabled and enables the ones it uses
	void SetUp() override
	{
		for (int i = 0; i < kSensorCount; i++) {
			setPriority(i, 0);
		}
	}

	static void setParam(const char *name, int32_t value)
	{
		const param_t handle = param_find(name);
		ASSERT_NE(handle, PARAM_INVALID) << name;
		ASSERT_EQ(param_set_no_notification(handle, &value), 0) << name;
	}

	static void setParam(const char *name, float value)
	{
		const param_t handle = param_find(name);
		ASSERT_NE(handle, PARAM_INVALID) << name;
		ASSERT_EQ(param_set_no_notification(handle, &value), 0) << name;
	}

	static void setPriority(int device, int32_t priority)
	{
		char name[20] {};
		snprintf(name, sizeof(name), "CAL_BARO%d_ID", device);
		setParam(name, static_cast<int32_t>(kDeviceIds[device]));
		snprintf(name, sizeof(name), "CAL_BARO%d_PRIO", device);
		setParam(name, priority);
	}

	static void publishSample(int device, float pressure_pa)
	{
		sensor_baro_s sample{};
		sample.timestamp_sample = hrt_absolute_time();
		sample.timestamp = sample.timestamp_sample;
		sample.device_id = kDeviceIds[device];
		sample.is_external = true;
		sample.pressure = pressure_pa;
		sample.temperature = 20.f;
		_sensor_baro_pubs[device].publish(sample);
	}

	static void publishParameterUpdate()
	{
		parameter_update_s update{};
		update.timestamp = hrt_absolute_time();
		_parameter_update_pub.publish(update);
	}

	// run the module at its own 50 ms cadence for the given time, feeding it the given samples
	template<typename Feed>
	static void runFor(VehicleAirDataTestable &module, hrt_abstime duration, Feed feed)
	{
		const hrt_abstime start = hrt_absolute_time();

		do {
			feed();
			module.RunPublic();
			px4_usleep(50_ms);
		} while (hrt_absolute_time() < start + duration);
	}

	static uORB::PublicationMulti<sensor_baro_s> _sensor_baro_pubs[kSensorCount];
	static uORB::Publication<parameter_update_s> _parameter_update_pub;

	static constexpr float kPressurePa = 101325.f;
};

constexpr uint32_t VehicleAirDataTest::kDeviceIds[];
uORB::PublicationMulti<sensor_baro_s> VehicleAirDataTest::_sensor_baro_pubs[kSensorCount] {{ORB_ID(sensor_baro)}};
uORB::Publication<parameter_update_s> VehicleAirDataTest::_parameter_update_pub{ORB_ID(parameter_update)};

TEST_F(VehicleAirDataTest, BarometerDisabledFromBootDoesNotDriveRun)
{
	// the only barometer is disabled before the module sees it: nothing ever gets selected, so
	// nothing ever clears the callbacks, and one registered here would wake Run() at the sample rate
	VehicleAirDataTestable module;

	runFor(module, 500_ms, [&]() { publishSample(0, kPressurePa); });

	EXPECT_EQ(module.selectedIndex(), -1);
	EXPECT_EQ(module.priority(0), 0);
	EXPECT_FALSE(module.callbackRegistered(0));

	// re-enabled, the sensor is selected and only then drives Run()
	setPriority(0, 75);
	publishParameterUpdate();

	// long enough for the module's throttled parameter subscription to pick up the change
	runFor(module, 1200_ms, [&]() { publishSample(0, kPressurePa); });

	EXPECT_EQ(module.selectedIndex(), 0);
	EXPECT_EQ(module.priority(0), 75);
	EXPECT_TRUE(module.callbackRegistered(0));
}

#if defined(CONFIG_SENSORS_BARO_COMPENSATION)
TEST_F(VehicleAirDataTest, CompensationCorrectsPublishedAltitude)
{
	// flying forward at 10 m/s through still air with the port reading high, and hovering at half
	// thrust with the propellers pulling the pressure down further
	setPriority(0, 75);
	setParam("SENS_BARO_K_XP", -0.4f);
	setParam("SENS_BARO_K_T", -20.f);

	VehicleAirDataTestable module;
	publishParameterUpdate();

	uORB::Publication<wind_s> wind_pub{ORB_ID(wind)};
	uORB::Publication<vehicle_local_position_s> local_position_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<vehicle_attitude_s> attitude_pub{ORB_ID(vehicle_attitude)};
	uORB::Publication<vehicle_thrust_setpoint_s> thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};

	runFor(module, 1500_ms, [&]() {
		const hrt_abstime now = hrt_absolute_time();

		wind_s wind{};
		wind.timestamp = now;
		wind_pub.publish(wind);

		vehicle_local_position_s local_position{};
		local_position.timestamp = now;
		local_position.xy_valid = true;
		local_position.v_xy_valid = true;
		local_position.vx = 10.f;
		local_position_pub.publish(local_position);

		vehicle_attitude_s attitude{};
		attitude.timestamp = now;
		attitude.q[0] = 1.f;
		attitude_pub.publish(attitude);

		vehicle_thrust_setpoint_s thrust_setpoint{};
		thrust_setpoint.timestamp = now;
		thrust_setpoint.xyz[2] = -0.5f;
		thrust_setpoint_pub.publish(thrust_setpoint);

		publishSample(0, kPressurePa);
	});

	uORB::Subscription air_data_sub{ORB_ID(vehicle_air_data)};
	vehicle_air_data_s air_data{};
	ASSERT_TRUE(air_data_sub.copy(&air_data));

	const float expected_correction = -20.f * 0.5f + 0.5f * -0.4f * 10.f * 10.f / CONSTANTS_ONE_G;
	EXPECT_NEAR(air_data.baro_alt_correction, expected_correction, 1e-3f);
	// standard pressure at the default QNH is 0 m before the correction
	EXPECT_NEAR(air_data.baro_alt_meter, expected_correction, 1e-2f);
}
#endif // CONFIG_SENSORS_BARO_COMPENSATION
