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
 * @file TemperatureCompensationModuleTest.cpp
 *
 * Where the module takes the temperature from when a sensor doesn't report one, and which device id
 * it publishes with the corrections.
 */

#include <gtest/gtest.h>

#include <drivers/drv_hrt.h>
#include <hrt_work.h>
#include <parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/uORBManager.hpp>

#include "TemperatureCompensationModule.h"

using temperature_compensation::TemperatureCompensationModule;

namespace temperature_compensation
{

// reaches the private state the tests drive and assert on
class TemperatureCompensationModuleTestPeer
{
public:
	static void run(TemperatureCompensationModule &module) { module.Run(); }
	static void parametersUpdate(TemperatureCompensationModule &module) { module.parameters_update(); }
	static const sensor_correction_s &corrections(const TemperatureCompensationModule &module) { return module._corrections; }
};

} // namespace temperature_compensation

using Peer = temperature_compensation::TemperatureCompensationModuleTestPeer;

class TemperatureCompensationModuleTest : public ::testing::Test
{
public:
	static void SetUpTestSuite()
	{
		hrt_init();
		hrt_work_queue_init();
		uORB::Manager::initialize();

		// each publisher takes instance 0 of its topic for the whole suite
		ASSERT_TRUE(_accel_pub.advertise());
		ASSERT_TRUE(_gyro_pub.advertise());
		ASSERT_TRUE(_mag_pub.advertise());
		ASSERT_TRUE(_baro_pub.advertise());
		ASSERT_EQ(_accel_pub.get_instance(), 0);
		ASSERT_EQ(_gyro_pub.get_instance(), 0);
		ASSERT_EQ(_mag_pub.get_instance(), 0);
		ASSERT_EQ(_baro_pub.get_instance(), 0);
	}

	// the barometer calibration a test enables is cleared again, so that the others run without one
	void TearDown() override
	{
		setParam("TC_B_ENABLE", 0);
		setParam("TC_B1_ID", 0);
	}

	static void setParam(const char *name, int32_t value)
	{
		const param_t handle = param_find(name);
		ASSERT_NE(handle, PARAM_INVALID) << name;
		ASSERT_EQ(param_set_no_notification(handle, &value), 0) << name;
	}

	static void publishAccel(float temperature, float y = 0.f)
	{
		sensor_accel_s sample{};
		sample.timestamp_sample = hrt_absolute_time();
		sample.device_id = kAccelId;
		sample.y = y;
		sample.temperature = temperature;
		sample.timestamp = hrt_absolute_time();
		_accel_pub.publish(sample);
	}

	static void publishGyro(float temperature)
	{
		sensor_gyro_s sample{};
		sample.timestamp_sample = hrt_absolute_time();
		sample.device_id = kGyroId;
		sample.temperature = temperature;
		sample.timestamp = hrt_absolute_time();
		_gyro_pub.publish(sample);
	}

	static void publishMag(float temperature)
	{
		sensor_mag_s sample{};
		sample.timestamp_sample = hrt_absolute_time();
		sample.device_id = kMagId;
		sample.temperature = temperature;
		sample.timestamp = hrt_absolute_time();
		_mag_pub.publish(sample);
	}

	static void publishBaro(float temperature)
	{
		sensor_baro_s sample{};
		sample.timestamp_sample = hrt_absolute_time();
		sample.device_id = kBaroId;
		sample.pressure = 101325.f;
		sample.temperature = temperature;
		sample.timestamp = hrt_absolute_time();
		_baro_pub.publish(sample);
	}

	// The sensors publish far faster than the module runs, so when it polls a sensor there are several samples
	// queued. The sensor's own poll takes one, and a fallback for another sensor takes the next. Filling a topic's
	// queue also leaves nothing from an earlier test in it, since the module is created fresh in each test and
	// starts reading at the oldest queued sample.
	static void publishAccelSamples(float temperature, float y = 0.f)
	{
		for (int i = 0; i < sensor_accel_s::ORB_QUEUE_LENGTH; i++) {
			publishAccel(temperature, y);
		}
	}

	static void publishGyroSamples(float temperature)
	{
		for (int i = 0; i < sensor_gyro_s::ORB_QUEUE_LENGTH; i++) {
			publishGyro(temperature);
		}
	}

	static void publishMagSamples(float temperature)
	{
		for (int i = 0; i < sensor_mag_s::ORB_QUEUE_LENGTH; i++) {
			publishMag(temperature);
		}
	}

	static void publishBaroSamples(float temperature)
	{
		for (int i = 0; i < sensor_baro_s::ORB_QUEUE_LENGTH; i++) {
			publishBaro(temperature);
		}
	}

	static constexpr uint32_t kAccelId = 0x2a0101;
	static constexpr uint32_t kGyroId = 0x2a0102;
	static constexpr uint32_t kMagId = 0x2a0103;
	static constexpr uint32_t kBaroId = 0x2a0104;

	static uORB::PublicationMulti<sensor_accel_s> _accel_pub;
	static uORB::PublicationMulti<sensor_gyro_s> _gyro_pub;
	static uORB::PublicationMulti<sensor_mag_s> _mag_pub;
	static uORB::PublicationMulti<sensor_baro_s> _baro_pub;
};

uORB::PublicationMulti<sensor_accel_s> TemperatureCompensationModuleTest::_accel_pub{ORB_ID(sensor_accel)};
uORB::PublicationMulti<sensor_gyro_s> TemperatureCompensationModuleTest::_gyro_pub{ORB_ID(sensor_gyro)};
uORB::PublicationMulti<sensor_mag_s> TemperatureCompensationModuleTest::_mag_pub{ORB_ID(sensor_mag)};
uORB::PublicationMulti<sensor_baro_s> TemperatureCompensationModuleTest::_baro_pub{ORB_ID(sensor_baro)};

// WHY: the fallback for a magnetometer without a temperature copied the primary accelerometer sample into a
// barometer struct, which is 8 bytes shorter, and read the accelerometer's y axis as the temperature.
// WHAT: the magnetometer takes the primary barometer's temperature, as its comment always said.
TEST_F(TemperatureCompensationModuleTest, MagnetometerWithoutTemperatureUsesThePrimaryBarometer)
{
	TemperatureCompensationModule module;

	publishAccelSamples(30.f, 5.f);
	publishBaroSamples(22.5f);
	publishMagSamples(NAN);

	Peer::run(module);

	EXPECT_FLOAT_EQ(Peer::corrections(module).mag_temperature[0], 22.5f);
	EXPECT_EQ(Peer::corrections(module).mag_device_ids[0], kMagId);
}

// WHY: after a parameter update the barometer entry carried the calibration slot index instead of the device id,
// so the consumer, which matches on the device id, found no correction for it.
// WHAT: the device id is published, as for the other three sensor types.
TEST_F(TemperatureCompensationModuleTest, BarometerCorrectionCarriesTheDeviceIdAfterAParameterUpdate)
{
	setParam("TC_B_ENABLE", 1);
	setParam("TC_B0_ID", 0);
	setParam("TC_B1_ID", static_cast<int32_t>(kBaroId)); // calibrated in slot 1, so a slot index would show as 1

	TemperatureCompensationModule module;

	publishBaroSamples(20.f);
	Peer::run(module);

	Peer::parametersUpdate(module);

	EXPECT_EQ(Peer::corrections(module).baro_device_ids[0], kBaroId);
}

// WHAT: a barometer without a temperature keeps using the primary accelerometer's.
TEST_F(TemperatureCompensationModuleTest, BarometerWithoutTemperatureUsesThePrimaryAccelerometer)
{
	TemperatureCompensationModule module;

	publishAccelSamples(31.f);
	publishBaroSamples(NAN);

	Peer::run(module);

	EXPECT_FLOAT_EQ(Peer::corrections(module).baro_temperature[0], 31.f);
}

// WHAT: a gyroscope without a temperature keeps using the accelerometer of the same instance.
TEST_F(TemperatureCompensationModuleTest, GyroscopeWithoutTemperatureUsesTheSameAccelerometer)
{
	TemperatureCompensationModule module;

	publishAccelSamples(31.f);
	publishGyroSamples(NAN);

	Peer::run(module);

	EXPECT_FLOAT_EQ(Peer::corrections(module).gyro_temperature[0], 31.f);
}
