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
 * Test for SensorMedian
 */

#include <gtest/gtest.h>

#include "../SensorMedian.hpp"

#include <drivers/drv_hrt.h>
#include <hrt_work.h>
#include <lib/sensor_calibration/Gyroscope.hpp>
#include <parameters/param.h>
#include <px4_platform_common/time.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/uORBManager.hpp>

using namespace time_literals;
using matrix::Vector3f;

class SensorMedianTest : public ::testing::Test
{
public:
	static constexpr int kGyroCount = 3;
	static constexpr uint32_t kGyroIds[kGyroCount] {0x2c3d01, 0x2c3d02, 0x2c3d03};

	static void SetUpTestSuite()
	{
		// the gtest harness does not bring up the hrt, see VotedSensorsUpdateTest
		hrt_init();
		hrt_work_queue_init();
		uORB::Manager::initialize();

		for (int i = 0; i < kGyroCount; i++) {
			ASSERT_TRUE(_sensor_gyro_pubs[i].advertise());
			ASSERT_EQ(_sensor_gyro_pubs[i].get_instance(), i);
		}
	}

	void SetUp() override
	{
		for (int i = 0; i < kGyroCount; i++) {
			setParam("CAL_GYRO%d_ID", i, static_cast<int32_t>(kGyroIds[i]));
			setParam("CAL_GYRO%d_PRIO", i, 50);
			setParam("CAL_GYRO%d_XOFF", i, 0.f);
		}
	}

	template<typename T>
	static void setParam(const char *format, int index, T value)
	{
		char name[20] {};
		snprintf(name, sizeof(name), format, index);
		const param_t handle = param_find(name);
		ASSERT_NE(handle, PARAM_INVALID) << name;
		ASSERT_EQ(param_set_no_notification(handle, &value), 0) << name;
	}

	static sensor_gyro_s publishGyro(int index, float x)
	{
		sensor_gyro_s sample{};
		sample.timestamp_sample = hrt_absolute_time();
		sample.timestamp = hrt_absolute_time();
		sample.device_id = kGyroIds[index];
		sample.x = x;
		_sensor_gyro_pubs[index].publish(sample);
		return sample;
	}

	static uORB::PublicationMulti<sensor_gyro_s> _sensor_gyro_pubs[kGyroCount];
};

constexpr uint32_t SensorMedianTest::kGyroIds[];
uORB::PublicationMulti<sensor_gyro_s> SensorMedianTest::_sensor_gyro_pubs[kGyroCount] {
	{ORB_ID(sensor_gyro)}, {ORB_ID(sensor_gyro)}, {ORB_ID(sensor_gyro)}
};

TEST_F(SensorMedianTest, TheMedianOfCalibratedGyrosIgnoresTheInconsistentOne)
{
	// gyro 1 reads 0.5 rad/s high and its calibration removes that
	setParam("CAL_GYRO%d_XOFF", 1, 0.5f);

	sensors::SensorMedian<sensor_gyro_s, calibration::Gyroscope> median{ORB_ID::sensor_gyro};
	median.ParametersUpdate();

	const sensor_gyro_s selected = publishGyro(0, 1.f);
	publishGyro(1, 0.5f);
	publishGyro(2, 0.02f);

	// uncalibrated, the median would be gyro 1 at 0.5 rad/s
	EXPECT_FLOAT_EQ(median.Apply(selected, Vector3f{1.f, 0.f, 0.f})(0), 0.02f);
}
