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
 * Test for VehicleMagnetometer
 */

#include <gtest/gtest.h>

#include "../VehicleMagnetometer.hpp"

#include <drivers/drv_hrt.h>
#include <hrt_work.h>
#include <parameters/param.h>
#include <px4_platform_common/time.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/sensor_preflight_mag.h>
#include <uORB/topics/sensors_status.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/uORBManager.hpp>

using matrix::Vector3f;

class VehicleMagnetometerTest : public ::testing::Test
{
public:
	class VehicleMagnetometerTestable : public sensors::VehicleMagnetometer
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
		unsigned failoverCount() { return voterFailoverCount(); }
		bool callbackRegistered(int index) const { return VehicleMagnetometer::callbackRegistered(index); }
		uint8_t priority(int index) const { return sensorPriority(index); }
	};

	static constexpr int kSensorCount = 3;

	// device i publishes on sensor_mag instance i and is calibrated in slot CAL_MAGi for the whole suite
	static constexpr uint32_t kDeviceIds[kSensorCount] {0x1a2b01, 0x1a2b02, 0x1a2b03};

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
			ASSERT_TRUE(_sensor_mag_pubs[i].advertise());
			ASSERT_EQ(_sensor_mag_pubs[i].get_instance(), i);
		}
	}

	// every test starts with all three devices disabled and enables the ones it uses
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

	static void setPriority(int device, int32_t priority)
	{
		char name[20] {};
		snprintf(name, sizeof(name), "CAL_MAG%d_ID", device);
		setParam(name, static_cast<int32_t>(kDeviceIds[device]));
		snprintf(name, sizeof(name), "CAL_MAG%d_PRIO", device);
		setParam(name, priority);
	}

	static void publishSample(int device, const Vector3f &field)
	{
		sensor_mag_s sample{};
		sample.timestamp_sample = hrt_absolute_time();
		sample.timestamp = sample.timestamp_sample;
		sample.device_id = kDeviceIds[device];
		sample.is_external = true;
		sample.x = field(0);
		sample.y = field(1);
		sample.z = field(2);
		_sensor_mag_pubs[device].publish(sample);
	}

	static void publishParameterUpdate()
	{
		parameter_update_s update{};
		update.timestamp = hrt_absolute_time();
		_parameter_update_pub.publish(update);
	}

	// run the module at its own 50 ms cadence for the given time, feeding it the given samples
	template<typename Feed>
	static void runFor(VehicleMagnetometerTestable &module, hrt_abstime duration, Feed feed)
	{
		const hrt_abstime start = hrt_absolute_time();

		do {
			feed();
			module.RunPublic();
			px4_usleep(50_ms);
		} while (hrt_absolute_time() < start + duration);
	}

	bool readStatus(sensors_status_s &status) { return _sensors_status_mag_sub.copy(&status); }

	bool readPreflight(sensor_preflight_mag_s &preflight) { return _sensor_preflight_mag_sub.copy(&preflight); }

	// consumes everything published since the last call
	bool magnetometerPublished()
	{
		bool published = false;
		vehicle_magnetometer_s unused;

		while (_vehicle_magnetometer_sub.update(&unused)) {
			published = true;
		}

		return published;
	}

	static uORB::PublicationMulti<sensor_mag_s> _sensor_mag_pubs[kSensorCount];
	static uORB::Publication<parameter_update_s> _parameter_update_pub;

	uORB::Subscription _sensors_status_mag_sub{ORB_ID(sensors_status_mag)};
	uORB::Subscription _sensor_preflight_mag_sub{ORB_ID(sensor_preflight_mag)};
	uORB::Subscription _vehicle_magnetometer_sub{ORB_ID(vehicle_magnetometer)};

	const Vector3f _field{0.2f, 0.f, 0.4f};
};

constexpr uint32_t VehicleMagnetometerTest::kDeviceIds[];
uORB::PublicationMulti<sensor_mag_s> VehicleMagnetometerTest::_sensor_mag_pubs[kSensorCount] {
	{ORB_ID(sensor_mag)}, {ORB_ID(sensor_mag)}, {ORB_ID(sensor_mag)}
};
uORB::Publication<parameter_update_s> VehicleMagnetometerTest::_parameter_update_pub{ORB_ID(parameter_update)};

TEST_F(VehicleMagnetometerTest, DisabledAtRuntimeHandsOverWithoutFailover)
{
	setPriority(0, 75);
	setPriority(1, 50);

	VehicleMagnetometerTestable module;

	runFor(module, 500_ms, [&]() {
		publishSample(0, _field);
		publishSample(1, _field);
	});

	ASSERT_EQ(module.selectedIndex(), 0);
	ASSERT_EQ(module.failoverCount(), 0u);

	// the operator disables device 0 while it is selected, and it keeps publishing
	setPriority(0, 0);
	publishParameterUpdate();

	// long enough for the module's throttled parameter subscription to pick up the change
	runFor(module, 1200_ms, [&]() {
		publishSample(0, _field);
		publishSample(1, _field);
	});

	EXPECT_EQ(module.selectedIndex(), 1);
	EXPECT_EQ(module.priority(0), 0);
	EXPECT_EQ(module.failoverCount(), 0u) << "leaving a disabled magnetometer is not a failure";

	sensors_status_s status{};
	ASSERT_TRUE(readStatus(status));
	EXPECT_EQ(status.device_id_primary, kDeviceIds[1]);
	EXPECT_EQ(status.priority[0], 0);
	EXPECT_FALSE(status.enabled[0]);
	EXPECT_TRUE(status.enabled[1]);

	// device 0 now reports the opposite field, which must not count against the consistency check
	runFor(module, 500_ms, [&]() {
		publishSample(0, -_field);
		publishSample(1, _field);
	});

	EXPECT_EQ(module.priority(0), 0);

	sensor_preflight_mag_s preflight{};
	ASSERT_TRUE(readPreflight(preflight));
	EXPECT_LT(preflight.mag_inconsistency_angle, 0.01f);
}

TEST_F(VehicleMagnetometerTest, MagnetometerAfterDisabledSlotIsVotedOn)
{
	// device 1 is disabled before the module ever sees it, and device 2 sits after it in instance order
	setPriority(0, 75);
	setPriority(2, 75);

	VehicleMagnetometerTestable module;

	runFor(module, 500_ms, [&]() {
		publishSample(0, _field);
		publishSample(1, _field);
		publishSample(2, _field);
	});

	ASSERT_EQ(module.selectedIndex(), 0);
	EXPECT_EQ(module.priority(1), 0);

	// device 0 stops, so the voter has to hand over to device 2 past the disabled one
	runFor(module, 600_ms, [&]() {
		publishSample(1, _field);
		publishSample(2, _field);
	});

	EXPECT_EQ(module.selectedIndex(), 2);
	EXPECT_EQ(module.priority(1), 0);

	sensors_status_s status{};
	ASSERT_TRUE(readStatus(status));
	EXPECT_EQ(status.device_id_primary, kDeviceIds[2]);
	EXPECT_FALSE(status.enabled[1]);
	EXPECT_TRUE(status.healthy[2]);
}

TEST_F(VehicleMagnetometerTest, OnlyMagnetometerDisabledClearsSelectionAndComesBack)
{
	setPriority(0, 75);

	VehicleMagnetometerTestable module;

	runFor(module, 500_ms, [&]() { publishSample(0, _field); });

	ASSERT_EQ(module.selectedIndex(), 0);
	ASSERT_TRUE(magnetometerPublished());

	setPriority(0, 0);
	publishParameterUpdate();

	runFor(module, 1200_ms, [&]() { publishSample(0, _field); });

	EXPECT_EQ(module.selectedIndex(), -1);
	EXPECT_EQ(module.failoverCount(), 0u);

	// the status keeps reporting with no primary, so the ground station sees the disabled sensor
	sensors_status_s status{};
	ASSERT_TRUE(readStatus(status));
	EXPECT_EQ(status.device_id_primary, 0u);
	EXPECT_FALSE(status.enabled[0]);
	EXPECT_EQ(status.priority[0], 0);
	EXPECT_TRUE(status.healthy[0]);

	(void)magnetometerPublished();
	runFor(module, 300_ms, [&]() { publishSample(0, _field); });
	EXPECT_FALSE(magnetometerPublished()) << "a disabled magnetometer is not published as vehicle_magnetometer";

	// re-enabled at an explicit priority, the sensor comes back at exactly that priority
	setPriority(0, 60);
	publishParameterUpdate();

	runFor(module, 1200_ms, [&]() { publishSample(0, _field); });

	EXPECT_EQ(module.selectedIndex(), 0);
	EXPECT_EQ(module.priority(0), 60);
	EXPECT_EQ(module.failoverCount(), 0u);
	EXPECT_TRUE(magnetometerPublished());
}

TEST_F(VehicleMagnetometerTest, MagnetometerDisabledFromBootDoesNotDriveRun)
{
	// the only magnetometer is disabled before the module sees it: nothing ever gets selected, so
	// nothing ever clears the callbacks, and one registered here would wake Run() at the sample rate
	VehicleMagnetometerTestable module;

	runFor(module, 500_ms, [&]() { publishSample(0, _field); });

	EXPECT_EQ(module.selectedIndex(), -1);
	EXPECT_EQ(module.priority(0), 0);
	EXPECT_FALSE(module.callbackRegistered(0));

	setPriority(0, 75);
	publishParameterUpdate();

	runFor(module, 1200_ms, [&]() { publishSample(0, _field); });

	EXPECT_EQ(module.selectedIndex(), 0);
	EXPECT_TRUE(module.callbackRegistered(0));
}
