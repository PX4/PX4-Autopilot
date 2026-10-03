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
 * Test for VotedSensorsUpdate
 */

#include <gtest/gtest.h>

#include "../voted_sensors_update.h"

#include <drivers/drv_hrt.h>
#include <hrt_work.h>
#include <parameters/param.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/time.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/sensors_status_imu.h>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/vehicle_imu_status.h>
#include <uORB/uORBManager.hpp>

using namespace time_literals;

namespace sensors
{

// reaches the private state the tests assert on
class VotedSensorsUpdateTestPeer
{
public:
	static unsigned accelFailoverCount(VotedSensorsUpdate &voter) { return voter._accel.voter.failover_count(); }
	static unsigned gyroFailoverCount(VotedSensorsUpdate &voter) { return voter._gyro.voter.failover_count(); }
	static int32_t accelPriority(const VotedSensorsUpdate &voter, int index) { return voter._accel.priority[index]; }
	static int32_t gyroPriority(const VotedSensorsUpdate &voter, int index) { return voter._gyro.priority[index]; }
	static uint8_t accelVoterPriority(VotedSensorsUpdate &voter, int index) { return voter._accel.voter.get_sensor_priority(index); }
	static uint8_t gyroVoterPriority(VotedSensorsUpdate &voter, int index) { return voter._gyro.voter.get_sensor_priority(index); }
};

} // namespace sensors

using sensors::VotedSensorsUpdate;
using Peer = sensors::VotedSensorsUpdateTestPeer;

// VotedSensorsUpdate borrows its vehicle_imu subscriptions from the sensors module, which owns
// them as callback subscriptions on its own work item. This stands in for that owner.
class ImuSubscriptionOwner : public px4::ScheduledWorkItem
{
public:
	ImuSubscriptionOwner() : ScheduledWorkItem("voted_sensors_update_test", px4::wq_configurations::nav_and_controllers) {}

	uORB::SubscriptionCallbackWorkItem subs[sensors::MAX_SENSOR_COUNT] {
		{this, ORB_ID(vehicle_imu), 0}, {this, ORB_ID(vehicle_imu), 1},
		{this, ORB_ID(vehicle_imu), 2}, {this, ORB_ID(vehicle_imu), 3}
	};

private:
	void Run() override {}
};

class VotedSensorsUpdateTest : public ::testing::Test
{
public:
	static constexpr int kImuCount = 3;

	// IMU i publishes on instance i of every topic and is calibrated in slots CAL_ACCi and CAL_GYROi
	static constexpr uint32_t kAccelIds[kImuCount] {0x2a3b01, 0x2a3b02, 0x2a3b03};
	static constexpr uint32_t kGyroIds[kImuCount] {0x2c3d01, 0x2c3d02, 0x2c3d03};

	static constexpr uint32_t kSampleDtUs = 10000;

	// uORB nodes outlive a manager restart, so the suite brings the manager up once and keeps its topics
	static void SetUpTestSuite()
	{
		// the gtest harness does not bring up the hrt: the callback subscriptions schedule through it,
		// and without hrt_init() the callout lock is not recursive, so the worker deadlocks on
		// its own nested lock on Linux the first time a callout is due
		hrt_init();
		hrt_work_queue_init();
		uORB::Manager::initialize();

		// the voter counts its validators from the sensor_accel and sensor_gyro instances and
		// reads its data from the vehicle_imu instances, and all of them have to line up by index
		for (int i = 0; i < kImuCount; i++) {
			ASSERT_TRUE(_sensor_accel_pubs[i].advertise());
			ASSERT_EQ(_sensor_accel_pubs[i].get_instance(), i);
			ASSERT_TRUE(_sensor_gyro_pubs[i].advertise());
			ASSERT_EQ(_sensor_gyro_pubs[i].get_instance(), i);
			ASSERT_TRUE(_vehicle_imu_pubs[i].advertise());
			ASSERT_EQ(_vehicle_imu_pubs[i].get_instance(), i);
			ASSERT_TRUE(_vehicle_imu_status_pubs[i].advertise());
			ASSERT_EQ(_vehicle_imu_status_pubs[i].get_instance(), i);
		}
	}

	// every test starts with every IMU disabled and enables the ones it uses
	void SetUp() override
	{
		setParam("SENS_IMU_MODE", 1);

		for (int i = 0; i < kImuCount; i++) {
			setPriority(i, 0, 0);
		}

		drainLogs();
	}

	static void setParam(const char *name, int32_t value)
	{
		const param_t handle = param_find(name);
		ASSERT_NE(handle, PARAM_INVALID) << name;
		ASSERT_EQ(param_set_no_notification(handle, &value), 0) << name;
	}

	static void setPriority(int imu, int32_t accel_priority, int32_t gyro_priority)
	{
		char name[20] {};
		snprintf(name, sizeof(name), "CAL_ACC%d_ID", imu);
		setParam(name, static_cast<int32_t>(kAccelIds[imu]));
		snprintf(name, sizeof(name), "CAL_ACC%d_PRIO", imu);
		setParam(name, accel_priority);
		snprintf(name, sizeof(name), "CAL_GYRO%d_ID", imu);
		setParam(name, static_cast<int32_t>(kGyroIds[imu]));
		snprintf(name, sizeof(name), "CAL_GYRO%d_PRIO", imu);
		setParam(name, gyro_priority);
	}

	// what VehicleIMU publishes for an enabled IMU: one integrated sample and its status. A
	// real sensor never repeats a value exactly, and the voter flags one that does as stale,
	// so every sample carries a little deterministic noise.
	static void publishImu(int imu, float accel_z, float gyro_x = 0.f)
	{
		static uint32_t sample_count = 0;
		const float noise = 1e-3f * static_cast<float>(sample_count++ % 7);
		accel_z += noise;
		gyro_x += noise;

		vehicle_imu_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.accel_device_id = kAccelIds[imu];
		status.gyro_device_id = kGyroIds[imu];
		_vehicle_imu_status_pubs[imu].publish(status);

		vehicle_imu_s sample{};
		sample.timestamp_sample = hrt_absolute_time();
		sample.timestamp = sample.timestamp_sample;
		sample.accel_device_id = kAccelIds[imu];
		sample.gyro_device_id = kGyroIds[imu];
		sample.delta_velocity_dt = kSampleDtUs;
		sample.delta_angle_dt = kSampleDtUs;
		sample.delta_velocity[2] = accel_z * 1e-6f * kSampleDtUs;
		sample.delta_angle[0] = gyro_x * 1e-6f * kSampleDtUs;
		_vehicle_imu_pubs[imu].publish(sample);
	}

	// run the voter at the rate the sensors module polls it, feeding it the given samples
	template<typename Feed>
	static void pollFor(VotedSensorsUpdate &voter, hrt_abstime duration, Feed feed)
	{
		const hrt_abstime start = hrt_absolute_time();
		sensor_combined_s raw{};

		do {
			feed();
			voter.sensorsPoll(raw);
			px4_usleep(10_ms);
		} while (hrt_absolute_time() < start + duration);
	}

	// the voter reads the calibration priorities against the device ids it has seen, so it
	// needs one sample from every enabled IMU before the priorities can take effect
	template<typename Feed>
	static void start(VotedSensorsUpdate &voter, Feed feed)
	{
		feed();
		voter.parametersUpdate();
		pollFor(voter, 300_ms, feed);
	}

	bool readSelection(sensor_selection_s &selection) { return _sensor_selection_sub.copy(&selection); }

	bool readStatus(sensors_status_imu_s &status) { return _sensors_status_imu_sub.copy(&status); }

	// consumes every mavlink_log message published since the last call and returns them joined
	std::string drainLogs()
	{
		std::string logs;
		mavlink_log_s log;

		while (_mavlink_log_sub.update(&log)) {
			logs += log.text;
			logs += "; ";
		}

		return logs;
	}

	static uORB::PublicationMulti<sensor_accel_s> _sensor_accel_pubs[kImuCount];
	static uORB::PublicationMulti<sensor_gyro_s> _sensor_gyro_pubs[kImuCount];
	static uORB::PublicationMulti<vehicle_imu_s> _vehicle_imu_pubs[kImuCount];
	static uORB::PublicationMulti<vehicle_imu_status_s> _vehicle_imu_status_pubs[kImuCount];

	uORB::Subscription _sensor_selection_sub{ORB_ID(sensor_selection)};
	uORB::Subscription _sensors_status_imu_sub{ORB_ID(sensors_status_imu)};
	uORB::Subscription _mavlink_log_sub{ORB_ID(mavlink_log)};
};

constexpr uint32_t VotedSensorsUpdateTest::kAccelIds[];
constexpr uint32_t VotedSensorsUpdateTest::kGyroIds[];
uORB::PublicationMulti<sensor_accel_s> VotedSensorsUpdateTest::_sensor_accel_pubs[kImuCount] {
	{ORB_ID(sensor_accel)}, {ORB_ID(sensor_accel)}, {ORB_ID(sensor_accel)}
};
uORB::PublicationMulti<sensor_gyro_s> VotedSensorsUpdateTest::_sensor_gyro_pubs[kImuCount] {
	{ORB_ID(sensor_gyro)}, {ORB_ID(sensor_gyro)}, {ORB_ID(sensor_gyro)}
};
uORB::PublicationMulti<vehicle_imu_s> VotedSensorsUpdateTest::_vehicle_imu_pubs[kImuCount] {
	{ORB_ID(vehicle_imu)}, {ORB_ID(vehicle_imu)}, {ORB_ID(vehicle_imu)}
};
uORB::PublicationMulti<vehicle_imu_status_s> VotedSensorsUpdateTest::_vehicle_imu_status_pubs[kImuCount] {
	{ORB_ID(vehicle_imu_status)}, {ORB_ID(vehicle_imu_status)}, {ORB_ID(vehicle_imu_status)}
};

TEST_F(VotedSensorsUpdateTest, DisablingTheSelectedImuHandsOverWithoutAFailover)
{
	setPriority(0, 75, 75);
	setPriority(1, 50, 50);

	ImuSubscriptionOwner owner;
	VotedSensorsUpdate voter(false, owner.subs);

	start(voter, [&]() {
		publishImu(0, 9.8f);
		publishImu(1, 9.8f);
	});

	sensor_selection_s selection{};
	ASSERT_TRUE(readSelection(selection));
	ASSERT_EQ(selection.accel_device_id, kAccelIds[0]);
	ASSERT_EQ(selection.gyro_device_id, kGyroIds[0]);
	ASSERT_EQ(Peer::accelFailoverCount(voter), 0u);
	ASSERT_EQ(Peer::gyroFailoverCount(voter), 0u);
	drainLogs();

	// the operator disables IMU 0 while it is selected, and VehicleIMU stops publishing it
	setPriority(0, 0, 0);
	voter.parametersUpdate();

	pollFor(voter, 600_ms, [&]() { publishImu(1, 9.8f); });

	ASSERT_TRUE(readSelection(selection));
	EXPECT_EQ(selection.accel_device_id, kAccelIds[1]);
	EXPECT_EQ(selection.gyro_device_id, kGyroIds[1]);
	EXPECT_EQ(Peer::accelFailoverCount(voter), 0u) << "leaving a disabled IMU is not a failure";
	EXPECT_EQ(Peer::gyroFailoverCount(voter), 0u);
	EXPECT_EQ(drainLogs(), "") << "a deliberate disable must not raise a sensor failure";

	// well past the voter timeout the IMU has stayed disabled rather than coming back at priority 1
	EXPECT_EQ(Peer::accelPriority(voter, 0), 0);
	EXPECT_EQ(Peer::gyroPriority(voter, 0), 0);
	EXPECT_EQ(Peer::accelVoterPriority(voter, 0), 0);
	EXPECT_EQ(Peer::gyroVoterPriority(voter, 0), 0);

	sensors_status_imu_s status{};
	ASSERT_TRUE(readStatus(status));
	EXPECT_EQ(status.accel_device_id_primary, kAccelIds[1]);
	EXPECT_EQ(status.gyro_device_id_primary, kGyroIds[1]);
	EXPECT_EQ(status.accel_device_ids[0], 0u) << "a disabled IMU is not reported as an enabled one";
	EXPECT_EQ(status.gyro_device_ids[0], 0u);
	EXPECT_EQ(status.accel_device_ids[1], kAccelIds[1]);
	EXPECT_EQ(status.accel_priority[1], 50);
}

TEST_F(VotedSensorsUpdateTest, ADisabledImuStaysOutOfTheConsistencyCheck)
{
	setPriority(0, 75, 75);
	setPriority(1, 50, 50);

	ImuSubscriptionOwner owner;
	VotedSensorsUpdate voter(false, owner.subs);

	start(voter, [&]() {
		publishImu(0, 9.8f);
		publishImu(1, 9.8f);
	});

	setPriority(0, 0, 0);
	voter.parametersUpdate();

	// IMU 0 is frozen at its last sample while IMU 1 sees the vehicle move. Compared against the
	// frozen sample, the live one would look inconsistent and fail the preflight check.
	float accel_z = 9.8f;
	pollFor(voter, 600_ms, [&]() {
		accel_z += 0.05f;
		publishImu(1, accel_z);
	});

	sensors_status_imu_s status{};
	ASSERT_TRUE(readStatus(status));
	EXPECT_EQ(status.accel_device_ids[0], 0u);
	EXPECT_LT(status.accel_inconsistency_m_s_s[1], 0.01f) << "the only enabled IMU is its own reference";
	EXPECT_LT(status.accel_inconsistency_m_s_s[0], 0.01f);
}

TEST_F(VotedSensorsUpdateTest, DisablingOnlyTheAccelerometerSilencesTheImuWithoutAFailover)
{
	setPriority(0, 75, 75);
	setPriority(1, 50, 50);

	ImuSubscriptionOwner owner;
	VotedSensorsUpdate voter(false, owner.subs);

	start(voter, [&]() {
		publishImu(0, 9.8f);
		publishImu(1, 9.8f);
	});

	drainLogs();

	// only the accelerometer of IMU 0 is disabled, but VehicleIMU stops the whole IMU, so the
	// gyro of IMU 0 goes silent with its priority untouched
	setPriority(0, 0, 75);
	voter.parametersUpdate();

	pollFor(voter, 600_ms, [&]() { publishImu(1, 9.8f); });

	sensor_selection_s selection{};
	ASSERT_TRUE(readSelection(selection));
	EXPECT_EQ(selection.accel_device_id, kAccelIds[1]);
	EXPECT_EQ(selection.gyro_device_id, kGyroIds[1]);
	EXPECT_EQ(Peer::accelFailoverCount(voter), 0u);
	EXPECT_EQ(Peer::gyroFailoverCount(voter), 0u) << "the silent gyro belongs to a disabled IMU";
	EXPECT_EQ(drainLogs(), "");

	sensors_status_imu_s status{};
	ASSERT_TRUE(readStatus(status));
	EXPECT_EQ(status.accel_device_ids[0], 0u);
	EXPECT_EQ(status.gyro_device_ids[0], 0u) << "the gyro of a disabled IMU is not an enabled sensor";
	EXPECT_EQ(status.gyro_device_ids[1], kGyroIds[1]);
}

TEST_F(VotedSensorsUpdateTest, ReEnablingAnImuMakesItSelectableAgain)
{
	setPriority(0, 75, 75);
	setPriority(1, 50, 50);

	ImuSubscriptionOwner owner;
	VotedSensorsUpdate voter(false, owner.subs);

	start(voter, [&]() {
		publishImu(0, 9.8f);
		publishImu(1, 9.8f);
	});

	setPriority(0, 0, 0);
	voter.parametersUpdate();
	pollFor(voter, 300_ms, [&]() { publishImu(1, 9.8f); });

	sensor_selection_s selection{};
	ASSERT_TRUE(readSelection(selection));
	ASSERT_EQ(selection.accel_device_id, kAccelIds[1]);

	// enabled again at the higher priority, VehicleIMU resumes publishing it
	setPriority(0, 75, 75);
	voter.parametersUpdate();
	drainLogs();

	pollFor(voter, 600_ms, [&]() {
		publishImu(0, 9.8f);
		publishImu(1, 9.8f);
	});

	ASSERT_TRUE(readSelection(selection));
	EXPECT_EQ(selection.accel_device_id, kAccelIds[0]);
	EXPECT_EQ(selection.gyro_device_id, kGyroIds[0]);
	EXPECT_EQ(Peer::accelFailoverCount(voter), 0u) << "preferring a higher priority is not a failure";
	EXPECT_EQ(Peer::gyroFailoverCount(voter), 0u);
	EXPECT_EQ(drainLogs(), "");
	EXPECT_EQ(Peer::accelPriority(voter, 0), 75);
	EXPECT_EQ(Peer::accelVoterPriority(voter, 0), 75);

	sensors_status_imu_s status{};
	ASSERT_TRUE(readStatus(status));
	EXPECT_EQ(status.accel_device_ids[0], kAccelIds[0]);
	EXPECT_EQ(status.accel_device_ids[1], kAccelIds[1]);
}

TEST_F(VotedSensorsUpdateTest, AnImuDisabledAtBootIsNeverSelected)
{
	// IMU 1 is disabled before the voter ever sees it and never publishes, IMU 2 sits after it
	setPriority(0, 75, 75);
	setPriority(2, 50, 50);

	ImuSubscriptionOwner owner;
	VotedSensorsUpdate voter(false, owner.subs);

	start(voter, [&]() {
		publishImu(0, 9.8f);
		publishImu(2, 9.8f);
	});

	sensor_selection_s selection{};
	ASSERT_TRUE(readSelection(selection));
	ASSERT_EQ(selection.accel_device_id, kAccelIds[0]);
	ASSERT_EQ(Peer::accelFailoverCount(voter), 0u);

	sensors_status_imu_s status{};
	ASSERT_TRUE(readStatus(status));
	EXPECT_EQ(status.accel_device_ids[1], 0u);
	EXPECT_EQ(status.accel_device_ids[2], kAccelIds[2]);

	// IMU 0 really fails, so the voter has to hand over to IMU 2 past the disabled one
	pollFor(voter, 600_ms, [&]() { publishImu(2, 9.8f); });

	ASSERT_TRUE(readSelection(selection));
	EXPECT_EQ(selection.accel_device_id, kAccelIds[2]);
	EXPECT_EQ(selection.gyro_device_id, kGyroIds[2]);
	EXPECT_EQ(Peer::accelFailoverCount(voter), 1u) << "a sensor that stopped is a real failure";
	EXPECT_NE(drainLogs(), "") << "a real failure is reported";
}
