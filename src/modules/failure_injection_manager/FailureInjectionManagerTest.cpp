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


// The manager-side FailureTable and the consumer-side Config helpers, which agree on the
// instance addressing. Config owns a uORB subscription and reads parameters, so this needs
// the uORB runtime and runs as a functional gtest.

#include <gtest/gtest.h>

#include "FailureTable.hpp"

#include <lib/failure_injection/FailureInjection.hpp>
#include <parameters/param.h>
#include <uORB/topics/sensor_gnss.h>

// FailureInjection.hpp transitively pulls in px4_platform_common/defines.h (via
// uORB Subscription), which defines an OK macro that would clash with the local
// OK constant below.
#undef OK

using namespace failure_injection;
using AckResult = FailureTable::AckResult;

namespace
{

// Minimal uORB-like message with the timestamp/timestamp_sample pair the
// generic process<>() maintains.
struct FakeMsg {
	uint64_t timestamp;
	uint64_t timestamp_sample;
	float value;
};

failure_injection_s make_config(uint8_t unit, uint16_t instance_mask, uint8_t failure_type)
{
	failure_injection_s cfg{};
	cfg.count = 1;
	cfg.unit[0] = unit;
	cfg.instance_mask[0] = instance_mask;
	cfg.failure_type[0] = failure_type;
	return cfg;
}

// Healthy 3D-fix sample, as a GNSS driver would publish it.
sensor_gnss_s clean_gnss()
{
	sensor_gnss_s gnss{};
	gnss.timestamp = 1000;
	gnss.timestamp_sample = 900;
	gnss.fix_type = sensor_gnss_s::FIX_TYPE_3D;
	gnss.latitude = 47.0;
	gnss.longitude = 8.0;
	gnss.altitude_msl = 500.0;
	gnss.altitude_ellipsoid = 500.0;
	gnss.eph = 0.9f;
	gnss.epv = 1.78f;
	gnss.satellites_used = 25;
	gnss.vel_north = 1.f;
	gnss.vel_east = 2.f;
	gnss.vel_down = 0.5f;
	gnss.ground_speed = 2.236f;
	gnss.vel_ned_valid = true;
	return gnss;
}

constexpr uint8_t GYRO  = failure_injection_s::FAILURE_UNIT_SENSOR_GYRO;
constexpr uint8_t GPS   = failure_injection_s::FAILURE_UNIT_SENSOR_GPS;
constexpr uint8_t ESC   = failure_injection_s::FAILURE_UNIT_SYSTEM_ESC;

constexpr uint8_t OK    = failure_injection_s::FAILURE_TYPE_OK;
constexpr uint8_t OFF   = failure_injection_s::FAILURE_TYPE_OFF;
constexpr uint8_t STUCK = failure_injection_s::FAILURE_TYPE_STUCK;
constexpr uint8_t WRONG = failure_injection_s::FAILURE_TYPE_WRONG;

} // namespace

TEST(FailureTable, UnsupportedIsRejectedWithoutChange)
{
	FailureTable table;
	EXPECT_EQ(table.inject(GYRO, WRONG, 0), AckResult::Unsupported);
	EXPECT_FALSE(table.changed());
	EXPECT_EQ(table.count(), 0);
}

TEST(FailureTable, InjectSpecificInstanceSetsBit)
{
	FailureTable table;
	EXPECT_EQ(table.inject(GYRO, OFF, 2), AckResult::Accepted); // 1-based instance 2
	EXPECT_TRUE(table.changed());
	EXPECT_EQ(table.count(), 1);

	failure_injection_s msg{};
	table.fill(msg);
	EXPECT_EQ(msg.count, 1);
	EXPECT_EQ(msg.unit[0], GYRO);
	EXPECT_EQ(msg.failure_type[0], OFF);
	EXPECT_EQ(msg.instance_mask[0], 0x2); // bit (2 - 1)
}

TEST(FailureTable, OkClearsOnlyTheAddressedInstance)
{
	FailureTable table;
	table.inject(GYRO, OFF, 0); // mask 0xFFFF

	// Clear only instance 2.
	EXPECT_EQ(table.inject(GYRO, OK, 2), AckResult::Accepted);
	failure_injection_s msg{};
	table.fill(msg);
	ASSERT_EQ(msg.count, 1);
	EXPECT_EQ(msg.instance_mask[0], 0xFFFF & ~0x2);
}

TEST(FailureTable, NewModeOnInstanceClearsOldMode)
{
	FailureTable table;
	table.inject(GYRO, OFF, 1);   // instance 1 OFF
	table.inject(GYRO, STUCK, 1); // instance 1 now STUCK -> OFF entry must drop

	failure_injection_s msg{};
	table.fill(msg);
	ASSERT_EQ(msg.count, 1);
	EXPECT_EQ(msg.unit[0], GYRO);
	EXPECT_EQ(msg.failure_type[0], STUCK);
	EXPECT_EQ(msg.instance_mask[0], 0x1);
}

TEST(FailureTable, OverflowBeyondFourDistinctFailuresIsRejected)
{
	FailureTable table;
	EXPECT_EQ(table.inject(GYRO, OFF, 1), AckResult::Accepted);
	EXPECT_EQ(table.inject(failure_injection_s::FAILURE_UNIT_SENSOR_ACCEL, OFF, 1), AckResult::Accepted);
	EXPECT_EQ(table.inject(failure_injection_s::FAILURE_UNIT_SENSOR_MAG, OFF, 1), AckResult::Accepted);
	EXPECT_EQ(table.inject(failure_injection_s::FAILURE_UNIT_SENSOR_BARO, OFF, 1), AckResult::Accepted);
	EXPECT_EQ(table.count(), 4);

	// A fifth distinct (unit, type) must be rejected and leave the table intact.
	table.clearChanged();
	EXPECT_EQ(table.inject(GPS, OFF, 1), AckResult::Rejected);
	EXPECT_FALSE(table.changed());
	EXPECT_EQ(table.count(), 4);
}

TEST(FailureTable, AddingInstanceToExistingEntryDoesNotCountAsNew)
{
	FailureTable table;
	table.inject(GYRO, OFF, 1);
	table.inject(failure_injection_s::FAILURE_UNIT_SENSOR_ACCEL, OFF, 1);
	table.inject(failure_injection_s::FAILURE_UNIT_SENSOR_MAG, OFF, 1);
	table.inject(failure_injection_s::FAILURE_UNIT_SENSOR_BARO, OFF, 1);
	ASSERT_EQ(table.count(), 4);

	// Table is full, but extending an existing entry to another instance must succeed.
	EXPECT_EQ(table.inject(GYRO, OFF, 2), AckResult::Accepted);
	EXPECT_EQ(table.count(), 4);

	failure_injection_s msg{};
	table.fill(msg);
	// gyro entry now covers instances 1 and 2
	EXPECT_EQ(msg.instance_mask[0], 0x1 | 0x2);
}

TEST(FailureInjectionConfig, SpecificInstanceMatch)
{
	Config config;
	// instance 2 -> bit (2 - 1) = 0x2
	config.set(make_config(GYRO, 0x2, OFF));

	EXPECT_TRUE(config.any_active());
	EXPECT_EQ(config.mode(GYRO, 2), Mode::Off);
	EXPECT_EQ(config.mode(GYRO, 1), Mode::Ok);
	EXPECT_EQ(config.mode(GYRO, 3), Mode::Ok);
	// Different unit, same instance -> no match.
	EXPECT_EQ(config.mode(failure_injection_s::FAILURE_UNIT_SENSOR_ACCEL, 2), Mode::Ok);
	// instance 0 matches any instance of the unit.
	EXPECT_EQ(config.mode(GYRO, 0), Mode::Off);
}

TEST(FailureInjectionConfig, ProcessStuckReplaysValueButKeepsLiveTimestamps)
{
	Stuck<FakeMsg> stuck;

	// A good sample is recorded while no failure is active.
	FakeMsg good{100, 90, 1.0f};
	EXPECT_TRUE(process(Mode::Ok, good, stuck));
	ASSERT_TRUE(stuck.valid);

	// A later sample arrives while Stuck: the payload is frozen, but both time
	// fields stay live so consumers that require monotonic timestamps
	// (VehicleIMU, EKF2 buffers) keep consuming.
	FakeMsg later{200, 190, 2.0f};
	EXPECT_TRUE(process(Mode::Stuck, later, stuck));
	EXPECT_FLOAT_EQ(later.value, 1.0f);
	EXPECT_EQ(later.timestamp, 200u);
	EXPECT_EQ(later.timestamp_sample, 190u);
}

TEST(FailureInjectionConfig, ProcessMessageLessSuppressesOnlyOff)
{
	// Message-less overload for payload-less producers (e.g. a heartbeat flag).
	EXPECT_FALSE(process(Mode::Off));
	EXPECT_TRUE(process(Mode::Ok));
	EXPECT_TRUE(process(Mode::Stuck));

	Config config;
	config.set(make_config(GYRO, 0x2, OFF));
	// Convenience overload maps the 0-based uORB instance to the 1-based failure instance.
	EXPECT_FALSE(process(config, GYRO, 1));
	EXPECT_TRUE(process(config, GYRO, 0));
}

TEST(FailureInjectionConfig, ProcessGnssWrongLeavesPositionUntouched)
{
	param_control_autosave(false);

	const int32_t fix_type = sensor_gnss_s::FIX_TYPE_2D;
	ASSERT_EQ(param_set(param_find("SYS_FAIL_GPS_WRG"), &fix_type), 0);

	Config config;
	config.set(make_config(GPS, 0x1, WRONG));

	Stuck<sensor_gnss_s> stuck;
	const sensor_gnss_s truth = clean_gnss();
	sensor_gnss_s gnss = truth;

	EXPECT_TRUE(process_gnss(config, 0, gnss, stuck));

	// Only the fix type changes: the reported solution stays coherent with the truth.
	EXPECT_EQ(gnss.fix_type, (uint8_t)sensor_gnss_s::FIX_TYPE_2D);
	EXPECT_DOUBLE_EQ(gnss.latitude, truth.latitude);
	EXPECT_DOUBLE_EQ(gnss.longitude, truth.longitude);
	EXPECT_DOUBLE_EQ(gnss.altitude_msl, truth.altitude_msl);
	EXPECT_DOUBLE_EQ(gnss.altitude_ellipsoid, truth.altitude_ellipsoid);
	EXPECT_FLOAT_EQ(gnss.vel_north, truth.vel_north);
	EXPECT_FLOAT_EQ(gnss.vel_east, truth.vel_east);
	EXPECT_FLOAT_EQ(gnss.vel_down, truth.vel_down);
	EXPECT_FLOAT_EQ(gnss.ground_speed, truth.ground_speed);
	EXPECT_FLOAT_EQ(gnss.eph, truth.eph);
	EXPECT_EQ(gnss.satellites_used, truth.satellites_used);
}

TEST(FailureInjectionConfig, ProcessEscMapsByActuatorFunction)
{
	// Motor 3 sits on ESC slot 0; failing motor 3 must take slot 0 offline.
	Config config;
	config.set(make_config(ESC, 0x4, OFF)); // ESC 3 -> instance bit (3 - 1)

	esc_status_s status{};
	status.esc_count = 1;
	status.esc_online_flags = 0x1;
	status.esc[0].actuator_function = esc_report_s::ACTUATOR_FUNCTION_MOTOR1 + 2; // Motor 3

	status = process_esc(config, status);

	EXPECT_EQ(status.esc_online_flags & 0x1u, 0u);
}
