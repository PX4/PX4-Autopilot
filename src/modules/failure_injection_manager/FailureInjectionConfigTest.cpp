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

// Functional tests for the consumer-side failure_injection helpers: the Config
// lookup, the process<>() mechanics, and the real uORB publish -> Config::update()
// round-trip. Config owns a uORB subscription, so this needs the uORB runtime and
// runs as a functional gtest.

#include <gtest/gtest.h>

#include <lib/failure_injection/FailureInjection.hpp>
#include <uORB/Publication.hpp>

// FailureInjection.hpp transitively pulls in px4_platform_common/defines.h (via
// uORB Subscription), which defines an OK macro that would clash with the local
// OK constant below.
#undef OK

using namespace failure_injection;

namespace
{

// Minimal uORB-like message with the timestamp/timestamp_sample pair the
// generic process<>() maintains.
struct FakeMsg {
	uint64_t timestamp;
	uint64_t timestamp_sample;
	float value;
};

// Some messages carry no timestamp_sample (e.g. distance_sensor_s); process<>()
// detects that at compile time and only keeps the timestamp live.
struct FakeMsgNoSample {
	uint64_t timestamp;
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

constexpr uint8_t GYRO  = failure_injection_s::FAILURE_UNIT_SENSOR_GYRO;
constexpr uint8_t GPS   = failure_injection_s::FAILURE_UNIT_SENSOR_GPS;
constexpr uint8_t MOTOR = failure_injection_s::FAILURE_UNIT_SYSTEM_MOTOR;

constexpr uint8_t OK    = failure_injection_s::FAILURE_TYPE_OK;
constexpr uint8_t OFF   = failure_injection_s::FAILURE_TYPE_OFF;
constexpr uint8_t STUCK = failure_injection_s::FAILURE_TYPE_STUCK;
constexpr uint8_t WRONG = failure_injection_s::FAILURE_TYPE_WRONG;

} // namespace

// ===========================================================================
// Config lookup (set / mode)
// ===========================================================================

TEST(FailureInjectionConfig, EmptyConfigReturnsOk)
{
	Config config;
	EXPECT_FALSE(config.any_active());
	EXPECT_EQ(config.mode(GYRO, 1), Mode::Ok);
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

TEST(FailureInjectionConfig, AllInstancesMask)
{
	Config config;
	config.set(make_config(failure_injection_s::FAILURE_UNIT_SENSOR_MAG, 0xFFFF, STUCK));

	EXPECT_EQ(config.mode(failure_injection_s::FAILURE_UNIT_SENSOR_MAG, 1), Mode::Stuck);
	EXPECT_EQ(config.mode(failure_injection_s::FAILURE_UNIT_SENSOR_MAG, 8), Mode::Stuck);
	EXPECT_EQ(config.mode(failure_injection_s::FAILURE_UNIT_SENSOR_MAG, 16), Mode::Stuck);
}

TEST(FailureInjectionConfig, MultipleEntriesResolveIndependently)
{
	failure_injection_s cfg{};
	cfg.count = 2;
	cfg.unit[0] = MOTOR;
	cfg.instance_mask[0] = 0x5; // motors 1 and 3
	cfg.failure_type[0] = OFF;
	cfg.unit[1] = GPS;
	cfg.instance_mask[1] = 0xFFFF;
	cfg.failure_type[1] = WRONG;

	Config config;
	config.set(cfg);

	EXPECT_EQ(config.mode(MOTOR, 1), Mode::Off);
	EXPECT_EQ(config.mode(MOTOR, 2), Mode::Ok);
	EXPECT_EQ(config.mode(MOTOR, 3), Mode::Off);
	EXPECT_EQ(config.mode(GPS, 1), Mode::Wrong);
}

TEST(FailureInjectionConfig, CountIsClampedToMax)
{
	failure_injection_s cfg{};
	cfg.count = 200; // absurd, must be clamped to MAX_FAILURES without reading OOB

	for (int i = 0; i < failure_injection_s::MAX_FAILURES; i++) {
		cfg.unit[i] = failure_injection_s::FAILURE_UNIT_SENSOR_BARO;
		cfg.instance_mask[i] = 0xFFFF;
		cfg.failure_type[i] = OFF;
	}

	Config config;
	config.set(cfg);
	EXPECT_TRUE(config.any_active());
	EXPECT_EQ(config.mode(failure_injection_s::FAILURE_UNIT_SENSOR_BARO, 1), Mode::Off);
}

// ===========================================================================
// uORB round-trip: publish -> Config::update() -> mode()
// ===========================================================================

TEST(FailureInjectionConfig, UpdatePicksUpPublishedSample)
{
	uORB::Publication<failure_injection_s> pub{ORB_ID(failure_injection)};
	pub.advertise();

	Config config;
	// Subscribed against the freshly advertised (empty) topic: nothing new yet.
	EXPECT_FALSE(config.update());
	EXPECT_FALSE(config.any_active());

	failure_injection_s msg = make_config(GYRO, 0x2, OFF);
	msg.timestamp = 1;
	pub.publish(msg);

	EXPECT_TRUE(config.update());
	EXPECT_TRUE(config.any_active());
	EXPECT_EQ(config.mode(GYRO, 2), Mode::Off);

	// No new sample -> update() reports no change but the cache is retained.
	EXPECT_FALSE(config.update());
	EXPECT_EQ(config.mode(GYRO, 2), Mode::Off);
}

// ===========================================================================
// process<>() Off / Stuck mechanics (single-message helper)
// ===========================================================================

TEST(FailureInjectionConfig, ProcessOffSuppresses)
{
	FakeMsg msg{100, 90, 1.0f};
	Stuck<FakeMsg> stuck;
	EXPECT_FALSE(process(Mode::Off, msg, stuck));
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

TEST(FailureInjectionConfig, ProcessStuckWithoutSnapshotLeavesMessageUnchanged)
{
	Stuck<FakeMsg> stuck; // never recorded a good sample
	FakeMsg msg{200, 190, 2.0f};
	EXPECT_TRUE(process(Mode::Stuck, msg, stuck));
	EXPECT_FLOAT_EQ(msg.value, 2.0f);
	EXPECT_EQ(msg.timestamp, 200u);
	EXPECT_EQ(msg.timestamp_sample, 190u);
}

TEST(FailureInjectionConfig, ProcessOkRecordsLastGood)
{
	Stuck<FakeMsg> stuck;
	FakeMsg msg{100, 90, 5.0f};
	EXPECT_TRUE(process(Mode::Ok, msg, stuck));
	EXPECT_TRUE(stuck.valid);
	EXPECT_FLOAT_EQ(stuck.value.value, 5.0f);
	// Message itself is untouched.
	EXPECT_FLOAT_EQ(msg.value, 5.0f);
}

TEST(FailureInjectionConfig, ProcessStuckWithoutTimestampSampleKeepsLiveTimestamp)
{
	Stuck<FakeMsgNoSample> stuck;

	FakeMsgNoSample good{100, 1.0f};
	EXPECT_TRUE(process(Mode::Ok, good, stuck));
	ASSERT_TRUE(stuck.valid);

	// Same replay mechanics for messages without a timestamp_sample field.
	FakeMsgNoSample later{200, 2.0f};
	EXPECT_TRUE(process(Mode::Stuck, later, stuck));
	EXPECT_FLOAT_EQ(later.value, 1.0f);
	EXPECT_EQ(later.timestamp, 200u);
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
