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


#include <gtest/gtest.h>
#include "checks/estimatorCheck.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/events.h>
#include <px4_platform_common/param.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/event.h>
#include <uORB/topics/vehicle_local_position.h>

#include <vector>

// to run: make tests TESTFILTER=estimatorChecks

/* EVENT
 * @skip-file
 */

using namespace time_literals;

class EstimatorChecksTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		param_control_autosave(false);

		int32_t one = 1;
		param_set(param_find("SYS_HAS_GPS"), &one);
		param_set(param_find("SENS_IMU_MODE"), &one);

		// the event topic has to exist before the first send, or the queued events are lost
		orb_advertise(ORB_ID(event), nullptr);

		_failsafe_flags = {};
		drainEvents();
	}

	// what the estimator reports: whether it fuses GNSS position, and which receiver checks fail
	void publishEstimatorStatus(bool gnss_fused, uint16_t gps_check_fail_flags)
	{
		estimator_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.control_mode_flags = gnss_fused ? (1ULL << estimator_status_s::CS_GNSS_POS) : 0;
		status.gps_check_fail_flags = gps_check_fail_flags;
		_estimator_status_pub.publish(status);
	}

	// the receiver's last sample, fresh or from before it went silent
	void publishReceiver(hrt_abstime age)
	{
		sensor_gps_s gps{};
		gps.timestamp = hrt_absolute_time() - age;
		gps.timestamp_sample = gps.timestamp;
		gps.device_id = 1;
		gps.fix_type = 3;
		_receiver_pub.publish(gps);
	}

	void publishLocalPosition(bool valid)
	{
		vehicle_local_position_s lpos{};
		lpos.timestamp = hrt_absolute_time();
		lpos.timestamp_sample = lpos.timestamp;
		lpos.xy_valid = valid;
		lpos.v_xy_valid = valid;
		lpos.z_valid = true;
		lpos.v_z_valid = true;
		lpos.eph = 0.5f;
		lpos.epv = 0.5f;
		lpos.evh = 0.2f;
		lpos.evv = 0.2f;
		_local_position_pub.publish(lpos);
	}

	// one commander cycle. The failsafe flags persist across cycles as they do in commander.
	void runCheck(bool armed)
	{
		vehicle_status_s status{};

		if (armed) {
			status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
		}

		_check.updateParams();
		Context context{status};
		Report reporter{_failsafe_flags, 0};
		_check.checkAndReport(context, reporter);
	}

	// everything published since the last drain is kept, so one test can count several ids
	void collectEvents()
	{
		event_s event;

		while (_event_sub.update(&event)) {
			_events.push_back(event);
		}
	}

	// number of events with the given id since the last drain
	int countEvents(uint32_t id, event_s *last = nullptr)
	{
		collectEvents();
		int count = 0;

		for (const event_s &event : _events) {
			if (event.id == id) {
				count++;

				if (last != nullptr) {
					*last = event;
				}
			}
		}

		return count;
	}

	void drainEvents()
	{
		collectEvents();
		_events.clear();
	}

	// a vehicle flying on GNSS with a valid position and a live receiver, the starting point of every case
	void flyOnGnss()
	{
		publishEstimatorStatus(true, 0);
		publishReceiver(0);
		publishLocalPosition(true);
		runCheck(true);
		runCheck(true);
		ASSERT_FALSE(_failsafe_flags.local_position_invalid);
		drainEvents();
	}

	static constexpr uint32_t kReasonEvent = events::ID("check_estimator_position_lost_gnss_reason");
	static constexpr uint32_t kNoDataEvent = events::ID("check_estimator_position_lost_gnss_no_data");
	static constexpr uint16_t kSpeedAccuracy = 1 << estimator_status_s::GPS_CHECK_FAIL_MAX_SPD_ERR;
	static constexpr uint16_t kSpoofed = 1 << estimator_status_s::GPS_CHECK_FAIL_SPOOFED;

	uORB::PublicationMulti<estimator_status_s> _estimator_status_pub{ORB_ID(estimator_status)};
	uORB::Publication<vehicle_local_position_s> _local_position_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<sensor_gps_s> _receiver_pub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription _event_sub{ORB_ID(event)};

	failsafe_flags_s _failsafe_flags{};
	std::vector<event_s> _events;
	EstimatorChecks _check;
};

TEST_F(EstimatorChecksTest, NamesTheFailingCheckWhenPositionIsLostInFlight)
{
	flyOnGnss();

	// the receiver's speed accuracy fails the in-flight check and the position estimate goes with it
	publishEstimatorStatus(true, kSpeedAccuracy);
	publishLocalPosition(false);
	runCheck(true);

	ASSERT_TRUE(_failsafe_flags.local_position_invalid);

	event_s event{};
	EXPECT_EQ(countEvents(kReasonEvent, &event), 1);
	uint16_t reported_flags = 0;
	memcpy(&reported_flags, event.arguments, sizeof(reported_flags));
	EXPECT_EQ(reported_flags, kSpeedAccuracy);
	EXPECT_STREQ(EstimatorChecks::gnssCheckFailText(kSpeedAccuracy), "speed accuracy too low");
}

TEST_F(EstimatorChecksTest, ReportsOnceWhilePositionStaysLost)
{
	flyOnGnss();

	publishEstimatorStatus(true, kSpeedAccuracy);
	publishLocalPosition(false);
	runCheck(true);
	ASSERT_EQ(countEvents(kReasonEvent), 1);
	drainEvents();

	for (int i = 0; i < 5; i++) {
		publishEstimatorStatus(false, kSpeedAccuracy);
		publishLocalPosition(false);
		runCheck(true);
	}

	EXPECT_EQ(countEvents(kReasonEvent), 0) << "the reason is reported when the position is lost, not while it stays lost";
}

TEST_F(EstimatorChecksTest, SpoofingIsNamedBeforeAnAccuracyCheck)
{
	flyOnGnss();

	publishEstimatorStatus(true, kSpeedAccuracy | kSpoofed);
	publishLocalPosition(false);
	runCheck(true);

	event_s event{};
	ASSERT_EQ(countEvents(kReasonEvent, &event), 1);
	uint16_t reported_flags = 0;
	memcpy(&reported_flags, event.arguments, sizeof(reported_flags));
	EXPECT_EQ(reported_flags, kSpeedAccuracy | kSpoofed) << "the event carries every failing check";
	EXPECT_STREQ(EstimatorChecks::gnssCheckFailText(kSpeedAccuracy | kSpoofed), "signal spoofed");
}

TEST_F(EstimatorChecksTest, NothingWhenNoCheckIsFailing)
{
	flyOnGnss();

	// position lost with the receiver alive and every check passing, so nothing gets blamed on GNSS
	publishEstimatorStatus(true, 0);
	publishReceiver(0);
	publishLocalPosition(false);
	runCheck(true);

	ASSERT_TRUE(_failsafe_flags.local_position_invalid);
	EXPECT_EQ(countEvents(kReasonEvent), 0);
	EXPECT_EQ(countEvents(kNoDataEvent), 0);
}

TEST_F(EstimatorChecksTest, NamesTheReceiverThatStoppedSending)
{
	flyOnGnss();

	// the receiver goes silent: no sample, so no check runs and no check bit is set
	publishEstimatorStatus(false, 0);
	publishReceiver(2_s);
	publishLocalPosition(false);
	runCheck(true);

	ASSERT_TRUE(_failsafe_flags.local_position_invalid);
	EXPECT_EQ(countEvents(kNoDataEvent), 1);
	EXPECT_EQ(countEvents(kReasonEvent), 0) << "a silent receiver is not a failing check";
}

TEST_F(EstimatorChecksTest, NothingWhenGnssWasNotInUse)
{
	// a vehicle with a valid position from another source, the receiver failing its checks all along
	publishEstimatorStatus(false, kSpeedAccuracy);
	publishReceiver(2_s);
	publishLocalPosition(true);
	runCheck(true);
	runCheck(true);
	ASSERT_FALSE(_failsafe_flags.local_position_invalid);
	drainEvents();

	publishLocalPosition(false);
	runCheck(true);

	ASSERT_TRUE(_failsafe_flags.local_position_invalid);
	EXPECT_EQ(countEvents(kReasonEvent), 0) << "a failing receiver check explains nothing when GNSS was not fused";
	EXPECT_EQ(countEvents(kNoDataEvent), 0) << "nor does a silent receiver";
}

TEST_F(EstimatorChecksTest, NothingWithoutGnssConfigured)
{
	// SYS_HAS_GPS off: GNSS is not part of this vehicle, so nothing is attributed to it
	int32_t zero = 0;
	param_set(param_find("SYS_HAS_GPS"), &zero);

	flyOnGnss();

	publishEstimatorStatus(true, kSpeedAccuracy);
	publishReceiver(2_s);
	publishLocalPosition(false);
	runCheck(true);

	ASSERT_TRUE(_failsafe_flags.local_position_invalid);
	EXPECT_EQ(countEvents(kReasonEvent), 0);
	EXPECT_EQ(countEvents(kNoDataEvent), 0);
}

TEST_F(EstimatorChecksTest, NothingWhileDisarmed)
{
	publishEstimatorStatus(true, 0);
	publishReceiver(0);
	publishLocalPosition(true);
	runCheck(false);
	runCheck(false);
	ASSERT_FALSE(_failsafe_flags.local_position_invalid);
	drainEvents();

	// on the ground the preflight checks already name the failing check
	publishEstimatorStatus(true, kSpeedAccuracy);
	publishLocalPosition(false);
	runCheck(false);

	ASSERT_TRUE(_failsafe_flags.local_position_invalid);
	EXPECT_EQ(countEvents(kReasonEvent), 0);
}
