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
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/estimator_status_flags.h>
#include <uORB/topics/health_report.h>
#include <uORB/topics/sensors_status_gnss.h>
#include <uORB/topics/vehicle_gnss.h>
#include <uORB/topics/event.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
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

		setParam("SYS_HAS_GPS", 1);
		setParam("SENS_IMU_MODE", 1);

		// back to their defaults, the cases below change them
		param_reset(param_find("COM_ARM_MAG_STR"));
		param_reset(param_find("COM_ARM_WO_GPS"));
		param_reset(param_find("COM_POS_LOW_EPH"));
		param_reset(param_find("COM_POS_LOW_ACT"));

		// the event topic has to exist before the first send, or the queued events are lost
		orb_advertise(ORB_ID(event), nullptr);

		_failsafe_flags = {};
		// every mode requires attitude and a local and global position, so a failed check shows on every mode
		_failsafe_flags.mode_req_attitude = ~0u;
		_failsafe_flags.mode_req_local_position = ~0u;
		_failsafe_flags.mode_req_global_position = ~0u;

		// the topics keep their last sample across cases, so each case starts from the same clean state
		publishEstimatorStatus(false, 0);
		estimator_status_flags_s flags{};
		flags.timestamp = hrt_absolute_time();
		flags.cs_yaw_align = true;
		publishStatusFlags(flags);
		estimator_sensor_bias_s bias{};
		bias.timestamp = hrt_absolute_time();
		publishSensorBias(bias);
		vehicle_global_position_s gpos{};
		gpos.timestamp = hrt_absolute_time();
		publishGlobalPosition(gpos);
		vehicle_attitude_s attitude{};
		attitude.timestamp = hrt_absolute_time();
		attitude.q[0] = 1.f;
		publishAttitude(attitude);
		vehicle_angular_velocity_s rates{};
		rates.timestamp = hrt_absolute_time();
		publishAngularVelocity(rates);
		publishHeadingState(sensors_status_gnss_s::HEADING_NONE);

		drainEvents();
	}

	void setParam(const char *name, int32_t value) { param_set(param_find(name), &value); }
	void setParam(const char *name, float value) { param_set(param_find(name), &value); }

	void publishEstimatorStatus(const estimator_status_s &status) { _estimator_status_pub.publish(status); }
	void publishStatusFlags(const estimator_status_flags_s &flags) { _status_flags_pub.publish(flags); }
	void publishSensorBias(const estimator_sensor_bias_s &bias) { _sensor_bias_pub.publish(bias); }
	void publishGlobalPosition(const vehicle_global_position_s &gpos) { _global_position_pub.publish(gpos); }
	void publishAttitude(const vehicle_attitude_s &attitude) { _attitude_pub.publish(attitude); }
	void publishAngularVelocity(const vehicle_angular_velocity_s &rates) { _angular_velocity_pub.publish(rates); }

	void publishHeadingState(uint8_t heading_state)
	{
		sensors_status_gnss_s status{};
		status.timestamp = hrt_absolute_time();
		status.heading_state = heading_state;
		_sensors_status_gnss_pub.publish(status);
	}

	void publishHeadingMissing(bool missing)
	{
		estimator_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.control_mode_flags = 1ULL << estimator_status_s::CS_GNSS_POS;
		status.pre_flt_fail_gnss_heading_missing = missing;
		_estimator_status_pub.publish(status);
	}

	// what the estimator reports: whether it fuses GNSS position, and which receiver checks fail.
	// An age stands in for a report seen that long ago.
	void publishEstimatorStatus(bool gnss_fused, uint16_t gps_check_fail_flags, hrt_abstime age = 0)
	{
		estimator_status_s status{};
		status.timestamp = hrt_absolute_time() - age;
		status.control_mode_flags = gnss_fused ? (1ULL << estimator_status_s::CS_GNSS_POS) : 0;
		status.gps_check_fail_flags = gps_check_fail_flags;
		_estimator_status_pub.publish(status);
	}

	// the receiver's last sample, fresh or from before it went silent
	void publishReceiver(hrt_abstime age)
	{
		vehicle_gnss_s gnss{};
		gnss.timestamp = hrt_absolute_time() - age;
		gnss.timestamp_sample = gnss.timestamp;
		gnss.receiver.timestamp = gnss.timestamp;
		gnss.receiver.timestamp_sample = gnss.timestamp;
		gnss.receiver.device_id = 1;
		gnss.receiver.fix_type = 3;
		_receiver_pub.publish(gnss);
	}

	void publishLocalPosition(bool valid, float eph = 0.5f, bool global_origin = false)
	{
		vehicle_local_position_s lpos{};
		lpos.timestamp = hrt_absolute_time();
		lpos.timestamp_sample = lpos.timestamp;
		lpos.xy_valid = valid;
		lpos.v_xy_valid = valid;
		lpos.z_valid = true;
		lpos.v_z_valid = true;
		lpos.xy_global = global_origin;
		lpos.eph = eph;
		lpos.epv = 0.5f;
		lpos.evh = 0.2f;
		lpos.evv = 0.2f;
		_local_position_pub.publish(lpos);
	}

	// one commander cycle. The failsafe flags persist across cycles as they do in commander, and the
	// health report of the cycle is kept for the assertions.
	void runCheck(bool armed, bool calibrating = false)
	{
		vehicle_status_s status{};

		if (armed) {
			status.arming_state = vehicle_status_s::ARMING_STATE_ARMED;
		}

		status.calibration_enabled = calibrating;

		_check.updateParams();
		Context context{status};
		Report reporter{_failsafe_flags, 0};
		_check.checkAndReport(context, reporter);
		reporter.getHealthReport(_report);
	}

	bool armingError(health_component_t component) const
	{
		return _report.arming_check_error_flags & (uint64_t)component;
	}

	bool armingWarning(health_component_t component) const
	{
		return _report.arming_check_warning_flags & (uint64_t)component;
	}

	bool isPresent(health_component_t component) const
	{
		return _report.health_is_present_flags & (uint64_t)component;
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
	static constexpr uint32_t kFusionStartedEvent = events::ID("check_estimator_gnss_fusion_started");
	static constexpr uint32_t kFusionStoppedEvent = events::ID("check_estimator_gnss_fusion_stopped");
	static constexpr uint32_t kSpoofingEvent = events::ID("check_estimator_gnss_warning_spoofing");
	static constexpr uint32_t kJammingEvent = events::ID("check_estimator_gnss_warning_jamming");
	static constexpr uint32_t kFailureImminentEvent = events::ID("check_estimator_position_failure_imminent");
	static constexpr uint16_t kSpeedAccuracy = 1 << estimator_status_s::GPS_CHECK_FAIL_MAX_SPD_ERR;
	static constexpr uint16_t kSpoofed = 1 << estimator_status_s::GPS_CHECK_FAIL_SPOOFED;
	static constexpr uint16_t kJammed = 1 << estimator_status_s::GPS_CHECK_FAIL_JAMMED;
	static constexpr uint16_t kFixTooLow = 1 << estimator_status_s::GPS_CHECK_FAIL_GPS_FIX;

	uORB::PublicationMulti<estimator_status_s> _estimator_status_pub{ORB_ID(estimator_status)};
	uORB::PublicationMulti<estimator_status_flags_s> _status_flags_pub{ORB_ID(estimator_status_flags)};
	uORB::PublicationMulti<estimator_sensor_bias_s> _sensor_bias_pub{ORB_ID(estimator_sensor_bias)};
	uORB::Publication<vehicle_local_position_s> _local_position_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<vehicle_global_position_s> _global_position_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<vehicle_attitude_s> _attitude_pub{ORB_ID(vehicle_attitude)};
	uORB::Publication<vehicle_angular_velocity_s> _angular_velocity_pub{ORB_ID(vehicle_angular_velocity)};
	uORB::Publication<vehicle_gnss_s> _receiver_pub{ORB_ID(vehicle_gnss)};
	uORB::Publication<sensors_status_gnss_s> _sensors_status_gnss_pub{ORB_ID(sensors_status_gnss)};
	uORB::Subscription _event_sub{ORB_ID(event)};

	health_report_s _report{};
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
}

TEST_F(EstimatorChecksTest, NamesACheckThatFailedShortlyBeforePositionWasLost)
{
	flyOnGnss();

	// the check fails on one sample, passes again on the next, and the position goes a cycle later.
	// EKF2 was still rejecting samples from the failure, so it is the reason even though the newest
	// sample is clean.
	publishEstimatorStatus(true, kSpeedAccuracy);
	publishReceiver(0);
	publishLocalPosition(true);
	runCheck(true);

	publishEstimatorStatus(true, 0);
	publishReceiver(0);
	publishLocalPosition(true);
	runCheck(true);
	ASSERT_FALSE(_failsafe_flags.local_position_invalid);
	drainEvents();

	publishEstimatorStatus(true, 0);
	publishReceiver(0);
	publishLocalPosition(false);
	runCheck(true);

	ASSERT_TRUE(_failsafe_flags.local_position_invalid);
	event_s event{};
	ASSERT_EQ(countEvents(kReasonEvent, &event), 1) << "the check that kept GNSS out is the reason";
	uint16_t reported_flags = 0;
	memcpy(&reported_flags, event.arguments, sizeof(reported_flags));
	EXPECT_EQ(reported_flags, kSpeedAccuracy);
}

TEST_F(EstimatorChecksTest, ForgetsACheckThatStoppedFailingLongBefore)
{
	flyOnGnss();

	// spoofing was flagged once, well before the speed accuracy failed and the position went. Each
	// check expires on its own, so the speed accuracy failing doesn't keep the spoofing in the reason.
	publishEstimatorStatus(true, kSpoofed, 15_s);
	runCheck(true);
	ASSERT_FALSE(_failsafe_flags.local_position_invalid);
	drainEvents();

	publishEstimatorStatus(true, kSpeedAccuracy);
	publishReceiver(0);
	publishLocalPosition(false);
	runCheck(true);

	ASSERT_TRUE(_failsafe_flags.local_position_invalid);
	event_s event{};
	ASSERT_EQ(countEvents(kReasonEvent, &event), 1);
	uint16_t reported_flags = 0;
	memcpy(&reported_flags, event.arguments, sizeof(reported_flags));
	EXPECT_EQ(reported_flags, kSpeedAccuracy);
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

TEST_F(EstimatorChecksTest, EveryFailingCheckIsInTheEvent)
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

TEST_F(EstimatorChecksTest, NamesTheReceiverThatStoppedAfterAFailingSample)
{
	flyOnGnss();

	// the last sample before the receiver went quiet failed a check, and EKF2 keeps those flags
	// since no newer sample replaces them. The silence is still the reason.
	publishEstimatorStatus(true, kSpeedAccuracy);
	publishReceiver(2_s);
	publishLocalPosition(false);
	runCheck(true);

	ASSERT_TRUE(_failsafe_flags.local_position_invalid);
	EXPECT_EQ(countEvents(kNoDataEvent), 1);
	EXPECT_EQ(countEvents(kReasonEvent), 0) << "flags left by the last sample do not outrank the silence";
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

// The rest of the checks in the file, pinned as they behave so a reorganisation of the file
// can be held against them

TEST_F(EstimatorChecksTest, PreflightInnovationFailureIsReportedOnTheGround)
{
	estimator_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.pre_flt_fail_innov_heading = true;
	publishEstimatorStatus(status);

	runCheck(false);
	EXPECT_TRUE(armingError(health_component_t::local_position_estimate));

	runCheck(true);
	EXPECT_FALSE(armingError(health_component_t::local_position_estimate)) << "the innovations are not judged in flight";

	runCheck(false, true);
	EXPECT_FALSE(armingError(health_component_t::local_position_estimate)) << "nor during calibration";

	// the last innovation in the chain is reported like the first
	status.pre_flt_fail_innov_heading = false;
	status.pre_flt_fail_innov_height = true;
	publishEstimatorStatus(status);
	runCheck(false);
	EXPECT_TRUE(armingError(health_component_t::local_position_estimate));

	status.pre_flt_fail_innov_height = false;
	publishEstimatorStatus(status);
	runCheck(false);
	EXPECT_FALSE(armingError(health_component_t::local_position_estimate));
}

TEST_F(EstimatorChecksTest, MagneticInterferenceIsReportedAsConfigured)
{
	estimator_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.pre_flt_fail_mag_field_disturbed = true;
	publishEstimatorStatus(status);

	setParam("COM_ARM_MAG_STR", 1); // deny arming
	runCheck(false);
	EXPECT_TRUE(armingWarning(health_component_t::local_position_estimate));
	EXPECT_EQ(_report.can_arm_mode_flags, 0u) << "every mode needs the attitude, so none can arm";

	setParam("COM_ARM_MAG_STR", 2); // warn only
	runCheck(false);
	EXPECT_TRUE(armingWarning(health_component_t::local_position_estimate));
	EXPECT_NE(_report.can_arm_mode_flags, 0u) << "the warning alone does not block arming";

	setParam("COM_ARM_MAG_STR", 0);
	runCheck(false);
	EXPECT_FALSE(armingWarning(health_component_t::local_position_estimate));

	setParam("COM_ARM_MAG_STR", 1);
	runCheck(true);
	EXPECT_FALSE(armingWarning(health_component_t::local_position_estimate)) << "only judged on the ground";
}

TEST_F(EstimatorChecksTest, GnssFusionStartAndStopAreReportedInFlight)
{
	publishLocalPosition(true);
	runCheck(true);
	drainEvents();

	publishEstimatorStatus(true, 0);
	runCheck(true);
	EXPECT_EQ(countEvents(kFusionStartedEvent), 1);

	runCheck(true);
	EXPECT_EQ(countEvents(kFusionStartedEvent), 1) << "reported on the change, not every cycle";

	event_s event{};
	publishEstimatorStatus(false, 0);
	runCheck(true);
	ASSERT_EQ(countEvents(kFusionStoppedEvent, &event), 1);
	EXPECT_EQ(event.log_levels & 0x0f, (uint8_t)events::Log::Error) << "an error while the position estimate is still valid";
}

TEST_F(EstimatorChecksTest, GnssFusionStopIsOnlyInformationOncePositionIsLost)
{
	flyOnGnss();

	publishLocalPosition(false);
	runCheck(true);
	ASSERT_TRUE(_failsafe_flags.local_position_invalid);
	drainEvents();

	event_s event{};
	publishEstimatorStatus(false, 0);
	runCheck(true);
	ASSERT_EQ(countEvents(kFusionStoppedEvent, &event), 1);
	EXPECT_EQ(event.log_levels & 0x0f, (uint8_t)events::Log::Info);
}

TEST_F(EstimatorChecksTest, GnssFusionChangesAreNotReportedOnTheGround)
{
	publishEstimatorStatus(true, 0);
	runCheck(false);
	publishEstimatorStatus(false, 0);
	runCheck(false);
	EXPECT_EQ(countEvents(kFusionStartedEvent), 0);
	EXPECT_EQ(countEvents(kFusionStoppedEvent), 0);

	// the state is still followed, so arming does not report a change that happened on the ground
	publishEstimatorStatus(true, 0);
	runCheck(false);
	runCheck(true);
	EXPECT_EQ(countEvents(kFusionStartedEvent), 0);
}

TEST_F(EstimatorChecksTest, SpoofingAndJammingAreReportedOnceUntilTheyClear)
{
	publishEstimatorStatus(true, kSpoofed);
	runCheck(false);
	runCheck(false);
	EXPECT_EQ(countEvents(kSpoofingEvent), 1);

	publishEstimatorStatus(true, 0);
	runCheck(false);
	publishEstimatorStatus(true, kSpoofed);
	runCheck(false);
	EXPECT_EQ(countEvents(kSpoofingEvent), 2) << "reported again once it cleared in between";

	publishEstimatorStatus(true, kJammed);
	runCheck(false);
	runCheck(false);
	EXPECT_EQ(countEvents(kJammingEvent), 1);
}

TEST_F(EstimatorChecksTest, AFailingGnssCheckBlocksArmingAsConfigured)
{
	publishEstimatorStatus(true, 0);
	runCheck(false);
	const uint64_t can_arm_with_good_gnss = _report.can_arm_mode_flags;

	publishEstimatorStatus(true, kFixTooLow);

	setParam("COM_ARM_WO_GPS", 0); // deny arming
	runCheck(false);
	EXPECT_TRUE(armingError(health_component_t::gps));
	EXPECT_EQ(_report.can_arm_mode_flags, 0u);

	setParam("COM_ARM_WO_GPS", 1); // warn only
	runCheck(false);
	EXPECT_FALSE(armingError(health_component_t::gps));
	EXPECT_TRUE(armingWarning(health_component_t::gps));
	EXPECT_EQ(_report.can_arm_mode_flags, can_arm_with_good_gnss);

	setParam("COM_ARM_WO_GPS", 2); // disabled
	runCheck(false);
	EXPECT_FALSE(armingError(health_component_t::gps));
	EXPECT_FALSE(armingWarning(health_component_t::gps));

	setParam("COM_ARM_WO_GPS", 0);
	runCheck(true);
	EXPECT_FALSE(armingError(health_component_t::gps)) << "the quality checks are only judged on the ground";
}

TEST_F(EstimatorChecksTest, GnssIsPresentWhileTheEstimatorFusesIt)
{
	publishEstimatorStatus(true, 0);
	runCheck(false);
	EXPECT_TRUE(isPresent(health_component_t::gps));

	publishEstimatorStatus(false, 0);
	runCheck(false);
	EXPECT_FALSE(isPresent(health_component_t::gps));
}

TEST_F(EstimatorChecksTest, HighSensorBiasBlocksArming)
{
	publishEstimatorStatus(true, 0);

	estimator_sensor_bias_s bias{};
	bias.timestamp = hrt_absolute_time();
	bias.accel_bias_valid = true;
	bias.accel_bias_limit = 0.4f;
	bias.accel_bias[1] = 0.35f; // above three quarters of the limit
	publishSensorBias(bias);

	runCheck(false);
	EXPECT_TRUE(armingError(health_component_t::local_position_estimate));

	runCheck(false, true);
	EXPECT_FALSE(armingError(health_component_t::local_position_estimate)) << "not judged during calibration";

	bias.timestamp = hrt_absolute_time() - 31_s;
	publishSensorBias(bias);
	runCheck(false);
	EXPECT_FALSE(armingError(health_component_t::local_position_estimate)) << "a stale estimate is not judged";

	bias = {};
	bias.timestamp = hrt_absolute_time();
	bias.gyro_bias_valid = true;
	bias.gyro_bias_limit = 0.1f;
	bias.gyro_bias[2] = 0.09f;
	publishSensorBias(bias);
	runCheck(false);
	EXPECT_TRUE(armingError(health_component_t::local_position_estimate));
}

TEST_F(EstimatorChecksTest, CompassFaultBlocksEveryMode)
{
	publishEstimatorStatus(true, 0);

	estimator_status_flags_s flags{};
	flags.timestamp = hrt_absolute_time();
	flags.cs_yaw_align = true;
	flags.cs_mag_fault = true;
	publishStatusFlags(flags);

	runCheck(true);
	EXPECT_TRUE(armingError(health_component_t::local_position_estimate));
	EXPECT_EQ(_report.can_arm_mode_flags, 0u);
}

TEST_F(EstimatorChecksTest, NoHeadingReferenceBlocksArmingOnlyWithAGlobalOrigin)
{
	publishEstimatorStatus(true, 0);

	estimator_status_flags_s flags{};
	flags.timestamp = hrt_absolute_time();
	flags.cs_yaw_align = false;
	publishStatusFlags(flags);

	publishLocalPosition(true, 0.5f, true);
	runCheck(false);
	EXPECT_TRUE(armingError(health_component_t::local_position_estimate));

	runCheck(true);
	EXPECT_FALSE(armingError(health_component_t::local_position_estimate)) << "only judged on the ground";

	publishLocalPosition(true, 0.5f, false);
	runCheck(false);
	EXPECT_FALSE(armingError(health_component_t::local_position_estimate)) << "no heading reference is needed without a global origin";
}

TEST_F(EstimatorChecksTest, MissingGnssHeadingBlocksArmingOnTheGround)
{
	publishHeadingMissing(false);
	runCheck(false);
	EXPECT_FALSE(armingError(health_component_t::gps));

	// every reason the sensors module gives blocks arming the same way, only the message differs
	for (uint8_t state = sensors_status_gnss_s::HEADING_NONE; state <= sensors_status_gnss_s::HEADING_PUBLISHED; state++) {
		publishHeadingState(state);
		publishHeadingMissing(true);
		runCheck(false);
		EXPECT_TRUE(armingError(health_component_t::gps)) << "heading state " << (int)state;
	}

	runCheck(true);
	EXPECT_FALSE(armingError(health_component_t::gps)) << "only judged on the ground";

	setParam("SYS_HAS_GPS", 0);
	runCheck(false);
	EXPECT_FALSE(armingError(health_component_t::gps)) << "not without GNSS";
}

TEST_F(EstimatorChecksTest, MissingGnssHeadingBlocksOnlyTheModesThatNeedAHeading)
{
	// only Position mode needs a local position, Takeoff always needs a heading
	_failsafe_flags.mode_req_local_position = 1u << vehicle_status_s::NAVIGATION_STATE_POSCTL;
	_failsafe_flags.mode_req_global_position = 0;

	publishHeadingMissing(false);
	runCheck(false);
	const uint64_t can_arm = _report.can_arm_mode_flags;
	ASSERT_TRUE(can_arm & (1u << vehicle_status_s::NAVIGATION_STATE_MANUAL));
	ASSERT_TRUE(can_arm & (1u << vehicle_status_s::NAVIGATION_STATE_POSCTL));

	publishHeadingMissing(true);
	runCheck(false);
	EXPECT_EQ(_report.can_arm_mode_flags, can_arm & ~((1u << vehicle_status_s::NAVIGATION_STATE_POSCTL)
			| (1u << vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF)));
}

TEST_F(EstimatorChecksTest, PositionFailureImminentIsWarnedOnceWhileDeadReckoning)
{
	publishEstimatorStatus(true, 0);
	publishLocalPosition(true);

	// a global position that is still valid but whose error is close to COM_POS_FS_EPH
	vehicle_global_position_s gpos{};
	gpos.timestamp = hrt_absolute_time();
	gpos.lat_lon_valid = true;
	gpos.alt_valid = true;
	gpos.eph = 12.f;
	publishGlobalPosition(gpos);

	estimator_status_flags_s flags{};
	flags.timestamp = hrt_absolute_time();
	flags.cs_yaw_align = true;
	flags.cs_inertial_dead_reckoning = true;
	publishStatusFlags(flags);

	runCheck(true);
	ASSERT_FALSE(_failsafe_flags.global_position_invalid);
	EXPECT_EQ(countEvents(kFailureImminentEvent), 1);

	runCheck(true);
	EXPECT_EQ(countEvents(kFailureImminentEvent), 1) << "warned once";

	flags.cs_inertial_dead_reckoning = false;
	publishStatusFlags(flags);
	runCheck(true);
	flags.cs_inertial_dead_reckoning = true;
	publishStatusFlags(flags);
	runCheck(true);
	EXPECT_EQ(countEvents(kFailureImminentEvent), 2) << "warned again after dead reckoning ended in between";
}

TEST_F(EstimatorChecksTest, LowPositionAccuracyIsFlaggedAndReportedInFlight)
{
	setParam("COM_POS_LOW_EPH", 1.f);
	setParam("COM_POS_LOW_ACT", 1);
	publishEstimatorStatus(true, 0);

	publishLocalPosition(true, 2.f);
	runCheck(true);
	ASSERT_FALSE(_failsafe_flags.local_position_invalid);
	EXPECT_TRUE(_failsafe_flags.position_accuracy_low);
	EXPECT_TRUE(armingError(health_component_t::local_position_estimate));

	runCheck(false);
	EXPECT_TRUE(_failsafe_flags.position_accuracy_low);
	EXPECT_FALSE(armingError(health_component_t::local_position_estimate)) << "flagged but not reported on the ground";

	publishLocalPosition(true, 0.5f);
	runCheck(true);
	EXPECT_FALSE(_failsafe_flags.position_accuracy_low);
}

TEST_F(EstimatorChecksTest, AttitudeValidityFollowsTheQuaternion)
{
	vehicle_attitude_s attitude{};
	attitude.timestamp = hrt_absolute_time();
	attitude.q[0] = 1.f;
	publishAttitude(attitude);
	runCheck(true);
	EXPECT_FALSE(_failsafe_flags.attitude_invalid);

	attitude.q[0] = 1.1f;
	publishAttitude(attitude);
	runCheck(true);
	EXPECT_TRUE(_failsafe_flags.attitude_invalid) << "not a unit quaternion";

	attitude.q[0] = 1.f;
	attitude.timestamp = hrt_absolute_time() - 2_s;
	publishAttitude(attitude);
	runCheck(true);
	EXPECT_TRUE(_failsafe_flags.attitude_invalid) << "stale";
}

TEST_F(EstimatorChecksTest, AngularVelocityValidityFollowsTheSamples)
{
	vehicle_angular_velocity_s rates{};
	rates.timestamp = hrt_absolute_time();
	publishAngularVelocity(rates);
	runCheck(true);
	EXPECT_FALSE(_failsafe_flags.angular_velocity_invalid);

	rates.xyz[0] = NAN;
	publishAngularVelocity(rates);
	runCheck(true);
	EXPECT_TRUE(_failsafe_flags.angular_velocity_invalid) << "not finite";

	rates.xyz[0] = 0.f;
	rates.timestamp = hrt_absolute_time() - 2_s;
	publishAngularVelocity(rates);
	runCheck(true);
	EXPECT_TRUE(_failsafe_flags.angular_velocity_invalid) << "stale";
}

TEST_F(EstimatorChecksTest, LocalAltitudeIsInvalidWithoutAValidHeight)
{
	publishLocalPosition(true);
	runCheck(true);
	EXPECT_FALSE(_failsafe_flags.local_altitude_invalid);

	vehicle_local_position_s lpos{};
	lpos.timestamp = hrt_absolute_time();
	lpos.xy_valid = true;
	lpos.v_xy_valid = true;
	lpos.z_valid = false;
	_local_position_pub.publish(lpos);
	runCheck(true);
	EXPECT_TRUE(_failsafe_flags.local_altitude_invalid);
}
