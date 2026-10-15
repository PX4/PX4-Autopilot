/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "Common.hpp"
#include "checks/estimatorCheck.hpp"
#include "checks/gnssRedundancyCheck.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/param.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/estimator_status_flags.h>
#include <uORB/topics/event.h>
#include <uORB/topics/health_report.h>
#include <uORB/topics/sensor_gnss.h>
#include <uORB/topics/sensors_status_gnss.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>

#include <stdint.h>

using namespace time_literals;

// to run: make tests TESTFILTER=HealthAndArmingChecks

/* EVENT
 * @skip-file
 */

class ReporterTest : public ::testing::Test

{
public:

	void SetUp() override
	{
		// ensure topic exists, otherwise we might lose first queued events
		orb_advertise(ORB_ID(event), nullptr);
	}

};


TEST_F(ReporterTest, basic_no_checks)
{
	failsafe_flags_s failsafe_flags{};
	Report reporter{failsafe_flags, 0_s};
	ASSERT_FALSE(reporter.canArm(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION));

	reporter.reset();
	reporter.finalize();
	reporter.report(false);

	ASSERT_TRUE(reporter.canArm(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION));
	ASSERT_EQ((uint8_t)reporter.armingCheckResults().can_arm, 0xff);
	ASSERT_EQ((uint64_t)reporter.armingCheckResults().error, 0);
	ASSERT_EQ((uint64_t)reporter.armingCheckResults().warning, 0);

	ASSERT_EQ((uint64_t)reporter.healthResults().is_present, 0);
	ASSERT_EQ((uint64_t)reporter.healthResults().error, 0);
	ASSERT_EQ((uint64_t)reporter.healthResults().warning, 0);
}

TEST_F(ReporterTest, basic_fail_all_modes)
{
	failsafe_flags_s failsafe_flags{};
	Report reporter{failsafe_flags, 0_s};

	// ensure arming is always denied with a NavModes::All failure
	for (uint8_t nav_state = 0; nav_state < vehicle_status_s::NAVIGATION_STATE_MAX; ++nav_state) {
		reporter.reset();
		reporter.armingCheckFailure(NavModes::All, health_component_t::remote_control,
					    events::ID("arming_test_basic_fail_all_modes_fail1"), events::Log::Info, "");
		reporter.finalize();
		reporter.report(false);

		ASSERT_FALSE(reporter.canArm(nav_state));
		ASSERT_TRUE(reporter.canRun(nav_state));
	}
}

TEST_F(ReporterTest, arming_checks_mode_category)
{
	failsafe_flags_s failsafe_flags{};
	Report reporter{failsafe_flags, 0_s};

	// arming must still be possible for non-relevant failures
	reporter.reset();
	reporter.armingCheckFailure(NavModes::PositionControl | NavModes::Stabilized, health_component_t::remote_control,
				    events::ID("arming_test_arming_checks_mode_category_fail1"), events::Log::Warning, "");
	reporter.clearCanRunBits(NavModes::PositionControl | NavModes::Stabilized);
	reporter.healthFailure(NavModes::PositionControl, health_component_t::local_position_estimate,
			       events::ID("arming_test_arming_checks_mode_category_fail2"), events::Log::Info, "");
	reporter.setIsPresent(health_component_t::battery);
	reporter.finalize();
	reporter.report(false);

	ASSERT_TRUE(reporter.canArm(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION));
	ASSERT_TRUE(reporter.canRun(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION));
	ASSERT_FALSE(reporter.canRun(vehicle_status_s::NAVIGATION_STATE_POSCTL));

	ASSERT_EQ((uint8_t)reporter.armingCheckResults().can_arm, (uint8_t)~(NavModes::PositionControl | NavModes::Stabilized));
	ASSERT_EQ((uint64_t)reporter.armingCheckResults().error, 0);
	ASSERT_EQ(reporter.armingCheckResults().warning, events::px4::enums::health_component_t::remote_control);

	ASSERT_EQ(reporter.healthResults().is_present, events::px4::enums::health_component_t::battery);
	ASSERT_EQ((uint64_t)reporter.healthResults().error, 0);
	ASSERT_EQ((uint64_t)reporter.healthResults().warning, 0);
}

TEST_F(ReporterTest, arming_checks_mode_category2)
{
	failsafe_flags_s failsafe_flags{};
	Report reporter{failsafe_flags, 0_s};

	// A matching mode category must deny arming
	reporter.reset();
	reporter.healthFailure(NavModes::Mission, health_component_t::remote_control,
			       events::ID("arming_test_arming_checks_mode_category2_fail1"), events::Log::Warning, "");
	reporter.finalize();
	reporter.report(false);

	ASSERT_FALSE(reporter.canArm(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION));

	ASSERT_EQ((uint8_t)reporter.armingCheckResults().can_arm, (uint8_t)~(NavModes::Mission));
	ASSERT_EQ((uint64_t)reporter.armingCheckResults().error, 0);
	ASSERT_EQ((uint64_t)reporter.armingCheckResults().warning, 0);

	ASSERT_EQ((uint64_t)reporter.healthResults().is_present, 0);
	ASSERT_EQ((uint64_t)reporter.healthResults().error, 0);
	ASSERT_EQ(reporter.healthResults().warning, events::px4::enums::health_component_t::remote_control);
}

TEST_F(ReporterTest, reporting)
{
	failsafe_flags_s failsafe_flags{};
	Report reporter{failsafe_flags, 0_s};

	uORB::Subscription event_sub{ORB_ID(event)};
	event_sub.subscribe();
	event_s event;

	while (event_sub.update(&event)); // clear all updates

	for (int j = 0; j < 2; ++j) { // test with and without additional report arguments
		const bool with_arg = j == 0;

		for (int i = 0; i < 3; ++i) { // repeat same report: we expect reporting only the first time
			reporter.reset();

			if (with_arg) {
				reporter.armingCheckFailure<uint16_t>(NavModes::All, health_component_t::remote_control,
								      events::ID("arming_test_reporting_fail1"), events::Log::Warning, "", 4938);

			} else {
				reporter.armingCheckFailure(NavModes::PositionControl, health_component_t::remote_control,
							    events::ID("arming_test_reporting_fail2"), events::Log::Warning, "");
			}

			reporter.finalize();
			reporter.report(false);
			ASSERT_FALSE(reporter.canArm(vehicle_status_s::NAVIGATION_STATE_POSCTL));

			if (i == 0) {
				ASSERT_TRUE(event_sub.update(&event));
				ASSERT_EQ(event.id, events::ID("commander_arming_check_summary"));
				ASSERT_TRUE(event_sub.update(&event));

				if (with_arg) {
					ASSERT_EQ(event.id, events::ID("arming_test_reporting_fail1"));

				} else {
					ASSERT_EQ(event.id, events::ID("arming_test_reporting_fail2"));
				}

				ASSERT_TRUE(event_sub.update(&event));
				ASSERT_EQ(event.id, events::ID("commander_health_summary"));

			} else {
				ASSERT_FALSE(event_sub.updated());
			}
		}
	}

	// now the same for health
	for (int j = 0; j < 2; ++j) {
		const bool with_arg = j == 0;

		for (int i = 0; i < 3; ++i) {
			reporter.reset();

			if (with_arg) {
				reporter.healthFailure<uint16_t>(NavModes::All, health_component_t::remote_control,
								 events::ID("arming_test_reporting_fail3"), events::Log::Warning, "", 4938);

			} else {
				reporter.healthFailure(NavModes::PositionControl, health_component_t::remote_control,
						       events::ID("arming_test_reporting_fail4"), events::Log::Warning, "");
			}

			reporter.finalize();
			reporter.report(false);
			ASSERT_FALSE(reporter.canArm(vehicle_status_s::NAVIGATION_STATE_POSCTL));

			if (i == 0) {
				ASSERT_TRUE(event_sub.update(&event));
				ASSERT_EQ(event.id, events::ID("commander_arming_check_summary"));
				ASSERT_TRUE(event_sub.update(&event));

				if (with_arg) {
					ASSERT_EQ(event.id, events::ID("arming_test_reporting_fail3"));

				} else {
					ASSERT_EQ(event.id, events::ID("arming_test_reporting_fail4"));
				}

				ASSERT_TRUE(event_sub.update(&event));
				ASSERT_EQ(event.id, events::ID("commander_health_summary"));

			} else {
				ASSERT_FALSE(event_sub.updated());
			}
		}
	}

}

TEST_F(ReporterTest, reporting_multiple)
{
	failsafe_flags_s failsafe_flags{};
	Report reporter{failsafe_flags, 0_s};

	uORB::Subscription event_sub{ORB_ID(event)};
	event_sub.subscribe();
	event_s event;

	while (event_sub.update(&event)); // clear all updates

	for (int i = 0; i < 3; ++i) {
		reporter.reset();
		reporter.armingCheckFailure<uint16_t>(NavModes::All, health_component_t::remote_control,
						      events::ID("arming_test_reporting_multiple_fail1"), events::Log::Warning, "", 4938);
		reporter.armingCheckFailure<float>(NavModes::All, health_component_t::remote_control,
						   events::ID("arming_test_reporting_multiple_fail2"), events::Log::Warning, "", 123.f);
		reporter.armingCheckFailure<uint8_t>(NavModes::All, health_component_t::remote_control,
						     events::ID("arming_test_reporting_multiple_fail3"), events::Log::Warning, "", 55);
		reporter.finalize();
		reporter.report(false);
		ASSERT_FALSE(reporter.canArm(vehicle_status_s::NAVIGATION_STATE_POSCTL));

		if (i == 0) {
			ASSERT_TRUE(event_sub.update(&event));
			ASSERT_EQ(event.id, events::ID("commander_arming_check_summary"));
			ASSERT_TRUE(event_sub.update(&event));
			ASSERT_EQ(event.id, events::ID("arming_test_reporting_multiple_fail1"));
			ASSERT_TRUE(event_sub.update(&event));
			ASSERT_EQ(event.id, events::ID("arming_test_reporting_multiple_fail2"));
			ASSERT_TRUE(event_sub.update(&event));
			ASSERT_EQ(event.id, events::ID("arming_test_reporting_multiple_fail3"));
			ASSERT_TRUE(event_sub.update(&event));
			ASSERT_EQ(event.id, events::ID("commander_health_summary"));

		} else {
			ASSERT_FALSE(event_sub.updated());
		}
	}
}

class EstimatorChecksTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		param_control_autosave(false);

		setParam("SYS_HAS_GPS", 1);
		setParam("SENS_IMU_MODE", 1);

		// back to their defaults, the cases below change them
		param_reset(param_find("COM_ARM_WO_GPS"));
		param_reset(param_find("COM_POS_LOW_EPH"));
		param_reset(param_find("COM_POS_LOW_ACT"));

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
		_status_flags_pub.publish(flags);
		estimator_sensor_bias_s bias{};
		bias.timestamp = hrt_absolute_time();
		publishSensorBias(bias);
		vehicle_global_position_s gpos{};
		gpos.timestamp = hrt_absolute_time();
		_global_position_pub.publish(gpos);
		vehicle_attitude_s attitude{};
		attitude.timestamp = hrt_absolute_time();
		attitude.q[0] = 1.f;
		publishAttitude(attitude);
		vehicle_angular_velocity_s rates{};
		rates.timestamp = hrt_absolute_time();
		publishAngularVelocity(rates);
	}

	void setParam(const char *name, int32_t value) { param_set(param_find(name), &value); }
	void setParam(const char *name, float value) { param_set(param_find(name), &value); }

	void publishSensorBias(const estimator_sensor_bias_s &bias) { _sensor_bias_pub.publish(bias); }
	void publishAttitude(const vehicle_attitude_s &attitude) { _attitude_pub.publish(attitude); }
	void publishAngularVelocity(const vehicle_angular_velocity_s &rates) { _angular_velocity_pub.publish(rates); }

	// what the estimator reports: whether it fuses GNSS position, and which receiver checks fail
	void publishEstimatorStatus(bool gnss_fused, uint16_t gps_check_fail_flags)
	{
		estimator_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.control_mode_flags = gnss_fused ? (1ULL << estimator_status_s::CS_GNSS_POS) : 0;
		status.gps_check_fail_flags = gps_check_fail_flags;
		_estimator_status_pub.publish(status);
	}

	void publishLocalPosition(bool valid, float eph = 0.5f)
	{
		vehicle_local_position_s lpos{};
		lpos.timestamp = hrt_absolute_time();
		lpos.timestamp_sample = lpos.timestamp;
		lpos.xy_valid = valid;
		lpos.v_xy_valid = valid;
		lpos.z_valid = true;
		lpos.v_z_valid = true;
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

	static constexpr uint16_t kFixTooLow = 1 << estimator_status_s::GPS_CHECK_FAIL_GPS_FIX;

	uORB::PublicationMulti<estimator_status_s> _estimator_status_pub{ORB_ID(estimator_status)};
	uORB::PublicationMulti<estimator_status_flags_s> _status_flags_pub{ORB_ID(estimator_status_flags)};
	uORB::PublicationMulti<estimator_sensor_bias_s> _sensor_bias_pub{ORB_ID(estimator_sensor_bias)};
	uORB::Publication<vehicle_local_position_s> _local_position_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<vehicle_global_position_s> _global_position_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<vehicle_attitude_s> _attitude_pub{ORB_ID(vehicle_attitude)};
	uORB::Publication<vehicle_angular_velocity_s> _angular_velocity_pub{ORB_ID(vehicle_angular_velocity)};

	health_report_s _report{};
	failsafe_flags_s _failsafe_flags{};
	EstimatorChecks _check;
};

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

// Base position (PX4 SITL default home).
static constexpr double BASE_LAT = 47.397742;
static constexpr double BASE_LON = 8.545594;

class GnssRedundancyChecksTest : public ::testing::Test
{
public:
	void SetUp() override
	{
		param_control_autosave(false);

		// Reset params that tests modify so state doesn't leak between tests.
		param_reset(param_find("SYS_HAS_NUM_GNSS"));
		param_reset(param_find("COM_GNSSLOSS_ACT"));

		// Claim uORB instances 0 and 1 before the check subscribes on first copy().
		sensor_gnss_s empty{};
		_gnss0_pub.publish(empty);
		_gnss1_pub.publish(empty);
	}

	sensor_gnss_s makeGnss(double lat, double lon, float eph = 0.02f, uint8_t fix_type = 6)
	{
		sensor_gnss_s gnss{};
		gnss.timestamp     = hrt_absolute_time();
		gnss.device_id     = 1;
		gnss.latitude      = lat;
		gnss.longitude     = lon;
		gnss.eph           = eph;
		gnss.fix_type      = fix_type;
		return gnss;
	}

	// Publish a receiver and the sensors module's status for it, healthy with a 3D fix. Instances 0 and 1 get device IDs
	// 1 and 2.
	void publishGnss(int instance, sensor_gnss_s gnss)
	{
		gnss.device_id = instance + 1;
		(instance == 0 ? _gnss0_pub : _gnss1_pub).publish(gnss);

		_status.device_ids[instance] = gnss.device_id;
		_status.healthy[instance] = (gnss.fix_type >= 3);
		_status.timestamp = hrt_absolute_time();
		_status_pub.publish(_status);
	}

	// Run the check and store results in _failsafe_flags, _health_warning_gps and _can_arm_all_modes.
	void runCheck(bool armed = false)
	{
		vehicle_status_s status{};

		if (armed) { status.arming_state = vehicle_status_s::ARMING_STATE_ARMED; }

		_check.updateParams();
		Context context{status};
		_failsafe_flags = {};
		Report reporter{_failsafe_flags, 0};
		_check.checkAndReport(context, reporter);
		// Capture any GPS health issue regardless of log level (Warning or Error).
		_health_warning_gps = (reporter.healthResults().warning | reporter.healthResults().error) & health_component_t::gps;
		_can_arm_all_modes = reporter.armingCheckResults().can_arm == NavModes::All;
	}

	void setGnssLossAct(int32_t com_gnssloss_act)
	{
		param_set(param_find("COM_GNSSLOSS_ACT"), &com_gnssloss_act);
	}

	uORB::PublicationMulti<sensor_gnss_s> _gnss0_pub{ORB_ID(sensor_gnss)};
	uORB::PublicationMulti<sensor_gnss_s> _gnss1_pub{ORB_ID(sensor_gnss)};
	uORB::Publication<sensors_status_gnss_s> _status_pub{ORB_ID(sensors_status_gnss)};
	sensors_status_gnss_s _status{};
	failsafe_flags_s  _failsafe_flags{};
	bool              _health_warning_gps{false};
	bool              _can_arm_all_modes{false};
	GnssRedundancyChecks _check;
};

// SYS_HAS_NUM_GNSS = 2, COM_GNSSLOSS_ACT = Return, only one receiver fixed → gnss_lost and arming blocked.
TEST_F(GnssRedundancyChecksTest, BelowRequiredSetsGnssLost)
{
	int required = 2;   param_set(param_find("SYS_HAS_NUM_GNSS"), &required);
	setGnssLossAct(1); // Return

	publishGnss(0, makeGnss(BASE_LAT, BASE_LON));
	runCheck();
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
	EXPECT_TRUE(_health_warning_gps);
	EXPECT_FALSE(_can_arm_all_modes);
}

// COM_GNSSLOSS_ACT = Warning reports the issue but never blocks arming.
TEST_F(GnssRedundancyChecksTest, BelowRequiredWarningDoesNotBlockArming)
{
	int required = 2;   param_set(param_find("SYS_HAS_NUM_GNSS"), &required);
	setGnssLossAct(0); // Warning

	publishGnss(0, makeGnss(BASE_LAT, BASE_LON));
	runCheck();
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
	EXPECT_TRUE(_health_warning_gps);
	EXPECT_TRUE(_can_arm_all_modes);
}

// A receiver counts only while the status names it and says it passes its checks.
TEST_F(GnssRedundancyChecksTest, StatusOfAnotherReceiverDoesNotCount)
{
	int required = 1;
	param_set(param_find("SYS_HAS_NUM_GNSS"), &required);

	publishGnss(0, makeGnss(BASE_LAT, BASE_LON));
	runCheck();
	EXPECT_FALSE(_failsafe_flags.gnss_lost);

	_status.device_ids[0] = 99;
	_status_pub.publish(_status);
	runCheck();
	EXPECT_TRUE(_failsafe_flags.gnss_lost);
}
