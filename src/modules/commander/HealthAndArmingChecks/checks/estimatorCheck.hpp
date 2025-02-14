/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#pragma once

#include "../Common.hpp"

#include <uORB/Subscription.hpp>
#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/estimator_status_flags.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/sensor_gps.h>

#include <lib/hysteresis/hysteresis.h>

using namespace time_literals;

class EstimatorChecks : public HealthAndArmingCheckBase
{
public:
	EstimatorChecks();
	~EstimatorChecks() = default;

	void checkAndReport(const Context &context, Report &reporter) override;

private:
	enum class GnssArmingCheck : uint8_t {
		DenyArming = 0,
		WarningOnly = 1,
		Disabled = 2
	};

	void checkEstimatorStatus(const Context &context, Report &reporter, const estimator_status_s &estimator_status,
				  NavModes required_groups);
	void checkSensorBias(const Context &context, Report &reporter, NavModes required_groups);
	void checkEstimatorStatusFlags(const Context &context, Report &reporter, const estimator_status_s &estimator_status,
				       const vehicle_local_position_s &lpos);
	void checkGps(const Context &context, Report &reporter, const sensor_gps_s &vehicle_gps_position) const;
	void lowPositionAccuracy(const Context &context, Report &reporter, const vehicle_local_position_s &lpos) const;

	void setModeRequirementFlags(const Context &context, bool pre_flt_fail_innov_heading,
				     bool pre_flt_fail_innov_vel_horiz, bool pre_flt_fail_innov_pos_horiz,
				     const vehicle_local_position_s &lpos, const sensor_gps_s &vehicle_gps_position,
				     failsafe_flags_s &failsafe_flags, Report &reporter);

	bool checkPosVelValidity(const hrt_abstime &now, const bool data_valid, const float data_accuracy,
				 const float required_accuracy,
				 const hrt_abstime &data_timestamp_us, hrt_abstime &last_fail_time_us,
				 const bool was_valid) const;


	uORB::Subscription _estimator_selector_status_sub{ORB_ID(estimator_selector_status)};
	uORB::Subscription _estimator_status_sub{ORB_ID(estimator_status)};
	uORB::Subscription _estimator_status_flags_sub{ORB_ID(estimator_status_flags)};
	uORB::Subscription _estimator_sensor_bias_sub{ORB_ID(estimator_sensor_bias)};

	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};

	hrt_abstime	_last_gpos_fail_time_us{0};	///< Last time that the global position validity recovery check failed (usec)
	hrt_abstime	_last_lpos_fail_time_us{0};	///< Last time that the local position validity recovery check failed (usec)
	hrt_abstime	_last_lpos_relaxed_fail_time_us{0};	///< Last time that the relaxed local position validity recovery check failed (usec)
	hrt_abstime	_last_lvel_fail_time_us{0};	///< Last time that the local velocity validity recovery check failed (usec)

	bool _gps_was_fused{false};
	bool _gnss_spoofed{false};

	bool _nav_failure_imminent_warned{false};

	DEFINE_PARAMETERS_CUSTOM_PARENT(HealthAndArmingCheckBase,
					(ParamInt<px4::params::SENS_IMU_MODE>) _param_sens_imu_mode,
					(ParamInt<px4::params::COM_ARM_MAG_STR>) _param_com_arm_mag_str,
					(ParamInt<px4::params::COM_ARM_WO_GPS>) _param_com_arm_wo_gps,
					(ParamBool<px4::params::SYS_HAS_GPS>) _param_sys_has_gps,
					(ParamFloat<px4::params::COM_POS_FS_EPH>) _param_com_pos_fs_eph,
					(ParamFloat<px4::params::COM_VEL_FS_EVH>) _param_com_vel_fs_evh,
					(ParamFloat<px4::params::COM_POS_LOW_EPH>) _param_com_low_eph,
					(ParamInt<px4::params::COM_POS_LOW_ACT>) _param_com_pos_low_act
				       )
};
