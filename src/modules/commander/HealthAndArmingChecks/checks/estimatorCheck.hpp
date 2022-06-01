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
#include <uORB/topics/estimator_sensor_bias.h>


class EstimatorChecks : public HealthAndArmingCheckBase
{
public:
	EstimatorChecks() = default;
	~EstimatorChecks() = default;

	void checkAndReport(const Context &context, Report &reporter) override;

private:
	void checkEstimatorStatus(const Context &context, Report &reporter, const estimator_status_s &estimator_status,
				  NavModes required_groups);
	void checkSensorBias(const Context &context, Report &reporter, NavModes required_groups);

	uORB::Subscription _estimator_selector_status_sub{ORB_ID(estimator_selector_status)};
	uORB::Subscription _estimator_status_sub{ORB_ID(estimator_status)};
	uORB::Subscription _estimator_sensor_bias_sub{ORB_ID(estimator_sensor_bias)};

	DEFINE_PARAMETERS_CUSTOM_PARENT(HealthAndArmingCheckBase,
					(ParamInt<px4::params::SYS_MC_EST_GROUP>) _param_sys_mc_est_group,
					(ParamInt<px4::params::SENS_IMU_MODE>) _param_sens_imu_mode,
					(ParamInt<px4::params::COM_ARM_MAG_STR>) _param_com_arm_mag_str,
					(ParamFloat<px4::params::COM_ARM_EKF_HGT>) _param_com_arm_ekf_hgt,
					(ParamFloat<px4::params::COM_ARM_EKF_VEL>) _param_com_arm_ekf_vel,
					(ParamFloat<px4::params::COM_ARM_EKF_POS>) _param_com_arm_ekf_pos,
					(ParamFloat<px4::params::COM_ARM_EKF_YAW>) _param_com_arm_ekf_yaw,
					(ParamBool<px4::params::COM_ARM_WO_GPS>) _param_com_arm_wo_gps,
					(ParamBool<px4::params::SYS_HAS_GPS>) _param_sys_has_gps
				       )
};
