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

#pragma once

#include "Common.hpp"
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/estimator_states.h>
#include <uORB/Subscription.hpp>

class Ekf2Checks : public HealthAndArmingCheckBase
{
public:
	Ekf2Checks() = default;
	~Ekf2Checks() = default;

	void checkAndReport(const Context &context, Report &reporter) override;
private:
	bool ekf2Enabled(const Context &context) const;

	void stateChecks(const Context &context, Report &reporter);

	uORB::Subscription _status_sub{ORB_ID(estimator_status)};
	uORB::Subscription _states_sub{ORB_ID(estimator_states)};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::COM_ARM_WO_GPS>) _param_arm_without_gps,
		(ParamBool<px4::params::COM_ARM_MAG_STR>) _param_mag_strength_check_enabled,
		(ParamInt<px4::params::SYS_MC_EST_GROUP>) _param_sys_mc_est_group,
		(ParamFloat<px4::params::COM_ARM_EKF_HGT>) _param_hgt_test_ratio_limit,
		(ParamFloat<px4::params::COM_ARM_EKF_VEL>) _param_vel_test_ratio_limit,
		(ParamFloat<px4::params::COM_ARM_EKF_POS>) _param_pos_test_ratio_limit,
		(ParamFloat<px4::params::COM_ARM_EKF_YAW>) _param_mag_test_ratio_limit,

		(ParamFloat<px4::params::COM_ARM_EKF_AB>) _param_ekf_ab_test_limit,
		(ParamFloat<px4::params::COM_ARM_EKF_GB>) _param_ekf_gb_test_limit
	)

};
