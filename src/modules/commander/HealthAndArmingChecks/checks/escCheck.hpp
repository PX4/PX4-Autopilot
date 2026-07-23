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

#include <lib/hysteresis/hysteresis.h>
#include <lib/motor_failure_detector/MotorFailureDetector.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/failure_detector_status.h>

class EscChecks : public HealthAndArmingCheckBase
{
public:
	EscChecks() = default;
	~EscChecks() = default;

	void checkAndReport(const Context &context, Report &reporter) override;

	uint16_t getMotorFailureMask() const { return _motor_failure_mask; }
	bool getEscArmStatus() const { return _esc_arm_hysteresis.get_state(); }

private:
	uint16_t checkEscOnline(const Context &context, Report &reporter, const esc_status_s &esc_status, hrt_abstime now);
	uint16_t checkEscStatus(const Context &context, Report &reporter, const esc_status_s &esc_status);
	uint16_t checkMotorStatus(const Context &context, Report &reporter, const esc_status_s &esc_status);
	void updateEscsStatus(const Context &context, Report &reporter, const esc_status_s &esc_status, hrt_abstime now);
	void checkEscTemperature(Report &reporter, const esc_status_s &esc_status);

	void configureMotorFailureDetector();

	static constexpr hrt_abstime ESC_TIMEOUT_US = 400_ms;
	static constexpr float MOTOR_FAILURE_LPF_TAU_S = 0.2f; ///< residual low-pass time constant [s]

	uORB::Subscription _esc_status_sub{ORB_ID(esc_status)};
	uORB::Subscription _actuator_motors_sub{ORB_ID(actuator_motors)};

	const hrt_abstime _start_time{hrt_absolute_time()};

	uint16_t _motor_failure_mask = 0;
	bool _esc_has_reported_current[esc_status_s::CONNECTED_ESC_MAX] {}; // true if ESC reported non-zero current before (some never report any)
	int8_t _motor_failure_direction[MotorFailureDetector::kMaxMotors] {}; ///< under/over latched at first trip: 0=none, <0=under, >0=over
	bool _esc_over_temp_warned{false};
	systemlib::Hysteresis _esc_arm_hysteresis;
	MotorFailureDetector _motor_failure_detector;
	bool _motor_failure_configured{false};   ///< guards a one-time lazy configure() on the first armed cycle

	DEFINE_PARAMETERS_CUSTOM_PARENT(HealthAndArmingCheckBase,
					(ParamBool<px4::params::COM_ARM_CHK_ESCS>) _param_com_arm_chk_escs,
					(ParamBool<px4::params::FD_ACT_EN>) _param_fd_act_en,
					(ParamFloat<px4::params::MOTFAIL_C2T>) _param_motfail_c2t,
					(ParamFloat<px4::params::MOTFAIL_IDLE>) _param_motfail_idle,
					(ParamFloat<px4::params::MOTFAIL_OFF>) _param_motfail_off,
					(ParamFloat<px4::params::MOTFAIL_TIME>) _param_motfail_time,
					(ParamFloat<px4::params::ESC_TEMP_WARN_TH>) _param_esc_temp_warn_th);
};
