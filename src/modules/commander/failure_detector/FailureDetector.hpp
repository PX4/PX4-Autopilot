
/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
* @file FailureDetector.hpp
* Base class for failure detection logic based on vehicle states
* for failsafe triggering.
*
* @author Mathieu Bresciani 	<brescianimathieu@gmail.com>
*
*/

#pragma once

#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <px4_platform_common/module_params.h>
#include <hysteresis/hysteresis.h>

// subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_imu_status.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/pwm_input.h>

union failure_detector_status_u {
	struct {
		uint16_t roll : 1;
		uint16_t pitch : 1;
		uint16_t alt : 1;
		uint16_t ext : 1;
		uint16_t arm_escs : 1;
		uint16_t high_wind : 1;
		uint16_t battery : 1;
		uint16_t imbalanced_prop : 1;
	} flags;
	uint16_t value;
};

using uORB::SubscriptionData;

class FailureDetector : public ModuleParams
{
public:
	FailureDetector(ModuleParams *parent);

	bool update(const vehicle_status_s &vehicle_status, const vehicle_control_mode_s &vehicle_control_mode);
	const failure_detector_status_u &getStatus() const { return _status; }
	const decltype(failure_detector_status_u::flags) &getStatusFlags() const { return _status.flags; }
	float getImbalancedPropMetric() const { return _imbalanced_prop_lpf.getState(); }

private:
	void updateAttitudeStatus();
	void updateExternalAtsStatus();
	void updateEscsStatus(const vehicle_status_s &vehicle_status);
	void updateImbalancedPropStatus();

	failure_detector_status_u _status{};

	systemlib::Hysteresis _roll_failure_hysteresis{false};
	systemlib::Hysteresis _pitch_failure_hysteresis{false};
	systemlib::Hysteresis _ext_ats_failure_hysteresis{false};
	systemlib::Hysteresis _esc_failure_hysteresis{false};

	static constexpr float _imbalanced_prop_lpf_time_constant{5.f};
	AlphaFilter<float> _imbalanced_prop_lpf{};
	uint32_t _selected_accel_device_id{0};
	hrt_abstime _imu_status_timestamp_prev{0};

	uORB::Subscription _vehicule_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _esc_status_sub{ORB_ID(esc_status)};
	uORB::Subscription _pwm_input_sub{ORB_ID(pwm_input)};
	uORB::Subscription _sensor_selection_sub{ORB_ID(sensor_selection)};
	uORB::Subscription _vehicle_imu_status_sub{ORB_ID(vehicle_imu_status)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::FD_FAIL_P>) _param_fd_fail_p,
		(ParamInt<px4::params::FD_FAIL_R>) _param_fd_fail_r,
		(ParamFloat<px4::params::FD_FAIL_R_TTRI>) _param_fd_fail_r_ttri,
		(ParamFloat<px4::params::FD_FAIL_P_TTRI>) _param_fd_fail_p_ttri,
		(ParamBool<px4::params::FD_EXT_ATS_EN>) _param_fd_ext_ats_en,
		(ParamInt<px4::params::FD_EXT_ATS_TRIG>) _param_fd_ext_ats_trig,
		(ParamInt<px4::params::FD_ESCS_EN>) _param_escs_en,
		(ParamInt<px4::params::FD_IMB_PROP_THR>) _param_fd_imb_prop_thr
	)
};
