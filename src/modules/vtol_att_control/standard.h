/****************************************************************************
 *
 *   Copyright (c) 2015-2022 PX4 Development Team. All rights reserved.
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
* @file standard.h
* VTOL with fixed multirotor motor configurations (such as quad) and a pusher
* (or puller aka tractor) motor for forward flight.
*
* @author Simon Wilks 		<simon@uaventure.com>
* @author Roman Bapst 		<bapstroman@gmail.com>
* @author Andreas Antener	<andreas@uaventure.com>
* @author Sander Smeets 	<sander@droneslab.com>
*
*/

#ifndef STANDARD_H
#define STANDARD_H
#include "vtol_type.h"

#include <lib/hysteresis/hysteresis.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_acceleration.h>

class Standard : public VtolType
{

public:

	Standard(VtolAttitudeControl *_att_controller);
	~Standard() override = default;

	void update_vtol_state() override;
	void update_transition_state() override;
	void update_fw_state() override;
	void update_mc_state() override;
	void fill_actuator_outputs() override;
	void waiting_on_tecs() override;
	void blendThrottleAfterFrontTransition(float scale) override;
	void blendPitchAfterFrontTransition(float scale) override;

private:

	enum class vtol_mode {
		MC_MODE = 0,
		TRANSITION_TO_FW,
		TRANSITION_TO_MC,
		FW_MODE
	};

	vtol_mode _vtol_mode{vtol_mode::MC_MODE};			/**< vtol flight mode, defined by enum vtol_mode */

	float _pusher_throttle{0.0f};
	float _airspeed_trans_blend_margin{0.0f};
	hrt_abstime _last_time_pusher_transition_update{0};
	float _front_transition_pitch{NAN};

	systemlib::Hysteresis _pusher_failure_hysteresis{false};
	bool _pusher_failed{false}; ///< latched until reboot or until the detection is disabled

	// front transition (ft) check: forward acceleration fitted to the pusher thrust added since the start
	uORB::Subscription _vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};

	struct FtPusherCheck {
		hrt_abstime last_sample{0};
		float transition_throttle{0.f}; ///< when the front transition started
		float start_throttle{0.f}; ///< pusher assist throttle when the front transition started
		float weighted_speed_gain{0.f}; ///< [m/s] forward acceleration integrated, weighted by the added thrust
		float weighted_time{0.f}; ///< [s] time integrated, weighted by the added thrust squared
		bool done{true}; ///< checked in this front transition, or not needed
	} _ft_check{};

	bool _ft_pusher_failed{false}; ///< latched until reboot or until VT_EF_FT_ACC is set to 0

	void parameters_update() override;

	bool isPusherFailureDetectionEnabled() const { return _param_vt_ef_time.get() > FLT_EPSILON; }
	bool isFtPusherCheckEnabled() const { return _param_vt_ef_ft_acc.get() > FLT_EPSILON; }

	/** Pusher assist above VT_EF_THROT, but the forward velocity stays below its setpoint */
	bool isPusherFailureDetected();

	/** Forward acceleration, fitted to the added pusher thrust, below VT_EF_FT_ACC at the front transition throttle */
	bool isPusherFailureDetectedInFrontTransition();

	DEFINE_PARAMETERS_CUSTOM_PARENT(VtolType,
					(ParamFloat<px4::params::VT_PSHER_SLEW>) _param_vt_psher_slew,
					(ParamFloat<px4::params::VT_B_TRANS_RAMP>) _param_vt_b_trans_ramp,
					(ParamFloat<px4::params::FW_PSP_OFF>) _param_fw_psp_off,
					(ParamFloat<px4::params::VT_EF_TIME>) _param_vt_ef_time,
					(ParamFloat<px4::params::VT_EF_THROT>) _param_vt_ef_throt,
					(ParamFloat<px4::params::VT_EF_VEL_ERR>) _param_vt_ef_vel_err,
					(ParamFloat<px4::params::VT_EF_FT_ACC>) _param_vt_ef_ft_acc
				       )
};
#endif
