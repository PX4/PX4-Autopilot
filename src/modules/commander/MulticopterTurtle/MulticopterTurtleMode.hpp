/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file MulticopterTurtleMode.hpp
 *
 * Changes to manage turtle mode for a multicopter (flip recovery mode).
 *
 * @author Michał Barciś <mbarcis@mbarcis.net>
 */

#pragma once

#include <px4_platform_common/module_params.h>
#include <px4_platform_common/time.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/vehicle_status.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include "TurtleUtil.hpp"


using namespace std;

class MulticopterTurtleMode : public ModuleParams
{
public:


	explicit MulticopterTurtleMode(ModuleParams *parent);
	~MulticopterTurtleMode() override = default;

	/**
	 * @return false if feature disabled
	 */
	bool isTurtleModeEnabled() const
	{
		return _turtle_mode_state == MulticopterTurtleUtil::TurtleModeState::ACTIVE_TURTLE;
	}

	/**
	 * Check if in ACTIVE_TURTLE state (should block regular arming)
	 * @return true if in ACTIVE_TURTLE state and feature is enabled
	 */
	bool isInTurtleMode() const
	{
		return _param_com_turtle_en.get() && _turtle_mode_state == MulticopterTurtleUtil::TurtleModeState::ACTIVE_TURTLE;
	}

	/**
	 * Get the desired navigation state for turtle mode
	 * @return NAVIGATION_STATE_TURTLE if active, NAVIGATION_STATE_STAB if optional/disabled, NAVIGATION_STATE_MAX if no change needed
	 */
	uint8_t getDesiredNavState() const;

	/**
	 * Check if turtle mode should arm the vehicle (only on transition to ACTIVE)
	 * @return true if turtle mode just became active and should arm
	 */
	bool shouldArm() const;

	/**
	 * Check if turtle mode should disarm the vehicle (only on transition from ACTIVE)
	 * @return true if turtle mode just left active state and should disarm
	 */
	bool shouldDisarm() const;

	/**
	 * Clear the disarm flag after disarming has been requested
	 */
	void clearDisarmFlag();

	/**
	 * Mark that nav_state change has been requested (to prevent repeated changes)
	 */
	void markNavStateChangeRequested();

	/**
	 * Main update of the state
	 * @param armed true if vehicle is armed
	 */
	void update(const bool armed);

	/**
	 * Publish motor commands with custom throttle values for each motor
	 * @param throttle array of 12 throttle values in range [-1, 1] where 1 = max forward, -1 = max reverse, NAN = disabled
	 */

private:
	void publishMotorCommands(const float throttle[12]);
	float getAuxChannel() const;
	float getAuxChannelValue();
	void updateDshot3dParameter(bool enable, bool armed);
	void setState(MulticopterTurtleUtil::TurtleModeState new_state);
	void getMotorData(MulticopterTurtleUtil::Motor_data motor_data[]);

	float thr_factor();
	float _thr_factor;

	MulticopterTurtleUtil::TurtleModeState _turtle_mode_state{MulticopterTurtleUtil::TurtleModeState::FLYING_DISABLED};
	MulticopterTurtleUtil::TurtleModeESCInternalState _turtle_mode_esc_internal_state{MulticopterTurtleUtil::TurtleModeESCInternalState::TURTLE_MODE_ESC_INTERNAL_STATE_3D_OFF};
	MulticopterTurtleUtil::TurtleModeState _previous_state{MulticopterTurtleUtil::TurtleModeState::FLYING_DISABLED};
	bool _turtle_mode_armed{false}; // Track if turtle mode was responsible for arming
	bool _should_disarm_on_exit{false}; // Flag to disarm when exiting turtle mode
	bool _nav_state_change_requested{false}; // Track if we've already requested nav_state change to STAB
	bool _waiting_for_dshot_command{false}; // Flag to delay arming until DShot command completes
	hrt_abstime _dshot_command_start_time{0}; // Time when DShot command was sent
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	
	MulticopterTurtleUtil::Motor_data _motor_data[actuator_motors_s::NUM_CONTROLS];

	MulticopterTurtleUtil::MotorCommands getMotorCommands();
	static constexpr float AUX_CHANNEL_THRESHOLD{0.5f};
	static constexpr uint16_t ALL_MOTORS_REVERSIBLE{0xFFFF};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::COM_TURTLE_EN>) _param_com_turtle_en,
		(ParamInt<px4::params::TURTLE_AUX_CHN>) _param_turtle_aux_chn,
		(ParamFloat<px4::params::MPC_THR_HOVER>) _param_mpc_thr_hover
	);
};
