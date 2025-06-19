/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

// PX4 includes
#include <px4_platform_common/module_params.h>

// Libraries
#include <lib/rover_control/RoverControl.hpp>
#include <math.h>

// uORB includes
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/rover_position_setpoint.h>

/**
 * @brief Class for Mecanum auto mode.
 */
class MecanumAutoMode : public ModuleParams
{
public:
	/**
	 * @brief Constructor for auto mode.
	 * @param parent The parent ModuleParams object.
	 */
	MecanumAutoMode(ModuleParams *parent);
	~MecanumAutoMode() = default;

	/**
	 * @brief Generate and publish roverPositionSetpoint from positionSetpointTriplet.
	 */
	void autoControl();

protected:
	/**
	 * @brief Update the parameters of the module.
	 */
	void updateParams() override;

private:
	/**
	 * @brief Calculate the speed at which the rover should arrive at the current waypoint. During waypoint transition the speed is restricted to
	 * Maximum_speed * (1 - normalized_transition_angle * RM_MISS_VEL_GAIN).
	 * @param cruising_speed Cruising speed [m/s].
	 * @param waypoint_transition_angle Angle between the prevWP-currWP and currWP-nextWP line segments [rad]
	 * @param max_speed Maximum speed setpoint [m/s]
	 * @param speed_red Tuning parameter for the speed reduction during waypoint transition.
	 * @param curr_wp_type Type of the current waypoint.
	 * @return Speed setpoint [m/s].
	 */
	float arrivalSpeed(const float cruising_speed, const float waypoint_transition_angle, const float max_speed,
			   const float speed_red, int curr_wp_type);

	// uORB subscriptions
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _position_setpoint_triplet_sub{ORB_ID(position_setpoint_triplet)};

	// uORB publications
	uORB::Publication<rover_position_setpoint_s>    _rover_position_setpoint_pub{ORB_ID(rover_position_setpoint)};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::RO_SPEED_LIM>) _param_ro_speed_limit,
		(ParamFloat<px4::params::RO_SPEED_RED>) _param_ro_speed_red
	)
};
