/****************************************************************************
 *
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskOrbit.hpp
 *
 * Flight task for orbiting in a circle around a target position
 *
 * @author Matthias Grob <maetugr@gmail.com>
 */

#pragma once

#include "FlightTaskManualAltitudeSmoothVel.hpp"
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/orbit_status.h>
#include <lib/slew_rate/SlewRateYaw.hpp>
#include <lib/motion_planning/HeadingSmoothing.hpp>
#include <lib/motion_planning/PositionSmoothing.hpp>
#include <lib/motion_planning/VelocitySmoothing.hpp>
#include <lib/slew_rate/SlewRate.hpp>


class FlightTaskOrbit : public FlightTaskManualAltitudeSmoothVel
{
public:

	FlightTaskOrbit();
	virtual ~FlightTaskOrbit() = default;

	bool applyCommandParameters(const vehicle_command_s &command, bool &success) override;
	bool activate(const trajectory_setpoint_s &last_setpoint) override;
	bool update() override;

protected:
	/**
	 * Send out telemetry information for the log and MAVLink.
	 * @return true on success, false on error
	 */
	bool sendTelemetry();

private:
	/* TODO: Should be controlled by params */
	static constexpr float _radius_min = 1.f;
	static constexpr float _velocity_max = 10.f;
	static constexpr float _acceleration_max = 2.f;
	static constexpr float _horizontal_acceptance_radius = 2.f;

	/**
	 * Check the feasibility of orbit parameters with respect to
	 * centripetal acceleration a = v^2 / r
	 * @param radius desired radius
	 * @param velocity desired velocity
	 * @param acceleration maximal allowed acceleration
	 * @return true on success, false if value not accepted
	 */
	bool _accelerationValid(float radius, float velocity, float acceleration) const;

	/**
	 * Checks if desired orbit params are feasible. If not,
	 * params are modified such that it is possible
	 * returns a feasible radius.
	 * @param radius The radius of the orbit. May get modified
	 * @param velocity The velocity of the orbit. May get modified
	 * @return Feasible orbit params
	 */
	void _sanitizeParams(float &radius, float &velocity) const;

	/**
	 * @brief updates the trajectory boundaries from props
	 */
	void _updateTrajectoryBoundaries();

	/**
	 * @brief Checks if the current position is on the circle or not
	 * Uses the params
	 */
	bool _is_position_on_circle() const;

	/** Adjusts radius and speed according to stick input */
	void _adjustParametersByStick();
	/** generates setpoints to smoothly reach the closest point on the circle when starting from far away */
	void _generate_circle_approach_setpoints();
	/** generates xy setpoints to make the vehicle orbit */
	void _generate_circle_setpoints();
	/** generates yaw setpoints to control the vehicle's heading */
	void _generate_circle_yaw_setpoints();

	float _orbit_velocity{};
	float _orbit_radius{};
	matrix::Vector3f _center; /**< local frame coordinates of the center point */

	bool _in_circle_approach = false;
	PositionSmoothing _position_smoothing;

	/** yaw behaviour during the orbit flight according to MAVLink's ORBIT_YAW_BEHAVIOUR enum */
	int _yaw_behaviour = orbit_status_s::ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER;
	bool _started_clockwise{true};
	bool _currently_orbiting{false};
	float _initial_heading = 0.f; /**< the heading of the drone when the orbit command was issued */
	HeadingSmoothing _heading_smoothing;
	SlewRate<float> _slew_rate_velocity;

	orb_advert_t _mavlink_log_pub{nullptr};
	uORB::PublicationMulti<orbit_status_s> _orbit_status_pub{ORB_ID(orbit_status)};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MC_ORBIT_RAD_MAX>) _param_mc_orbit_rad_max,
		(ParamInt<px4::params::MC_ORBIT_YAW_MOD>) _param_mc_orbit_yaw_mod,
		(ParamFloat<px4::params::MPC_XY_CRUISE>) _param_mpc_xy_cruise, /**< cruise speed for circle approach */
		(ParamFloat<px4::params::MPC_YAWRAUTO_MAX>) _param_mpc_yawrauto_max,
		(ParamFloat<px4::params::MPC_YAWRAUTO_ACC>) _param_mpc_yawrauto_acc,
		(ParamFloat<px4::params::MPC_XY_TRAJ_P>) _param_mpc_xy_traj_p,
		(ParamFloat<px4::params::NAV_MC_ALT_RAD>)
		_param_nav_mc_alt_rad, //vertical acceptance radius at which waypoints are updated
		(ParamFloat<px4::params::MPC_XY_ERR_MAX>) _param_mpc_xy_err_max,
		(ParamFloat<px4::params::MPC_ACC_HOR>) _param_mpc_acc_hor, // acceleration in flight
		(ParamFloat<px4::params::MPC_JERK_AUTO>) _param_mpc_jerk_auto,
		(ParamFloat<px4::params::MPC_ACC_UP_MAX>) _param_mpc_acc_up_max,
		(ParamFloat<px4::params::MPC_ACC_DOWN_MAX>) _param_mpc_acc_down_max,
		(ParamFloat<px4::params::MPC_Z_V_AUTO_UP>) _param_mpc_z_v_auto_up,
		(ParamFloat<px4::params::MPC_Z_V_AUTO_DN>) _param_mpc_z_v_auto_dn
	)
};
