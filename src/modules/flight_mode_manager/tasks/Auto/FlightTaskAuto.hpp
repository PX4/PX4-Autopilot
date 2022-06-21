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
 * @file FlightTaskAuto.hpp
 *
 * Map from global triplet to local quadruple.
 */

#pragma once

#include "FlightTask.hpp"
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/position_setpoint.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/motion_planning/PositionSmoothing.hpp>
#include "Sticks.hpp"
#include "StickAccelerationXY.hpp"
#include "StickYaw.hpp"

// TODO: make this switchable in the board config, like a module
#if CONSTRAINED_FLASH
#include <lib/avoidance/ObstacleAvoidance_dummy.hpp>
#else
#include <lib/avoidance/ObstacleAvoidance.hpp>
#endif

/**
 * This enum has to agree with position_setpoint_s type definition
 * The only reason for not using the struct position_setpoint is because
 * of the size
 */
enum class WaypointType : int {
	position = position_setpoint_s::SETPOINT_TYPE_POSITION,
	velocity = position_setpoint_s::SETPOINT_TYPE_VELOCITY,
	loiter = position_setpoint_s::SETPOINT_TYPE_LOITER,
	takeoff = position_setpoint_s::SETPOINT_TYPE_TAKEOFF,
	land = position_setpoint_s::SETPOINT_TYPE_LAND,
	idle = position_setpoint_s::SETPOINT_TYPE_IDLE,
	follow_target = position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET,
};

enum class State {
	offtrack, /**< Vehicle is more than cruise speed away from track */
	target_behind, /**< Vehicle is in front of target. */
	previous_infront, /**< Vehilce is behind previous waypoint.*/
	none /**< Vehicle is in normal tracking mode from triplet previous to triplet target */
};

class FlightTaskAuto : public FlightTask
{
public:
	FlightTaskAuto();

	virtual ~FlightTaskAuto() = default;
	bool activate(const trajectory_setpoint_s &last_setpoint) override;
	void reActivate() override;
	bool updateInitialize() override;
	bool update() override;

	/**
	 * Sets an external yaw handler which can be used to implement a different yaw control strategy.
	 */
	void setYawHandler(WeatherVane *ext_yaw_handler) override {_ext_yaw_handler = ext_yaw_handler;}
	void overrideCruiseSpeed(const float cruise_speed_m_s) override;

protected:
	matrix::Vector2f _getTargetVelocityXY(); /**< only used for follow-me and only here because of legacy reason.*/
	void _updateInternalWaypoints(); /**< Depending on state of vehicle, the internal waypoints might differ from target (for instance if offtrack). */
	bool _compute_heading_from_2D_vector(float &heading, matrix::Vector2f v); /**< Computes and sets heading a 2D vector */

	/** Reset position or velocity setpoints in case of EKF reset event */
	void _ekfResetHandlerPositionXY(const matrix::Vector2f &delta_xy) override;
	void _ekfResetHandlerVelocityXY(const matrix::Vector2f &delta_vxy) override;
	void _ekfResetHandlerPositionZ(float delta_z) override;
	void _ekfResetHandlerVelocityZ(float delta_vz) override;
	void _ekfResetHandlerHeading(float delta_psi) override;

	void _checkEmergencyBraking();
	bool _generateHeadingAlongTraj(); /**< Generates heading along trajectory. */
	bool isTargetModified() const;
	void _updateTrajConstraints();

	/** determines when to trigger a takeoff (ignored in flight) */
	bool _checkTakeoff() override { return _want_takeoff; };

	void _prepareLandSetpoints();
	bool _highEnoughForLandingGear(); /**< Checks if gears can be lowered. */

	void updateParams() override; /**< See ModuleParam class */

	matrix::Vector3f _prev_prev_wp{}; /**< Pre-previous waypoint (local frame). This will be used for smoothing trajectories -> not used yet. */
	matrix::Vector3f _prev_wp{}; /**< Previous waypoint  (local frame). If no previous triplet is available, the prev_wp is set to current position. */
	bool _prev_was_valid{false};
	matrix::Vector3f _target{}; /**< Target waypoint  (local frame).*/
	matrix::Vector3f _next_wp{}; /**< The next waypoint after target (local frame). If no next setpoint is available, next is set to target. */
	bool _next_was_valid{false};
	float _mc_cruise_speed{NAN}; /**< Requested cruise speed. If not valid, default cruise speed is used. */
	WaypointType _type{WaypointType::idle}; /**< Type of current target triplet. */

	uORB::SubscriptionData<home_position_s>			_sub_home_position{ORB_ID(home_position)};
	uORB::SubscriptionData<vehicle_status_s>		_sub_vehicle_status{ORB_ID(vehicle_status)};

	State _current_state{State::none};
	float _target_acceptance_radius{0.0f}; /**< Acceptances radius of the target */
	int _mission_gear{landing_gear_s::GEAR_KEEP};

	float _yaw_sp_prev{NAN};
	AlphaFilter<float> _yawspeed_filter;
	bool _yaw_sp_aligned{false};

	ObstacleAvoidance _obstacle_avoidance; /**< class adjusting setpoints according to external avoidance module's input */

	PositionSmoothing _position_smoothing;
	Vector3f _unsmoothed_velocity_setpoint;
	Sticks _sticks;
	StickAccelerationXY _stick_acceleration_xy;
	StickYaw _stick_yaw;
	matrix::Vector3f _land_position;
	float _land_heading;
	WaypointType _type_previous{WaypointType::idle}; /**< Previous type of current target triplet. */
	bool _is_emergency_braking_active{false};
	bool _want_takeoff{false};

	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTask,
					(ParamFloat<px4::params::MPC_XY_CRUISE>) _param_mpc_xy_cruise,
					(ParamFloat<px4::params::NAV_MC_ALT_RAD>)
					_param_nav_mc_alt_rad, //vertical acceptance radius at which waypoints are updated
					(ParamInt<px4::params::MPC_YAW_MODE>) _param_mpc_yaw_mode, // defines how heading is executed,
					(ParamInt<px4::params::COM_OBS_AVOID>) _param_com_obs_avoid, // obstacle avoidance active
					(ParamFloat<px4::params::MPC_YAWRAUTO_MAX>) _param_mpc_yawrauto_max,
					(ParamFloat<px4::params::MIS_YAW_ERR>) _param_mis_yaw_err, // yaw-error threshold
					(ParamBool<px4::params::WV_EN>) _param_wv_en, // enable/disable weather vane (VTOL)
					(ParamFloat<px4::params::MPC_ACC_HOR>) _param_mpc_acc_hor, // acceleration in flight
					(ParamFloat<px4::params::MPC_ACC_UP_MAX>) _param_mpc_acc_up_max,
					(ParamFloat<px4::params::MPC_ACC_DOWN_MAX>) _param_mpc_acc_down_max,
					(ParamFloat<px4::params::MPC_JERK_AUTO>) _param_mpc_jerk_auto,
					(ParamFloat<px4::params::MPC_XY_TRAJ_P>) _param_mpc_xy_traj_p,
					(ParamFloat<px4::params::MPC_XY_ERR_MAX>) _param_mpc_xy_err_max,
					(ParamFloat<px4::params::MPC_LAND_SPEED>) _param_mpc_land_speed,
					(ParamFloat<px4::params::MPC_LAND_CRWL>) _param_mpc_land_crawl_speed,
					(ParamInt<px4::params::MPC_LAND_RC_HELP>) _param_mpc_land_rc_help,
					(ParamFloat<px4::params::MPC_LAND_ALT1>)
					_param_mpc_land_alt1, // altitude at which we start ramping down speed
					(ParamFloat<px4::params::MPC_LAND_ALT2>)
					_param_mpc_land_alt2, // altitude at which we descend at land speed
					(ParamFloat<px4::params::MPC_LAND_ALT3>)
					_param_mpc_land_alt3, // altitude where we switch to crawl speed, if LIDAR available
					(ParamFloat<px4::params::MPC_Z_V_AUTO_UP>) _param_mpc_z_v_auto_up,
					(ParamFloat<px4::params::MPC_Z_V_AUTO_DN>) _param_mpc_z_v_auto_dn,
					(ParamFloat<px4::params::MPC_TKO_SPEED>) _param_mpc_tko_speed,
					(ParamFloat<px4::params::MPC_TKO_RAMP_T>)
					_param_mpc_tko_ramp_t, // time constant for smooth takeoff ramp
					(ParamFloat<px4::params::MPC_MAN_Y_MAX>) _param_mpc_man_y_max
				       );

private:
	matrix::Vector2f _lock_position_xy{NAN, NAN}; /**< if no valid triplet is received, lock positition to current position */
	bool _yaw_lock{false}; /**< if within acceptance radius, lock yaw to current yaw */

	uORB::SubscriptionData<position_setpoint_triplet_s> _sub_triplet_setpoint{ORB_ID(position_setpoint_triplet)};

	matrix::Vector3f
	_triplet_target; /**< current triplet from navigator which may differ from the intenal one (_target) depending on the vehicle state. */
	matrix::Vector3f
	_triplet_prev_wp; /**< previous triplet from navigator which may differ from the intenal one (_prev_wp) depending on the vehicle state.*/
	matrix::Vector3f
	_triplet_next_wp; /**< next triplet from navigator which may differ from the intenal one (_next_wp) depending on the vehicle state.*/
	matrix::Vector3f _closest_pt; /**< closest point to the vehicle position on the line previous - target */

	hrt_abstime _time_last_cruise_speed_override{0}; ///< timestamp the cruise speed was last time overriden using DO_CHANGE_SPEED

	MapProjection _reference_position{}; /**< Class used to project lat/lon setpoint into local frame. */
	float _reference_altitude{NAN}; /**< Altitude relative to ground. */
	hrt_abstime _time_stamp_reference{0}; /**< time stamp when last reference update occured. */

	WeatherVane *_ext_yaw_handler{nullptr};	/**< external weathervane library, used to implement a yaw control law that turns the vehicle nose into the wind */


	void _limitYawRate(); /**< Limits the rate of change of the yaw setpoint. */
	bool _evaluateTriplets(); /**< Checks and sets triplets. */
	bool _isFinite(const position_setpoint_s &sp); /**< Checks if all waypoint triplets are finite. */
	bool _evaluateGlobalReference(); /**< Check is global reference is available. */
	State _getCurrentState(); /**< Computes the current vehicle state based on the vehicle position and navigator triplets. */
	void _set_heading_from_mode(); /**< @see  MPC_YAW_MODE */
};
