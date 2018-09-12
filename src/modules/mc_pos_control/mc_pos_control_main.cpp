/****************************************************************************
 *
 *   Copyright (c) 2013 - 2017 PX4 Development Team. All rights reserved.
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
 * @file mc_pos_control_main.cpp
 * Multicopter position controller.
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_module_params.h>
#include <px4_tasks.h>
#include <px4_module.h>
#include <px4_posix.h>
#include <drivers/drv_hrt.h>
#include <systemlib/hysteresis/hysteresis.h>
#include <commander/px4_custom_mode.h>

#include <uORB/topics/home_position.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_trajectory_waypoint.h>

#include <float.h>
#include <mathlib/mathlib.h>
#include <systemlib/mavlink_log.h>

#include <controllib/blocks.hpp>

#include <lib/FlightTasks/FlightTasks.hpp>
#include <lib/WeatherVane/WeatherVane.hpp>
#include "PositionControl.hpp"
#include "Utility/ControlMath.hpp"

/**
 * Multicopter position control app start / stop handling function
 */
extern "C" __EXPORT int mc_pos_control_main(int argc, char *argv[]);

class MulticopterPositionControl : public ModuleBase<MulticopterPositionControl>, public control::SuperBlock,
	public ModuleParams
{
public:
	MulticopterPositionControl();

	~MulticopterPositionControl();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static MulticopterPositionControl *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

private:

	bool 		_in_smooth_takeoff = false; 		/**<true if takeoff ramp is applied */

	orb_advert_t	_att_sp_pub{nullptr};			/**< attitude setpoint publication */
	orb_advert_t	_local_pos_sp_pub{nullptr};		/**< vehicle local position setpoint publication */
	orb_advert_t _traj_wp_avoidance_desired_pub{nullptr}; /**< trajectory waypoint desired publication */
	orb_advert_t _pub_vehicle_command{nullptr};           /**< vehicle command publication */
	orb_id_t _attitude_setpoint_id{nullptr};

	int		_vehicle_status_sub{-1};		/**< vehicle status subscription */
	int		_vehicle_land_detected_sub{-1};	/**< vehicle land detected subscription */
	int		_control_mode_sub{-1};		/**< vehicle control mode subscription */
	int		_params_sub{-1};			/**< notification of parameter updates */
	int		_local_pos_sub{-1};			/**< vehicle local position */
	int		_home_pos_sub{-1}; 			/**< home position */
	int		_traj_wp_avoidance_sub{-1};	/**< trajectory waypoint */

	int _task_failure_count{0};         /**< counter for task failures */

	float _takeoff_speed = -1.f; /**< For flighttask interface used only. It can be thrust or velocity setpoints */
	float _takeoff_reference_z; /**< Z-position when takeoff was initiated */

	vehicle_status_s 			_vehicle_status{};		/**< vehicle status */
	vehicle_land_detected_s 		_vehicle_land_detected{};	/**< vehicle land detected */
	vehicle_attitude_setpoint_s		_att_sp{};			/**< vehicle attitude setpoint */
	vehicle_control_mode_s			_control_mode{};		/**< vehicle control mode */
	vehicle_local_position_s		_local_pos{};			/**< vehicle local position */
	home_position_s				_home_pos{};			/**< home position */
	vehicle_trajectory_waypoint_s		_traj_wp_avoidance{};		/**< trajectory waypoint */
	vehicle_trajectory_waypoint_s		_traj_wp_avoidance_desired{};	/**< desired waypoints, inputs to an obstacle avoidance module */

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MPC_TKO_RAMP_T>) _takeoff_ramp_time, /**< time constant for smooth takeoff ramp */
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _vel_max_up,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _vel_max_down,
		(ParamFloat<px4::params::MPC_LAND_SPEED>) _land_speed,
		(ParamFloat<px4::params::MPC_TKO_SPEED>) _tko_speed,
		(ParamFloat<px4::params::MPC_LAND_ALT2>) MPC_LAND_ALT2, // altitude at which speed limit downwards reached minimum speed
		(ParamInt<px4::params::MPC_POS_MODE>) MPC_POS_MODE,
		(ParamInt<px4::params::MPC_ALT_MODE>) MPC_ALT_MODE,
		(ParamFloat<px4::params::MPC_IDLE_TKO>) MPC_IDLE_TKO, /**< time constant for smooth takeoff ramp */
		(ParamInt<px4::params::MPC_OBS_AVOID>) MPC_OBS_AVOID /**< enable obstacle avoidance */
	);

	control::BlockDerivative _vel_x_deriv; /**< velocity derivative in x */
	control::BlockDerivative _vel_y_deriv; /**< velocity derivative in y */
	control::BlockDerivative _vel_z_deriv; /**< velocity derivative in z */

	FlightTasks _flight_tasks; /**< class that generates position controller tracking setpoints*/
	PositionControl _control; /**< class that handles the core PID position controller */
	PositionControlStates _states; /**< structure that contains required state information for position control */

	hrt_abstime _last_warn = 0; /**< timer when the last warn message was sent out */

	bool _in_failsafe = false; /**< true if failsafe was entered within current cycle */

	/** Timeout in us for trajectory data to get considered invalid */
	static constexpr uint64_t TRAJECTORY_STREAM_TIMEOUT_US = 500000;
	/**< number of tries before switching to a failsafe flight task */
	static constexpr int NUM_FAILURE_TRIES = 10;
	/**< If Flighttask fails, keep 0.2 seconds the current setpoint before going into failsafe land */
	static constexpr uint64_t LOITER_TIME_BEFORE_DESCEND = 200000;


	/**
	 * Hysteresis that turns true once vehicle is armed for MPC_IDLE_TKO seconds.
	 * A real vehicle requires some time to accelerates the propellers to IDLE speed. To ensure
	 * that the propellers reach idle speed before initiating a takeoff, a delay of MPC_IDLE_TKO
	 * is added.
	 */
	systemlib::Hysteresis _arm_hysteresis{false}; /**< becomes true once vehicle is armed for MPC_IDLE_TKO seconds */

	systemlib::Hysteresis _failsafe_land_hysteresis{false}; /**< becomes true if task did not update correctly for LOITER_TIME_BEFORE_DESCEND */

	WeatherVane *_wv_controller{nullptr};

	/**
	 * Update our local parameter cache.
	 * Parameter update can be forced when argument is true.
	 * @param force forces parameter update.
	 */
	int		parameters_update(bool force);

	/**
	 * Check for changes in subscribed topics.
	 */
	void		poll_subscriptions();

	/**
	 * Check for validity of positon/velocity states.
	 * @param vel_sp_z velocity setpoint in z-direction
	 */
	void set_vehicle_states(const float &vel_sp_z);

	/**
	 * Limit altitude based on land-detector.
	 * @param setpoint needed to detect vehicle intention.
	 */
	void limit_altitude(vehicle_local_position_setpoint_s &setpoint);

	/**
	 * Prints a warning message at a lowered rate.
	 * @param str the message that has to be printed.
	 */
	void warn_rate_limited(const char *str);

	/**
	 * Publish attitude.
	 */
	void publish_attitude();

	/**
	 * Publish local position setpoint.
	 * This is only required for logging.
	 */
	void publish_local_pos_sp(const vehicle_local_position_setpoint_s &local_pos_sp);

	/**
	 * Checks if smooth takeoff is initiated.
	 * @param position_setpoint_z the position setpoint in the z-Direction
	 * @param velocity setpoint_z the velocity setpoint in the z-Direction
	 */
	void check_for_smooth_takeoff(const float &position_setpoint_z, const float &velocity_setpoint_z,
				      const vehicle_constraints_s &constraints);

	/**
	 * Check if smooth takeoff has ended and updates accordingly.
	 * @param position_setpoint_z the position setpoint in the z-Direction
	 * @param velocity setpoint_z the velocity setpoint in the z-Direction
	 */
	void update_smooth_takeoff(const float &position_setpoint_z, const float &velocity_setpoint_z);

	/**
	 * Adjust the thrust setpoint during landing.
	 * Thrust is adjusted to support the land-detector during detection.
	 * @param thrust_setpoint gets adjusted based on land-detector state
	 */
	void limit_thrust_during_landing(matrix::Vector3f &thrust_sepoint);

	/**
	 * Start flightasks based on navigation state.
	 * This methods activates a task based on the navigation state.
	 */
	void start_flight_task();

	/**
	 * Failsafe.
	 * If flighttask fails for whatever reason, then do failsafe. This could
	 * occur if the commander fails to switch to a mode in case of invalid states or
	 * setpoints. The failsafe will occur after LOITER_TIME_BEFORE_DESCEND. If force is set
	 * to true, the failsafe will be initiated immediately.
	 */
	void failsafe(vehicle_local_position_setpoint_s &setpoint, const PositionControlStates &states, const bool force,
		      const bool warn);

	/**
	 * Fill desired vehicle_trajectory_waypoint:
	 * point1: current position, desired velocity
	 * point2: current triplet only if in auto mode
	 * @param states current vehicle state
	 */
	void update_avoidance_waypoint_desired(PositionControlStates &states, vehicle_local_position_setpoint_s &setpoint);

	/**
	 * Check whether or not use the obstacle avoidance waypoint
	 */
	bool use_obstacle_avoidance();

	/**
	 * Overwrite setpoint with waypoint coming from obstacle avoidance
	 */
	void execute_avoidance_waypoint(vehicle_local_position_setpoint_s &setpoint);

	/**
	 * Publish desired vehicle_trajectory_waypoint
	 */
	void publish_avoidance_desired_waypoint();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static int	task_main_trampoline(int argc, char *argv[]);

	/**
	 * check if task should be switched because of failsafe
	 */
	void check_failure(bool task_failure, uint8_t nav_state);
	/**
	 * send vehicle command to inform commander about failsafe
	 */
	void send_vehicle_cmd_do(uint8_t nav_state);

	/**
	 * Main sensor collection task.
	 */
	void		task_main();
};


int MulticopterPositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The controller has two loops: a P loop for position error and a PID loop for velocity error.
Output of the velocity controller is thrust vector that is split to thrust direction
(i.e. rotation matrix for multicopter orientation) and thrust scalar (i.e. multicopter thrust itself).

The controller doesn't use Euler angles for its work, they are generated only for more human-friendly control and
logging.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


MulticopterPositionControl::MulticopterPositionControl() :
	SuperBlock(nullptr, "MPC"),
	ModuleParams(nullptr),
	_vel_x_deriv(this, "VELD"),
	_vel_y_deriv(this, "VELD"),
	_vel_z_deriv(this, "VELD"),
	_control(this)
{
	// fetch initial parameter values
	parameters_update(true);
	// set failsafe hysteresis
	_failsafe_land_hysteresis.set_hysteresis_time_from(false, LOITER_TIME_BEFORE_DESCEND);
}

MulticopterPositionControl::~MulticopterPositionControl()
{
	if (_wv_controller != nullptr) {
		delete _wv_controller;
	}
}

void
MulticopterPositionControl::warn_rate_limited(const char *string)
{
	hrt_abstime now = hrt_absolute_time();

	if (now - _last_warn > 200000) {
		PX4_WARN(string);
		_last_warn = now;
	}
}

int
MulticopterPositionControl::parameters_update(bool force)
{
	bool updated;
	struct parameter_update_s param_upd;

	orb_check(_params_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_upd);
	}

	if (updated || force) {
		ModuleParams::updateParams();
		SuperBlock::updateParams();

		_flight_tasks.handleParameterUpdate();

		// initialize vectors from params and enforce constraints
		_tko_speed.set(math::min(_tko_speed.get(), _vel_max_up.get()));
		_land_speed.set(math::min(_land_speed.get(), _vel_max_down.get()));

		// set trigger time for arm hysteresis
		_arm_hysteresis.set_hysteresis_time_from(false, (int)(MPC_IDLE_TKO.get() * 1000000.0f));

		if (_wv_controller != nullptr) {
			_wv_controller->update_parameters();
		}
	}

	return OK;
}

void
MulticopterPositionControl::poll_subscriptions()
{
	bool updated;

	orb_check(_vehicle_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

		// set correct uORB ID, depending on if vehicle is VTOL or not
		if (!_attitude_setpoint_id) {
			if (_vehicle_status.is_vtol) {
				_attitude_setpoint_id = ORB_ID(mc_virtual_attitude_setpoint);

			} else {
				_attitude_setpoint_id = ORB_ID(vehicle_attitude_setpoint);
			}
		}

		// if vehicle is a VTOL we want to enable weathervane capabilities
		if (_wv_controller == nullptr && _vehicle_status.is_vtol) {
			_wv_controller = new WeatherVane();
		}
	}

	orb_check(_vehicle_land_detected_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);
	}

	orb_check(_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
	}

	orb_check(_local_pos_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
	}

	orb_check(_home_pos_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(home_position), _home_pos_sub, &_home_pos);
	}

	orb_check(_traj_wp_avoidance_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_trajectory_waypoint), _traj_wp_avoidance_sub, &_traj_wp_avoidance);
	}
}

void
MulticopterPositionControl::limit_altitude(vehicle_local_position_setpoint_s &setpoint)
{
	if (_vehicle_land_detected.alt_max < 0.0f || !_home_pos.valid_alt || !_local_pos.v_z_valid) {
		// there is no altitude limitation present or the required information not available
		return;
	}

	float altitude_above_home = -(_states.position(2) - _home_pos.z);

	if (altitude_above_home > _vehicle_land_detected.alt_max) {
		// we are above maximum altitude
		setpoint.z = -_vehicle_land_detected.alt_max +  _home_pos.z;
		setpoint.vz = 0.0f;

	} else if (setpoint.vz <= 0.0f) {
		// we want to fly upwards: check if vehicle does not exceed altitude

		float delta_p = _vehicle_land_detected.alt_max - altitude_above_home;

		if (fabsf(setpoint.vz) * _dt > delta_p) {
			setpoint.z = -_vehicle_land_detected.alt_max +  _home_pos.z;
			setpoint.vz = 0.0f;
		}
	}
}

void
MulticopterPositionControl::set_vehicle_states(const float &vel_sp_z)
{
	if (_local_pos.timestamp == 0) {
		return;
	}

	// only set position states if valid and finite
	if (PX4_ISFINITE(_local_pos.x) && PX4_ISFINITE(_local_pos.y) && _local_pos.xy_valid) {
		_states.position(0) = _local_pos.x;
		_states.position(1) = _local_pos.y;

	} else {
		_states.position(0) = _states.position(1) = NAN;
	}

	if (PX4_ISFINITE(_local_pos.z) && _local_pos.z_valid) {
		_states.position(2) = _local_pos.z;

	} else {
		_states.position(2) = NAN;
	}

	if (PX4_ISFINITE(_local_pos.vx) && PX4_ISFINITE(_local_pos.vy) && _local_pos.v_xy_valid) {
		_states.velocity(0) = _local_pos.vx;
		_states.velocity(1) = _local_pos.vy;
		_states.acceleration(0) = _vel_x_deriv.update(-_states.velocity(0));
		_states.acceleration(1) = _vel_y_deriv.update(-_states.velocity(1));

	} else {
		_states.velocity(0) = _states.velocity(1) = NAN;
		_states.acceleration(0) = _states.acceleration(1) = NAN;

		// since no valid velocity, update derivate with 0
		_vel_x_deriv.update(0.0f);
		_vel_y_deriv.update(0.0f);
	}

	if (MPC_ALT_MODE.get() && _local_pos.dist_bottom_valid && PX4_ISFINITE(_local_pos.dist_bottom_rate)) {
		// terrain following
		_states.velocity(2) = -_local_pos.dist_bottom_rate;
		_states.acceleration(2) = _vel_z_deriv.update(-_states.velocity(2));

	} else if (PX4_ISFINITE(_local_pos.vz)) {

		_states.velocity(2) = _local_pos.vz;

		if (PX4_ISFINITE(vel_sp_z) && fabsf(vel_sp_z) > FLT_EPSILON && PX4_ISFINITE(_local_pos.z_deriv)) {
			// A change in velocity is demanded. Set velocity to the derivative of position
			// because it has less bias but blend it in across the landing speed range
			float weighting = fminf(fabsf(vel_sp_z) / _land_speed.get(), 1.0f);
			_states.velocity(2) = _local_pos.z_deriv * weighting + _local_pos.vz * (1.0f - weighting);
		}

		_states.acceleration(2) = _vel_z_deriv.update(-_states.velocity(2));

	} else {
		_states.velocity(2) = _states.acceleration(2) = NAN;
		// since no valid velocity, update derivate with 0
		_vel_z_deriv.update(0.0f);

	}

	if (PX4_ISFINITE(_local_pos.yaw)) {
		_states.yaw = _local_pos.yaw;
	}
}

void
MulticopterPositionControl::run()
{
	// do subscriptions
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_home_pos_sub = orb_subscribe(ORB_ID(home_position));
	_traj_wp_avoidance_sub = orb_subscribe(ORB_ID(vehicle_trajectory_waypoint));

	parameters_update(true);

	// get an initial update for all sensor and status data
	poll_subscriptions();

	// We really need to know from the beginning if we're landed or in-air.
	orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);

	hrt_abstime t_prev = 0;

	// Let's be safe and have the landing gear down by default
	_att_sp.landing_gear = vehicle_attitude_setpoint_s::LANDING_GEAR_DOWN;

	// wakeup source
	px4_pollfd_struct_t fds[1];

	fds[0].fd = _local_pos_sub;
	fds[0].events = POLLIN;

	while (!should_exit()) {
		// wait for up to 20ms for data
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 20);

		// pret == 0: go through the loop anyway to copy manual input at 50 Hz.

		// this is undesirable but not much we can do
		if (pret < 0) {
			PX4_ERR("poll error %d, %d", pret, errno);
			continue;
		}

		poll_subscriptions();

		parameters_update(false);

		hrt_abstime t = hrt_absolute_time();
		const float dt = t_prev != 0 ? (t - t_prev) / 1e6f : 0.004f;
		t_prev = t;

		// set dt for control blocks
		setDt(dt);

		const bool was_in_failsafe = _in_failsafe;
		_in_failsafe = false;

		// activate the weathervane controller if required. If activated a flighttask can use it to implement a yaw-rate control strategy
		// that turns the nose of the vehicle into the wind
		if (_wv_controller != nullptr) {

			// in manual mode we just want to use weathervane if position is controlled as well
			if (_wv_controller->weathervane_enabled() && !(_control_mode.flag_control_manual_enabled
					&& !_control_mode.flag_control_position_enabled)) {
				_wv_controller->activate();

			} else {
				_wv_controller->deactivate();
			}

			_wv_controller->update(matrix::Quatf(_att_sp.q_d), _local_pos.yaw);
		}

		if (_control_mode.flag_armed) {
			// as soon vehicle is armed check for flighttask
			start_flight_task();
			// arm hysteresis prevents vehicle to takeoff
			// before propeller reached idle speed.
			_arm_hysteresis.set_state_and_update(true);

		} else {
			// disable flighttask
			_flight_tasks.switchTask(FlightTaskIndex::None);
			// reset arm hysteresis
			_arm_hysteresis.set_state_and_update(false);
		}

		// check if any task is active
		if (_flight_tasks.isAnyTaskActive()) {

			// setpoints from flighttask
			vehicle_local_position_setpoint_s setpoint;

			_flight_tasks.setYawHandler(_wv_controller);

			// update task
			if (!_flight_tasks.update()) {
				// FAILSAFE
				// Task was not able to update correctly. Do Failsafe.
				failsafe(setpoint, _states, false, !was_in_failsafe);

			} else {
				setpoint = _flight_tasks.getPositionSetpoint();
				_failsafe_land_hysteresis.set_state_and_update(false);

				// Check if position, velocity or thrust pairs are valid -> trigger failsaife if no pair is valid
				if (!(PX4_ISFINITE(setpoint.x) && PX4_ISFINITE(setpoint.y)) &&
				    !(PX4_ISFINITE(setpoint.vx) && PX4_ISFINITE(setpoint.vy)) &&
				    !(PX4_ISFINITE(setpoint.thrust[0]) && PX4_ISFINITE(setpoint.thrust[1]))) {
					failsafe(setpoint, _states, true, !was_in_failsafe);
				}

				// Check if altitude, climbrate or thrust in D-direction are valid -> trigger failsafe if none
				// of these setpoints are valid
				if (!PX4_ISFINITE(setpoint.z) && !PX4_ISFINITE(setpoint.vz) && !PX4_ISFINITE(setpoint.thrust[2])) {
					failsafe(setpoint, _states, true, !was_in_failsafe);
				}
			}

			/* desired waypoints for obstacle avoidance:
			 * point_0 contains the current position with the desired velocity
			 * point_1 contains _pos_sp_triplet.current if valid
			 */
			update_avoidance_waypoint_desired(_states, setpoint);

			vehicle_constraints_s constraints = _flight_tasks.getConstraints();

			// check if all local states are valid and map accordingly
			set_vehicle_states(setpoint.vz);

			// we can only do a smooth takeoff if a valid velocity or position is available and are
			// armed long enough
			if (_arm_hysteresis.get_state() && PX4_ISFINITE(_states.position(2)) && PX4_ISFINITE(_states.velocity(2))) {
				check_for_smooth_takeoff(setpoint.z, setpoint.vz, constraints);
				update_smooth_takeoff(setpoint.z, setpoint.vz);
			}

			if (_in_smooth_takeoff) {
				// during smooth takeoff, constrain speed to takeoff speed
				constraints.speed_up = _takeoff_speed;
				// disable yaw command
				setpoint.yaw = setpoint.yawspeed = NAN;
				// don't control position in xy
				setpoint.x = setpoint.y = NAN;
				setpoint.vx = setpoint.vy = 0.0f;
			}

			if (_vehicle_land_detected.landed && !_in_smooth_takeoff && !PX4_ISFINITE(setpoint.thrust[2])) {
				// Keep throttle low when landed and NOT in smooth takeoff
				setpoint.thrust[0] = setpoint.thrust[1] = setpoint.thrust[2] = 0.0f;
				setpoint.x = setpoint.y = setpoint.z = NAN;
				setpoint.vx = setpoint.vy = setpoint.vz = NAN;
				setpoint.yawspeed = NAN;
				setpoint.yaw = _states.yaw;
				constraints.landing_gear = vehicle_constraints_s::GEAR_KEEP;
				// reactivate the task which will reset the setpoint to current state
				_flight_tasks.reActivate();
			}

			// limit altitude only if local position is valid
			if (PX4_ISFINITE(_states.position(2))) {
				limit_altitude(setpoint);
			}

			// Update states, setpoints and constraints.
			_control.updateConstraints(constraints);
			_control.updateState(_states);

			// adjust setpoints based on avoidance
			if (use_obstacle_avoidance()) {
				execute_avoidance_waypoint(setpoint);
			}

			// update position controller setpoints
			if (!_control.updateSetpoint(setpoint)) {
				warn_rate_limited("Position-Control Setpoint-Update failed");
			}

			// Generate desired thrust and yaw.
			_control.generateThrustYawSetpoint(_dt);

			matrix::Vector3f thr_sp = _control.getThrustSetpoint();

			// Adjust thrust setpoint based on landdetector only if the
			// vehicle is NOT in pure Manual mode and NOT in smooth takeoff
			if (!_in_smooth_takeoff && !PX4_ISFINITE(setpoint.thrust[2])) {
				limit_thrust_during_landing(thr_sp);
			}

			// Fill local position, velocity and thrust setpoint.
			vehicle_local_position_setpoint_s local_pos_sp{};
			local_pos_sp.timestamp = hrt_absolute_time();
			local_pos_sp.x = _control.getPosSp()(0);
			local_pos_sp.y = _control.getPosSp()(1);
			local_pos_sp.z = _control.getPosSp()(2);
			local_pos_sp.yaw = _control.getYawSetpoint();
			local_pos_sp.yawspeed = _control.getYawspeedSetpoint();

			local_pos_sp.vx = _control.getVelSp()(0);
			local_pos_sp.vy = _control.getVelSp()(1);
			local_pos_sp.vz = _control.getVelSp()(2);
			thr_sp.copyTo(local_pos_sp.thrust);

			// Publish local position setpoint (for logging only) and attitude setpoint (for attitude controller).
			publish_local_pos_sp(local_pos_sp);


			// Fill attitude setpoint. Attitude is computed from yaw and thrust setpoint.
			_att_sp = ControlMath::thrustToAttitude(thr_sp, _control.getYawSetpoint());
			_att_sp.yaw_sp_move_rate = _control.getYawspeedSetpoint();
			_att_sp.fw_control_yaw = false;
			_att_sp.apply_flaps = false;

			if (!constraints.landing_gear) {
				if (constraints.landing_gear == vehicle_constraints_s::GEAR_UP) {
					_att_sp.landing_gear = vehicle_attitude_setpoint_s::LANDING_GEAR_UP;
				}

				if (constraints.landing_gear == vehicle_constraints_s::GEAR_DOWN) {
					_att_sp.landing_gear = vehicle_attitude_setpoint_s::LANDING_GEAR_DOWN;
				}
			}

			// publish attitude setpoint
			// Note: this requires review. The reason for not sending
			// an attitude setpoint is because for non-flighttask modes
			// the attitude septoint should come from another source, otherwise
			// they might conflict with each other such as in offboard attitude control.
			publish_attitude();

		} else {
			// no flighttask is active: set attitude setpoint to idle
			_att_sp.roll_body = _att_sp.pitch_body = 0.0f;
			_att_sp.yaw_body = _local_pos.yaw;
			_att_sp.yaw_sp_move_rate = 0.0f;
			_att_sp.fw_control_yaw = false;
			_att_sp.apply_flaps = false;
			matrix::Quatf q_sp = matrix::Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body);
			q_sp.copyTo(_att_sp.q_d);
			_att_sp.q_d_valid = true;
			_att_sp.thrust = 0.0f;
		}
	}

	orb_unsubscribe(_vehicle_status_sub);
	orb_unsubscribe(_vehicle_land_detected_sub);
	orb_unsubscribe(_control_mode_sub);
	orb_unsubscribe(_params_sub);
	orb_unsubscribe(_local_pos_sub);
	orb_unsubscribe(_home_pos_sub);
	orb_unsubscribe(_traj_wp_avoidance_sub);
}

void
MulticopterPositionControl::start_flight_task()
{
	bool task_failure = false;
	int prev_failure_count = _task_failure_count;

	// offboard
	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD
	    && (_control_mode.flag_control_altitude_enabled ||
		_control_mode.flag_control_position_enabled ||
		_control_mode.flag_control_climb_rate_enabled ||
		_control_mode.flag_control_velocity_enabled ||
		_control_mode.flag_control_acceleration_enabled)) {

		int error = _flight_tasks.switchTask(FlightTaskIndex::Offboard);

		if (error != 0) {
			if (prev_failure_count == 0) {
				PX4_WARN("Offboard activation failed with error: %s", _flight_tasks.errorToString(error));
			}
			task_failure = true;
			_task_failure_count++;

		} else {
			// we want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}
	}

	// Auto-follow me
	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET) {
		int error = _flight_tasks.switchTask(FlightTaskIndex::AutoFollowMe);

		if (error != 0) {
			if (prev_failure_count == 0) {
				PX4_WARN("Follow-Me activation failed with error: %s", _flight_tasks.errorToString(error));
			}
			task_failure = true;
			_task_failure_count++;

		} else {
			// we want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}

	} else if (_control_mode.flag_control_auto_enabled) {
		// Auto relate maneuvers
		int error = _flight_tasks.switchTask(FlightTaskIndex::AutoLine);

		if (error != 0) {
			if (prev_failure_count == 0) {
				PX4_WARN("Auto activation failed with error: %s", _flight_tasks.errorToString(error));
			}
			task_failure = true;
			_task_failure_count++;

		} else {
			// we want to be in this mode, reset the failure count
			_task_failure_count = 0;
		}
	}

	// manual position control
	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL || task_failure) {

		int error = 0;

		switch (MPC_POS_MODE.get()) {
		case 0:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualPosition);
			break;

		case 1:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualPositionSmooth);
			break;

		case 2:
			error =  _flight_tasks.switchTask(FlightTaskIndex::Sport);
			break;

		default:
			error =  _flight_tasks.switchTask(FlightTaskIndex::ManualPosition);
			break;
		}

		if (error != 0) {
			if (prev_failure_count == 0) {
				PX4_WARN("Position-Ctrl activation failed with error: %s", _flight_tasks.errorToString(error));
			}
			task_failure = true;
			_task_failure_count++;

		} else {
			check_failure(task_failure, vehicle_status_s::NAVIGATION_STATE_POSCTL);
			task_failure = false;
		}
	}

	// manual altitude control
	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL || task_failure) {
		int error = _flight_tasks.switchTask(FlightTaskIndex::ManualAltitude);

		if (error != 0) {
			if (prev_failure_count == 0) {
				PX4_WARN("Altitude-Ctrl activation failed with error: %s", _flight_tasks.errorToString(error));
			}
			task_failure = true;
			_task_failure_count++;

		} else {
			check_failure(task_failure, vehicle_status_s::NAVIGATION_STATE_ALTCTL);
			task_failure = false;
		}
	}

	// manual stabilized control
	if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL
	    ||  _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_STAB || task_failure) {
		int error = _flight_tasks.switchTask(FlightTaskIndex::ManualStabilized);

		if (error != 0) {
			if (prev_failure_count == 0) {
				PX4_WARN("Stabilized-Ctrl failed with error: %s", _flight_tasks.errorToString(error));
			}
			task_failure = true;
			_task_failure_count++;

		} else {
			check_failure(task_failure, vehicle_status_s::NAVIGATION_STATE_STAB);
			task_failure = false;
		}
	}

	// check task failure
	if (task_failure) {

		// for some reason no flighttask was able to start.
		// go into failsafe flighttask
		int error = _flight_tasks.switchTask(FlightTaskIndex::Failsafe);

		if (error != 0) {
			// No task was activated.
			_flight_tasks.switchTask(FlightTaskIndex::None);
		}
	}
}

void
MulticopterPositionControl::check_for_smooth_takeoff(const float &z_sp, const float &vz_sp,
		const vehicle_constraints_s &constraints)
{
	// Check for smooth takeoff
	if (_vehicle_land_detected.landed && !_in_smooth_takeoff) {
		// Vehicle is still landed and no takeoff was initiated yet.
		// Adjust for different takeoff cases.
		// The minimum takeoff altitude needs to be at least 20cm above minimum distance or, if valid, above minimum distance
		// above ground.
		float min_altitude = PX4_ISFINITE(constraints.min_distance_to_ground) ? (constraints.min_distance_to_ground + 0.05f) :
				     0.2f;

		if ((PX4_ISFINITE(z_sp) && z_sp < _states.position(2) - min_altitude) ||
		    (PX4_ISFINITE(vz_sp) && vz_sp < math::min(-_tko_speed.get(), -0.6f))) {
			// There is a position setpoint above current position or velocity setpoint larger than
			// takeoff speed. Enable smooth takeoff.
			_in_smooth_takeoff = true;
			_takeoff_speed = -0.5f;
			_takeoff_reference_z = _states.position(2);

		} else {
			// Default
			_in_smooth_takeoff = false;
		}
	}
}

void
MulticopterPositionControl::update_smooth_takeoff(const float &z_sp, const float &vz_sp)
{
	// If in smooth takeoff, adjust setpoints based on what is valid:
	// 1. position setpoint is valid -> go with takeoffspeed to specific altitude
	// 2. position setpoint not valid but velocity setpoint valid: ramp up velocity
	if (_in_smooth_takeoff) {
		float desired_tko_speed = -vz_sp;

		// If there is a valid position setpoint, then set the desired speed to the takeoff speed.
		if (PX4_ISFINITE(z_sp)) {
			desired_tko_speed = _tko_speed.get();
		}

		// Ramp up takeoff speed.
		_takeoff_speed += desired_tko_speed * _dt / _takeoff_ramp_time.get();
		_takeoff_speed = math::min(_takeoff_speed, desired_tko_speed);

		// Smooth takeoff is achieved once desired altitude/velocity setpoint is reached.
		if (PX4_ISFINITE(z_sp)) {
			_in_smooth_takeoff = _states.position(2) - 0.2f > math::max(z_sp, _takeoff_reference_z - MPC_LAND_ALT2.get());

		} else  {
			_in_smooth_takeoff = _takeoff_speed < -vz_sp;
		}

	} else {
		_in_smooth_takeoff = false;
	}
}

void
MulticopterPositionControl::limit_thrust_during_landing(matrix::Vector3f &thr_sp)
{
	if (_vehicle_land_detected.ground_contact) {
		// Set thrust in xy to zero
		thr_sp(0) = 0.0f;
		thr_sp(1) = 0.0f;
		// Reset integral in xy is required because PID-controller does
		// know about the overwrite and would therefore increase the intragral term
		_control.resetIntegralXY();
	}

	if (_vehicle_land_detected.maybe_landed) {
		// we set thrust to zero
		// this will help to decide if we are actually landed or not
		thr_sp.zero();
		// We need to reset all integral terms otherwise the PID-controller
		// will continue to integrate
		_control.resetIntegralXY();
		_control.resetIntegralZ();
	}
}

void
MulticopterPositionControl::failsafe(vehicle_local_position_setpoint_s &setpoint, const PositionControlStates &states,
				     const bool force, const bool warn)
{
	_failsafe_land_hysteresis.set_state_and_update(true);

	if (!_failsafe_land_hysteresis.get_state() && !force) {
		// just keep current setpoint and don't do anything.



	} else {
		setpoint.x = setpoint.y = setpoint.z = NAN;
		setpoint.vx = setpoint.vy = setpoint.vz = NAN;
		setpoint.thrust[0] = setpoint.thrust[1] = setpoint.thrust[2] = NAN;
		setpoint.yaw = setpoint.yawspeed = NAN;

		if (PX4_ISFINITE(_states.velocity(2))) {
			// We have a valid velocity in D-direction.
			// descend downwards with landspeed.
			setpoint.vz = _land_speed.get();
			setpoint.thrust[0] = setpoint.thrust[1] = 0.0f;
			if (warn) {
				PX4_WARN("Failsafe: Descend with land-speed.");
			}

		} else {
			// Use the failsafe from the PositionController.
			if (warn) {
				PX4_WARN("Failsafe: Descend with just attitude control.");
			}
		}
		_in_failsafe = true;
	}
}

void
MulticopterPositionControl::update_avoidance_waypoint_desired(PositionControlStates &states,
		vehicle_local_position_setpoint_s &setpoint)
{
	if (MPC_OBS_AVOID.get()) {
		_traj_wp_avoidance_desired = _flight_tasks.getAvoidanceWaypoint();
		_traj_wp_avoidance_desired.timestamp = hrt_absolute_time();
		_traj_wp_avoidance_desired.type = vehicle_trajectory_waypoint_s::MAV_TRAJECTORY_REPRESENTATION_WAYPOINTS;

		states.position.copyTo(_traj_wp_avoidance_desired.waypoints[vehicle_trajectory_waypoint_s::POINT_0].position);

		_traj_wp_avoidance_desired.waypoints[vehicle_trajectory_waypoint_s::POINT_0].velocity[0] = setpoint.vx;
		_traj_wp_avoidance_desired.waypoints[vehicle_trajectory_waypoint_s::POINT_0].velocity[1] = setpoint.vy;
		_traj_wp_avoidance_desired.waypoints[vehicle_trajectory_waypoint_s::POINT_0].velocity[2] = setpoint.vz;

		states.acceleration.copyTo(_traj_wp_avoidance_desired.waypoints[vehicle_trajectory_waypoint_s::POINT_0].acceleration);

		_traj_wp_avoidance_desired.waypoints[vehicle_trajectory_waypoint_s::POINT_0].yaw = states.yaw;
		_traj_wp_avoidance_desired.waypoints[vehicle_trajectory_waypoint_s::POINT_0].yaw_speed = NAN;
		_traj_wp_avoidance_desired.waypoints[vehicle_trajectory_waypoint_s::POINT_0].point_valid = true;

		publish_avoidance_desired_waypoint();
	}
}

void
MulticopterPositionControl::execute_avoidance_waypoint(vehicle_local_position_setpoint_s &setpoint)
{
	setpoint.x = _traj_wp_avoidance.waypoints[vehicle_trajectory_waypoint_s::POINT_0].position[0];
	setpoint.y = _traj_wp_avoidance.waypoints[vehicle_trajectory_waypoint_s::POINT_0].position[1];
	setpoint.z = _traj_wp_avoidance.waypoints[vehicle_trajectory_waypoint_s::POINT_0].position[2];

	setpoint.vx = _traj_wp_avoidance.waypoints[vehicle_trajectory_waypoint_s::POINT_0].velocity[0];
	setpoint.vy = _traj_wp_avoidance.waypoints[vehicle_trajectory_waypoint_s::POINT_0].velocity[1];
	setpoint.vz = _traj_wp_avoidance.waypoints[vehicle_trajectory_waypoint_s::POINT_0].velocity[2];

	setpoint.acc_x = setpoint.acc_y = setpoint.acc_z = NAN;

	setpoint.yaw = _traj_wp_avoidance.waypoints[vehicle_trajectory_waypoint_s::POINT_0].yaw;
	setpoint.yawspeed = _traj_wp_avoidance.waypoints[vehicle_trajectory_waypoint_s::POINT_0].yaw_speed;
	Vector3f(NAN, NAN, NAN).copyTo(setpoint.thrust);
}

bool
MulticopterPositionControl::use_obstacle_avoidance()
{
	/* check that external obstacle avoidance is sending data and that the first point is valid */
	return (MPC_OBS_AVOID.get()
		&& (hrt_elapsed_time((hrt_abstime *)&_traj_wp_avoidance.timestamp) < TRAJECTORY_STREAM_TIMEOUT_US)
		&& (_traj_wp_avoidance.waypoints[vehicle_trajectory_waypoint_s::POINT_0].point_valid == true)
		&& ((_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION) ||
		    (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL)));
}

void
MulticopterPositionControl::publish_avoidance_desired_waypoint()
{
	// publish desired waypoint
	if (_traj_wp_avoidance_desired_pub != nullptr) {
		orb_publish(ORB_ID(vehicle_trajectory_waypoint_desired), _traj_wp_avoidance_desired_pub, &_traj_wp_avoidance_desired);

	} else {
		_traj_wp_avoidance_desired_pub = orb_advertise(ORB_ID(vehicle_trajectory_waypoint_desired),
						 &_traj_wp_avoidance_desired);
	}
}

void
MulticopterPositionControl::publish_attitude()
{
	_att_sp.timestamp = hrt_absolute_time();

	if (_att_sp_pub != nullptr) {
		orb_publish(_attitude_setpoint_id, _att_sp_pub, &_att_sp);

	} else if (_attitude_setpoint_id) {
		_att_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
	}
}

void
MulticopterPositionControl::publish_local_pos_sp(const vehicle_local_position_setpoint_s &local_pos_sp)
{
	// publish local position setpoint
	if (_local_pos_sp_pub != nullptr) {
		orb_publish(ORB_ID(vehicle_local_position_setpoint), _local_pos_sp_pub, &local_pos_sp);

	} else {
		_local_pos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &local_pos_sp);
	}
}

void MulticopterPositionControl::check_failure(bool task_failure, uint8_t nav_state)
{
	if (!task_failure) {
		// we want to be in this mode, reset the failure count
		_task_failure_count = 0;

	} else if (_task_failure_count > NUM_FAILURE_TRIES) {
		// tell commander to switch mode
		PX4_WARN("Previous flight task failed, switching to mode %d", nav_state);
		send_vehicle_cmd_do(nav_state);
		_task_failure_count = 0; // avoid immediate resending of a vehicle command in the next iteration
	}
}

void MulticopterPositionControl::send_vehicle_cmd_do(uint8_t nav_state)
{
	vehicle_command_s command{};
	command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	command.param1 = (float)1; // base mode
	command.param3 = (float)0; // sub mode
	command.target_system = 1;
	command.target_component = 1;
	command.source_system = 1;
	command.source_component = 1;
	command.confirmation = false;
	command.from_external = false;

	// set the main mode
	switch (nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_STAB:
		command.param2 = (float)PX4_CUSTOM_MAIN_MODE_STABILIZED;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		command.param2 = (float)PX4_CUSTOM_MAIN_MODE_ALTCTL;
		break;

	default: //vehicle_status_s::NAVIGATION_STATE_POSCTL
		command.param2 = (float)PX4_CUSTOM_MAIN_MODE_POSCTL;
		break;
	}

	// publish the vehicle command
	if (_pub_vehicle_command == nullptr) {
		_pub_vehicle_command = orb_advertise_queue(ORB_ID(vehicle_command), &command,
				       vehicle_command_s::ORB_QUEUE_LENGTH);

	} else {
		orb_publish(ORB_ID(vehicle_command), _pub_vehicle_command, &command);
	}
}

int MulticopterPositionControl::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("mc_pos_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_POSITION_CONTROL,
					   1900,
					   (px4_main_t)&run_trampoline,
					   (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

MulticopterPositionControl *MulticopterPositionControl::instantiate(int argc, char *argv[])
{
	return new MulticopterPositionControl();
}

int MulticopterPositionControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int mc_pos_control_main(int argc, char *argv[])
{
	return MulticopterPositionControl::main(argc, argv);
}
