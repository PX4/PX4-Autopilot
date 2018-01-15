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
 *
 * Original publication for the desired attitude generation:
 * Daniel Mellinger and Vijay Kumar. Minimum Snap Trajectory Generation and Control for Quadrotors.
 * Int. Conf. on Robotics and Automation, Shanghai, China, May 2011
 *
 * Also inspired by https://pixhawk.org/firmware/apps/fw_pos_control_l1
 *
 * The controller has two loops: P loop for position error and PID loop for velocity error.
 * Output of velocity controller is thrust vector that splitted to thrust direction
 * (i.e. rotation matrix for multicopter orientation) and thrust module (i.e. multicopter thrust itself).
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <drivers/drv_hrt.h>
#include <systemlib/hysteresis/hysteresis.h>

#include <uORB/topics/home_position.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_status.h>

#include <float.h>
#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>
#include <systemlib/mavlink_log.h>

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

using math::min;
using math::max;
using math::radians;

using control::BlockParamFloat;
using control::BlockParamInt;
using control::BlockDerivative;

#define SIGMA_SINGLE_OP			0.000001f
#define SIGMA_NORM			0.001f
/**
 * Multicopter position control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_pos_control_main(int argc, char *argv[]);

class MulticopterPositionControl : public control::SuperBlock
{
public:
	MulticopterPositionControl();
	virtual ~MulticopterPositionControl();

	int		start();

	bool		cross_sphere_line(const math::Vector<3> &sphere_c, const float sphere_r,
					  const math::Vector<3> &line_a, const math::Vector<3> &line_b, math::Vector<3> &res);

private:

	/** Time in us that direction change condition has to be true for direction change state */
	static constexpr uint64_t DIRECTION_CHANGE_TRIGGER_TIME_US = 100000;

	bool		_task_should_exit = false;			/**<true if task should exit */
	int		_control_task{-1};			/**< task handle for task */

	bool		_gear_state_initialized = false;		/**<true if the gear state has been initialized */
	bool 		_reset_pos_sp = true;  				/**<true if position setpoint needs a reset */
	bool 		_reset_alt_sp = true; 				/**<true if altitude setpoint needs a reset */
	bool 		_do_reset_alt_pos_flag = true; 			/**< TODO: check if we need this */
	bool		_mode_auto = false ;  				/**<true if in auot mode */
	bool 		_pos_hold_engaged = false; 			/**<true if hold positon in xy desired */
	bool 		_alt_hold_engaged = false; 			/**<true if hold in z desired */
	bool 		_run_pos_control = true;  			/**< true if position controller should be used */
	bool 		_run_alt_control = true; 			/**<true if altitude controller should be used */
	bool 		_reset_int_z = true; 				/**<true if reset integral in z */
	bool		_reset_int_xy = true; 				/**<true if reset integral in xy */
	bool		_reset_yaw_sp = true; 				/**<true if reset yaw setpoint */
	bool 		_hold_offboard_xy = false; 			/**<TODO : check if we need this extra hold_offboard flag */
	bool 		_hold_offboard_z = false;
	bool 		_in_smooth_takeoff = false; 			/**<true if takeoff ramp is applied */
	bool 		_in_landing = false;				/**<true if landing descent (only used in auto) */
	bool 		_lnd_reached_ground = false; 	/**<true if controller assumes the vehicle has reached the ground after landing */
	bool 		_triplet_lat_lon_finite = true; 		/**<true if triplets current is non-finite */

	int		_vehicle_status_sub{-1};		/**< vehicle status subscription */
	int		_vehicle_land_detected_sub{-1};		/**< vehicle land detected subscription */
	int		_vehicle_attitude_sub{-1};		/**< control state subscription */
	int		_control_mode_sub{-1};			/**< vehicle control mode subscription */
	int		_params_sub{-1};			/**< notification of parameter updates */
	int		_manual_sub{-1};			/**< notification of manual control updates */
	int		_local_pos_sub{-1};			/**< vehicle local position */
	int		_pos_sp_triplet_sub{-1};		/**< position setpoint triplet */
	int		_home_pos_sub{-1}; 			/**< home position */

	orb_advert_t	_att_sp_pub{nullptr};			/**< attitude setpoint publication */
	orb_advert_t	_local_pos_sp_pub{nullptr};		/**< vehicle local position setpoint publication */

	orb_id_t _attitude_setpoint_id;

	vehicle_status_s 			_vehicle_status{}; 		/**< vehicle status */
	vehicle_land_detected_s 		_vehicle_land_detected{};	/**< vehicle land detected */
	vehicle_attitude_s			_att{};				/**< vehicle attitude */
	vehicle_attitude_setpoint_s		_att_sp{};			/**< vehicle attitude setpoint */
	manual_control_setpoint_s		_manual{};			/**< r/c channel data */
	vehicle_control_mode_s			_control_mode{};		/**< vehicle control mode */
	vehicle_local_position_s		_local_pos{};			/**< vehicle local position */
	position_setpoint_triplet_s		_pos_sp_triplet{};		/**< vehicle global position setpoint triplet */
	vehicle_local_position_setpoint_s	_local_pos_sp{};		/**< vehicle local position setpoint */
	home_position_s				_home_pos{}; 			/**< home position */

	BlockParamFloat _manual_thr_min; /**< minimal throttle output when flying in manual mode */
	BlockParamFloat _manual_thr_max; /**< maximal throttle output when flying in manual mode */
	BlockParamFloat _xy_vel_man_expo; /**< ratio of exponential curve for stick input in xy direction pos mode */
	BlockParamFloat _z_vel_man_expo; /**< ratio of exponential curve for stick input in xy direction pos mode */
	BlockParamFloat _hold_dz; /**< deadzone around the center for the sticks when flying in position mode */
	BlockParamFloat _acceleration_hor_max; /**<maximum velocity setpoint slewrate for auto & fast manual brake */
	BlockParamFloat _acceleration_hor; /**<acceleration for auto and maximum for manual in velocity control mode*/
	BlockParamFloat _deceleration_hor_slow; /**< slow velocity setpoint slewrate for manual deceleration*/
	BlockParamFloat _acceleration_z_max_up; /** max acceleration up */
	BlockParamFloat _acceleration_z_max_down; /** max acceleration down */
	BlockParamFloat _cruise_speed_90; /**<speed when angle is 90 degrees between prev-current/current-next*/
	BlockParamFloat _velocity_hor_manual; /**< target velocity in manual controlled mode at full speed*/
	BlockParamFloat _takeoff_ramp_time; /**< time contant for smooth takeoff ramp */
	BlockParamFloat _jerk_hor_max; /**< maximum jerk in manual controlled mode when braking to zero */
	BlockParamFloat _jerk_hor_min; /**< minimum jerk in manual controlled mode when braking to zero */
	BlockParamFloat _thr_min;
	BlockParamFloat _thr_max;
	BlockParamFloat _thr_hover;
	BlockParamFloat _z_p;
	BlockParamFloat _z_vel_p;
	BlockParamFloat _z_vel_i;
	BlockParamFloat _z_vel_d;
	BlockParamFloat _z_vel_max_up;
	BlockParamFloat _z_vel_max_down;
	BlockParamFloat _slow_land_alt1;
	BlockParamFloat _slow_land_alt2;
	BlockParamFloat _xy_p;
	BlockParamFloat _xy_vel_p;
	BlockParamFloat _xy_vel_i;
	BlockParamFloat _xy_vel_d;
	BlockParamFloat _xy_vel_max;
	BlockParamFloat _xy_vel_cruise;
	BlockParamFloat _tilt_max_air;
	BlockParamFloat _land_speed;
	BlockParamFloat _tko_speed;
	BlockParamFloat _tilt_max_land;
	BlockParamFloat _man_tilt_max;
	BlockParamFloat _man_yaw_max;
	BlockParamFloat _hold_max_xy;
	BlockParamFloat _hold_max_z;
	BlockParamInt _alt_mode;

	// non-MPC params
	BlockParamFloat _mc_att_yaw_p;
	BlockParamFloat _global_yaw_max;
	BlockParamFloat _mis_yaw_error; /**< yaw error threshold that is used in mission as update criteria */
	BlockParamFloat _nav_rad; /**< radius that is used by navigator that defines when to update triplets */
	BlockParamFloat _rc_flt_smp_rate;
	BlockParamFloat _rc_flt_cutoff;

	BlockDerivative _vel_x_deriv;
	BlockDerivative _vel_y_deriv;
	BlockDerivative _vel_z_deriv;

	systemlib::Hysteresis _manual_direction_change_hysteresis{false};

	math::LowPassFilter2p _filter_manual_pitch{50.0f, 10.0f};
	math::LowPassFilter2p _filter_manual_roll{50.0f, 10.0f};

	enum manual_stick_input {
		brake,
		direction_change,
		acceleration,
		deceleration
	};

	manual_stick_input _user_intention_xy{brake}; /**< defines what the user intends to do derived from the stick input */
	manual_stick_input _user_intention_z{brake}; /**< defines what the user intends to do derived from the stick input in z direction */

	struct {
		param_t opt_recover;
	} _params_handles{};

	struct {
		bool opt_recover;

		math::Vector<3> pos_p;
		math::Vector<3> vel_p;
		math::Vector<3> vel_i;
		math::Vector<3> vel_d;
	} _params{};

	struct map_projection_reference_s _ref_pos {};
	float _ref_alt{0.0f};
	bool _ref_alt_is_global{false}; /** true when the reference altitude is defined in a global reference frame */
	hrt_abstime _ref_timestamp{0};
	hrt_abstime _last_warn{0};

	math::Vector<3> _thrust_int;
	math::Vector<3> _pos;
	math::Vector<3> _pos_sp;
	math::Vector<3> _vel;
	math::Vector<3> _vel_sp;
	math::Vector<3> _vel_prev;			/**< velocity on previous step */
	math::Vector<3> _vel_sp_prev;
	math::Vector<3> _vel_err_d;		/**< derivative of current velocity */
	math::Vector<3> _curr_pos_sp;  /**< current setpoint of the triplets */
	math::Vector<3> _prev_pos_sp; /**< previous setpoint of the triples */
	matrix::Vector2f _stick_input_xy_prev; /**< for manual controlled mode to detect direction change */

	math::Matrix<3, 3> _R;			/**< rotation matrix from attitude quaternions */
	float _yaw{0.0f};				/**< yaw angle (euler) */
	float _yaw_takeoff{0.0f};	/**< home yaw angle present when vehicle was taking off (euler) */
	float _man_yaw_offset{0.0f}; /**< current yaw offset in manual mode */

	float _vel_max_xy{0.0f};  /**< equal to vel_max except in auto mode when close to target */
	float _acceleration_state_dependent_xy{0.0f}; /**< acceleration limit applied in manual mode */
	float _acceleration_state_dependent_z{0.0f}; /**< acceleration limit applied in manual mode in z */
	float _manual_jerk_limit_xy{1.0f}; /**< jerk limit in manual mode dependent on stick input */
	float _manual_jerk_limit_z{1.0f}; /**< jerk limit in manual mode in z */
	float _z_derivative{0.0f};; /**< velocity in z that agrees with position rate */

	float _takeoff_vel_limit{0.0f}; /**< velocity limit value which gets ramped up */

	// counters for reset events on position and velocity states
	// they are used to identify a reset event
	uint8_t _z_reset_counter{0};
	uint8_t _xy_reset_counter{0};
	uint8_t _heading_reset_counter{0};

	matrix::Dcmf _R_setpoint;

	/**
	 * Update our local parameter cache.
	 */
	void		parameters_update(bool force);

	/**
	 * Check for changes in subscribed topics.
	 */
	void		poll_subscriptions();

	float		throttle_curve(float ctl, float ctr);

	/**
	 * Update reference for local position projection
	 */
	void		update_ref();

	/**
	 * Reset position setpoint to current position.
	 *
	 * This reset will only occur if the _reset_pos_sp flag has been set.
	 * The general logic is to first "activate" the flag in the flight
	 * regime where a switch to a position control mode should hold the
	 * very last position. Once switching to a position control mode
	 * the last position is stored once.
	 */
	void		reset_pos_sp();

	/**
	 * Reset altitude setpoint to current altitude.
	 *
	 * This reset will only occur if the _reset_alt_sp flag has been set.
	 * The general logic follows the reset_pos_sp() architecture.
	 */
	void		reset_alt_sp();

	/**
	 * Set position setpoint using manual control
	 */
	void		control_manual();

	void		control_non_manual();

	/**
	 * Set position setpoint using offboard control
	 */
	void		control_offboard();

	/**
	 * Set position setpoint for AUTO
	 */
	void		control_auto();

	void control_position();
	void calculate_velocity_setpoint();
	void calculate_thrust_setpoint();

	void vel_sp_slewrate();

	void update_velocity_derivative();

	void do_control();

	void generate_attitude_setpoint();

	float get_cruising_speed_xy();

	bool in_auto_takeoff();

	float get_vel_close(const matrix::Vector2f &unit_prev_to_current, const matrix::Vector2f &unit_current_to_next);

	void set_manual_acceleration_xy(matrix::Vector2f &stick_input_xy_NED);

	void set_manual_acceleration_z(float &max_acc_z, const float stick_input_z_NED);

	/**
	 * limit altitude based on several conditions
	 */
	void limit_altitude();

	void warn_rate_limited(const char *str);

	bool manual_wants_takeoff();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main();
};
