/****************************************************************************
 *
 *   Copyright (c) 2013 - 2016 PX4 Development Team. All rights reserved.
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
 * @file mc_quat_pos_control_main.cpp
 * Multicopter quaternion position controller.
 *
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * @author Moses Bangura <moses.bangura@anu.edu.au, dnovichman@hotmail.com>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4.h>
#include <stdio.h>
#include <cstdio>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <functional>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>

#include <uORB/topics/parameter_update.h>

#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/mc_virtual_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_land_detected.h>

#include <systemlib/systemlib.h>
#include <systemlib/mavlink_log.h>
#include <platforms/px4_defines.h>

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#include <px4_eigen.h>
#include <uORB/topics/home_position.h>

#include <uORB/topics/vehicle_velocity_est_inertial.h>
#include <uORB/topics/vehicle_velocity_meas_inertial.h>
#include <uORB/topics/vehicle_velocity_meas_est_body.h>  
#include <av_estimator/include/av_estimator_params.h> 
//#include <av_estimator/include/av_estimator_main.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_omega_ff_setpoint.h>

#define TILT_COS_MAX	0.7f
#define SIGMA			0.000001f
#define MIN_DIST		0.01f
#define MANUAL_THROTTLE_MAX_MULTICOPTER	0.9f
#define ONE_G	9.8066f

#define acc_grav  9.81f

using namespace px4;
using namespace Eigen;

/**
 * Multicopter position control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_quat_pos_control_main(int argc, char *argv[]);

class MulticopterQuatPositionControl : public control::SuperBlock
{
public:
	/**
	 * Constructor
	 */
	MulticopterQuatPositionControl();

	/**
	 * Destructor, also kills task.
	 */
	~MulticopterQuatPositionControl();

	/**
	 * Start task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:
	bool		_task_should_exit;		/**< if true, task should exit */
	int		_control_task;			/**< task handle for task */
	orb_advert_t	_mavlink_log_pub;		/**< mavlink log advert */

	int		_vehicle_status_sub;		/**< vehicle status subscription */
	int		_vehicle_land_detected_sub;	/**< vehicle land detected subscription */
	int		_ctrl_state_sub;		/**< control state subscription */
	int		_att_sp_sub;			/**< vehicle attitude setpoint */
	int		_control_mode_sub;		/**< vehicle control mode subscription */
	int		_params_sub;			/**< notification of parameter updates */
	int		_manual_sub;			/**< notification of manual control updates */
	int		_arming_sub;			/**< arming status of outputs */
	int		_local_pos_sub;			/**< vehicle local position */
	int		_pos_sp_triplet_sub;		/**< position setpoint triplet */
	int		_local_pos_sp_sub;		/**< offboard local position setpoint */
	int		_global_vel_sp_sub;		/**< offboard global velocity setpoint */

	orb_advert_t	_att_sp_pub;			/**< attitude setpoint publication */
	orb_advert_t	_local_pos_sp_pub;		/**< vehicle local position setpoint publication */
	orb_advert_t	_global_vel_sp_pub;		/**< vehicle global velocity setpoint publication */

	orb_id_t _attitude_setpoint_id;

	int _att_sub;

	struct vehicle_status_s 			_vehicle_status; 	/**< vehicle status */
	struct vehicle_land_detected_s 			_vehicle_land_detected;	/**< vehicle land detected */
	struct control_state_s				_ctrl_state;		/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s		_att_sp;		/**< vehicle attitude setpoint */
	struct manual_control_setpoint_s		_manual;		/**< r/c channel data */
	struct vehicle_control_mode_s			_control_mode;		/**< vehicle control mode */
	struct actuator_armed_s				_arming;		/**< actuator arming status */
	struct vehicle_local_position_s			_local_pos;		/**< vehicle local position */
	struct position_setpoint_triplet_s		_pos_sp_triplet;	/**< vehicle global position setpoint triplet */
	struct vehicle_local_position_setpoint_s	_local_pos_sp;		/**< vehicle local position setpoint */
	struct vehicle_global_velocity_setpoint_s	_global_vel_sp;		/**< vehicle global velocity setpoint */

	control::BlockParamFloat _manual_thr_min;
	control::BlockParamFloat _manual_thr_max;

	control::BlockDerivative _vel_x_deriv;
	control::BlockDerivative _vel_y_deriv;
	control::BlockDerivative _vel_z_deriv;

	struct {
		param_t thr_min;
		param_t thr_max;
		param_t thr_hover;
		param_t alt_ctl_dz;
		param_t alt_ctl_dy;
		param_t z_p;
		param_t z_vel_p;
		param_t z_vel_i;
		param_t z_vel_d;
		param_t z_vel_max_up;
		param_t z_vel_max_down;
		param_t z_ff;
		param_t mg;
		param_t xy_p;
		param_t xy_vel_p;
		param_t xy_vel_i;
		param_t xy_vel_d;
		param_t xy_vel_max;
		param_t xy_vel_cruise;
		param_t xy_ff;
		param_t tilt_max_air;
		param_t land_speed;
		param_t tko_speed;
		param_t tilt_max_land;
		param_t man_roll_max;
		param_t man_pitch_max;
		param_t man_yaw_max;
		param_t global_yaw_max;
		param_t mc_att_yaw_p;
		param_t hold_xy_dz;
		param_t hold_max_xy;
		param_t hold_max_z;
		param_t acc_hor_max;
		param_t alt_mode;

	}		_params_handles;		/**< handles for interesting parameters */

	struct {
		float thr_min;
		float thr_max;
		float thr_hover;
		float alt_ctl_dz;
		float alt_ctl_dy;
		float tilt_max_air;
		float land_speed;
		float tko_speed;
		float tilt_max_land;
		float man_roll_max;
		float man_pitch_max;
		float man_yaw_max;
		float global_yaw_max;
		float mc_att_yaw_p;
		float hold_xy_dz;
		float hold_max_xy;
		float hold_max_z;
		float acc_hor_max;
		float vel_max_up;
		float vel_max_down;
		uint32_t alt_mode;
		float mg;
		math::Vector<3> pos_p;
		math::Vector<3> vel_p;
		math::Vector<3> vel_i;
		math::Vector<3> vel_d;
		math::Vector<3> vel_ff;
		math::Vector<3> vel_max;
		math::Vector<3> vel_cruise;
		math::Vector<3> sp_offs_max;
	}		_params;

	Vector3f pos_d = Vector3f::Zero(), vel_d = Vector3f::Zero(), acc_d = Vector3f::Zero();
	
	/* Initialisation of time and filters */
	float 		x, y, z, vx = 0.0f, vy = 0.0f, vz = 0.0f;
	float 		ivx = 0.0f, ivy = 0.0f, ivz = 0.0f;
	float 		px_tmp = 0.0f, py_tmp = 0.0f, pz_tmp = 0.0f; 
	float 		qd[4], thrust;
	int 		sign_q0 = 1;
	uint64_t 	old_time = 0;
	uint64_t 	gps_old_time = 0, _v_est_time_old = 0;
	float 		dtt = 0.02, dt = 0.018182;

	Vector3f    gps_body_pos_hat = Vector3f::Zero();
	Vector3f    gps_body_pos_hat_dot = Vector3f::Zero();

	//math::Matrix<3,3> Rot;
	//math::Vector<3>	   veh_vel_body = Vector3f::Zero();
	Matrix3f 	Rot 		= Matrix3f::Identity();
	Vector3f    veh_vel_body = Vector3f::Zero();
	Matrix3f	Omega_x;

	Matrix3f pos_filter_gains = Matrix3f::Identity();
	
	bool 		old_auto = false, vel_updated = false;
	float 		des_yaw = 0.0f;

	uint64_t 	old_meas_time;
	float 		d_meas, home_lat = 0.0f, home_lon = 0.0f, home_alt = 0.0f;
	float 		dt_pos_est;

	bool 		initialise_home = false; 					/**< GPS home initialisation based on 20 gps coordinates */
	int 		home_count 	= 0;  							/**< counter for home position in gps mode */
	bool 		vel_pub = false;

	orb_advert_t home_position_pub; //int 		home_position_pub 			= -1;
	orb_advert_t _local_pos_pub;
	struct 		vehicle_local_position_s 	veh_local_pos;
	struct 		home_position_s 			_veh_home;
	//struct 	vehicle_local_position_s 		_local_pos_pub ;//				= -1;

	struct 		vehicle_velocity_est_inertial_s		_v_vel_est;			/**< vehicle velocity estimated */
	int         _v_vel_est_sub = -1;			/**< Subscribe to vehicle est velo */
	struct vehicle_attitude_s _att;				    /**< vehicle attitude */
	struct vehicle_vicon_position_s 			_vicon_pos;   			/**< vicon position */
	struct vehicle_gps_position_s 			_veh_gps_pos;			/**< Subscribe to gps position */
	struct vehicle_velocity_meas_inertial_s			_veh_vel;			/**< vehicle velocity */
	struct vehicle_velocity_meas_est_body_s			_veh_vel_body;  	/**< Body fixed frame velocity measurements */

	int _veh_vel_body_sub = -1;
	orb_advert_t _veh_vel_pub;

	/* subscribe to raw data */
	int 		sub_raw 			= -1; 
	struct 		sensor_combined_s 	raw;

	orb_advert_t 		_omega_pub;// = -1;
	struct 		vehicle_omega_ff_setpoint_s 	omega_d;

	int _veh_gps_pos_sub = -1;
	int _vicon_pos_sub = -1;


	struct map_projection_reference_s _ref_pos;
	float _ref_alt;
	uint64_t _ref_timestamp;

	bool _reset_pos_sp;
	bool _reset_alt_sp;
	bool _mode_auto;
	bool _pos_hold_engaged;
	bool _alt_hold_engaged;
	bool _run_quat_pos_control;
	bool _run_alt_control;

	math::Vector<3> _pos;
	math::Vector<3> _pos_sp;
	math::Vector<3> _vel;
	math::Vector<3> _vel_sp;
	math::Vector<3> _vel_prev;			/**< velocity on previous step */
	math::Vector<3> _vel_ff;
	math::Vector<3> _vel_sp_prev;
	math::Vector<3> _thrust_sp_prev;
	math::Vector<3> _vel_err_d;		/**< derivative of current velocity */

	math::Matrix<3, 3> _R;			/**< rotation matrix from attitude quaternions */
	float _yaw;				/**< yaw angle (euler) */
	bool _in_landing;	/**< the vehicle is in the landing descent */
	bool _lnd_reached_ground; /**< controller assumes the vehicle has reached the ground after landing */
	bool _takeoff_jumped;
	float _vel_z_lp;
	float _acc_z_lp;
	float _takeoff_thrust_sp;
	bool control_vel_enabled_prev;	/**< previous loop was in velocity controlled mode (control_state.flag_control_velocity_enabled) */

	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update(bool force);

	/**
	 * Update control outputs
	 */
	void		control_update();

	/**
	 * Check for changes in subscribed topics.
	 */
	void		poll_subscriptions();

	static float	scale_control(float ctl, float end, float dz, float dy);
	static float    throttle_curve(float ctl, float ctr);

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
	 * Check if position setpoint is too far from current position and adjust it if needed.
	 */
	void		limit_pos_sp_offset();

	/**
	 * Set position setpoint using manual control
	 */
	void		control_manual(float dt);

	/**
	 * Set position setpoint using offboard control
	 */
	void		control_offboard(float dt);

	bool		cross_sphere_line(const math::Vector<3> &sphere_c, float sphere_r,
					  const math::Vector<3> line_a, const math::Vector<3> line_b, math::Vector<3> &res);

	/**
	 * Set position setpoint for AUTO
	 */
	void		control_auto(float dt);

	/**
	 * Select between barometric and global (AMSL) altitudes
	 */
	void		select_alt(bool global);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main();

	/**
	 * Update our local parameter cache.
	 */
	int			parameters_update();

	
	/** 
	  * Functions to do control >*/
	//void			quat_pos_control(); 

	void			quat_traj_control();

	/** 
	  * Operate on desired position
	*/
	void  			desired_setpoint();
	
	/**
	  * Do rotations
	*/
	void 			dorotations();

	/**
	 * Check for changes in vehicle estimated velocity
	*/
	void 		vehicle_velocity_estimated_poll();

	/**
	 * Check for changes in vehicle body fixed frame velocities (est and meas)
	*/
	void 		vehicle_velocity_body_poll();

	/**
	 * Check for changes in vehicle GPS data
	*/
	void 		vehicle_gps_position_poll();

	/**
	  * Check for changes in vicon position 
	*/
	void        vehicle_vicon_position_poll();

	/**
	  * Check for changes in vehicle local position setpoint 
	*/
	void        vehicle_local_position_setpoint_poll();

	/**
	  * Check for changes in vehicle vehicle control mode 
	*/
	void        vehicle_control_mode_poll();

	/**
	 * Check for changes in vehicle attitude
	*/
	void 		vehicle_attitude_poll();

	/**
	  * Poll vehicle IMU for feedforward term
	*/
	void 		vehicle_imu_poll();

	/**
	  * Fast inverse implementation of squareroot
	*/
	float 			invSqrt(float number);

	float 			vector_dot_product(math::Vector<3> &Vec1, math::Vector<3> &Vec2);

	math::Vector<3> quat_to_Euler(float q0, float q1, float q2, float q3);						/**< quaternion to Euler, now obsolete */

	void 			getQuatFromRPY123(const float roll, const float pitch, const float yaw, float* quat); /**< Extract quaternion from RPM now obsolete */

	void 			quat_mult(const float *a,const float *b,float *prod);

	math::Vector<3>	map_projection(void); //home_position_s &ref, px4_vehicle_gps_position &gps_pos

	void 			gps_vel_setup(void); //home_position_s &ref, px4_vehicle_gps_position &gps_pos
	/**< End functions */
};

namespace quat_pos_control
{

MulticopterQuatPositionControl	*g_control;
}

MulticopterQuatPositionControl::MulticopterQuatPositionControl() :
	SuperBlock(NULL, "MPC"),
	_task_should_exit(false),
	_control_task(-1),
	_mavlink_log_pub(nullptr),

	/* subscriptions */
	_ctrl_state_sub(-1),
	_att_sp_sub(-1),
	_control_mode_sub(-1),
	_params_sub(-1),
	_manual_sub(-1),
	_arming_sub(-1),
	_local_pos_sub(-1),
	_pos_sp_triplet_sub(-1),
	_global_vel_sp_sub(-1),

	/* publications */
	_att_sp_pub(nullptr),
	_local_pos_sp_pub(nullptr),
	_global_vel_sp_pub(nullptr),
	_attitude_setpoint_id(0),
	_vehicle_status{},
	_vehicle_land_detected{},
	_ctrl_state{},
	_att_sp{},
	_manual{},
	_control_mode{},
	_arming{},
	_local_pos{},
	_pos_sp_triplet{},
	_local_pos_sp{},
	_global_vel_sp{},
	_manual_thr_min(this, "MANTHR_MIN"),
	_manual_thr_max(this, "MANTHR_MAX"),
	_vel_x_deriv(this, "VELD"),
	_vel_y_deriv(this, "VELD"),
	_vel_z_deriv(this, "VELD"),
	_ref_alt(0.0f),
	_ref_timestamp(0),

	_reset_pos_sp(true),
	_reset_alt_sp(true),
	_mode_auto(false),
	_pos_hold_engaged(false),
	_alt_hold_engaged(false),
	_run_quat_pos_control(true),
	_run_alt_control(true),
	_yaw(0.0f),
	_in_landing(false),
	_lnd_reached_ground(false),
	_takeoff_jumped(false),
	_vel_z_lp(0),
	_acc_z_lp(0),
	_takeoff_thrust_sp(0.0f),
	control_vel_enabled_prev(false)
{
	// Make the quaternion valid for control state
	_ctrl_state.q[0] = 1.0f;

	memset(&_ref_pos, 0, sizeof(_ref_pos));

	_params.pos_p.zero();
	_params.vel_p.zero();
	_params.vel_i.zero();
	_params.vel_d.zero();
	_params.vel_max.zero();
	_params.vel_cruise.zero();
	_params.vel_ff.zero();
	_params.sp_offs_max.zero();

	_pos.zero();
	_pos_sp.zero();
	_vel.zero();
	_vel_sp.zero();
	_vel_prev.zero();
	_vel_ff.zero();
	_vel_sp_prev.zero();
	_vel_err_d.zero();

	_R.identity();

	_params_handles.thr_min		= param_find("MPC_THR_MIN");
	_params_handles.thr_max		= param_find("MPC_THR_MAX");
	_params_handles.thr_hover	= param_find("MPC_THR_HOVER");
	_params_handles.alt_ctl_dz	= param_find("MPC_ALTCTL_DZ");
	_params_handles.alt_ctl_dy	= param_find("MPC_ALTCTL_DY");
	_params_handles.z_p		= param_find("MPC_Z_P");
	_params_handles.z_vel_p		= param_find("MPC_Z_VEL_P");
	_params_handles.z_vel_i		= param_find("MPC_Z_VEL_I");
	_params_handles.z_vel_d		= param_find("MPC_Z_VEL_D");
	_params_handles.z_vel_max_up	= param_find("MPC_Z_VEL_MAX_UP");
	_params_handles.z_vel_max_down	= param_find("MPC_Z_VEL_MAX");
	_params_handles.mg	= param_find("MPC_MG");

	// transitional support: Copy param values from max to down
	// param so that max param can be renamed in 1-2 releases
	// (currently at 1.3.0)
	float p;
	param_get(param_find("MPC_Z_VEL_MAX"), &p);
	param_set(param_find("MPC_Z_VEL_MAX_DN"), &p);

	_params_handles.z_ff		= param_find("MPC_Z_FF");
	_params_handles.xy_p		= param_find("MPC_XY_P");
	_params_handles.xy_vel_p	= param_find("MPC_XY_VEL_P");
	_params_handles.xy_vel_i	= param_find("MPC_XY_VEL_I");
	_params_handles.xy_vel_d	= param_find("MPC_XY_VEL_D");
	_params_handles.xy_vel_max	= param_find("MPC_XY_VEL_MAX");
	_params_handles.xy_vel_cruise	= param_find("MPC_XY_CRUISE");
	_params_handles.xy_ff		= param_find("MPC_XY_FF");
	_params_handles.tilt_max_air	= param_find("MPC_TILTMAX_AIR");
	_params_handles.land_speed	= param_find("MPC_LAND_SPEED");
	_params_handles.tko_speed	= param_find("MPC_TKO_SPEED");
	_params_handles.tilt_max_land	= param_find("MPC_TILTMAX_LND");
	_params_handles.man_roll_max = param_find("MPC_MAN_R_MAX");
	_params_handles.man_pitch_max = param_find("MPC_MAN_P_MAX");
	_params_handles.man_yaw_max = param_find("MPC_MAN_Y_MAX");
	_params_handles.global_yaw_max = param_find("MC_YAWRATE_MAX");
	_params_handles.mc_att_yaw_p = param_find("MC_YAW_P");
	_params_handles.hold_xy_dz = param_find("MPC_HOLD_XY_DZ");
	_params_handles.hold_max_xy = param_find("MPC_HOLD_MAX_XY");
	_params_handles.hold_max_z = param_find("MPC_HOLD_MAX_Z");
	_params_handles.acc_hor_max = param_find("MPC_ACC_HOR_MAX");
	_params_handles.alt_mode = param_find("MPC_ALT_MODE");

	/* fetch initial parameter values */
	parameters_update(true);
}

MulticopterQuatPositionControl::~MulticopterQuatPositionControl()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	quat_pos_control::g_control = nullptr;
}

int
MulticopterQuatPositionControl::parameters_update(bool force)
{
	bool updated = false;
	struct parameter_update_s param_upd;

	if (updated) {
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_upd);
	}

	if (updated || force) {
		/* update C++ param system */
		updateParams();

		/* update legacy C interface params */
		param_get(_params_handles.thr_min, &_params.thr_min);
		param_get(_params_handles.thr_max, &_params.thr_max);
		param_get(_params_handles.thr_hover, &_params.thr_hover);
		param_get(_params_handles.alt_ctl_dz, &_params.alt_ctl_dz);
		param_get(_params_handles.alt_ctl_dy, &_params.alt_ctl_dy);
		param_get(_params_handles.tilt_max_air, &_params.tilt_max_air);
		_params.tilt_max_air = math::radians(_params.tilt_max_air);
		param_get(_params_handles.land_speed, &_params.land_speed);
		param_get(_params_handles.tko_speed, &_params.tko_speed);
		param_get(_params_handles.tilt_max_land, &_params.tilt_max_land);
		_params.tilt_max_land = math::radians(_params.tilt_max_land);

		float v;
		uint32_t v_i;
		param_get(_params_handles.xy_p, &v);
		_params.pos_p(0) = v;
		_params.pos_p(1) = v;
		param_get(_params_handles.z_p, &v);
		_params.pos_p(2) = v;
		param_get(_params_handles.xy_vel_p, &v);
		_params.vel_p(0) = v;
		_params.vel_p(1) = v;
		param_get(_params_handles.z_vel_p, &v);
		_params.vel_p(2) = v;
		param_get(_params_handles.xy_vel_i, &v);
		_params.vel_i(0) = v;
		_params.vel_i(1) = v;
		param_get(_params_handles.z_vel_i, &v);
		_params.vel_i(2) = v;
		param_get(_params_handles.xy_vel_d, &v);
		_params.vel_d(0) = v;
		_params.vel_d(1) = v;
		param_get(_params_handles.z_vel_d, &v);
		_params.vel_d(2) = v;
		param_get(_params_handles.xy_vel_max, &v);
		_params.vel_max(0) = v;
		_params.vel_max(1) = v;
		param_get(_params_handles.z_vel_max_up, &v);
		_params.vel_max_up = v;
		_params.vel_max(2) = v;
		param_get(_params_handles.z_vel_max_down, &v);
		_params.vel_max_down = v;
		param_get(_params_handles.xy_vel_cruise, &v);
		_params.vel_cruise(0) = v;
		_params.vel_cruise(1) = v;
		/* using Z max up for now */
		param_get(_params_handles.z_vel_max_up, &v);
		_params.vel_cruise(2) = v;
		param_get(_params_handles.xy_ff, &v);
		v = math::constrain(v, 0.0f, 1.0f);
		_params.vel_ff(0) = v;
		_params.vel_ff(1) = v;
		param_get(_params_handles.z_ff, &v);
		v = math::constrain(v, 0.0f, 1.0f);
		_params.vel_ff(2) = v;
		param_get(_params_handles.hold_xy_dz, &v);
		v = math::constrain(v, 0.0f, 1.0f);
		_params.hold_xy_dz = v;
		param_get(_params_handles.hold_max_xy, &v);
		_params.hold_max_xy = (v < 0.0f ? 0.0f : v);
		param_get(_params_handles.hold_max_z, &v);
		_params.hold_max_z = (v < 0.0f ? 0.0f : v);
		param_get(_params_handles.acc_hor_max, &v);
		_params.acc_hor_max = v;
		param_get(_params_handles.alt_mode, &v_i);
		_params.alt_mode = v_i;

		_params.sp_offs_max = _params.vel_cruise.edivide(_params.pos_p) * 2.0f;

		/* mc attitude control parameters*/
		/* manual control scale */
		param_get(_params_handles.man_roll_max, &_params.man_roll_max);
		param_get(_params_handles.man_pitch_max, &_params.man_pitch_max);
		param_get(_params_handles.man_yaw_max, &_params.man_yaw_max);
		param_get(_params_handles.global_yaw_max, &_params.global_yaw_max);
		_params.man_roll_max = math::radians(_params.man_roll_max);
		_params.man_pitch_max = math::radians(_params.man_pitch_max);
		_params.man_yaw_max = math::radians(_params.man_yaw_max);
		_params.global_yaw_max = math::radians(_params.global_yaw_max);

		param_get(_params_handles.mc_att_yaw_p, &v);
		_params.mc_att_yaw_p = v;

		/* takeoff and land velocities should not exceed maximum */
		_params.tko_speed = fminf(_params.tko_speed, _params.vel_max_up);
		_params.land_speed = fminf(_params.land_speed, _params.vel_max_down);
	}

	return OK;
}

void
MulticopterQuatPositionControl::task_main_trampoline(int argc, char *argv[])
{
	quat_pos_control::g_control->task_main();
}

/**
  * Poll estimated velocity for GPS
*/
void
MulticopterQuatPositionControl::vehicle_velocity_estimated_poll()
{
	/* check if there is a new velocity estimated */
	bool updated;
	orb_check(_v_vel_est_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_velocity_est_inertial), _v_vel_est_sub, &_v_vel_est);
	}
}

/**
  * Poll estimated velocity for GPS
*/
void
MulticopterQuatPositionControl::vehicle_velocity_body_poll()
{
	/* check if there is a new velocity */
	bool updated;
	orb_check(_v_vel_est_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_velocity_meas_est_body), _veh_vel_body_sub, &_veh_vel_body);
	}
}

/**
  * Poll for local position setpoint
*/
void
MulticopterQuatPositionControl::vehicle_local_position_setpoint_poll()
{
	bool updated;
	orb_check(_local_pos_sp_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position_setpoint), _local_pos_sp_sub, &_local_pos_sp);
	}
}

/**
  * Poll for getting vicon position estimates
*/
void
MulticopterQuatPositionControl::vehicle_vicon_position_poll()
{
	bool updated;
	orb_check(_vicon_pos_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_vicon_position), _vicon_pos_sub, &_vicon_pos);
	}
}

/**
  * Poll for getting GPS data
*/
void
MulticopterQuatPositionControl::vehicle_gps_position_poll()
{
	/* check if there is a new velocity estimated */
	bool updated;
	orb_check(_veh_gps_pos_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_gps_position), _veh_gps_pos_sub, &_veh_gps_pos);
		//printf("sat %u %d %d\n",_veh_gps_pos.timestamp_time,_veh_gps_pos.fix_type, _veh_gps_pos.satellites_used);
	}
}

/**
  * Poll for getting vehicle attitude
*/
void
MulticopterQuatPositionControl::vehicle_attitude_poll()
{
	/* check if there is a new velocity estimated */
	bool updated;
	orb_check(_att_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_att);
	}
}

/**
  * Poll for vehicle control mode
*/
void
MulticopterQuatPositionControl::vehicle_control_mode_poll()
{
	/* check if there is a new velocity estimated */
	bool updated;
	orb_check(_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
	}
}


/**
  * Do polling
*/
void
MulticopterQuatPositionControl::poll_subscriptions()
{
	/* poll estimated velocity for gps pos control mode */
	vehicle_velocity_estimated_poll();

	/* Poll GPS data */
	vehicle_gps_position_poll();

	/* Poll IMU for feedforward */
	vehicle_imu_poll();

	/* Poll vehicle attitude */
	vehicle_attitude_poll();

	/* Poll vicon position */
	vehicle_vicon_position_poll();

	/* Poll vehicle position setpoint */
	vehicle_local_position_setpoint_poll();

	/* Poll vehicle control mode */
	vehicle_control_mode_poll();

	/* Poll vehicle body frame velocity */
	vehicle_velocity_body_poll();
}

/**
  * Poll vehicle IMU for feedforward
*/
void
MulticopterQuatPositionControl::vehicle_imu_poll()
{
	/* You need to obtain vehicle IMU data */
	bool updated;
	orb_check(sub_raw , &updated);
	if (updated) {
		orb_copy(ORB_ID(sensor_combined), sub_raw, &raw);
	}
}

/**
  * Inverse square root efficient computation
*/
float 
MulticopterQuatPositionControl::invSqrt(float number) {
    volatile long i;
    volatile float xi, yi;
    volatile const float f = 1.5F;

    xi  = number * 0.5F;
    yi  = number;
    i   = * (( long * ) &yi);
    i   = 0x5f375a86 - ( i >> 1 );
    yi  = * (( float * ) &i);
    yi  = yi * ( f - ( xi * yi * yi ) );
    return yi;
}

/**
  * Vector dot product
*/
float 
MulticopterQuatPositionControl::vector_dot_product(math::Vector<3> &Vec1, math::Vector<3> &Vec2)
{
	float output = 0.0f;

	for (int i=0; i<3; i++)
	{
		output += Vec1(i) * Vec2(i); 
	}
	return output;
}

/** 
  * Quaternion to Euler, now obsolete
*/
math::Vector<3> 
MulticopterQuatPositionControl::quat_to_Euler(float q0, float q1, float q2, float q3)
{
	math::Vector<3> output;
	output(0) = atan2(2 * (q0 * q1 + q2 * q3),(1 - 2*(q1 * q1 + q2 * q2))); 	//phi roll
	output(1) = asin(2 * (q0 * q2 - q3 * q1));			      			// theta pitch
	output(2) = atan2(2*(q0 * q3 + q1 * q2),(1 -2 * (q2 * q2 + q3 * q3)));  	//psi yaw
	return output;
}

/**
  * Quaternion extraction from Euler using roll-pitch-yaw standard
*/
void 
MulticopterQuatPositionControl::getQuatFromRPY123(const float roll, const float pitch, const float yaw, float* quat)
{
	float c_phi, c_theta, c_psi;
	float s_phi, s_theta, s_psi;
	float qu[4] = {1.0f, 0.0f, 0.0f, 0.0f};

	/* Precompute frequently used variables. */
	c_phi 	= cosf(0.5f * roll);
	c_theta = cosf(0.5f * pitch);
	c_psi 	= cosf(0.5f * yaw);

	s_phi 	= sinf(0.5f * roll);
	s_theta = sinf(0.5f * pitch);
	s_psi 	= sinf(0.5f * yaw);

	/* Convert RPY to Quaternion by 1-2-3 representation. See equation (297) of James Diebel paper. */	
	qu[0] = c_phi  * c_theta * c_psi   + s_phi   * s_theta * s_psi;
	qu[1] = -c_phi * s_theta * s_psi   + c_theta * c_psi   * s_phi;
	qu[2] = c_phi  * c_psi   * s_theta + s_phi   * c_theta * s_psi;
	qu[3] = c_phi  * c_theta * s_psi   - s_phi   * c_psi   * s_theta;

	float recipNorm = invSqrt(qu[0] * qu[0] + qu[1] * qu[1] + qu[2] * qu[2] + qu[3] * qu[3]);

	/* Normalization */
	quat[0] = qu[0]*recipNorm;
	quat[1] = qu[1]*recipNorm;
	quat[2] = qu[2]*recipNorm;
	quat[3] = qu[3]*recipNorm;
}

/**
  * Quaternion products
*/
void 
MulticopterQuatPositionControl::quat_mult(const float *a, const float *b, float *prod)
{
	prod[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    prod[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    prod[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    prod[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
	/* Do we need to normalise */
}

/**
   * GPS position control 
*/
void 
MulticopterQuatPositionControl::gps_vel_setup(void) //home_position_s &home, px4_vehicle_gps_position &_gps_pos
{
	math::Vector<3> x_gps;
	math::Vector<3> vel_i;	
/*
	Rot(0,0) = _att.R[0]; 
	Rot(0,1) = _att.R[1]; 
	Rot(0,2) = _att.R[2];     

	Rot(1,0) = _att.R[3]; 
	Rot(1,1) = _att.R[4]; 
	Rot(1,2) = _att.R[5];		    

	Rot(2,0) = _att.R[6]; 
	Rot(2,1) = _att.R[7]; 
	Rot(2,2) = _att.R[8]; 	
	*/
	
	float roll, pitch, yaw;
	float ctheta, stheta, cphi, sphi, cpsi, spsi;
	roll 	= _att.roll;	
	pitch 	= _att.pitch;	
	yaw 	= _att.yaw;	

	ctheta = cos(pitch);
	stheta = sin(pitch);
	cphi = cos(-roll);
	sphi = sin(-roll);
	cpsi = cos(-yaw);
	spsi = sin(-yaw);
	Rot(0,0) = ctheta*cpsi; 
	Rot(0,1) = sphi*stheta*cpsi-cphi*spsi; 
	Rot(0,2) = cphi*stheta*cpsi+sphi*spsi;     

	Rot(1,0) = ctheta*spsi; 
	Rot(1,1) = sphi*stheta*spsi+cphi*cpsi; 
	Rot(1,2) = cphi*stheta*spsi-sphi*cpsi;		    

	Rot(2,0) = -stheta; 
	Rot(2,1) = ctheta*sphi; 
	Rot(2,2) = ctheta*cphi; 
	
	 
	Rot.transpose();
	
	if (initialise_home)
	{
		x_gps 	 = map_projection();
	}
	
	vel_i(0) = _veh_gps_pos.vel_n_m_s;
	vel_i(1) = _veh_gps_pos.vel_e_m_s;
	vel_i(2) = -_veh_gps_pos.vel_d_m_s;

	ivx = vel_i(0);
	ivy = vel_i(1);
	ivz = vel_i(2);

	x = x_gps(0)*Rot(0,0) + x_gps(1)*Rot(0,1) + x_gps(2)*Rot(0,2); 
	y = x_gps(0)*Rot(1,0) + x_gps(1)*Rot(1,1) + x_gps(2)*Rot(1,2); 
	z = x_gps(0)*Rot(2,0) + x_gps(1)*Rot(2,1) + x_gps(2)*Rot(2,2); 
	
	vx = vel_i(0)*Rot(0,0) + vel_i(1)*Rot(0,1) + vel_i(2)*Rot(0,2); 
	vy = vel_i(0)*Rot(1,0) + vel_i(1)*Rot(1,1) + vel_i(2)*Rot(1,2); 
	vz = vel_i(0)*Rot(2,0) + vel_i(1)*Rot(2,1) + vel_i(2)*Rot(2,2); 

	veh_local_pos.x = x;  
	veh_local_pos.y = y;  
	veh_local_pos.z = z;  
	//printf("xy c %3.3f %3.3f %3.3f\n",double(vx), double(vy), double(vz));

	/* Please delete these lines */
	veh_local_pos.vx = vx; 
	veh_local_pos.vy = vy; 
	veh_local_pos.vz = vz; 
	veh_local_pos.xy_valid = true;
	veh_local_pos.z_valid = true;
	veh_local_pos.xy_global = true;
	veh_local_pos.z_global = true;
	veh_local_pos.ref_timestamp = hrt_absolute_time();
	veh_local_pos.timestamp = hrt_absolute_time();
	veh_local_pos.yaw = _att.yaw;

	px_tmp = x_gps(0);
	py_tmp = x_gps(1);
	pz_tmp = x_gps(2);
	
	
}

/**
  * Use rotation matrices to do rotation and compute measured velocities
*/
void 
MulticopterQuatPositionControl::dorotations(void)
{
	float px, py, pz, vx_hat, vy_hat, vz_hat, x_tmp, y_tmp, z_tmp;

	float ivx_hat, ivy_hat, ivz_hat;

	x_tmp = _vicon_pos.x;
	y_tmp = _vicon_pos.y;
	z_tmp = _vicon_pos.z; 

	/* Comment these out for now  */
	/*
	Rot(0,0) = _att.R[0]; 
	Rot(0,1) = _att.R[1]; 
	Rot(0,2) = _att.R[2];     

	Rot(1,0) = _att.R[3]; 
	Rot(1,1) = _att.R[4]; 
	Rot(1,2) = _att.R[5];		    

	Rot(2,0) = _att.R[6]; 
	Rot(2,1) = _att.R[7]; 
	Rot(2,2) = _att.R[8]; 
	*/
	float roll, pitch, yaw;
	float ctheta, stheta, cphi, sphi, cpsi, spsi;
	roll 	= _att.roll;	
	pitch 	= _att.pitch;	
	yaw 	= _att.yaw;	

	ctheta = cos(pitch);
	stheta = sin(pitch);
	cphi = cos(-roll);
	sphi = sin(-roll);
	cpsi = cos(-yaw);
	spsi = sin(-yaw);
	Rot(0,0) = ctheta*cpsi; 
	Rot(0,1) = sphi*stheta*cpsi-cphi*spsi; 
	Rot(0,2) = cphi*stheta*cpsi+sphi*spsi;     

	Rot(1,0) = ctheta*spsi; 
	Rot(1,1) = sphi*stheta*spsi+cphi*cpsi; 
	Rot(1,2) = cphi*stheta*spsi-sphi*cpsi;		    

	Rot(2,0) = -stheta; 
	Rot(2,1) = ctheta*sphi; 
	Rot(2,2) = ctheta*cphi; 
	

	Rot.transpose(); //uncomment

	/* Rotate Vehicle & reference position from \frameA to \frameB Consider doing matrix multiplications*/
	x = x_tmp * Rot(0,0) + y_tmp * Rot(0,1) + z_tmp * Rot(0,2); 
	y = x_tmp * Rot(1,0) + y_tmp * Rot(1,1) + z_tmp * Rot(1,2); 
	z = x_tmp * Rot(2,0) + y_tmp * Rot(2,1) + z_tmp * Rot(2,2); 


	px = px_tmp * Rot(0,0) + py_tmp * Rot(0,1) + pz_tmp * Rot(0,2); 
	py = px_tmp * Rot(1,0) + py_tmp * Rot(1,1) + pz_tmp * Rot(1,2); 
	pz = px_tmp * Rot(2,0) + py_tmp * Rot(2,1) + pz_tmp * Rot(2,2); 


	dtt = (_vicon_pos.timestamp - old_time) * 0.000001f;

    vx_hat = (x - px) / dtt;
    vy_hat = (y - py) / dtt;
    vz_hat = (z - pz) / dtt;
    //printf("Vicon dt and freq %3.3f %3.3f\n",double(dtt), double(1/dtt));

    /* Make restart possible by noting that max vehicle velocity <= 5 TODO read from params*/
    if (vx_hat > 5.0f || vx_hat < -5.0f)
    	vx_hat = 0.0f;
    if (vy_hat > 5.0f || vy_hat < -5.0f)
    	vy_hat = 0.0f;
    if (vz_hat > 5.0f || vz_hat < -5.0f)
    	vz_hat = 0.0f;

    vx = vx * 0.5f + vx_hat * 0.5f; 
    vy = vy * 0.5f + vy_hat * 0.5f;
    vz = vz * 0.5f + vz_hat * 0.5f;


	if (isinf(vx) || isnan(vx))
		vx = 0.0f;
	if (isinf(vy) || isnan(vy))
		vy = 0.0f;
	if (isinf(vz) || isnan(vz))
		vz = 0.0f;

	/* implement inertial filter */
	ivx_hat = (x_tmp - px_tmp) / dtt;
	ivy_hat = (y_tmp - py_tmp) / dtt;
	ivz_hat = (z_tmp - pz_tmp) / dtt;

	if (ivx_hat > 5.0f || ivx_hat < -5.0f)
    	ivx_hat = 0.0f;
    if (ivy_hat > 5.0f || ivy_hat < -5.0f)
    	ivy_hat = 0.0f;
    if (ivz_hat > 5.0f || ivz_hat < -5.0f)
    	ivz_hat = 0.0f;

    ivx = ivx * 0.5f + ivx_hat * 0.5f; 
    ivy = ivy * 0.5f + ivy_hat * 0.5f;
    ivz = ivz * 0.5f + ivz_hat * 0.5f;
	if (isinf(ivx) || isnan(ivx))
		ivx = 0.0f;
	if (isinf(ivy) || isnan(ivy))
		ivy = 0.0f;
	if (isinf(ivz) || isnan(ivz))
		ivz = 0.0f;
	/* End inertial filter */

	px_tmp = x_tmp;
	py_tmp = y_tmp;
	pz_tmp = z_tmp; 
}

void 
MulticopterQuatPositionControl::desired_setpoint(void)
{
	Vector3f   xd_tmp = Vector3f::Zero();
	Vector3f   vd_tmp = Vector3f::Zero();
	Vector3f   ad_tmp = Vector3f::Zero();
	
	/* Let us put the desired offbard inputs into local variables */
	xd_tmp(0) = _local_pos_sp.x;
	xd_tmp(1) = _local_pos_sp.y;
	xd_tmp(2) = _local_pos_sp.z; 
	vd_tmp(0) = _local_pos_sp.vx;
	vd_tmp(1) = _local_pos_sp.vy;
	vd_tmp(2) = _local_pos_sp.vz;
	ad_tmp(0) = _local_pos_sp.acc_x;
	ad_tmp(1) = _local_pos_sp.acc_y;
	ad_tmp(2) = _local_pos_sp.acc_z;

	des_yaw = -_local_pos_sp.yaw;

	/* build desired rotation matrix */
	Matrix3f 	Rot_td; 
	Rot_td(0,0) = cos(-des_yaw);
	Rot_td(1,0) = -sin(-des_yaw);
	Rot_td(2,0) = 0;
	Rot_td(0,1) = sin(-des_yaw);
	Rot_td(1,1) = cos(-des_yaw);
	Rot_td(2,1) = 0;
	Rot_td(0,2) = 0;
	Rot_td(1,2) = 0;
	Rot_td(2,2) = 1;

	pos_d = Rot_td * xd_tmp;
	vel_d = Rot_td * vd_tmp;

	/* Capture NaN and inf in desired states */
	if (isinf(pos_d(0)) || isnan(pos_d(0)))
		pos_d(0) = 0.0f;
	if (isinf(pos_d(1)) || isnan(pos_d(1)))
		pos_d(1) = 0.0f;
	if (isinf(pos_d(2)) || isnan(pos_d(2)))
		pos_d(2) = -1.10f;

	if (isinf(vel_d(0)) || isnan(vel_d(0)))
		vel_d(0) = 0.0f;
	if (isinf(vel_d(1)) || isnan(vel_d(1)))
		vel_d(1) = 0.0f;
	if (isinf(vel_d(2)) || isnan(vel_d(2)))
		vel_d(2) = 0.0f;
	

	if (isinf(des_yaw) || isnan(des_yaw))
		des_yaw = 0.0f;
}

/**
  * quaternion position controller
*/
void 
MulticopterQuatPositionControl::quat_traj_control(void)
{
	//Vector3f qvec, postildedot, veltildedot, Omega_d, err_xyz, pos, c_bar_vec, Rd_trans = Vector3f::Zero();
	Matrix3f c_bar_vec 		= Matrix3f::Zero();
	Vector3f err_xyz 		= Vector3f::Zero();
	//Vector3f Rd_trans 		= Vector3f::Zero();
	Vector3f pos 			= Vector3f::Zero();
	//Vector3f yaw_rot 		= Vector3f::Zero();
	Matrix3f Rdtrans 		= Matrix3f::Zero();
	//Vector3f nu 			= Vector3f::Zero();
	//Vector3f xi 			= Vector3f::Zero();

	Matrix3f Kp 			= Matrix3f::Identity();
	Matrix3f Kd 			= Matrix3f::Identity();
	Kp(0,0) 				= _params.pos_p(0);
	Kp(1,1) 				= _params.pos_p(1);
	Kp(2,2) 				= _params.pos_p(2);

	Kd(0,0) 				= _params.vel_p(0);
	Kd(1,1) 				= _params.vel_p(1);
	Kd(2,2) 				= _params.vel_p(2);
	
	
	if(_veh_gps_pos.fix_type >= 3)
	{
		pos = gps_body_pos_hat;
	} else
	{
		pos(0) = x;
		pos(1) = y;
		pos(2) = z;

		veh_vel_body(0) = vx;
		veh_vel_body(1) = vy;
		veh_vel_body(2) = vz;

	}

	err_xyz = Kp * (pos - pos_d) + Kd * (veh_vel_body - vel_d) - c_bar_vec * vel_d; 

	/* TODO bounds on ex ey ez should be in parameters file as in max in attitude or R_Max*/
	float max_erxy = 0.23f * 2.0f;
	if (err_xyz(0) > max_erxy) 
		err_xyz(0) = max_erxy;
	if (err_xyz(0) < -max_erxy)
		err_xyz(0) = -max_erxy;
	if (err_xyz(1) > max_erxy)
		err_xyz(1) = max_erxy;
	if (err_xyz(1) < -max_erxy)
		err_xyz(1) = -max_erxy;
	

	/* New method which is faster implementation */
	getQuatFromRPY123(err_xyz(1), err_xyz(0), des_yaw, qd);
	qd[1] = -qd[1];
	qd[3] = -qd[3];

	//printf("atd %3.3f %3.3f %3.3f\n", double(qd[1]), double(qd[2]), double(qd[3]));
	//printf("posd %3.3f %3.3f %3.3f %3.3f\n", double(err_xyz(0)), double(err_xyz(1)), double(err_xyz(2)), double(des_yaw));

	thrust = (_params.mg + err_xyz(2));


	if (thrust > _params.thr_max) 
		thrust = _params.thr_max; 
	if (thrust < _params.thr_min )
		thrust = _params.thr_min;

	/* Implement landing procedure */
	if ((int)(_local_pos_sp.z) == 0)
	{
		if (fabs(z) < 0.25)
			thrust = 0.1f;
	}

	/* Capture inf and NaN in thrust this should capture all the others*/
	if ((isfinite(thrust) == false) || !(isnan(thrust) == false))
		thrust = 0.10f;

	/* Capture inf and NaN in q this should capture all the others. Based on ex, ey, this should never happen */
	if ((isfinite(qd[0]) == false) || !(isnan(qd[0]) == false))
		qd[0] = 1.0f;

	if ((isfinite(qd[1]) == false) || !(isnan(qd[1]) == false))
		qd[1] = 0.10f;

	if ((isfinite(qd[2]) == false) || !(isnan(qd[2]) == false))
		qd[2] = 0.10f;

	if ((isfinite(qd[3]) == false) || !(isnan(qd[3]) == false))
		qd[3] = 0.10f;

	/* Determination of feedforward terms */
	/* We assume \dot{T}_d = 0 */
	Vector3f veh_acc = Vector3f::Zero();
	Vector3f veh_acc_tilde = Vector3f::Zero();
	veh_acc(0) = -raw.accelerometer_m_s2[0] - 0.2f;
	veh_acc(1) = -raw.accelerometer_m_s2[1] - 0.2f;
	veh_acc(2) = raw.accelerometer_m_s2[2] +  acc_grav;

	veh_acc_tilde = veh_acc - acc_d;
	Vector3f Omega_dxy = Rdtrans.transpose()*(Kp*(veh_vel_body - vel_d) + Kd*veh_acc_tilde - c_bar_vec*thrust*acc_d);

	omega_d.rates[1] = Omega_dxy(0);
	omega_d.rates[0] = Omega_dxy(1);
	omega_d.rates[2] = 0.0f; //See thesis
	omega_d.validity = true;

	if (_omega_pub != nullptr) {
		orb_publish(ORB_ID(vehicle_omega_ff_setpoint), _omega_pub, &omega_d);
	} else {
		_omega_pub = orb_advertise(ORB_ID(vehicle_omega_ff_setpoint), &omega_d);
	}

}


/**
  * Determine position based on gps
*/
math::Vector<3> 
MulticopterQuatPositionControl::map_projection(void) 
{
	math::Vector<3> output;

	double gps_poslat = _veh_gps_pos.lat; 
	double gps_poslon = _veh_gps_pos.lon; 

	gps_poslat = gps_poslat * M_DEG_TO_RAD*0.0000001;
	gps_poslon = gps_poslon * M_DEG_TO_RAD*0.0000001;
	double d_lon = gps_poslon - _veh_home.lon;

	double c = acos(sin(_veh_home.lat) * sin(gps_poslat) + cos(_veh_home.lat) * cos(gps_poslat) * cos(d_lon));
	double k = c/sin(c); 
	//if (fabs(c) < DBL_EPSILON)
	//	k = 1.0f;

	output(0) = k*(cos(_veh_home.lat) * sin(gps_poslat) - sin(_veh_home.lat) * cos(gps_poslat) * cos(d_lon)) * CONSTANTS_RADIUS_OF_EARTH; 
	
	output(1) = k*cos(_veh_home.lat) * sin(gps_poslon - _veh_home.lon) * CONSTANTS_RADIUS_OF_EARTH;
	output(2) = -(_veh_gps_pos.alt * 0.001f - _veh_home.alt);

	/* Just to make sure we dont get garbage */
	if (output(0) > 100.0f || output(1) > 100.0f || isnan(output(0)) == true ||isfinite(output(0)) == false)
	{
		output(0) = 0.0;
		output(1) = 0.0;
		output(2) = 0.0;
	}
	//printf("lat %3.3f %3.3f\n",double(_veh_home.lat),double(gps_poslat));
	return output;
}

void
MulticopterQuatPositionControl::task_main()
{

	/*
	 * do subscriptions
	 */
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_arming_sub = orb_subscribe(ORB_ID(actuator_armed));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	_local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
	_global_vel_sp_sub = orb_subscribe(ORB_ID(vehicle_global_velocity_setpoint));

	_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));

	_veh_gps_pos_sub = orb_subscribe(ORB_ID(vehicle_gps_position));

	_vicon_pos_sub = orb_subscribe(ORB_ID(vehicle_vicon_position));

	_v_vel_est_sub = orb_subscribe(ORB_ID(vehicle_velocity_est_inertial)); 

	_veh_vel_body_sub = orb_subscribe(ORB_ID(vehicle_velocity_meas_est_body));

	/* Do I need these two ? */
	_veh_vel_pub 	= nullptr;
	_omega_pub 		= nullptr;
	parameters_update(true);

	/* initialize values of critical structs until first regular update */
	_arming.armed = false;

	/* get an initial update for all sensor and status data */
	poll_subscriptions();

	hrt_abstime t_prev = 0;

	math::Vector<3> thrust_int;
	thrust_int.zero();


	math::Matrix<3, 3> R;
	R.identity();

	/* wakeup source */
	px4_pollfd_struct_t fds[1];

	fds[0].fd = _att_sub;
	fds[0].events = POLLIN;

	memset(&veh_local_pos, 0, sizeof(veh_local_pos));

	while (!_task_should_exit) {

		/* wait for up to 500ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 500);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}
		/* run controller on attitude changes FIXME enable this*/
		//if (fds[0].revents & POLLIN) {
		/* Do polling */
		poll_subscriptions(); // This should go outside of that loop

		parameters_update(false);

		hrt_abstime t = hrt_absolute_time();
		dt = t_prev != 0 ? (t - t_prev) * 0.000001f : 0.0f;
		t_prev = t;

		d_meas = (t - old_meas_time)*0.000001f;

		/* Setup for control stuff */
		if (!old_auto && _control_mode.flag_control_auto_enabled) {
		if (_att.q[0] >= 0.0f)
			sign_q0 = 1;
		else 
			sign_q0 = -1;
		}
		old_auto = _control_mode.flag_control_auto_enabled;

		_veh_vel.body_valid 			= false;
		_veh_vel.inertial_valid 		= false;
	
	//printf("%s\n", _control_mode.flag_control_auto_enabled ? "true" : "false");
	//printf("%s\n", _control_mode.flag_control_manual_enabled ? "true" : "false");
	/* Do GPS pos control */
	if (_veh_gps_pos.fix_type >= 3 && _v_vel_est.timestamp != _v_est_time_old)// && initialise_home)// && _v_vel_est.body_valid)
	{
		//usleep(10000);
		veh_vel_body(0) = _v_vel_est.inertial_bvx;
		veh_vel_body(1) = _v_vel_est.inertial_bvy;
		veh_vel_body(2) = _v_vel_est.inertial_bvz;

		/* If we haven't taken off or in the process of landing use barometer measurements z and vz or altitude is less than yyy*/

		dt_pos_est = (_v_vel_est.timestamp - _v_est_time_old)*0.000001f;
		//printf("dt freq %3.3f %3.3f\n",double(dt_pos_est),double(1/dt_pos_est));

		/* GPS Position estimation */
		gps_body_pos_hat_dot(0)  = veh_vel_body(0) - _params.vel_ff(0)*(gps_body_pos_hat(0) - x); 
		gps_body_pos_hat_dot(1)  = veh_vel_body(1) - _params.vel_ff(1)*(gps_body_pos_hat(1) - y);
		gps_body_pos_hat_dot(2)  = veh_vel_body(2) - _params.vel_ff(2)*(gps_body_pos_hat(2) - z);

		gps_body_pos_hat = gps_body_pos_hat + gps_body_pos_hat_dot*dt_pos_est;
		//printf("x hatx %3.3f %3.3f %3.3f\n",double(x), double(gps_body_pos_hat(0)), double(veh_vel_body(0)));

		if (isinf(gps_body_pos_hat(0)) || isnan(gps_body_pos_hat(0)))
			gps_body_pos_hat(0) = x;

		if (isinf(gps_body_pos_hat(1)) || isnan(gps_body_pos_hat(1)))
			gps_body_pos_hat(1) = y;

		if (isinf(gps_body_pos_hat(2)) || isnan(gps_body_pos_hat(2)))
			gps_body_pos_hat(2) = z;

		veh_local_pos.ref_lat = gps_body_pos_hat(0);
		veh_local_pos.ref_lon = gps_body_pos_hat(1);
		veh_local_pos.ref_alt = gps_body_pos_hat(2);

		veh_local_pos.x = x;  
		veh_local_pos.y = y;  
		veh_local_pos.z = z;  
		
		veh_local_pos.vx = vx; 
		veh_local_pos.vy = vy; 
		veh_local_pos.vz = vz; 
		veh_local_pos.xy_valid = true;
		veh_local_pos.z_valid = true;
		veh_local_pos.xy_global = true;
		veh_local_pos.z_global = true;
		veh_local_pos.ref_timestamp = hrt_absolute_time();
		veh_local_pos.timestamp = hrt_absolute_time();
		veh_local_pos.yaw = _att.yaw;

		
	if (_local_pos_pub != nullptr ) {
		orb_publish(ORB_ID(vehicle_local_position), _local_pos_pub, &veh_local_pos);

	} else {
		_local_pos_pub = orb_advertise(ORB_ID(vehicle_local_position), &veh_local_pos);	
	}

		if (_control_mode.flag_control_auto_enabled && initialise_home)
		{	
			desired_setpoint();
			quat_traj_control();

			_att_sp.q_d_valid = true;
			_att_sp.q_d[0] = qd[0];
			_att_sp.q_d[1] = qd[1];
			_att_sp.q_d[2] = qd[2];
			_att_sp.q_d[3] = qd[3];
			//_att_sp_msg.data().thrust = thrust;
			//printf("auto mode\n");
			
			/* Use Autonomous mode */			
			if (_att_sp_pub != nullptr) {
				orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &_att_sp);

			} else {
				_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_att_sp);
			}
		}

		_v_est_time_old = _v_vel_est.timestamp;
	}
	usleep(5000);	
//printf("sat %lu %d %3.3f, %3.5f, %3.3f\n",_veh_gps_pos.timestamp_position, _veh_gps_pos.fix_type, (double)_veh_gps_pos.lat*1e-7, (double)_veh_gps_pos.lat*1e-7, double(_veh_gps_pos.alt*1e-3));

	// This should only run when Vicon is not available
		/* GPS is only valid when the fix is >=3D which requires a minim of 3 satellites. Do position control and  */ 
	if (_veh_gps_pos.timestamp_time != gps_old_time && _veh_gps_pos.fix_type >= 3 && _veh_gps_pos.satellites_used > 4)
	{
		/* Handle home position and publish */
		if(!initialise_home)
		{
			//if (home_count > 1)
			//{
				home_lat += _veh_gps_pos.lat;
				home_lon += _veh_gps_pos.lon;
				home_alt += _veh_gps_pos.alt;
			//}			
				
			home_count++;
			//printf("xy c %3.3f %3.3f %u\n",double(_veh_gps_pos.lat), double(_veh_gps_pos.lon), home_count);
		}


		if(home_count > 20 && !initialise_home)
		{
			
			initialise_home = true;
			_veh_home.lat = home_lat/(home_count-2*0);
			_veh_home.lon = home_lon/(home_count-2*0);
			_veh_home.alt = home_alt/(home_count-2*0);

			if (home_position_pub == 0) {
				//home_position_pub->publish(_veh_home);
				orb_publish(ORB_ID(home_position), home_position_pub, &_veh_home);

			} else {
				home_position_pub= orb_advertise(ORB_ID(home_position), &_veh_home);
				//_att_sp_pub = _n.advertise<px4_vehicle_attitude_setpoint>();
			}

			_veh_home.lat = (double)_veh_home.lat*1e-7*M_DEG_TO_RAD;
			_veh_home.lon = (double)_veh_home.lon*1e-7*M_DEG_TO_RAD;
			_veh_home.alt = (double)_veh_home.alt*0.001;
		}	
//printf("home %3.3f %3.3f %3.3f %3.3f\n",double(_veh_home.lat), double(_veh_home.lon), double(_veh_home.alt), double(M_DEG_TO_RAD));
		vel_pub = _veh_gps_pos.vel_ned_valid;
		gps_vel_setup();
		quat_traj_control();
		if (_control_mode.flag_control_auto_enabled)
		{	
			printf("auto mode");
			_att_sp.q_d_valid = true;
			_att_sp.q_d[0] = qd[0];
			_att_sp.q_d[1] = qd[1];
			_att_sp.q_d[2] = qd[2];
			_att_sp.q_d[3] = qd[3];
			//_att_sp_msg.data().thrust = thrust;

			/* Use Autonomous mode */
			if (_att_sp_pub != nullptr) {
				orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &_att_sp);

			} else {
				_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_att_sp);
			}
		} 
		float dtttss = (_veh_gps_pos.timestamp_time -gps_old_time )*0.000001f;
		printf("dt gps freq %3.3f %3.3f\n",double(dtttss),double(1/dtttss));
		gps_old_time 	= _veh_gps_pos.timestamp_time;

		old_meas_time 	= get_time_micros();

		if (dtttss < 1){ //GPS pub wasn't initialised to nullptr
			_veh_vel.body_valid 			= _veh_gps_pos.vel_ned_valid;
			_veh_vel.body_vx 			= vx;
			_veh_vel.body_vy 			= vy;
			_veh_vel.body_vz 			= vz;
			_veh_vel.inertial_vx 		= ivx;
			_veh_vel.inertial_vy 		= ivy;
			_veh_vel.inertial_vz 		= ivz;
			_veh_vel.inertial_valid 		= _veh_gps_pos.vel_ned_valid;
			_veh_vel.timestamp 			= _veh_gps_pos.timestamp_time; 
			vel_updated = true;
		}	else {
			_veh_vel.inertial_valid 	= false;
		}	

	} else if (_vicon_pos.timestamp != old_time)
	{
		vel_pub = true;
		desired_setpoint();
		dorotations();
		old_time = _vicon_pos.timestamp;
		old_meas_time = get_time_micros();
		//printf("receivinf vicon\n");
		
		if (_control_mode.flag_control_auto_enabled)
		{
			printf("auto mode ");
			printf("%3.3f %3.3f %3.3f %3.3f\n", double(_att_sp.q_d[0]),double(_att_sp.q_d[1]),double(_att_sp.q_d[2]),double(_att_sp.q_d[3]));

			veh_vel_body(0) = vx;
			veh_vel_body(1) = vy;
			veh_vel_body(2) = vz;
			quat_traj_control();
			_att_sp.q_d_valid = true;
			_att_sp.q_d[0] = qd[0];
			_att_sp.q_d[1] = qd[1];
			_att_sp.q_d[2] = qd[2];
			_att_sp.q_d[3] = qd[3];
			//_att_sp.thrust = thrust;

			/* Use Autonomous mode */
			if (_att_sp_pub != nullptr) {
				orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &_att_sp);

			} else {
				orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_att_sp);
			}
		}

		/* Publish Body Velocities */
		if (dtt < 1){
			_veh_vel.body_valid 		= true;
			_veh_vel.body_vx 		= vx;
			_veh_vel.body_vy 		= vy;
			_veh_vel.body_vz 		= vz;
			_veh_vel.inertial_vx 	= ivx;
			_veh_vel.inertial_vy 	= ivy;
			_veh_vel.inertial_vz 	= ivz;
			_veh_vel.inertial_valid 	= true;
			_veh_vel.timestamp 		= _vicon_pos.timestamp; 
			vel_updated = true;	
		}	else {
			_veh_vel.inertial_valid 	= false;
		}		
	}

	if (d_meas > 0.80f && d_meas < 1.5f) // we need to set the velocities to false then update
	{
		_veh_vel.body_valid 			= false;
		_veh_vel.inertial_valid 		= false;
		_vicon_pos.valid 			= false;
		vel_updated 				= true;
		vel_pub 				= true;
	} else if (d_meas > 1.5f){
		_veh_vel.body_valid 			= false;
		_veh_vel.inertial_valid 		= false;
		_vicon_pos.valid 			= false;
		vel_updated 				= false;
		vel_pub 				= false;
	} 

	/* Publish measured velocities when they are valid */
	if (vel_pub && vel_updated)
	{
		if (_veh_vel_pub != nullptr) {
			orb_publish(ORB_ID(vehicle_velocity_meas_inertial), _veh_vel_pub, &_veh_vel);
		} else {
			_veh_vel_pub = orb_advertise(ORB_ID(vehicle_velocity_meas_inertial), &_veh_vel);
		}
	}
	vel_updated = false;
	}

	mavlink_log_info(&_mavlink_log_pub, "[mpc] stopped");

	_control_task = -1;
}

int
MulticopterQuatPositionControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("mc_quat_pos_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1900,
					   (px4_main_t)&MulticopterQuatPositionControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int mc_quat_pos_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: mc_quat_pos_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (quat_pos_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		quat_pos_control::g_control = new MulticopterQuatPositionControl;

		if (quat_pos_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != quat_pos_control::g_control->start()) {
			delete quat_pos_control::g_control;
			quat_pos_control::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (quat_pos_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete quat_pos_control::g_control;
		quat_pos_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (quat_pos_control::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
