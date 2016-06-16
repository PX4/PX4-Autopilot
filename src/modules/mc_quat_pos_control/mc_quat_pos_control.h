/****************************************************************************
 *
 *   Copyright (c) 2013 - 2015 PX4 Development Team. All rights reserved.
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
 * @file mc_pos_control.h
 * Multicopter position controller.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#pragma once

#include <px4.h>
#include <cstdio>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
// #include <poll.h>
// #include <drivers/drv_hrt.h>
// #include <arch/board/board.h>
// #include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
// #include <mavlink/mavlink_log.h>

#include <px4_eigen.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_local_position.h>

#include <uORB/topics/vehicle_velocity_est_inertial.h>
#include <uORB/topics/vehicle_velocity_meas_inertial.h>
#include <uORB/topics/vehicle_velocity_meas_est_body.h>  
#include <av_estimator/include/av_estimator_params.h> 
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_omega_ff_setpoint.h>

#ifndef MC_QUAT_POS_CONTROL_H
#define MC_QUAT_POS_CONTROL_H


using namespace px4;
using namespace Eigen;

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

	int 	start();


protected:
	const float alt_ctl_dz = 0.2f;

	bool		_task_should_exit;		/**< if true, task should exit */
	int		_control_task ;			/**< task handle for task */
	orb_advert_t	_mavlink_log_pub;		/**< mavlink log advert */

	int		_local_pos_sub;
	int		_global_vel_sp_sub;
	int		_vehicle_status_sub;		/**< vehicle status subscription */
	int		_vehicle_land_detected_sub;	/**< vehicle land detected subscription */
	int		_ctrl_state_sub;		/**< control state subscription */
	bool 		vel_pub = false;

	orb_advert_t	_att_sp_pub;			/**< attitude setpoint publication */
	orb_advert_t	_local_pos_sp_pub;		/**< vehicle local position setpoint publication */
	orb_advert_t	_global_vel_sp_pub;		/**< vehicle global velocity setpoint publication */
	orb_advert_t		_veh_vel_pub;			/**<Publish measured vehicle velocities */


	struct vehicle_status_s 			_vehicle_status; 	/**< vehicle status */
	struct vehicle_land_detected_s 			_vehicle_land_detected;	/**< vehicle land detected */
	struct control_state_s				_ctrl_state;		/**< vehicle attitude */
	struct vehicle_attitude_s _att;				    /**< vehicle attitude */
	struct vehicle_control_mode_s _control_mode;		    /**< vehicle control mode */
	struct parameter_update_s _parameter_update;		    /**< parameter update */
	struct manual_control_setpoint_s _manual_control_sp;	    /**< manual control setpoint */
	struct actuator_armed_s _armed;				    /**< actuator arming status */
	struct vehicle_local_position_s _local_pos;		    /**< local position */
	struct position_setpoint_triplet_s _pos_sp_triplet;	    /**< local position */
	struct vehicle_local_position_setpoint_s _local_pos_sp;	    /**< local position */
	struct vehicle_global_velocity_setpoint_s _global_vel_sp;   /**< local position */

	struct vehicle_attitude_setpoint_s _att_sp_msg;
	struct vehicle_local_position_setpoint_s _local_pos_sp_msg;
	struct vehicle_global_velocity_setpoint_s _global_vel_sp_msg;

	struct vehicle_vicon_position_s 			_vicon_pos;   			/**< vicon position */
	struct vehicle_gps_position_s 			_veh_gps_pos;			/**< Subscribe to gps position */
	struct vehicle_velocity_meas_inertial_s			_veh_vel_msg;			/**< vehicle velocity */

	struct {
		px4::ParameterFloat thr_min;
		px4::ParameterFloat thr_max;
		px4::ParameterFloat z_p;
		px4::ParameterFloat z_vel_p;
		px4::ParameterFloat z_vel_i;
		px4::ParameterFloat z_vel_d;
		px4::ParameterFloat z_vel_max;
		px4::ParameterFloat z_ff;
		px4::ParameterFloat mg;
		px4::ParameterFloat xy_p;
		px4::ParameterFloat xy_vel_p;
		px4::ParameterFloat xy_vel_i;
		px4::ParameterFloat xy_vel_d;
		px4::ParameterFloat xy_vel_max;
		px4::ParameterFloat xy_ff;
		px4::ParameterFloat tilt_max_air;
		px4::ParameterFloat land_speed;
		px4::ParameterFloat tilt_max_land;
		px4::ParameterFloat man_roll_max;
		px4::ParameterFloat man_pitch_max;
		px4::ParameterFloat man_yaw_max;
		px4::ParameterFloat mc_att_yaw_p;   // needed for calculating reasonable attitude setpoints in manual
	}		_params_handles;		/**< handles for interesting parameters */

	struct {
		float thr_min;
		float thr_max;
		float tilt_max_air;
		float land_speed;
		float tilt_max_land;
		float man_roll_max;
		float man_pitch_max;
		float man_yaw_max;
		float mc_att_yaw_p;

		float mg;

		math::Vector<3> pos_p;
		math::Vector<3> vel_p;
		math::Vector<3> vel_i;
		math::Vector<3> vel_d;
		math::Vector<3> vel_ff;
		math::Vector<3> vel_max;
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
	float 		d_meas, home_lat, home_lon, home_alt;
	float 		dt_pos_est;

	bool 		initialise_home = false; 					/**< GPS home initialisation based on 20 gps coordinates */
	int 		home_count 	= 0;  							/**< counter for home position in gps mode */

	orb_advert_t home_position_pub; //int 		home_position_pub 			= -1;
	struct 		vehicle_local_position_s 	veh_local_pos;
	struct 		home_position_s 			_veh_home;
	Publisher<px4_vehicle_local_position> 		*_local_pos_pub ;//				= -1;

	struct 		vehicle_velocity_est_inertial_s		_v_vel_est;			/**< vehicle velocity estimated */
	int         _v_vel_est_sub = -1;			/**< Subscribe to vehicle est velo */

	/* subscribe to raw data */
	int 		sub_raw 			= -1; 
	struct 		sensor_combined_s 	raw;

	orb_advert_t 		_omega_pub;// = -1;
	struct 		vehicle_omega_ff_setpoint_s 	omega_d;


	struct map_projection_reference_s _ref_pos;
	float _ref_alt;
	uint64_t _ref_timestamp;

	bool _reset_pos_sp;
	bool _reset_alt_sp;
	bool _mode_auto;

	math::Vector<3> _pos;
	math::Vector<3> _pos_sp;
	math::Vector<3> _vel;
	math::Vector<3> _vel_sp;
	math::Vector<3> _vel_prev;			/**< velocity on previous step */
	math::Vector<3> _vel_ff;
	math::Vector<3> _sp_move_rate;

	math::Vector<3> _thrust_int;
	math::Matrix<3, 3> _R;

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
#endif
