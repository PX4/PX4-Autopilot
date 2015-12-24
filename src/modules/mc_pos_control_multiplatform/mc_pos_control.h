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

using namespace px4;

class MulticopterPositionControlMultiplatform
{
public:
	/**
	 * Constructor
	 */
	MulticopterPositionControlMultiplatform();

	/**
	 * Destructor, also kills task.
	 */
	~MulticopterPositionControlMultiplatform();

	/* Callbacks for topics */
	void handle_vehicle_attitude(const px4_vehicle_attitude &msg);
	void handle_parameter_update(const px4_parameter_update &msg);
	void handle_position_setpoint_triplet(const px4_position_setpoint_triplet &msg);

	void spin() { _n.spin(); }

protected:
	const float alt_ctl_dz = 0.2f;

	bool		_task_should_exit;		/**< if true, task should exit */
	int		_control_task;			/**< task handle for task */
	int		_mavlink_fd;			/**< mavlink fd */

	Publisher<px4_vehicle_attitude_setpoint>	*_att_sp_pub;			/**< attitude setpoint publication */
	Publisher<px4_vehicle_local_position_setpoint>	*_local_pos_sp_pub;		/**< vehicle local position setpoint publication */
	Publisher<px4_vehicle_global_velocity_setpoint>	*_global_vel_sp_pub;		/**< vehicle global velocity setpoint publication */


	Subscriber<px4_vehicle_attitude> *_att;				    /**< vehicle attitude */
	Subscriber<px4_vehicle_control_mode> *_control_mode;		    /**< vehicle control mode */
	Subscriber<px4_parameter_update> *_parameter_update;		    /**< parameter update */
	Subscriber<px4_manual_control_setpoint> *_manual_control_sp;	    /**< manual control setpoint */
	Subscriber<px4_actuator_armed> *_armed;				    /**< actuator arming status */
	Subscriber<px4_vehicle_local_position> *_local_pos;		    /**< local position */
	Subscriber<px4_position_setpoint_triplet> *_pos_sp_triplet;	    /**< local position */
	Subscriber<px4_vehicle_local_position_setpoint> *_local_pos_sp;	    /**< local position */
	Subscriber<px4_vehicle_global_velocity_setpoint> *_global_vel_sp;   /**< local position */

	px4_vehicle_attitude_setpoint _att_sp_msg;
	px4_vehicle_local_position_setpoint _local_pos_sp_msg;
	px4_vehicle_global_velocity_setpoint _global_vel_sp_msg;

	px4::NodeHandle _n;

	px4::AppState _appState;

	struct {
		px4::ParameterFloat thr_min;
		px4::ParameterFloat thr_max;
		px4::ParameterFloat z_p;
		px4::ParameterFloat z_vel_p;
		px4::ParameterFloat z_vel_i;
		px4::ParameterFloat z_vel_d;
		px4::ParameterFloat z_vel_max;
		px4::ParameterFloat z_ff;
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

		math::Vector<3> pos_p;
		math::Vector<3> vel_p;
		math::Vector<3> vel_i;
		math::Vector<3> vel_d;
		math::Vector<3> vel_ff;
		math::Vector<3> vel_max;
		math::Vector<3> sp_offs_max;
	}		_params;

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
	 * Update control outputs
	 */
	void		control_update();

	static float	scale_control(float ctl, float end, float dz);

	/**
	 * Update reference for local position projection
	 */
	void		update_ref();
	/**
	 * Reset position setpoint to current position
	 */
	void		reset_pos_sp();

	/**
	 * Reset altitude setpoint to current altitude
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

	bool		cross_sphere_line(const math::Vector<3>& sphere_c, float sphere_r,
					const math::Vector<3> line_a, const math::Vector<3> line_b, math::Vector<3>& res);

	/**
	 * Set position setpoint for AUTO
	 */
	void		control_auto(float dt);

	/**
	 * Select between barometric and global (AMSL) altitudes
	 */
	void		select_alt(bool global);
};
