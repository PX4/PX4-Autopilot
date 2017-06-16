/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file mc_pos_control.cpp
 * Multicopter position controller.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#include "mc_pos_control.h"
#include "mc_pos_control_params.h"
/* The following inclue is needed because the pos controller depens on a parameter from attitude control to set a
 * reasonable yaw setpoint in manual mode */
#include "../mc_att_control_multiplatform/mc_att_control_params.h"

#define TILT_COS_MAX	0.7f
#define SIGMA		0.000001f
#define MIN_DIST	0.01f

MulticopterPositionControlMultiplatform::MulticopterPositionControlMultiplatform() :

	_task_should_exit(false),
	_control_task(-1),
	//_mavlink_log_pub(nullptr),

	/* publications */
	_att_sp_pub(nullptr),
	_local_pos_sp_pub(nullptr),
	_global_vel_sp_pub(nullptr),

	/* outgoing messages */
	_att_sp_msg(),
	_local_pos_sp_msg(),
	_global_vel_sp_msg(),

	_n(_appState),

	/* parameters */
	_params_handles(
{
	.thr_min	    = px4::ParameterFloat("MPP_THR_MIN", PARAM_MPP_THR_MIN_DEFAULT),
		.thr_max	    = px4::ParameterFloat("MPP_THR_MAX", PARAM_MPP_THR_MAX_DEFAULT),
			.z_p		    = px4::ParameterFloat("MPP_Z_P", PARAM_MPP_Z_P_DEFAULT),
				   .z_vel_p	    = px4::ParameterFloat("MPP_Z_VEL_P", PARAM_MPP_Z_VEL_P_DEFAULT),
					   .z_vel_i	    = px4::ParameterFloat("MPP_Z_VEL_I", PARAM_MPP_Z_VEL_I_DEFAULT),
						   .z_vel_d	    = px4::ParameterFloat("MPP_Z_VEL_D", PARAM_MPP_Z_VEL_D_DEFAULT),
							   .z_vel_max	    = px4::ParameterFloat("MPP_Z_VEL_MAX", PARAM_MPP_Z_VEL_MAX_DEFAULT),
								 .z_ff		    = px4::ParameterFloat("MPP_Z_FF", PARAM_MPP_Z_FF_DEFAULT),
									   .xy_p		    = px4::ParameterFloat("MPP_XY_P", PARAM_MPP_XY_P_DEFAULT),
										     .xy_vel_p	    = px4::ParameterFloat("MPP_XY_VEL_P", PARAM_MPP_XY_VEL_P_DEFAULT),
											    .xy_vel_i	    = px4::ParameterFloat("MPP_XY_VEL_I", PARAM_MPP_XY_VEL_I_DEFAULT),
												   .xy_vel_d	    = px4::ParameterFloat("MPP_XY_VEL_D", PARAM_MPP_XY_VEL_D_DEFAULT),
													  .xy_vel_max	    = px4::ParameterFloat("MPP_XY_VEL_MAX", PARAM_MPP_XY_VEL_MAX_DEFAULT),
													       .xy_ff		    = px4::ParameterFloat("MPP_XY_FF", PARAM_MPP_XY_FF_DEFAULT),
															.tilt_max_air	    = px4::ParameterFloat("MPP_TILTMAX_AIR", PARAM_MPP_TILTMAX_AIR_DEFAULT),
															   .land_speed	    = px4::ParameterFloat("MPP_LAND_SPEED", PARAM_MPP_LAND_SPEED_DEFAULT),
																.tilt_max_land	    = px4::ParameterFloat("MPP_TILTMAX_LND", PARAM_MPP_TILTMAX_LND_DEFAULT),
																  .man_roll_max	    = px4::ParameterFloat("MPP_MAN_R_MAX", PARAM_MPP_MAN_R_MAX_DEFAULT),
																     .man_pitch_max	    = px4::ParameterFloat("MPP_MAN_P_MAX", PARAM_MPP_MAN_P_MAX_DEFAULT),
																       .man_yaw_max	    = px4::ParameterFloat("MPP_MAN_Y_MAX", PARAM_MPP_MAN_Y_MAX_DEFAULT),
																	   .mc_att_yaw_p	    = px4::ParameterFloat("MP_YAW_P", PARAM_MP_YAW_P_DEFAULT)
}),
_ref_alt(0.0f),
_ref_timestamp(0),

_reset_pos_sp(true),
_reset_alt_sp(true),
_mode_auto(false),
_thrust_int(),
_R()
{
	memset(&_ref_pos, 0, sizeof(_ref_pos));

	/*
	 * Do subscriptions
	 */
	_att = _n.subscribe<px4_vehicle_attitude>(&MulticopterPositionControlMultiplatform::handle_vehicle_attitude, this, 0);
	_control_mode = _n.subscribe<px4_vehicle_control_mode>(0);
	_parameter_update = _n.subscribe<px4_parameter_update>(
				    &MulticopterPositionControlMultiplatform::handle_parameter_update, this, 1000);
	_manual_control_sp = _n.subscribe<px4_manual_control_setpoint>(0);
	_armed = _n.subscribe<px4_actuator_armed>(0);
	_local_pos = _n.subscribe<px4_vehicle_local_position>(0);
	_pos_sp_triplet = _n.subscribe<px4_position_setpoint_triplet>
			  (&MulticopterPositionControlMultiplatform::handle_position_setpoint_triplet, this, 0);
	_local_pos_sp = _n.subscribe<px4_vehicle_local_position_setpoint>(0);
	_global_vel_sp = _n.subscribe<px4_vehicle_global_velocity_setpoint>(0);


	_params.pos_p.zero();
	_params.vel_p.zero();
	_params.vel_i.zero();
	_params.vel_d.zero();
	_params.vel_max.zero();
	_params.vel_ff.zero();
	_params.sp_offs_max.zero();

	_pos.zero();
	_pos_sp.zero();
	_vel.zero();
	_vel_sp.zero();
	_vel_prev.zero();
	_vel_ff.zero();
	_sp_move_rate.zero();

	/* fetch initial parameter values */
	parameters_update();

	_R.identity();
}

MulticopterPositionControlMultiplatform::~MulticopterPositionControlMultiplatform()
{
}

int
MulticopterPositionControlMultiplatform::parameters_update()
{
	_params.thr_min = _params_handles.thr_min.update();
	_params.thr_max = _params_handles.thr_max.update();
	_params.tilt_max_air = math::radians(_params_handles.tilt_max_air.update());
	_params.land_speed = _params_handles.land_speed.update();
	_params.tilt_max_land = math::radians(_params_handles.tilt_max_land.update());

	/* manual control scale */
	_params.man_roll_max = math::radians(_params_handles.man_roll_max.update());
	_params.man_pitch_max = math::radians(_params_handles.man_pitch_max.update());
	_params.man_yaw_max = math::radians(_params_handles.man_yaw_max.update());

	_params.mc_att_yaw_p = _params_handles.mc_att_yaw_p.update();

	_params.pos_p(0) = _params.pos_p(1) = _params_handles.xy_p.update();
	_params.pos_p(2) = _params_handles.z_p.update();
	_params.vel_p(0) = _params.vel_p(1) = _params_handles.xy_vel_p.update();
	_params.vel_p(2) = _params_handles.z_vel_p.update();
	_params.vel_i(0) = _params.vel_i(1) = _params_handles.xy_vel_i.update();
	_params.vel_i(2) = _params_handles.z_vel_i.update();
	_params.vel_d(0) = _params.vel_d(1) = _params_handles.xy_vel_d.update();
	_params.vel_d(2) = _params_handles.z_vel_d.update();
	_params.vel_max(0) = _params.vel_max(1) = _params_handles.xy_vel_max.update();
	_params.vel_max(2) = _params_handles.z_vel_max.update();

	_params.vel_ff(0) = _params.vel_ff(1) = math::constrain(_params_handles.xy_ff.update(), 0.0f, 1.0f);
	_params.vel_ff(2) = math::constrain(_params_handles.z_ff.update(), 0.0f, 1.0f);

	_params.sp_offs_max = _params.vel_max.edivide(_params.pos_p) * 2.0f;

	return OK;
}


float
MulticopterPositionControlMultiplatform::scale_control(float ctl, float end, float dz)
{
	if (ctl > dz) {
		return (ctl - dz) / (end - dz);

	} else if (ctl < -dz) {
		return (ctl + dz) / (end - dz);

	} else {
		return 0.0f;
	}
}

void
MulticopterPositionControlMultiplatform::update_ref()
{
	if (_local_pos->data().ref_timestamp != _ref_timestamp) {
		double lat_sp, lon_sp;
		float alt_sp = 0.0f;

		if (_ref_timestamp != 0) {
			/* calculate current position setpoint in global frame */
			map_projection_reproject(&_ref_pos, _pos_sp(0), _pos_sp(1), &lat_sp, &lon_sp);
			alt_sp = _ref_alt - _pos_sp(2);
		}

		/* update local projection reference */
		map_projection_init(&_ref_pos, _local_pos->data().ref_lat, _local_pos->data().ref_lon);
		_ref_alt = _local_pos->data().ref_alt;

		if (_ref_timestamp != 0) {
			/* reproject position setpoint to new reference */
			map_projection_project(&_ref_pos, lat_sp, lon_sp, &_pos_sp.data[0], &_pos_sp.data[1]);
			_pos_sp(2) = -(alt_sp - _ref_alt);
		}

		_ref_timestamp = _local_pos->data().ref_timestamp;
	}
}

void
MulticopterPositionControlMultiplatform::reset_pos_sp()
{
	if (_reset_pos_sp) {
		_reset_pos_sp = false;
		/* shift position setpoint to make attitude setpoint continuous */
		_pos_sp(0) = _pos(
				     0); //+ (_vel(0) - PX4_R(_att_sp_msg.data().R_body, 0, 2) * _att_sp_msg.data().thrust / _params.vel_p(0)
		// - _params.vel_ff(0) * _sp_move_rate(0)) / _params.pos_p(0);
		_pos_sp(1) = _pos(
				     1); //+ (_vel(1) - PX4_R(_att_sp_msg.data().R_body, 1, 2) * _att_sp_msg.data().thrust / _params.vel_p(1)
		// - _params.vel_ff(1) * _sp_move_rate(1)) / _params.pos_p(1);

		//XXX: port this once a mavlink like interface is available
		// mavlink_log_info(&_mavlink_log_pub, "[mpc] reset pos sp: %d, %d", (int)_pos_sp(0), (int)_pos_sp(1));
		PX4_INFO("[mpc] reset pos sp: %2.3f, %2.3f", (double)_pos_sp(0), (double)_pos_sp(1));
	}
}

void
MulticopterPositionControlMultiplatform::reset_alt_sp()
{
	if (_reset_alt_sp) {
		_reset_alt_sp = false;
		_pos_sp(2) = _pos(2) + (_vel(2) - _params.vel_ff(2) * _sp_move_rate(2)) / _params.pos_p(2);

		//XXX hack until #1741 is in/ported
		/* reset yaw sp */
		matrix::Eulerf euler = matrix::Quatf(_att->data().q);
		_att_sp_msg.data().yaw_body = euler.psi();

		//XXX: port this once a mavlink like interface is available
		// mavlink_log_info(&_mavlink_log_pub, "[mpc] reset alt sp: %d", -(int)_pos_sp(2));
		PX4_INFO("[mpc] reset alt sp: %2.3f", -(double)_pos_sp(2));
	}
}

void
MulticopterPositionControlMultiplatform::limit_pos_sp_offset()
{
	math::Vector<3> pos_sp_offs;
	pos_sp_offs.zero();

	if (_control_mode->data().flag_control_position_enabled) {
		pos_sp_offs(0) = (_pos_sp(0) - _pos(0)) / _params.sp_offs_max(0);
		pos_sp_offs(1) = (_pos_sp(1) - _pos(1)) / _params.sp_offs_max(1);
	}

	if (_control_mode->data().flag_control_altitude_enabled) {
		pos_sp_offs(2) = (_pos_sp(2) - _pos(2)) / _params.sp_offs_max(2);
	}

	float pos_sp_offs_norm = pos_sp_offs.length();

	if (pos_sp_offs_norm > 1.0f) {
		pos_sp_offs /= pos_sp_offs_norm;
		_pos_sp = _pos + pos_sp_offs.emult(_params.sp_offs_max);
	}
}

void
MulticopterPositionControlMultiplatform::control_manual(float dt)
{
	_sp_move_rate.zero();

	if (_control_mode->data().flag_control_altitude_enabled) {
		/* move altitude setpoint with throttle stick */
		_sp_move_rate(2) = -scale_control(_manual_control_sp->data().z - 0.5f, 0.5f, alt_ctl_dz);
	}

	if (_control_mode->data().flag_control_position_enabled) {
		/* move position setpoint with roll/pitch stick */
		_sp_move_rate(0) = _manual_control_sp->data().x;
		_sp_move_rate(1) = _manual_control_sp->data().y;
	}

	/* limit setpoint move rate */
	float sp_move_norm = _sp_move_rate.length();

	if (sp_move_norm > 1.0f) {
		_sp_move_rate /= sp_move_norm;
	}

	/* _sp_move_rate scaled to 0..1, scale it to max speed and rotate around yaw */
	math::Matrix<3, 3> R_yaw_sp;
	R_yaw_sp.from_euler(0.0f, 0.0f, _att_sp_msg.data().yaw_body);
	_sp_move_rate = R_yaw_sp * _sp_move_rate.emult(_params.vel_max);

	if (_control_mode->data().flag_control_altitude_enabled) {
		/* reset alt setpoint to current altitude if needed */
		reset_alt_sp();
	}

	if (_control_mode->data().flag_control_position_enabled) {
		/* reset position setpoint to current position if needed */
		reset_pos_sp();
	}

	/* feed forward setpoint move rate with weight vel_ff */
	_vel_ff = _sp_move_rate.emult(_params.vel_ff);

	/* move position setpoint */
	_pos_sp += _sp_move_rate * dt;

	/* check if position setpoint is too far from actual position */
	math::Vector<3> pos_sp_offs;
	pos_sp_offs.zero();

	if (_control_mode->data().flag_control_position_enabled) {
		pos_sp_offs(0) = (_pos_sp(0) - _pos(0)) / _params.sp_offs_max(0);
		pos_sp_offs(1) = (_pos_sp(1) - _pos(1)) / _params.sp_offs_max(1);
	}

	if (_control_mode->data().flag_control_altitude_enabled) {
		pos_sp_offs(2) = (_pos_sp(2) - _pos(2)) / _params.sp_offs_max(2);
	}

	float pos_sp_offs_norm = pos_sp_offs.length();

	if (pos_sp_offs_norm > 1.0f) {
		pos_sp_offs /= pos_sp_offs_norm;
		_pos_sp = _pos + pos_sp_offs.emult(_params.sp_offs_max);
	}
}

void
MulticopterPositionControlMultiplatform::control_offboard(float dt)
{
	if (_pos_sp_triplet->data().current.valid) {
		if (_control_mode->data().flag_control_position_enabled && _pos_sp_triplet->data().current.position_valid) {
			/* control position */
			_pos_sp(0) = _pos_sp_triplet->data().current.x;
			_pos_sp(1) = _pos_sp_triplet->data().current.y;

		} else if (_control_mode->data().flag_control_velocity_enabled && _pos_sp_triplet->data().current.velocity_valid) {
			/* control velocity */
			/* reset position setpoint to current position if needed */
			reset_pos_sp();

			/* set position setpoint move rate */
			_sp_move_rate(0) = _pos_sp_triplet->data().current.vx;
			_sp_move_rate(1) = _pos_sp_triplet->data().current.vy;
		}

		if (_pos_sp_triplet->data().current.yaw_valid) {
			_att_sp_msg.data().yaw_body = _pos_sp_triplet->data().current.yaw;

		} else if (_pos_sp_triplet->data().current.yawspeed_valid) {
			_att_sp_msg.data().yaw_body = _att_sp_msg.data().yaw_body + _pos_sp_triplet->data().current.yawspeed * dt;
		}

		if (_control_mode->data().flag_control_altitude_enabled && _pos_sp_triplet->data().current.position_valid) {
			/* Control altitude */
			_pos_sp(2) = _pos_sp_triplet->data().current.z;

		} else if (_control_mode->data().flag_control_climb_rate_enabled && _pos_sp_triplet->data().current.velocity_valid) {
			/* reset alt setpoint to current altitude if needed */
			reset_alt_sp();

			/* set altitude setpoint move rate */
			_sp_move_rate(2) = _pos_sp_triplet->data().current.vz;
		}

		/* feed forward setpoint move rate with weight vel_ff */
		_vel_ff = _sp_move_rate.emult(_params.vel_ff);

		/* move position setpoint */
		_pos_sp += _sp_move_rate * dt;

	} else {
		reset_pos_sp();
		reset_alt_sp();
	}
}

bool
MulticopterPositionControlMultiplatform::cross_sphere_line(const math::Vector<3> &sphere_c, const float sphere_r,
		const math::Vector<3> &line_a, const math::Vector<3> &line_b, math::Vector<3> &res)
{
	/* project center of sphere on line */
	/* normalized AB */
	math::Vector<3> ab_norm = line_b - line_a;
	ab_norm.normalize();
	math::Vector<3> d = line_a + ab_norm * ((sphere_c - line_a) * ab_norm);
	float cd_len = (sphere_c - d).length();

	/* we have triangle CDX with known CD and CX = R, find DX */
	if (sphere_r > cd_len) {
		/* have two roots, select one in A->B direction from D */
		float dx_len = sqrtf(sphere_r * sphere_r - cd_len * cd_len);
		res = d + ab_norm * dx_len;
		return true;

	} else {
		/* have no roots, return D */
		res = d;
		return false;
	}
}

void
MulticopterPositionControlMultiplatform::control_auto(float dt)
{
	if (!_mode_auto) {
		_mode_auto = true;
		/* reset position setpoint on AUTO mode activation */
		reset_pos_sp();
		reset_alt_sp();
	}

	if (_pos_sp_triplet->data().current.valid) {
		/* in case of interrupted mission don't go to waypoint but stay at current position */
		_reset_pos_sp = true;
		_reset_alt_sp = true;

		/* project setpoint to local frame */
		math::Vector<3> curr_sp;
		map_projection_project(&_ref_pos,
				       _pos_sp_triplet->data().current.lat, _pos_sp_triplet->data().current.lon,
				       &curr_sp.data[0], &curr_sp.data[1]);
		curr_sp(2) = -(_pos_sp_triplet->data().current.alt - _ref_alt);

		/* scaled space: 1 == position error resulting max allowed speed, L1 = 1 in this space */
		math::Vector<3> scale = _params.pos_p.edivide(_params.vel_max);	// TODO add mult param here

		/* convert current setpoint to scaled space */
		math::Vector<3> curr_sp_s = curr_sp.emult(scale);

		/* by default use current setpoint as is */
		math::Vector<3> pos_sp_s = curr_sp_s;

		if (_pos_sp_triplet->data().current.type == _pos_sp_triplet->data().current.SETPOINT_TYPE_POSITION
		    && _pos_sp_triplet->data().previous.valid) {
			/* follow "previous - current" line */
			math::Vector<3> prev_sp;
			map_projection_project(&_ref_pos,
					       _pos_sp_triplet->data().previous.lat, _pos_sp_triplet->data().previous.lon,
					       &prev_sp.data[0], &prev_sp.data[1]);
			prev_sp(2) = -(_pos_sp_triplet->data().previous.alt - _ref_alt);

			if ((curr_sp - prev_sp).length() > MIN_DIST) {

				/* find X - cross point of L1 sphere and trajectory */
				math::Vector<3> pos_s = _pos.emult(scale);
				math::Vector<3> prev_sp_s = prev_sp.emult(scale);
				math::Vector<3> prev_curr_s = curr_sp_s - prev_sp_s;
				math::Vector<3> curr_pos_s = pos_s - curr_sp_s;
				float curr_pos_s_len = curr_pos_s.length();

				if (curr_pos_s_len < 1.0f) {
					/* copter is closer to waypoint than L1 radius */
					/* check next waypoint and use it to avoid slowing down when passing via waypoint */
					if (_pos_sp_triplet->data().next.valid) {
						math::Vector<3> next_sp;
						map_projection_project(&_ref_pos,
								       _pos_sp_triplet->data().next.lat, _pos_sp_triplet->data().next.lon,
								       &next_sp.data[0], &next_sp.data[1]);
						next_sp(2) = -(_pos_sp_triplet->data().next.alt - _ref_alt);

						if ((next_sp - curr_sp).length() > MIN_DIST) {
							math::Vector<3> next_sp_s = next_sp.emult(scale);

							/* calculate angle prev - curr - next */
							math::Vector<3> curr_next_s = next_sp_s - curr_sp_s;
							math::Vector<3> prev_curr_s_norm = prev_curr_s.normalized();

							/* cos(a) * curr_next, a = angle between current and next trajectory segments */
							float cos_a_curr_next = prev_curr_s_norm * curr_next_s;

							/* cos(b), b = angle pos - curr_sp - prev_sp */
							float cos_b = -curr_pos_s * prev_curr_s_norm / curr_pos_s_len;

							if (cos_a_curr_next > 0.0f && cos_b > 0.0f) {
								float curr_next_s_len = curr_next_s.length();

								/* if curr - next distance is larger than L1 radius, limit it */
								if (curr_next_s_len > 1.0f) {
									cos_a_curr_next /= curr_next_s_len;
								}

								/* feed forward position setpoint offset */
								math::Vector<3> pos_ff = prev_curr_s_norm *
											 cos_a_curr_next * cos_b * cos_b * (1.0f - curr_pos_s_len) *
											 (1.0f - expf(-curr_pos_s_len * curr_pos_s_len * 20.0f));
								pos_sp_s += pos_ff;
							}
						}
					}

				} else {
					bool near = cross_sphere_line(pos_s, 1.0f, prev_sp_s, curr_sp_s, pos_sp_s);

					if (near) {
						/* L1 sphere crosses trajectory */

					} else {
						/* copter is too far from trajectory */
						/* if copter is behind prev waypoint, go directly to prev waypoint */
						if ((pos_sp_s - prev_sp_s) * prev_curr_s < 0.0f) {
							pos_sp_s = prev_sp_s;
						}

						/* if copter is in front of curr waypoint, go directly to curr waypoint */
						if ((pos_sp_s - curr_sp_s) * prev_curr_s > 0.0f) {
							pos_sp_s = curr_sp_s;
						}

						pos_sp_s = pos_s + (pos_sp_s - pos_s).normalized();
					}
				}
			}
		}

		/* move setpoint not faster than max allowed speed */
		math::Vector<3> pos_sp_old_s = _pos_sp.emult(scale);

		/* difference between current and desired position setpoints, 1 = max speed */
		math::Vector<3> d_pos_m = (pos_sp_s - pos_sp_old_s).edivide(_params.pos_p);
		float d_pos_m_len = d_pos_m.length();

		if (d_pos_m_len > dt) {
			pos_sp_s = pos_sp_old_s + (d_pos_m / d_pos_m_len * dt).emult(_params.pos_p);
		}

		/* scale result back to normal space */
		_pos_sp = pos_sp_s.edivide(scale);

		/* update yaw setpoint if needed */
		if (PX4_ISFINITE(_pos_sp_triplet->data().current.yaw)) {
			_att_sp_msg.data().yaw_body = _pos_sp_triplet->data().current.yaw;
		}

	} else {
		/* no waypoint, do nothing, setpoint was already reset */
	}
}

void MulticopterPositionControlMultiplatform::handle_parameter_update(const px4_parameter_update &msg)
{
	parameters_update();
}

void MulticopterPositionControlMultiplatform::handle_position_setpoint_triplet(const px4_position_setpoint_triplet &msg)
{
	/* Make sure that the position setpoint is valid */
	if (!PX4_ISFINITE(_pos_sp_triplet->data().current.lat) ||
	    !PX4_ISFINITE(_pos_sp_triplet->data().current.lon) ||
	    !PX4_ISFINITE(_pos_sp_triplet->data().current.alt)) {
		_pos_sp_triplet->data().current.valid = false;
	}
}

void  MulticopterPositionControlMultiplatform::handle_vehicle_attitude(const px4_vehicle_attitude &msg)
{
	static bool reset_int_z = true;
	static bool reset_int_z_manual = false;
	static bool reset_int_xy = true;
	static bool reset_yaw_sp = true;
	static bool was_armed = false;
	static uint64_t t_prev = 0;

	matrix::Eulerf euler = matrix::Quatf(_att->data().q);
	matrix::Dcmf R = matrix::Quatf(_att->data().q);

	uint64_t t = get_time_micros();
	float dt = t_prev != 0 ? (t - t_prev) * 0.000001f : 0.005f;
	t_prev = t;

	if (_control_mode->data().flag_armed && !was_armed) {
		/* reset setpoints and integrals on arming */
		_reset_pos_sp = true;
		_reset_alt_sp = true;
		reset_int_z = true;
		reset_int_xy = true;
		reset_yaw_sp = true;
	}

	/* Update previous arming state */
	was_armed = _control_mode->data().flag_armed;

	update_ref();

	if (_control_mode->data().flag_control_altitude_enabled ||
	    _control_mode->data().flag_control_position_enabled ||
	    _control_mode->data().flag_control_climb_rate_enabled ||
	    _control_mode->data().flag_control_velocity_enabled) {

		_pos(0) = _local_pos->data().x;
		_pos(1) = _local_pos->data().y;
		_pos(2) = _local_pos->data().z;

		_vel(0) = _local_pos->data().vx;
		_vel(1) = _local_pos->data().vy;
		_vel(2) = _local_pos->data().vz;

		_vel_ff.zero();
		_sp_move_rate.zero();

		/* select control source */
		if (_control_mode->data().flag_control_manual_enabled) {
			/* manual control */
			control_manual(dt);
			_mode_auto = false;

		} else if (_control_mode->data().flag_control_offboard_enabled) {
			/* offboard control */
			control_offboard(dt);
			_mode_auto = false;

		} else {
			/* AUTO */
			control_auto(dt);
		}

		if (!_control_mode->data().flag_control_manual_enabled && _pos_sp_triplet->data().current.valid
		    && _pos_sp_triplet->data().current.type == _pos_sp_triplet->data().current.SETPOINT_TYPE_IDLE) {
			/* idle state, don't run controller and set zero thrust */
			_R.identity();
			math::Quaternion qd;
			qd.from_dcm(_R);
			memcpy(&_att_sp_msg.data().q_d[0], qd.data, sizeof(_att_sp_msg.data().q_d));
			_att_sp_msg.data().q_d_valid = true;

			_att_sp_msg.data().roll_body = 0.0f;
			_att_sp_msg.data().pitch_body = 0.0f;
			_att_sp_msg.data().yaw_body = euler.psi();
			_att_sp_msg.data().thrust = 0.0f;

			_att_sp_msg.data().timestamp = get_time_micros();

			/* publish attitude setpoint */
			if (_att_sp_pub != nullptr) {
				_att_sp_pub->publish(_att_sp_msg);

			} else {
				_att_sp_pub = _n.advertise<px4_vehicle_attitude_setpoint>();
			}

		} else {
			/* run position & altitude controllers, calculate velocity setpoint */
			math::Vector<3> pos_err = _pos_sp - _pos;

			_vel_sp = pos_err.emult(_params.pos_p) + _vel_ff;

			/* make sure velocity setpoint is saturated in xy*/
			float vel_norm_xy = sqrtf(_vel_sp(0) * _vel_sp(0) +
						  _vel_sp(1) * _vel_sp(1));

			if (vel_norm_xy > _params.vel_max(0)) {
				/* note assumes vel_max(0) == vel_max(1) */
				_vel_sp(0) = _vel_sp(0) * _params.vel_max(0) / vel_norm_xy;
				_vel_sp(1) = _vel_sp(1) * _params.vel_max(1) / vel_norm_xy;
			}

			/* make sure velocity setpoint is saturated in z*/
			float vel_norm_z = sqrtf(_vel_sp(2) * _vel_sp(2));

			if (vel_norm_z > _params.vel_max(2)) {
				_vel_sp(2) = _vel_sp(2) * _params.vel_max(2) / vel_norm_z;
			}

			if (!_control_mode->data().flag_control_altitude_enabled) {
				_reset_alt_sp = true;
				_vel_sp(2) = 0.0f;
			}

			if (!_control_mode->data().flag_control_position_enabled) {
				_reset_pos_sp = true;
				_vel_sp(0) = 0.0f;
				_vel_sp(1) = 0.0f;
			}

			/* use constant descend rate when landing, ignore altitude setpoint */
			if (!_control_mode->data().flag_control_manual_enabled && _pos_sp_triplet->data().current.valid
			    && _pos_sp_triplet->data().current.type == _pos_sp_triplet->data().current.SETPOINT_TYPE_LAND) {
				_vel_sp(2) = _params.land_speed;
			}

			_global_vel_sp_msg.data().vx = _vel_sp(0);
			_global_vel_sp_msg.data().vy = _vel_sp(1);
			_global_vel_sp_msg.data().vz = _vel_sp(2);

			/* publish velocity setpoint */
			if (_global_vel_sp_pub != nullptr) {
				_global_vel_sp_pub->publish(_global_vel_sp_msg);

			} else {
				_global_vel_sp_pub = _n.advertise<px4_vehicle_global_velocity_setpoint>();
			}

			if (_control_mode->data().flag_control_climb_rate_enabled || _control_mode->data().flag_control_velocity_enabled) {
				/* reset integrals if needed */
				if (_control_mode->data().flag_control_climb_rate_enabled) {
					if (reset_int_z) {
						reset_int_z = false;
						float i = _params.thr_min;

						if (reset_int_z_manual) {
							i = _manual_control_sp->data().z;

							if (i < _params.thr_min) {
								i = _params.thr_min;

							} else if (i > _params.thr_max) {
								i = _params.thr_max;
							}
						}

						_thrust_int(2) = -i;
					}

				} else {
					reset_int_z = true;
				}

				if (_control_mode->data().flag_control_velocity_enabled) {
					if (reset_int_xy) {
						reset_int_xy = false;
						_thrust_int(0) = 0.0f;
						_thrust_int(1) = 0.0f;
					}

				} else {
					reset_int_xy = true;
				}

				/* velocity error */
				math::Vector<3> vel_err = _vel_sp - _vel;

				/* derivative of velocity error, not includes setpoint acceleration */
				math::Vector<3> vel_err_d = (_sp_move_rate - _vel).emult(_params.pos_p) - (_vel - _vel_prev) / dt;
				_vel_prev = _vel;

				/* thrust vector in NED frame */
				math::Vector<3> thrust_sp = vel_err.emult(_params.vel_p) + vel_err_d.emult(_params.vel_d) + _thrust_int;

				if (!_control_mode->data().flag_control_velocity_enabled) {
					thrust_sp(0) = 0.0f;
					thrust_sp(1) = 0.0f;
				}

				if (!_control_mode->data().flag_control_climb_rate_enabled) {
					thrust_sp(2) = 0.0f;
				}

				/* limit thrust vector and check for saturation */
				bool saturation_xy = false;
				bool saturation_z = false;

				/* limit min lift */
				float thr_min = _params.thr_min;

				if (!_control_mode->data().flag_control_velocity_enabled && thr_min < 0.0f) {
					/* don't allow downside thrust direction in manual attitude mode */
					thr_min = 0.0f;
				}

				float tilt_max = _params.tilt_max_air;

				/* adjust limits for landing mode */
				if (!_control_mode->data().flag_control_manual_enabled && _pos_sp_triplet->data().current.valid &&
				    _pos_sp_triplet->data().current.type == _pos_sp_triplet->data().current.SETPOINT_TYPE_LAND) {
					/* limit max tilt and min lift when landing */
					tilt_max = _params.tilt_max_land;

					if (thr_min < 0.0f) {
						thr_min = 0.0f;
					}
				}

				/* limit min lift */
				if (-thrust_sp(2) < thr_min) {
					thrust_sp(2) = -thr_min;
					saturation_z = true;
				}

				if (_control_mode->data().flag_control_velocity_enabled) {
					/* limit max tilt */
					if (thr_min >= 0.0f && tilt_max < M_PI_F / 2 - 0.05f) {
						/* absolute horizontal thrust */
						float thrust_sp_xy_len = math::Vector<2>(thrust_sp(0), thrust_sp(1)).length();

						if (thrust_sp_xy_len > 0.01f) {
							/* max horizontal thrust for given vertical thrust*/
							float thrust_xy_max = -thrust_sp(2) * tanf(tilt_max);

							if (thrust_sp_xy_len > thrust_xy_max) {
								float k = thrust_xy_max / thrust_sp_xy_len;
								thrust_sp(0) *= k;
								thrust_sp(1) *= k;
								saturation_xy = true;
							}
						}
					}

				} else {
					/* thrust compensation for altitude only control mode */
					float att_comp;

					if (R(2, 2) > TILT_COS_MAX) {
						att_comp = 1.0f / R(2, 2);

					} else if (R(2, 2) > 0.0f) {
						att_comp = ((1.0f / TILT_COS_MAX - 1.0f) / TILT_COS_MAX) * R(2, 2) + 1.0f;
						saturation_z = true;

					} else {
						att_comp = 1.0f;
						saturation_z = true;
					}

					thrust_sp(2) *= att_comp;
				}

				/* limit max thrust */
				float thrust_abs = thrust_sp.length();

				if (thrust_abs > _params.thr_max) {
					if (thrust_sp(2) < 0.0f) {
						if (-thrust_sp(2) > _params.thr_max) {
							/* thrust Z component is too large, limit it */
							thrust_sp(0) = 0.0f;
							thrust_sp(1) = 0.0f;
							thrust_sp(2) = -_params.thr_max;
							saturation_xy = true;
							saturation_z = true;

						} else {
							/* preserve thrust Z component and lower XY, keeping altitude is more important than position */
							float thrust_xy_max = sqrtf(_params.thr_max * _params.thr_max - thrust_sp(2) * thrust_sp(2));
							float thrust_xy_abs = math::Vector<2>(thrust_sp(0), thrust_sp(1)).length();
							float k = thrust_xy_max / thrust_xy_abs;
							thrust_sp(0) *= k;
							thrust_sp(1) *= k;
							saturation_xy = true;
						}

					} else {
						/* Z component is negative, going down, simply limit thrust vector */
						float k = _params.thr_max / thrust_abs;
						thrust_sp *= k;
						saturation_xy = true;
						saturation_z = true;
					}

					thrust_abs = _params.thr_max;
				}

				/* update integrals */
				if (_control_mode->data().flag_control_velocity_enabled && !saturation_xy) {
					_thrust_int(0) += vel_err(0) * _params.vel_i(0) * dt;
					_thrust_int(1) += vel_err(1) * _params.vel_i(1) * dt;
				}

				if (_control_mode->data().flag_control_climb_rate_enabled && !saturation_z) {
					_thrust_int(2) += vel_err(2) * _params.vel_i(2) * dt;

					/* protection against flipping on ground when landing */
					if (_thrust_int(2) > 0.0f) {
						_thrust_int(2) = 0.0f;
					}
				}

				/* calculate attitude setpoint from thrust vector */
				if (_control_mode->data().flag_control_velocity_enabled) {
					/* desired body_z axis = -normalize(thrust_vector) */
					math::Vector<3> body_x;
					math::Vector<3> body_y;
					math::Vector<3> body_z;

					if (thrust_abs > SIGMA) {
						body_z = -thrust_sp / thrust_abs;

					} else {
						/* no thrust, set Z axis to safe value */
						body_z.zero();
						body_z(2) = 1.0f;
					}

					/* vector of desired yaw direction in XY plane, rotated by PI/2 */
					math::Vector<3> y_C(-sinf(_att_sp_msg.data().yaw_body), cosf(_att_sp_msg.data().yaw_body), 0.0f);

					if (fabsf(body_z(2)) > SIGMA) {
						/* desired body_x axis, orthogonal to body_z */
						body_x = y_C % body_z;

						/* keep nose to front while inverted upside down */
						if (body_z(2) < 0.0f) {
							body_x = -body_x;
						}

						body_x.normalize();

					} else {
						/* desired thrust is in XY plane, set X downside to construct correct matrix,
							* but yaw component will not be used actually */
						body_x.zero();
						body_x(2) = 1.0f;
					}

					/* desired body_y axis */
					body_y = body_z % body_x;

					/* fill rotation matrix */
					for (int i = 0; i < 3; i++) {
						_R(i, 0) = body_x(i);
						_R(i, 1) = body_y(i);
						_R(i, 2) = body_z(i);
					}

					math::Quaternion q;
					q.from_dcm(_R);
					memcpy(_att_sp_msg.data().q_d, q.data, sizeof(_att_sp_msg.data().q_d));
					_att_sp_msg.data().q_d_valid = true;

					/* calculate euler angles, for logging only, must not be used for control */
					math::Vector<3> eul = _R.to_euler();
					_att_sp_msg.data().roll_body = eul(0);
					_att_sp_msg.data().pitch_body = eul(1);
					/* yaw already used to construct rot matrix, but actual rotation matrix can have different yaw near singularity */

				} else if (!_control_mode->data().flag_control_manual_enabled) {
					/* autonomous altitude control without position control (failsafe landing),
						* force level attitude, don't change yaw */
					_R.from_euler(0.0f, 0.0f, _att_sp_msg.data().yaw_body);

					math::Quaternion q;
					q.from_dcm(_R);
					memcpy(_att_sp_msg.data().q_d, q.data, sizeof(_att_sp_msg.data().q_d));
					_att_sp_msg.data().q_d_valid = true;

					_att_sp_msg.data().roll_body = 0.0f;
					_att_sp_msg.data().pitch_body = 0.0f;
				}

				_att_sp_msg.data().thrust = thrust_abs;

				/* save thrust setpoint for logging */
				_local_pos_sp_msg.data().acc_x = thrust_sp(0);
				_local_pos_sp_msg.data().acc_y = thrust_sp(1);
				_local_pos_sp_msg.data().acc_z = thrust_sp(2);

				_att_sp_msg.data().timestamp = get_time_micros();

				/* publish attitude setpoint */
				if (_att_sp_pub != nullptr) {
					_att_sp_pub->publish(_att_sp_msg);

				} else {
					_att_sp_pub = _n.advertise<px4_vehicle_attitude_setpoint>();
				}

			} else {
				reset_int_z = true;
			}
		}

		/* fill local position setpoint */
		_local_pos_sp_msg.data().timestamp = get_time_micros();
		_local_pos_sp_msg.data().x = _pos_sp(0);
		_local_pos_sp_msg.data().y = _pos_sp(1);
		_local_pos_sp_msg.data().z = _pos_sp(2);
		_local_pos_sp_msg.data().yaw = _att_sp_msg.data().yaw_body;
		_local_pos_sp_msg.data().vx = _vel_sp(0);
		_local_pos_sp_msg.data().vy = _vel_sp(1);
		_local_pos_sp_msg.data().vz = _vel_sp(2);

		/* publish local position setpoint */
		if (_local_pos_sp_pub != nullptr) {
			_local_pos_sp_pub->publish(_local_pos_sp_msg);

		} else {
			_local_pos_sp_pub = _n.advertise<px4_vehicle_local_position_setpoint>();
		}


	} else {
		/* position controller disabled, reset setpoints */
		_reset_alt_sp = true;
		_reset_pos_sp = true;
		_mode_auto = false;
		reset_int_z = true;
		reset_int_xy = true;
	}

	/* generate attitude setpoint from manual controls */
	if (_control_mode->data().flag_control_manual_enabled && _control_mode->data().flag_control_attitude_enabled) {

		/* reset yaw setpoint to current position if needed */
		if (reset_yaw_sp) {
			reset_yaw_sp = false;
			_att_sp_msg.data().yaw_body = euler.psi();
		}

		/* do not move yaw while arming */
		else if (_manual_control_sp->data().z > 0.1f) {
			const float YAW_OFFSET_MAX = _params.man_yaw_max / _params.mc_att_yaw_p;

			_att_sp_msg.data().yaw_sp_move_rate = _manual_control_sp->data().r * _params.man_yaw_max;
			_att_sp_msg.data().yaw_body = _wrap_pi(_att_sp_msg.data().yaw_body + _att_sp_msg.data().yaw_sp_move_rate * dt);
			float yaw_offs = _wrap_pi(_att_sp_msg.data().yaw_body - euler.psi());

			if (yaw_offs < - YAW_OFFSET_MAX) {
				_att_sp_msg.data().yaw_body = _wrap_pi(euler.psi() - YAW_OFFSET_MAX);

			} else if (yaw_offs > YAW_OFFSET_MAX) {
				_att_sp_msg.data().yaw_body = _wrap_pi(euler.psi() + YAW_OFFSET_MAX);
			}
		}

		/* Control roll and pitch directly if we no aiding velocity controller is active */
		if (!_control_mode->data().flag_control_velocity_enabled) {
			_att_sp_msg.data().roll_body = _manual_control_sp->data().y * _params.man_roll_max;
			_att_sp_msg.data().pitch_body = -_manual_control_sp->data().x * _params.man_pitch_max;
		}

		/* Control climb rate directly if no aiding altitude controller is active */
		if (!_control_mode->data().flag_control_climb_rate_enabled) {
			_att_sp_msg.data().thrust = _manual_control_sp->data().z;
		}

		/* Construct attitude setpoint rotation matrix */
		math::Matrix<3, 3> R_sp;
		R_sp.from_euler(_att_sp_msg.data().roll_body, _att_sp_msg.data().pitch_body, _att_sp_msg.data().yaw_body);
		math::Quaternion q;
		q.from_dcm(R_sp);
		memcpy(_att_sp_msg.data().q_d, q.data, sizeof(_att_sp_msg.data().q_d));
		_att_sp_msg.data().q_d_valid = true;
		_att_sp_msg.data().timestamp = get_time_micros();

	} else {
		reset_yaw_sp = true;
	}

	/* publish attitude setpoint
	 * Do not publish if offboard is enabled but position/velocity control is disabled, in this case the attitude setpoint
	 * is published by the mavlink app
	 */
	if (!(_control_mode->data().flag_control_offboard_enabled &&
	      !(_control_mode->data().flag_control_position_enabled ||
		_control_mode->data().flag_control_velocity_enabled))) {
		if (_att_sp_pub != nullptr) {
			_att_sp_pub->publish(_att_sp_msg);

		} else {
			_att_sp_pub = _n.advertise<px4_vehicle_attitude_setpoint>();
		}
	}

	/* reset altitude controller integral (hovering throttle) to manual throttle after manual throttle control */
	reset_int_z_manual = _control_mode->data().flag_armed && _control_mode->data().flag_control_manual_enabled
			     && !_control_mode->data().flag_control_climb_rate_enabled;
}
