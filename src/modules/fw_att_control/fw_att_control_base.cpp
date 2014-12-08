/* Copyright (c) 2014 PX4 Development Team. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in
* the documentation and/or other materials provided with the
* distribution.
* 3. Neither the name PX4 nor the names of its contributors may be
* used to endorse or promote products derived from this software
* without specific prior written permission.
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
 * @file mc_att_control_base.cpp
 *
 * @author Roman Bapst <bapstr@ethz.ch>
 *
 */

#include "fw_att_control_base.h"
#include <math.h>
#include <mathlib/mathlib.h>
#include <drivers/drv_hrt.h>

using namespace std;

FixedwingAttitudeControlBase::FixedwingAttitudeControlBase() :

		_task_should_exit(false), _task_running(false), _control_task(-1),

		/* performance counters */
		_loop_perf(perf_alloc(PC_ELAPSED, "fw att control")), _nonfinite_input_perf(
				perf_alloc(PC_COUNT, "fw att control nonfinite input")), _nonfinite_output_perf(
				perf_alloc(PC_COUNT, "fw att control nonfinite output")),
		/* states */
		_setpoint_valid(false), _debug(false) {
	/* safely initialize structs */
	_att = {};
	_att_sp = {};
	_manual = {};
	_airspeed = {};
	_vcontrol_mode = {};
	_actuators = {};
	_actuators_airframe = {};
	_global_pos = {};
	_vehicle_status = {};

}

FixedwingAttitudeControlBase::~FixedwingAttitudeControlBase() {

}

void FixedwingAttitudeControlBase::control_attitude() {
	bool lock_integrator = false;
	static int loop_counter = 0;
	/* scale around tuning airspeed */

	float airspeed;

	/* if airspeed is not updating, we assume the normal average speed */
	if (bool nonfinite = !isfinite(_airspeed.true_airspeed_m_s)
			|| hrt_elapsed_time(&_airspeed.timestamp) > 1e6) {
		airspeed = _parameters.airspeed_trim;
		if (nonfinite) {
			perf_count(_nonfinite_input_perf);
		}
	} else {
		/* prevent numerical drama by requiring 0.5 m/s minimal speed */
		airspeed = math::max(0.5f, _airspeed.true_airspeed_m_s);
	}

	/*
	 * For scaling our actuators using anything less than the min (close to stall)
	 * speed doesn't make any sense - its the strongest reasonable deflection we
	 * want to do in flight and its the baseline a human pilot would choose.
	 *
	 * Forcing the scaling to this value allows reasonable handheld tests.
	 */

	float airspeed_scaling = _parameters.airspeed_trim
			/ ((airspeed < _parameters.airspeed_min) ?
					_parameters.airspeed_min : airspeed);

	float roll_sp = _parameters.rollsp_offset_rad;
	float pitch_sp = _parameters.pitchsp_offset_rad;
	float throttle_sp = 0.0f;

	if (_vcontrol_mode.flag_control_velocity_enabled
			|| _vcontrol_mode.flag_control_position_enabled) {
		/* read in attitude setpoint from attitude setpoint uorb topic */
		roll_sp = _att_sp.roll_body + _parameters.rollsp_offset_rad;
		pitch_sp = _att_sp.pitch_body + _parameters.pitchsp_offset_rad;
		throttle_sp = _att_sp.thrust;

		/* reset integrals where needed */
		if (_att_sp.roll_reset_integral) {
			_roll_ctrl.reset_integrator();
		}
		if (_att_sp.pitch_reset_integral) {
			_pitch_ctrl.reset_integrator();
		}
		if (_att_sp.yaw_reset_integral) {
			_yaw_ctrl.reset_integrator();
		}
	} else {
		/*
		 * Scale down roll and pitch as the setpoints are radians
		 * and a typical remote can only do around 45 degrees, the mapping is
		 * -1..+1 to -man_roll_max rad..+man_roll_max rad (equivalent for pitch)
		 *
		 * With this mapping the stick angle is a 1:1 representation of
		 * the commanded attitude.
		 *
		 * The trim gets subtracted here from the manual setpoint to get
		 * the intended attitude setpoint. Later, after the rate control step the
		 * trim is added again to the control signal.
		 */
		roll_sp = (_manual.y * _parameters.man_roll_max - _parameters.trim_roll)
				+ _parameters.rollsp_offset_rad;
		pitch_sp = -(_manual.x * _parameters.man_pitch_max
				- _parameters.trim_pitch) + _parameters.pitchsp_offset_rad;
		throttle_sp = _manual.z;
		_actuators.control[4] = _manual.flaps;

		/*
		 * in manual mode no external source should / does emit attitude setpoints.
		 * emit the manual setpoint here to allow attitude controller tuning
		 * in attitude control mode.
		 */
		struct vehicle_attitude_setpoint_s att_sp;
		att_sp.timestamp = hrt_absolute_time();
		att_sp.roll_body = roll_sp;
		att_sp.pitch_body = pitch_sp;
		att_sp.yaw_body = 0.0f - _parameters.trim_yaw;
		att_sp.thrust = throttle_sp;

	}

	/* If the aircraft is on ground reset the integrators */
	if (_vehicle_status.condition_landed) {
		_roll_ctrl.reset_integrator();
		_pitch_ctrl.reset_integrator();
		_yaw_ctrl.reset_integrator();
	}

	/* Prepare speed_body_u and speed_body_w */
	float speed_body_u = 0.0f;
	float speed_body_v = 0.0f;
	float speed_body_w = 0.0f;
	if (_att.R_valid) {
		speed_body_u = _att.R[0][0] * _global_pos.vel_n
				+ _att.R[1][0] * _global_pos.vel_e
				+ _att.R[2][0] * _global_pos.vel_d;
		speed_body_v = _att.R[0][1] * _global_pos.vel_n
				+ _att.R[1][1] * _global_pos.vel_e
				+ _att.R[2][1] * _global_pos.vel_d;
		speed_body_w = _att.R[0][2] * _global_pos.vel_n
				+ _att.R[1][2] * _global_pos.vel_e
				+ _att.R[2][2] * _global_pos.vel_d;
	} else {
		if (_debug && loop_counter % 10 == 0) {
			warnx("Did not get a valid R\n");
		}
	}

	/* Run attitude controllers */
	if (isfinite(roll_sp) && isfinite(pitch_sp)) {
		_roll_ctrl.control_attitude(roll_sp, _att.roll);
		_pitch_ctrl.control_attitude(pitch_sp, _att.roll, _att.pitch, airspeed);
		_yaw_ctrl.control_attitude(_att.roll, _att.pitch, speed_body_u,
				speed_body_v, speed_body_w, _roll_ctrl.get_desired_rate(),
				_pitch_ctrl.get_desired_rate()); //runs last, because is depending on output of roll and pitch attitude

		/* Run attitude RATE controllers which need the desired attitudes from above, add trim */
		float roll_u = _roll_ctrl.control_bodyrate(_att.pitch, _att.rollspeed,
				_att.yawspeed, _yaw_ctrl.get_desired_rate(),
				_parameters.airspeed_min, _parameters.airspeed_max, airspeed,
				airspeed_scaling, lock_integrator);
		_actuators.control[0] =
				(isfinite(roll_u)) ?
						roll_u + _parameters.trim_roll : _parameters.trim_roll;
		if (!isfinite(roll_u)) {
			_roll_ctrl.reset_integrator();
			perf_count(_nonfinite_output_perf);

			if (_debug && loop_counter % 10 == 0) {
				warnx("roll_u %.4f", (double) roll_u);
			}
		}

		float pitch_u = _pitch_ctrl.control_bodyrate(_att.roll, _att.pitch,
				_att.pitchspeed, _att.yawspeed, _yaw_ctrl.get_desired_rate(),
				_parameters.airspeed_min, _parameters.airspeed_max, airspeed,
				airspeed_scaling, lock_integrator);
		_actuators.control[1] =
				(isfinite(pitch_u)) ?
						pitch_u + _parameters.trim_pitch :
						_parameters.trim_pitch;
		if (!isfinite(pitch_u)) {
			_pitch_ctrl.reset_integrator();
			perf_count(_nonfinite_output_perf);
			if (_debug && loop_counter % 10 == 0) {
				warnx("pitch_u %.4f, _yaw_ctrl.get_desired_rate() %.4f,"
						" airspeed %.4f, airspeed_scaling %.4f,"
						" roll_sp %.4f, pitch_sp %.4f,"
						" _roll_ctrl.get_desired_rate() %.4f,"
						" _pitch_ctrl.get_desired_rate() %.4f"
						" att_sp.roll_body %.4f", (double) pitch_u,
						(double) _yaw_ctrl.get_desired_rate(),
						(double) airspeed, (double) airspeed_scaling,
						(double) roll_sp, (double) pitch_sp,
						(double) _roll_ctrl.get_desired_rate(),
						(double) _pitch_ctrl.get_desired_rate(),
						(double) _att_sp.roll_body);
			}
		}

		float yaw_u = _yaw_ctrl.control_bodyrate(_att.roll, _att.pitch,
				_att.pitchspeed, _att.yawspeed, _pitch_ctrl.get_desired_rate(),
				_parameters.airspeed_min, _parameters.airspeed_max, airspeed,
				airspeed_scaling, lock_integrator);
		_actuators.control[2] =
				(isfinite(yaw_u)) ?
						yaw_u + _parameters.trim_yaw : _parameters.trim_yaw;
		if (!isfinite(yaw_u)) {
			_yaw_ctrl.reset_integrator();
			perf_count(_nonfinite_output_perf);
			if (_debug && loop_counter % 10 == 0) {
				warnx("yaw_u %.4f", (double) yaw_u);
			}
		}

		/* throttle passed through */
		_actuators.control[3] = (isfinite(throttle_sp)) ? throttle_sp : 0.0f;
		if (!isfinite(throttle_sp)) {
			if (_debug && loop_counter % 10 == 0) {
				warnx("throttle_sp %.4f", (double) throttle_sp);
			}
		}
	} else {
		perf_count(_nonfinite_input_perf);
		if (_debug && loop_counter % 10 == 0) {
			warnx("Non-finite setpoint roll_sp: %.4f, pitch_sp %.4f",
					(double) roll_sp, (double) pitch_sp);
		}
	}

}

void FixedwingAttitudeControlBase::set_attitude(const Eigen::Quaternion<double> attitude) {
	// watch out, still need to see where we modify attitude for the tailsitter case
	math::Quaternion quat;
	quat(0) = (float)attitude.w();
	quat(1) = (float)attitude.x();
	quat(2) = (float)attitude.y();
	quat(3) = (float)attitude.z();

	_att.q[0] = quat(0);
	_att.q[1] = quat(1);
	_att.q[2] = quat(2);
	_att.q[3] = quat(3);

	math::Matrix<3,3> Rot = quat.to_dcm();
	_att.R[0][0] = Rot(0,0);
	_att.R[1][0] = Rot(1,0);
	_att.R[2][0] = Rot(2,0);
	_att.R[0][1] = Rot(0,1);
	_att.R[1][1] = Rot(1,1);
	_att.R[2][1] = Rot(2,1);
	_att.R[0][2] = Rot(0,2);
	_att.R[1][2] = Rot(1,2);
	_att.R[2][2] = Rot(2,2);

	_att.R_valid = true;
}
void FixedwingAttitudeControlBase::set_attitude_rates(const Eigen::Vector3d& angular_rate) {
	_att.rollspeed  = angular_rate(0);
	_att.pitchspeed = angular_rate(1);
	_att.yawspeed   = angular_rate(2);
}
void FixedwingAttitudeControlBase::set_attitude_reference(const Eigen::Vector4d& control_attitude_thrust_reference) {
	_att_sp.roll_body  = control_attitude_thrust_reference(0);
	_att_sp.pitch_body = control_attitude_thrust_reference(1);
	_att_sp.yaw_body   = control_attitude_thrust_reference(2);
	_att_sp.thrust     = (control_attitude_thrust_reference(3) -30)*(-1)/30;

	// setup rotation matrix
	math::Matrix<3,3> Rot_sp;
	Rot_sp.from_euler(_att_sp.roll_body,_att_sp.pitch_body,_att_sp.yaw_body);
	_att_sp.R_body[0][0] = Rot_sp(0,0);
	_att_sp.R_body[1][0] = Rot_sp(1,0);
	_att_sp.R_body[2][0] = Rot_sp(2,0);
	_att_sp.R_body[0][1] = Rot_sp(0,1);
	_att_sp.R_body[1][1] = Rot_sp(1,1);
	_att_sp.R_body[2][1] = Rot_sp(2,1);
	_att_sp.R_body[0][2] = Rot_sp(0,2);
	_att_sp.R_body[1][2] = Rot_sp(1,2);
	_att_sp.R_body[2][2] = Rot_sp(2,2);
}
void FixedwingAttitudeControlBase::get_mixer_input(Eigen::Vector4d& motor_inputs) {
	motor_inputs(0) = _actuators.control[0];
	motor_inputs(1) = _actuators.control[1];
	motor_inputs(2) = _actuators.control[2];
	motor_inputs(3) = _actuators.control[3];
}
