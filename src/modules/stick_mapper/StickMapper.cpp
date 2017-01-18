/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

#include "StickMapper.hpp"

#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <uORB/topics/fw_virtual_attitude_setpoint.h>
#include <uORB/topics/fw_virtual_rates_setpoint.h>

using math::constrain;
using math::min;
using math::max;
using math::radians;

using matrix::Eulerf;
using matrix::Quatf;

StickMapper::StickMapper() :
	SuperBlock(NULL, "SM"),

	// manual input scaling
	_man_roll_scale(0.0f),
	_man_pitch_scale(0.0f),
	_man_yaw_scale(0.0f),

	// acro maximum rates
	_acro_roll_max(0.0f),
	_acro_pitch_max(0.0f),
	_acro_yaw_max(0.0f),

	// roll and pitch offsets
	_man_roll_max(0.0f),
	_man_pitch_max(0.0f),

	_roll_offset(0.0f),
	_pitch_offset(0.0f),

	_taskShouldExit(false),
	_taskIsRunning(false),
	_work{},

	_polls(),
	_timestamp(hrt_absolute_time()),

	// subscriptions
	_manual_control_setpoint_sub(ORB_ID(manual_control_setpoint), 0, 0, &getSubscriptions()),
	_parameter_update_sub(ORB_ID(parameter_update), 0, 0, &getSubscriptions()),
	_vehicle_control_mode_sub(ORB_ID(vehicle_control_mode), 0, 0, &getSubscriptions()),
	_vehicle_status_sub(ORB_ID(vehicle_status), 0, 0, &getSubscriptions()),

	// publications
	_actuator_controls_0_pub(nullptr, -1, &getPublications()),
	_vehicle_attitude_setpoint_pub(nullptr, -1, &getPublications()),
	_vehicle_rates_setpoint_pub(nullptr, -1, &getPublications()),

	// trim params
	_trim_roll(nullptr, "TRIM_ROLL"),
	_trim_pitch(nullptr, "TRIM_PITCH"),
	_trim_yaw(nullptr, "TRIM_YAW")
{
	parameters_update();

	_polls[POLL_MANUAL_CONTROL].fd = _manual_control_setpoint_sub.getHandle();
	_polls[POLL_MANUAL_CONTROL].events = POLLIN;
	_polls[POLL_CONTROL_MODE].fd = _vehicle_control_mode_sub.getHandle();
	_polls[POLL_CONTROL_MODE].events = POLLIN;
}

StickMapper::~StickMapper()
{
}

void
StickMapper::parameters_update()
{
	updateParams();

	// FW/MC/VTOL specific param update depending on mode
}

void
StickMapper::start()
{
	_taskShouldExit = false;

	// schedule a cycle to start things
	work_queue(HPWORK, &_work, (worker_t)&StickMapper::cycle_trampoline, this, 0);
}

void StickMapper::stop()
{
	_taskShouldExit = true;
}

void
StickMapper::cycle_trampoline(void *arg)
{
	StickMapper *dev = reinterpret_cast<StickMapper *>(arg);
	dev->cycle();
}

void
StickMapper::cycle()
{
	_taskIsRunning = true;

	// wait for a state update, check for exit condition every 100 ms
	int ret = px4_poll(_polls, n_poll, 100);

	if (ret < 0) {
		return;
	}

	// calculate dt
	hrt_abstime new_timestamp = hrt_absolute_time();
	float dt = (new_timestamp - _timestamp) / 1.0e6f;
	_timestamp = new_timestamp;

	// set dt for all child blocks
	setDt(dt);

	bool params_updated = _parameter_update_sub.updated();
	bool status_updated = _vehicle_status_sub.updated();

	// set correct uORB ID, depending on if vehicle is VTOL or not
	if (status_updated) {
		if (_vehicle_status_sub.get().is_vtol) {

			// TODO: VTOL handle actuator controls MC
			//  need to publish both?
			_actuator_controls_0_pub.setMeta(ORB_ID(actuator_controls_virtual_fw));
			_vehicle_attitude_setpoint_pub.setMeta(ORB_ID(fw_virtual_attitude_setpoint));
			_vehicle_rates_setpoint_pub.setMeta(ORB_ID(fw_virtual_rates_setpoint));

		} else {
			_actuator_controls_0_pub.setMeta(ORB_ID(actuator_controls_0));
			_vehicle_attitude_setpoint_pub.setMeta(ORB_ID(vehicle_attitude_setpoint));
			_vehicle_rates_setpoint_pub.setMeta(ORB_ID(vehicle_rates_setpoint));
		}
	}

	// update all subscriptions
	updateSubscriptions();

	// update parameters
	if (params_updated) {
		parameters_update();
	}

	const vehicle_control_mode_s &vmode = _vehicle_control_mode_sub.get();

	if (vmode.flag_control_manual_enabled) {
		if (vmode.flag_control_attitude_enabled &&
		    vmode.flag_control_rates_enabled &&
		    !vmode.flag_control_climb_rate_enabled &&
		    !vmode.flag_control_altitude_enabled &&
		    !vmode.flag_control_velocity_enabled &&
		    !vmode.flag_control_position_enabled) {

			// attitude only
			publish_vehicle_attitude_setpoint();

		} else if (vmode.flag_control_rates_enabled &&
			   !vmode.flag_control_attitude_enabled) {

			// rates only
			publish_vehicle_rates_setpoint();

		} else if (!vmode.flag_control_rates_enabled &&
			   !vmode.flag_control_attitude_enabled) {

			// manual
			publish_actuator_controls();
		}
	}

	if (!_taskShouldExit) {
		// schedule next cycle
		work_queue(HPWORK, &_work, (worker_t)&StickMapper::cycle_trampoline, this,
			   USEC2TICK(1000000 / STICK_MAPPER_UPDATE_RATE_HZ));

	} else {
		_taskIsRunning = false;
	}
}

void
StickMapper::publish_actuator_controls()
{
	const manual_control_setpoint_s &manual = _manual_control_setpoint_sub.get();

	// publish actuator controls
	actuator_controls_s &pub = _actuator_controls_0_pub.get();
	pub.timestamp = _timestamp;
	pub.timestamp_sample = manual.timestamp;
	pub.control[actuator_controls_s::INDEX_ROLL] = manual.y * _man_roll_scale + _trim_roll.get();
	pub.control[actuator_controls_s::INDEX_PITCH] = -manual.x * _man_pitch_scale + _trim_pitch.get();
	pub.control[actuator_controls_s::INDEX_YAW] = manual.r + _man_yaw_scale + _trim_yaw.get();
	pub.control[actuator_controls_s::INDEX_THROTTLE] = manual.z;
	pub.control[actuator_controls_s::INDEX_FLAPS] = 0.0f; // flaps_applied
	pub.control[actuator_controls_s::INDEX_SPOILERS] = manual.aux1;
	pub.control[actuator_controls_s::INDEX_AIRBRAKES] = 0.0f; // flaperons_applied;
	pub.control[actuator_controls_s::INDEX_LANDING_GEAR] = manual.aux3;

	_actuator_controls_0_pub.update();
}

void
StickMapper::publish_vehicle_rates_setpoint()
{
	const manual_control_setpoint_s &manual = _manual_control_setpoint_sub.get();
	const float acro_max_throttle = 0.9f;

	vehicle_rates_setpoint_s &pub = _vehicle_rates_setpoint_pub.get();
	pub.timestamp = _timestamp;
	pub.roll = manual.y * radians(_acro_roll_max);
	pub.pitch = manual.x * radians(_acro_pitch_max);
	pub.yaw = manual.r * radians(_acro_yaw_max);
	pub.thrust = min(manual.z, acro_max_throttle);

	_vehicle_rates_setpoint_pub.update();
}

void
StickMapper::publish_vehicle_attitude_setpoint()
{
	const manual_control_setpoint_s &manual = _manual_control_setpoint_sub.get();

	// MC
	// _att_sp.roll_body = _manual.y * _params.man_roll_max;
	// _att_sp.pitch_body = -_manual.x * _params.man_pitch_max;

	float roll_sp = manual.y * radians(_man_roll_max + _roll_offset);
	roll_sp = constrain(roll_sp, -radians(_man_roll_max), radians(_man_roll_max));

	float pitch_sp = -manual.x * radians(_man_pitch_max + _pitch_offset);
	pitch_sp = constrain(pitch_sp, -radians(_man_pitch_max), radians(_man_pitch_max));

	float yaw_sp = 0.0f;

	const Eulerf att_sp(roll_sp, pitch_sp, yaw_sp);
	const Quatf att_q(att_sp);

	vehicle_attitude_setpoint_s &pub = _vehicle_attitude_setpoint_pub.get();
	pub.timestamp = _timestamp;
	pub.roll_body = att_sp.phi();
	pub.pitch_body = att_sp.theta();
	pub.yaw_body = att_sp.psi();
	pub.yaw_sp_move_rate = NAN;
	pub.thrust = manual.z;

	pub.q_d[0] = att_q(0);
	pub.q_d[1] = att_q(1);
	pub.q_d[2] = att_q(2);
	pub.q_d[3] = att_q(3);
	pub.q_d_valid = true;

	pub.landing_gear = 0.0f;

	pub.roll_reset_integral = false;
	pub.pitch_reset_integral = false;
	pub.yaw_reset_integral = false;

	pub.fw_control_yaw = false;
	pub.disable_mc_yaw_control = false;
	pub.apply_flaps = false;

	_vehicle_attitude_setpoint_pub.update();
}
