/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "ThroughControl.hpp"

#include <float.h>
#include <mathlib/mathlib.h>
#include <systemlib/mavlink_log.h>

using namespace matrix;
using through::ModeRegistration;

ThroughControl::ThroughControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	updateParams();
}

ThroughControl::~ThroughControl()
{
	perf_free(_loop_perf);
}

bool ThroughControl::init()
{
	ScheduleOnInterval(RUN_INTERVAL);
	return true;
}

void ThroughControl::updateParams()
{
	ModuleParams::updateParams();

	through::PnGuidance::Params params{};
	params.thr_min = _param_thru_thr_min.get();
	params.thr_max = _param_thru_thr_max.get();
	params.tilt_max = math::radians(_param_thru_tilt_max.get());
	params.k_vdamp = _param_thru_k_vdamp.get();
	params.pn_n = _param_thru_pn_n.get();
	params.a_max = _param_thru_a_max.get();
	params.v_max = _param_thru_v_max.get();
	params.k_vlim = _param_thru_k_vlim.get();
	params.vc_min = _param_thru_vc_min.get();
	_guidance.setParams(params);

	through::LosEstimator::Params los_params{};
	los_params.lpf_cutoff_hz = _param_thru_los_lpf.get();
	_los.setParams(los_params);

	_gimbal.setIds(_param_mav_sys_id.get(), _param_mav_comp_id.get());
}

void ThroughControl::Run()
{
	if (should_exit()) {
		_mode_registration.unregister();
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	const hrt_abstime now = hrt_absolute_time();
	const float dt = (_last_run_time > 0) ? math::constrain((now - _last_run_time) * 1e-6f, 0.002f, 0.05f) : 0.01f;
	_last_run_time = now;

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	updateSubscriptions(now);

	_mode_registration.update(now);

	if (_mode_registration.failed()) {
		PX4_ERR("could not register the THROUGH mode, exiting");
		perf_end(_loop_perf);
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	const bool active = _mode_registration.registered() && _armed
			    && (_nav_state == static_cast<uint8_t>(_mode_registration.modeId()));

	// While the mode runs we always report "can run": exits are handled here, not by commander failsafes
	_mode_registration.handleArmingCheckRequest(active ? ModeRegistration::ArmingFailure::None : checkConditionOne(now));

	if (!active) {
		if (_phase != through_status_s::PHASE_IDLE) {
			if (_phase != through_status_s::PHASE_EXIT) {
				_exit_reason = through_status_s::REASON_USER_MODE_SWITCH;
				mavlink_log_info(&_mavlink_log_pub, "THROUGH stopped: %s", reasonString(_exit_reason));
			}

			enterPhase(through_status_s::PHASE_IDLE, now);
			_gimbal.release();
		}

	} else {
		if (_phase == through_status_s::PHASE_IDLE) {
			_mode_start_time = now;
			_exit_reason = through_status_s::REASON_NONE;

			// latch the hover thrust for the whole run (VERIFICATION.md D4)
			const bool hte_valid = _hover_thrust_estimate.valid && (now < _hover_thrust_estimate.timestamp + 1_s);
			_hover_thrust = math::constrain(hte_valid ? _hover_thrust_estimate.hover_thrust : _param_mpc_thr_hover.get(), 0.1f,
							0.9f);

			_yaw_sp = Eulerf(Quatf(_attitude.q)).psi();
			_yaw_target = _yaw_sp;
			_aligned_since = 0;
			_saturated_since = 0;
			_opening_since = 0;
			_ramp_alpha = 0.f;

			// ALIGN entry: start the LOS filter (warm-up before GUIDE) and take the gimbal
			_los.reset();
			_vision_target_new = false;
			_gimbal.acquire(now);
			enterPhase(through_status_s::PHASE_ALIGN, now);
		}

		const float time_in_mode = (now - _mode_start_time) * 1e-6f;

		if (_phase != through_status_s::PHASE_EXIT && time_in_mode > _param_thru_t_max.get()) {
			enterExit(through_status_s::REASON_MODE_TIMEOUT, now);
		}

		if (_phase != through_status_s::PHASE_EXIT) {
			updateLos();
		}

		_guidance_active = false;
		const Phase phase = _phase;

		switch (phase) {
		case through_status_s::PHASE_ALIGN:
			runAlign(now, dt);
			break;

		case through_status_s::PHASE_RAMP:
			runRamp(now, dt);
			break;

		case through_status_s::PHASE_GUIDE:
			runGuide(now, dt);
			break;

		case through_status_s::PHASE_TERMINAL:
			runTerminal(now);
			break;

		case through_status_s::PHASE_EXIT:
			runExit(now);
			break;
		}

		// a phase that exited in this cycle continues as EXIT right away (setpoint stream without gaps)
		if (phase != _phase) {
			if (_phase == through_status_s::PHASE_EXIT) {
				runExit(now);

			} else if (_phase == through_status_s::PHASE_TERMINAL) {
				runTerminal(now);
			}
		}
	}

	publishStatus(now);

	perf_end(_loop_perf);
}

void ThroughControl::updateSubscriptions(hrt_abstime now)
{
	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			_nav_state = vehicle_status.nav_state;
			_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
		}
	}

	if (_vehicle_land_detected_sub.updated()) {
		vehicle_land_detected_s vehicle_land_detected;

		if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
			_landed = vehicle_land_detected.landed;
		}
	}

	_vehicle_local_position_sub.update(&_local_pos);
	_hover_thrust_estimate_sub.update(&_hover_thrust_estimate);

	gimbal_device_attitude_status_s gimbal_status;

	if (_gimbal_device_attitude_status_sub.update(&gimbal_status)) {
		// camera attitude in NED at the gimbal IMU sample time (GZGimbal, VERIFICATION.md D1)
		_q_gimbal = Quatf(gimbal_status.q);
		_gimbal_status_time = now;
		_los.addGimbalAttitude(gimbal_status.timestamp, _q_gimbal);
	}

	if (_vehicle_attitude_sub.update(&_attitude)) {
		// fallback camera attitude: vehicle attitude * commanded gimbal pitch (spec 7 step 1)
		const float pitch_down = PX4_ISFINITE(_gimbal.lastCommand()) ? _gimbal.lastCommand() : 0.f;
		_los.addFallbackAttitude(_attitude.timestamp_sample, Quatf(_attitude.q) * Quatf(Eulerf(0.f, -pitch_down, 0.f)));
	}

	if (_vision_target_sub.update(&_vision_target)) {
		// reception time: robust against timestamps of the ROS side
		_vision_target_last_time = now;
		_vision_target_new = true;

		if (_vision_target.target_valid) {
			_vision_target_last_valid_time = now;
			_last_valid_target_size = _vision_target.target_size;
		}
	}

	checkExitCommandAck();
}

bool ThroughControl::targetFresh(hrt_abstime now) const
{
	return (_vision_target_last_time > 0)
	       && (now < _vision_target_last_time + static_cast<hrt_abstime>(_param_thru_lost_to.get() * 1e6f));
}

float ThroughControl::altitude() const
{
	return _local_pos.z_valid ? -_local_pos.z : NAN;
}

ModeRegistration::ArmingFailure ThroughControl::checkConditionOne(hrt_abstime now) const
{
	if (_landed) {
		return ModeRegistration::ArmingFailure::NotInAir;
	}

	if (!(altitude() >= _param_thru_min_alt.get())) {
		return ModeRegistration::ArmingFailure::AltitudeLow;
	}

	if (!targetFresh(now)) {
		return ModeRegistration::ArmingFailure::TargetStale;
	}

	if (!_vision_target.uncage) {
		return ModeRegistration::ArmingFailure::NotLocked;
	}

	if (!_vision_target.target_valid) {
		return ModeRegistration::ArmingFailure::TargetInvalid;
	}

	return ModeRegistration::ArmingFailure::None;
}

void ThroughControl::enterPhase(Phase phase, hrt_abstime now)
{
	if (phase != _phase) {
		mavlink_log_info(&_mavlink_log_pub, "THROUGH: %s -> %s", phaseString(_phase), phaseString(phase));
	}

	_phase = phase;
	_phase_start_time = now;
}

void ThroughControl::enterExit(Reason reason, hrt_abstime now)
{
	_exit_reason = reason;

	if (reason == through_status_s::REASON_COMPLETED) {
		mavlink_log_info(&_mavlink_log_pub, "THROUGH completed");

	} else {
		mavlink_log_critical(&_mavlink_log_pub, "THROUGH aborted: %s", reasonString(reason));
	}

	enterPhase(through_status_s::PHASE_EXIT, now);
	_gimbal.release();

	_exit_command_acked = false;
	sendExitModeCommand(now);
}

bool ThroughControl::checkCommonExits(hrt_abstime now)
{
	const bool target_lost = now > _vision_target_last_valid_time
				 + static_cast<hrt_abstime>(_param_thru_lost_to.get() * 1e6f);

	if (targetFresh(now) && !_vision_target.uncage) {
		// spec 8.2 lists USER_UNLOCK for RAMP/GUIDE only; also applied in ALIGN (VERIFICATION.md D5)
		enterExit(through_status_s::REASON_USER_UNLOCK, now);
		return true;
	}

	if (target_lost) {
		// GUIDE: losing the target right in front of the ring is expected (it leaves the image)
		if (_phase == through_status_s::PHASE_GUIDE
		    && _last_valid_target_size > _param_thru_term_sz.get() * _param_thru_term_near.get()) {
			enterPhase(through_status_s::PHASE_TERMINAL, now);

		} else {
			enterExit(through_status_s::REASON_TARGET_LOST, now);
		}

		return true;
	}

	if (!(altitude() >= _param_thru_min_alt.get())) {
		// also applied in ALIGN (VERIFICATION.md D5)
		enterExit(through_status_s::REASON_MIN_ALT, now);
		return true;
	}

	return false;
}

void ThroughControl::updateLos()
{
	if (!_vision_target_new) {
		return;
	}

	_vision_target_new = false;

	through::LosEstimator::Measurement measurement{};
	measurement.timestamp_sample = _vision_target.timestamp_sample;
	measurement.pan = _vision_target.pan_error;
	measurement.tilt = _vision_target.tilt_error;
	measurement.valid = _vision_target.target_valid;
	_los.update(measurement);
}

float ThroughControl::measuredGimbalPitch(hrt_abstime now) const
{
	if (_gimbal_status_time == 0 || now > _gimbal_status_time + 100_ms) {
		return NAN;
	}

	// camera optical axis (camera x) expressed against the body x-z plane
	return through::GimbalTracker::bodyPitchDown(Quatf(_attitude.q), _q_gimbal.rotateVector(Vector3f{1.f, 0.f, 0.f}));
}

void ThroughControl::trackTarget(hrt_abstime now, float dt)
{
	if (!_los.initialized()) {
		return; // no LOS yet: keep heading and gimbal
	}

	// yaw: LOS azimuth, rate limited (spec 7 step 6)
	_yaw_target = through::PnGuidance::losYaw(_los.u(), _yaw_target);
	_yaw_sp = through::PnGuidance::rateLimitYaw(_yaw_sp, _yaw_target, math::radians(_param_thru_yaw_rmax.get()), dt);

	// gimbal: camera axis along the LOS, in body frame because the simulated gimbal is not stabilized (step 7)
	_gimbal.acquire(now);
	const Quatf q_nb(_attitude.q);
	const float pitch_des = through::GimbalTracker::bodyPitchDown(q_nb, _los.u());
	const float pitch_cmd = through::GimbalTracker::pitchCommand(measuredGimbalPitch(now), pitch_des,
				_param_thru_gmb_k.get());
	_gimbal.commandPitch(pitch_cmd);
}

void ThroughControl::runAlign(hrt_abstime now, float dt)
{
	const float time_in_phase = (now - _phase_start_time) * 1e-6f;

	if (checkCommonExits(now)) {
		return;
	}

	if (time_in_phase > _param_thru_aln_t_max.get()) {
		enterExit(through_status_s::REASON_ALIGN_TIMEOUT, now);
		return;
	}

	trackTarget(now, dt);
	publishHoverSetpoint(now, dt);

	// ALIGN -> RAMP: target centered and heading on the LOS azimuth for THRU_ALN_T_OK (spec 8.2)
	const float aln_err = math::radians(_param_thru_aln_err.get());
	const float yaw = Eulerf(Quatf(_attitude.q)).psi();
	const bool aligned = _los.valid() && targetFresh(now) && _vision_target.target_valid
			     && fabsf(_vision_target.pan_error) < aln_err
			     && fabsf(_vision_target.tilt_error) < aln_err
			     && fabsf(wrap_pi(_yaw_target - yaw)) < aln_err;

	if (!aligned) {
		_aligned_since = 0;

	} else if (_aligned_since == 0) {
		_aligned_since = now;

	} else if (now > _aligned_since + static_cast<hrt_abstime>(_param_thru_aln_t_ok.get() * 1e6f)) {
		enterPhase(through_status_s::PHASE_RAMP, now);
	}
}

void ThroughControl::runRamp(hrt_abstime now, float dt)
{
	if (checkCommonExits(now)) {
		return;
	}

	const float ramp_t = _param_thru_ramp_t.get();
	_ramp_alpha = (ramp_t > FLT_EPSILON) ? math::constrain((now - _phase_start_time) * 1e-6f / ramp_t, 0.f, 1.f) : 1.f;

	trackTarget(now, dt);
	publishGuidanceSetpoint(now, _ramp_alpha);

	if (_ramp_alpha >= 1.f) {
		_saturated_since = 0;
		_opening_since = 0;
		enterPhase(through_status_s::PHASE_GUIDE, now);
	}
}

void ThroughControl::runGuide(hrt_abstime now, float dt)
{
	if (checkCommonExits(now)) {
		return;
	}

	// close to the ring: freeze and coast (spec 8.2)
	if (targetFresh(now) && _vision_target.target_valid && _vision_target.target_size > _param_thru_term_sz.get()) {
		enterPhase(through_status_s::PHASE_TERMINAL, now);
		return;
	}

	trackTarget(now, dt);
	publishGuidanceSetpoint(now, 1.f);

	// tilt or thrust saturated for too long: the PN demand cannot be flown
	if (!(_last_cmd.tilt_limited || _last_cmd.thrust_saturated)) {
		_saturated_since = 0;

	} else if (_saturated_since == 0) {
		_saturated_since = now;

	} else if (now > _saturated_since + static_cast<hrt_abstime>(_param_thru_sat_t.get() * 1e6f)) {
		enterExit(through_status_s::REASON_TILT_SAT, now);
		return;
	}

	// moving away from the target
	if (_last_guidance.vc >= 0.f) {
		_opening_since = 0;

	} else if (_opening_since == 0) {
		_opening_since = now;

	} else if (now > _opening_since + OPENING_TIMEOUT) {
		enterExit(through_status_s::REASON_OPENING, now);
	}
}

void ThroughControl::runTerminal(hrt_abstime now)
{
	// coast with the attitude and thrust frozen at TERMINAL entry; uncage and target validity are ignored
	publishAttitudeSetpoint(_last_cmd, now);

	if (now >= _phase_start_time + static_cast<hrt_abstime>(_param_thru_term_t.get() * 1e6f)) {
		enterExit(through_status_s::REASON_COMPLETED, now);
	}
}

void ThroughControl::runExit(hrt_abstime now)
{
	// keep flying a damped hover until the commander has switched the mode
	publishHoverSetpoint(now, 0.f);

	if (!_exit_command_acked && now > _exit_command_time + EXIT_RETRY_INTERVAL) {
		sendExitModeCommand(now);
	}
}

Vector3f ThroughControl::velocity() const
{
	Vector3f v_ned{};

	if (_local_pos.v_xy_valid) {
		v_ned(0) = _local_pos.vx;
		v_ned(1) = _local_pos.vy;
	}

	if (_local_pos.v_z_valid) {
		v_ned(2) = _local_pos.vz;
	}

	return v_ned;
}

void ThroughControl::publishHoverSetpoint(hrt_abstime now, float dt)
{
	(void)dt;

	const Vector3f f = through::PnGuidance::specificForce(_guidance.hoverAcceleration(velocity()));
	_last_cmd = _guidance.forceToAttitude(f, _yaw_sp, _hover_thrust);
	publishAttitudeSetpoint(_last_cmd, now);
}

void ThroughControl::publishGuidanceSetpoint(hrt_abstime now, float alpha)
{
	const Vector3f v_ned = velocity();

	// RAMP: the base thrust goes linearly from the hover thrust to THRU_THR_BASE (spec 8.1)
	const float thr_base = _hover_thrust + alpha * (_param_thru_thr_base.get() - _hover_thrust);
	_last_guidance = _guidance.guide(_los.u(), _los.omega(), v_ned, thr_base, _hover_thrust);
	_guidance_active = true;

	const Vector3f a = (1.f - alpha) * _guidance.hoverAcceleration(v_ned) + alpha * _last_guidance.a;
	_last_cmd = _guidance.forceToAttitude(through::PnGuidance::specificForce(a), _yaw_sp, _hover_thrust);
	publishAttitudeSetpoint(_last_cmd, now);
}

void ThroughControl::publishAttitudeSetpoint(const through::AttitudeCommand &cmd, hrt_abstime now)
{
	vehicle_attitude_setpoint_s att_sp{};
	cmd.q_d.copyTo(att_sp.q_d);
	att_sp.yaw_sp_move_rate = 0.f;
	att_sp.thrust_body[0] = 0.f;
	att_sp.thrust_body[1] = 0.f;
	att_sp.thrust_body[2] = -cmd.thrust;
	att_sp.timestamp = now;
	_vehicle_attitude_setpoint_pub.publish(att_sp);
}

void ThroughControl::sendExitModeCommand(hrt_abstime now)
{
	const uint8_t nav_state = (_param_thru_exit_mode.get() == 1)
				  ? vehicle_status_s::NAVIGATION_STATE_POSCTL
				  : vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;

	vehicle_command_s command{};
	command.command = vehicle_command_s::VEHICLE_CMD_SET_NAV_STATE;
	command.param1 = nav_state;
	command.source_system = _param_mav_sys_id.get();
	command.source_component = _param_mav_comp_id.get();
	command.target_system = _param_mav_sys_id.get();
	command.target_component = _param_mav_comp_id.get();
	command.from_external = false;
	command.timestamp = hrt_absolute_time();
	_vehicle_command_pub.publish(command);

	_exit_command_time = now;
}

void ThroughControl::checkExitCommandAck()
{
	vehicle_command_ack_s ack;
	int tries = vehicle_command_ack_s::ORB_QUEUE_LENGTH;

	while (_vehicle_command_ack_sub.update(&ack) && --tries >= 0) {
		if (ack.command != vehicle_command_s::VEHICLE_CMD_SET_NAV_STATE || _phase != through_status_s::PHASE_EXIT) {
			continue;
		}

		if (ack.result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED) {
			_exit_command_acked = true;

		} else {
			PX4_WARN("exit mode switch rejected (result %d), retrying", ack.result);
		}
	}
}

void ThroughControl::publishStatus(hrt_abstime now)
{
	// full rate while the mode runs, low rate when idle
	if (_phase == through_status_s::PHASE_IDLE && now < _last_status_publish_time + 500_ms) {
		return;
	}

	through_status_s status{};
	status.phase = _phase;
	status.exit_reason = _exit_reason;
	status.time_in_phase = (_phase != through_status_s::PHASE_IDLE) ? (now - _phase_start_time) * 1e-6f : 0.f;
	status.time_in_mode = (_phase != through_status_s::PHASE_IDLE) ? (now - _mode_start_time) * 1e-6f : 0.f;

	const bool los = _los.initialized() && (_phase != through_status_s::PHASE_IDLE);
	const Vector3f omega = _los.omega();

	// PN quantities only while RAMP/GUIDE compute them
	for (int i = 0; i < 3; ++i) {
		status.los_ned[i] = los ? _los.u()(i) : NAN;
		status.los_rate[i] = los ? omega(i) : NAN;
		status.a_perp[i] = _guidance_active ? _last_guidance.a_perp(i) : NAN;
		status.f_cmd[i] = _last_cmd.f_cmd(i);
	}

	status.los_elevation = los ? _los.elevation() : NAN;
	status.los_azimuth = los ? _los.azimuth() : NAN;
	status.vc = los ? _los.closingVelocity(velocity()) : NAN;
	status.s_along = _guidance_active ? _last_guidance.s : NAN;
	status.thrust_cmd = _last_cmd.thrust;
	status.hover_thrust = _hover_thrust;
	status.thrust_saturated = _last_cmd.thrust_saturated;
	status.tilt_limited = _last_cmd.tilt_limited;
	status.a_perp_limited = _guidance_active && _last_guidance.a_perp_limited;

	switch (_phase) {
	case through_status_s::PHASE_RAMP: status.ramp_alpha = _ramp_alpha; break;

	case through_status_s::PHASE_GUIDE:
	case through_status_s::PHASE_TERMINAL: status.ramp_alpha = 1.f; break;

	default: status.ramp_alpha = 0.f; break;
	}

	status.pan_error = _vision_target.pan_error;
	status.tilt_error = _vision_target.tilt_error;
	status.target_size = _vision_target.target_size;
	status.target_valid = _vision_target.target_valid && targetFresh(now);
	status.uncage = _vision_target.uncage;

	// + = camera down, relative to the body x axis
	status.gimbal_pitch_cmd = _gimbal.lastCommand();
	status.gimbal_pitch_meas = measuredGimbalPitch(now);

	status.timestamp = now;
	_through_status_pub.publish(status);
	_last_status_publish_time = now;
}

const char *ThroughControl::phaseString(Phase phase)
{
	switch (phase) {
	case through_status_s::PHASE_IDLE: return "IDLE";

	case through_status_s::PHASE_ALIGN: return "ALIGN";

	case through_status_s::PHASE_RAMP: return "RAMP";

	case through_status_s::PHASE_GUIDE: return "GUIDE";

	case through_status_s::PHASE_TERMINAL: return "TERMINAL";

	case through_status_s::PHASE_EXIT: return "EXIT";
	}

	return "UNKNOWN";
}

const char *ThroughControl::reasonString(Reason reason)
{
	switch (reason) {
	case through_status_s::REASON_NONE: return "NONE";

	case through_status_s::REASON_COMPLETED: return "COMPLETED";

	case through_status_s::REASON_USER_UNLOCK: return "USER_UNLOCK";

	case through_status_s::REASON_USER_MODE_SWITCH: return "USER_MODE_SWITCH";

	case through_status_s::REASON_ALIGN_TIMEOUT: return "ALIGN_TIMEOUT";

	case through_status_s::REASON_TARGET_LOST: return "TARGET_LOST";

	case through_status_s::REASON_MIN_ALT: return "MIN_ALT";

	case through_status_s::REASON_TILT_SAT: return "TILT_SAT";

	case through_status_s::REASON_MODE_TIMEOUT: return "MODE_TIMEOUT";

	case through_status_s::REASON_OPENING: return "OPENING";
	}

	return "UNKNOWN";
}

int ThroughControl::task_spawn(int argc, char *argv[])
{
	ThroughControl *instance = new ThroughControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int ThroughControl::print_status()
{
	static constexpr const char *registration_states[] {"unregistered", "waiting for reply", "registered", "failed"};

	PX4_INFO("registration: %s, mode_id (nav_state): %d, arming_check_id: %d",
		 registration_states[static_cast<int>(_mode_registration.state())],
		 _mode_registration.modeId(), _mode_registration.armingCheckId());
	PX4_INFO("phase: %s, last exit reason: %s", phaseString(_phase), reasonString(_exit_reason));
	PX4_INFO("condition one: %s",
		 ModeRegistration::armingFailureString(checkConditionOne(hrt_absolute_time())));
	PX4_INFO("hover thrust (latched): %.3f", (double)_hover_thrust);

	static constexpr const char *attitude_sources[] {"none", "gimbal feedback", "fallback"};
	PX4_INFO("LOS: %s, source %s, elevation %.2f deg, azimuth %.2f deg",
		 _los.valid() ? "valid" : "invalid", attitude_sources[static_cast<int>(_los.attitudeSource())],
		 (double)math::degrees(_los.elevation()), (double)math::degrees(_los.azimuth()));
	PX4_INFO("gimbal: control %s, pitch cmd %.2f deg, meas %.2f deg (+ = down)", _gimbal.inControl() ? "yes" : "no",
		 (double)math::degrees(_gimbal.lastCommand()), (double)math::degrees(measuredGimbalPitch(hrt_absolute_time())));
	perf_print_counter(_loop_perf);
	return 0;
}

int ThroughControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ThroughControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
THROUGH external flight mode: dive through a colored ring tracked by the gimbal camera,
using proportional navigation. The mode registers with the commander as an external
mode and can only be entered in air while the vision target is locked (uncage) and valid.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("through_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int through_control_main(int argc, char *argv[])
{
	return ThroughControl::main(argc, argv);
}
