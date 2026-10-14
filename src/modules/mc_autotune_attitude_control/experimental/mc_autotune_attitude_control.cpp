/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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
 * @file mc_autotune_attitude_control.cpp
 *
 * @author Mathieu Bresciani <mathieu@auterion.com>
 */

#include "mc_autotune_attitude_control.hpp"


using namespace matrix;

ModuleBase::Descriptor McAutotuneAttitudeControl::desc{task_spawn, custom_command, print_usage};

McAutotuneAttitudeControl::McAutotuneAttitudeControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	_autotune_attitude_control_status_pub.advertise();
	_validation = new ControllerValidation;
}

McAutotuneAttitudeControl::~McAutotuneAttitudeControl()
{
	delete _validation;
	perf_free(_cycle_perf);
}

bool McAutotuneAttitudeControl::init()
{
	if (!_validation) { return false; }

	if (!_vehicle_torque_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void McAutotuneAttitudeControl::Run()
{
	if (should_exit()) {
		_experiment_active = false;
		publishExcitation(hrt_absolute_time());
		ScheduleClear();
		_vehicle_torque_setpoint_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	const state previous_state = _state;

	if (_state != state::idle) { ScheduleDelayed(100_ms); }

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		if (_experiment_active) {
			PX4_WARN("parameters changed during measurement");
			_experiment_active = false;
			_state = state::fail;
			_state_start_time = hrt_absolute_time();
			publishExcitation(_state_start_time);
		}

		// update parameters from storage
		updateParams();
		updateStateMachine(hrt_absolute_time());
	}

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
			_nav_state = vehicle_status.nav_state;
		}
	}

	if (_actuator_controls_status_sub.updated()) {
		actuator_controls_status_s controls_status;

		if (_actuator_controls_status_sub.copy(&controls_status)) {
			_control_power = Vector3f(controls_status.control_power);
		}
	}

	if (_vehicle_command_sub.updated()) {
		vehicle_command_s vehicle_command;

		if (_vehicle_command_sub.copy(&vehicle_command)) {
			if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_AUTOTUNE_ENABLE) {
				vehicle_status_s vehicle_status{};
				_vehicle_status_sub.copy(&vehicle_status);

				// Both autotune modules run on VTOL; only the active vehicle type owns the command.
				if (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
				    && !vehicle_status.in_transition_mode
				    && fabsf(vehicle_command.param1 - 1.0f) < FLT_EPSILON && fabsf(vehicle_command.param2) < FLT_EPSILON) {
					_vehicle_cmd_start_autotune = true;
				}
			}
		}
	}

	const hrt_abstime watchdog_now = hrt_absolute_time();

	if (_experiment_active && (!_armed || watchdog_now - _response_time > 500_ms)) {
		PX4_WARN("disarmed or response stream lost");
		_state = state::fail;
		_state_start_time = watchdog_now;
		_experiment_active = false;
	}

	updateStateMachine(watchdog_now);
	publishExcitation(watchdog_now);

	// Report terminal/idle transitions even when control samples have stopped.
	// MAVLink uses IDLE to allow the next autotune command.
	if (_state != previous_state) {
		autotune_attitude_control_status_s status{};
		status.timestamp = watchdog_now;
		status.state = static_cast<int>(_state);
		_autotune_attitude_control_status_pub.publish(status);
	}

	// new control data needed every iteration
	if ((_state == state::idle && !_vehicle_cmd_start_autotune)
	    || !_vehicle_torque_setpoint_sub.updated()) {
		return;
	}

	vehicle_torque_setpoint_s vehicle_torque_setpoint;
	vehicle_angular_velocity_s angular_velocity;

	if (!_vehicle_torque_setpoint_sub.copy(&vehicle_torque_setpoint)
	    || !_vehicle_angular_velocity_sub.copy(&angular_velocity)) {
		return;
	}

	perf_begin(_cycle_perf);

	const hrt_abstime timestamp_sample = vehicle_torque_setpoint.timestamp;

	// collect sample interval average for filters
	if (_last_run > 0) {
		// Guard against too small (< 0.125ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((timestamp_sample - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_interval_sum += dt;
		_interval_count++;

	} else {
		_interval_sum = 0.f;
		_interval_count = 0.f;
	}

	_last_run = timestamp_sample;

	if (!_experiment_active) { checkFilters(); }

	const bool identifying = _state == state::roll || _state == state::pitch || _state == state::yaw;
	autotune_response_s response{};

	if (_experiment_active && _autotune_response_sub.update(&response)) {
		_response_time = hrt_absolute_time();

		if (identifying && _response_time >= _settle_until) {
			control_allocator_status_s allocation{};
			vehicle_attitude_s attitude{};
			const bool fresh = _control_allocator_status_sub.copy(&allocation)
					   && hrt_elapsed_time(&allocation.timestamp) < 250_ms
					   && _vehicle_attitude_sub.copy(&attitude) && hrt_elapsed_time(&attitude.timestamp) < 100_ms;
			const Eulerf angles{Quatf{attitude.q}};

			if (!fresh || !PX4_ISFINITE(angles.phi()) || !PX4_ISFINITE(angles.theta())) {
				PX4_WARN("stale measurement status");
				_state = state::fail;
				_state_start_time = _response_time;
				_experiment_active = false;

			} else if (fabsf(angles.phi()) > math::radians(5.f) || fabsf(angles.theta()) > math::radians(5.f)
				   || !allocation.torque_setpoint_achieved || !allocation.thrust_setpoint_achieved) {
				_excitation_amplitude *= .5f;

				if (_excitation_amplitude < .00005f) {
					PX4_WARN("no small-signal operating point");
					_state = state::fail;
					_state_start_time = _response_time;
					_experiment_active = false;

				} else {
					PX4_DEBUG("Autotune axis %d: reducing excitation to %.6f", _excited_axis, (double)_excitation_amplitude);
					startAxis(_excited_axis, _response_time);
				}

			} else {
				const bool new_sample = _validation->update(response.timestamp, response.dt, Vector3f(response.torque),
							Vector3f(response.angular_velocity), Vector3f(response.angular_acceleration),
							response.excitation[_excited_axis]);

				if (!_validation->validData()) {
					PX4_WARN("response error %d: sample %.6f s, controller dt %.6f s",
						 static_cast<int>(_validation->error()), (double)_validation->sampleInterval(), (double)response.dt);
					_state = state::fail;
					_state_start_time = _response_time;
					_experiment_active = false;

				} else if (new_sample && !_candidate_ready) {
					if (_validation->trained(response.timestamp)) {
						// The first verification sample must not train the candidate.
						copyGains(_excited_axis);
						_candidate_ready = true;

					} else {
						_sys_id.updateFilters(_input_scale * response.torque[_excited_axis], response.angular_velocity[_excited_axis]);

						if (++_model_update_counter >= _model_update_scaler) {
							_sys_id.update();
							_model_update_counter = 0;
						}
					}
				}
			}
		}
	}

	if (hrt_elapsed_time(&_last_publish) > _publishing_dt_hrt || _last_publish == 0) {
		const hrt_abstime now = hrt_absolute_time();
		updateStateMachine(now);

		Vector<float, 5> coeff = _sys_id.getCoefficients();
		coeff(2) *= _input_scale;
		coeff(3) *= _input_scale;
		coeff(4) *= _input_scale;

		const float model_dt = static_cast<float>(_model_update_scaler) * _filter_dt;
		// Candidate telemetry; application is gated by the independent response check.
		computeGains(coeff);

		const Vector<float, 5> &coeff_var = _sys_id.getVariances();

		const Vector3f rate_sp{};

		autotune_attitude_control_status_s status{};
		status.timestamp = now;
		coeff.copyTo(status.coeff);
		coeff_var.copyTo(status.coeff_var);
		status.fitness = _sys_id.getFitness();
		status.dt_model = model_dt;
		status.innov = _sys_id.getInnovation();
		status.u_filt = _sys_id.getFilteredInputData();
		status.y_filt = _sys_id.getFilteredOutputData();
		status.kc = _kid(0);
		status.ki = _kid(1);
		status.kd = _kid(2);
		status.att_p = _attitude_p;
		rate_sp.copyTo(status.rate_sp);
		status.state = static_cast<int>(_state);
		_autotune_attitude_control_status_pub.publish(status);

		_last_publish = now;
	}

	perf_end(_cycle_perf);
}

void McAutotuneAttitudeControl::checkFilters()
{
	if (_interval_count > 1000) {
		// calculate sensor update rate
		_sample_interval_avg = _interval_sum / _interval_count;

		// check if sample rate error is greater than 1%
		bool reset_filters = false;

		if ((fabsf(_filter_dt - _sample_interval_avg) / _filter_dt) > 0.01f) {
			reset_filters = true;
		}

		if (reset_filters || !_are_filters_initialized) {
			_filter_dt = _sample_interval_avg;

			const float filter_rate_hz = 1.f / _filter_dt;

			_sys_id.setLpfCutoffFrequency(filter_rate_hz, _param_imu_gyro_cutoff.get());
			_sys_id.setHpfCutoffFrequency(filter_rate_hz, 1.f / (5.f * _param_mc_at_period.get()));

			// Set the model sampling time depending on the gyro cutoff frequency
			// as this is a good indicator of the maximum control loop bandwidth
			float model_dt = math::constrain(math::max(1.f / (2.f * _param_imu_gyro_cutoff.get()), _filter_dt), _model_dt_min,
							 _model_dt_max);

			_model_update_scaler = math::max(int(model_dt / _filter_dt), 1);
			model_dt = _model_update_scaler * _filter_dt;

			_sys_id.setForgettingFactor(60.f, model_dt);
			_sys_id.setFitnessLpfTimeConstant(1.f, model_dt);

			_are_filters_initialized = true;
		}

		// reset sample interval accumulator
		_last_run = 0;
	}
}

void McAutotuneAttitudeControl::updateStateMachine(hrt_abstime now)
{
	// Abort only the active identification/test sequence. Landing and pilot inputs
	// must not turn a completed tune into a failure while its result is being reported.
	if (_state != state::idle && _state != state::wait_for_disarm
	    && _state != state::complete && _state != state::fail) {
		manual_control_setpoint_s manual_control_setpoint{};
		_manual_control_setpoint_sub.copy(&manual_control_setpoint);

		const bool timeout = !PX4_ISFINITE(_param_mc_at_timeout.get()) || _param_mc_at_timeout.get() < 20.f
				     || _param_mc_at_timeout.get() > 14400.f || (now - (_experiment_active ? _tune_start : _state_start_time)) >
				     static_cast<hrt_abstime>(_param_mc_at_timeout.get() * 1e6f);
		const bool mode_changed = (_start_flight_mode != _nav_state);
		const bool pilot_intervention = (fabsf(manual_control_setpoint.roll) > 0.05f)
						|| (fabsf(manual_control_setpoint.pitch) > 0.05f)
						|| (fabsf(manual_control_setpoint.yaw) > 0.05f);

		if (timeout || mode_changed || pilot_intervention) {
			PX4_WARN("aborted in state %u: %s", static_cast<unsigned>(_state),
				 timeout ? "timeout" : (mode_changed ? "flight mode changed" : "pilot intervention"));

			if (_state == state::test) {
				revertParamGains();
			}

			_state = state::fail;
			_state_start_time = now;
			_experiment_active = false;
			publishExcitation(now);
			return;
		}
	}

	switch (_state) {
	case state::idle:
		if (_vehicle_cmd_start_autotune) {
			_state = state::init;

			_state_start_time = now;
			_start_flight_mode = _nav_state;
		}

		break;

	case state::init:
		if (_are_filters_initialized) {
			if (!startExperiment(now)) {
				PX4_WARN("unsupported configuration or out of memory");
				_state = state::fail;
				_state_start_time = now;
			}
		}

		break;

	case state::roll:
	case state::pitch:
	case state::yaw:
		if (isAxisConverged()) {
			_state = _excited_axis == 0 ? state::roll_pause : (_excited_axis == 1 ? state::pitch_pause : state::yaw_pause);
			_state_start_time = now;
		}

		break;

	case state::roll_pause:
	case state::pitch_pause:
		if ((now - _state_start_time) > 2_s) { startAxis(_excited_axis + 1, now); }

		break;

	case state::yaw_pause:
		if ((now - _state_start_time) > 2_s) { _state = state::verification; _state_start_time = now; }

		break;

	case state::verification:
		if (!_experiment_active || !areGainsGood()) {
			PX4_WARN("invalid gains or missing measurements");
			_state = state::fail;
			_state_start_time = now;
			_experiment_active = false;

		} else if (validateGains()) {
			_state = state::apply;
			_state_start_time = now;
			_experiment_active = false;

		} else {
			// validateGains may restart measurement with a longer period.
			if (_state == state::verification) {
				_state = state::fail;
				_state_start_time = now;
				_experiment_active = false;
			}
		}

		break;

	case state::apply:
		if ((_param_mc_at_apply.get() == 1)) {
			_state = state::wait_for_disarm;

		} else if (_param_mc_at_apply.get() == 2) {
			backupAndSaveGainsToParams();
			_state = state::test;

		} else {
			_state = state::complete;
		}

		_state_start_time = now;

		break;

	case state::wait_for_disarm:
		if (!_armed) {
			saveGainsToParams();
			_state = state::complete;
			_state_start_time = now;
		}

		break;

	case state::test:
		if ((now - _state_start_time) > 4_s) {
			_state = state::complete;
			_state_start_time = now;

		} else if ((now - _state_start_time) < 4_s
			   && (now - _state_start_time) > 1_s
			   && _control_power.longerThan(0.1f)) {
			_state = state::fail;
			revertParamGains();
			_state_start_time = now;
		}

		break;

	case state::complete:

	// fallthrough
	case state::fail:

		// Wait a bit in that state to make sure
		// the other components are aware of the final result
		if ((now - _state_start_time) > 2_s) {
			_state = state::idle;
			stopAutotune();
		}

		break;
	}
}

void McAutotuneAttitudeControl::backupAndSaveGainsToParams()
{
	float backup_gains[15] = {};
	backup_gains[0] = _param_mc_rollrate_k.get();
	backup_gains[1] = _param_mc_rollrate_p.get();
	backup_gains[2] = _param_mc_rollrate_i.get();
	backup_gains[3] = _param_mc_rollrate_d.get();
	backup_gains[4] = _param_mc_roll_p.get();
	backup_gains[5] = _param_mc_pitchrate_k.get();
	backup_gains[6] = _param_mc_pitchrate_p.get();
	backup_gains[7] = _param_mc_pitchrate_i.get();
	backup_gains[8] = _param_mc_pitchrate_d.get();
	backup_gains[9] = _param_mc_pitch_p.get();
	backup_gains[10] = _param_mc_yawrate_k.get();
	backup_gains[11] = _param_mc_yawrate_p.get();
	backup_gains[12] = _param_mc_yawrate_i.get();
	backup_gains[13] = _param_mc_yawrate_d.get();
	backup_gains[14] = _param_mc_yaw_p.get();

	saveGainsToParams();

	_rate_k(0) = backup_gains[0] * backup_gains[1]; // convert and save as standard form
	_rate_i(0) = backup_gains[2] / backup_gains[1];
	_rate_d(0) = backup_gains[3] / backup_gains[1];
	_att_p(0) = backup_gains[4];
	_rate_k(1) = backup_gains[5] * backup_gains[6];
	_rate_i(1) = backup_gains[7] / backup_gains[6];
	_rate_d(1) = backup_gains[8] / backup_gains[6];
	_att_p(1) = backup_gains[9];
	_rate_k(2) = backup_gains[10] * backup_gains[11];
	_rate_i(2) = backup_gains[12] / backup_gains[11];
	_rate_d(2) = backup_gains[13] / backup_gains[11];
	_att_p(2) = backup_gains[14];

	_gains_backup_available = true;
}

void McAutotuneAttitudeControl::revertParamGains()
{
	if (_gains_backup_available) {
		saveGainsToParams();
	}
}

void McAutotuneAttitudeControl::computeGains(const Vector<float, 5> &coeff)
{
	const Vector3f num(coeff(2), coeff(3), coeff(4));
	const Vector3f den(1.f, coeff(0), coeff(1));

	const float model_dt = static_cast<float>(_model_update_scaler) * _filter_dt;

	const float desired_rise_time = ((_state == state::yaw)
					 || (_state == state::yaw_pause)) ? math::max(.2f, _param_mc_at_rise_time.get()) : _param_mc_at_rise_time.get();
	_kid = pid_design::computePidGmvc(num, den, model_dt, desired_rise_time, 0.f, 0.7f);

	// Prevent the D term from going just negative if it is not needed
	if ((_kid(2) < 0.f) && (_kid(2) > -0.001f)) {
		_kid(2) = 0.f;
	}

	// To compute the attitude gain, use the following empirical rule:
	// "An error of 60 degrees should produce the maximum control output"
	// or K_att * K_rate * rad(60) = 1
	_attitude_p = math::constrain(1.f / (math::radians(60.f) * _kid(0)), 2.f, 6.5f);
}

bool McAutotuneAttitudeControl::isAxisConverged() const
{
	return _validation && _candidate_ready && _validation->finished() && _validation->validData();
}

void McAutotuneAttitudeControl::copyGains(int index)
{
	if (index <= 2) {
		// Freeze a candidate before the independent measurement periods.
		Vector<float, 5> coeff = _sys_id.getCoefficients();
		coeff(2) *= _input_scale;
		coeff(3) *= _input_scale;
		coeff(4) *= _input_scale;
		computeGains(coeff);
#if defined(CONFIG_COMMON_SIMULATION)
		PX4_INFO("candidate axis %d A %.9g %.9g dt %.9g", index, (double)coeff(0), (double)coeff(1),
			 (double)(_model_update_scaler * _filter_dt));
		PX4_INFO("candidate axis %d B %.9g %.9g %.9g", index, (double)coeff(2), (double)coeff(3), (double)coeff(4));
#endif
		_rate_k(index) = _kid(0);
		_rate_i(index) = _kid(1);
		_rate_d(index) = _kid(2);
		_att_p(index) = _attitude_p;
	}
}

bool McAutotuneAttitudeControl::areGainsGood() const
{
	if (!_rate_k.isAllFinite() || !_rate_i.isAllFinite() || !_rate_d.isAllFinite() || !_att_p.isAllFinite()) {
		return false;
	}

	const bool are_positive = _rate_k.min() > 0.f
				  && _rate_i.min() > 0.f
				  && _rate_d.min() >= 0.f
				  && _att_p.min() > 0.f;

	const bool are_small_enough = _rate_k.max() < 0.5f
				      && _rate_i.max() < 10.f
				      && _rate_d.max() < 0.1f
				      && _att_p.max() < 12.f;

	return are_positive && are_small_enough;
}

void McAutotuneAttitudeControl::saveGainsToParams()
{
	// save as parallel form
	_param_mc_rollrate_p.set(_rate_k(0));
	_param_mc_rollrate_k.set(1.f);
	_param_mc_rollrate_i.set(_rate_k(0) * _rate_i(0));
	_param_mc_rollrate_d.set(_rate_k(0) * _rate_d(0));
	_param_mc_roll_p.set(_att_p(0));
	_param_mc_rollrate_p.commit_no_notification();
	_param_mc_rollrate_k.commit_no_notification();
	_param_mc_rollrate_i.commit_no_notification();
	_param_mc_rollrate_d.commit_no_notification();
	_param_mc_roll_p.commit_no_notification();

	_param_mc_pitchrate_p.set(_rate_k(1));
	_param_mc_pitchrate_k.set(1.f);
	_param_mc_pitchrate_i.set(_rate_k(1) * _rate_i(1));
	_param_mc_pitchrate_d.set(_rate_k(1) * _rate_d(1));
	_param_mc_pitch_p.set(_att_p(1));
	_param_mc_pitchrate_p.commit_no_notification();
	_param_mc_pitchrate_k.commit_no_notification();
	_param_mc_pitchrate_i.commit_no_notification();
	_param_mc_pitchrate_d.commit_no_notification();
	_param_mc_pitch_p.commit_no_notification();

	_param_mc_yawrate_p.set(_rate_k(2));
	_param_mc_yawrate_k.set(1.f);
	_param_mc_yawrate_i.set(_rate_k(2) * _rate_i(2));
	_param_mc_yawrate_d.set(_rate_k(2) * _rate_d(2));
	_param_mc_yaw_p.set(_att_p(2));
	_param_mc_yawrate_p.commit_no_notification();
	_param_mc_yawrate_k.commit_no_notification();
	_param_mc_yawrate_i.commit_no_notification();
	_param_mc_yawrate_d.commit_no_notification();
	_param_mc_yaw_p.commit();
}

void McAutotuneAttitudeControl::stopAutotune()
{
	_experiment_active = false;
	publishExcitation(hrt_absolute_time());
	ScheduleClear();
	_vehicle_cmd_start_autotune = false;
}

ControllerValidation::Gains McAutotuneAttitudeControl::currentGains() const
{
	ControllerValidation::Gains result;
	const Vector3f scale(_param_mc_rollrate_k.get(), _param_mc_pitchrate_k.get(), _param_mc_yawrate_k.get());
	result.p = scale.emult(Vector3f(_param_mc_rollrate_p.get(), _param_mc_pitchrate_p.get(), _param_mc_yawrate_p.get()));
	result.i = scale.emult(Vector3f(_param_mc_rollrate_i.get(), _param_mc_pitchrate_i.get(), _param_mc_yawrate_i.get()));
	result.d = scale.emult(Vector3f(_param_mc_rollrate_d.get(), _param_mc_pitchrate_d.get(), _param_mc_yawrate_d.get()));
	result.attitude = Vector3f(_param_mc_roll_p.get(), _param_mc_pitch_p.get(), _param_mc_yaw_p.get());
	return result;
}

bool McAutotuneAttitudeControl::startExperiment(hrt_abstime now)
{
	if (!_validation || !_armed || _nav_state != vehicle_status_s::NAVIGATION_STATE_POSCTL
	    || _param_mc_bat_scale_en.get() || fabsf(_param_mc_rollrate_ff.get()) > FLT_EPSILON
	    || fabsf(_param_mc_pitchrate_ff.get()) > FLT_EPSILON || fabsf(_param_mc_yawrate_ff.get()) > FLT_EPSILON) { return false; }

	if (!PX4_ISFINITE(_param_mc_at_period.get()) || !PX4_ISFINITE(_param_mc_at_timeout.get())
	    || !PX4_ISFINITE(_param_mc_at_sysid_amp.get()) || _param_mc_at_sysid_amp.get() <= 0.f
	    || !PX4_ISFINITE(_filter_dt) || _filter_dt <= 0.f) { return false; }

	_baseline = currentGains();

	if (!_baseline.p.isAllFinite() || !_baseline.i.isAllFinite() || !_baseline.d.isAllFinite()
	    || !_baseline.attitude.isAllFinite() || _baseline.p.min() <= 0.f || _baseline.i.min() <= 0.f
	    || _baseline.d.min() < 0.f || _baseline.attitude.min() <= 0.f) { return false; }

	_measurement_period = math::constrain(_param_mc_at_period.get(), 4.f, 128.f);
	const float maximum_frequency = math::min(.2f / _filter_dt, math::max(10.f, 2.f * _param_imu_gyro_cutoff.get()));
	_validation->configure(_measurement_period, maximum_frequency);
	_tune_start = now;
	_response_time = now;
	_experiment_active = true;
	_gains_backup_available = false;
	_rate_k.zero(); _rate_i.zero(); _rate_d.zero(); _att_p.zero();
	_excitation_amplitude = math::min(.003f * _param_mc_at_sysid_amp.get() / .7f, .08f / _validation->frequencies());
	PX4_DEBUG("Autotune response verification: period %.1f s, %d frequencies", (double)_measurement_period, _validation->frequencies());
	startAxis(0, now);
	return true;
}

void McAutotuneAttitudeControl::startAxis(int axis, hrt_abstime now)
{
	_excited_axis = axis;
	_state = axis == 0 ? state::roll : (axis == 1 ? state::pitch : state::yaw);
	_state_start_time = now;
	_settle_until = now + 2_s;
	_validation->beginAxis(axis, _settle_until, _excitation_amplitude);
	_candidate_ready = false;
	_rate_k(axis) = _rate_i(axis) = _rate_d(axis) = _att_p(axis) = 0.f;
	_sys_id.setHpfCutoffFrequency(1.f / _filter_dt, 1.f / (5.f * _measurement_period));
	_sys_id.setForgettingFactor(math::max(60.f, 4.f * _measurement_period), _model_update_scaler * _filter_dt);
	_sys_id.reset();
	_model_update_counter = 0;
	_input_scale = 1.f / _baseline.p(axis);
}

void McAutotuneAttitudeControl::publishExcitation(hrt_abstime now)
{
	if (!_experiment_active && !_excitation_active) { return; }

	_excitation_active = _experiment_active;
	autotune_excitation_s excitation{};
	excitation.timestamp = _experiment_active ? now : 0;
	excitation.nav_state = _start_flight_mode;

	if (_experiment_active && now >= _settle_until
	    && (_state == state::roll || _state == state::pitch || _state == state::yaw)) {
		excitation.torque[_excited_axis] = _validation->excitation(now);
	}

	_autotune_excitation_pub.publish(excitation);
}

bool McAutotuneAttitudeControl::validateGains()
{
	ControllerValidation::Gains requested;
	requested.p = _rate_k;
	requested.i = _rate_k.emult(_rate_i);
	requested.d = _rate_k.emult(_rate_d);
	requested.attitude = _att_p;

	for (int option = 0; option < 4; ++option) {
		if (option == 0 && fabsf(_param_mc_ref_ff.get()) > FLT_EPSILON) { continue; }

		const float fraction = option <= 1 ? 1.f : (option == 2 ? .5f : .25f);
		ControllerValidation::Gains candidate;
		candidate.p = _baseline.p + fraction * (requested.p - _baseline.p);
		candidate.i = _baseline.i + fraction * (requested.i - _baseline.i);
		candidate.d = _baseline.d + fraction * (requested.d - _baseline.d);
		candidate.attitude = option == 0 ? requested.attitude : _baseline.attitude;
		float minimum = 0.f;
		const auto result = _validation->check(_baseline, candidate, _param_mc_yaw_tq_cutoff.get(), minimum);

		if (result == ControllerValidation::Result::InsufficientBandwidth) {
			if (_measurement_period < 128.f) {
				_measurement_period = math::min(2.f * _measurement_period, 128.f);
				_validation->configure(_measurement_period, math::min(.2f / _filter_dt, math::max(10.f, 2.f * _param_imu_gyro_cutoff.get())));
				_excitation_amplitude = math::min(_excitation_amplitude, .08f / _validation->frequencies());
				PX4_DEBUG("Autotune: extending response period to %.1f s", (double)_measurement_period);
				_rate_k.zero(); _rate_i.zero(); _rate_d.zero(); _att_p.zero();
				startAxis(0, hrt_absolute_time());

			} else { PX4_WARN("insufficient frequency coverage"); }

			return false;
		}

		if (!PX4_ISFINITE(minimum)) {
			PX4_WARN("insufficient measurement coverage or quality");
			return false;
		}

		PX4_DEBUG("Autotune candidate %d: response bound %.3f", option, (double)minimum);

		if (result == ControllerValidation::Result::Pass) {
			// A negligible update is not a successful identification.
			if ((candidate.p - _baseline.p).norm() + (candidate.i - _baseline.i).norm()
			    + (candidate.d - _baseline.d).norm() < .01f * (_baseline.p.norm() + _baseline.i.norm() + _baseline.d.norm())) {
				PX4_WARN("no significant gain change");
				return false;
			}

			_rate_k = candidate.p;
			_rate_i = candidate.i.edivide(candidate.p);
			_rate_d = candidate.d.edivide(candidate.p);
			_att_p = candidate.attitude;
			PX4_DEBUG("Autotune validated: rate fraction %.2f, attitude %s", (double)fraction, option == 0 ? "updated" : "retained");
			return true;
		}
	}

	PX4_WARN("controller verification failed");
	return false;
}

int McAutotuneAttitudeControl::task_spawn(int argc, char *argv[])
{
	McAutotuneAttitudeControl *instance = new McAutotuneAttitudeControl();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int McAutotuneAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int McAutotuneAttitudeControl::print_status()
{
	perf_print_counter(_cycle_perf);

	return 0;
}

int McAutotuneAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_autotune_attitude_control", "autotune");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_autotune_attitude_control_main(int argc, char *argv[])
{
	return ModuleBase::main(McAutotuneAttitudeControl::desc, argc, argv);
}
