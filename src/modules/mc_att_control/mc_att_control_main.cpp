/****************************************************************************
 *
 *   Copyright (c) 2013-2025 PX4 Development Team. All rights reserved.
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
 * @file mc_att_control_main.cpp
 * Multicopter attitude controller.
 *
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 * @author Matthias Grob	<maetugr@gmail.com>
 * @author Beat Küng		<beat-kueng@gmx.net>
 *
 */

#include "mc_att_control.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

#include "AttitudeControl/AttitudeControlMath.hpp"

using namespace matrix;

MulticopterAttitudeControl::MulticopterAttitudeControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_vehicle_attitude_setpoint_pub(vtol ? ORB_ID(mc_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_vtol(vtol)
{
	parameters_updated();
	// Rate of change 5% per second -> 1.6 seconds to ramp to default 8% MPC_MANTHR_MIN
	_manual_throttle_minimum.setSlewRate(0.05f);
	// Rate of change 50% per second -> 2 seconds to ramp to 100%
	_manual_throttle_maximum.setSlewRate(0.5f);
	// Rate of change 5% per second -> 6 seconds to ramp 30% if hover thrust parameter is off
	_hover_thrust_slew_rate.setSlewRate(0.05f);
}

MulticopterAttitudeControl::~MulticopterAttitudeControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterAttitudeControl::init()
{
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void
MulticopterAttitudeControl::parameters_updated()
{
	// Store some of the parameters in a more convenient way & precompute often-used values
	_attitude_control.setProportionalGain(Vector3f(_param_mc_roll_p.get(), _param_mc_pitch_p.get(), _param_mc_yaw_p.get()),
					      _param_mc_yaw_weight.get());

	// angular rate limits
	using math::radians;
	_attitude_control.setRateLimit(Vector3f(radians(_param_mc_rollrate_max.get()), radians(_param_mc_pitchrate_max.get()),
						radians(_param_mc_yawrate_max.get())));

	// Update from hover thrust parameter if there's no valid estimate in use
	if (!PX4_ISFINITE(_hover_thrust_estimate)) {
		_hover_thrust_slew_rate.setForcedValue(_param_mpc_thr_hover.get());
	}

	_man_tilt_max = math::radians(_param_mpc_man_tilt_max.get());
}

float
MulticopterAttitudeControl::throttle_curve(float throttle_stick_input)
{
	float thrust = 0.f;

	// throttle_stick_input is in range [-1, 1]
	switch (_param_mpc_thr_curve.get()) {
	case 1: // no rescaling
		thrust = math::interpolate(throttle_stick_input, -1.f, 1.f,
					   _manual_throttle_minimum.getState(), _param_mpc_thr_max.get());
		break;

	case 2: // rescale to hover thrust param at 0 stick input
		thrust = math::interpolateNXY(throttle_stick_input,
		{-1.f, 0.f, 1.f},
		{_manual_throttle_minimum.getState(), _param_mpc_thr_hover.get(), _param_mpc_thr_max.get()});
		break;

	default: // 0 or other: rescale to HTE value
		thrust = math::interpolateNXY(throttle_stick_input,
		{-1.f, 0.f, 1.f},
		{_manual_throttle_minimum.getState(), _hover_thrust_slew_rate.getState(), _param_mpc_thr_max.get()});
		break;
	}

	return math::min(thrust, _manual_throttle_maximum.getState());
}

void
MulticopterAttitudeControl::generate_attitude_setpoint(const Quatf &q, float dt)
{
	vehicle_attitude_setpoint_s attitude_setpoint{};

	// Avoid accumulating absolute yaw error with arming stick gesture
	const bool arming_gesture = (_manual_control_setpoint.throttle < -.9f) && (_param_mc_airmode.get() != 2);

	if (arming_gesture || !_heading_good_for_control) {
		_yaw_setpoint_stabilized = NAN;
	}

	const float yaw = Eulerf(q).psi();
	const float yaw_stick_input = math::expo_deadzone(_manual_control_setpoint.yaw, _param_mpc_yaw_expo.get(),
				      _param_mpc_hold_dz.get());
	_stick_yaw.generateYawSetpoint(attitude_setpoint.yaw_sp_move_rate, _yaw_setpoint_stabilized, yaw_stick_input, yaw, dt,
				       _unaided_heading);

	/*
	 * Input mapping for roll & pitch setpoints
	 * ----------------------------------------
	 * We control the following 2 angles:
	 * - tilt angle, given by sqrt(roll*roll + pitch*pitch)
	 * - the direction of the maximum tilt in the XY-plane, which also defines the direction of the motion
	 *
	 * This allows a simple limitation of the tilt angle, the vehicle flies towards the direction that the stick
	 * points to, and changes of the stick input are linear.
	 */
	_man_roll_input_filter.setParameters(dt, _param_mc_man_tilt_tau.get());
	_man_pitch_input_filter.setParameters(dt, _param_mc_man_tilt_tau.get());

	// we want to fly towards the direction of (roll, pitch)
	Vector2f v = Vector2f(_man_roll_input_filter.update(_manual_control_setpoint.roll * _man_tilt_max),
			      -_man_pitch_input_filter.update(_manual_control_setpoint.pitch * _man_tilt_max));
	float v_norm = v.norm(); // the norm of v defines the tilt angle

	if (v_norm > _man_tilt_max) { // limit to the configured maximum tilt angle
		v *= _man_tilt_max / v_norm;
	}

	Quatf q_sp_rp = AxisAnglef(v(0), v(1), 0.f);
	// Make sure there's a valid attitude quaternion with no yaw error when yaw is unlocked (NAN)
	const float yaw_setpoint = PX4_ISFINITE(_yaw_setpoint_stabilized) ? _yaw_setpoint_stabilized : yaw;
	// The axis angle can change the yaw as well (noticeable at higher tilt angles).
	// This is the formula by how much the yaw changes:
	//   let a := tilt angle, b := atan(y/x) (direction of maximum tilt)
	//   yaw = atan(-2 * sin(b) * cos(b) * sin^2(a/2) / (1 - 2 * cos^2(b) * sin^2(a/2))).
	const Quatf q_sp_yaw(cosf(yaw_setpoint / 2.f), 0.f, 0.f, sinf(yaw_setpoint / 2.f));

	if (_vtol) {
		// Modify the setpoints for roll and pitch such that they reflect the user's intention even
		// if a large yaw error(yaw_sp - yaw) is present. In the presence of a yaw error constructing
		// an attitude setpoint from the yaw setpoint will lead to unexpected attitude behaviour from
		// the user's view as the tilt will not be aligned with the heading of the vehicle.

		AttitudeControlMath::correctTiltSetpointForYawError(q_sp_rp, q, q_sp_yaw);
	}

	// Align the desired tilt with the yaw setpoint
	Quatf q_sp = q_sp_yaw * q_sp_rp;

	q_sp.copyTo(attitude_setpoint.q_d);

	attitude_setpoint.thrust_body[2] = -throttle_curve(_manual_control_setpoint.throttle);

	attitude_setpoint.timestamp = hrt_absolute_time();
	_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);
}

void
MulticopterAttitudeControl::Run()
{
	if (should_exit()) {
		_vehicle_attitude_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_updated();
	}

	// Update hover thrust for stick scaling
	if (_hover_thrust_estimate_sub.updated()) {
		hover_thrust_estimate_s hover_thrust_estimate;

		if (_hover_thrust_estimate_sub.update(&hover_thrust_estimate)) {
			if (hover_thrust_estimate.valid) {
				_hover_thrust_estimate = math::constrain(hover_thrust_estimate.hover_thrust, .05f, .9f);

			} else {
				// Possibly bad estimate before it got invalid, slew back to parameter
				_hover_thrust_estimate = _param_mpc_thr_hover.get();
			}
		}
	}

	// run controller on attitude updates
	vehicle_attitude_s v_att;

	if (_vehicle_attitude_sub.update(&v_att)) {

		// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
		const float dt = math::constrain(((v_att.timestamp_sample - _last_run) * 1e-6f), 0.0002f, 0.02f);
		_last_run = v_att.timestamp_sample;

		const Quatf q{v_att.q};

		/* check for updates in other topics */
		_manual_control_setpoint_sub.update(&_manual_control_setpoint);
		_vehicle_control_mode_sub.update(&_vehicle_control_mode);

		if (_vehicle_status_sub.updated()) {
			vehicle_status_s vehicle_status;

			if (_vehicle_status_sub.copy(&vehicle_status)) {
				_vehicle_type_rotary_wing = (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING);
				_vtol = vehicle_status.is_vtol;
				_vtol_in_transition_mode = vehicle_status.in_transition_mode;
				_vtol_tailsitter = vehicle_status.is_vtol_tailsitter;

				const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
				_spooled_up = armed && hrt_elapsed_time(&vehicle_status.armed_time) > _param_com_spoolup_time.get() * 1_s;
			}
		}

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
			}
		}

		if (_vehicle_local_position_sub.updated()) {
			vehicle_local_position_s vehicle_local_position;

			if (_vehicle_local_position_sub.copy(&vehicle_local_position)) {
				_heading_good_for_control = vehicle_local_position.heading_good_for_control;
				_unaided_heading = vehicle_local_position.unaided_heading;
			}
		}

		// during transitions VTOL module generates attitude setpoints
		const bool is_hovering = (_vehicle_type_rotary_wing && !_vtol_in_transition_mode);
		const bool is_tailsitter_transition = (_vtol_tailsitter && _vtol_in_transition_mode);

		const bool run_att_ctrl = _vehicle_control_mode.flag_control_attitude_enabled
					  && (is_hovering || is_tailsitter_transition);

		if (run_att_ctrl) {
			// Generate the attitude setpoint from stick inputs if we are in Manual/Stabilized mode
			if (_vehicle_control_mode.flag_control_manual_enabled &&
			    !_vehicle_control_mode.flag_control_altitude_enabled &&
			    !_vehicle_control_mode.flag_control_velocity_enabled &&
			    !_vehicle_control_mode.flag_control_position_enabled) {

				generate_attitude_setpoint(q, dt);

			} else {
				_man_roll_input_filter.reset(0.f);
				_man_pitch_input_filter.reset(0.f);
				_yaw_setpoint_stabilized = NAN;
				_stick_yaw.reset(Eulerf(q).psi(), _unaided_heading);
			}

			// Check for new attitude setpoint
			if (_vehicle_attitude_setpoint_sub.updated()) {
				vehicle_attitude_setpoint_s vehicle_attitude_setpoint;

				if (_vehicle_attitude_setpoint_sub.copy(&vehicle_attitude_setpoint)
				    && (vehicle_attitude_setpoint.timestamp > _last_attitude_setpoint)) {

					_attitude_control.setAttitudeSetpoint(Quatf(vehicle_attitude_setpoint.q_d), vehicle_attitude_setpoint.yaw_sp_move_rate);
					_thrust_setpoint_body = Vector3f(vehicle_attitude_setpoint.thrust_body);
					_last_attitude_setpoint = vehicle_attitude_setpoint.timestamp;
				}
			}

			// Check for a heading reset
			if (_quat_reset_counter != v_att.quat_reset_counter) {
				const Quatf delta_q_reset(v_att.delta_q_reset);
				const float delta_psi = Eulerf(delta_q_reset).psi();

				// Only offset the yaw setpoint when the heading is locked
				if (PX4_ISFINITE(_yaw_setpoint_stabilized)) {
					_yaw_setpoint_stabilized = wrap_pi(_yaw_setpoint_stabilized + delta_psi);
				}

				_stick_yaw.ekfResetHandler(delta_psi);

				if (v_att.timestamp > _last_attitude_setpoint) {
					// adapt existing attitude setpoint unless it was generated after the current attitude estimate
					_attitude_control.adaptAttitudeSetpoint(delta_q_reset);
				}

				_quat_reset_counter = v_att.quat_reset_counter;
			}

			Vector3f rates_sp = _attitude_control.update(q);

			const hrt_abstime now = hrt_absolute_time();
			autotune_attitude_control_status_s pid_autotune;

			if (_autotune_attitude_control_status_sub.copy(&pid_autotune)) {
				if ((pid_autotune.state == autotune_attitude_control_status_s::STATE_ROLL
				     || pid_autotune.state == autotune_attitude_control_status_s::STATE_PITCH
				     || pid_autotune.state == autotune_attitude_control_status_s::STATE_YAW
				     || pid_autotune.state == autotune_attitude_control_status_s::STATE_TEST)
				    && ((now - pid_autotune.timestamp) < 1_s)) {
					rates_sp += Vector3f(pid_autotune.rate_sp);
				}
			}

			// publish rate setpoint
			vehicle_rates_setpoint_s rates_setpoint{};
			rates_setpoint.roll = rates_sp(0);
			rates_setpoint.pitch = rates_sp(1);
			rates_setpoint.yaw = rates_sp(2);
			_thrust_setpoint_body.copyTo(rates_setpoint.thrust_body);
			rates_setpoint.timestamp = hrt_absolute_time();

			_vehicle_rates_setpoint_pub.publish(rates_setpoint);

		} else {
			_man_roll_input_filter.reset(0.f);
			_man_pitch_input_filter.reset(0.f);
			_yaw_setpoint_stabilized = NAN;
			_stick_yaw.reset(Eulerf(q).psi(), _unaided_heading);
		}

		if (_landed) {
			_manual_throttle_minimum.update(0.f, dt);

		} else {
			_manual_throttle_minimum.update(_param_mpc_manthr_min.get(), dt);
		}

		if (_spooled_up) {
			_manual_throttle_maximum.update(1.f, dt);

		} else {
			_manual_throttle_maximum.setForcedValue(0.f);
		}

		if (PX4_ISFINITE(_hover_thrust_estimate)) {
			_hover_thrust_slew_rate.update(_hover_thrust_estimate, dt);
		}
	}

	perf_end(_loop_perf);
}

int MulticopterAttitudeControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterAttitudeControl *instance = new MulticopterAttitudeControl(vtol);

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

int MulticopterAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter attitude controller. It takes attitude
setpoints (`vehicle_attitude_setpoint`) as inputs and outputs a rate setpoint.

The controller has a P loop for angular error

Publication documenting the implemented Quaternion Attitude Control:
Nonlinear Quadrocopter Attitude Control (2013)
by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
Institute for Dynamic Systems and Control (IDSC), ETH Zurich

https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


/**
 * Multicopter attitude control app start / stop handling function
 */
extern "C" __EXPORT int mc_att_control_main(int argc, char *argv[])
{
	return MulticopterAttitudeControl::main(argc, argv);
}
