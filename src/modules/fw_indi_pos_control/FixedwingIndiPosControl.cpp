/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
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

#include "FixedwingIndiPosControl.hpp"

using namespace time_literals;
using namespace matrix;

using math::constrain;
using math::interpolate;
using math::radians;

FixedwingIndiPosControl::FixedwingIndiPosControl(bool vtol) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	/* fetch initial parameter values */
	parameters_update();
}

FixedwingIndiPosControl::~FixedwingIndiPosControl()
{
	perf_free(_loop_perf);
}

bool
FixedwingIndiPosControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

int
FixedwingIndiPosControl::parameters_update()
{
	_mass = _param_fw_mass.get();
	_K_x *= 0.f;
	_K_x(0, 0) = _param_kp_x.get();
	_K_x(1, 1) = _param_kp_y.get();
	_K_x(2, 2) = _param_kp_z.get();
	_K_v *= 0.f;
	_K_v(0, 0) = _param_kv_x.get();
	_K_v(1, 1) = _param_kv_y.get();
	_K_v(2, 2) = _param_kv_z.get();

	_K_q(0, 0) = _param_rot_k_roll.get();
	_K_q(1, 1) = _param_rot_k_pitch.get();
	_K_q(2, 2) = 0.5;
	return PX4_OK;
}

void
FixedwingIndiPosControl::vehicle_acceleration_poll()
{
	vehicle_acceleration_s acceleration;

	if (_vehicle_acceleration_sub.update(&acceleration)) {
		Dcmf R_ib(vehicle_attitude_);
		_acc = R_ib * Vector3f(acceleration.xyz) - Vector3f{0.f, 0.f, 9.81f};
	}

	if (hrt_absolute_time() - acceleration.timestamp > 20_ms
	    && _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
		PX4_ERR("linear acceleration sample is too old");
	}
}

void
FixedwingIndiPosControl::vehicle_attitude_poll()
{
	vehicle_attitude_s vehicle_attitude;

	if (_vehicle_attitude_sub.update(&vehicle_attitude)) {
		vehicle_attitude_ = Quatf(vehicle_attitude.q);
	}

}


void
FixedwingIndiPosControl::vehicle_local_position_poll()
{
	vehicle_local_position_s pos;

	if (_vehicle_local_position_sub.update(&pos)) {
		vehicle_position_ = Vector3f{pos.x, pos.y, pos.z};
		vehicle_velocity_ = Vector3f{pos.vx, pos.vy, pos.vz};
		// take accel from faster message, since 50Hz is too slow...
	}
}

void
FixedwingIndiPosControl::airspeed_poll()
{
	bool airspeed_valid = _airspeed_valid;
	airspeed_validated_s airspeed_validated;

	if (_airspeed_validated_sub.update(&airspeed_validated)) {

		if (PX4_ISFINITE(airspeed_validated.calibrated_airspeed_m_s)
		    && PX4_ISFINITE(airspeed_validated.true_airspeed_m_s)
		    && (airspeed_validated.calibrated_airspeed_m_s > 0.0f)) {

			airspeed_valid = true;

			_airspeed_last_valid = airspeed_validated.timestamp;
			_true_airspeed = airspeed_validated.true_airspeed_m_s;
			_cal_airspeed = airspeed_validated.calibrated_airspeed_m_s;
		}

	} else {
		// no airspeed updates for one second
		if (airspeed_valid && (hrt_elapsed_time(&_airspeed_last_valid) > 1_s)) {
			airspeed_valid = false;
		}
	}

	_airspeed_valid = airspeed_valid;
}

void
FixedwingIndiPosControl::vehicle_status_poll()
{
	vehicle_status_s vehicle_status;

	if (_vehicle_status_sub.update(&vehicle_status)) {
		_vehicle_status = vehicle_status;
	}
}

Quatf
FixedwingIndiPosControl::get_flat_attitude(Vector3f vel, Vector3f f, float rho_corrected)
{

	const Vector3f vel_air = vel - wind_estimate_;

	// compute force component projected onto lift axis
	const Vector3f vel_normalized = vel_air.normalized();

	Vector3f f_drag = (f * vel_normalized) * vel_normalized;
	Vector3f f_lift = f - f_drag;

	const Vector3f lift_normalized = f_lift.normalized();
	const Vector3f wing_normalized = -vel_normalized.cross(lift_normalized);
	// compute rotation matrix between ENU and FRD frame
	Dcmf R_bi;
	R_bi(0, 0) = vel_normalized(0);
	R_bi(0, 1) = vel_normalized(1);
	R_bi(0, 2) = vel_normalized(2);
	R_bi(1, 0) = wing_normalized(0);
	R_bi(1, 1) = wing_normalized(1);
	R_bi(1, 2) = wing_normalized(2);
	R_bi(2, 0) = lift_normalized(0);
	R_bi(2, 1) = lift_normalized(1);
	R_bi(2, 2) = lift_normalized(2);
	R_bi.renormalize();

	// compute required AoA
	Vector3f f_phi = R_bi * f_lift;
	float AoA = ((2.f * f_phi(2)) / (rho_corrected * _area * (vel_air * vel_air) + 0.001f) - _C_L0) / _C_L1 - _aoa_offset;
	// compute final rotation matrix
	Eulerf e(0.f, AoA, 0.f);
	Dcmf R_pitch(e);
	Dcmf Rotation(R_pitch * R_bi);

	Quatf q(Rotation);
	return q;
}

Quatf
FixedwingIndiPosControl::computeIndi(Vector3f pos_ref, Vector3f vel_ref, Vector3f acc_ref)
{
	Dcmf R_ib(vehicle_attitude_);
	Dcmf R_bi(R_ib.transpose());

	// apply LP filter to acceleration & velocity
	// Poll acceleration
	PX4_INFO("[PX4PosiNDI]   - _acc: %f, %f, %f", double(_acc(0)), double(_acc(1)), double(_acc(2)));
	acc_filtered_(0) = _lp_filter_accel[0].apply(_acc(0));
	acc_filtered_(1) = _lp_filter_accel[1].apply(_acc(1));
	acc_filtered_(2) = _lp_filter_accel[2].apply(_acc(2));

	// =========================================
	// apply PD control law on the body position
	// =========================================
	Vector3f acc_command_local = (_K_x * R_bi * (pos_ref - vehicle_position_) + _K_v * R_bi *
				      (vel_ref - vehicle_velocity_) + _K_a * R_bi *
				      (acc_ref - _acc));
	Vector3f acc_command = R_ib * acc_command_local + acc_ref; ///TODO: Why add acc_ref again?

	// ==================================
	// compute expected aerodynamic force
	// ==================================
	Vector3f airvel_body = R_bi * (vehicle_velocity_ - wind_estimate_);

	float angle_of_attack = atan2f(airvel_body(2), airvel_body(0)) + _aoa_offset;

	float C_l = _C_L0 + _C_L1 * angle_of_attack;
	float C_d = _C_D0 + _C_D1 * angle_of_attack + _C_D2 * powf(angle_of_attack, 2);
	// compute actual air density
	float rho_corrected;

	if (_cal_airspeed >= _stall_speed) {
		rho_corrected = _rho * powf(_cal_airspeed / _true_airspeed, 2);

	} else {
		rho_corrected = _rho;
	}

	float factor = -0.5f * rho_corrected * _area * sqrtf(airvel_body * airvel_body);

	// Wind axis
	Vector3f w_x = airvel_body;
	Vector3f w_z = w_x.cross(Vector3f{0.f, 1.f, 0.f});
	Vector3f f_current = R_ib * (factor * (C_l * w_z + C_d * w_x));
	// apply LP filter to force
	Vector3f f_current_filtered;
	f_current_filtered(0) = _lp_filter_force[0].apply(f_current(0));
	f_current_filtered(1) = _lp_filter_force[1].apply(f_current(1));
	f_current_filtered(2) = _lp_filter_force[2].apply(f_current(2));

	// ================================
	// get force command in world frame
	// ================================
	Vector3f f_command = _mass * (acc_command - acc_filtered_) + f_current_filtered;

	// ============================================================================================================
	// apply some filtering to the force command. This introduces some time delay,
	// which is not desired for stability reasons, but it rejects some of the noise fed to the low-level controller
	// ============================================================================================================
	f_command(0) = _lp_filter_ctrl0[0].apply(f_command(0));
	f_command(1) = _lp_filter_ctrl0[1].apply(f_command(1));
	f_command(2) = _lp_filter_ctrl0[2].apply(f_command(2));
	_f_command = f_command;
	// limit maximum lift force by the maximum lift force, the aircraft can produce (assume max force at 15° aoa)

	// ====================================================================
	// saturate force command to avoid overly agressive maneuvers and stall
	// ====================================================================
	if (_switch_saturation) {
		float speed = airvel_body * airvel_body;
		// compute maximum achievable force
		float f_max;

		if (speed > _stall_speed) {
			f_max = -factor * sqrtf(airvel_body * airvel_body) * (_C_L0 + _C_L1 * 0.25f); // assume stall at 15° AoA

		} else {
			f_max = -factor * _stall_speed * (_C_L0 + _C_L1 * 0.25f); // assume stall at 15° AoA
		}

		// compute current command
		float f_now = sqrtf(f_command * f_command);

		// saturate current command
		if (f_now > f_max) {
			f_command = f_max / f_now * f_command;
		}
	}

	PX4_INFO("[PX4PosiNDI]   - f_command: %f, %f, %f", double(f_command(0)), double(f_command(1)), double(f_command(2)));
	PX4_INFO("[PX4PosiNDI]   - vel_ref: %f, %f, %f", double(vel_ref(0)), double(vel_ref(1)), double(vel_ref(2)));

	// ==========================================================================
	// get required attitude (assuming we can fly the target velocity), and error
	// ==========================================================================
	Quatf ref_attitude = get_flat_attitude(vel_ref, f_command, rho_corrected);
	return ref_attitude;
}

Vector3f FixedwingIndiPosControl::controlAttitude(Quatf ref_attitude)
{
	PX4_INFO("[PX4PosiNDI]   - R_ref: %f, %f, %f, %f", double(ref_attitude(0)), double(ref_attitude(1)),
		 double(ref_attitude(2)), double(ref_attitude(3)));

	Dcmf R_ref(ref_attitude);
	Dcmf R_ib(vehicle_attitude_);
	Dcmf R_bi(R_ib.transpose());

	// get attitude error
	Dcmf R_ref_true(R_ref.transpose()*R_ib);
	// get required rotation vector (in body frame)
	AxisAnglef q_err(R_ref_true);
	Vector3f w_err;

	// project rotation angle to [-pi,pi]
	if (q_err.angle()*q_err.angle() < M_PI_F * M_PI_F) {
		w_err = -q_err.angle() * q_err.axis();

	} else {
		if (q_err.angle() > 0.f) {
			w_err = (2.f * M_PI_F - (float)fmod(q_err.angle(), 2.f * M_PI_F)) * q_err.axis();

		} else {
			w_err = (-2.f * M_PI_F - (float)fmod(q_err.angle(), 2.f * M_PI_F)) * q_err.axis();
		}
	}

	// TODO: Reference rate to rate controller
	// =========================================
	// apply PD control law on the body attitude
	// =========================================
	// Vector3f rot_acc_command = _K_q*w_err + _K_w*(omega_ref-_omega) + alpha_ref;
	Vector3f rot_vel_command = _K_q * w_err;
	PX4_INFO("[PX4PosiNDI]   - w_err: %f, %f, %f", double(w_err(0)), double(w_err(1)), double(w_err(2)));

	if (sqrtf(w_err * w_err) > M_PI_F) {
		PX4_ERR("rotation angle larger than pi: \t%.2f, \t%.2f, \t%.2f", (double)sqrtf(w_err * w_err), (double)q_err.angle(),
			(double)(q_err.axis()*q_err.axis()));
	}


	// ==============================================================
	// overwrite rudder rot_acc_command with turn coordination values
	// ==============================================================
	Vector3f vel_air = vehicle_velocity_ - wind_estimate_;
	Vector3f vel_normalized = vel_air.normalized();
	Vector3f acc_normalized = acc_filtered_.normalized();
	PX4_INFO("[PX4PosiNDI]   - acc_normalized: %f, %f, %f", double(acc_normalized(0)), double(acc_normalized(1)),
		 double(acc_normalized(2)));

	// compute ideal angular velocity
	Vector3f omega_turn_ref_normalized = vel_normalized.cross(acc_normalized);
	Vector3f omega_turn_ref;
	// constuct acc perpendicular to flight path
	Vector3f acc_perp = acc_filtered_ - (acc_filtered_ * vel_normalized) * vel_normalized;

	if (_airspeed_valid && _cal_airspeed > _stall_speed) {
		omega_turn_ref = sqrtf(acc_perp * acc_perp) / (_true_airspeed) * R_bi * omega_turn_ref_normalized.normalized();
		//PX4_INFO("yaw rate ref, yaw rate: \t%.2f\t%.2f", (double)(omega_turn_ref(2)), (double)(omega_filtered(2)));

	} else {
		omega_turn_ref = sqrtf(acc_perp * acc_perp) / (_stall_speed) * R_bi * omega_turn_ref_normalized.normalized();
		//PX4_ERR("No valid airspeed message detected or airspeed too low");
	}

	// apply some smoothing since we don't want HF components in our rudder output
	omega_turn_ref(2) = _lp_filter_rud.apply(omega_turn_ref(2));
	PX4_INFO("[PX4PosiNDI]   - omega_turn_ref: %f, %f, %f", double(omega_turn_ref(0)), double(omega_turn_ref(1)),
		 double(omega_turn_ref(2)));
	rot_vel_command(2) = omega_turn_ref(2);

	return rot_vel_command;
}

void FixedwingIndiPosControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// only run controller if angular velocity changed
	if (_vehicle_angular_velocity_sub.updated() || (hrt_elapsed_time(&_last_run) > 20_ms)) { //TODO rate!

		// only update parameters if they changed
		bool params_updated = _parameter_update_sub.updated();

		// check for parameter updates
		if (params_updated) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			updateParams();
			parameters_update();
		}

		vehicle_status_poll();
		// Poll local position
		vehicle_attitude_poll();

		vehicle_local_position_poll();
		vehicle_acceleration_poll();

		// Poll airspeed
		airspeed_poll();

		// Poll trajectory setpoints
		trajectory_setpoint_s trajectory_setpoint;

		if (_trajectory_setpoint_sub.update(&trajectory_setpoint)) {
			///TODO: Check if references are invalid
			pos_ref_ = Vector3f(trajectory_setpoint.position);
			vel_ref_ = Vector3f(trajectory_setpoint.velocity);
			acc_ref_ = Vector3f(trajectory_setpoint.acceleration);

			///TODO: Publish offboard mode

			///TODO: Publish bodyrate commands in offboard mode
			// if (_control_mode.flag_control_offboard_enabled) {
			// }

		}

		// static double getLineProgress(const Eigen::Vector3d position, const Eigen::Vector3d &segment_start,
		// 	const Eigen::Vector3d &segment_end) {
		// 	Eigen::Vector3d progress_vector = (segment_end - segment_start).normalized();
		// 	double segment_length = (segment_end - segment_start).norm();
		// 	Eigen::Vector3d error_vector = position - segment_start;
		// 	// Get Path Progress
		// 	double theta = error_vector.dot(progress_vector) / segment_length;
		// 	return theta;
		// }

		///TODO: Need to handle invalid velocity (zero)
		// ///TODO: Get closest point
		// Vector3f loiter_center{0.0, 0.0, -30.0};
		// float radius = 30.0;

		// Vector3f radial_error = (vehicle_position_ - loiter_center);
		// radial_error(0) = 0.0;
		// radial_error.normalize();
		// PX4_INFO("[PX4PosiNDI] radial_error: %f, %f, %f", double(radial_error(0)), double(radial_error(1)),
		// 	 double(radial_error(2)));


		// Vector3f closest_point = radius * radial_error;
		// pos_ref_(0) = closest_point(0) + loiter_center(0);
		// pos_ref_(1) = closest_point(1) + loiter_center(1);
		// pos_ref_(2) = closest_point(2) + loiter_center(2);

		// vel_ref_(0) = -radial_error(1);
		// vel_ref_(1) = radial_error(0);
		// vel_ref_(2) = 0.0;

		Vector3f progress_vector = vehicle_velocity_;
		progress_vector(2) = 0.0;
		progress_vector.normalize();

		Vector3f error_vector = vehicle_position_ - segment_start;
		float progress = error_vector.dot(progress_vector);

		Vector3f closest_point = progress * progress_vector + segment_start;
		pos_ref_ = closest_point;
		vel_ref_ = 15.0f * progress_vector;

		// Compute bodyrate commands
		Quatf ref_attitude = computeIndi(pos_ref_, vel_ref_, acc_ref_);
		// TODO: Place holder
		// ref_attitude = Quatf(1.0, 0.0, 0.0, 0.0);
		Vector3f rate_command = controlAttitude(ref_attitude);
		PX4_INFO("[PX4PosiNDI] angular_rate_command: %f, %f, %f", double(rate_command(0)), double(rate_command(1)),
			 double(rate_command(2)));

		if (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
			vehicle_attitude_setpoint_s attitude_sp;
			attitude_sp.timestamp = hrt_absolute_time();
			ref_attitude.copyTo(attitude_sp.q_d);
			_attitude_sp_pub.publish(attitude_sp);

			vehicle_rates_setpoint_s rates_sp;
			rates_sp.thrust_body[0] = 0.7; ///TODO: Add some throttle
			rates_sp.thrust_body[1] = 0.0;
			rates_sp.thrust_body[2] = 0.0;
			rates_sp.roll = rate_command(0);
			rates_sp.pitch = rate_command(1);
			rates_sp.yaw = rate_command(2);
			rates_sp.timestamp = hrt_absolute_time();
			_rate_sp_pub.publish(rates_sp);
		}

		// =================================
		// publish offboard control commands
		// =================================
		offboard_control_mode_s ocm{};
		ocm.body_rate = true;
		ocm.timestamp = hrt_absolute_time();
		_offboard_control_mode_pub.publish(ocm);



	} else {
		segment_start = vehicle_position_;
	}

	// backup schedule
	ScheduleDelayed(20_ms);

	perf_end(_loop_perf);
}

int FixedwingIndiPosControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	FixedwingIndiPosControl *instance = new FixedwingIndiPosControl(vtol);

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

int FixedwingIndiPosControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FixedwingIndiPosControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_rate_control is the fixed-wing rate controller.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fw_indi_pos_control_main(int argc, char *argv[])
{
	return FixedwingIndiPosControl::main(argc, argv);
}
