/****************************************************************************
 * CdusAllocator.cpp
 ****************************************************************************/

#include "CdusAllocatorDuct.hpp"
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>

CdusAllocatorDuct::CdusAllocatorDuct() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	init_effectiveness_matrix();

	if (!_torque_sp_sub.registerCallback()) {
		PX4_WARN("Callback registration failed");
	}
	#ifndef ENABLE_LOCKSTEP_SCHEDULER
		ScheduleDelayed(50_ms);   // backup periodic run
	#endif

	ScheduleNow();
}

void CdusAllocatorDuct::init_effectiveness_matrix()
{
	//Experimentally derived effectiveness matrix
	_B(0,0) = -0.175623f; _B(0,1) =  0.293600f; _B(0,2) =  0.769231f; _B(0,3) = -1.0f;
	_B(1,0) =  0.175623f; _B(1,1) = -0.293600f; _B(1,2) =  0.769231f; _B(1,3) = -1.0f;
	_B(2,0) =  0.175623f; _B(2,1) =  0.293600f; _B(2,2) = -0.769231f; _B(2,3) = -1.0f;
	_B(3,0) = -0.175623f; _B(3,1) = -0.293600f; _B(3,2) = -0.769231f; _B(3,3) = -1.0f;

	matrix::geninv(_B,_B_pinv);
	normalize_allocation_matrix();

}

void CdusAllocatorDuct::normalize_allocation_matrix() {
	int num_non_zero_roll_torque = 0;
	int num_non_zero_pitch_torque = 0;

	for (int i = 0; i < NUM_MOTORS; i++) {

		if (fabsf(_B_pinv(i, 0)) > 1e-3f) {
			++num_non_zero_roll_torque;
		}

		if (fabsf(_B_pinv(i, 1)) > 1e-3f) {
			++num_non_zero_pitch_torque;
		}
	}

	float roll_norm_scale = 1.f;

	if (num_non_zero_roll_torque > 0) {
		roll_norm_scale = sqrtf(_B_pinv.col(0).norm_squared() / (num_non_zero_roll_torque / 2.f));
	}

	float pitch_norm_scale = 1.f;

	if (num_non_zero_pitch_torque > 0) {
		pitch_norm_scale = sqrtf(_B_pinv.col(1).norm_squared() / (num_non_zero_pitch_torque / 2.f));
	}

	_control_allocation_scale(0) = fmaxf(roll_norm_scale, pitch_norm_scale);
	_control_allocation_scale(1) = _control_allocation_scale(0);

	// Scale yaw separately
	_control_allocation_scale(2) = _B_pinv.col(2).max();

	if (_control_allocation_scale(0) > FLT_EPSILON) {
		_B_pinv.col(0) /= _control_allocation_scale(0);
		_B_pinv.col(1) /= _control_allocation_scale(1);
	}

	if (_control_allocation_scale(2) > FLT_EPSILON) {
		_B_pinv.col(2) /= _control_allocation_scale(2);
	}

	// Set all the small elements to 0 to avoid issues
	// in the control allocation algorithms
	for (int i = 0; i < NUM_MOTORS; i++) {
		for (int j = 0; j < 4; j++) {
			if (fabsf(_B_pinv(i, j)) < 1e-3f) {
				_B_pinv(i, j) = 0.f;
			}
		}
	}
}

void CdusAllocatorDuct::Run()
{
	if (should_exit()) {
		_torque_sp_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	vehicle_status_s vehicle_status{};
	if (_vehicle_status_sub.update(&vehicle_status)) {
		_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	}

	vehicle_control_mode_s vcm{};
	if (_vehicle_control_mode_sub.update(&vcm)) {
		_rate_control_enabled = vcm.flag_control_rates_enabled;
	}

	// Disarmed or not in rate control => publish zero motors
	if (!_armed || !_rate_control_enabled) {
		actuator_motors_s out{};
		out.timestamp = hrt_absolute_time();
		for (int i = 0; i < NUM_MOTORS; i++) { out.control[i] = 0.f; }
		_actuator_motors_pub.publish(out);
		return;
	}

	vehicle_torque_setpoint_s torque_sp{};
	if (!_torque_sp_sub.update(&torque_sp)) {
		return;
	}

	vehicle_thrust_setpoint_s thrust_sp{};
	_thrust_sp_sub.copy(&thrust_sp);

	manual_control_setpoint_s manual_control_input;
	_manual_control_setpoint_sub.update(&manual_control_input);


	// Build desired vector
	Vector<float, 4> desired{};
	desired(0) = torque_sp.xyz[0];
	desired(1) = torque_sp.xyz[1];
	desired(2) = torque_sp.xyz[2];
	desired(3) = thrust_sp.xyz[2];

	if(_manual_torque_test) {
		desired(0) = 0.1f * manual_control_input.roll;
		desired(1) = 0.1f * manual_control_input.pitch;
		desired(2) = 0.1f * manual_control_input.yaw;
		desired(3) = 0.1f * manual_control_input.throttle;
	}

	// Generate delta PWM for each actuator and normalize
	Vector4f d_PWM = _B_pinv * desired;
	d_PWM(0) /= 800.f;
	d_PWM(1) /= 800.f;
	d_PWM(2) /= 200.f;
	d_PWM(3) /= 200.f;

	for (int i = 0; i < NUM_MOTORS; i++) {
		float cmd = d_PWM(i);
		if (!PX4_ISFINITE(cmd)) cmd = 0.f;
		if (cmd < 0.f) cmd = 0.f;
		if (cmd > 1.f) cmd = 1.f;
		d_PWM(i) = cmd;
	}

	Vector4f actuator_trim(0.65, 0.65, 0.5, 0.5);

	// Solve using pseudo-inverse
	Vector<float, NUM_MOTORS> u = actuator_trim + d_PWM;

	actuator_motors_s out{};
	out.timestamp = hrt_absolute_time();

	for (int i = 0; i < NUM_MOTORS; i++) {
		float cmd = u(i);
		if (!PX4_ISFINITE(cmd)) cmd = 0.f;
		if (cmd < 0.f) cmd = 0.f;
		if (cmd > 1.f) cmd = 1.f;
		out.control[i] = cmd;
	}

	// PX4_INFO("Motor cmds: %.3f %.3f %.3f %.3f %.3f",
    //      (double)out.control[0],
    //      (double)out.control[1],
    //      (double)out.control[2],
    //      (double)out.control[3],
	//  (double)_hover_thrust
	// );


	_actuator_motors_pub.publish(out);
}

int CdusAllocatorDuct::task_spawn(int argc, char *argv[])
{
	CdusAllocatorDuct *instance = new CdusAllocatorDuct();
	if (!instance) {
		PX4_ERR("Allocation failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;
	return PX4_OK;
}

CdusAllocatorDuct *CdusAllocatorDuct::instantiate(int argc, char *argv[])
{
	return new CdusAllocatorDuct();
}

int CdusAllocatorDuct::custom_command(int argc, char *argv[])
{
	return print_usage("Unknown command");
}

int CdusAllocatorDuct::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR(
CdusAllocatorDuct — simple ducted drone control allocator.

Hard-coded geometry + pseudo-inverse mapping:
    torque/thrust → actuator_motors

Drop-in replacement for ControlAllocator for duct-only experiments.
)DESCR"
	);

	PRINT_MODULE_USAGE_NAME("cdus_allocator_duct", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");

	return 0;
}

extern "C" __EXPORT int cdus_allocator_duct_main(int argc, char *argv[])
{
	return CdusAllocatorDuct::main(argc, argv);
}
