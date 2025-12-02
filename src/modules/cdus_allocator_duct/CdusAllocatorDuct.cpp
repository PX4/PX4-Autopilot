/****************************************************************************
 * CdusAllocator.cpp
 ****************************************************************************/

#include "CdusAllocatorDuct.hpp"
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>
using namespace time_literals;

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
	_B(0,0) =  0.00000f; _B(0,1) =  0.00000f; _B(0,2) = -0.00031f; _B(0,3) =  0.00135f;
	_B(1,0) =  0.00000f; _B(1,1) =  0.00000f; _B(1,2) =  0.00055f; _B(1,3) =  0.00074f;
	_B(2,0) =  0.00030f; _B(2,1) = -0.00030f; _B(2,2) =  0.00000f; _B(2,3) =  0.00000f;
	_B(3,0) =  0.01037f; _B(3,1) =  0.00967f; _B(3,2) =  0.00000f; _B(3,3) =  0.00000f;
	matrix::geninv(_B,_B_pinv);
	// normalize_allocation_matrix();

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
	// When using backstepping controller make sure not to flag _rate_control_enabled as there will be no rate control
	if (!_armed || !_rate_control_enabled) {
		actuator_motors_s out{};
		out.timestamp = hrt_absolute_time();
		for (int i = 0; i < NUM_MOTORS; i++) { out.control[i] = 0.f; }
		_actuator_motors_pub.publish(out);
		return;
	}

	// Get torque setpoint from controllers
	vehicle_torque_setpoint_s torque_sp{};
	if (!_torque_sp_sub.update(&torque_sp)) {
		return;
	}

	// Get thrust setpoint from controllers
	vehicle_thrust_setpoint_s thrust_sp{};
	_thrust_sp_sub.copy(&thrust_sp);

	// For open loop testing get manual control setpoint
	if(_manual_control_setpoint_sub.updated()){
		_manual_control_setpoint_sub.copy(&_manual_control_input);
	}


	// Build desired vector
	Vector<float, 4> desired{};
	desired(0) = -torque_sp.xyz[0];
	desired(1) = torque_sp.xyz[1];
	desired(2) = -torque_sp.xyz[2];
	desired(3) = -thrust_sp.xyz[2];

	// Set _manual_torque_test true in header for open loop testing
	if(_manual_torque_test) {
		desired(0) = -0.15f * _manual_control_input.roll;
		desired(1) = -0.15f * _manual_control_input.pitch;
		desired(2) = -0.15f * _manual_control_input.yaw;
		desired(3) = 1.0f * _manual_control_input.throttle;
	}

	// PX4_INFO("Torques: %.3f %.3f %.3f %.3f",
    //      (double)desired(0),
    //      (double)desired(1),
	// 	 (double)desired(2),
	// 	 (double)desired(3)
	// );

	// Generate delta PWM for each actuator and normalize
	Vector4f d_PWM = _B_pinv * desired;
	float d_s1 = d_PWM(2);
	float d_s2 = d_PWM(3);
	const float d_s1_lim = 200.f;
	const float d_s2_lim = 200.f;

	//clip and preserve ratios of servo d_PWM vals
	if(std::fabs(d_s2) > 1e-3f) {
		const float r = d_PWM(2) / d_PWM(3);

		if(std::fabs(d_s1) > d_s1_lim && std::fabs(d_s2) < d_s2_lim) {
			d_PWM(3) = (d_s1_lim / r) * (d_s2 / std::fabs(d_s2));
			d_PWM(2) = d_s1_lim * (d_s1 / std::fabs(d_s1));
		} 

		else if(std::fabs(d_s2) > d_s2_lim && std::fabs(d_s1) < d_s1_lim) {
			d_PWM(2) = (d_s2_lim * r) * (d_s1 / std::fabs(d_s1));
			d_PWM(3) = d_s2_lim * (d_s2 / std::fabs(d_s2));
		}

		else if(std::fabs(d_s2) > d_s2_lim && std::fabs(d_s1) > d_s1_lim) {
			if(std::fabs(r) >= std::fabs(d_s1_lim/d_s2_lim)) {
				d_PWM(3) = (d_s1_lim / r) * (d_s2 / std::fabs(d_s2));
				d_PWM(2) = d_s1_lim * (d_s1 / std::fabs(d_s1));
			} else {
				d_PWM(2) = (d_s2_lim * r) * (d_s1 / std::fabs(d_s1));
				d_PWM(3) = d_s2_lim * (d_s2 / std::fabs(d_s2));
			}
		}
	}

	Vector4f actuator_trim(1570.f, 1570.f, 1450.f, 1450.f);

	// Solve using pseudo-inverse
	Vector<float, NUM_MOTORS> u = actuator_trim + d_PWM;

	const float s_max = 1650.f;
	const float s_min = 1250.f;

	const float m_max = 2000.f;
	const float m_min = 1000.f;

	// normalize
	for(int i=0; i < 2; i++) {
		u(i) = (u(i) - m_min) / (m_max - m_min);
	}

	for(int i=2; i < 4; i++) {
		u(i) = (u(i) - s_min) / (s_max - s_min);
	}

	actuator_motors_s out{};
	out.timestamp = hrt_absolute_time();

	// Ensure motor actuator commands stay [0,1]
	for (int i = 0; i < NUM_MOTORS; i++) {
		float cmd = u(i);
		if (!PX4_ISFINITE(cmd)) cmd = 0.f;
		if (cmd < 0.f) cmd = 0.f;
		if (cmd > 1.f) cmd = 1.f;
		out.control[i] = cmd;
	}



	// PX4_INFO("Motor cmds: %.3f %.3f %.3f %.3f",
    //      (double)out.control[0],
    //      (double)out.control[1],
    //      (double)out.control[2],
    //      (double)out.control[3]
	// );

	// Publish normalized acutator setpoints
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
