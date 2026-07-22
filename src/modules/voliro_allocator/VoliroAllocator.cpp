#include "VoliroAllocator.hpp"

#include <cmath>
#include <cstring>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/log.h>

using namespace matrix;
using namespace time_literals;

ModuleBase::Descriptor VoliroAllocator::desc{task_spawn, custom_command, print_usage};
constexpr char VoliroAllocator::TILT_FEEDBACK_NAME[];

VoliroAllocator::VoliroAllocator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	_measured_tilt.setZero();
	updateConfiguration();
}

VoliroAllocator::~VoliroAllocator()
{
	perf_free(_loop_perf);
}

bool VoliroAllocator::init()
{
	if (!_param_enable.get()) {
		PX4_ERR("VOLA_EN is disabled");
		return false;
	}
	if (!_allocator.configured()) {
		PX4_ERR("invalid allocator configuration");
		return false;
	}
	if (!_torque_sp_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}
#ifndef ENABLE_LOCKSTEP_SCHEDULER
	ScheduleDelayed(50_ms);
#endif
	return true;
}

bool VoliroAllocator::updateConfiguration()
{
	updateParams();
	if (_param_tilt_min.get() >= _param_tilt_max.get()) {
		PX4_ERR("invalid tilt limits");
		return false;
	}
	VoliroAllocation updated;
	if (!updated.configure(_param_arm_radius.get(), _param_max_thrust.get(), _param_kappa.get(),
			       _param_regularization.get(), _param_solver_tolerance.get(), _param_max_iterations.get())) {
		return false;
	}
	_allocator = updated;
	return true;
}

void VoliroAllocator::updateTiltFeedback()
{
	debug_array_s feedback;
	while (_debug_array_sub.update(&feedback)) {
		if (feedback.id == TILT_FEEDBACK_ID
		    && strncmp(feedback.name, TILT_FEEDBACK_NAME, sizeof(feedback.name)) == 0) {
			bool valid = true;
			for (int rotor = 0; rotor < VoliroAllocation::NUM_ROTORS; ++rotor) {
				valid &= PX4_ISFINITE(feedback.data[rotor]);
			}
			if (valid) {
				for (int rotor = 0; rotor < VoliroAllocation::NUM_ROTORS; ++rotor) {
					_measured_tilt(rotor) = feedback.data[rotor];
				}
				_last_feedback = feedback.timestamp;
			}
		}
	}
}

void VoliroAllocator::conventionalFallback(const Vector3f &thrust_sp, const Vector3f &torque_sp,
		VoliroAllocation::RotorVector &motor_normalized) const
{
	const float rp_gain = sqrtf(2.f / 3.f);
	for (int rotor = 0; rotor < VoliroAllocation::NUM_ROTORS; ++rotor) {
		const float theta = rotor * M_PI_F / 3.f;
		const float spin = (rotor % 2 == 0) ? 1.f : -1.f;
		const float command = -thrust_sp(2)
				      + rp_gain * (sinf(theta) * torque_sp(0) + cosf(theta) * torque_sp(1))
				      - spin * torque_sp(2);
		motor_normalized(rotor) = math::constrain(command, 0.f, 1.f);
	}
}

VoliroAllocation::RotorVector VoliroAllocator::limitTiltCommand(const VoliroAllocation::RotorVector &target,
		float dt, uint8_t &rate_limited_mask) const
{
	VoliroAllocation::RotorVector command;
	const float max_step = _param_tilt_rate.get() * dt;
	rate_limited_mask = 0;
	for (int rotor = 0; rotor < VoliroAllocation::NUM_ROTORS; ++rotor) {
		const float delta = atan2f(sinf(target(rotor) - _measured_tilt(rotor)),
					   cosf(target(rotor) - _measured_tilt(rotor)));
		const float limited_delta = math::constrain(delta, -max_step, max_step);
		if (fabsf(limited_delta - delta) > 1e-6f) { rate_limited_mask |= 1u << rotor; }
		command(rotor) = math::constrain(_measured_tilt(rotor) + limited_delta,
						 _param_tilt_min.get(), _param_tilt_max.get());
	}
	return command;
}

void VoliroAllocator::publishOutputs(const VoliroAllocation::RotorVector &motor_normalized,
		const VoliroAllocation::RotorVector &tilt_command, uint64_t timestamp_sample)
{
	if (!_publish_controls) { return; }
	actuator_motors_s motors{};
	motors.timestamp = hrt_absolute_time();
	motors.timestamp_sample = timestamp_sample;
	motors.reversible_flags = 0;
	actuator_servos_s servos{};
	servos.timestamp = motors.timestamp;
	servos.timestamp_sample = timestamp_sample;
	for (int i = 0; i < actuator_motors_s::NUM_CONTROLS; ++i) { motors.control[i] = NAN; }
	for (int i = 0; i < actuator_servos_s::NUM_CONTROLS; ++i) { servos.control[i] = NAN; }
	const float tilt_span = _param_tilt_max.get() - _param_tilt_min.get();
	for (int rotor = 0; rotor < VoliroAllocation::NUM_ROTORS; ++rotor) {
		motors.control[rotor] = _armed ? motor_normalized(rotor) : 0.f;
		servos.control[rotor] = 2.f * (tilt_command(rotor) - _param_tilt_min.get()) / tilt_span - 1.f;
	}
	_actuator_motors_pub.publish(motors);
	_actuator_servos_pub.publish(servos);
}

void VoliroAllocator::publishStatus(const VoliroAllocation::WrenchVector &requested,
		const VoliroAllocation::WrenchVector &achieved, const VoliroAllocation::RotorVector &thrust,
		const VoliroAllocation::RotorVector &tilt_command, uint8_t thrust_saturation_mask,
		uint8_t tilt_rate_limited_mask, uint8_t fallback_reason, bool optimization_used,
		bool solver_success, uint16_t iterations, float solver_time_us, uint64_t timestamp_sample)
{
	const hrt_abstime now = hrt_absolute_time();
	const bool feedback_valid = _last_feedback != 0
		&& hrt_elapsed_time(&_last_feedback) <= static_cast<hrt_abstime>(_param_feedback_timeout.get() * 1e6f);
	const VoliroAllocation::WrenchVector residual = achieved - requested;
	const float roll_pitch_scale = _allocator.maxThrust() * _allocator.armRadius() * sqrtf(6.f);
	const float yaw_scale = fmaxf(6.f * _allocator.maxThrust() * _allocator.kappa(), FLT_EPSILON);

	voliro_allocator_status_s status{};
	status.timestamp = now;
	status.timestamp_sample = timestamp_sample;
	status.residual_norm = residual.norm();
	status.solver_time_us = solver_time_us;
	status.feedback_age_s = _last_feedback == 0 ? INFINITY : hrt_elapsed_time(&_last_feedback) * 1e-6f;
	status.iterations = iterations;
	status.thrust_saturation_mask = thrust_saturation_mask;
	status.tilt_rate_limited_mask = tilt_rate_limited_mask;
	status.fallback_reason = fallback_reason;
	status.optimization_used = optimization_used;
	status.solver_success = solver_success;
	status.feedback_valid = feedback_valid;
	for (int axis = 0; axis < 6; ++axis) {
		status.requested_wrench[axis] = requested(axis);
		status.achieved_wrench[axis] = achieved(axis);
		status.residual_wrench[axis] = residual(axis);
		status.measured_tilt[axis] = _measured_tilt(axis);
		status.commanded_tilt[axis] = tilt_command(axis);
		status.commanded_thrust[axis] = thrust(axis);
	}
	_voliro_status_pub.publish(status);

	control_allocator_status_s ca_status{};
	ca_status.timestamp = now;
	ca_status.unallocated_torque[0] = -residual(3) / roll_pitch_scale;
	ca_status.unallocated_torque[1] = residual(4) / roll_pitch_scale;
	ca_status.unallocated_torque[2] = residual(5) / yaw_scale;
	ca_status.unallocated_thrust[2] = residual(2) / (6.f * _allocator.maxThrust());
	ca_status.torque_setpoint_achieved = Vector3f(ca_status.unallocated_torque).norm_squared() < 1e-6f;
	ca_status.thrust_setpoint_achieved = Vector3f(ca_status.unallocated_thrust).norm_squared() < 1e-6f;
	for (int rotor = 0; rotor < VoliroAllocation::NUM_ROTORS; ++rotor) {
		if (thrust_saturation_mask & (1u << rotor)) {
			ca_status.actuator_saturation[rotor] = control_allocator_status_s::ACTUATOR_SATURATION_UPPER;
		}
		if (tilt_rate_limited_mask & (1u << rotor)) {
			ca_status.actuator_saturation[6 + rotor] = control_allocator_status_s::ACTUATOR_SATURATION_UPPER_DYN;
		}
	}
	_control_allocator_status_pub.publish(ca_status);
}

void VoliroAllocator::Run()
{
	if (should_exit()) {
		_torque_sp_sub.unregisterCallback();
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}
	perf_begin(_loop_perf);
#ifndef ENABLE_LOCKSTEP_SCHEDULER
	ScheduleDelayed(50_ms);
#endif
	if (_parameter_update_sub.updated()) {
		parameter_update_s update;
		_parameter_update_sub.copy(&update);
		if (!updateConfiguration()) { PX4_ERR("allocator parameter update rejected"); }
	}
	vehicle_status_s vehicle_status;
	if (_vehicle_status_sub.update(&vehicle_status)) {
		_armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
	}
	vehicle_control_mode_s control_mode;
	if (_vehicle_control_mode_sub.update(&control_mode)) {
		_publish_controls = control_mode.flag_control_allocation_enabled;
	}
	vehicle_land_detected_s land_detected;
	if (_vehicle_land_detected_sub.update(&land_detected)) {
		_landed = land_detected.landed;
	}
	updateTiltFeedback();

	vehicle_torque_setpoint_s torque_message;
	if (!_torque_sp_sub.update(&torque_message)) {
		perf_end(_loop_perf);
		return;
	}
	vehicle_thrust_setpoint_s thrust_message{};
	_thrust_sp_sub.copy(&thrust_message);
	const Vector3f torque_sp(torque_message.xyz);
	const Vector3f thrust_sp(thrust_message.xyz);
	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain((now - _last_run) * 1e-6f, 0.0002f, 0.02f);
	_last_run = now;

	const auto requested = _allocator.setpointToWrench(thrust_sp, torque_sp);
	VoliroAllocation::RotorVector thrust; thrust.setZero();
	VoliroAllocation::RotorVector motor_normalized; motor_normalized.setZero();
	VoliroAllocation::RotorVector target_tilt; target_tilt.setZero();
	uint8_t thrust_saturation_mask = 0;
	uint8_t tilt_rate_limited_mask = 0;
	uint8_t fallback_reason = voliro_allocator_status_s::FALLBACK_NONE;
	bool optimization_used = false;
	bool solver_success = false;
	uint16_t iterations = 0;
	float solver_time_us = 0.f;
	const bool finite_setpoint = thrust_sp.isAllFinite() && torque_sp.isAllFinite();
	const bool feedback_valid = _last_feedback != 0
		&& hrt_elapsed_time(&_last_feedback) <= static_cast<hrt_abstime>(_param_feedback_timeout.get() * 1e6f);

	if (!_armed) {
		fallback_reason = voliro_allocator_status_s::FALLBACK_DISARMED;
		conventionalFallback(finite_setpoint ? thrust_sp : Vector3f{},
				     finite_setpoint ? torque_sp : Vector3f{}, motor_normalized);
	} else if (!finite_setpoint) {
		fallback_reason = voliro_allocator_status_s::FALLBACK_INVALID_SETPOINT;
		conventionalFallback(Vector3f{}, Vector3f{}, motor_normalized);
	} else if (_landed) {
		fallback_reason = voliro_allocator_status_s::FALLBACK_LANDED;
		conventionalFallback(thrust_sp, torque_sp, motor_normalized);
	} else if (!feedback_valid) {
		fallback_reason = voliro_allocator_status_s::FALLBACK_FEEDBACK_STALE;
		conventionalFallback(thrust_sp, torque_sp, motor_normalized);
	} else {
		const hrt_abstime solver_start = hrt_absolute_time();
		const auto result = _allocator.allocate(requested, _measured_tilt);
		solver_time_us = hrt_elapsed_time(&solver_start);
		optimization_used = result.optimization_used;
		solver_success = result.success;
		iterations = result.iterations;
		if (result.success) {
			thrust = result.thrust;
			target_tilt = result.tilt;
			thrust_saturation_mask = result.saturation_mask;
			for (int rotor = 0; rotor < VoliroAllocation::NUM_ROTORS; ++rotor) {
				motor_normalized(rotor) = math::constrain(thrust(rotor) / _allocator.maxThrust(), 0.f, 1.f);
			}
		} else {
			fallback_reason = voliro_allocator_status_s::FALLBACK_SOLVER_FAILED;
			conventionalFallback(thrust_sp, torque_sp, motor_normalized);
		}
	}

	if (fallback_reason != voliro_allocator_status_s::FALLBACK_NONE) {
		target_tilt.setZero();
		for (int rotor = 0; rotor < VoliroAllocation::NUM_ROTORS; ++rotor) {
			thrust(rotor) = motor_normalized(rotor) * _allocator.maxThrust();
		}
	}
	// A stale encoder sample must not become the reference for another incremental
	// command. Command center directly and let the servo's physical rate limit act.
	const auto tilt_command = feedback_valid
		? limitTiltCommand(target_tilt, dt, tilt_rate_limited_mask)
		: target_tilt;
	const auto achieved = _allocator.wrenchFromCommands(thrust, feedback_valid ? _measured_tilt : target_tilt);
	publishOutputs(motor_normalized, tilt_command, torque_message.timestamp_sample);
	publishStatus(requested, achieved, thrust, tilt_command, thrust_saturation_mask, tilt_rate_limited_mask,
		      fallback_reason, optimization_used, solver_success, iterations, solver_time_us,
		      torque_message.timestamp_sample);
	_last_fallback_reason = fallback_reason;
	perf_end(_loop_perf);
}

int VoliroAllocator::task_spawn(int argc, char *argv[])
{
	VoliroAllocator *instance = new VoliroAllocator();
	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;
		if (instance->init()) { return PX4_OK; }
	}
	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;
	return PX4_ERROR;
}

int VoliroAllocator::print_status()
{
	PX4_INFO("armed: %s", _armed ? "yes" : "no");
	PX4_INFO("landed: %s", _landed ? "yes" : "no");
	const double feedback_age = _last_feedback == 0 ? static_cast<double>(INFINITY)
				    : static_cast<double>(hrt_elapsed_time(&_last_feedback)) * 1e-6;
	PX4_INFO("tilt feedback age: %.3f s", feedback_age);
	PX4_INFO("fallback reason: %u", _last_fallback_reason);
	perf_print_counter(_loop_perf);
	return 0;
}

int VoliroAllocator::custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }

int VoliroAllocator::print_usage(const char *reason)
{
	if (reason) { PX4_WARN("%s", reason); }
	PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
Dedicated six-motor, six-independent-tilt Voliro allocator. The first-stage
interface consumes stock multicopter body torque and Z-thrust setpoints. It
requires measured tilt feedback named VOLA_TILT on debug_array id 4242.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("voliro_allocator", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int voliro_allocator_main(int argc, char *argv[])
{
	return ModuleBase::main(VoliroAllocator::desc, argc, argv);
}
