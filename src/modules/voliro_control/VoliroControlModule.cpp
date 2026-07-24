#include "VoliroControlModule.hpp"

#include <cmath>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/log.h>

using namespace matrix;
using namespace time_literals;

ModuleBase::Descriptor VoliroControlModule::desc{task_spawn, custom_command, print_usage};

VoliroControlModule::VoliroControlModule() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_setpoint_angular_acceleration.setZero();
	_previous_setpoint_angular_velocity.setZero();
	updateConfiguration();
}

VoliroControlModule::~VoliroControlModule()
{
	perf_free(_loop_perf);
}

bool VoliroControlModule::init()
{
	if (!_param_enable.get()) {
		PX4_ERR("VCTRL_EN is disabled");
		return false;
	}

	if (!_controller.configured()) {
		PX4_ERR("invalid controller configuration");
		return false;
	}

	if (!_odometry_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

bool VoliroControlModule::updateConfiguration()
{
	updateParams();
	VoliroControl::Configuration configuration{};
	configuration.mass = _param_mass.get();
	configuration.gravity = 9.81f;
	configuration.inertia = Vector3f{_param_ixx.get(), _param_iyy.get(), _param_izz.get()};
	configuration.position_gain =
		Vector3f{_param_position_x.get(), _param_position_y.get(), _param_position_z.get()};
	configuration.velocity_gain =
		Vector3f{_param_velocity_x.get(), _param_velocity_y.get(), _param_velocity_z.get()};
	configuration.attitude_gain =
		Vector3f{_param_attitude_roll.get(), _param_attitude_pitch.get(), _param_attitude_yaw.get()};
	configuration.angular_rate_gain =
		Vector3f{_param_rate_roll.get(), _param_rate_pitch.get(), _param_rate_yaw.get()};
	configuration.max_rotor_thrust = _param_max_thrust.get();
	configuration.arm_radius = _param_arm_radius.get();
	configuration.kappa = _param_kappa.get();
	return _controller.configure(configuration);
}

bool VoliroControlModule::updateSetpoint()
{
	trajectory_setpoint6dof_s newest{};
	bool updated = false;

	while (_trajectory_setpoint_sub.update(&newest)) {
		updated = true;
	}

	if (!updated) {
		return false;
	}

	const Vector3f angular_velocity{newest.angular_velocity};
	_setpoint_angular_acceleration.setZero();

	if (_previous_setpoint_timestamp != 0 && newest.timestamp > _previous_setpoint_timestamp
	    && angular_velocity.isAllFinite() && _previous_setpoint_angular_velocity.isAllFinite()) {
		const float dt = (newest.timestamp - _previous_setpoint_timestamp) * 1e-6f;

		if (dt >= 0.002f && dt <= 0.1f) {
			_setpoint_angular_acceleration =
				(angular_velocity - _previous_setpoint_angular_velocity) / dt;
		}
	}

	_previous_setpoint_timestamp = newest.timestamp;
	_previous_setpoint_angular_velocity = angular_velocity;
	_setpoint = newest;
	_last_valid_setpoint = hrt_absolute_time();
	return true;
}

bool VoliroControlModule::controlEnabled() const
{
	return _param_enable.get()
	       && _vehicle_control_mode.flag_control_offboard_enabled
	       && _vehicle_control_mode.flag_control_allocation_enabled
	       && _offboard_control_mode.thrust_and_torque;
}

bool VoliroControlModule::setpointValid(hrt_abstime now) const
{
	const Quatf attitude_setpoint{_setpoint.quaternion};
	const bool finite = Vector3f{_setpoint.position}.isAllFinite()
			    && Vector3f{_setpoint.velocity}.isAllFinite()
			    && Vector3f{_setpoint.acceleration}.isAllFinite()
			    && attitude_setpoint.isAllFinite() && attitude_setpoint.norm() > 1e-6f
			    && Vector3f{_setpoint.angular_velocity}.isAllFinite();

	if (!finite || _setpoint.timestamp == 0 || _last_valid_setpoint == 0) {
		return false;
	}

	const hrt_abstime timeout =
		static_cast<hrt_abstime>(math::max(_param_setpoint_timeout.get(), 0.01f) * 1e6f);
	const bool receive_time_valid = now >= _last_valid_setpoint && now - _last_valid_setpoint <= timeout;
	const bool source_time_valid = now >= _setpoint.timestamp
				       ? now - _setpoint.timestamp <= timeout
				       : _setpoint.timestamp - now <= 50_ms;
	return receive_time_valid && source_time_valid;
}

bool VoliroControlModule::stateFromOdometry(const vehicle_odometry_s &odometry,
		VoliroControl::State &state) const
{
	if (odometry.pose_frame != vehicle_odometry_s::POSE_FRAME_NED) {
		return false;
	}

	state.position_ned = Vector3f{odometry.position};
	state.attitude_ned_frd = Quatf{odometry.q};
	state.angular_velocity_frd = Vector3f{odometry.angular_velocity};

	if (odometry.velocity_frame == vehicle_odometry_s::VELOCITY_FRAME_NED) {
		state.velocity_ned = Vector3f{odometry.velocity};

	} else if (odometry.velocity_frame == vehicle_odometry_s::VELOCITY_FRAME_BODY_FRD) {
		state.velocity_ned = state.attitude_ned_frd.rotateVector(Vector3f{odometry.velocity});

	} else {
		return false;
	}

	return state.position_ned.isAllFinite() && state.velocity_ned.isAllFinite()
	       && state.attitude_ned_frd.isAllFinite() && state.attitude_ned_frd.norm() > 1e-6f
	       && state.angular_velocity_frd.isAllFinite();
}

void VoliroControlModule::publishWrench(const VoliroControl::Output &output, uint64_t timestamp_sample)
{
	const hrt_abstime now = hrt_absolute_time();
	vehicle_thrust_setpoint_s thrust{};
	thrust.timestamp = now;
	thrust.timestamp_sample = timestamp_sample;
	output.thrust_normalized.copyTo(thrust.xyz);
	_thrust_setpoint_pub.publish(thrust);

	// The allocator work item is triggered by torque, so publish it after the
	// matching thrust sample.
	vehicle_torque_setpoint_s torque{};
	torque.timestamp = now;
	torque.timestamp_sample = timestamp_sample;
	output.torque_normalized.copyTo(torque.xyz);
	_torque_setpoint_pub.publish(torque);
}

void VoliroControlModule::Run()
{
	if (should_exit()) {
		_odometry_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_loop_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s update{};
		_parameter_update_sub.copy(&update);

		if (!updateConfiguration()) {
			PX4_ERR("controller parameter update rejected");
		}
	}

	_offboard_control_mode_sub.update(&_offboard_control_mode);
	_vehicle_control_mode_sub.update(&_vehicle_control_mode);
	updateSetpoint();

	vehicle_odometry_s odometry{};

	if (!_odometry_sub.update(&odometry)) {
		perf_end(_loop_perf);
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	VoliroControl::State state{};
	_active = controlEnabled() && setpointValid(now) && stateFromOdometry(odometry, state);

	if (_active) {
		VoliroControl::Setpoint setpoint{};
		setpoint.position_ned = Vector3f{_setpoint.position};
		setpoint.velocity_ned = Vector3f{_setpoint.velocity};
		setpoint.acceleration_ned = Vector3f{_setpoint.acceleration};
		setpoint.attitude_ned_frd = Quatf{_setpoint.quaternion};
		setpoint.angular_velocity_frd = Vector3f{_setpoint.angular_velocity};
		setpoint.angular_acceleration_frd = _setpoint_angular_acceleration;
		_last_output = _controller.calculate(state, setpoint);

		if (_last_output.valid) {
			publishWrench(_last_output, odometry.timestamp_sample);

		} else {
			_active = false;
		}
	}

	perf_end(_loop_perf);
}

int VoliroControlModule::task_spawn(int argc, char *argv[])
{
	VoliroControlModule *instance = new VoliroControlModule();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;
	return PX4_ERROR;
}

int VoliroControlModule::print_status()
{
	PX4_INFO("active: %s", _active ? "yes" : "no");
	const double setpoint_age = _last_valid_setpoint == 0 ? static_cast<double>(INFINITY)
				    : hrt_elapsed_time(&_last_valid_setpoint) * 1e-6;
	PX4_INFO("setpoint receive age: %.3f s", setpoint_age);
	PX4_INFO("force limited: %s", _last_output.force_limited ? "yes" : "no");
	PX4_INFO("torque limited mask: 0x%02x", _last_output.torque_limited_mask);
	perf_print_counter(_loop_perf);
	return 0;
}

int VoliroControlModule::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int VoliroControlModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
Fully actuated Voliro position-and-attitude controller. It consumes NED/FRD
trajectory_setpoint6dof references and estimator vehicle_odometry, then
publishes normalized three-axis body thrust and torque for voliro_allocator.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("voliro_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int voliro_control_main(int argc, char *argv[])
{
	return ModuleBase::main(VoliroControlModule::desc, argc, argv);
}
