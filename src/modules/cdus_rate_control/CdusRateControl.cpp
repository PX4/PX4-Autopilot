/****************************************************************************
 *
 *   Simple quadcopter rate controller module (CdusRateControl)
 *
 ****************************************************************************/

#include "CdusRateControl.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>

using math::constrain;

CdusRateControl::CdusRateControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	// nothing else to do here
}

CdusRateControl::~CdusRateControl()
{
	// _angular_velocity_sub.unregisterCallback();
}

bool CdusRateControl::init()
{
	// Run whenever vehicle_angular_velocity updates
	if (!_angular_velocity_sub.registerCallback()) {
		PX4_ERR("angular velocity callback registration failed");
		return false;
	}

	return true;
}

void CdusRateControl::reset_integrator()
{
	_integral.setAll(0.f);
}

void CdusRateControl::Run()
{
	if (should_exit()) {
		_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	vehicle_angular_velocity_s angular_velocity{};

	// Run only when we have a new gyro sample
	if (!_angular_velocity_sub.update(&angular_velocity)) {
		return;
	}

	const uint64_t now = angular_velocity.timestamp_sample;

	// Compute dt from gyro sample timestamps (in seconds)
	if (_last_run == 0) {
		_last_run = now;
		return; // wait one cycle to get a valid dt
	}

	float dt = (now - _last_run) * 1e-6f;
	_last_run = now;

	// Guard against unreasonable dt values
	dt = constrain(dt, 0.0005f, 0.02f);

	// Current body rates and acceleration
	const Vector3f rates{angular_velocity.xyz};
	const Vector3f rates_derivative{angular_velocity.xyz_derivative};

	// Update control mode (for enabling/disabling controller)
	_control_mode_sub.update(&_control_mode);

	// If rate control is not enabled, do nothing
	if (!_control_mode.flag_control_rates_enabled) {
		reset_integrator();
		return;
	}

	// Get latest rate + thrust setpoint (if any)
	vehicle_rates_setpoint_s rates_sp_msg{};

	if (_rates_sp_sub.copy(&rates_sp_msg)) {
		_rate_sp(0) = PX4_ISFINITE(rates_sp_msg.roll)  ? rates_sp_msg.roll  : 0.f;
		_rate_sp(1) = PX4_ISFINITE(rates_sp_msg.pitch) ? rates_sp_msg.pitch : 0.f;
		_rate_sp(2) = PX4_ISFINITE(rates_sp_msg.yaw)   ? rates_sp_msg.yaw   : 0.f;

		_thrust_sp = Vector3f(rates_sp_msg.thrust_body);
	}

	// Reset integrator when disarmed
	if (!_control_mode.flag_armed) {
		reset_integrator();
	}

	// PID rate control
	const Vector3f error = _rate_sp - rates;

	// Integrator with simple clamping
	_integral += error * dt;

	for (int i = 0; i < 3; i++) {
		_integral(i) = constrain(_integral(i), -_integrator_limit, _integrator_limit);
	}

	// P + I - D * (measured angular acceleration)
	const Vector3f torque_sp =
		_k_p.emult(error) +
		_k_i.emult(_integral) -
		_k_d.emult(rates_derivative);

	// Publish torque setpoint
	vehicle_torque_setpoint_s torque_msg{};
	torque_msg.timestamp_sample = angular_velocity.timestamp_sample;
	torque_msg.timestamp = hrt_absolute_time();

	torque_msg.xyz[0] = PX4_ISFINITE(torque_sp(0)) ? torque_sp(0) : 0.f;
	torque_msg.xyz[1] = PX4_ISFINITE(torque_sp(1)) ? torque_sp(1) : 0.f;
	torque_msg.xyz[2] = PX4_ISFINITE(torque_sp(2)) ? torque_sp(2) : 0.f;

	_torque_sp_pub.publish(torque_msg);

	// Publish thrust setpoint (pass-through from vehicle_rates_setpoint)
	vehicle_thrust_setpoint_s thrust_msg{};
	thrust_msg.timestamp_sample = angular_velocity.timestamp_sample;
	thrust_msg.timestamp = hrt_absolute_time();

	_thrust_sp.copyTo(thrust_msg.xyz);
	_thrust_sp_pub.publish(thrust_msg);
}

/** ModuleBase interface **/

int CdusRateControl::task_spawn(int argc, char *argv[])
{
	CdusRateControl *instance = new CdusRateControl();

	if (!instance) {
		PX4_ERR("allocation failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (instance->init()) {
		return PX4_OK;
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int CdusRateControl::custom_command(int argc, char *argv[])
{
	// No custom commands yet
	return print_usage("unknown command");
}

int CdusRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
Simple quadcopter rate controller.

- Subscribes: vehicle_angular_velocity, vehicle_rates_setpoint, vehicle_control_mode
- Publishes: vehicle_torque_setpoint, vehicle_thrust_setpoint
- Uses fixed PID gains on body rates (roll, pitch, yaw).
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("cdus_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int cdus_rate_control_main(int argc, char *argv[])
{
	return CdusRateControl::main(argc, argv);
}
