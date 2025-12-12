/****************************************************************************
 *
 *   Backstepping control module (CdusBackstepping)
 *
 ****************************************************************************/

#include "CdusBacksteppingAttitude.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>

using math::constrain;

CdusBacksteppingAttitude::CdusBacksteppingAttitude() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

CdusBacksteppingAttitude::~CdusBacksteppingAttitude()
{
}

bool CdusBacksteppingAttitude::init()
{
	// Run whenever vehicle_angular_velocity updates
	if (!_angular_velocity_sub.registerCallback()) {
		PX4_ERR("angular velocity callback registration failed");
		return false;
	}

	return true;
}


void CdusBacksteppingAttitude::Run()
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
	_rates_body(0) = angular_velocity.xyz[0];
	_rates_body(1) = angular_velocity.xyz[1];
	_rates_body(2) = angular_velocity.xyz[2];

    	// Local position & velocity (NED)
	if (_local_position_sub.updated()) {
		_local_position_sub.copy(&_local_position);

		_vel_est_ned(0) = _local_position.vx; // [m/s] North
		_vel_est_ned(1) = _local_position.vy; // [m/s] East
		_vel_est_ned(2) = _local_position.vz; // [m/s] Down
	}

	// Attitude (quaternion)
	if (_attitude_sub.updated()) {
		_attitude_sub.copy(&_attitude);

		_q_att = matrix::Quatf(_attitude.q[0],
			       _attitude.q[1],
			       _attitude.q[2],
			       _attitude.q[3]);
	}

    // Attitude setpoint (quaternion)
    if (_attitude_setpoint_sub.updated()) {
        _attitude_setpoint_sub.copy(&_attitude_sp);

        _q_att_sp = matrix::Quatf(_attitude_sp.q_d[0],
                        _attitude_sp.q_d[1],
                        _attitude_sp.q_d[2],
                        _attitude_sp.q_d[3]);
        _thrust_sp(2) = _attitude_sp.thrust_body[2];
    }

    calcRollTorque();
    calcPitchTorque();
    calcYawTorque();

	_torque_sp *= _torque_scale;

    // Publish torque setpoint
	vehicle_torque_setpoint_s torque_msg{};
	torque_msg.timestamp_sample = angular_velocity.timestamp_sample;
	torque_msg.timestamp = hrt_absolute_time();

	torque_msg.xyz[0] = PX4_ISFINITE(_torque_sp(0)) ? _torque_sp(0) : 0.f;
	torque_msg.xyz[1] = PX4_ISFINITE(_torque_sp(1)) ? _torque_sp(1) : 0.f;
	torque_msg.xyz[2] = PX4_ISFINITE(_torque_sp(2)) ? _torque_sp(2) : 0.f;

	_torque_sp_pub.publish(torque_msg);

	// Publish thrust setpoint
	vehicle_thrust_setpoint_s thrust_msg{};
	thrust_msg.timestamp_sample = angular_velocity.timestamp_sample;
	thrust_msg.timestamp = hrt_absolute_time();
    
    thrust_msg.xyz[0] = PX4_ISFINITE(_thrust_sp(0)) ? _thrust_sp(0) : 0.f;
	thrust_msg.xyz[1] = PX4_ISFINITE(_thrust_sp(1)) ? _thrust_sp(1) : 0.f;
	thrust_msg.xyz[2] = PX4_ISFINITE(_thrust_sp(2)) ? _thrust_sp(2) : 0.f;

	_thrust_sp_pub.publish(thrust_msg);
}

void CdusBacksteppingAttitude::calcRollTorque() {
    const float Ix = _Ixx;
    const float Iy = _Iyy;
    const float Iz = _Izz;
    const float k1 = _Kv;
    const float k2 = _Ka;
    const float q0 = _q_att(0);
    const float qv1 = _q_att(1);
    const float qv2 = _q_att(2);
    const float qv3 = _q_att(3);
    const float qc1 = _q_att_sp(0);
    const float qc2 = _q_att_sp(1);
    const float qc3 = _q_att_sp(2);
    const float qc4 = _q_att_sp(3);
    const float omega1 = _rates_body(0);
    const float omega2 = _rates_body(1);
    const float omega3 = _rates_body(2);

    _torque_sp(0) = -Ix*(k1*omega1 + k2*omega1 - q0*qc2 + qc1*qv1 - qc3*qv3 + qc4*qv2 - k1*k2*q0*qc2 + k1*k2*qc1*qv1 - k1*k2*qc3*qv3 + k1*k2*qc4*qv2 + Iy*omega2*omega3/Ix - Iz*omega2*omega3/Ix);
}

void CdusBacksteppingAttitude::calcPitchTorque() {
    const float Ix = _Ixx;
    const float Iy = _Iyy;
    const float Iz = _Izz;
    const float k1 = _Kv;
    const float k2 = _Ka;
    const float q0 = _q_att(0);
    const float qv1 = _q_att(1);
    const float qv2 = _q_att(2);
    const float qv3 = _q_att(3);
    const float qc1 = _q_att_sp(0);
    const float qc2 = _q_att_sp(1);
    const float qc3 = _q_att_sp(2);
    const float qc4 = _q_att_sp(3);
    const float omega1 = _rates_body(0);
    const float omega2 = _rates_body(1);
    const float omega3 = _rates_body(2);

    _torque_sp(1) = -Iy*(k1*omega2 + k2*omega2 - q0*qc3 + qc1*qv2 + qc2*qv3 - qc4*qv1 - k1*k2*q0*qc3 + k1*k2*qc1*qv2 + k1*k2*qc2*qv3 - k1*k2*qc4*qv1 - Ix*omega1*omega3/Iy + Iz*omega1*omega3/Iy);
}

void CdusBacksteppingAttitude::calcYawTorque() {
    const float Ix = _Ixx;
    const float Iy = _Iyy;
    const float Iz = _Izz;
    const float k1 = _Kv;
    const float k2 = _Ka;
    const float q0 = _q_att(0);
    const float qv1 = _q_att(1);
    const float qv2 = _q_att(2);
    const float qv3 = _q_att(3);
    const float qc1 = _q_att_sp(0);
    const float qc2 = _q_att_sp(1);
    const float qc3 = _q_att_sp(2);
    const float qc4 = _q_att_sp(3);
    const float omega1 = _rates_body(0);
    const float omega2 = _rates_body(1);
    const float omega3 = _rates_body(2);

    _torque_sp(2) = -Iz*(k1*omega3 + k2*omega3 - q0*qc4 + qc1*qv3 - qc2*qv2 + qc3*qv1 - k1*k2*q0*qc4 + k1*k2*qc1*qv3 - k1*k2*qc2*qv2 + k1*k2*qc3*qv1 + Ix*omega1*omega2/Iz - Iy*omega1*omega2/Iz);
}

/** ModuleBase interface **/

int CdusBacksteppingAttitude::task_spawn(int argc, char *argv[])
{
	CdusBacksteppingAttitude *instance = new CdusBacksteppingAttitude();

	if (!instance) {
		PX4_ERR("backstepping attitude failed");
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

int CdusBacksteppingAttitude::custom_command(int argc, char *argv[])
{
	// No custom commands yet
	return print_usage("unknown command");
}

int CdusBacksteppingAttitude::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
Vertical vehicle backstepping controller.

- Subscribes: manual_control_input, vehicle local position, vehicle_attitude, vehicle_angular_velocity, vehicle_control_mode
- Publishes: vehicle_torque_setpoint, vehicle_thrust_setpoint
- Uses fixed gains for controller.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("cdus_backstepping_attitude", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int cdus_backstepping_attitude_main(int argc, char *argv[])
{
	return CdusBacksteppingAttitude::main(argc, argv);
}