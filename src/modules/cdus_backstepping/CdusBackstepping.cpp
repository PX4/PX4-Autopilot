/****************************************************************************
 *
 *   Backstepping control module (CdusBackstepping)
 *
 ****************************************************************************/

#include "CdusBackstepping.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>


using math::constrain;

CdusBackstepping::CdusBackstepping() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

CdusBackstepping::~CdusBackstepping()
{
}

bool CdusBackstepping::init()
{
	// Run whenever vehicle_angular_velocity updates
	if (!_angular_velocity_sub.registerCallback()) {
		PX4_ERR("angular velocity callback registration failed");
		return false;
	}

	return true;
}

void CdusBackstepping::Run()
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
	// const Vector3f rates_derivative{angular_velocity.xyz_derivative};

	// Get other updates
	// Manual control input
	if (_manual_control_sub.updated()) {
		_manual_control_sub.copy(&_manual_control);
		updateVelocitySp();
		updateYawRateSp();
	}

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

	// Get thrust and torque through backstepping equations
	computeThrust();
	computeYawTorque();
	computeRollTorque();
	computePitchTorque();

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

void CdusBackstepping::updateVelocitySp() {
	_vel_sp_ned(0) = -_vel_scale*_manual_control.roll;
	_vel_sp_ned(1) = _vel_scale*_manual_control.pitch;
	_vel_sp_ned(2) = -_vel_scale*_manual_control.throttle;
}

void CdusBackstepping::updateYawRateSp() {
	_omega_z_sp = _omega_z_scale * _manual_control.yaw;
}

/* Backstepping specific equations */
void CdusBackstepping::computeThrust()
{
	const float g = 9.81f;

	// VI_z is NED velocity z, VI_zc is its setpoint
	const float VI_z  = _vel_est_ned(2);
	const float VI_zc = _vel_sp_ned(2);

	// Numerator
	const float top = -(_mass * (g - _Cd * VI_z + _Kv * (VI_z - VI_zc)));

	// Denominator using attitude quaternion: q = [w, x, y, z]
	const float q1 = _q_att(1);
	const float q2 = _q_att(2);

	const float bottom = (q1 * q1 * 2.0f + q2 * q2 * 2.0f - 1.0f);

	// Avoid divide-by-zero; if bottom is very small, return 0 thrust
	if (std::fabs(bottom) < 1e-6f) {
		return;
	}

	const float thrust = top / bottom; // positive upward [N]

	_thrust_sp(0) = 0.0;
	_thrust_sp(1) = 0.0;
	_thrust_sp(2) = -thrust;
}

void CdusBackstepping::computeYawTorque()
{
	const float omega1 = _rates_body(0); // p
	const float omega2 = _rates_body(1); // q
	const float omega3 = _rates_body(2); // r

	const float N =
		_Izz * _omega_z_sp * _Kv
		- _Izz * _Kv * omega3
		- _Ixx * omega1 * omega2
		+ _Iyy * omega1 * omega2;

	// Original code sets _torque_sp(2) = -N;
	// Here we return the same sign convention as that torque_sp(2) content:
	_torque_sp(2) = N;
}

void CdusBackstepping::computeRollTorque()
{
	const float I_x = _Ixx;
	const float I_y = _Iyy;
	const float I_z = _Izz;
	const float K_d = _Cd;

	const float V_I1 = _vel_est_ned(0);
	const float V_I2 = _vel_est_ned(1);
	const float Vxc  = _vel_sp_ned(0);
	const float Vyc  = _vel_sp_ned(1);

	const float k1 = _Kv;
	const float k2 = _Ka;
	const float k3 = _Ki;

	const float mass = _mass;

	const float omega1 = _rates_body(0);
	const float omega2 = _rates_body(1);
	const float omega3 = _rates_body(2);

	const float q_0  = _q_att(0);
	const float q_v1 = _q_att(1);
	const float q_v2 = _q_att(2);
	const float q_v3 = _q_att(3);

	// In your ROS code: T = -_thrust_sp(2) (thrust positive upward)
	const float T = -_thrust_sp(2);

	auto sq   = [](float x) { return x * x; };
	auto cube = [](float x) { return x * x * x; };

	// t2..t24
	const float t2  = sq(K_d);
	const float t3  = cube(K_d);
	const float t4  = sq(omega1);
	const float t5  = sq(omega2);
	const float t6  = sq(q_0);
	const float t8  = sq(q_v1);
	const float t9  = sq(q_v2);
	const float t11 = sq(q_v3);
	const float t14 = 1.0f / I_x;
	const float t15 = 1.0f / I_y;
	const float t16 = 1.0f / T;
	const float t17 = 1.0f / mass;

	const float t7  = sq(t6);
	const float t10 = sq(t8);
	const float t12 = sq(t9);
	const float t13 = sq(t11);
	const float t20 = t6 * t11 * 2.0f;
	const float t21 = t8 * t9 * 2.0f;
	const float t18 = -t10;
	const float t19 = -t12;
	const float t22 = -t21;
	const float t23 = t7 + t13 + t18 + t19 + t20 + t22;
	const float t24 = 1.0f / t23;

	// et1..et6
	const float et1 =
		K_d * V_I1 * 2.0f - V_I1 * k3 + Vxc * k3 + V_I1 * t3 - V_I1 * k1 * t2 - V_I1 * k2 * t2 - V_I1 * k3 * t2
		+ T * q_0 * q_v2 * t17 * 4.0f + T * q_v1 * q_v3 * t17 * 4.0f
		+ K_d * V_I1 * k1 * k2 + K_d * V_I1 * k1 * k3 + K_d * V_I1 * k2 * k3
		- V_I1 * k1 * k2 * k3 + Vxc * k1 * k2 * k3
		- K_d * T * omega2 * t6 * t17 - K_d * T * omega2 * t8 * t17 + K_d * T * omega2 * t9 * t17 + K_d * T * omega2 * t11 * t17
		+ T * k1 * omega2 * t6 * t17 + T * k2 * omega2 * t6 * t17 + T * k1 * omega2 * t8 * t17 + T * k3 * omega2 * t6 * t17
		- T * k1 * omega2 * t9 * t17 + T * k2 * omega2 * t8 * t17 - T * k2 * omega2 * t9 * t17 + T * k3 * omega2 * t8 * t17
		- T * k1 * omega2 * t11 * t17 - T * k3 * omega2 * t9 * t17 - T * k2 * omega2 * t11 * t17 - T * k3 * omega2 * t11 * t17
		+ T * omega1 * omega3 * t6 * t17 + T * omega1 * omega3 * t8 * t17;

	const float et2 =
		- T * omega1 * omega3 * t9 * t17 - T * omega1 * omega3 * t11 * t17
		+ T * q_0 * q_v2 * t2 * t17 * 2.0f - T * q_0 * q_v2 * t4 * t17 * 2.0f - T * q_0 * q_v2 * t5 * t17 * 2.0f
		+ T * q_v1 * q_v3 * t2 * t17 * 2.0f - T * q_v1 * q_v3 * t4 * t17 * 2.0f - T * q_v1 * q_v3 * t5 * t17 * 2.0f
		- K_d * T * k1 * q_0 * q_v2 * t17 * 2.0f - K_d * T * k2 * q_0 * q_v2 * t17 * 2.0f - K_d * T * k3 * q_0 * q_v2 * t17 * 2.0f
		- K_d * T * k1 * q_v1 * q_v3 * t17 * 2.0f - K_d * T * k2 * q_v1 * q_v3 * t17 * 2.0f - K_d * T * k3 * q_v1 * q_v3 * t17 * 2.0f
		- K_d * T * omega1 * q_0 * q_v3 * t17 * 2.0f + K_d * T * omega1 * q_v1 * q_v2 * t17 * 2.0f
		+ T * k1 * k2 * q_0 * q_v2 * t17 * 2.0f + T * k1 * k3 * q_0 * q_v2 * t17 * 2.0f + T * k2 * k3 * q_0 * q_v2 * t17 * 2.0f
		+ T * k1 * k2 * q_v1 * q_v3 * t17 * 2.0f + T * k1 * k3 * q_v1 * q_v3 * t17 * 2.0f + T * k2 * k3 * q_v1 * q_v3 * t17 * 2.0f
		+ T * k1 * omega1 * q_0 * q_v3 * t17 * 2.0f + T * k2 * omega1 * q_0 * q_v3 * t17 * 2.0f;

	const float et3 =
		T * k3 * omega1 * q_0 * q_v3 * t17 * 2.0f - T * k1 * omega1 * q_v1 * q_v2 * t17 * 2.0f - T * k2 * omega1 * q_v1 * q_v2 * t17 * 2.0f
		- T * k3 * omega1 * q_v1 * q_v2 * t17 * 2.0f - T * omega2 * omega3 * q_0 * q_v3 * t17 * 2.0f + T * omega2 * omega3 * q_v1 * q_v2 * t17 * 2.0f
		- I_x * T * omega1 * omega3 * t6 * t15 * t17 - I_x * T * omega1 * omega3 * t8 * t15 * t17 + I_x * T * omega1 * omega3 * t9 * t15 * t17
		+ I_x * T * omega1 * omega3 * t11 * t15 * t17 + I_z * T * omega1 * omega3 * t6 * t15 * t17 + I_z * T * omega1 * omega3 * t8 * t15 * t17
		- I_z * T * omega1 * omega3 * t9 * t15 * t17 - I_z * T * omega1 * omega3 * t11 * t15 * t17
		+ I_y * T * omega2 * omega3 * q_0 * q_v3 * t14 * t17 * 2.0f - I_z * T * omega2 * omega3 * q_0 * q_v3 * t14 * t17 * 2.0f
		- I_y * T * omega2 * omega3 * q_v1 * q_v2 * t14 * t17 * 2.0f + I_z * T * omega2 * omega3 * q_v1 * q_v2 * t14 * t17 * 2.0f;

	const float et4 =
		K_d * V_I2 * 2.0f - V_I2 * k3 + Vyc * k3 + V_I2 * t3 - V_I2 * k1 * t2 - V_I2 * k2 * t2 - V_I2 * k3 * t2
		- T * q_0 * q_v1 * t17 * 4.0f + T * q_v2 * q_v3 * t17 * 4.0f
		+ K_d * V_I2 * k1 * k2 + K_d * V_I2 * k1 * k3 + K_d * V_I2 * k2 * k3
		- V_I2 * k1 * k2 * k3 + Vyc * k1 * k2 * k3
		+ K_d * T * omega1 * t6 * t17 - K_d * T * omega1 * t8 * t17 + K_d * T * omega1 * t9 * t17 - K_d * T * omega1 * t11 * t17
		- T * k1 * omega1 * t6 * t17 - T * k2 * omega1 * t6 * t17 + T * k1 * omega1 * t8 * t17 - T * k3 * omega1 * t6 * t17
		- T * k1 * omega1 * t9 * t17 + T * k2 * omega1 * t8 * t17 - T * k2 * omega1 * t9 * t17 + T * k3 * omega1 * t8 * t17
		+ T * k1 * omega1 * t11 * t17 - T * k3 * omega1 * t9 * t17 + T * k2 * omega1 * t11 * t17 + T * k3 * omega1 * t11 * t17
		+ T * omega2 * omega3 * t6 * t17 - T * omega2 * omega3 * t8 * t17;

	const float et5 =
		T * omega2 * omega3 * t9 * t17 - T * omega2 * omega3 * t11 * t17
		- T * q_0 * q_v1 * t2 * t17 * 2.0f + T * q_0 * q_v1 * t4 * t17 * 2.0f + T * q_0 * q_v1 * t5 * t17 * 2.0f
		+ T * q_v2 * q_v3 * t2 * t17 * 2.0f - T * q_v2 * q_v3 * t4 * t17 * 2.0f - T * q_v2 * q_v3 * t5 * t17 * 2.0f
		+ K_d * T * k1 * q_0 * q_v1 * t17 * 2.0f + K_d * T * k2 * q_0 * q_v1 * t17 * 2.0f + K_d * T * k3 * q_0 * q_v1 * t17 * 2.0f
		- K_d * T * k1 * q_v2 * q_v3 * t17 * 2.0f - K_d * T * k2 * q_v2 * q_v3 * t17 * 2.0f - K_d * T * k3 * q_v2 * q_v3 * t17 * 2.0f
		- K_d * T * omega2 * q_0 * q_v3 * t17 * 2.0f - K_d * T * omega2 * q_v1 * q_v2 * t17 * 2.0f
		- T * k1 * k2 * q_0 * q_v1 * t17 * 2.0f - T * k1 * k3 * q_0 * q_v1 * t17 * 2.0f - T * k2 * k3 * q_0 * q_v1 * t17 * 2.0f
		+ T * k1 * k2 * q_v2 * q_v3 * t17 * 2.0f + T * k1 * k3 * q_v2 * q_v3 * t17 * 2.0f + T * k2 * k3 * q_v2 * q_v3 * t17 * 2.0f
		+ T * k1 * omega2 * q_0 * q_v3 * t17 * 2.0f + T * k2 * omega2 * q_0 * q_v3 * t17 * 2.0f;

	const float et6 =
		T * k3 * omega2 * q_0 * q_v3 * t17 * 2.0f + T * k1 * omega2 * q_v1 * q_v2 * t17 * 2.0f + T * k2 * omega2 * q_v1 * q_v2 * t17 * 2.0f
		+ T * k3 * omega2 * q_v1 * q_v2 * t17 * 2.0f + T * omega1 * omega3 * q_0 * q_v3 * t17 * 2.0f + T * omega1 * omega3 * q_v1 * q_v2 * t17 * 2.0f
		- I_y * T * omega2 * omega3 * t6 * t14 * t17 + I_y * T * omega2 * omega3 * t8 * t14 * t17
		- I_y * T * omega2 * omega3 * t9 * t14 * t17 + I_y * T * omega2 * omega3 * t11 * t14 * t17
		+ I_z * T * omega2 * omega3 * t6 * t14 * t17 - I_z * T * omega2 * omega3 * t8 * t14 * t17
		+ I_z * T * omega2 * omega3 * t9 * t14 * t17 - I_z * T * omega2 * omega3 * t11 * t14 * t17
		- I_x * T * omega1 * omega3 * q_0 * q_v3 * t15 * t17 * 2.0f + I_z * T * omega1 * omega3 * q_0 * q_v3 * t15 * t17 * 2.0f
		- I_x * T * omega1 * omega3 * q_v1 * q_v2 * t15 * t17 * 2.0f + I_z * T * omega1 * omega3 * q_v1 * q_v2 * t15 * t17 * 2.0f;

	const float L =
		I_x * mass * t16 * t24 * (q_0 * q_v3 + q_v1 * q_v2) * (et1 + et2 + et3) * -2.0f
		+ I_x * mass * t16 * t24 * (et4 + et5 + et6) * (t6 + t8 - t9 - t11);

	_torque_sp(0) = L;
}

void CdusBackstepping::computePitchTorque()
{
	const float I_x = _Ixx;
	const float I_y = _Iyy;
	const float I_z = _Izz;
	const float K_d = _Cd;

	const float V_I1 = _vel_est_ned(0);
	const float V_I2 = _vel_est_ned(1);
	const float Vxc  = _vel_sp_ned(0);
	const float Vyc  = _vel_sp_ned(1);

	const float k1 = _Kv;
	const float k2 = _Ka;
	const float k3 = _Ki;

	const float mass = _mass;

	const float omega1 = _rates_body(0);
	const float omega2 = _rates_body(1);
	const float omega3 = _rates_body(2);

	const float q_0  = _q_att(0);
	const float q_v1 = _q_att(1);
	const float q_v2 = _q_att(2);
	const float q_v3 = _q_att(3);

	const float T = -_thrust_sp(2);

	auto sq   = [](float x) { return x * x; };
	auto cube = [](float x) { return x * x * x; };

	// t2..t28
	const float t2  = sq(K_d);
	const float t3  = cube(K_d);
	const float t4  = sq(omega1);
	const float t5  = sq(omega2);
	const float t6  = sq(q_0);
	const float t7  = cube(q_0);
	const float t9  = sq(q_v1);
	const float t10 = cube(q_v1);
	const float t11 = sq(q_v2);
	const float t13 = cube(q_v2);
	const float t14 = sq(q_v3);
	const float t16 = cube(q_v3);

	const float t8  = sq(t6);
	const float t12 = sq(t9);
	const float t15 = sq(t11);
	const float t17 = sq(t14);

	const float t24 = T * t6 * t14 * 2.0f;
	const float t25 = T * t9 * t11 * 2.0f;

	const float t18 = T * t8;
	const float t19 = T * t12;
	const float t20 = T * t15;
	const float t21 = T * t17;
	const float t26 = -t25;
	const float t22 = -t19;
	const float t23 = -t20;

	const float t27 = t18 + t21 + t22 + t23 + t24 + t26;
	const float t28 = 1.0f / t27;

	// et1..et9
	const float et1 =
		I_y * K_d * omega2 * t18 * t28 + I_y * K_d * omega2 * t21 * t28 + I_y * K_d * omega2 * t22 * t28
		+ I_y * K_d * omega2 * t23 * t28 + I_y * K_d * omega2 * t24 * t28
		- I_y * T * q_0 * t13 * t28 * 4.0f - I_y * T * q_v2 * t7 * t28 * 4.0f + I_y * T * q_v3 * t10 * t28 * 4.0f
		+ I_y * T * q_v1 * t16 * t28 * 4.0f
		- I_y * k1 * omega2 * t18 * t28 + I_y * k1 * omega2 * t19 * t28 - I_y * k2 * omega2 * t18 * t28
		+ I_y * k1 * omega2 * t20 * t28 + I_y * k2 * omega2 * t19 * t28 - I_y * k3 * omega2 * t18 * t28
		- I_y * k1 * omega2 * t21 * t28 + I_y * k2 * omega2 * t20 * t28 + I_y * k3 * omega2 * t19 * t28
		- I_y * k2 * omega2 * t21 * t28 + I_y * k3 * omega2 * t20 * t28 - I_y * k3 * omega2 * t21 * t28
		+ I_y * k1 * omega2 * t25 * t28 + I_y * k2 * omega2 * t25 * t28 + I_y * k3 * omega2 * t25 * t28
		+ I_x * omega1 * omega3 * t18 * t28 + I_x * omega1 * omega3 * t21 * t28 + I_x * omega1 * omega3 * t22 * t28
		+ I_x * omega1 * omega3 * t23 * t28 + I_x * omega1 * omega3 * t24 * t28
		- I_y * omega1 * omega3 * t18 * t28;

	const float et2 =
		I_y * omega1 * omega3 * t19 * t28 + I_y * omega1 * omega3 * t20 * t28 - I_y * omega1 * omega3 * t21 * t28
		+ I_y * omega1 * omega3 * t25 * t28 - I_z * omega1 * omega3 * t18 * t28 + I_z * omega1 * omega3 * t19 * t28
		+ I_z * omega1 * omega3 * t20 * t28 - I_z * omega1 * omega3 * t21 * t28 + I_z * omega1 * omega3 * t25 * t28
		- I_y * K_d * V_I1 * mass * t6 * t28 * 2.0f + I_y * K_d * V_I1 * mass * t9 * t28 * 2.0f
		- I_y * K_d * V_I1 * mass * t11 * t28 * 2.0f + I_y * K_d * V_I1 * mass * t14 * t28 * 2.0f
		+ I_y * V_I1 * k3 * mass * t6 * t28 - I_y * V_I1 * k3 * mass * t9 * t28 + I_y * V_I1 * k3 * mass * t11 * t28
		- I_y * V_I1 * k3 * mass * t14 * t28 - I_y * Vxc * k3 * mass * t6 * t28 + I_y * Vxc * k3 * mass * t9 * t28
		- I_y * Vxc * k3 * mass * t11 * t28 + I_y * Vxc * k3 * mass * t14 * t28
		- I_y * T * q_0 * q_v2 * t9 * t28 * 4.0f - I_y * T * q_0 * q_v2 * t14 * t28 * 4.0f
		+ I_y * T * q_v1 * q_v3 * t6 * t28 * 4.0f + I_y * T * q_v1 * q_v3 * t11 * t28 * 4.0f;

	const float et3 =
		- I_y * V_I1 * mass * t3 * t6 * t28 + I_y * V_I1 * mass * t3 * t9 * t28 - I_y * V_I1 * mass * t3 * t11 * t28
		+ I_y * V_I1 * mass * t3 * t14 * t28
		- I_y * T * q_0 * t2 * t13 * t28 * 2.0f + I_y * T * q_0 * t4 * t13 * t28 * 2.0f + I_y * T * q_0 * t5 * t13 * t28 * 2.0f
		- I_y * T * q_v2 * t2 * t7 * t28 * 2.0f + I_y * T * q_v2 * t4 * t7 * t28 * 2.0f + I_y * T * q_v2 * t5 * t7 * t28 * 2.0f
		+ I_y * T * q_v3 * t2 * t10 * t28 * 2.0f - I_y * T * q_v3 * t4 * t10 * t28 * 2.0f - I_y * T * q_v3 * t5 * t10 * t28 * 2.0f
		+ I_y * T * q_v1 * t2 * t16 * t28 * 2.0f - I_y * T * q_v1 * t4 * t16 * t28 * 2.0f - I_y * T * q_v1 * t5 * t16 * t28 * 2.0f
		+ I_y * K_d * T * k1 * q_0 * t13 * t28 * 2.0f + I_y * K_d * T * k2 * q_0 * t13 * t28 * 2.0f + I_y * K_d * T * k3 * q_0 * t13 * t28 * 2.0f
		+ I_y * K_d * T * k1 * q_v2 * t7 * t28 * 2.0f + I_y * K_d * T * k2 * q_v2 * t7 * t28 * 2.0f + I_y * K_d * T * k3 * q_v2 * t7 * t28 * 2.0f
		- I_y * K_d * T * k1 * q_v3 * t10 * t28 * 2.0f - I_y * K_d * T * k2 * q_v3 * t10 * t28 * 2.0f - I_y * K_d * T * k3 * q_v3 * t10 * t28 * 2.0f
		- I_y * K_d * T * k1 * q_v1 * t16 * t28 * 2.0f - I_y * K_d * T * k2 * q_v1 * t16 * t28 * 2.0f - I_y * K_d * T * k3 * q_v1 * t16 * t28 * 2.0f
		- I_y * K_d * V_I2 * mass * q_0 * q_v3 * t28 * 4.0f + I_y * K_d * V_I2 * mass * q_v1 * q_v2 * t28 * 4.0f
		- I_y * K_d * T * omega2 * t9 * t11 * t28 * 2.0f;

	const float et4 =
		- I_y * T * k1 * k2 * q_0 * t13 * t28 * 2.0f - I_y * T * k1 * k3 * q_0 * t13 * t28 * 2.0f - I_y * T * k2 * k3 * q_0 * t13 * t28 * 2.0f
		- I_y * T * k1 * k2 * q_v2 * t7 * t28 * 2.0f - I_y * T * k1 * k3 * q_v2 * t7 * t28 * 2.0f - I_y * T * k2 * k3 * q_v2 * t7 * t28 * 2.0f
		+ I_y * T * k1 * k2 * q_v3 * t10 * t28 * 2.0f + I_y * T * k1 * k3 * q_v3 * t10 * t28 * 2.0f + I_y * T * k2 * k3 * q_v3 * t10 * t28 * 2.0f
		+ I_y * T * k1 * k2 * q_v1 * t16 * t28 * 2.0f + I_y * T * k1 * k3 * q_v1 * t16 * t28 * 2.0f + I_y * T * k2 * k3 * q_v1 * t16 * t28 * 2.0f
		+ I_y * V_I2 * k3 * mass * q_0 * q_v3 * t28 * 2.0f - I_y * V_I2 * k3 * mass * q_v1 * q_v2 * t28 * 2.0f
		- I_y * Vyc * k3 * mass * q_0 * q_v3 * t28 * 2.0f + I_y * Vyc * k3 * mass * q_v1 * q_v2 * t28 * 2.0f
		- I_y * T * k1 * omega2 * t6 * t14 * t28 * 2.0f - I_y * T * k2 * omega2 * t6 * t14 * t28 * 2.0f - I_y * T * k3 * omega2 * t6 * t14 * t28 * 2.0f
		+ I_y * V_I1 * k1 * mass * t2 * t6 * t28 + I_y * V_I1 * k2 * mass * t2 * t6 * t28 + I_y * V_I1 * k3 * mass * t2 * t6 * t28
		- I_y * V_I1 * k1 * mass * t2 * t9 * t28 - I_y * V_I1 * k2 * mass * t2 * t9 * t28 + I_y * V_I1 * k1 * mass * t2 * t11 * t28
		- I_y * V_I1 * k3 * mass * t2 * t9 * t28 + I_y * V_I1 * k2 * mass * t2 * t11 * t28 + I_y * V_I1 * k3 * mass * t2 * t11 * t28
		- I_y * V_I1 * k1 * mass * t2 * t14 * t28 - I_y * V_I1 * k2 * mass * t2 * t14 * t28 - I_y * V_I1 * k3 * mass * t2 * t14 * t28
		- I_y * V_I2 * mass * q_0 * q_v3 * t3 * t28 * 2.0f + I_y * V_I2 * mass * q_v1 * q_v2 * t3 * t28 * 2.0f;

	const float et5 =
		I_x * T * omega1 * omega3 * t9 * t11 * t28 * -2.0f - I_y * T * omega1 * omega3 * t6 * t14 * t28 * 2.0f
		- I_z * T * omega1 * omega3 * t6 * t14 * t28 * 2.0f
		- I_y * T * q_0 * q_v2 * t2 * t9 * t28 * 2.0f + I_y * T * q_0 * q_v2 * t4 * t9 * t28 * 2.0f + I_y * T * q_0 * q_v2 * t5 * t9 * t28 * 2.0f
		- I_y * T * q_0 * q_v2 * t2 * t14 * t28 * 2.0f + I_y * T * q_0 * q_v2 * t4 * t14 * t28 * 2.0f + I_y * T * q_0 * q_v2 * t5 * t14 * t28 * 2.0f
		+ I_y * T * q_v1 * q_v3 * t2 * t6 * t28 * 2.0f - I_y * T * q_v1 * q_v3 * t4 * t6 * t28 * 2.0f - I_y * T * q_v1 * q_v3 * t5 * t6 * t28 * 2.0f
		+ I_y * T * q_v1 * q_v3 * t2 * t11 * t28 * 2.0f - I_y * T * q_v1 * q_v3 * t4 * t11 * t28 * 2.0f - I_y * T * q_v1 * q_v3 * t5 * t11 * t28 * 2.0f
		- I_y * K_d * V_I1 * k1 * k2 * mass * t6 * t28 - I_y * K_d * V_I1 * k1 * k3 * mass * t6 * t28 - I_y * K_d * V_I1 * k2 * k3 * mass * t6 * t28
		+ I_y * K_d * V_I1 * k1 * k2 * mass * t9 * t28 + I_y * K_d * V_I1 * k1 * k3 * mass * t9 * t28;

	const float et6 =
		- I_y * K_d * V_I1 * k1 * k2 * mass * t11 * t28 + I_y * K_d * V_I1 * k2 * k3 * mass * t9 * t28
		- I_y * K_d * V_I1 * k1 * k3 * mass * t11 * t28 - I_y * K_d * V_I1 * k2 * k3 * mass * t11 * t28
		+ I_y * K_d * V_I1 * k1 * k2 * mass * t14 * t28 + I_y * K_d * V_I1 * k1 * k3 * mass * t14 * t28 + I_y * K_d * V_I1 * k2 * k3 * mass * t14 * t28
		+ I_y * K_d * T * k1 * q_0 * q_v2 * t9 * t28 * 2.0f + I_y * K_d * T * k2 * q_0 * q_v2 * t9 * t28 * 2.0f + I_y * K_d * T * k3 * q_0 * q_v2 * t9 * t28 * 2.0f
		+ I_y * K_d * T * k1 * q_0 * q_v2 * t14 * t28 * 2.0f + I_y * K_d * T * k2 * q_0 * q_v2 * t14 * t28 * 2.0f + I_y * K_d * T * k3 * q_0 * q_v2 * t14 * t28 * 2.0f
		- I_y * K_d * T * k1 * q_v1 * q_v3 * t6 * t28 * 2.0f - I_y * K_d * T * k2 * q_v1 * q_v3 * t6 * t28 * 2.0f - I_y * K_d * T * k3 * q_v1 * q_v3 * t6 * t28 * 2.0f
		- I_y * K_d * T * k1 * q_v1 * q_v3 * t11 * t28 * 2.0f - I_y * K_d * T * k2 * q_v1 * q_v3 * t11 * t28 * 2.0f - I_y * K_d * T * k3 * q_v1 * q_v3 * t11 * t28 * 2.0f;

	const float et7 =
		I_y * V_I1 * k1 * k2 * k3 * mass * t6 * t28 - I_y * V_I1 * k1 * k2 * k3 * mass * t9 * t28 + I_y * V_I1 * k1 * k2 * k3 * mass * t11 * t28
		- I_y * V_I1 * k1 * k2 * k3 * mass * t14 * t28
		- I_y * Vxc * k1 * k2 * k3 * mass * t6 * t28 + I_y * Vxc * k1 * k2 * k3 * mass * t9 * t28 - I_y * Vxc * k1 * k2 * k3 * mass * t11 * t28
		+ I_y * Vxc * k1 * k2 * k3 * mass * t14 * t28
		- I_y * T * k1 * k2 * q_0 * q_v2 * t9 * t28 * 2.0f - I_y * T * k1 * k3 * q_0 * q_v2 * t9 * t28 * 2.0f - I_y * T * k2 * k3 * q_0 * q_v2 * t9 * t28 * 2.0f
		- I_y * T * k1 * k2 * q_0 * q_v2 * t14 * t28 * 2.0f - I_y * T * k1 * k3 * q_0 * q_v2 * t14 * t28 * 2.0f - I_y * T * k2 * k3 * q_0 * q_v2 * t14 * t28 * 2.0f
		+ I_y * T * k1 * k2 * q_v1 * q_v3 * t6 * t28 * 2.0f + I_y * T * k1 * k3 * q_v1 * q_v3 * t6 * t28 * 2.0f + I_y * T * k2 * k3 * q_v1 * q_v3 * t6 * t28 * 2.0f
		+ I_y * T * k1 * k2 * q_v1 * q_v3 * t11 * t28 * 2.0f + I_y * T * k1 * k3 * q_v1 * q_v3 * t11 * t28 * 2.0f + I_y * T * k2 * k3 * q_v1 * q_v3 * t11 * t28 * 2.0f;

	const float et8 =
		I_y * V_I2 * k1 * mass * q_0 * q_v3 * t2 * t28 * 2.0f + I_y * V_I2 * k2 * mass * q_0 * q_v3 * t2 * t28 * 2.0f + I_y * V_I2 * k3 * mass * q_0 * q_v3 * t2 * t28 * 2.0f
		- I_y * V_I2 * k1 * mass * q_v1 * q_v2 * t2 * t28 * 2.0f - I_y * V_I2 * k2 * mass * q_v1 * q_v2 * t2 * t28 * 2.0f - I_y * V_I2 * k3 * mass * q_v1 * q_v2 * t2 * t28 * 2.0f
		+ I_y * V_I2 * k1 * k2 * k3 * mass * q_0 * q_v3 * t28 * 2.0f - I_y * V_I2 * k1 * k2 * k3 * mass * q_v1 * q_v2 * t28 * 2.0f
		- I_y * Vyc * k1 * k2 * k3 * mass * q_0 * q_v3 * t28 * 2.0f + I_y * Vyc * k1 * k2 * k3 * mass * q_v1 * q_v2 * t28 * 2.0f
		- I_y * K_d * V_I2 * k1 * k2 * mass * q_0 * q_v3 * t28 * 2.0f - I_y * K_d * V_I2 * k1 * k3 * mass * q_0 * q_v3 * t28 * 2.0f
		- I_y * K_d * V_I2 * k2 * k3 * mass * q_0 * q_v3 * t28 * 2.0f + I_y * K_d * V_I2 * k1 * k2 * mass * q_v1 * q_v2 * t28 * 2.0f
		+ I_y * K_d * V_I2 * k1 * k3 * mass * q_v1 * q_v2 * t28 * 2.0f + I_y * K_d * V_I2 * k2 * k3 * mass * q_v1 * q_v2 * t28 * 2.0f;

	const float M = et1 + et2 + et3 + et4 + et5 + et6 + et7 + et8;

	// Original: _torque_sp(1) = -M;
	_torque_sp(1) = M;
}

/** ModuleBase interface **/

int CdusBackstepping::task_spawn(int argc, char *argv[])
{
	CdusBackstepping *instance = new CdusBackstepping();

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

int CdusBackstepping::custom_command(int argc, char *argv[])
{
	// No custom commands yet
	return print_usage("unknown command");
}

int CdusBackstepping::print_usage(const char *reason)
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

	PRINT_MODULE_USAGE_NAME("cdus_backstepping", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int cdus_backstepping_main(int argc, char *argv[])
{
	return CdusBackstepping::main(argc, argv);
}
