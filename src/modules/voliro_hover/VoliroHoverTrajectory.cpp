#include "VoliroHoverTrajectory.hpp"

#include <cmath>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

bool VoliroHoverTrajectory::configure(float height, float launch_acceleration, float touchdown_acceleration,
		float takeoff_duration, float land_duration)
{
	if (!PX4_ISFINITE(height) || !PX4_ISFINITE(launch_acceleration)
	    || !PX4_ISFINITE(touchdown_acceleration)
	    || !PX4_ISFINITE(takeoff_duration) || !PX4_ISFINITE(land_duration)
	    || height <= 0.f || launch_acceleration <= 0.f || touchdown_acceleration <= 0.f
	    || takeoff_duration <= 0.f || land_duration <= 0.f) {
		return false;
	}

	_height = height;
	_launch_acceleration = launch_acceleration;
	_touchdown_acceleration = touchdown_acceleration;
	_takeoff_duration = takeoff_duration;
	_land_duration = land_duration;
	return true;
}

void VoliroHoverTrajectory::reset(const Vector3f &ground_position_ned, float yaw_ned)
{
	_ground_position_ned = ground_position_ned;
	_yaw_ned = yaw_ned;
	_takeoff_start_z = ground_position_ned(2);
	_land_start_z = ground_position_ned(2);
	_phase_start = 0;
	_phase = Phase::Ready;
	_initialized = ground_position_ned.isAllFinite() && PX4_ISFINITE(yaw_ned);

	if (!_initialized) {
		_phase = Phase::Inactive;
	}
}

bool VoliroHoverTrajectory::beginLaunch()
{
	if (!_initialized || (_phase != Phase::Ready && _phase != Phase::Landed)) {
		return false;
	}

	_phase = Phase::Launch;
	return true;
}

bool VoliroHoverTrajectory::startTakeoff(uint64_t now_us, float current_z_ned)
{
	if (!_initialized || _phase != Phase::Launch || !PX4_ISFINITE(current_z_ned)) {
		return false;
	}

	_takeoff_start_z = current_z_ned;
	_phase_start = now_us;
	_phase = Phase::Takeoff;
	return true;
}

bool VoliroHoverTrajectory::startLand(uint64_t now_us, float current_z_ned)
{
	if (!_initialized || !PX4_ISFINITE(current_z_ned)
	    || (_phase != Phase::Takeoff && _phase != Phase::Hold)) {
		return false;
	}

	_land_start_z = current_z_ned;
	_phase_start = now_us;
	_phase = Phase::Land;
	return true;
}

VoliroHoverTrajectory::Profile VoliroHoverTrajectory::minimumJerk(float elapsed, float duration)
{
	const float u = math::constrain(elapsed / duration, 0.f, 1.f);
	const float u2 = u * u;
	const float u3 = u2 * u;
	const float u4 = u3 * u;
	const float u5 = u4 * u;

	Profile profile{};
	profile.position = 10.f * u3 - 15.f * u4 + 6.f * u5;
	profile.velocity = (30.f * u2 - 60.f * u3 + 30.f * u4) / duration;
	profile.acceleration = (60.f * u - 180.f * u2 + 120.f * u3) / (duration * duration);
	return profile;
}

VoliroHoverTrajectory::Setpoint VoliroHoverTrajectory::update(uint64_t now_us)
{
	Setpoint setpoint{};
	setpoint.position_ned = _ground_position_ned;
	setpoint.velocity_ned.setZero();
	setpoint.acceleration_ned.setZero();
	setpoint.attitude_ned_frd = Quatf{Eulerf{0.f, 0.f, _yaw_ned}};
	setpoint.angular_velocity_frd.setZero();

	if (!_initialized) {
		setpoint.position_ned = Vector3f{NAN, NAN, NAN};
		return setpoint;
	}

	if (_phase == Phase::Launch) {
		setpoint.acceleration_ned(2) = -_launch_acceleration;

	} else if (_phase == Phase::Takeoff) {
		const float elapsed = (now_us - _phase_start) * 1e-6f;
		const Profile profile = minimumJerk(elapsed, _takeoff_duration);
		const float target_z = _ground_position_ned(2) - _height;
		const float displacement = target_z - _takeoff_start_z;
		setpoint.position_ned(2) = _takeoff_start_z + displacement * profile.position;
		setpoint.velocity_ned(2) = displacement * profile.velocity;
		setpoint.acceleration_ned(2) = displacement * profile.acceleration;

		if (elapsed >= _takeoff_duration) {
			_phase = Phase::Hold;
		}

	} else if (_phase == Phase::Hold) {
		setpoint.position_ned(2) = _ground_position_ned(2) - _height;

	} else if (_phase == Phase::Land) {
		const float elapsed = (now_us - _phase_start) * 1e-6f;
		const Profile profile = minimumJerk(elapsed, _land_duration);
		const float displacement = _ground_position_ned(2) - _land_start_z;
		setpoint.position_ned(2) = _land_start_z + displacement * profile.position;
		setpoint.velocity_ned(2) = displacement * profile.velocity;
		setpoint.acceleration_ned(2) = displacement * profile.acceleration;

		if (elapsed >= _land_duration) {
			_phase = Phase::Landed;
		}

	} else if (_phase == Phase::Landed) {
		setpoint.position_ned(2) = _ground_position_ned(2);
		setpoint.acceleration_ned(2) = _touchdown_acceleration;

	} else if (_phase == Phase::Ready) {
		setpoint.position_ned(2) = _ground_position_ned(2);
	}

	return setpoint;
}
