#include "VoliroControl.hpp"

#include <cmath>
#include <float.h>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

bool VoliroControl::configure(const Configuration &configuration)
{
	const bool positive_scalars = PX4_ISFINITE(configuration.mass) && configuration.mass > FLT_EPSILON
				      && PX4_ISFINITE(configuration.gravity) && configuration.gravity > FLT_EPSILON
				      && PX4_ISFINITE(configuration.max_rotor_thrust)
				      && configuration.max_rotor_thrust > FLT_EPSILON
				      && PX4_ISFINITE(configuration.arm_radius) && configuration.arm_radius > FLT_EPSILON
				      && PX4_ISFINITE(configuration.kappa) && configuration.kappa > FLT_EPSILON;
	const bool finite_vectors = configuration.inertia.isAllFinite()
				    && configuration.position_gain.isAllFinite()
				    && configuration.velocity_gain.isAllFinite()
				    && configuration.attitude_gain.isAllFinite()
				    && configuration.angular_rate_gain.isAllFinite();
	const bool positive_inertia = (configuration.inertia.min() > FLT_EPSILON);
	const bool nonnegative_gains = (configuration.position_gain.min() >= 0.f)
				       && (configuration.velocity_gain.min() >= 0.f)
				       && (configuration.attitude_gain.min() >= 0.f)
				       && (configuration.angular_rate_gain.min() >= 0.f);

	_configured = positive_scalars && finite_vectors && positive_inertia && nonnegative_gains;

	if (_configured) {
		_configuration = configuration;
	}

	return _configured;
}

Vector3f VoliroControl::attitudeError(const Quatf &attitude_ned_frd, const Quatf &desired_ned_frd)
{
	// q_error represents R_desired^T R_current. Canonicalizing selects the
	// shortest rotation branch. Unlike the common skew/trace error, this
	// logarithmic error remains nonzero at a 180-degree attitude error.
	const Quatf q_error = (desired_ned_frd.inversed() * attitude_ned_frd).canonical();
	const Vector3f imaginary{q_error(1), q_error(2), q_error(3)};
	const float imaginary_norm = imaginary.norm();

	if (imaginary_norm < 1e-6f) {
		return 2.f * imaginary;
	}

	const float angle = 2.f * atan2f(imaginary_norm, math::max(q_error(0), 0.f));
	return imaginary * (angle / imaginary_norm);
}

VoliroControl::Output VoliroControl::calculate(const State &state, const Setpoint &setpoint) const
{
	Output output{};

	if (!_configured || !state.position_ned.isAllFinite() || !state.velocity_ned.isAllFinite()
	    || !state.attitude_ned_frd.isAllFinite() || !state.angular_velocity_frd.isAllFinite()
	    || !setpoint.position_ned.isAllFinite() || !setpoint.velocity_ned.isAllFinite()
	    || !setpoint.acceleration_ned.isAllFinite() || !setpoint.attitude_ned_frd.isAllFinite()
	    || !setpoint.angular_velocity_frd.isAllFinite()
	    || !setpoint.angular_acceleration_frd.isAllFinite()) {
		return output;
	}

	Quatf attitude = state.attitude_ned_frd;
	Quatf attitude_desired = setpoint.attitude_ned_frd;

	if (attitude.norm() < 1e-6f || attitude_desired.norm() < 1e-6f) {
		return output;
	}

	attitude.normalize();
	attitude_desired.normalize();

	const Vector3f position_error = setpoint.position_ned - state.position_ned;
	const Vector3f velocity_error = setpoint.velocity_ned - state.velocity_ned;
	const Vector3f gravity_ned{0.f, 0.f, -_configuration.gravity};
	const Vector3f force_ned =
		_configuration.mass * (gravity_ned + setpoint.acceleration_ned)
		+ _configuration.position_gain.emult(position_error)
		+ _configuration.velocity_gain.emult(velocity_error);
	output.force_frd = attitude.rotateVectorInverse(force_ned);

	output.attitude_error = attitudeError(attitude, attitude_desired);
	const Matrix3f desired_to_current = Dcmf(attitude).transpose() * Dcmf(attitude_desired);
	const Vector3f desired_rate_in_current = desired_to_current * setpoint.angular_velocity_frd;
	const Vector3f desired_acceleration_in_current =
		desired_to_current * setpoint.angular_acceleration_frd;
	output.angular_rate_error = state.angular_velocity_frd - desired_rate_in_current;

	const Vector3f angular_momentum = _configuration.inertia.emult(state.angular_velocity_frd);
	const Vector3f feedforward = -_configuration.inertia.emult(
					     state.angular_velocity_frd.cross(desired_rate_in_current)
					     - desired_acceleration_in_current);
	output.moment_frd =
		-_configuration.attitude_gain.emult(output.attitude_error)
		- _configuration.angular_rate_gain.emult(output.angular_rate_error)
		+ state.angular_velocity_frd.cross(angular_momentum)
		+ feedforward;

	const float force_scale = 6.f * _configuration.max_rotor_thrust;
	output.thrust_normalized = output.force_frd / force_scale;
	const float normalized_force = output.thrust_normalized.norm();

	if (normalized_force > 1.f) {
		output.thrust_normalized /= normalized_force;
		output.force_limited = true;
	}

	const float roll_pitch_scale =
		_configuration.max_rotor_thrust * _configuration.arm_radius * sqrtf(6.f);
	const float yaw_scale = 6.f * _configuration.max_rotor_thrust * _configuration.kappa;
	const Vector3f moment_scale{roll_pitch_scale, roll_pitch_scale, yaw_scale};

	for (int axis = 0; axis < 3; ++axis) {
		const float unbounded = output.moment_frd(axis) / moment_scale(axis);

		if (fabsf(unbounded) > 1.f) {
			output.torque_limited_mask |= static_cast<uint8_t>(1u << axis);
		}

		output.torque_normalized(axis) = math::constrain(unbounded, -1.f, 1.f);
	}

	output.valid = output.force_frd.isAllFinite() && output.moment_frd.isAllFinite()
		       && output.thrust_normalized.isAllFinite() && output.torque_normalized.isAllFinite();
	return output;
}
