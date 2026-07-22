/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 ****************************************************************************/
#include "VoliroAllocation.hpp"

#include <float.h>
#include <cmath>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

bool VoliroAllocation::configure(float arm_radius, float max_thrust, float kappa, float regularization,
				 float tolerance, int max_iterations)
{
	_configured = false;
	if (!PX4_ISFINITE(arm_radius) || arm_radius <= FLT_EPSILON
	    || !PX4_ISFINITE(max_thrust) || max_thrust <= FLT_EPSILON
	    || !PX4_ISFINITE(kappa) || kappa < 0.f
	    || !PX4_ISFINITE(regularization) || regularization < 0.f
	    || !PX4_ISFINITE(tolerance) || tolerance <= 0.f || max_iterations < 1) {
		return false;
	}

	_arm_radius = arm_radius;
	_max_thrust = max_thrust;
	_kappa = kappa;
	_regularization = regularization;
	_tolerance = tolerance;
	_max_iterations = max_iterations;
	_effectiveness.setZero();

	for (int rotor = 0; rotor < NUM_ROTORS; ++rotor) {
		const float theta = rotor * M_PI_F / 3.f;
		const float spin = (rotor % 2 == 0) ? 1.f : -1.f;
		const Vector3f position{_arm_radius * cosf(theta), _arm_radius * sinf(theta), 0.f};
		const Vector3f vertical{0.f, 0.f, 1.f};
		const Vector3f lateral{sinf(theta), -cosf(theta), 0.f};
		const Vector3f vertical_moment = position.cross(vertical) + spin * _kappa * vertical;
		const Vector3f lateral_moment = position.cross(lateral) + spin * _kappa * lateral;

		for (int axis = 0; axis < 3; ++axis) {
			_effectiveness(axis, 2 * rotor) = vertical(axis);
			_effectiveness(axis, 2 * rotor + 1) = lateral(axis);
			_effectiveness(axis + 3, 2 * rotor) = vertical_moment(axis);
			_effectiveness(axis + 3, 2 * rotor + 1) = lateral_moment(axis);
		}
	}

	if (!geninv(_effectiveness, _pseudo_inverse) || !_pseudo_inverse.isAllFinite()) {
		return false;
	}

	_lipschitz = computeLipschitzConstant() + _regularization;
	_configured = PX4_ISFINITE(_lipschitz) && _lipschitz > FLT_EPSILON;
	return _configured;
}

float VoliroAllocation::computeLipschitzConstant() const
{
	const SquareMatrix<float, NUM_WRENCH_AXES> gram = _effectiveness * _effectiveness.transpose();
	WrenchVector direction;
	direction.setAll(1.f / sqrtf(static_cast<float>(NUM_WRENCH_AXES)));

	for (int iteration = 0; iteration < 32; ++iteration) {
		const WrenchVector next = gram * direction;
		const float norm = next.norm();
		if (norm <= FLT_EPSILON || !PX4_ISFINITE(norm)) { return 0.f; }
		direction = next / norm;
	}

	return direction.dot(gram * direction);
}

VoliroAllocation::ComponentVector VoliroAllocation::projectToThrustDisks(const ComponentVector &components) const
{
	ComponentVector projected = components;
	for (int rotor = 0; rotor < NUM_ROTORS; ++rotor) {
		const int vertical_index = 2 * rotor;
		const int lateral_index = vertical_index + 1;
		const float magnitude = hypotf(projected(vertical_index), projected(lateral_index));
		if (magnitude > _max_thrust) {
			const float scale = _max_thrust / magnitude;
			projected(vertical_index) *= scale;
			projected(lateral_index) *= scale;
		}
	}
	return projected;
}

bool VoliroAllocation::componentsFeasible(const ComponentVector &components) const
{
	for (int rotor = 0; rotor < NUM_ROTORS; ++rotor) {
		if (hypotf(components(2 * rotor), components(2 * rotor + 1)) > _max_thrust + 1e-5f) { return false; }
	}
	return true;
}

float VoliroAllocation::wrapPi(float angle)
{
	return atan2f(sinf(angle), cosf(angle));
}

VoliroAllocation::Result VoliroAllocation::allocate(const WrenchVector &wrench, const RotorVector &current_tilt) const
{
	Result result{};
	result.tilt = current_tilt;
	if (!_configured || !wrench.isAllFinite() || !current_tilt.isAllFinite()) { return result; }

	ComponentVector components = _pseudo_inverse * wrench;
	if (componentsFeasible(components)) {
		result.success = true;
	} else {
		result.optimization_used = true;
		ComponentVector x = projectToThrustDisks(components);
		ComponentVector y = x;
		float momentum = 1.f;
		for (int iteration = 1; iteration <= _max_iterations; ++iteration) {
			const ComponentVector gradient = _effectiveness.transpose() * (_effectiveness * y - wrench)
						 + _regularization * y;
			const ComponentVector x_next = projectToThrustDisks(y - gradient / _lipschitz);
			result.iterations = iteration;
			const ComponentVector step = x_next - x;
			if (step.norm() <= _tolerance * (1.f + x.norm())) {
				x = x_next;
				result.success = true;
				break;
			}
			const float momentum_next = 0.5f * (1.f + sqrtf(1.f + 4.f * momentum * momentum));
			y = x_next + ((momentum - 1.f) / momentum_next) * (x_next - x);
			x = x_next;
			momentum = momentum_next;
		}
		components = x;
	}

	for (int rotor = 0; rotor < NUM_ROTORS; ++rotor) {
		const float vertical = components(2 * rotor);
		const float lateral = components(2 * rotor + 1);
		const float thrust = hypotf(vertical, lateral);
		result.thrust(rotor) = thrust;
		if (thrust > 1e-7f) {
			const float principal = atan2f(lateral, vertical);
			result.tilt(rotor) = current_tilt(rotor) + wrapPi(principal - current_tilt(rotor));
		}
		if (thrust >= _max_thrust - 1e-5f) { result.saturation_mask |= 1u << rotor; }
	}

	result.achieved_wrench = wrenchFromCommands(result.thrust, result.tilt);
	result.residual_wrench = result.achieved_wrench - wrench;
	return result;
}

VoliroAllocation::WrenchVector VoliroAllocation::wrenchFromCommands(const RotorVector &thrust,
		const RotorVector &tilt) const
{
	ComponentVector components;
	for (int rotor = 0; rotor < NUM_ROTORS; ++rotor) {
		components(2 * rotor) = thrust(rotor) * cosf(tilt(rotor));
		components(2 * rotor + 1) = thrust(rotor) * sinf(tilt(rotor));
	}
	return _effectiveness * components;
}

VoliroAllocation::WrenchVector VoliroAllocation::setpointToWrench(const Vector3f &thrust_sp_frd,
		const Vector3f &torque_sp_frd) const
{
	WrenchVector wrench;
	wrench.setZero();
	const float roll_pitch_scale = _max_thrust * _arm_radius * sqrtf(6.f);
	const float yaw_scale = 6.f * _max_thrust * _kappa;
	wrench(2) = -thrust_sp_frd(2) * 6.f * _max_thrust;
	wrench(3) = torque_sp_frd(0) * roll_pitch_scale;
	wrench(4) = -torque_sp_frd(1) * roll_pitch_scale;
	wrench(5) = -torque_sp_frd(2) * yaw_scale;
	return wrench;
}
