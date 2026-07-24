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

VoliroAllocation::ThrustMatrix VoliroAllocation::thrustMatrixForTilt(const RotorVector &tilt) const
{
	ThrustMatrix matrix;
	for (int axis = 0; axis < NUM_WRENCH_AXES; ++axis) {
		for (int rotor = 0; rotor < NUM_ROTORS; ++rotor) {
			matrix(axis, rotor) = _effectiveness(axis, 2 * rotor) * cosf(tilt(rotor))
					      + _effectiveness(axis, 2 * rotor + 1) * sinf(tilt(rotor));
		}
	}
	return matrix;
}

VoliroAllocation::RotorVector VoliroAllocation::projectToThrustBox(const RotorVector &thrust) const
{
	RotorVector projected;
	for (int rotor = 0; rotor < NUM_ROTORS; ++rotor) {
		projected(rotor) = math::constrain(thrust(rotor), 0.f, _max_thrust);
	}
	return projected;
}

float VoliroAllocation::computeLipschitzConstant(const ThrustMatrix &matrix)
{
	const ThrustMatrix gram = matrix * matrix.transpose();
	RotorVector direction;
	direction.setAll(1.f / sqrtf(static_cast<float>(NUM_ROTORS)));

	for (int iteration = 0; iteration < 32; ++iteration) {
		const RotorVector next = gram * direction;
		const float norm = next.norm();
		if (norm <= FLT_EPSILON || !PX4_ISFINITE(norm)) { return 0.f; }
		direction = next / norm;
	}

	return direction.dot(gram * direction);
}

VoliroAllocation::RotorVector VoliroAllocation::predictTilt(const RotorVector &measured_angle,
		const RotorVector &measured_velocity, const RotorVector &command, float horizon,
		float tau, float rate_max)
{
	if (!measured_angle.isAllFinite() || !measured_velocity.isAllFinite() || !command.isAllFinite()
	    || !PX4_ISFINITE(horizon) || horizon < 0.f || !PX4_ISFINITE(tau) || tau <= 0.f
	    || !PX4_ISFINITE(rate_max) || rate_max <= 0.f) {
		return measured_angle;
	}

	const float natural_frequency = 1.f / tau;
	const float decay = expf(-natural_frequency * horizon);
	const float max_step = rate_max * horizon;
	RotorVector predicted;
	for (int rotor = 0; rotor < NUM_ROTORS; ++rotor) {
		const float error = measured_angle(rotor) - command(rotor);
		const float response = command(rotor)
				       + (error + (measured_velocity(rotor) + natural_frequency * error) * horizon) * decay;
		predicted(rotor) = measured_angle(rotor) + math::constrain(
			response - measured_angle(rotor), -max_step, max_step);
	}
	return predicted;
}

VoliroAllocation::RotorVector VoliroAllocation::slewTiltCommand(const RotorVector &target,
		const RotorVector &previous_command, float tilt_min, float tilt_max,
		float rate_max, float dt, uint8_t &rate_limited_mask)
{
	RotorVector command = previous_command;
	rate_limited_mask = 0;
	if (!target.isAllFinite() || !previous_command.isAllFinite()
	    || !PX4_ISFINITE(tilt_min) || !PX4_ISFINITE(tilt_max) || tilt_min >= tilt_max
	    || !PX4_ISFINITE(rate_max) || rate_max <= 0.f
	    || !PX4_ISFINITE(dt) || dt <= 0.f) {
		return command;
	}

	const float max_step = rate_max * dt;
	for (int rotor = 0; rotor < NUM_ROTORS; ++rotor) {
		const float reference = math::constrain(previous_command(rotor), tilt_min, tilt_max);
		// Thrust direction is 2*pi-periodic, but the joint travel is finite.
		// Choose an equivalent target inside the configured range, then move
		// linearly from the persistent command. Do not wrap the remaining
		// delta, because a modulo-short path can point through a hard stop.
		float feasible_target = target(rotor);
		while (feasible_target < tilt_min) {
			feasible_target += 2.f * M_PI_F;
		}
		while (feasible_target > tilt_max) {
			feasible_target -= 2.f * M_PI_F;
		}
		const float delta = feasible_target - reference;
		const float limited_delta = math::constrain(delta, -max_step, max_step);
		if (fabsf(limited_delta - delta) > 1e-6f) {
			rate_limited_mask |= 1u << rotor;
		}
		command(rotor) = math::constrain(reference + limited_delta, tilt_min, tilt_max);
	}
	return command;
}

VoliroAllocation::Result VoliroAllocation::allocateThrustForTilt(const WrenchVector &wrench,
		const RotorVector &tilt, const RotorVector &initial_thrust) const
{
	Result result{};
	result.tilt = tilt;
	if (!_configured || !wrench.isAllFinite() || !tilt.isAllFinite()
	    || !initial_thrust.isAllFinite()) {
		return result;
	}

	const ThrustMatrix matrix = thrustMatrixForTilt(tilt);
	ThrustMatrix pseudo_inverse;
	RotorVector unconstrained;
	unconstrained.setZero();
	const bool pseudo_inverse_valid = geninv(matrix, pseudo_inverse)
					  && pseudo_inverse.isAllFinite();
	if (pseudo_inverse_valid) {
		unconstrained = pseudo_inverse * wrench;
	}

	const float exact_tolerance = 1e-5f * (1.f + wrench.norm());
	const WrenchVector exact_residual = matrix * unconstrained - wrench;
	const bool exact_feasible = pseudo_inverse_valid
				    && unconstrained.min() >= -1e-5f
				    && unconstrained.max() <= _max_thrust + 1e-5f
				    && exact_residual.norm() <= exact_tolerance;

	if (exact_feasible) {
		result.thrust = projectToThrustBox(unconstrained);
		result.success = true;
	} else {
		result.optimization_used = true;
		const float weight_data[NUM_WRENCH_AXES] {1.f, 1.f, 2.f, 8.f, 8.f, 8.f};
		ThrustMatrix weighted_matrix;
		WrenchVector weighted_wrench;
		for (int axis = 0; axis < NUM_WRENCH_AXES; ++axis) {
			weighted_wrench(axis) = weight_data[axis] * wrench(axis);
			for (int rotor = 0; rotor < NUM_ROTORS; ++rotor) {
				weighted_matrix(axis, rotor) = weight_data[axis] * matrix(axis, rotor);
			}
		}

		const float lipschitz = computeLipschitzConstant(weighted_matrix) + _regularization;
		if (!PX4_ISFINITE(lipschitz) || lipschitz <= FLT_EPSILON) {
			return result;
		}

		RotorVector x = projectToThrustBox(initial_thrust);
		if (pseudo_inverse_valid) {
			// The instantaneous-stage thrust is a useful continuity warm start,
			// but during a large tilt transition it can be far from the
			// fixed-direction optimum. Also consider the projected fixed-tilt
			// least-squares solution and start from whichever has the lower
			// weighted objective.
			const RotorVector candidate = projectToThrustBox(unconstrained);
			const WrenchVector initial_residual =
				weighted_matrix * x - weighted_wrench;
			const WrenchVector candidate_residual =
				weighted_matrix * candidate - weighted_wrench;
			const float initial_cost = initial_residual.dot(initial_residual)
						   + _regularization * x.dot(x);
			const float candidate_cost = candidate_residual.dot(candidate_residual)
						     + _regularization * candidate.dot(candidate);
			if (candidate_cost < initial_cost) {
				x = candidate;
			}
		}
		RotorVector y = x;
		float momentum = 1.f;
		for (int iteration = 1; iteration <= _max_iterations; ++iteration) {
			const RotorVector gradient = weighted_matrix.transpose()
						 * (weighted_matrix * y - weighted_wrench)
						 + _regularization * y;
			const RotorVector x_next = projectToThrustBox(y - gradient / lipschitz);
			result.iterations = iteration;
			const RotorVector step = x_next - x;
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
		result.thrust = x;
		// Reaching the deterministic iteration budget is not a hard allocation
		// failure. Every projected iterate is finite and inside the actuator
		// box, so the last iterate is safer and more useful than discarding it
		// for a centered-tilt conventional fallback. Invalid configuration,
		// inputs, or numerical constants still return success=false above.
		result.success = result.thrust.isAllFinite();
	}

	for (int rotor = 0; rotor < NUM_ROTORS; ++rotor) {
		if (result.thrust(rotor) >= _max_thrust - 1e-5f) {
			result.saturation_mask |= 1u << rotor;
		}
	}
	result.achieved_wrench = matrix * result.thrust;
	result.residual_wrench = result.achieved_wrench - wrench;
	return result;
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
	const float force_scale = 6.f * _max_thrust;
	const float roll_pitch_scale = _max_thrust * _arm_radius * sqrtf(6.f);
	const float yaw_scale = 6.f * _max_thrust * _kappa;
	// PX4 publishes normalized body force in FRD. The allocation model uses
	// physical body force in FLU, so X is unchanged while Y and Z change sign.
	wrench(0) = thrust_sp_frd(0) * force_scale;
	wrench(1) = -thrust_sp_frd(1) * force_scale;
	wrench(2) = -thrust_sp_frd(2) * force_scale;
	wrench(3) = torque_sp_frd(0) * roll_pitch_scale;
	wrench(4) = -torque_sp_frd(1) * roll_pitch_scale;
	wrench(5) = -torque_sp_frd(2) * yaw_scale;
	return wrench;
}
