/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team. All rights reserved.
 ****************************************************************************/
#pragma once

#include <cstdint>
#include <matrix/matrix/math.hpp>

class VoliroAllocation
{
public:
	static constexpr int NUM_ROTORS = 6;
	static constexpr int NUM_COMPONENTS = 2 * NUM_ROTORS;
	static constexpr int NUM_WRENCH_AXES = 6;
	using RotorVector = matrix::Vector<float, NUM_ROTORS>;
	using ComponentVector = matrix::Vector<float, NUM_COMPONENTS>;
	using WrenchVector = matrix::Vector<float, NUM_WRENCH_AXES>;
	using ThrustMatrix = matrix::SquareMatrix<float, NUM_ROTORS>;

	struct Result {
		RotorVector thrust;
		RotorVector tilt;
		WrenchVector achieved_wrench;
		WrenchVector residual_wrench;
		uint16_t iterations{0};
		uint8_t saturation_mask{0};
		bool optimization_used{false};
		bool success{false};
	};

	bool configure(float arm_radius, float max_thrust, float kappa, float regularization,
		       float tolerance, int max_iterations);
	Result allocate(const WrenchVector &wrench, const RotorVector &current_tilt) const;
	Result allocateThrustForTilt(const WrenchVector &wrench, const RotorVector &tilt,
				     const RotorVector &initial_thrust) const;
	static RotorVector predictTilt(const RotorVector &measured_angle,
				       const RotorVector &measured_velocity,
				       const RotorVector &command, float horizon,
				       float tau, float rate_max);
	static RotorVector slewTiltCommand(const RotorVector &target,
					   const RotorVector &previous_command,
					   float tilt_min, float tilt_max,
					   float rate_max, float dt,
					   uint8_t &rate_limited_mask);
	WrenchVector wrenchFromCommands(const RotorVector &thrust, const RotorVector &tilt) const;
	WrenchVector setpointToWrench(const matrix::Vector3f &thrust_sp_frd,
				     const matrix::Vector3f &torque_sp_frd) const;
	float maxThrust() const { return _max_thrust; }
	float armRadius() const { return _arm_radius; }
	float kappa() const { return _kappa; }
	bool configured() const { return _configured; }

private:
	static float wrapPi(float angle);
	ComponentVector projectToThrustDisks(const ComponentVector &components) const;
	bool componentsFeasible(const ComponentVector &components) const;
	ThrustMatrix thrustMatrixForTilt(const RotorVector &tilt) const;
	RotorVector projectToThrustBox(const RotorVector &thrust) const;
	static float computeLipschitzConstant(const ThrustMatrix &matrix);
	float computeLipschitzConstant() const;
	matrix::Matrix<float, NUM_WRENCH_AXES, NUM_COMPONENTS> _effectiveness;
	matrix::Matrix<float, NUM_COMPONENTS, NUM_WRENCH_AXES> _pseudo_inverse;
	float _arm_radius{0.315f};
	float _max_thrust{24.5f};
	float _kappa{0.015f};
	float _regularization{1e-10f};
	float _tolerance{1e-6f};
	float _lipschitz{6.f};
	int _max_iterations{100};
	bool _configured{false};
};
