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
	float computeLipschitzConstant() const;
	matrix::Matrix<float, NUM_WRENCH_AXES, NUM_COMPONENTS> _effectiveness;
	matrix::Matrix<float, NUM_COMPONENTS, NUM_WRENCH_AXES> _pseudo_inverse;
	float _arm_radius{0.315f};
	float _max_thrust{13.7f};
	float _kappa{0.015f};
	float _regularization{1e-10f};
	float _tolerance{1e-6f};
	float _lipschitz{6.f};
	int _max_iterations{100};
	bool _configured{false};
};
