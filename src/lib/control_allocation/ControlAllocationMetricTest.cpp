/****************************************************************************
 *
 *   Copyright (C) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ControlAllocationTest.cpp
 *
 * Tests for Control Allocation metric class
 *
 * @author Pedro Roque <padr@kth.se>
 */

#include <gtest/gtest.h>
#include <ControlAllocationMetric.hpp>
#include <../ActuatorEffectiveness/ActuatorEffectivenessThrusters.hpp>
#include <iostream>

using namespace matrix;

namespace
{
	// Creates a 2D spacecraft with 8 thrusters (one in each corner of the axis, NED Frame):
	//           1 <--> 0
	//               x
	//		 |
	//      ^6 v7    +----y  ^4 v5
	//
	//
	//           3 <--> 2
	ActuatorEffectivenessThrusters::Geometry make_spacecraft_x_geometry()
	{
		ActuatorEffectivenessThrusters::Geometry geometry = {};
		geometry.thrusters[0].position(0) = 1.0f;
		geometry.thrusters[0].position(1) = 0.0f;
		geometry.thrusters[0].position(2) = 0.0f;
		geometry.thrusters[0].axis(0) = 0.0f;
		geometry.thrusters[0].axis(1) = 1.0f;
		geometry.thrusters[0].axis(2) = 0.0f;
		geometry.thrusters[0].thrust_coef = 2.0f;

		geometry.thrusters[1].position(0) = 1.0f;
		geometry.thrusters[1].position(1) = 0.0f;
		geometry.thrusters[1].position(2) = 0.0f;
		geometry.thrusters[1].axis(0) = 0.0f;
		geometry.thrusters[1].axis(1) = -1.0f;
		geometry.thrusters[1].axis(2) = 0.0f;
		geometry.thrusters[1].thrust_coef = 2.0f;

		geometry.thrusters[2].position(0) = -1.0f;
		geometry.thrusters[2].position(1) = 0.0f;
		geometry.thrusters[2].position(2) = 0.0f;
		geometry.thrusters[2].axis(0) = 0.0f;
		geometry.thrusters[2].axis(1) = 1.0f;
		geometry.thrusters[2].axis(2) = 0.0f;
		geometry.thrusters[2].thrust_coef = 2.0f;

		geometry.thrusters[3].position(0) = -1.0f;
		geometry.thrusters[3].position(1) = 0.0f;
		geometry.thrusters[3].position(2) = 0.0f;
		geometry.thrusters[3].axis(0) = 0.0f;
		geometry.thrusters[3].axis(1) = -1.0f;
		geometry.thrusters[3].axis(2) = 0.0f;
		geometry.thrusters[3].thrust_coef = 2.0f;

		geometry.thrusters[4].position(0) = 0.0f;
		geometry.thrusters[4].position(1) = 1.0f;
		geometry.thrusters[4].position(2) = 0.0f;
		geometry.thrusters[4].axis(0) = 1.0f;
		geometry.thrusters[4].axis(1) = 0.0f;
		geometry.thrusters[4].axis(2) = 0.0f;
		geometry.thrusters[4].thrust_coef = 2.0f;

		geometry.thrusters[5].position(0) = 0.0f;
		geometry.thrusters[5].position(1) = 1.0f;
		geometry.thrusters[5].position(2) = 0.0f;
		geometry.thrusters[5].axis(0) = -1.0f;
		geometry.thrusters[5].axis(1) = 0.0f;
		geometry.thrusters[5].axis(2) = 0.0f;
		geometry.thrusters[5].thrust_coef = 2.0f;

		geometry.thrusters[6].position(0) = 0.0f;
		geometry.thrusters[6].position(1) = -1.0f;
		geometry.thrusters[6].position(2) = 0.0f;
		geometry.thrusters[6].axis(0) = 1.0f;
		geometry.thrusters[6].axis(1) = 0.0f;
		geometry.thrusters[6].axis(2) = 0.0f;
		geometry.thrusters[6].thrust_coef = 2.0f;

		geometry.thrusters[7].position(0) = 0.0f;
		geometry.thrusters[7].position(1) = -1.0f;
		geometry.thrusters[7].position(2) = 0.0f;
		geometry.thrusters[7].axis(0) = -1.0f;
		geometry.thrusters[7].axis(1) = 0.0f;
		geometry.thrusters[7].axis(2) = 0.0f;
		geometry.thrusters[7].thrust_coef = 2.0f;

		geometry.num_thrusters = 8;

		return geometry;
	}

	ActuatorEffectiveness::EffectivenessMatrix make_spacecraft_x_effectiveness()
	{
		ActuatorEffectiveness::EffectivenessMatrix effectiveness;
		effectiveness.setZero();
		const auto geometry = make_spacecraft_x_geometry();
		ActuatorEffectivenessThrusters::computeEffectivenessMatrix(geometry, effectiveness);
		PX4_INFO("Effectiveness matrix (transposed): ");
		effectiveness.T().print();
		return effectiveness;
	}

	void setup_spacecraft_allocator(ControlAllocationMetric &allocator)
	{
		const auto effectiveness = make_spacecraft_x_effectiveness();
		matrix::Vector<float, ActuatorEffectiveness::NUM_ACTUATORS> actuator_trim;
		matrix::Vector<float, ActuatorEffectiveness::NUM_ACTUATORS> linearization_point;
		constexpr bool UPDATE_NORMALIZATION_SCALE{false};
		allocator.setEffectivenessMatrix(
			effectiveness,
			actuator_trim,
			linearization_point,
			ActuatorEffectiveness::NUM_ACTUATORS,
			UPDATE_NORMALIZATION_SCALE
		);
	}
} // namespace


TEST(ControlAllocationMetricTest, AllZeroCase)
{
	ControlAllocationMetric method;

	matrix::Vector<float, 6> control_sp;
	matrix::Vector<float, 6> control_allocated;
	matrix::Vector<float, 6> control_allocated_expected;
	matrix::Matrix<float, 6, 16> effectiveness;
	matrix::Vector<float, 16> actuator_sp;
	matrix::Vector<float, 16> actuator_trim;
	matrix::Vector<float, 16> linearization_point;
	matrix::Vector<float, 16> actuator_sp_expected;

	method.setEffectivenessMatrix(effectiveness, actuator_trim, linearization_point, 16, false);
	method.setControlSetpoint(control_sp);
	method.allocate();
	method.clipActuatorSetpoint();
	actuator_sp = method.getActuatorSetpoint();
	control_allocated_expected = method.getAllocatedControl();

	EXPECT_EQ(actuator_sp, actuator_sp_expected);
	EXPECT_EQ(control_allocated, control_allocated_expected);
}


TEST(ControlAllocationMetricTest, MetricOutputProportional)
{
	ControlAllocationMetric allocator;
	setup_spacecraft_allocator(allocator);
	matrix::Vector<float, ActuatorEffectiveness::NUM_AXES> control_sp;

	control_sp(ControlAllocation::ControlAxis::ROLL) = 0.f;
	control_sp(ControlAllocation::ControlAxis::PITCH) = 0.f;
	control_sp(ControlAllocation::ControlAxis::YAW) = 0.f;
	control_sp(ControlAllocation::ControlAxis::THRUST_X) = 0.5f; // newton
	control_sp(ControlAllocation::ControlAxis::THRUST_Y) = 0.f;
	control_sp(ControlAllocation::ControlAxis::THRUST_Z) = 0.f;
	allocator.setControlSetpoint(control_sp);

	allocator.allocate();

	const auto &actuator_sp = allocator.getActuatorSetpoint();

	// print each actuator setpoint
	actuator_sp.print();

	// ensure that outputs match expected
	EXPECT_NEAR(actuator_sp(0), 0.0f, 1e-4);
	EXPECT_NEAR(actuator_sp(1), 0.0f, 1e-4);
	EXPECT_NEAR(actuator_sp(2), 0.0f, 1e-4);
	EXPECT_NEAR(actuator_sp(3), 0.0f, 1e-4);
	EXPECT_NEAR(actuator_sp(4), 0.06250f, 1e-4);
	EXPECT_NEAR(actuator_sp(5), -0.06250f, 1e-4);
	EXPECT_NEAR(actuator_sp(6), 0.06250f, 1e-4);
	EXPECT_NEAR(actuator_sp(7), -0.06250f, 1e-4);

}
