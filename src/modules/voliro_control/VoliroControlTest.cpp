#include "VoliroControl.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <lib/matrix/matrix/math.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

namespace
{

VoliroControl configuredController()
{
	VoliroControl controller;
	EXPECT_TRUE(controller.configure(VoliroControl::Configuration{}));
	return controller;
}

VoliroControl::State levelState()
{
	VoliroControl::State state{};
	state.position_ned.setZero();
	state.velocity_ned.setZero();
	state.attitude_ned_frd = Quatf{1.f, 0.f, 0.f, 0.f};
	state.angular_velocity_frd.setZero();
	return state;
}

VoliroControl::Setpoint levelSetpoint()
{
	VoliroControl::Setpoint setpoint{};
	setpoint.position_ned.setZero();
	setpoint.velocity_ned.setZero();
	setpoint.acceleration_ned.setZero();
	setpoint.attitude_ned_frd = Quatf{1.f, 0.f, 0.f, 0.f};
	setpoint.angular_velocity_frd.setZero();
	setpoint.angular_acceleration_frd.setZero();
	return setpoint;
}

} // namespace

TEST(VoliroControl, RejectsInvalidConfiguration)
{
	VoliroControl controller;
	auto configuration = VoliroControl::Configuration{};
	configuration.mass = -1.f;
	EXPECT_FALSE(controller.configure(configuration));
	EXPECT_FALSE(controller.configured());
}

TEST(VoliroControl, LevelHover)
{
	const VoliroControl controller = configuredController();
	const auto output = controller.calculate(levelState(), levelSetpoint());
	ASSERT_TRUE(output.valid);
	EXPECT_NEAR(output.force_frd(0), 0.f, 1e-6f);
	EXPECT_NEAR(output.force_frd(1), 0.f, 1e-6f);
	EXPECT_NEAR(output.force_frd(2), -41.9868f, 1e-4f);
	EXPECT_NEAR(output.thrust_normalized(2), -41.9868f / 147.f, 1e-6f);
	EXPECT_NEAR(output.moment_frd.norm(), 0.f, 1e-6f);
	EXPECT_FALSE(output.force_limited);
	EXPECT_EQ(output.torque_limited_mask, 0);
}

TEST(VoliroControl, LogarithmicErrorRemainsNonzeroAtInvertedAttitude)
{
	const Quatf current{Eulerf{M_PI_F, 0.f, 0.f}};
	const Quatf desired{1.f, 0.f, 0.f, 0.f};
	const Vector3f error = VoliroControl::attitudeError(current, desired);
	EXPECT_NEAR(fabsf(error(0)), M_PI_F, 1e-5f);
	EXPECT_NEAR(error(1), 0.f, 1e-5f);
	EXPECT_NEAR(error(2), 0.f, 1e-5f);
}

TEST(VoliroControl, MatchesPythonNedFrdReference)
{
	const VoliroControl controller = configuredController();
	VoliroControl::State state{};
	state.position_ned = Vector3f{0.2f, -0.1f, -1.1f};
	state.velocity_ned = Vector3f{0.3f, -0.2f, 0.1f};
	state.attitude_ned_frd = Quatf{Eulerf{
			math::radians(20.f), math::radians(-15.f), math::radians(30.f)}};
	state.angular_velocity_frd = Vector3f{0.2f, -0.1f, 0.4f};

	VoliroControl::Setpoint setpoint{};
	setpoint.position_ned = Vector3f{0.5f, 0.4f, -1.5f};
	setpoint.velocity_ned = Vector3f{0.1f, 0.f, -0.2f};
	setpoint.acceleration_ned = Vector3f{0.2f, -0.1f, 0.3f};
	setpoint.attitude_ned_frd = Quatf{Eulerf{
			math::radians(45.f), math::radians(-20.f), math::radians(10.f)}};
	setpoint.angular_velocity_frd = Vector3f{0.4f, -0.2f, 0.3f};
	setpoint.angular_acceleration_frd = Vector3f{0.1f, 0.05f, -0.2f};

	const auto output = controller.calculate(state, setpoint);
	ASSERT_TRUE(output.valid);
	const Vector3f expected_force{-10.31753502f, -14.34495901f, -40.31553485f};
	const Vector3f expected_moment{1.39122199f, -1.42981111f, -0.78156912f};
	const Vector3f expected_thrust{-0.07018731f, -0.09758476f, -0.27425534f};
	const Vector3f expected_torque{0.07359430f, -0.07563563f, -0.35445312f};

	for (int axis = 0; axis < 3; ++axis) {
		EXPECT_NEAR(output.force_frd(axis), expected_force(axis), 2e-5f);
		EXPECT_NEAR(output.moment_frd(axis), expected_moment(axis), 2e-5f);
		EXPECT_NEAR(output.thrust_normalized(axis), expected_thrust(axis), 2e-6f);
		EXPECT_NEAR(output.torque_normalized(axis), expected_torque(axis), 2e-6f);
	}
}

TEST(VoliroControl, PreservesForceDirectionWhenLimited)
{
	const VoliroControl controller = configuredController();
	auto setpoint = levelSetpoint();
	setpoint.position_ned = Vector3f{1000.f, -500.f, -1000.f};
	const auto output = controller.calculate(levelState(), setpoint);
	ASSERT_TRUE(output.valid);
	EXPECT_TRUE(output.force_limited);
	EXPECT_NEAR(output.thrust_normalized.norm(), 1.f, 1e-6f);
	const Vector3f expected_direction = output.force_frd.normalized();
	EXPECT_NEAR((output.thrust_normalized - expected_direction).norm(), 0.f, 1e-6f);
}
