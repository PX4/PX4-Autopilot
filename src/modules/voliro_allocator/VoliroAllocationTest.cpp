#include <gtest/gtest.h>
#include <cmath>
#include "VoliroAllocation.hpp"
using namespace matrix;

class VoliroAllocationTest : public ::testing::Test
{
public:
	void SetUp() override { ASSERT_TRUE(_allocator.configure(0.315f, 13.7f, 0.015f, 1e-10f, 1e-6f, 100)); }
	VoliroAllocation _allocator;
};

static void expectGolden(VoliroAllocation &allocator, const float wrench_data[6],
		const float expected_thrust[6], const float expected_tilt[6], float tolerance = 2e-4f)
{
	VoliroAllocation::WrenchVector wrench(wrench_data);
	VoliroAllocation::RotorVector current_tilt; current_tilt.setZero();
	const auto result = allocator.allocate(wrench, current_tilt);
	ASSERT_TRUE(result.success);
	ASSERT_FALSE(result.optimization_used);
	for (int rotor = 0; rotor < VoliroAllocation::NUM_ROTORS; ++rotor) {
		EXPECT_NEAR(result.thrust(rotor), expected_thrust[rotor], tolerance);
		EXPECT_NEAR(result.tilt(rotor), expected_tilt[rotor], tolerance);
	}
	EXPECT_LT(result.residual_wrench.norm(), 2e-4f);
}

TEST_F(VoliroAllocationTest, PythonGoldenHoverSymmetry)
{
	const float wrench[6] {0.f, 0.f, 42.f, 0.f, 0.f, 0.f};
	const float thrust[6] {7.f, 7.f, 7.f, 7.f, 7.f, 7.f};
	const float tilt[6] {};
	expectGolden(_allocator, wrench, thrust, tilt);
}

TEST_F(VoliroAllocationTest, PythonGoldenPureBodyTorques)
{
	const float roll_wrench[6] {0.f, 0.f, 42.f, 0.5f, 0.f, 0.f};
	const float roll_thrust[6] {7.f, 7.457209593f, 7.457209593f, 7.f, 6.542858404f, 6.542858404f};
	const float roll_tilt[6] {0.f, -0.002919377f, 0.002919377f, 0.f, -0.003327355f, 0.003327355f};
	expectGolden(_allocator, roll_wrench, roll_thrust, roll_tilt);

	const float pitch_wrench[6] {0.f, 0.f, 42.f, 0.f, 0.5f, 0.f};
	const float pitch_thrust[6] {6.472145350f, 6.736059992f, 7.263962609f, 7.527945442f, 7.263962609f, 6.736059992f};
	const float pitch_tilt[6] {-0.003884079f, 0.001865948f, 0.001730342f, -0.003339332f, 0.001730342f, 0.001865948f};
	expectGolden(_allocator, pitch_wrench, pitch_thrust, pitch_tilt);

	const float yaw_wrench[6] {0.f, 0.f, 42.f, 0.f, 0.f, 0.2f};
	const float yaw_thrust[6] {7.005823270f, 6.995769109f, 7.005823270f, 6.995769109f, 7.005823270f, 6.995769109f};
	const float yaw_tilt[6] {-0.015070990f, -0.015092651f, -0.015070990f, -0.015092651f, -0.015070990f, -0.015092651f};
	expectGolden(_allocator, yaw_wrench, yaw_thrust, yaw_tilt);
}

TEST_F(VoliroAllocationTest, PythonGoldenHorizontalForce)
{
	const float wrench[6] {5.f, -3.f, 42.f, 0.f, 0.f, 0.f};
	const float thrust[6] {7.071067812f, 7.264758014f, 7.063282357f, 7.071067812f, 7.264758014f, 7.063282357f};
	const float tilt[6] {0.141897055f, 0.270805100f, 0.133960828f, -0.141897055f, -0.270805100f, -0.133960828f};
	expectGolden(_allocator, wrench, thrust, tilt);
}

TEST_F(VoliroAllocationTest, PythonGoldenFeasibleWrench)
{
	const float wrench_data[6] {2.f, -1.f, 42.f, 0.2f, -0.1f, 0.05f};
	VoliroAllocation::WrenchVector wrench(wrench_data);
	VoliroAllocation::RotorVector current_tilt; current_tilt.setZero();
	const auto result = _allocator.allocate(wrench, current_tilt);
	const float expected_thrust[6] {7.11368143f, 7.26881077f, 7.14202031f, 6.90228227f, 6.81059715f, 6.88216414f};
	const float expected_tilt[6] {0.04386841f, 0.09733593f, 0.05470127f, -0.05141157f, -0.11502078f, -0.06264984f};
	ASSERT_TRUE(result.success); EXPECT_FALSE(result.optimization_used);
	for (int i = 0; i < 6; ++i) {
		EXPECT_NEAR(result.thrust(i), expected_thrust[i], 2e-4f);
		EXPECT_NEAR(result.tilt(i), expected_tilt[i], 2e-4f);
	}
	EXPECT_LT(result.residual_wrench.norm(), 2e-4f);
}

TEST_F(VoliroAllocationTest, PythonGoldenSaturatedWrench)
{
	const float wrench_data[6] {200.f, 50.f, 100.f, 20.f, -10.f, 5.f};
	VoliroAllocation::WrenchVector wrench(wrench_data);
	VoliroAllocation::RotorVector current_tilt; current_tilt.setZero();
	const auto result = _allocator.allocate(wrench, current_tilt);
	const float expected_tilt[6] {-0.55235634f, 1.07664121f, 1.19620667f, 0.56853419f, -1.18273588f, -1.24605172f};
	ASSERT_TRUE(result.success); EXPECT_TRUE(result.optimization_used); EXPECT_EQ(result.saturation_mask, 0x3f);
	for (int i = 0; i < 6; ++i) {
		EXPECT_NEAR(result.thrust(i), 13.7f, 2e-4f);
		EXPECT_NEAR(result.tilt(i), expected_tilt[i], 2e-3f);
	}
}

TEST_F(VoliroAllocationTest, ZeroThrustHoldsMeasuredAngle)
{
	VoliroAllocation::WrenchVector wrench; wrench.setZero();
	const float current_data[6] {-1.f, -0.6f, -0.2f, 0.2f, 0.6f, 1.f};
	VoliroAllocation::RotorVector current_tilt(current_data);
	const auto result = _allocator.allocate(wrench, current_tilt);
	ASSERT_TRUE(result.success); EXPECT_LT(result.thrust.norm(), 1e-6f);
	const VoliroAllocation::RotorVector tilt_error = result.tilt - current_tilt;
	EXPECT_LT(tilt_error.norm(), 1e-6f);
}

TEST_F(VoliroAllocationTest, InvertedWrenchUnwrapsAcrossBranchCut)
{
	const float wrench_data[6] {0.f, 0.f, -42.f, 0.f, 0.f, 0.f};
	const float current_data[6] {3.10f, 3.11f, 3.12f, 3.13f, 3.14f, 3.15f};
	VoliroAllocation::WrenchVector wrench(wrench_data);
	VoliroAllocation::RotorVector current_tilt(current_data);
	const auto result = _allocator.allocate(wrench, current_tilt);
	ASSERT_TRUE(result.success);
	for (int rotor = 0; rotor < VoliroAllocation::NUM_ROTORS; ++rotor) {
		EXPECT_NEAR(result.thrust(rotor), 7.f, 2e-4f);
		EXPECT_NEAR(result.tilt(rotor), M_PI_F, 2e-4f);
		EXPECT_LT(fabsf(result.tilt(rotor) - current_tilt(rotor)), 0.05f);
	}
}

TEST_F(VoliroAllocationTest, NearbyWrenchesProduceContinuousCommands)
{
	const float first_data[6] {2.f, -1.f, 42.f, 0.2f, -0.1f, 0.05f};
	const float second_data[6] {2.001f, -1.001f, 42.001f, 0.2001f, -0.1001f, 0.0501f};
	VoliroAllocation::WrenchVector first_wrench(first_data);
	VoliroAllocation::WrenchVector second_wrench(second_data);
	VoliroAllocation::RotorVector current_tilt; current_tilt.setZero();
	const auto first = _allocator.allocate(first_wrench, current_tilt);
	const auto second = _allocator.allocate(second_wrench, first.tilt);
	ASSERT_TRUE(first.success);
	ASSERT_TRUE(second.success);
	const VoliroAllocation::RotorVector thrust_delta = second.thrust - first.thrust;
	const VoliroAllocation::RotorVector tilt_delta = second.tilt - first.tilt;
	EXPECT_LT(thrust_delta.norm(), 0.01f);
	EXPECT_LT(tilt_delta.norm(), 0.01f);
}

TEST_F(VoliroAllocationTest, RejectsInvalidConfigurationAndSetpoint)
{
	VoliroAllocation invalid;
	EXPECT_FALSE(invalid.configure(0.f, 13.7f, 0.015f, 1e-10f, 1e-6f, 100));
	EXPECT_FALSE(invalid.configure(0.315f, -1.f, 0.015f, 1e-10f, 1e-6f, 100));
	EXPECT_FALSE(invalid.configure(0.315f, 13.7f, 0.015f, 1e-10f, 1e-6f, 0));

	VoliroAllocation::WrenchVector wrench; wrench.setZero();
	wrench(0) = NAN;
	VoliroAllocation::RotorVector current_tilt; current_tilt.setZero();
	const auto result = _allocator.allocate(wrench, current_tilt);
	EXPECT_FALSE(result.success);
	EXPECT_FALSE(result.optimization_used);
}

TEST_F(VoliroAllocationTest, Px4SetpointUsesPhysicalFluContract)
{
	const auto wrench = _allocator.setpointToWrench(Vector3f{0.f, 0.f, -0.5f}, Vector3f{0.1f, -0.2f, 0.3f});
	EXPECT_FLOAT_EQ(wrench(0), 0.f); EXPECT_FLOAT_EQ(wrench(1), 0.f);
	EXPECT_NEAR(wrench(2), 41.1f, 1e-5f); EXPECT_NEAR(wrench(3), 1.0570773f, 1e-5f);
	EXPECT_NEAR(wrench(4), 2.1141546f, 1e-5f); EXPECT_NEAR(wrench(5), -0.3699f, 1e-5f);
}
