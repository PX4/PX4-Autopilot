#include <gtest/gtest.h>
#include <AttitudeControl.hpp>

using namespace matrix;

TEST(AttitudeControlTest, AllZeroCase)
{
	AttitudeControl attitude_control;
	matrix::Vector3f rate_setpoint = attitude_control.update(Quatf(), Quatf(), 0.f);
	EXPECT_EQ(rate_setpoint, Vector3f());
}
