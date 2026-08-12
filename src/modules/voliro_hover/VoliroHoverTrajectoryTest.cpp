#include "VoliroHoverTrajectory.hpp"

#include <gtest/gtest.h>

using namespace matrix;

TEST(VoliroHoverTrajectory, RejectsInvalidConfiguration)
{
	VoliroHoverTrajectory trajectory;
	EXPECT_FALSE(trajectory.configure(0.f, 0.8f, 9.81f, 5.f, 5.f));
	EXPECT_FALSE(trajectory.configure(1.f, 0.f, 9.81f, 5.f, 5.f));
	EXPECT_FALSE(trajectory.configure(1.f, 0.8f, 0.f, 5.f, 5.f));
	EXPECT_FALSE(trajectory.configure(1.f, 0.8f, 9.81f, -1.f, 5.f));
	EXPECT_FALSE(trajectory.configure(1.f, 0.8f, 9.81f, 5.f, NAN));
}

TEST(VoliroHoverTrajectory, BoundedLaunchPrecedesTakeoff)
{
	VoliroHoverTrajectory trajectory;
	ASSERT_TRUE(trajectory.configure(1.f, 0.8f, 9.81f, 4.f, 5.f));
	trajectory.reset(Vector3f{2.f, -3.f, 0.25f}, 0.7f);

	ASSERT_TRUE(trajectory.beginLaunch());
	auto launch = trajectory.update(500'000);
	EXPECT_EQ(trajectory.phase(), VoliroHoverTrajectory::Phase::Launch);
	EXPECT_FLOAT_EQ(launch.position_ned(2), 0.25f);
	EXPECT_FLOAT_EQ(launch.velocity_ned(2), 0.f);
	EXPECT_FLOAT_EQ(launch.acceleration_ned(2), -0.8f);

	ASSERT_TRUE(trajectory.startTakeoff(1'000'000, 0.20f));
	auto start = trajectory.update(1'000'000);
	EXPECT_FLOAT_EQ(start.position_ned(2), 0.20f);
	EXPECT_FLOAT_EQ(start.velocity_ned(2), 0.f);
	EXPECT_FLOAT_EQ(start.acceleration_ned(2), 0.f);
}

TEST(VoliroHoverTrajectory, MinimumJerkTakeoffAndLand)
{
	VoliroHoverTrajectory trajectory;
	ASSERT_TRUE(trajectory.configure(1.f, 0.8f, 9.81f, 4.f, 5.f));
	trajectory.reset(Vector3f{2.f, -3.f, 0.25f}, 0.7f);

	ASSERT_TRUE(trajectory.beginLaunch());
	ASSERT_TRUE(trajectory.startTakeoff(1'000'000, 0.25f));
	auto start = trajectory.update(1'000'000);
	EXPECT_FLOAT_EQ(start.position_ned(2), 0.25f);
	EXPECT_FLOAT_EQ(start.velocity_ned(2), 0.f);
	EXPECT_FLOAT_EQ(start.acceleration_ned(2), 0.f);

	auto middle = trajectory.update(3'000'000);
	EXPECT_NEAR(middle.position_ned(2), -0.25f, 1e-6f);
	EXPECT_LT(middle.velocity_ned(2), 0.f);
	EXPECT_NEAR(middle.acceleration_ned(2), 0.f, 1e-6f);

	auto top = trajectory.update(5'000'000);
	EXPECT_NEAR(top.position_ned(2), -0.75f, 1e-6f);
	EXPECT_FLOAT_EQ(top.velocity_ned(2), 0.f);
	EXPECT_EQ(trajectory.phase(), VoliroHoverTrajectory::Phase::Hold);

	ASSERT_TRUE(trajectory.startLand(6'000'000, -0.70f));
	auto landing_middle = trajectory.update(8'500'000);
	EXPECT_NEAR(landing_middle.position_ned(2), -0.225f, 1e-6f);
	EXPECT_GT(landing_middle.velocity_ned(2), 0.f);

	auto ground = trajectory.update(11'000'000);
	EXPECT_NEAR(ground.position_ned(2), 0.25f, 1e-6f);
	EXPECT_FLOAT_EQ(ground.velocity_ned(2), 0.f);
	EXPECT_EQ(trajectory.phase(), VoliroHoverTrajectory::Phase::Landed);
	auto unload = trajectory.update(11'020'000);
	EXPECT_FLOAT_EQ(unload.acceleration_ned(2), 9.81f);
}

TEST(VoliroHoverTrajectory, CommandsArePhaseGated)
{
	VoliroHoverTrajectory trajectory;
	ASSERT_TRUE(trajectory.configure(0.5f, 0.8f, 9.81f, 5.f, 5.f));
	EXPECT_FALSE(trajectory.beginLaunch());
	EXPECT_FALSE(trajectory.startTakeoff(1, 0.f));
	trajectory.reset(Vector3f{0.f, 0.f, 0.f}, 0.f);
	EXPECT_FALSE(trajectory.startLand(1, 0.f));
	ASSERT_TRUE(trajectory.beginLaunch());
	EXPECT_FALSE(trajectory.beginLaunch());
	EXPECT_FALSE(trajectory.startTakeoff(1, NAN));
	ASSERT_TRUE(trajectory.startTakeoff(1, 0.f));
	EXPECT_FALSE(trajectory.startTakeoff(2, 0.f));
	EXPECT_FALSE(trajectory.startLand(2, NAN));
}
