#include <gtest/gtest.h>
#include <matrix/matrix/math.hpp>
#include "Functions.hpp"

#include "TrajMath.hpp"

using namespace math;
using matrix::Vector2f;

TEST(MaxSpeedFromDistance, MaxSpeedFromDistance)
{
	// When max acceleration is higher we should be able to fly faster
	const float acc_hor_small = 1.f;
	const float acc_hor_large = 10.f;

	const float jerk_max = 8.f;
	const float stop_distance = 10.f;
	const float final_speed = 0.f;

	const float max_vel_smaller = trajectory::computeMaxSpeedFromDistance(jerk_max, acc_hor_small, stop_distance, final_speed);
	const float max_vel_larger = trajectory::computeMaxSpeedFromDistance(jerk_max, acc_hor_large, stop_distance, final_speed);

	printf("max_vel_larger: %f\n", (double)max_vel_larger);
	printf("max_vel_smaller: %f\n", (double)max_vel_smaller);


	EXPECT_TRUE(max_vel_larger < max_vel_smaller);
}
