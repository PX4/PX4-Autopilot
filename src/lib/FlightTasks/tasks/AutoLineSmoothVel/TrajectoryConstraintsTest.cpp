#include <gtest/gtest.h>
#include <FlightTasks/tasks/AutoLineSmoothVel/TrajectoryConstraints.hpp>

using namespace matrix;
using namespace math::trajectory;

class TrajectoryConstraintsTest : public ::testing::Test
{
public:
    vehicle_dynamic_limits config;

    Vector3f vehicle_location;
    Vector3f target;
    Vector3f next_target;

    float final_speed = 0;

	void SetUp() override
	{
        config.z_accept_rad = 1;
        config.xy_accept_rad = 1;

        config.max_acc_xy = 3;
        config.max_jerk = 10;

        config.max_speed_xy = 6;

        config.max_acc_xy_radius_scale = 0.8;

        /*
         *             (20,20)
         *              Next target
         *
         *              ^
         *              |
         *
         * (10,10)      (20,10)
         * Vehicle  ->  Target
         *
         */
        vehicle_location = Vector3f(10, 10, 5);
        target = Vector3f(20, 10, 5);
        next_target = Vector3f(20, 20, 5);
	}
};

TEST_F(TrajectoryConstraintsTest, testStraight) {

}





TEST(TrajectoryConstraintsClamp, clampToXYNormNoEffectLarge)
{
    // GIVEN: a short vector
    Vector3f vec(1,2,3);

    // WHEN: we clamp it on XY with a long cutoff
    clampToXYNorm(vec, 1000.f);

    // THEN: it shouldn't change
    EXPECT_EQ(vec, Vector3f(1,2,3));
}

TEST(TrajectoryConstraintsClamp, clampToZNormNoEffect)
{
    // GIVEN: a short vector
    Vector3f vec(1,2,3);

    // WHEN: we clamp it on XY with a long cutoff
    clampToZNorm(vec, 1000.f);

    // THEN: it shouldn't change
    EXPECT_EQ(vec, Vector3f(1,2,3));
}

TEST(TrajectoryConstraintsClamp, clampToXYNormNoEffectExact)
{
    // GIVEN: a vector
    Vector3f vec(3,4,1);

    // WHEN: we clamp it on XY with exact cutoff
    clampToXYNorm(vec, 5.f);

    // THEN: it shouldn't change
    EXPECT_EQ(vec, Vector3f(3,4,1));
}

TEST(TrajectoryConstraintsClamp, clampToZNormNoEffectExact)
{
    // GIVEN: a vector
    Vector3f vec(3,4,-1);

    // WHEN: we clamp it on Z with exact cutoff
    clampToZNorm(vec, 1.f);

    // THEN: it shouldn't change
    EXPECT_EQ(vec, Vector3f(3,4,-1));
}

TEST(TrajectoryConstraintsClamp, clampToXYNormHalf)
{
    // GIVEN: a vector
    Vector3f vec(3,4,1);

    // WHEN: we clamp it on XY with half hypot length
    clampToXYNorm(vec, 2.5f);

    // THEN: it should be half length
    EXPECT_TRUE(vec == Vector3f(1.5f,2.f,0.5f));
}

TEST(TrajectoryConstraintsClamp, clampToZNormHalf)
{
    // GIVEN: a vector
    Vector3f vec(3,4,10);

    // WHEN: we clamp it on Z with half length
    clampToZNorm(vec, 5.f);

    // THEN: it should be half length
    EXPECT_TRUE(vec == Vector3f(1.5f,2.f,5.f));
}

TEST(TrajectoryConstraintsClamp, clampToXYNormZero)
{
    // GIVEN: a vector
    Vector3f vec(3,4,1);

    // WHEN: we clamp it on XY with half hypot length
    clampToXYNorm(vec, 0.f);

    // THEN: it should be 0
    EXPECT_TRUE(vec == Vector3f(0.f,0.f,0.f));
}

TEST(TrajectoryConstraintsClamp, clampToZNormZero)
{
    // GIVEN: a vector
    Vector3f vec(3,4,1);

    // WHEN: we clamp it on Z with half hypot length
    clampToZNorm(vec, 0.f);

    // THEN: it should be 0
    EXPECT_TRUE(vec == Vector3f(0.f,0.f,0.f));
}

TEST(TrajectoryConstraintsClamp, clampToXYNormVecZero)
{
    // GIVEN: a vector
    Vector3f vec(0,0,0);

    // WHEN: we clamp it on XY
    clampToXYNorm(vec, 1.f);

    // THEN: it should be 0 still
    EXPECT_TRUE(vec == Vector3f(0.f,0.f,0.f));
}

TEST(TrajectoryConstraintsClamp, clampToZNormVecZero)
{
    // GIVEN: a vector
    Vector3f vec(0,0,0);

    // WHEN: we clamp it on Z
    clampToZNorm(vec, 1.f);

    // THEN: it should be 0 still
    EXPECT_TRUE(vec == Vector3f(0.f,0.f,0.f));
}

TEST(TrajectoryConstraintsClamp, clampToXYNormVecZeroToZero)
{
    // GIVEN: a vector
    Vector3f vec(0,0,0);

    // WHEN: we clamp it on XY
    clampToXYNorm(vec, 0.f);

    // THEN: it should be 0 still
    EXPECT_TRUE(vec == Vector3f(0.f,0.f,0.f));
}

TEST(TrajectoryConstraintsClamp, clampToZNormVecZeroToZero)
{
    // GIVEN: a vector
    Vector3f vec(0,0,0);

    // WHEN: we clamp it on XY
    clampToZNorm(vec, 0.f);

    // THEN: it should be 0 still
    EXPECT_TRUE(vec == Vector3f(0.f,0.f,0.f));
}
