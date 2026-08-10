#include "GL40IIBench.hpp"

#include <cmath>
#include <gtest/gtest.h>

using namespace gl40ii;

TEST(GL40IIBench, MinimumJerkEndpointsAndMidpoint)
{
	EXPECT_FLOAT_EQ(minimumJerk(-1.f), 0.f);
	EXPECT_FLOAT_EQ(minimumJerk(0.f), 0.f);
	EXPECT_FLOAT_EQ(minimumJerk(0.5f), 0.5f);
	EXPECT_FLOAT_EQ(minimumJerk(1.f), 1.f);
	EXPECT_FLOAT_EQ(minimumJerk(2.f), 1.f);
	EXPECT_LT(minimumJerk(0.25f), minimumJerk(0.5f));
	EXPECT_LT(minimumJerk(0.5f), minimumJerk(0.75f));
}

TEST(GL40IIBench, DurationRespectsPeakRateAndMinimum)
{
	EXPECT_FLOAT_EQ(minimumJerkDuration(0.f, 1.f, 1.f), 1.f);
	EXPECT_FLOAT_EQ(minimumJerkDuration(0.1f, 1.f, 1.f), 1.f);
	EXPECT_NEAR(minimumJerkDuration(2.f, 1.f, 1.f), 3.75f, 1e-6f);
	EXPECT_TRUE(std::isnan(minimumJerkDuration(1.f, 0.f, 1.f)));
}

TEST(GL40IIBench, RelativeMoveMustFitWithMargin)
{
	EXPECT_TRUE(benchRangeValid(0.f, 1.f, 12.5f, 0.5f));
	EXPECT_TRUE(benchRangeValid(-0.5f, 12.f, 12.5f, 0.5f));
	EXPECT_FALSE(benchRangeValid(0.f, 12.1f, 12.5f, 0.5f));
	EXPECT_FALSE(benchRangeValid(12.1f, -1.f, 12.5f, 0.5f));
	EXPECT_FALSE(benchRangeValid(0.f, 1.f, 0.f, 0.5f));
}

TEST(GL40IIBench, BidirectionalMoveChecksBothEndpoints)
{
	EXPECT_TRUE(benchBidirectionalRangeValid(0.f, 6.283185f, 12.5f, 0.5f));
	EXPECT_TRUE(benchBidirectionalRangeValid(5.7f, 6.283185f, 12.5f, 0.5f));
	EXPECT_FALSE(benchBidirectionalRangeValid(5.8f, 6.283185f, 12.5f, 0.5f));
	EXPECT_FALSE(benchBidirectionalRangeValid(-5.8f, 6.283185f, 12.5f, 0.5f));
	EXPECT_FALSE(benchBidirectionalRangeValid(0.f, -1.f, 12.5f, 0.5f));
}

TEST(GL40IIBench, GuardMaskPollsEveryUnselectedDrive)
{
	EXPECT_EQ(benchGuardMask(0x3f, 0, true), 0x3e);
	EXPECT_EQ(benchGuardMask(0x3f, 4, true), 0x2f);
	EXPECT_EQ(benchGuardMask(0x3f, 5, true), 0x1f);
	EXPECT_EQ(benchGuardMask(0x01, 0, true), 0x00);
	EXPECT_EQ(benchGuardMask(0x3f, 8, true), 0x3f);
}

TEST(GL40IIBench, GuardMaskPollsEveryDriveDuringInterMotorDwell)
{
	EXPECT_EQ(benchGuardMask(0x3f, 0, false), 0x3f);
	EXPECT_EQ(benchGuardMask(0x3f, 1, false), 0x3f);
	EXPECT_EQ(benchGuardMask(0x3f, 5, false), 0x3f);
}
