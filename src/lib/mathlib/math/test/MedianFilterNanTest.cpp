/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * make tests TESTFILTER=MedianFilterNan
 */

#include <gtest/gtest.h>
#include <cmath>
#include <limits>

#include <lib/mathlib/math/filter/MedianFilter.hpp>

using math::MedianFilter;

TEST(MedianFilterNan, finiteSamplesMedian)
{
	MedianFilter<float, 3> mf;
	// cold start is zero-filled; load a full clear window first
	(void)mf.apply(1.f);
	(void)mf.apply(5.f);
	const float m = mf.apply(3.f); // window {1,5,3} -> median 3
	EXPECT_FLOAT_EQ(m, 3.f);
}

TEST(MedianFilterNan, nanPushedAsideVersusFinite)
{
	MedianFilter<float, 3> mf;
	const float nan = std::numeric_limits<float>::quiet_NaN();
	mf.insert(1.f);
	mf.insert(2.f);
	mf.insert(nan);
	const float m = mf.median();
	// non-finite sorts after finite in cmp → median tends finite when majority finite
	EXPECT_TRUE(std::isfinite(m));
}
