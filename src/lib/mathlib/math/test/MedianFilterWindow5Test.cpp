/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include <gtest/gtest.h>
#include <mathlib/math/filter/MedianFilter.hpp>

using math::MedianFilter;

TEST(MedianFilterWindow5, SteadySequence)
{
	MedianFilter<int, 5> mf;
	// fill
	EXPECT_EQ(mf.apply(1), 1); // buffer lacks full semantics but median of what is inserted
	mf.apply(2);
	mf.apply(3);
	mf.apply(4);
	EXPECT_EQ(mf.apply(5), 3); // 1..5 median 3
	EXPECT_EQ(mf.apply(6), 4); // 2..6 median 4
}

TEST(MedianFilterWindow5, SpikeRejected)
{
	MedianFilter<float, 5> mf;
	mf.apply(1.f);
	mf.apply(1.f);
	mf.apply(1.f);
	mf.apply(1.f);
	EXPECT_FLOAT_EQ(mf.apply(1.f), 1.f);
	// spike
	EXPECT_FLOAT_EQ(mf.apply(100.f), 1.f);
}
