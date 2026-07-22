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

	// Buffer starts zero-filled; fill the full window before reading medians.
	for (int i = 1; i <= 4; i++) {
		mf.apply(i);
	}

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
