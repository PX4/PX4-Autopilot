/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * make tests TESTFILTER=FilteredDerivative
 */

#include <gtest/gtest.h>
#include <cmath>

#include <lib/mathlib/math/filter/FilteredDerivative.hpp>

TEST(FilteredDerivative, constantSignalYieldsNearZeroDerivative)
{
	FilteredDerivative<float> fd;
	fd.setParameters(0.01f, 0.05f);
	float out = 0.f;

	for (int i = 0; i < 50; i++) {
		out = fd.update(5.f);
	}

	EXPECT_NEAR(out, 0.f, 0.2f);
}

TEST(FilteredDerivative, linearRampApproachesSlope)
{
	FilteredDerivative<float> fd;
	const float dt = 0.01f;
	const float slope = 2.5f;
	fd.setParameters(dt, 0.02f);
	float out = 0.f;
	float s = 0.f;

	for (int i = 0; i < 200; i++) {
		s = slope * (i * dt);
		out = fd.update(s);
	}

	EXPECT_NEAR(out, slope, 0.35f);
}

TEST(FilteredDerivative, resetClearsInitializedPath)
{
	FilteredDerivative<float> fd;
	fd.setParameters(0.01f, 0.05f);
	(void)fd.update(0.f);
	(void)fd.update(1.f);
	fd.reset(0.f);
	// after reset, first sample only primes; next three stay low for flat signal
	(void)fd.update(0.f);
	float out = fd.update(0.f);
	out =
fd.update(0.f);
	EXPECT_NEAR(out, 0.f, 0.15f);
}
