/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * Extra Utilities.hpp coverage (sq, taitBryan312ToRotMat, quatToInverseRotMat)
 * make tests TESTFILTER=UtilitiesExtras
 */

#include <gtest/gtest.h>
#include <cmath>
#include <mathlib/mathlib.h>

using namespace math::Utilities;

TEST(UtilitiesExtras, sq)
{
	EXPECT_FLOAT_EQ(sq(0.f), 0.f);
	EXPECT_FLOAT_EQ(sq(3.f), 9.f);
	EXPECT_FLOAT_EQ(sq(-2.f), 4.f);
}

TEST(UtilitiesExtras, taitBryan312Identity)
{
	const matrix::Dcmf R = taitBryan312ToRotMat(matrix::Vector3f(0.f, 0.f, 0.f));
	EXPECT_NEAR(R(0, 0), 1.f, 1e-5f);
	EXPECT_NEAR(R(1, 1), 1.f, 1e-5f);
	EXPECT_NEAR(R(2, 2), 1.f, 1e-5f);
	EXPECT_NEAR(R(0, 1), 0.f, 1e-5f);
}

TEST(UtilitiesExtras, quatToInverseRotMatMatchesDcmTranspose)
{
	matrix::Quatf q(matrix::Eulerf(0.2f, -0.1f, 0.4f));
	q.normalize();
	const matrix::Dcmf R(q);
	const matrix::Dcmf Rinv = quatToInverseRotMat(q);

	// inverse rotation should equal R^T for pure rotation
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			EXPECT_NEAR(Rinv(i, j), R(j, i), 1e-4f);
		}
	}
}
