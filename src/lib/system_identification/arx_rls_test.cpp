/****************************************************************************
 *
 *   Copyright (C) 2020-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * Test code for the ArxRls class
 * Run this test only using make tests TESTFILTER=arx_rls
 */

#include <gtest/gtest.h>
#include <matrix/matrix/math.hpp>

#include "arx_rls.hpp"

using namespace matrix;

class ArxRlsTest : public ::testing::Test
{
public:
	ArxRlsTest() {};
};

TEST_F(ArxRlsTest, test211)
{
	ArxRls<2, 1, 1> _rls;

	for (int i = 0; i < (2 + 1 + 1); i++) {
		// Fill the buffers with zeros
		_rls.update(0.f, 0.f);
	}

	_rls.update(1, 2);
	_rls.update(3, 4);
	_rls.update(5, 6);
	const Vector4f coefficients = _rls.getCoefficients();
	const Vector4f coefficients_check(-1.79f, 0.97f, 0.42f, -0.48f); // generated from Python script
	float eps = 1e-2;
	EXPECT_TRUE((coefficients - coefficients_check).abs().max() < eps);
}

TEST_F(ArxRlsTest, test221)
{
	ArxRls<2, 2, 1> _rls;

	for (int i = 0; i < (2 + 2 + 1); i++) {
		// Fill the buffers with zeros
		_rls.update(0.f, 0.f);
	}

	_rls.update(1, 2);
	_rls.update(3, 4);
	_rls.update(5, 6);
	_rls.update(7, 8);
	const Vector<float, 5> coefficients = _rls.getCoefficients();
	float data_check[] = {-1.81, 1.06f, 0.38f, -0.27f, 0.26f};
	const Vector<float, 5> coefficients_check(data_check);
	float eps = 1e-2;
	EXPECT_TRUE((coefficients - coefficients_check).abs().max() < eps);
	coefficients.print();
	coefficients_check.print();
}

TEST_F(ArxRlsTest, resetTest)
{
	ArxRls<2, 2, 1> _rls;
	_rls.update(1, 2);
	_rls.update(3, 4);
	_rls.update(5, 6);
	_rls.update(7, 8);
	const Vector<float, 5> coefficients = _rls.getCoefficients();

	// WHEN: resetting
	_rls.reset();

	// THEN: the variances and coefficients should be properly reset
	EXPECT_TRUE(_rls.getVariances().min() > 5000.f);
	EXPECT_TRUE(_rls.getCoefficients().abs().max() < 1e-8f);

	// AND WHEN: running the same sequence of inputs-outputs
	_rls.update(1, 2);
	_rls.update(3, 4);
	_rls.update(5, 6);
	_rls.update(7, 8);

	// THEN: the result should be exactly the same
	EXPECT_TRUE((coefficients - _rls.getCoefficients()).abs().max() < 1e-8f);
}
