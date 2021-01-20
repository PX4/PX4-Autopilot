/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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
 * Test code for the Magnetometer calibration routine
 * Run this test only using make tests TESTFILTER=mag_calibration
 *
 * @author Mathieu Bresciani <mathieu@auterion.com>
 */

#include <gtest/gtest.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>

#include "lm_fit.hpp"
#include "mag_calibration_test_data.h"

using matrix::Vector3f;

class MagCalTest : public ::testing::Test
{
public:
	void generate2SidesMagData(float *x, float *y, float *z, unsigned int n_samples, float mag_str);

	/* Generate regularly spaced data on a sphere
	 * Ref.: How to generate equidistributed points on the surface of a sphere, Markus Deserno, 2004
	 */
	void generateRegularData(float *x, float *y, float *z, unsigned int n_samples, float mag_str);

	void modifyOffsetScale(float *x, float *y, float *z, unsigned int n_samples, Vector3f offsets, Vector3f scale_factors);
};

void MagCalTest::generate2SidesMagData(float *x, float *y, float *z, unsigned int n_samples, float mag_str)
{
	float psi = 0.f;
	float theta = 0.f;
	const float d_angle = 2.f * M_PI_F / float(n_samples / 2);

	for (int i = 0; i < int(n_samples / 2); i++) {
		x[i] = mag_str * sinf(psi);
		y[i] = mag_str * cosf(psi);
		z[i] = 0.f;
		psi += d_angle;
	}

	for (int i = int(n_samples / 2); i < int(n_samples); i++) {
		x[i] = mag_str * sinf(theta);
		y[i] = 0.f;
		z[i] = mag_str * cosf(theta);
		theta += d_angle;
	}
}

void MagCalTest::generateRegularData(float *x, float *y, float *z, unsigned int n_samples, float mag_str)
{
	const float a = 4.f * M_PI_F * mag_str * mag_str / n_samples;
	const float d = sqrtf(a);
	const int m_theta = static_cast<int>(M_PI_F / d);
	const float d_theta = M_PI_F / static_cast<float>(m_theta);
	const float d_phi = a / d_theta;

	unsigned int n_count = 0;

	for (int m = 0; m < m_theta; m++) {
		const float theta = M_PI_F * (m + 0.5f) / static_cast<float>(m_theta);
		const int m_phi = static_cast<int>(2.f * M_PI_F * sinf(theta / d_phi));

		for (int n = 0; n < m_phi; n++) {
			const float phi = 2.f * M_PI_F * n / static_cast<float>(m_phi);
			x[n_count] = mag_str * sinf(theta) * cosf(phi);
			y[n_count] = mag_str * sinf(theta) * sinf(phi);
			z[n_count] = mag_str * cosf(theta);
			n_count++;
		}
	}

	if (n_count > n_samples) {
		printf("Error placing samples, n = %d\n", n_count);
		return;
	}

	// Padd with constant data
	while (n_count < n_samples) {
		x[n_count] = x[n_count - 1];
		y[n_count] = y[n_count - 1];
		z[n_count] = z[n_count - 1];
		n_count++;
	}
}

void MagCalTest::modifyOffsetScale(float *x, float *y, float *z, unsigned int n_samples, Vector3f offsets,
				   Vector3f scale_factors)
{
	for (unsigned int k = 0; k < n_samples; k++) {
		x[k] = x[k] * scale_factors(0) + offsets(0);
		y[k] = y[k] * scale_factors(1) + offsets(1);
		z[k] = z[k] * scale_factors(2) + offsets(2);
	}
}

TEST_F(MagCalTest, sphere2Sides)
{
	// GIVEN: a dataset of points located on two orthogonal circles
	// perfectly centered on the origin
	static constexpr unsigned int N_SAMPLES = 240;

	const float mag_str_true = 0.4f;
	const Vector3f offset_true;
	const Vector3f scale_true = {1.f, 1.f, 1.f};

	float x[N_SAMPLES];
	float y[N_SAMPLES];
	float z[N_SAMPLES];

	generate2SidesMagData(x, y, z, N_SAMPLES, mag_str_true);

	float fitness = 1.0e30f;
	float sphere_lambda = 1.f;
	float sphere_radius = 0.2f;
	Vector3f offset;
	Vector3f diag = {1.f, 1.f, 1.f};
	Vector3f offdiag;

	// WHEN: fitting a sphere with the data and given a wrong initial radius
	int ret = run_lm_sphere_fit(x, y, z,
				    fitness, sphere_lambda, N_SAMPLES,
				    &offset(0), &offset(1), &offset(2),
				    &sphere_radius,
				    &diag(0), &diag(1), &diag(2),
				    &offdiag(0), &offdiag(1), &offdiag(2));

	// THEN: the algorithm should converge in a single step
	EXPECT_EQ(ret, 0);
	EXPECT_LT(fitness, 1e-5f);
	EXPECT_NEAR(sphere_radius, mag_str_true, 0.001f) << "radius: " << sphere_radius;
	EXPECT_NEAR(offset(0), offset_true(0), 0.001f) << "offset X: " << offset(0);
	EXPECT_NEAR(offset(1), offset_true(1), 0.001f) << "offset Y: " << offset(1);
	EXPECT_NEAR(offset(2), offset_true(2), 0.001f) << "offset Z: " << offset(2);
	EXPECT_NEAR(diag(0), scale_true(0), 0.001f) << "scale X: " << scale_true(0);
	EXPECT_NEAR(diag(1), scale_true(1), 0.001f) << "scale Y: " << scale_true(1);
	EXPECT_NEAR(diag(2), scale_true(2), 0.001f) << "scale Z: " << scale_true(2);
}

TEST_F(MagCalTest, sphereRegularlySpaced)
{
	// GIVEN: a dataset of regularly spaced points
	// on a perfect sphere but not centered on the origin
	static constexpr unsigned int N_SAMPLES = 240;

	const float mag_str_true = 0.4f;
	const Vector3f offset_true = {-1.07f, 0.35f, -0.78f};
	const Vector3f scale_true = {1.f, 1.f, 1.f};

	float x[N_SAMPLES];
	float y[N_SAMPLES];
	float z[N_SAMPLES];
	generateRegularData(x, y, z, N_SAMPLES, mag_str_true);
	modifyOffsetScale(x, y, z, N_SAMPLES, offset_true, scale_true);

	float fitness = 1.0e30f;
	float sphere_lambda = 1.f;
	float sphere_radius = 0.2f;
	Vector3f offset;
	Vector3f diag = {1.f, 1.f, 1.f};
	Vector3f offdiag;

	bool sphere_fit_success = false;

	// WHEN: fitting a sphere to the data
	for (int i = 0; i < 8; i++) {
		const bool ret = run_lm_sphere_fit(x, y, z,
						   fitness, sphere_lambda, N_SAMPLES,
						   &offset(0), &offset(1), &offset(2),
						   &sphere_radius,
						   &diag(0), &diag(1), &diag(2),
						   &offdiag(0), &offdiag(1), &offdiag(2));

		if (ret == 0) {
			sphere_fit_success = true;

		} else if (sphere_fit_success) {
			break;
		}
	}

	// THEN: the algorithm should converge in a few iterations and
	// find the correct parameters
	EXPECT_TRUE(sphere_fit_success);
	EXPECT_LT(fitness, 1e-6f);
	EXPECT_NEAR(sphere_radius, mag_str_true, 0.001f) << "radius: " << sphere_radius;
	EXPECT_NEAR(offset(0), offset_true(0), 0.001f) << "offset X: " << offset(0);
	EXPECT_NEAR(offset(1), offset_true(1), 0.001f) << "offset Y: " << offset(1);
	EXPECT_NEAR(offset(2), offset_true(2), 0.001f) << "offset Z: " << offset(2);
	EXPECT_NEAR(diag(0), scale_true(0), 0.001f) << "scale X: " << scale_true(0);
	EXPECT_NEAR(diag(1), scale_true(1), 0.001f) << "scale Y: " << scale_true(1);
	EXPECT_NEAR(diag(2), scale_true(2), 0.001f) << "scale Z: " << scale_true(2);
}

TEST_F(MagCalTest, replayTestData)
{
	// GIVEN: a real test dataset with large offsets
	// and where the two first iterations of the LM algorithm
	// produces a negative radius and a constant fitness value
	constexpr unsigned int N_SAMPLES = 231;

	const float mag_str_true = 0.4f;
	const Vector3f offset_true = {-0.18f, 0.05f, -0.58f};
	const Vector3f scale_true = {1.f, 1.06f, 0.94f};

	float fitness = 1.0e30f;
	float sphere_lambda = 1.f;
	float sphere_radius = 0.2f;
	Vector3f offset;
	Vector3f diag = {1.f, 1.f, 1.f};
	Vector3f offdiag;

	bool sphere_fit_success = false;

	// WHEN: fitting a sphere to the data
	for (int i = 0; i < 100; i++) {
		const bool ret = run_lm_sphere_fit(mag_data1_x, mag_data1_y, mag_data1_z,
						   fitness, sphere_lambda, N_SAMPLES,
						   &offset(0), &offset(1), &offset(2),
						   &sphere_radius,
						   &diag(0), &diag(1), &diag(2),
						   &offdiag(0), &offdiag(1), &offdiag(2));

		printf("fitness: %.6f\t sphere_lambda: %.3f\t radius: %.3f\n",
		       (double)fitness, (double)sphere_lambda, (double)sphere_radius);

		// This is fragile because it is a copy of the code and not a
		// test of the code itself. TODO: move the check in a function that
		// can be tested here
		if (ret == 0) {

			sphere_fit_success = true;

		} else if (sphere_fit_success
			   && (i > 10)
			   && (fitness < 0.01f)
			   && (sphere_radius >= 0.2f)
			   && (sphere_radius <= 0.7f)) {
			break;
		}
	}

	printf("Ellipsoid fit\n");
	bool ellipsoid_fit_success = false;

	for (int i = 0; i < 100; i++) {
		const bool ret = run_lm_ellipsoid_fit(mag_data1_x, mag_data1_y, mag_data1_z,
						      fitness, sphere_lambda, N_SAMPLES,
						      &offset(0), &offset(1), &offset(2),
						      &sphere_radius,
						      &diag(0), &diag(1), &diag(2),
						      &offdiag(0), &offdiag(1), &offdiag(2));

		printf("fitness: %.6f\t sphere_lambda: %.3f\t radius: %.3f\n",
		       (double)fitness, (double)sphere_lambda, (double)sphere_radius);

		if (ret == 0) {
			ellipsoid_fit_success = true;

		} else if (ellipsoid_fit_success) {
			break;
		}
	}

	// THEN: the algorithm should converge and find the correct parameters
	EXPECT_TRUE(sphere_fit_success);
	EXPECT_TRUE(ellipsoid_fit_success);
	EXPECT_LT(fitness, 1e-3f);
	EXPECT_NEAR(sphere_radius, mag_str_true, 0.1f) << "radius: " << sphere_radius;
	EXPECT_NEAR(offset(0), offset_true(0), 0.01f) << "offset X: " << offset(0);
	EXPECT_NEAR(offset(1), offset_true(1), 0.01f) << "offset Y: " << offset(1);
	EXPECT_NEAR(offset(2), offset_true(2), 0.01f) << "offset Z: " << offset(2);
	EXPECT_NEAR(diag(0), scale_true(0), 0.01f) << "scale X: " << diag(0);
	EXPECT_NEAR(diag(1), scale_true(1), 0.01f) << "scale Y: " << diag(1);
	EXPECT_NEAR(diag(2), scale_true(2), 0.01f) << "scale Z: " << diag(2);
}
