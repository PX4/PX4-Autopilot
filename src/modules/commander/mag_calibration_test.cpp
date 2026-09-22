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
#include "mag_rotation_detection.hpp"
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

	// WHEN: fitting a sphere with the data and given a wrong initial radius
	sphere_params sphere;
	sphere.diag = {1.f, 1.f, 1.f};
	sphere.radius = 0.2;
	int success = lm_mag_fit(x, y, z, N_SAMPLES, sphere, false);

	// THEN: the algorithm should converge in a single step
	EXPECT_EQ(success, PX4_OK);
	EXPECT_NEAR(sphere.radius, mag_str_true, 0.001f) << "radius: " << sphere.radius;
	EXPECT_NEAR(sphere.offset(0), offset_true(0), 0.001f) << "offset X: " << sphere.offset(0);
	EXPECT_NEAR(sphere.offset(1), offset_true(1), 0.001f) << "offset Y: " << sphere.offset(1);
	EXPECT_NEAR(sphere.offset(2), offset_true(2), 0.001f) << "offset Z: " << sphere.offset(2);
	EXPECT_NEAR(sphere.diag(0), scale_true(0), 0.001f) << "scale X: " << sphere.diag(0);
	EXPECT_NEAR(sphere.diag(1), scale_true(1), 0.001f) << "scale Y: " << sphere.diag(1);
	EXPECT_NEAR(sphere.diag(2), scale_true(2), 0.001f) << "scale Z: " << sphere.diag(2);
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

	// WHEN: fitting a sphere to the data
	sphere_params sphere;
	sphere.diag = {1.f, 1.f, 1.f};
	sphere.radius = 0.2;
	int success = lm_mag_fit(x, y, z, N_SAMPLES, sphere, false);

	// THEN: the algorithm should converge in a few iterations and
	// find the correct parameters
	EXPECT_EQ(success, PX4_OK);
	EXPECT_NEAR(sphere.radius, mag_str_true, 0.001f) << "radius: " << sphere.radius;
	EXPECT_NEAR(sphere.offset(0), offset_true(0), 0.001f) << "offset X: " << sphere.offset(0);
	EXPECT_NEAR(sphere.offset(1), offset_true(1), 0.001f) << "offset Y: " << sphere.offset(1);
	EXPECT_NEAR(sphere.offset(2), offset_true(2), 0.001f) << "offset Z: " << sphere.offset(2);
	EXPECT_NEAR(sphere.diag(0), scale_true(0), 0.001f) << "scale X: " << scale_true(0);
	EXPECT_NEAR(sphere.diag(1), scale_true(1), 0.001f) << "scale Y: " << scale_true(1);
	EXPECT_NEAR(sphere.diag(2), scale_true(2), 0.001f) << "scale Z: " << scale_true(2);
}

TEST_F(MagCalTest, replayTestData)
{
	// GIVEN: a real test dataset with large offsets
	// and where the two first iterations of the LM algorithm
	// produces a negative radius and a constant fitness value
	constexpr unsigned int N_SAMPLES = 231;

	const float mag_str_true = 0.4f;
	const Vector3f offset_true = {-0.18f, 0.05f, -0.58f};

	// WHEN: fitting a sphere to the data
	sphere_params sphere;
	sphere.diag = {1.f, 1.f, 1.f};
	sphere.radius = 0.2;
	int sphere_success = lm_mag_fit(mag_data1_x, mag_data1_y, mag_data1_z, N_SAMPLES, sphere, false);

	// THEN: the algorithm should converge and find the correct parameters
	EXPECT_EQ(sphere_success, PX4_OK);
	EXPECT_NEAR(sphere.radius, mag_str_true, 0.1f) << "radius: " << sphere.radius;
	EXPECT_NEAR(sphere.offset(0), offset_true(0), 0.01f) << "offset X: " << sphere.offset(0);
	EXPECT_NEAR(sphere.offset(1), offset_true(1), 0.01f) << "offset Y: " << sphere.offset(1);
	EXPECT_NEAR(sphere.offset(2), offset_true(2), 0.01f) << "offset Z: " << sphere.offset(2);

	printf("Ellipsoid fit\n");
	sphere_params ellipsoid;
	ellipsoid.diag = {1.f, 1.f, 1.f};
	ellipsoid.radius = 0.2;
	int ellipsoid_step_1_success = lm_mag_fit(mag_data1_x, mag_data1_y, mag_data1_z, N_SAMPLES, ellipsoid, false);
	int ellipsoid_success = lm_mag_fit(mag_data1_x, mag_data1_y, mag_data1_z, N_SAMPLES, ellipsoid, true);
	const Vector3f scale_true = {1.f, 1.06f, 0.94f};

	EXPECT_EQ(ellipsoid_step_1_success, PX4_OK);
	EXPECT_EQ(ellipsoid_success, PX4_OK);
	EXPECT_NEAR(ellipsoid.radius, mag_str_true, 0.1f) << "radius: " << sphere.radius;
	EXPECT_NEAR(ellipsoid.offset(0), offset_true(0), 0.01f) << "offset X: " << ellipsoid.offset(0);
	EXPECT_NEAR(ellipsoid.offset(1), offset_true(1), 0.01f) << "offset Y: " << ellipsoid.offset(1);
	EXPECT_NEAR(ellipsoid.offset(2), offset_true(2), 0.01f) << "offset Z: " << ellipsoid.offset(2);
	EXPECT_NEAR(ellipsoid.diag(0), scale_true(0), 0.01f) << "scale X: " << ellipsoid.diag(0);
	EXPECT_NEAR(ellipsoid.diag(1), scale_true(1), 0.01f) << "scale Y: " << ellipsoid.diag(1);
	EXPECT_NEAR(ellipsoid.diag(2), scale_true(2), 0.01f) << "scale Z: " << ellipsoid.diag(2);
}

/* Simulate a 6 sides calibration: the vehicle rests on each side while rotated a full turn around the vertical axis.
 * Mag samples are generated in sensor frame for a given mag rotation (sensor to body), accel in body frame.
 */
static unsigned generateGravityRotationData(float *x, float *y, float *z, Vector3f *accel, Rotation mag_rotation,
		bool all_sides, float noise_gauss = 0.005f, float noise_mss = 0.3f)
{
	using matrix::Dcmf;
	using matrix::Eulerf;

	// earth field in NED (inclination ~62 deg)
	const Vector3f mag_earth{0.24f, 0.02f, 0.45f};
	const Vector3f gravity_ned{0.f, 0.f, -9.80665f}; // specific force at rest

	const Eulerf sides[6] {
		{0.f, 0.f, 0.f},               // level
		{M_PI_F, 0.f, 0.f},            // upside down
		{M_PI_F / 2.f, 0.f, 0.f},      // left
		{-M_PI_F / 2.f, 0.f, 0.f},     // right
		{0.f, M_PI_F / 2.f, 0.f},      // nose up
		{0.f, -M_PI_F / 2.f, 0.f},     // nose down
	};

	const Dcmf R_mag = get_rot_matrix(mag_rotation);
	const unsigned n_sides = all_sides ? 6 : 1;
	const unsigned n_per_side = 40;

	srand(1234);
	auto noise = [](float amplitude) { return amplitude * (2.f * (float)rand() / (float)RAND_MAX - 1.f); };

	unsigned n = 0;

	for (unsigned s = 0; s < n_sides; s++) {
		for (unsigned i = 0; i < n_per_side; i++) {
			const float yaw = 2.f * M_PI_F * i / n_per_side;
			const Dcmf C_nb = Dcmf(Eulerf(0.f, 0.f, yaw)) * Dcmf(sides[s]); // body to NED

			const Vector3f mag_body = C_nb.transpose() * mag_earth;
			const Vector3f mag_sensor = R_mag.transpose() * mag_body;

			x[n] = mag_sensor(0) + noise(noise_gauss);
			y[n] = mag_sensor(1) + noise(noise_gauss);
			z[n] = mag_sensor(2) + noise(noise_gauss);
			accel[n] = C_nb.transpose() * gravity_ned + Vector3f{noise(noise_mss), noise(noise_mss), noise(noise_mss)};
			n++;
		}
	}

	return n;
}

TEST_F(MagCalTest, rotationFromGravity)
{
	static constexpr unsigned N_MAX = 240;
	float x[N_MAX], y[N_MAX], z[N_MAX];
	Vector3f accel[N_MAX];

	for (int r = ROTATION_NONE; r < ROTATION_MAX; r++) {
		if (mag_rotation_detection::rotation_skipped(r)) {
			continue;
		}

		// GIVEN: 6 sides of data from a mag mounted with rotation r
		const unsigned n = generateGravityRotationData(x, y, z, accel, (Rotation)r, true);

		// WHEN: we detect the rotation using gravity
		const mag_rotation_detection::Result res = mag_rotation_detection::detect(x, y, z, accel, n);

		// THEN: the rotation is found
		EXPECT_TRUE(res.valid) << "rotation " << r << " confidence " << res.confidence << " std " << res.best_std;
		EXPECT_EQ((int)res.best_rotation, r) << "confidence " << res.confidence;
	}
}

TEST_F(MagCalTest, rotationFromGravityUpsideDown)
{
	static constexpr unsigned N_MAX = 240;
	float x[N_MAX], y[N_MAX], z[N_MAX];
	Vector3f accel[N_MAX];

	// GIVEN: mag mounted upside down, with larger sensor noise
	const unsigned n = generateGravityRotationData(x, y, z, accel, ROTATION_PITCH_180, true, 0.02f, 1.f);

	// WHEN: we detect the rotation using gravity
	const mag_rotation_detection::Result res = mag_rotation_detection::detect(x, y, z, accel, n);

	// THEN: CAL_MAGx_ROT 12 is found
	EXPECT_TRUE(res.valid);
	EXPECT_EQ(res.best_rotation, ROTATION_PITCH_180);
}

TEST_F(MagCalTest, rotationFromGravityAmbiguous)
{
	static constexpr unsigned N_MAX = 240;
	float x[N_MAX], y[N_MAX], z[N_MAX];
	Vector3f accel[N_MAX];

	// GIVEN: data from a single side only (yaw rotations can not be distinguished)
	const unsigned n = generateGravityRotationData(x, y, z, accel, ROTATION_YAW_90, false);

	// WHEN: we detect the rotation using gravity
	const mag_rotation_detection::Result res = mag_rotation_detection::detect(x, y, z, accel, n);

	// THEN: no rotation is reported
	EXPECT_FALSE(res.valid);
}

TEST_F(MagCalTest, rotationFromGravityNoAccel)
{
	static constexpr unsigned N_MAX = 240;
	float x[N_MAX], y[N_MAX], z[N_MAX];
	Vector3f accel[N_MAX];

	// GIVEN: valid mag data but no accel data available
	const unsigned n = generateGravityRotationData(x, y, z, accel, ROTATION_PITCH_180, true);

	for (unsigned i = 0; i < n; i++) {
		accel[i] = Vector3f{NAN, NAN, NAN};
	}

	// WHEN: we detect the rotation using gravity
	const mag_rotation_detection::Result res = mag_rotation_detection::detect(x, y, z, accel, n);

	// THEN: no rotation is reported
	EXPECT_FALSE(res.valid);
	EXPECT_EQ(res.samples_used, 0u);
}
