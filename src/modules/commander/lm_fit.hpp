/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

/**
 * Least-squares fit of a sphere to a set of points.
 *
 * Fits a sphere to a set of points on the sphere surface.
 *
 * @param x point coordinates on the X axis
 * @param y point coordinates on the Y axis
 * @param z point coordinates on the Z axis
 * @param size number of points
 * @param max_iterations abort if maximum number of iterations have been reached. If unsure, set to 100.
 * @param sphere_x coordinate of the sphere center on the X axis
 * @param sphere_y coordinate of the sphere center on the Y axis
 * @param sphere_z coordinate of the sphere center on the Z axis
 * @param sphere_radius sphere radius
 *
 * @return 0 on success, 1 on failure
 */
inline int run_lm_sphere_fit(const float x[], const float y[], const float z[], float &_fitness, float &_sphere_lambda,
			     unsigned int samples_collected, float *offset_x, float *offset_y, float *offset_z,
			     float *sphere_radius, float *diag_x, float *diag_y, float *diag_z, float *offdiag_x, float *offdiag_y, float *offdiag_z)
{
	// Run Sphere Fit using Levenberg Marquardt LSq Fit
	const float lma_damping = 10.f;
	float fitness = _fitness;
	float fit1 = 0.f;
	float fit2 = 0.f;

	matrix::SquareMatrix<float, 4> JTJ{};
	matrix::SquareMatrix<float, 4> JTJ2{};
	float JTFI[4] {};
	float residual = 0.0f;

	// Gauss Newton Part common for all kind of extensions including LM
	for (uint16_t k = 0; k < samples_collected; k++) {

		float sphere_jacob[4];
		//Calculate Jacobian
		float A = (*diag_x    * (x[k] - *offset_x)) + (*offdiag_x * (y[k] - *offset_y)) + (*offdiag_y * (z[k] - *offset_z));
		float B = (*offdiag_x * (x[k] - *offset_x)) + (*diag_y    * (y[k] - *offset_y)) + (*offdiag_z * (z[k] - *offset_z));
		float C = (*offdiag_y * (x[k] - *offset_x)) + (*offdiag_z * (y[k] - *offset_y)) + (*diag_z    * (z[k] - *offset_z));
		float length = sqrtf(A * A + B * B + C * C);

		// 0: partial derivative (radius wrt fitness fn) fn operated on sample
		sphere_jacob[0] = 1.0f;
		// 1-3: partial derivative (offsets wrt fitness fn) fn operated on sample
		sphere_jacob[1] = 1.0f * (((*diag_x    * A) + (*offdiag_x * B) + (*offdiag_y * C)) / length);
		sphere_jacob[2] = 1.0f * (((*offdiag_x * A) + (*diag_y    * B) + (*offdiag_z * C)) / length);
		sphere_jacob[3] = 1.0f * (((*offdiag_y * A) + (*offdiag_z * B) + (*diag_z    * C)) / length);
		residual = *sphere_radius - length;

		for (uint8_t i = 0; i < 4; i++) {
			// compute JTJ
			for (uint8_t j = 0; j < 4; j++) {
				JTJ(i, j) += sphere_jacob[i] * sphere_jacob[j];
				JTJ2(i, j) += sphere_jacob[i] * sphere_jacob[j]; //a backup JTJ for LM
			}

			JTFI[i] += sphere_jacob[i] * residual;
		}
	}


	//------------------------Levenberg-Marquardt-part-starts-here---------------------------------//
	// refer: http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm#Choice_of_damping_parameter
	float fit1_params[4] = {*sphere_radius, *offset_x, *offset_y, *offset_z};
	float fit2_params[4];
	memcpy(fit2_params, fit1_params, sizeof(fit1_params));

	for (uint8_t i = 0; i < 4; i++) {
		JTJ(i, i) += _sphere_lambda;
		JTJ2(i, i) += _sphere_lambda / lma_damping;
	}

	if (!JTJ.I(JTJ)) {
		return -1;
	}

	if (!JTJ2.I(JTJ2)) {
		return -1;
	}

	for (uint8_t row = 0; row < 4; row++) {
		for (uint8_t col = 0; col < 4; col++) {
			fit1_params[row] -= JTFI[col] * JTJ(row, col);
			fit2_params[row] -= JTFI[col] * JTJ2(row, col);
		}
	}

	// Calculate mean squared residuals
	for (uint16_t k = 0; k < samples_collected; k++) {
		float A = (*diag_x    * (x[k] - fit1_params[1])) + (*offdiag_x * (y[k] - fit1_params[2])) + (*offdiag_y *
				(z[k] + fit1_params[3]));
		float B = (*offdiag_x * (x[k] - fit1_params[1])) + (*diag_y    * (y[k] - fit1_params[2])) + (*offdiag_z *
				(z[k] + fit1_params[3]));
		float C = (*offdiag_y * (x[k] - fit1_params[1])) + (*offdiag_z * (y[k] - fit1_params[2])) + (*diag_z    *
				(z[k] - fit1_params[3]));
		float length = sqrtf(A * A + B * B + C * C);
		residual = fit1_params[0] - length;
		fit1 += residual * residual;

		A = (*diag_x    * (x[k] - fit2_params[1])) + (*offdiag_x * (y[k] - fit2_params[2])) + (*offdiag_y *
				(z[k] - fit2_params[3]));
		B = (*offdiag_x * (x[k] - fit2_params[1])) + (*diag_y    * (y[k] - fit2_params[2])) + (*offdiag_z *
				(z[k] - fit2_params[3]));
		C = (*offdiag_y * (x[k] - fit2_params[1])) + (*offdiag_z * (y[k] - fit2_params[2])) + (*diag_z    *
				(z[k] - fit2_params[3]));
		length = sqrtf(A * A + B * B + C * C);
		residual = fit2_params[0] - length;
		fit2 += residual * residual;
	}

	fit1 = sqrtf(fit1) / samples_collected;
	fit2 = sqrtf(fit2) / samples_collected;

	if (fit1 > _fitness && fit2 > _fitness) {
		_sphere_lambda *= lma_damping;

	} else if (fit2 < _fitness && fit2 < fit1) {
		_sphere_lambda /= lma_damping;
		memcpy(fit1_params, fit2_params, sizeof(fit1_params));
		fitness = fit2;

	} else if (fit1 < _fitness) {
		fitness = fit1;
	}

	//--------------------Levenberg-Marquardt-part-ends-here--------------------------------//

	if (PX4_ISFINITE(fitness) && fitness < _fitness) {
		_fitness = fitness;
		*sphere_radius = fit1_params[0];
		*offset_x = fit1_params[1];
		*offset_y = fit1_params[2];
		*offset_z = fit1_params[3];
		return 0;

	} else {
		return -1;
	}
}

inline int run_lm_ellipsoid_fit(const float x[], const float y[], const float z[], float &_fitness,
				float &_sphere_lambda, unsigned int samples_collected, float *offset_x, float *offset_y, float *offset_z,
				float *sphere_radius, float *diag_x, float *diag_y, float *diag_z, float *offdiag_x, float *offdiag_y, float *offdiag_z)
{
	// Run Sphere Fit using Levenberg Marquardt LSq Fit
	const float lma_damping = 10.0f;
	float fitness = _fitness;
	float fit1 = 0.0f;
	float fit2 = 0.0f;

	float JTJ[81] {};
	float JTJ2[81] {};
	float JTFI[9] {};
	float residual = 0.0f;
	float ellipsoid_jacob[9];

	// Gauss Newton Part common for all kind of extensions including LM
	for (uint16_t k = 0; k < samples_collected; k++) {

		// Calculate Jacobian
		float A = (*diag_x    * (x[k] - *offset_x)) + (*offdiag_x * (y[k] - *offset_y)) + (*offdiag_y * (z[k] - *offset_z));
		float B = (*offdiag_x * (x[k] - *offset_x)) + (*diag_y    * (y[k] - *offset_y)) + (*offdiag_z * (z[k] - *offset_z));
		float C = (*offdiag_y * (x[k] - *offset_x)) + (*offdiag_z * (y[k] - *offset_y)) + (*diag_z    * (z[k] - *offset_z));
		float length = sqrtf(A * A + B * B + C * C);
		residual = *sphere_radius - length;
		fit1 += residual * residual;
		// 0-2: partial derivative (offset wrt fitness fn) fn operated on sample
		ellipsoid_jacob[0] = 1.0f * (((*diag_x    * A) + (*offdiag_x * B) + (*offdiag_y * C)) / length);
		ellipsoid_jacob[1] = 1.0f * (((*offdiag_x * A) + (*diag_y    * B) + (*offdiag_z * C)) / length);
		ellipsoid_jacob[2] = 1.0f * (((*offdiag_y * A) + (*offdiag_z * B) + (*diag_z    * C)) / length);
		// 3-5: partial derivative (diag offset wrt fitness fn) fn operated on sample
		ellipsoid_jacob[3] = -1.0f * ((x[k] - *offset_x) * A) / length;
		ellipsoid_jacob[4] = -1.0f * ((y[k] - *offset_y) * B) / length;
		ellipsoid_jacob[5] = -1.0f * ((z[k] - *offset_z) * C) / length;
		// 6-8: partial derivative (off-diag offset wrt fitness fn) fn operated on sample
		ellipsoid_jacob[6] = -1.0f * (((y[k] - *offset_y) * A) + ((x[k] - *offset_x) * B)) / length;
		ellipsoid_jacob[7] = -1.0f * (((z[k] - *offset_z) * A) + ((x[k] - *offset_x) * C)) / length;
		ellipsoid_jacob[8] = -1.0f * (((z[k] - *offset_z) * B) + ((y[k] - *offset_y) * C)) / length;

		for (uint8_t i = 0; i < 9; i++) {
			// compute JTJ
			for (uint8_t j = 0; j < 9; j++) {
				JTJ[i * 9 + j] += ellipsoid_jacob[i] * ellipsoid_jacob[j];
				JTJ2[i * 9 + j] += ellipsoid_jacob[i] * ellipsoid_jacob[j]; // a backup JTJ for LM
			}

			JTFI[i] += ellipsoid_jacob[i] * residual;
		}
	}


	//------------------------Levenberg-Marquardt-part-starts-here---------------------------------//
	// refer: http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm#Choice_of_damping_parameter
	float fit1_params[9] = {*offset_x, *offset_y, *offset_z, *diag_x, *diag_y, *diag_z, *offdiag_x, *offdiag_y, *offdiag_z};
	float fit2_params[9];
	memcpy(fit2_params, fit1_params, sizeof(fit1_params));

	for (uint8_t i = 0; i < 9; i++) {
		JTJ[i * 9 + i] += _sphere_lambda;
		JTJ2[i * 9 + i] += _sphere_lambda / lma_damping;
	}


	if (!mat_inverse(JTJ, JTJ, 9)) {
		return -1;
	}

	if (!mat_inverse(JTJ2, JTJ2, 9)) {
		return -1;
	}

	for (uint8_t row = 0; row < 9; row++) {
		for (uint8_t col = 0; col < 9; col++) {
			fit1_params[row] -= JTFI[col] * JTJ[row * 9 + col];
			fit2_params[row] -= JTFI[col] * JTJ2[row * 9 + col];
		}
	}

	// Calculate mean squared residuals
	for (uint16_t k = 0; k < samples_collected; k++) {
		float A = (fit1_params[3]    * (x[k] - fit1_params[0])) + (fit1_params[6] * (y[k] - fit1_params[1])) + (fit1_params[7] *
				(z[k] - fit1_params[2]));
		float B = (fit1_params[6] * (x[k] - fit1_params[0])) + (fit1_params[4]   * (y[k] - fit1_params[1])) + (fit1_params[8] *
				(z[k] - fit1_params[2]));
		float C = (fit1_params[7] * (x[k] - fit1_params[0])) + (fit1_params[8] * (y[k] - fit1_params[1])) + (fit1_params[5]    *
				(z[k] - fit1_params[2]));
		float length = sqrtf(A * A + B * B + C * C);
		residual = *sphere_radius - length;
		fit1 += residual * residual;

		A = (fit2_params[3]    * (x[k] - fit2_params[0])) + (fit2_params[6] * (y[k] - fit2_params[1])) + (fit2_params[7] *
				(z[k] - fit2_params[2]));
		B = (fit2_params[6] * (x[k] - fit2_params[0])) + (fit2_params[4]   * (y[k] - fit2_params[1])) + (fit2_params[8] *
				(z[k] - fit2_params[2]));
		C = (fit2_params[7] * (x[k] - fit2_params[0])) + (fit2_params[8] * (y[k] - fit2_params[1])) + (fit2_params[5]    *
				(z[k] - fit2_params[2]));
		length = sqrtf(A * A + B * B + C * C);
		residual = *sphere_radius - length;
		fit2 += residual * residual;
	}

	fit1 = sqrtf(fit1) / samples_collected;
	fit2 = sqrtf(fit2) / samples_collected;

	if (fit1 > _fitness && fit2 > _fitness) {
		_sphere_lambda *= lma_damping;

	} else if (fit2 < _fitness && fit2 < fit1) {
		_sphere_lambda /= lma_damping;
		memcpy(fit1_params, fit2_params, sizeof(fit1_params));
		fitness = fit2;

	} else if (fit1 < _fitness) {
		fitness = fit1;
	}

	//--------------------Levenberg-Marquardt-part-ends-here--------------------------------//
	if (PX4_ISFINITE(fitness) && fitness < _fitness) {
		_fitness = fitness;
		*offset_x = fit1_params[0];
		*offset_y = fit1_params[1];
		*offset_z = fit1_params[2];
		*diag_x = fit1_params[3];
		*diag_y = fit1_params[4];
		*diag_z = fit1_params[5];
		*offdiag_x = fit1_params[6];
		*offdiag_y = fit1_params[7];
		*offdiag_z = fit1_params[8];
		return 0;

	} else {
		return -1;
	}
}
