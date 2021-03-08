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

#include "lm_fit.hpp"

struct iteration_result {
	float gradient_damping;
	float cost;
	enum class STATUS {
		SUCCESS, FAILURE
	} result = STATUS::SUCCESS;
};

static void lm_sphere_fit_iteration(const matrix::Vector3f data[], unsigned samples_collected, sphere_params &p,
				    iteration_result &result)
{
	// Run Sphere Fit using Levenberg Marquardt LSq Fit
	const float lma_damping = 10.f;
	float fit1 = 0.f;
	float fit2 = 0.f;

	matrix::SquareMatrix<float, 4> JTJ;
	float JTFI[4] {};

	// Gauss Newton Part common for all kind of extensions including LM
	for (unsigned k = 0; k < samples_collected; k++) {
		float x = data[k](0);
		float y = data[k](1);
		float z = data[k](2);

		// Calculate Jacobian
		float A = (p.diag(0)    * (x - p.offset(0))) + (p.offdiag(0) * (y - p.offset(1))) + (p.offdiag(1) * (z - p.offset(2)));
		float B = (p.offdiag(0) * (x - p.offset(0))) + (p.diag(1)    * (y - p.offset(1))) + (p.offdiag(2) * (z - p.offset(2)));
		float C = (p.offdiag(1) * (x - p.offset(0))) + (p.offdiag(2) * (y - p.offset(1))) + (p.diag(2)    * (z - p.offset(2)));

		float length = sqrtf(A * A + B * B + C * C);

		float sphere_jacob[4];
		// 0: partial derivative (radius wrt fitness fn) fn operated on sample
		sphere_jacob[0] = 1.f;
		// 1-3: partial derivative (offsets wrt fitness fn) fn operated on sample
		sphere_jacob[1] = 1.f * (((p.diag(0)    * A) + (p.offdiag(0) * B) + (p.offdiag(1) * C)) / length);
		sphere_jacob[2] = 1.f * (((p.offdiag(0) * A) + (p.diag(1)    * B) + (p.offdiag(2) * C)) / length);
		sphere_jacob[3] = 1.f * (((p.offdiag(1) * A) + (p.offdiag(2) * B) + (p.diag(2)    * C)) / length);

		float residual = p.radius - length;

		for (int i = 0; i < 4; i++) {
			// compute JTJ
			for (int j = 0; j < 4; j++) {
				JTJ(i, j) += sphere_jacob[i] * sphere_jacob[j];
			}

			JTFI[i] += sphere_jacob[i] * residual;
		}
	}


	//------------------------Levenberg-Marquardt-part-starts-here---------------------------------//
	// refer: https://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm#Choice_of_damping_parameter
	float fit1_p[4] {p.radius, p.offset(0), p.offset(1), p.offset(2)};
	float fit2_p[4] {p.radius, p.offset(0), p.offset(1), p.offset(2)};

	JTJ = (JTJ + JTJ.transpose()) * 0.5f; // fix numerical issues
	matrix::SquareMatrix<float, 4> JTJ2{JTJ};

	for (uint8_t i = 0; i < 4; i++) {
		JTJ(i, i) += result.gradient_damping;
		JTJ2(i, i) += result.gradient_damping / lma_damping;
	}

	if (!JTJ.I(JTJ)) {
		result.result = iteration_result::STATUS::FAILURE;
		return;
	}

	if (!JTJ2.I(JTJ2)) {
		result.result = iteration_result::STATUS::FAILURE;
		return;
	}

	for (int row = 0; row < 4; row++) {
		for (int col = 0; col < 4; col++) {
			fit1_p[row] -= JTFI[col] * JTJ(row, col);
			fit2_p[row] -= JTFI[col] * JTJ2(row, col);
		}
	}

	// Calculate mean squared residuals
	for (unsigned k = 0; k < samples_collected; k++) {
		float x = data[k](0);
		float y = data[k](1);
		float z = data[k](2);

		float A = (p.diag(0)    * (x - fit1_p[1])) + (p.offdiag(0) * (y - fit1_p[2])) + (p.offdiag(1) * (z + fit1_p[3]));
		float B = (p.offdiag(0) * (x - fit1_p[1])) + (p.diag(1)    * (y - fit1_p[2])) + (p.offdiag(2) * (z + fit1_p[3]));
		float C = (p.offdiag(1) * (x - fit1_p[1])) + (p.offdiag(2) * (y - fit1_p[2])) + (p.diag(2)    * (z - fit1_p[3]));

		float length = sqrtf(A * A + B * B + C * C);
		float residual = fit1_p[0] - length;
		fit1 += residual * residual;

		A = (p.diag(0)    * (x - fit2_p[1])) + (p.offdiag(0) * (y - fit2_p[2])) + (p.offdiag(1) * (z - fit2_p[3]));
		B = (p.offdiag(0) * (x - fit2_p[1])) + (p.diag(1)    * (y - fit2_p[2])) + (p.offdiag(2) * (z - fit2_p[3]));
		C = (p.offdiag(1) * (x - fit2_p[1])) + (p.offdiag(2) * (y - fit2_p[2])) + (p.diag(2)    * (z - fit2_p[3]));
		length = sqrtf(A * A + B * B + C * C);
		residual = fit2_p[0] - length;
		fit2 += residual * residual;
	}

	float fitness = result.cost;

	fit1 = sqrtf(fit1) / samples_collected;
	fit2 = sqrtf(fit2) / samples_collected;

	if (fit1 > result.cost && fit2 > result.cost) {
		result.gradient_damping *= lma_damping;

	} else if (fit2 < result.cost && fit2 < fit1) {
		result.gradient_damping /= lma_damping;
		memcpy(fit1_p, fit2_p, sizeof(fit1_p));
		fitness = fit2;

	} else if (fit1 < result.cost) {
		fitness = fit1;
	}

	//--------------------Levenberg-Marquardt-part-ends-here--------------------------------//
	if (PX4_ISFINITE(fitness) && fitness <= result.cost) {
		result.cost = fitness;

		p.radius    = fit1_p[0];

		p.offset(0) = fit1_p[1];
		p.offset(1) = fit1_p[2];
		p.offset(2) = fit1_p[3];

		result.result = iteration_result::STATUS::SUCCESS;

	} else {
		result.result = iteration_result::STATUS::FAILURE;
	}
}

static void lm_ellipsoid_fit_iteration(const matrix::Vector3f data[], unsigned samples_collected, sphere_params &p,
				       iteration_result &result)
{
	// Run Sphere Fit using Levenberg Marquardt LSq Fit
	const float lma_damping = 10.f;

	float fit1 = 0.f;
	float fit2 = 0.f;

	matrix::SquareMatrix<float, 9> JTJ{};
	float JTFI[9] {};

	// Gauss Newton Part common for all kind of extensions including LM
	for (unsigned k = 0; k < samples_collected; k++) {
		float x = data[k](0);
		float y = data[k](1);
		float z = data[k](2);

		// Calculate Jacobian
		float A = (p.diag(0)    * (x - p.offset(0))) + (p.offdiag(0) * (y - p.offset(1))) + (p.offdiag(1) * (z - p.offset(2)));
		float B = (p.offdiag(0) * (x - p.offset(0))) + (p.diag(1)    * (y - p.offset(1))) + (p.offdiag(2) * (z - p.offset(2)));
		float C = (p.offdiag(1) * (x - p.offset(0))) + (p.offdiag(2) * (y - p.offset(1))) + (p.diag(2)    * (z - p.offset(2)));
		float length = sqrtf(A * A + B * B + C * C);
		float residual = p.radius - length;
		fit1 += residual * residual;

		float ellipsoid_jacob[9];
		// 0-2: partial derivative (offset wrt fitness fn) fn operated on sample
		ellipsoid_jacob[0] =  1.f * (((p.diag(0)    * A) + (p.offdiag(0) * B) + (p.offdiag(1) * C)) / length);
		ellipsoid_jacob[1] =  1.f * (((p.offdiag(0) * A) + (p.diag(1)    * B) + (p.offdiag(2) * C)) / length);
		ellipsoid_jacob[2] =  1.f * (((p.offdiag(1) * A) + (p.offdiag(2) * B) + (p.diag(2)    * C)) / length);
		// 3-5: partial derivative (diag offset wrt fitness fn) fn operated on sample
		ellipsoid_jacob[3] = -1.f * ((x - p.offset(0)) * A) / length;
		ellipsoid_jacob[4] = -1.f * ((y - p.offset(1)) * B) / length;
		ellipsoid_jacob[5] = -1.f * ((z - p.offset(2)) * C) / length;
		// 6-8: partial derivative (off-diag offset wrt fitness fn) fn operated on sample
		ellipsoid_jacob[6] = -1.f * (((y - p.offset(1)) * A) + ((x - p.offset(0)) * B)) / length;
		ellipsoid_jacob[7] = -1.f * (((z - p.offset(2)) * A) + ((x - p.offset(0)) * C)) / length;
		ellipsoid_jacob[8] = -1.f * (((z - p.offset(2)) * B) + ((y - p.offset(1)) * C)) / length;

		for (int i = 0; i < 9; i++) {
			// compute JTJ
			for (int j = 0; j < 9; j++) {
				JTJ(i, j) += ellipsoid_jacob[i] * ellipsoid_jacob[j];
			}

			JTFI[i] += ellipsoid_jacob[i] * residual;
		}
	}


	//------------------------Levenberg-Marquardt-part-starts-here---------------------------------//
	// refer: http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm#Choice_of_damping_parameter
	float fit1_p[9] {p.offset(0), p.offset(1), p.offset(2), p.diag(0), p.diag(1), p.diag(2), p.offdiag(0), p.offdiag(1), p.offdiag(2)};
	float fit2_p[9];
	memcpy(fit2_p, fit1_p, sizeof(fit1_p));

	matrix::SquareMatrix<float, 9> JTJ2{JTJ};

	for (int i = 0; i < 9; i++) {
		JTJ(i, i) += result.gradient_damping;
		JTJ2(i, i) += result.gradient_damping / lma_damping;
	}


	if (!JTJ.I(JTJ)) {
		result.result = iteration_result::STATUS::FAILURE;
		return;
	}

	if (!JTJ2.I(JTJ2)) {
		result.result = iteration_result::STATUS::FAILURE;
		return;
	}

	for (int row = 0; row < 9; row++) {
		for (int col = 0; col < 9; col++) {
			fit1_p[row] -= JTFI[col] * JTJ(row, col);
			fit2_p[row] -= JTFI[col] * JTJ2(row, col);
		}
	}

	// Calculate mean squared residuals
	for (unsigned k = 0; k < samples_collected; k++) {
		float x = data[k](0);
		float y = data[k](1);
		float z = data[k](2);

		float A = (fit1_p[3] * (x - fit1_p[0])) + (fit1_p[6] * (y - fit1_p[1])) + (fit1_p[7] * (z - fit1_p[2]));
		float B = (fit1_p[6] * (x - fit1_p[0])) + (fit1_p[4] * (y - fit1_p[1])) + (fit1_p[8] * (z - fit1_p[2]));
		float C = (fit1_p[7] * (x - fit1_p[0])) + (fit1_p[8] * (y - fit1_p[1])) + (fit1_p[5] * (z - fit1_p[2]));
		float length = sqrtf(A * A + B * B + C * C);
		float residual = p.radius - length;
		fit1 += residual * residual;

		A = (fit2_p[3] * (x - fit2_p[0])) + (fit2_p[6] * (y - fit2_p[1])) + (fit2_p[7] * (z - fit2_p[2]));
		B = (fit2_p[6] * (x - fit2_p[0])) + (fit2_p[4] * (y - fit2_p[1])) + (fit2_p[8] * (z - fit2_p[2]));
		C = (fit2_p[7] * (x - fit2_p[0])) + (fit2_p[8] * (y - fit2_p[1])) + (fit2_p[5] * (z - fit2_p[2]));
		length = sqrtf(A * A + B * B + C * C);
		residual = p.radius - length;
		fit2 += residual * residual;
	}

	float fitness = result.cost;

	fit1 = sqrtf(fit1) / samples_collected;
	fit2 = sqrtf(fit2) / samples_collected;

	if (fit1 > result.cost && fit2 > result.cost) {
		result.gradient_damping *= lma_damping;

	} else if (fit2 < result.cost && fit2 < fit1) {
		result.gradient_damping /= lma_damping;
		memcpy(fit1_p, fit2_p, sizeof(fit1_p));
		fitness = fit2;

	} else if (fit1 < result.cost) {
		fitness = fit1;
	}

	//--------------------Levenberg-Marquardt-part-ends-here--------------------------------//
	if (PX4_ISFINITE(fitness) && fitness <= result.cost) {
		result.cost = fitness;

		p.offset(0)  = fit1_p[0];
		p.offset(1)  = fit1_p[1];
		p.offset(2)  = fit1_p[2];
		p.diag(0)    = fit1_p[3];
		p.diag(1)    = fit1_p[4];
		p.diag(2)    = fit1_p[5];
		p.offdiag(0) = fit1_p[6];
		p.offdiag(1) = fit1_p[7];
		p.offdiag(2) = fit1_p[8];

		result.result = iteration_result::STATUS::SUCCESS;

	} else {
		result.result = iteration_result::STATUS::FAILURE;
	}
}

int lm_fit(const matrix::Vector3f data[], unsigned samples_collected, sphere_params &params, bool full_ellipsoid)
{
	static constexpr int min_iterations = 10;
	static constexpr int max_iterations = 100;

	static constexpr float cost_threshold = 0.02f;
	static constexpr float step_threshold = 0.001f;

	iteration_result iter;
	iter.cost = 1e30f;
	iter.gradient_damping = 1;

	bool success = false;

	for (int i = 0; i < max_iterations; i++) {
		if (full_ellipsoid) {
			lm_ellipsoid_fit_iteration(data, samples_collected, params, iter);

			PX4_INFO("[%d] O:[%.5f, %.5f, %.5f], S:[%.3f, %.3f, %.3f], Result: %d, Cost: %.6f, Damping: %.6f\n",
				 i,
				 (double)params.offset(0), (double)params.offset(1), (double)params.offset(2),
				 (double)params.diag(0), (double)params.diag(1), (double)params.diag(2),
				 iter.result == iteration_result::STATUS::SUCCESS, (double)iter.cost, (double)iter.gradient_damping);

		} else {
			lm_sphere_fit_iteration(data, samples_collected, params, iter);

			PX4_INFO("[%d] O:[%.5f, %.5f, %.5f], Result: %d, Cost: %.6f, Damping: %.6f, Radius: %.4f\n",
				 i,
				 (double)params.offset(0), (double)params.offset(1), (double)params.offset(2),
				 iter.result == iteration_result::STATUS::SUCCESS, (double)iter.cost, (double)iter.gradient_damping,
				 (double)params.radius);
		}

		if (iter.result == iteration_result::STATUS::SUCCESS
		    && i > min_iterations && (iter.cost < cost_threshold || iter.gradient_damping < step_threshold)) {

			success = true;
			break;
		}
	}

	if (success) {
		return PX4_OK;
	}

	return 1;
}

