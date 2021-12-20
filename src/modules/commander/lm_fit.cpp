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
#include <lib/mathlib/mathlib.h>

#define LM_MAX_SIZE 9
enum class FIT_TYPE {

	SPHERE = 0,
	ELLIPSOID = 1
};

struct iteration_result {
	float gradient_damping;
	float cost;
	enum class STATUS {
		SUCCESS, FAILURE
	} result = STATUS::SUCCESS;
};

void compute_jacob(FIT_TYPE type, const float x, const float y, const float z,
		   const sphere_params &p, float* jacob, float& residual) {

	float x_unbiased = x - p.offset(0);
	float y_unbiased = y - p.offset(1);
	float z_unbiased = z - p.offset(2);
	float A = p.diag(0)    * x_unbiased + p.offdiag(0) * y_unbiased + p.offdiag(1) * z_unbiased;
	float B = p.offdiag(0) * x_unbiased + p.diag(1)    * y_unbiased + p.offdiag(2) * z_unbiased;
	float C = p.offdiag(1) * x_unbiased + p.offdiag(2) * y_unbiased + p.diag(2)    * z_unbiased;

	float length = sqrtf(A * A + B * B + C * C);
	residual =  p.radius - length;

	if (jacob != nullptr) {
		switch(type) {
		case FIT_TYPE::SPHERE:
			jacob[0] = 1.0f;
			// 1-3: partial derivative (offsets wrt fitness fn) fn operated on sample
			jacob[1] = (p.diag(0)    * A + p.offdiag(0) * B + p.offdiag(1) * C) / length;
			jacob[2] = (p.offdiag(0) * A + p.diag(1)    * B + p.offdiag(2) * C) / length;
			jacob[3] = (p.offdiag(1) * A + p.offdiag(2) * B + p.diag(2)    * C) / length;
			break;
		case FIT_TYPE::ELLIPSOID:
		default:
			// 0-2: partial derivative (offsets wrt fitness fn) fn operated on sample
			jacob[0] = (p.diag(0)    * A + p.offdiag(0) * B + p.offdiag(1) * C) / length;
			jacob[1] = (p.offdiag(0) * A + p.diag(1)    * B + p.offdiag(2) * C) / length;
			jacob[2] = (p.offdiag(1) * A + p.offdiag(2) * B + p.diag(2)    * C) / length;
			// 3-5: partial derivative (diag offset wrt fitness fn) fn operated on sample
			jacob[3] = -x_unbiased * A / length;
			jacob[4] = -y_unbiased * B / length;
			jacob[5] = -z_unbiased * C / length;
			// 6-8: partial derivative (off-diag offset wrt fitness fn) fn operated on sample
			jacob[6] = -(y_unbiased * A + x_unbiased * B) / length;
			jacob[7] = -(z_unbiased * A + x_unbiased * C) / length;
			jacob[8] = -(z_unbiased * B + y_unbiased * C) / length;
		}
	}
}

int lm_size(FIT_TYPE type) {
	int size = 0;
	switch(type) {
	case FIT_TYPE::SPHERE:
		size = 4;
		break;
	case FIT_TYPE::ELLIPSOID:
	default:
		size = 9;
	}
	return math::min(LM_MAX_SIZE,size);
}

void lm_param_to_array(FIT_TYPE type, const sphere_params &params, float param_array[]) {

	switch(type) {
	case FIT_TYPE::SPHERE:
		param_array[0] = params.radius;
		param_array[1] = params.offset(0);
		param_array[2] = params.offset(1);
		param_array[3] = params.offset(2);
		break;
	case FIT_TYPE::ELLIPSOID:
	default:
		param_array[0] = params.offset(0);
		param_array[1] = params.offset(1);
		param_array[2] = params.offset(2);
		param_array[3] = params.diag(0);
		param_array[4] = params.diag(1);
		param_array[5] = params.diag(2);
		param_array[6] = params.offdiag(0);
		param_array[7] = params.offdiag(1);
		param_array[8] = params.offdiag(2);
	}
}

void lm_array_to_param(FIT_TYPE type, const float param_array[], sphere_params &params) {

	switch(type) {
	case FIT_TYPE::SPHERE:
		params.radius = param_array[0];
		params.offset(0) = param_array[1];
		params.offset(1) = param_array[2];
		params.offset(2) = param_array[3];
		break;
	case FIT_TYPE::ELLIPSOID:
	default:
		params.offset(0) = param_array[0];
		params.offset(1) = param_array[1];
		params.offset(2) = param_array[2];
		params.diag(0) = param_array[3];
		params.diag(1) = param_array[4];
		params.diag(2) = param_array[5];
		params.offdiag(0) = param_array[6];
		params.offdiag(1) = param_array[7];
		params.offdiag(2) = param_array[8];
	}
}

void lm_fit_iteration(const float x[], const float y[], const float z[],
			     unsigned int samples_collected, sphere_params &params, iteration_result &result,
			     FIT_TYPE type)
{
	// Run fit using Levenberg Marquardt LSq Fit
	const float lma_damping = 10.f;
	float fitness = result.cost;
	float fit1 = 0.f;
	float fit2 = 0.f;

	int max_size = lm_size(type);

	matrix::SquareMatrix<float, LM_MAX_SIZE> JTJ;
	JTJ.setIdentity();
	float JTFI[LM_MAX_SIZE] {};
	float residual = 0.0f;
	float jacob[LM_MAX_SIZE]{0.f};

	// Gauss Newton Part common for all kind of extensions including LM
	for (uint16_t k = 0; k < samples_collected; k++) {

		compute_jacob(type,x[k] ,y[k] ,z[k] ,params,jacob,residual);

		for (uint8_t i = 0; i < max_size; i++) {
			// compute JTJ
			for (uint8_t j = 0; j < max_size; j++) {
				JTJ(i, j) += jacob[i] * jacob[j];
			}

			JTFI[i] += jacob[i] * residual;
		}
	}

	//------------------------Levenberg-Marquardt-part-starts-here---------------------------------//
	// refer: http://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm#Choice_of_damping_parameter
	sphere_params params1 = params;
	sphere_params params2 = params;
	float fit1_params[LM_MAX_SIZE];
	lm_param_to_array(type, params, fit1_params);
	float fit2_params[LM_MAX_SIZE];
	memcpy(fit2_params, fit1_params, sizeof(fit1_params));

	JTJ = (JTJ + JTJ.transpose()) * 0.5f; // fix numerical issues
	matrix::SquareMatrix<float, LM_MAX_SIZE> JTJ2 = JTJ;

	for (uint8_t i = 0; i < max_size; i++) {
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

	for (uint8_t row = 0; row < max_size; row++) {
		for (uint8_t col = 0; col < max_size; col++) {
			fit1_params[row] -= JTFI[col] * JTJ(row, col);
			fit2_params[row] -= JTFI[col] * JTJ2(row, col);
		}
	}

	// Calculate mean squared residuals
	for (uint16_t k = 0; k < samples_collected; k++) {
		lm_array_to_param(type,fit1_params,params1);
		float residual1;
		compute_jacob(type,x[k] ,y[k] ,z[k], params1, nullptr, residual1);
		fit1 += residual1 * residual1;

		lm_array_to_param(type,fit2_params,params2);
		float residual2;
		compute_jacob(type,x[k] ,y[k] ,z[k], params2, nullptr, residual2);
		fit2 += residual2 * residual2;
	}

	fit1 = sqrtf(fit1) / samples_collected;
	fit2 = sqrtf(fit2) / samples_collected;

	if (fit1 > result.cost && fit2 > result.cost) {
		result.gradient_damping *= lma_damping;

	} else if (fit2 < result.cost && fit2 < fit1) {
		result.gradient_damping /= lma_damping;
		memcpy(fit1_params, fit2_params, sizeof(fit1_params));
		fitness = fit2;

	} else if (fit1 < result.cost) {
		fitness = fit1;
	}

	//--------------------Levenberg-Marquardt-part-ends-here--------------------------------//
	if (PX4_ISFINITE(fitness) && fitness <= result.cost) {
		result.cost = fitness;
		lm_array_to_param(type,fit1_params,params);
		result.result = iteration_result::STATUS::SUCCESS;

	} else {
		result.result = iteration_result::STATUS::FAILURE;
	}
}

int lm_mag_fit(const float x[], const float y[], const float z[], unsigned int samples_collected, sphere_params &params,
	       bool full_ellipsoid)
{

	const int max_iterations = 100;
	const int min_iterations = 10;
	const float cost_threshold = 0.01;
	const float step_threshold = 0.001;

	const float min_radius = 0.2;
	const float max_radius = 0.7;

	iteration_result iter;
	iter.cost = 1e30f;
	iter.gradient_damping = 1;

	bool success = false;

	for (int i = 0; i < max_iterations; i++) {
		if (full_ellipsoid) {
			lm_fit_iteration(x, y, z, samples_collected, params, iter, FIT_TYPE::ELLIPSOID);

		} else {
			lm_fit_iteration(x, y, z, samples_collected, params, iter, FIT_TYPE::SPHERE);
		}

		if (iter.result == iteration_result::STATUS::SUCCESS
		    && min_radius < params.radius && params.radius < max_radius
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

