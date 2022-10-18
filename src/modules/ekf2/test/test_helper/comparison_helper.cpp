/****************************************************************************
 *
 *   Copyright (C) 2022 PX4 Development Team. All rights reserved.
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

#include "comparison_helper.h"

float randf()
{
	return (float)rand() / (float)RAND_MAX;
}

SquareMatrix24f createRandomCovarianceMatrix24f()
{
	// Create a symmetric square matrix
	SquareMatrix24f P;

	for (int col = 0; col <= 23; col++) {
		for (int row = 0; row <= col; row++) {
			if (row == col) {
				P(row, col) = randf();

			} else {
				P(col, row) = P(row, col) = 2.0f * (randf() - 0.5f);
			}
		}
	}

	// Make it positive definite
	P = P.transpose() * P;

	return P;
}

DiffRatioReport computeDiffRatioVector24f(const Vector24f &v1, const Vector24f &v2)
{
	DiffRatioReport report = {};

	for (int row = 0; row < 24; row++) {
		float diff_fraction;

		if (fabsf(v1(row)) > FLT_EPSILON) {
			diff_fraction = fabsf(v2(row) - v1(row)) / fabsf(v1(row));

		} else if (fabsf(v2(row)) > FLT_EPSILON) {
			diff_fraction = fabsf(v2(row) - v1(row)) / fabsf(v2(row));

		} else {
			diff_fraction = 0.0f;
		}

		if (diff_fraction > report.max_diff_fraction) {
			report.max_diff_fraction = diff_fraction;
			report.max_row = row;
			report.max_v1 = v1(row);
			report.max_v2 = v2(row);
		}
	}

	return report;
}

DiffRatioReport computeDiffRatioSquareMatrix24f(const SquareMatrix24f &m1, const SquareMatrix24f &m2)
{
	DiffRatioReport report = {};

	for (int row = 0; row < 24; row++) {
		for (int col = 0; col < 24; col++) {
			float diff_fraction;

			if (fabsf(m1(row, col)) > FLT_EPSILON) {
				diff_fraction = fabsf(m2(row, col) - m1(row, col)) / fabsf(m1(row, col));

			} else if (fabsf(m2(row, col)) > FLT_EPSILON) {
				diff_fraction = fabsf(m2(row, col) - m1(row, col)) / fabsf(m2(row, col));

			} else {
				diff_fraction = 0.0f;
			}

			if (diff_fraction > report.max_diff_fraction) {
				report.max_diff_fraction = diff_fraction;
				report.max_row = row;
				report.max_v1 = m1(row, col);
				report.max_v2 = m2(row, col);
			}
		}
	}

	return report;
}
