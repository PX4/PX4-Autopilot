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

#ifndef EKF_COMPARISON_HELPER
#define EKF_COMPARISON_HELPER

#include "EKF/ekf.h"

typedef matrix::Vector<float, 24> Vector24f;
typedef matrix::SquareMatrix<float, 24> SquareMatrix24f;
template<int ... Idxs>
using SparseVector24f = matrix::SparseVectorf<24, Idxs...>;

struct DiffRatioReport {
	float max_diff_fraction;
	float max_row;
	float max_v1;
	float max_v2;
};

float randf();

// Create a symmetrical positive dfinite matrix with off diagonals between -1 and 1 and diagonals between 0 and 1
SquareMatrix24f createRandomCovarianceMatrix24f();

// Find largest element-wise difference as a fraction of v1 or v2
DiffRatioReport computeDiffRatioVector24f(const Vector24f &v1, const Vector24f &v2);
DiffRatioReport computeDiffRatioSquareMatrix24f(const SquareMatrix24f &m1, const SquareMatrix24f &m2);
#endif
