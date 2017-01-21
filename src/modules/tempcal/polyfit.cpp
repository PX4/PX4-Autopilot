/****************************************************************************
 *
 *   Copyright (c) 2015-2016 PX4 Development Team. All rights reserved.
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
/*
Polygon linear fit
Author: Siddharth Bharat Purohit
*/
#include "polyfit.h"
#include "matrix_alg.h"

#define DEBUG 0
#if DEBUG
#define PF_DEBUG(fmt, ...) printf(fmt, ##__VA_ARGS__);
#else
#define PF_DEBUG(fmt, ...)
#endif

int polyfitter::init(uint8_t order)
{
	_forder = order + 1;
	VTV = new double[_forder * _forder];

	if (VTV == NULL) {
		return -1;
	}

	VTY = new double[_forder];

	if (VTY == NULL) {
		return -1;
	}

	memset(VTV, 0, sizeof(double)*_forder * _forder);
	memset(VTY, 0, sizeof(double)*_forder);
	return 0;
}

void polyfitter::update(double x, double y)
{
	update_VTV(x);
	update_VTY(x, y);
}

void polyfitter::update_VTY(double x, double y)
{
	double temp = 1.0f;
	PF_DEBUG("O %.6f\n", (double)x);

	for (int8_t i = _forder - 1; i >= 0; i--) {
		VTY[i] += y * temp;
		temp *= x;
		PF_DEBUG("%.6f ", (double)VTY[i]);
	}

	PF_DEBUG("\n");
}


void polyfitter::update_VTV(double x)
{
	double temp = 1.0f;
	int8_t z;

	for (uint8_t i = 0; i < _forder; i++) {
		for (int j = 0; j < _forder; j++) {
			PF_DEBUG("%.10f ", (double)VTV[i * _forder + j]);
		}

		PF_DEBUG("\n");
	}

	for (int8_t i = 2 * _forder - 2; i >= 0; i--) {
		if (i < _forder) {
			z = 0.0f;

		} else {
			z = i - _forder + 1;
		}

		for (int8_t j = i - z; j >= z; j--) {
			uint8_t row = j;
			uint8_t col = i - j;
			VTV[row * _forder  + col] += (double)temp;
		}

		temp *= x;
	}
}

bool polyfitter::fit(double res[])
{
	//Do inverse of VTV
	double *IVTV = new double[_forder * _forder];

	if (VTV == NULL) {
		return false;
	}

	if (inverse4x4(VTV, IVTV)) {
		for (uint8_t i = 0; i < _forder; i++) {
			for (int j = 0; j < _forder; j++) {
				PF_DEBUG("%.10f ", (double)IVTV[i * _forder + j]);
			}

			PF_DEBUG("\n");
		}

		for (uint8_t i = 0; i < _forder; i++) {
			res[i] = 0.0f;

			for (int j = 0; j < _forder; j++) {
				res[i] += IVTV[i * _forder + j] * (double)VTY[j];
			}

			PF_DEBUG("%.10f ", res[i]);
		}

		return true;
	}

	return false;
}