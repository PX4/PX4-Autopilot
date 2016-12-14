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

int polyfitter::init(uint8_t order)
{
	_forder = order + 1;
	VTV = new float[_forder * _forder];

	if (VTV == NULL) {
		return -1;
	}

	VTY = new float[_forder];

	if (VTY == NULL) {
		return -1;
	}

	memset(VTV, 0, sizeof(float)*_forder * _forder);
	memset(VTY, 0, sizeof(float)*_forder);
	return 0;
}

void polyfitter::update(float x, float y)
{
	update_VTV(x);
	update_VTY(x, y);
}

void polyfitter::update_VTY(float x, float y)
{
	float temp = 1.0f;

	for (int8_t i = _forder - 1; i >= 0; i--) {
		VTY[i] += y * temp;
		temp *= x;
	}
}


void polyfitter::update_VTV(float x)
{
	float temp = 1.0f;
	int8_t z;

	for (int8_t i = 2 * _forder - 2; i >= 0; i--) {
		if (i < _forder) {
			z = 0.0f;

		} else {
			z = i - _forder + 1;
		}

		for (int8_t j = i - z; j >= z; j--) {
			uint8_t row = j;
			uint8_t col = i - j;
			VTV[row * _forder  + col] += temp;
		}

		temp *= x;
	}
}

bool polyfitter::fit(float res[])
{
	//Do inverse of VTV
	float *IVTV = new float[_forder * _forder];

	if (VTV == NULL) {
		return false;
	}

	if (inverse(VTV, IVTV, _forder)) {
		for (uint8_t i = 0; i < _forder; i++) {
			res[i] = 0.0f;

			for (int j = 0; j < _forder; j++) {
				res[i] += IVTV[i * _forder + j] * VTY[j];
			}
		}

		return true;
	}

	return false;
}