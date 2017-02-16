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
Considering Polynomial Form:
y = a0 + a1.x + a2.x^2 + a3.x^3 + .... + an.x^n

and model for m number of samples will be
yi = a0 + a1.xi + a2.xi^2 + a3.xi^3 + .... + an.xi^n + ei where i =[0,m]
yi are the effect of cause xi and ei random fit error

in Vector Form:
Y = V.A + E
where V is Vandermonde matrix in x https://en.wikipedia.org/wiki/Vandermonde_matrix

so by Ordinary Least Squares derivation by minimising ∑(i=0..m)ei^2 here: http://www2.warwick.ac.uk/fac/soc/economics/staff/vetroeger/teaching/po906_week567.pdf

we get A = inv(transpose(V)*V)*(transpose(V)*Y)

we accumulate VTV and VTY as they are of fixed size and can be recursed upon
we can write VTV as
 __                                                                                                                        __
|      m                      x0+x1+...+xn                   x0^2+x1^2+...+xn^3   ..........  x0^m+x1^m+...+xn^m             |
|x0+x1+...+xn              x0^2+x1^2+...+xn^3                x0^3+x1^3+...+xn^3   ..........  x0^(m+1)+x1^(m+1)+...+xn^(m+1) |
|      .                            .                                  .                             .                       |
|      .                            .                                  .                             .                       |
|      .                            .                                  .                             .                       |
|x0^m+x1^m+...+xn^m     x0^(m+1)+x1^(m+1)+...+xn^(m+1)  x0^(m+2)+x1^(m+2)+...+xn^(m+2) ....  x0^(2m)+x1^(2m)+...+xn^(2m)     |
|__                                                                                                                        __|


and for VTY
 __            __
|  ∑(i=0..m)yi   |
| ∑(i=0..m)yi*xi |
|       .        |
|       .        |
|       .        |
|∑(i=0..m)yi*xi^n|
|__            __|

for more on expansion refer https://github.com/jgoppert/iekf_analysis/blob/master/Temperature%20Calibration.ipynb
*/

/*
Polygon linear fit
Author: Siddharth Bharat Purohit
*/

#pragma once
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <float.h>
#include <matrix/math.hpp>

#define DEBUG 0
#if DEBUG
#define PF_DEBUG(fmt, ...) printf(fmt, ##__VA_ARGS__);
#else
#define PF_DEBUG(fmt, ...)
#endif

template<size_t _forder>
class polyfitter
{
public:
	polyfitter() {}

	void update(double x, double y)
	{
		update_VTV(x);
		update_VTY(x, y);
	}

	bool fit(double res[])
	{
		//Do inverse of VTV
		matrix::SquareMatrix<double, _forder> IVTV;

		IVTV = _VTV.I();

		for (unsigned i = 0; i < _forder; i++) {
			for (int j = 0; j < _forder; j++) {
				PF_DEBUG("%.10f ", (double)IVTV(i, j));
			}

			PF_DEBUG("\n");
		}

		for (unsigned i = 0; i < _forder; i++) {
			res[i] = 0.0f;

			for (int j = 0; j < _forder; j++) {
				res[i] += IVTV(i, j) * (double)_VTY(j);
			}

			PF_DEBUG("%.10f ", res[i]);
		}

		return true;
	}

private:
	matrix::SquareMatrix<double, _forder> _VTV;
	matrix::Vector<double, _forder> _VTY;

	void update_VTY(double x, double y)
	{
		double temp = 1.0f;
		PF_DEBUG("O %.6f\n", (double)x);

		for (int8_t i = _forder - 1; i >= 0; i--) {
			_VTY(i) += y * temp;
			temp *= x;
			PF_DEBUG("%.6f ", (double)_VTY(i));
		}

		PF_DEBUG("\n");
	}

	void update_VTV(double x)
	{
		double temp = 1.0f;
		int8_t z;

		for (unsigned i = 0; i < _forder; i++) {
			for (int j = 0; j < _forder; j++) {
				PF_DEBUG("%.10f ", (double)_VTV(i, j));
			}

			PF_DEBUG("\n");
		}

		for (int8_t i = 2 * _forder - 2; i >= 0; i--) {
			if (i < _forder) {
				z = 0.0f;

			} else {
				z = i - _forder + 1;
			}

			for (int j = i - z; j >= z; j--) {
				unsigned row = j;
				unsigned col = i - j;
				_VTV(row, col) += (double)temp;
			}

			temp *= x;
		}
	}
};