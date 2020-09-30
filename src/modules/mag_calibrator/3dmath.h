/*
Copyright (C) 2013 Ben Dyer

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef _3DMATH_H_
#define _3DMATH_H_

#ifdef __cplusplus
extern "C" {
#endif

#define X 0
#define Y 1
#define Z 2
#define W 3

#ifndef absval
#define absval(x) ((x) < 0 ? -x : x)
#endif

#ifndef min
#define min(a,b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef max
#define max(a,b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef M_PI
#define M_PI ((real_t)3.14159265358979323846)
#define M_PI_2 (M_PI * 0.5)
#define M_PI_4 (M_PI * 0.25)
#endif

#define sqrt_inv(x) (1.0f / (float)sqrt((x)))
#define divide(a, b) ((a) / (b))
#define recip(a) (1.0f / (a))
#define _nassert(x)

static void matrix_cholesky_decomp_scale_f(unsigned int dim, float L[],
		const float A[], const float mul)
{
	assert(L && A && dim);
	_nassert((size_t)L % 8 == 0);
	_nassert((size_t)A % 8 == 0);

	/*
	9x9:
	900 mult
	72 div
	9 sqrt
	*/

	unsigned int i, j, kn, in, jn;

	for (i = 0, in = 0; i < dim; i++, in += dim) {
		L[i + 0] = (i == 0) ? sqrtf(A[i + in] * mul) : recip(L[0]) * (A[i] * mul);

		for (j = 1, jn = dim; j <= i; j++, jn += dim) {
			float s = 0;

			for (kn = 0; kn < j * dim; kn += dim) {
				s += L[i + kn] * L[j + kn];
			}

			L[i + jn] = (i == j) ? sqrtf(A[i + in] * mul - s) : recip(L[j + jn]) * (A[i + jn] * mul - s);
		}
	}
}

#ifdef __cplusplus
}
#endif

#endif
