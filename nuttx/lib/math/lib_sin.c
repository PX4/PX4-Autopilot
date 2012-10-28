/*
 * Copyright (C) 2009-2011 Nick Johnson <nickbjohnson4224 at gmail.com>
 * 
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <stdint.h>
#include <unistd.h>
#include <float.h>
#include <apps/math.h>

static float _flt_inv_fact[] = {
	1.0 / 1.0,			// 1 / 1!
	1.0 / 6.0,			// 1 / 3!
	1.0 / 120.0,		// 1 / 5!
	1.0 / 5040.0,		// 1 / 7!
	1.0 / 362880.0,		// 1 / 9!
	1.0 / 39916800.0,	// 1 / 11!
};

float sinf(float x) {
	float x_squared;
	float sin_x;
	size_t i;
	
	/* move x to [-pi, pi) */
	x = fmodf(x, 2 * M_PI);
	if (x >= M_PI) x -= 2 * M_PI;
	if (x < -M_PI) x += 2 * M_PI;

	/* move x to [-pi/2, pi/2) */
	if (x >= M_PI_2) x =  M_PI - x;
	if (x < -M_PI_2) x = -M_PI - x;

	x_squared = x * x;
	sin_x = 0.0;

	/* perform Taylor series approximation for sin(x) with six terms */
	for (i = 0; i < 6; i++) {
		if (i % 2 == 0) {
			sin_x += x * _flt_inv_fact[i];
		}
		else {
			sin_x -= x * _flt_inv_fact[i];
		}

		x *= x_squared;
	}

	return sin_x;
}
	
static double _dbl_inv_fact[] = {
	1.0 / 1.0,						// 1 / 1!
	1.0 / 6.0,						// 1 / 3!
	1.0 / 120.0,					// 1 / 5!
	1.0 / 5040.0,					// 1 / 7!
	1.0 / 362880.0,					// 1 / 9!
	1.0 / 39916800.0,				// 1 / 11!
	1.0 / 6227020800.0,				// 1 / 13!
	1.0 / 1307674368000.0,			// 1 / 15!
	1.0 / 355687428096000.0,		// 1 / 17!
	1.0 / 121645100408832000.0,		// 1 / 19!
};

double sin(double x) {
	double x_squared;
	double sin_x;
	size_t i;
	
	/* move x to [-pi, pi) */
	x = fmod(x, 2 * M_PI);
	if (x >= M_PI) x -= 2 * M_PI;
	if (x < -M_PI) x += 2 * M_PI;

	/* move x to [-pi/2, pi/2) */
	if (x >= M_PI_2) x =  M_PI - x;
	if (x < -M_PI_2) x = -M_PI - x;

	x_squared = x * x;
	sin_x = 0.0;

	/* perform Taylor series approximation for sin(x) with ten terms */
	for (i = 0; i < 10; i++) {
		if (i % 2 == 0) {
			sin_x += x * _dbl_inv_fact[i];
		}
		else {
			sin_x -= x * _dbl_inv_fact[i];
		}

		x *= x_squared;
	}

	return sin_x;
}

static long double _ldbl_inv_fact[] = {
	1.0 / 1.0,						// 1 / 1!
	1.0 / 6.0,						// 1 / 3!
	1.0 / 120.0,					// 1 / 5!
	1.0 / 5040.0,					// 1 / 7!
	1.0 / 362880.0,					// 1 / 9!
	1.0 / 39916800.0,				// 1 / 11!
	1.0 / 6227020800.0,				// 1 / 13!
	1.0 / 1307674368000.0,			// 1 / 15!
	1.0 / 355687428096000.0,		// 1 / 17!
	1.0 / 121645100408832000.0,		// 1 / 19!
};

long double sinl(long double x) {
	long double x_squared;
	long double sin_x;
	size_t i;
	
	/* move x to [-pi, pi) */
	x = fmodl(x, 2 * M_PI);
	if (x >= M_PI) x -= 2 * M_PI;
	if (x < -M_PI) x += 2 * M_PI;

	/* move x to [-pi/2, pi/2) */
	if (x >= M_PI_2) x =  M_PI - x;
	if (x < -M_PI_2) x = -M_PI - x;

	x_squared = x * x;
	sin_x = 0.0;

	/* perform Taylor series approximation for sin(x) with ten terms */
	for (i = 0; i < 10; i++) {
		if (i % 2 == 0) {
			sin_x += x * _ldbl_inv_fact[i];
		}
		else {
			sin_x -= x * _ldbl_inv_fact[i];
		}

		x *= x_squared;
	}

	return sin_x;
}

