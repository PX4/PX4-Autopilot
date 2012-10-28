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

#include <apps/math.h>
#include <float.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>

#define M_E2 	(M_E * M_E)
#define M_E4 	(M_E2 * M_E2)
#define M_E8	(M_E4 * M_E4)
#define M_E16	(M_E8 * M_E8)
#define M_E32	(M_E16 * M_E16)
#define M_E64	(M_E32 * M_E32)
#define M_E128	(M_E64 * M_E64)
#define M_E256	(M_E128 * M_E128)
#define M_E512	(M_E256 * M_E256)
#define M_E1024	(M_E512 * M_E512)

static double _expi_square_tbl[11] = {
	M_E,		// e^1
	M_E2,		// e^2
	M_E4,		// e^4
	M_E8,		// e^8
	M_E16,		// e^16
	M_E32,		// e^32
	M_E64,		// e^64
	M_E128,		// e^128
	M_E256,		// e^256
	M_E512,		// e^512
	M_E1024,	// e^1024
};

static double _expi(size_t n) {
	size_t i;
	double val;

	if (n > 1024) {
		return INFINITY;
	}

	val = 1.0;

	for (i = 0; n; i++) {
		if (n & (1 << i)) {
			n &= ~(1 << i);
			val *= _expi_square_tbl[i];
		}
	}

	return val;
}

static float _flt_inv_fact[] = {
	1.0 / 1.0,			// 1/0!
	1.0 / 1.0,			// 1/1!
	1.0 / 2.0,			// 1/2!
	1.0 / 6.0,			// 1/3!
	1.0 / 24.0,			// 1/4!
	1.0 / 120.0,		// 1/5!
	1.0 / 720.0,		// 1/6!
	1.0 / 5040.0,		// 1/7!
	1.0 / 40320.0,		// 1/8!
	1.0 / 362880.0,		// 1/9!
	1.0 / 3628800.0,	// 1/10!
};	

float expf(float x) {
	size_t int_part;
	bool invert;
	float value;
	float x0;
	size_t i;

	if (x == 0) {
		return 1;
	}
	else if (x < 0) {
		invert = true;
		x = -x;
	}
	else {
		invert = false;
	}

	/* extract integer component */
	int_part = (size_t) x;

	/* set x to fractional component */
	x -= (float) int_part;

	/* perform Taylor series approximation with eleven terms */
	value = 0.0;
	x0 = 1.0;
	for (i = 0; i < 10; i++) {
		value += x0 * _flt_inv_fact[i];
		x0 *= x;
	}
	
	/* multiply by exp of the integer component */
	value *= _expi(int_part);

	if (invert) {
		return (1.0 / value);
	}
	else {
		return value;
	}
}

static double _dbl_inv_fact[] = {
	1.0 / 1.0,					// 1 / 0!
	1.0 / 1.0,					// 1 / 1!
	1.0 / 2.0,					// 1 / 2!
	1.0 / 6.0,					// 1 / 3!
	1.0 / 24.0,					// 1 / 4!
	1.0 / 120.0,				// 1 / 5!
	1.0 / 720.0,				// 1 / 6!
	1.0 / 5040.0,				// 1 / 7!
	1.0 / 40320.0,				// 1 / 8!
	1.0 / 362880.0,				// 1 / 9!
	1.0 / 3628800.0,			// 1 / 10!
	1.0 / 39916800.0,			// 1 / 11!
	1.0 / 479001600.0,			// 1 / 12!
	1.0 / 6227020800.0,			// 1 / 13!
	1.0 / 87178291200.0,		// 1 / 14!
	1.0 / 1307674368000.0,		// 1 / 15!
	1.0 / 20922789888000.0,		// 1 / 16!
	1.0 / 355687428096000.0,	// 1 / 17!
	1.0 / 6402373705728000.0,	// 1 / 18!
};

double exp(double x) {
	size_t int_part;
	bool invert;
	double value;
	double x0;
	size_t i;

	if (x == 0) {
		return 1;
	}
	else if (x < 0) {
		invert = true;
		x = -x;
	}
	else {
		invert = false;
	}

	/* extract integer component */
	int_part = (size_t) x;

	/* set x to fractional component */
	x -= (double) int_part;

	/* perform Taylor series approximation with nineteen terms */
	value = 0.0;
	x0 = 1.0;
	for (i = 0; i < 19; i++) {
		value += x0 * _dbl_inv_fact[i];
		x0 *= x;
	}
	
	/* multiply by exp of the integer component */
	value *= _expi(int_part);

	if (invert) {
		return (1.0 / value);
	}
	else {
		return value;
	}
}

static long double _ldbl_inv_fact[] = {
	1.0 / 1.0,					// 1 / 0!
	1.0 / 1.0,					// 1 / 1!
	1.0 / 2.0,					// 1 / 2!
	1.0 / 6.0,					// 1 / 3!
	1.0 / 24.0,					// 1 / 4!
	1.0 / 120.0,				// 1 / 5!
	1.0 / 720.0,				// 1 / 6!
	1.0 / 5040.0,				// 1 / 7!
	1.0 / 40320.0,				// 1 / 8!
	1.0 / 362880.0,				// 1 / 9!
	1.0 / 3628800.0,			// 1 / 10!
	1.0 / 39916800.0,			// 1 / 11!
	1.0 / 479001600.0,			// 1 / 12!
	1.0 / 6227020800.0,			// 1 / 13!
	1.0 / 87178291200.0,		// 1 / 14!
	1.0 / 1307674368000.0,		// 1 / 15!
	1.0 / 20922789888000.0,		// 1 / 16!
	1.0 / 355687428096000.0,	// 1 / 17!
	1.0 / 6402373705728000.0,	// 1 / 18!
};

long double expl(long double x) {
	size_t int_part;
	bool invert;
	long double value;
	long double x0;
	size_t i;

	if (x == 0) {
		return 1;
	}
	else if (x < 0) {
		invert = true;
		x = -x;
	}
	else {
		invert = false;
	}

	/* extract integer component */
	int_part = (size_t) x;

	/* set x to fractional component */
	x -= (long double) int_part;

	/* perform Taylor series approximation with nineteen terms */
	value = 0.0;
	x0 = 1.0;
	for (i = 0; i < 19; i++) {
		value += x0 * _ldbl_inv_fact[i];
		x0 *= x;
	}
	
	/* multiply by exp of the integer component */
	value *= _expi(int_part);

	if (invert) {
		return (1.0 / value);
	}
	else {
		return value;
	}
}
