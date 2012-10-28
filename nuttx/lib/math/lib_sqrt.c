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
#include <float.h>
#include <errno.h>
#include <apps/math.h>

static float __sqrt_approx(float x) {
	int32_t i;

	// floats + bit manipulation = +inf fun!
	i = *((int32_t*) &x);
	i = 0x1FC00000 + (i >> 1);
	x = *((float*) &i);

	return x;
}

float sqrtf(float x) {
	float y;

	// filter out invalid/trivial inputs
	if (x < 0.0) { errno = EDOM; return NAN; }
	if (isnan(x)) return NAN;
	if (isinf(x)) return INFINITY;
	if (x == 0.0) return 0.0;

	// guess square root (using bit manipulation)
	y = __sqrt_approx(x);

	// perform three iterations of approximation
	// this number (3) is definitely optimal
	y = 0.5 * (y + x / y);
	y = 0.5 * (y + x / y);
	y = 0.5 * (y + x / y);

	return y;
}

double sqrt(double x) {
	long double y, y1;
	
	// filter out invalid/trivial inputs
	if (x < 0.0) { errno = EDOM; return NAN; }
	if (isnan(x)) return NAN;
	if (isinf(x)) return INFINITY;
	if (x == 0.0) return 0.0;

	// guess square root (using bit manipulation)
	y = __sqrt_approx(x);

	// perform four iterations of approximation
	// this number (4) is definitely optimal
	y = 0.5 * (y + x / y);
	y = 0.5 * (y + x / y);
	y = 0.5 * (y + x / y);
	y = 0.5 * (y + x / y);

	// if guess was terribe (out of range of float)
	// repeat approximation until convergence
	if (y * y < x - 1.0 || y * y > x + 1.0) {
		y1 = -1.0;
		while (y != y1) {
			y1 = y;
			y = 0.5 * (y + x / y);
		}
	}

	return y;
}

long double sqrtl(long double x) {
	long double y, y1;

	// filter out invalid/trivial inputs
	if (x < 0.0) { errno = EDOM; return NAN; }
	if (isnan(x)) return NAN;
	if (isinf(x)) return INFINITY;
	if (x == 0.0) return 0.0;

	// guess square root (using bit manipulation)
	y = __sqrt_approx(x);

	// perform four iterations of approximation
	// this number (4) is definitely optimal
	y = 0.5 * (y + x / y);
	y = 0.5 * (y + x / y);
	y = 0.5 * (y + x / y);
	y = 0.5 * (y + x / y);

	// if guess was terribe (out of range of float)
	// repeat approximation until convergence
	if (y * y < x - 1.0 || y * y > x + 1.0) {
		y1 = -1.0;
		while (y != y1) {
			y1 = y;
			y = 0.5 * (y + x / y);
		}
	}

	return y;
}
