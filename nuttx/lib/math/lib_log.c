/*
 * Copyright (C) 2009, 2010 Nick Johnson <nickbjohnson4224 at gmail.com>
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

float logf(float x) {
	float y, y_old, ey, epsilon;

	y = 0.0;
	y_old = 1.0;
	epsilon = FLT_EPSILON;

	while (y > y_old + epsilon || y < y_old - epsilon) {
		y_old = y;
		ey = exp(y);
		y -= (ey - x) / ey;

		if (y > 700.0) {
			y = 700.0;
		}
		if (y < -700.0) {
			y = -700.0;
		}

		epsilon = (fabs(y) > 1.0) ? fabs(y) * FLT_EPSILON : FLT_EPSILON;
	}

	if (y == 700.0) {
		return INFINITY;
	}
	if (y == -700.0) {
		return INFINITY;
	}

	return y;
}

double log(double x) {
	double y, y_old, ey, epsilon;

	y = 0.0;
	y_old = 1.0;
	epsilon = DBL_EPSILON;

	while (y > y_old + epsilon || y < y_old - epsilon) {
		y_old = y;
		ey = exp(y);
		y -= (ey - x) / ey;

		if (y > 700.0) {
			y = 700.0;
		}
		if (y < -700.0) {
			y = -700.0;
		}

		epsilon = (fabs(y) > 1.0) ? fabs(y) * DBL_EPSILON : DBL_EPSILON;
	}

	if (y == 700.0) {
		return INFINITY;
	}
	if (y == -700.0) {
		return INFINITY;
	}

	return y;
}

long double logl(long double x) {
	long double y, y_old, ey, epsilon;

	y = 0.0;
	y_old = 1.0;
	epsilon = 1e-6; //fixme

	while (y > y_old + epsilon || y < y_old - epsilon) {
		y_old = y;
		ey = expl(y);
		y -= (ey - x) / ey;

		if (y > 700.0) {
			y = 700.0;
		}
		if (y < -700.0) {
			y = -700.0;
		}
	}

	if (y == 700.0) {
		return INFINITY;
	}
	if (y == -700.0) {
		return INFINITY;
	}

	return y;
}

