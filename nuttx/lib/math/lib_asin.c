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

float asinf(float x) {
	long double y, y_sin, y_cos;

	y = 0;

	while (1) {
		y_sin = sinf(y);
		y_cos = cosf(y);

		if (y > M_PI_2 || y < -M_PI_2) {
			y = fmodf(y, M_PI);
		}

		if (y_sin + DBL_EPSILON >= x && y_sin - DBL_EPSILON <= x) {
			break;
		}

		y = y - (y_sin - x) / y_cos;
	}

	return y;
}

double asin(double x) {
	long double y, y_sin, y_cos;

	y = 0;

	while (1) {
		y_sin = sin(y);
		y_cos = cos(y);

		if (y > M_PI_2 || y < -M_PI_2) {
			y = fmod(y, M_PI);
		}

		if (y_sin + DBL_EPSILON >= x && y_sin - DBL_EPSILON <= x) {
			break;
		}

		y = y - (y_sin - x) / y_cos;
	}

	return y;
}

long double asinl(long double x) {
	long double y, y_sin, y_cos;

	y = 0;

	while (1) {
		y_sin = sinl(y);
		y_cos = cosl(y);

		if (y > M_PI_2 || y < -M_PI_2) {
			y = fmodl(y, M_PI);
		}

		if (y_sin + LDBL_EPSILON >= x && y_sin - LDBL_EPSILON <= x) {
			break;
		}

		y = y - (y_sin - x) / y_cos;
	}

	return y;
}

