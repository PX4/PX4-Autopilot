/*
 * inertial_filter.c
 *
 *   Copyright (C) 2013 Anton Babushkin. All rights reserved.
 *   Author: 	Anton Babushkin	<rk3dov@gmail.com>
 */

#include "inertial_filter.h"

void inertial_filter_predict(float dt, float x[3])
{
	x[0] += x[1] * dt + x[2] * dt * dt / 2.0f;
	x[1] += x[2] * dt;
}

void inertial_filter_correct(float e, float dt, float x[3], int i, float w)
{
	float ewdt = w * dt;
	if (ewdt > 1.0f)
		ewdt = 1.0f;	// prevent over-correcting
	ewdt *= e;
	x[i] += ewdt;

	if (i == 0) {
		x[1] += w * ewdt;
		x[2] += w * w * ewdt / 3.0;

	} else if (i == 1) {
		x[2] += w * ewdt;
	}
}
