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

void inertial_filter_correct(float dt, float x[3], int i, float z, float w)
{
	float e = z - x[i];
	x[i] += e * w * dt;

	if (i == 0) {
		x[1] += e * w * w * dt;
		//x[2] += e * w * w * w * dt / 3.0; // ~ 0.0

	} else if (i == 1) {
		x[2] += e * w * w * dt;
	}
}
