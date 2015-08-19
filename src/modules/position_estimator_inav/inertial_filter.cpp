/**
 *	 @file: inertial_filter.cpp
 *
 *   Copyright (C) 2013-2015 Anton Babushkin. All rights reserved.
 *   @author Anton Babushkin <rk3dov@gmail.com>
 */

#include "inertial_filter.h"

InertialFilter::InertialFilter()
{};

void InertialFilter::inertial_filter_predict(float dt, Vector2f &x, float acc)
{
	if (isfinite(dt)) {
		if (!isfinite(acc)) {
			acc = 0.0f;
		}

		x[0] += x[1] * dt + acc * dt * dt / 2.0f;
		x[1] += acc * dt;
	}
}

void InertialFilter::inertial_filter_correct(float e, float dt, Vector2f &x, int i, float w)
{
	if (isfinite(e) && isfinite(w) && isfinite(dt)) {
		float ewdt = e * w * dt;
		x[i] += ewdt;

		if (i == 0) {
			x[1] += w * ewdt;
		}
	}
}
