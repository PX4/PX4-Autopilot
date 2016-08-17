/*
 * inertial_filter.c
 *
 *   Copyright (C) 2013 Anton Babushkin. All rights reserved.
 *   Author: 	Anton Babushkin	<rk3dov@gmail.com>
 */

#include <math.h>

#include "inertial_filter.h"

void inertial_filter_predict(float dt, float x[2], float acc)
{
	if (isfinite(dt)) {
		if (!isfinite(acc)) {
			acc = 0.0f;
		}

		/*
		 * x[2]={pos,vel}
		 * 在单位时间有限的情况下，距离x[0]加上单位增量，包括速度x[1]和加速度acc两个部分；
		 * 而速度x[1]也加上单位增量，即加速度acc*dt。
		 * 当然acc非有限的情况则忽略acc作用，给acc置0，即无加速度情况。
 		 */
		x[0] += x[1] * dt + acc * dt * dt / 2.0f; 
		x[1] += acc * dt;
	}
}

void inertial_filter_correct(float e, float dt, float x[2], int i, float w)
{
	if (isfinite(e) && isfinite(w) && isfinite(dt)) {
		float ewdt = e * w * dt;
		/*
		 * 补偿中可以将e看作是误差，w是权重，位置增量由e,w,dt补偿，速度增量由e,w2,dt补偿
		 */
		x[i] += ewdt;

		if (i == 0) {
			x[1] += w * ewdt;
		}
	}
}
