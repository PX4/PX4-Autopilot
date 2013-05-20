/*
 * kalman_filter_inertial.c
 *
 *   Copyright (C) 2013 Anton Babushkin. All rights reserved.
 *   Author: 	Anton Babushkin	<rk3dov@gmail.com>
 */

#include "kalman_filter_inertial.h"

void kalman_filter_inertial_predict(float dt, float x[3]) {
	x[0] += x[1] * dt + x[2] * dt * dt / 2.0f;
	x[1] += x[2] * dt;
}

void kalman_filter_inertial2_update(float x[3], float z[2], float k[3][2], bool use[2]) {
	float y[2];
	// y = z - H x
	y[0] = z[0] - x[0];
	y[1] = z[1] - x[2];
	// x = x + K * y
	for (int i = 0; i < 3; i++) {		// Row
		for (int j = 0; j < 2; j++) {	// Column
			if (use[j])
				x[i] += k[i][j] * y[j];
		}
	}
}

void kalman_filter_inertial3_update(float x[3], float z[3], float k[3][3], bool use[3]) {
	float y[2];
	// y = z - H x
	y[0] = z[0] - x[0];
	y[1] = z[1] - x[1];
	y[2] = z[2] - x[2];
	// x = x + K * y
	for (int i = 0; i < 3; i++) {		// Row
		for (int j = 0; j < 3; j++) {	// Column
			if (use[j])
				x[i] += k[i][j] * y[j];
		}
	}
}
