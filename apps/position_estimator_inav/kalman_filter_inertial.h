/*
 * kalman_filter_inertial.h
 *
 *   Copyright (C) 2013 Anton Babushkin. All rights reserved.
 *   Author: 	Anton Babushkin	<rk3dov@gmail.com>
 */

#include <stdbool.h>

void kalman_filter_inertial_predict(float dt, float x[3]);

void kalman_filter_inertial_update(float x[3], float z[2], float k[3][2], bool use[2]);
