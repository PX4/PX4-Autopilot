/*
 * inertial_filter.h
 *
 *   Copyright (C) 2013 Anton Babushkin. All rights reserved.
 *   Author: 	Anton Babushkin	<rk3dov@gmail.com>
 */

#include <stdbool.h>
#include <drivers/drv_hrt.h>

void inertial_filter_predict(float dt, float x[3], float acc);

void inertial_filter_correct(float e, float dt, float x[3], int i, float w);
