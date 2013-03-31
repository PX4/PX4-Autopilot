/*
 * acceleration_transform.h
 *
 *   Copyright (C) 2013 Anton Babushkin. All rights reserved.
 *   Author: 	Anton Babushkin	<rk3dov@gmail.com>
 */

#ifndef ACCELERATION_TRANSFORM_H_
#define ACCELERATION_TRANSFORM_H_

#include <unistd.h>

void acceleration_correct(float accel_corr[3], int16_t accel_raw[3],
		float accel_T[3][3], int16_t accel_offs[3]);

int calculate_calibration_values(int16_t accel_raw_ref[6][3],
		float accel_T[3][3], int16_t accel_offs[3], float g);

#endif /* ACCELERATION_TRANSFORM_H_ */
