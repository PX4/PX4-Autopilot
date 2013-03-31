/*
 * acceleration_transform.c
 *
 *   Copyright (C) 2013 Anton Babushkin. All rights reserved.
 *   Author: 	Anton Babushkin	<rk3dov@gmail.com>
 *
 * Transform acceleration vector to true orientation and scale
 *
 * * * * Model * * *
 * accel_corr = accel_T * (accel_raw - accel_offs)
 *
 * accel_corr[3] - fully corrected acceleration vector in UAV frame
 * accel_T[3][3] - accelerometers transform matrix, rotation and scaling transform
 * accel_raw[3]  - raw acceleration vector
 * accel_offs[3] - true acceleration offset vector, don't use sensors offset because
 *                 it's caused not only by real offset but also by sensor rotation
 *
 * * * * Calibration * * *
 *
 * Reference vectors
 * accel_corr_ref[6][3] = [  g  0  0 ]     // positive x
 *                        | -g  0  0 |     // negative x
 *                        |  0  g  0 |     // positive y
 *                        |  0 -g  0 |     // negative y
 *                        |  0  0  g |     // positive z
 *                        [  0  0 -g ]     // negative z
 * accel_raw_ref[6][3]
 *
 * accel_corr_ref[i] = accel_T * (accel_raw_ref[i] - accel_offs), i = 0...5
 *
 * 6 reference vectors * 3 axes = 18 equations
 * 9 (accel_T) + 3 (accel_offs) = 12 unknown constants
 *
 * Find accel_offs
 *
 * accel_offs[i] = (accel_raw_ref[i*2][i] + accel_raw_ref[i*2+1][i]) / 2
 *
 *
 * Find accel_T
 *
 * 9 unknown constants
 * need 9 equations -> use 3 of 6 measurements -> 3 * 3 = 9 equations
 *
 * accel_corr_ref[i*2] = accel_T * (accel_raw_ref[i*2] - accel_offs), i = 0...2
 *
 * A x = b
 *
 * x = [ accel_T[0][0] ]
 *     | accel_T[0][1] |
 *     | accel_T[0][2] |
 *     | accel_T[1][0] |
 *     | accel_T[1][1] |
 *     | accel_T[1][2] |
 *     | accel_T[2][0] |
 *     | accel_T[2][1] |
 *     [ accel_T[2][2] ]
 *
 * b = [ accel_corr_ref[0][0] ]	// One measurement per axis is enough
 *     | accel_corr_ref[0][1] |
 *     | accel_corr_ref[0][2] |
 *     | accel_corr_ref[2][0] |
 *     | accel_corr_ref[2][1] |
 *     | accel_corr_ref[2][2] |
 *     | accel_corr_ref[4][0] |
 *     | accel_corr_ref[4][1] |
 *     [ accel_corr_ref[4][2] ]
 *
 * a[i][j] = accel_raw_ref[i][j] - accel_offs[j], i = 0...5, j = 0...2
 *
 * A = [ a[0][0]  a[0][1]  a[0][2]        0        0        0        0        0        0 ]
 *     |       0        0        0  a[0][0]  a[0][1]  a[0][2]        0        0        0 |
 *     |       0        0        0        0        0        0  a[0][0]  a[0][1]  a[0][2] |
 *     | a[1][0]  a[1][1]  a[1][2]        0        0        0        0        0        0 |
 *     |       0        0        0  a[1][0]  a[1][1]  a[1][2]        0        0        0 |
 *     |       0        0        0        0        0        0  a[1][0]  a[1][1]  a[1][2] |
 *     | a[2][0]  a[2][1]  a[2][2]        0        0        0        0        0        0 |
 *     |       0        0        0  a[2][0]  a[2][1]  a[2][2]        0        0        0 |
 *     [       0        0        0        0        0        0  a[2][0]  a[2][1]  a[2][2] ]
 *
 * x = A^-1 b
 */

#include "acceleration_transform.h"

void acceleration_correct(float accel_corr[3], int16_t accel_raw[3],
		float accel_T[3][3], int16_t accel_offs[3]) {
	for (int i = 0; i < 3; i++) {
		accel_corr[i] = 0.0f;
		for (int j = 0; j < 3; j++) {
			accel_corr[i] += accel_T[i][j] * (accel_raw[j] - accel_offs[j]);
		}
	}
}

void calculate_calibration_values(int16_t accel_raw_ref[6][3],
		float accel_T[3][3], int16_t accel_offs[3], float g) {
	for (int i = 0; i < 3; i++) {
		accel_offs[i] = (int16_t) (((int32_t) (accel_raw_ref[i * 2][i])
				+ (int32_t) (accel_raw_ref[i * 2 + 1][i])) / 2);
	}
}
