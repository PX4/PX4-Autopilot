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
 * Tests
 *
 * accel_corr_x_p = [  g  0  0 ]^T	// positive x
 * accel_corr_x_n = [ -g  0  0 ]^T	// negative x
 * accel_corr_y_p = [  0  g  0 ]^T	// positive y
 * accel_corr_y_n = [  0 -g  0 ]^T	// negative y
 * accel_corr_z_p = [  0  0  g ]^T	// positive z
 * accel_corr_z_n = [  0  0 -g ]^T	// negative z
 *
 * 6 tests * 3 axes = 18 equations
 * 9 (accel_T) + 3 (accel_offs) = 12 unknown constants
 *
 * Find accel_offs
 *
 * accel_offs = (accel_raw_x_p + accel_raw_x_n +
 * 				 accel_raw_y_p + accel_raw_y_n +
 * 				 accel_raw_z_p + accel_raw_z_n) / 6
 *
 *
 * Find accel_T
 *
 * 9 unknown constants
 * need 9 equations -> use 3 of 6 measurements -> 3 * 3 = 9 equations
 *
 * accel_corr[i] = accel_T[i][0] * (accel_raw[0] - accel_offs[0]) +
 * 				   accel_T[i][1] * (accel_raw[1] - accel_offs[1]) +
 * 				   accel_T[i][2] * (accel_raw[2] - accel_offs[2])
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
 * b = [ accel_corr_x_p[0] ]	// One measurement per axis is enough
 *     | accel_corr_x_p[1] |
 *     | accel_corr_x_p[2] |
 *     | accel_corr_y_p[0] |
 *     | accel_corr_y_p[1] |
 *     | accel_corr_y_p[2] |
 *     | accel_corr_z_p[0] |
 *     | accel_corr_z_p[1] |
 *     [ accel_corr_z_p[2] ]
 *
 * a_x[i] = accel_raw_x_p[i] - accel_offs[i]	// Measurement for X axis
 * ...
 * A = [ a_x[0]  a_x[1]  a_x[2]       0       0       0       0       0       0 ]
 *     |      0       0       0  a_x[0]  a_x[1]  a_x[2]       0       0       0 |
 *     |      0       0       0       0       0       0  a_x[0]  a_x[1]  a_x[2] |
 *     | a_y[0]  a_y[1]  a_y[2]       0       0       0       0       0       0 |
 *     |      0       0       0  a_y[0]  a_y[1]  a_y[2]       0       0       0 |
 *     |      0       0       0       0       0       0  a_y[0]  a_y[1]  a_y[2] |
 *     | a_z[0]  a_z[1]  a_z[2]       0       0       0       0       0       0 |
 *     |      0       0       0  a_z[0]  a_z[1]  a_z[2]       0       0       0 |
 *     [      0       0       0       0       0       0  a_z[0]  a_z[1]  a_z[2] ]
 *
 * x = A^-1 b
 */

#include "acceleration_transform.h"

void acceleration_correct(float accel_corr[3], int16_t accel_raw[3], float accel_T[3][3], int16_t accel_offs[3]) {
	for (int i = 0; i < 3; i++) {
		accel_corr[i] = 0.0f;
		for (int j = 0; j < 3; j++) {
			accel_corr[i] += accel_T[i][j] * (accel_raw[j] - accel_offs[j]);
		}
	}
}
