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
 * Solve separate system for each row of accel_T:
 *
 * accel_corr_ref[j*2][i] = accel_T[i] * (accel_raw_ref[j*2] - accel_offs), j = 0...2
 *
 * A x = b
 *
 * x = [ accel_T[0][i] ]
 *     | accel_T[1][i] |
 *     [ accel_T[2][i] ]
 *
 * b = [ accel_corr_ref[0][i] ]	// One measurement per axis is enough
 *     | accel_corr_ref[2][i] |
 *     [ accel_corr_ref[4][i] ]
 *
 * a[i][j] = accel_raw_ref[i][j] - accel_offs[j], i = 0;2;4, j = 0...2
 *
 * Matrix A is common for all three systems:
 * A = [ a[0][0]  a[0][1]  a[0][2] ]
 *     | a[2][0]  a[2][1]  a[2][2] |
 *     [ a[4][0]  a[4][1]  a[4][2] ]
 *
 * x = A^-1 b
 *
 * accel_T = A^-1 * g
 * g = 9.81
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

int mat_invert3(float src[3][3], float dst[3][3]) {
	float det = src[0][0] * (src[1][1] * src[2][2] - src[1][2] * src[2][1]) -
			src[0][1] * (src[1][0] * src[2][2] - src[1][2] * src[2][0]) +
			src[0][2] * (src[1][0] * src[2][1] - src[1][1] * src[2][0]);
	if (det == 0.0)
		return -1;	// Singular matrix
	dst[0][0] = (src[1][1] * src[2][2] - src[1][2] * src[2][1]) / det;
	dst[1][0] = (src[1][2] * src[2][0] - src[1][0] * src[2][2]) / det;
	dst[2][0] = (src[1][0] * src[2][1] - src[1][1] * src[2][0]) / det;
	dst[0][1] = (src[0][2] * src[2][1] - src[0][1] * src[2][2]) / det;
	dst[1][1] = (src[0][0] * src[2][2] - src[0][2] * src[2][0]) / det;
	dst[2][1] = (src[0][1] * src[2][0] - src[0][0] * src[2][1]) / det;
	dst[0][2] = (src[0][1] * src[1][2] - src[0][2] * src[1][1]) / det;
	dst[1][2] = (src[0][2] * src[1][0] - src[0][0] * src[1][2]) / det;
	dst[2][2] = (src[0][0] * src[1][1] - src[0][1] * src[1][0]) / det;
	return 0;
}

int calculate_calibration_values(int16_t accel_raw_ref[6][3],
		float accel_T[3][3], int16_t accel_offs[3], float g) {
	/* calculate raw offsets */
	for (int i = 0; i < 3; i++) {
		accel_offs[i] = (int16_t) (((int32_t) (accel_raw_ref[i * 2][i])
				+ (int32_t) (accel_raw_ref[i * 2 + 1][i])) / 2);
	}
	/* fill matrix A for linear equations system*/
	float mat_A[3][3];
	memset(mat_A, 0, sizeof(mat_A));
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			float a = (accel_raw_ref[i * 2][j] - accel_offs[j]);
			mat_A[i][j] = a;
		}
	}
	/* calculate inverse matrix for A */
	float mat_A_inv[3][3];
	mat_invert3(mat_A, mat_A_inv);
	for (int i = 0; i < 3; i++) {
		/* copy results to accel_T */
		for (int j = 0; j < 3; j++) {
			/* simplify matrices mult because b has only one non-zero element == g at index i */
			accel_T[j][i] = mat_A_inv[j][i] * g;
		}
	}
	return 0;
}
