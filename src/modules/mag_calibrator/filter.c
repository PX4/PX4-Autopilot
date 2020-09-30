/*
Copyright (C) 2013 Ben Dyer

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <assert.h>
#include <math.h>
#include <float.h>
#include <string.h>

#include "TRICAL.h"
#include "filter.h"
#include "3dmath.h"

#ifdef DEBUG
#include <stdio.h>

static void _print_matrix(const char *label, float mat[], size_t rows,
			  size_t cols);

static void _print_matrix(const char *label, float mat[], size_t rows,
			  size_t cols)
{
	printf("%s", label);

	for (size_t i = 0; i < cols; i++) {
		for (size_t j = 0; j < rows; j++) {
			printf("%12.6f ", mat[j * cols + i]);
		}

		printf("\n");
	}
}
#else
#define _print_matrix(a, b, c, d)
#endif

/*
A bit about the UKF formulation in this file: main references are
[1]: http://www.acsu.buffalo.edu/~johnc/mag_cal05.pdf
[2]: http://malcolmdshuster.com/Pub_2002c_J_scale_scan.pdf

Since we don't know the attitude matrix, we use scalar values for measurements
and scalar measurement noise.

The relevant equations are (2a) and (2b) in [2]. Basically, the measurement
value is:

z = -Bt x (2D + D^2) x B + 2Bt x (I + D) x b - |b|^2

where B is the 3-axis measurement vector, and H is the reference field (making
|H| the norm of the reference field, which we know). Here, t is used as the
transpose operator.

The measurement noise is a scalar:

v = 2[(I + D) x B - b]t x E - |E|^2

where B is the 3-axis measurement vector, b is the 3-axis bias estimate,
E is the measurement noise, I is the 3x3 identity matrix, and D is the 3x3
scale estimate.

The implementation follows the general approach of https://github.com/sfwa/ukf
however there a number of simplifications due to the restricted problem
domain.

Since there is no process model, the apriori mean is the same as the state at
the start of the update step, and W' is just the set of sigma points. That in
turn means that there's no need to calculate propagated state covariance from
W'; it's the same as the covariance was at the start of the update step.

The scalar measurement model also simplifies a lot of the cross-correlation
and Kalman gain calculation.

Like the full SFWA UKF, and unlike the papers upon which this approach is
based, we're actually using the scaled unscented Kalman filter. The main
differences are in the choices of scaling parameters, including alpha, beta
and kappa. Given the relatively small dimensionality of the filter it's
probably not strictly necessary to use the scaled formulation, but we'd
already implemented that so it seemed easier to continue with that approach.
*/

/*
_trical_measurement_reduce
Reduces `measurement` to a scalar value based on the calibration estimate in
`state` and the current field direction estimate `field` (if no absolute
orientation calibration is required, the same vector can be supplied for
`measurement` and `field`).

FIXME: It's not clear to me why the measurement model should be as in (2a)
above, when we can just apply the current calibration estimate to the
measurement and take its magnitude. Look into this further if the below
approach doesn't work.
*/
float _trical_measurement_reduce(float state[TRICAL_STATE_DIM], float
				 measurement[3], float field[3])
{
	float temp[3];
	_trical_measurement_calibrate(state, measurement, temp);

	return sqrtf(fabsf(temp[X] * field[X] + temp[Y] * field[Y] + temp[Z] * field[Z]));
}

/*
_trical_measurement_calibrate
Calibrates `measurement` based on the calibration estimate in `state` and
copies the result to `calibrated_measurement`. The `measurement` and
`calibrated_measurement` parameters may be pointers to the same vector.

Implements
B' = (I_{3x3} + D)B - b

where B' is the calibrated measurement, I_{3x3} is the 3x3 identity matrix,
D is the scale calibration matrix, B is the raw measurement, and b is the bias
vector.
*/
void _trical_measurement_calibrate(float state[TRICAL_STATE_DIM],
				   float measurement[3], float calibrated_measurement[3])
{
	assert(state && measurement && calibrated_measurement);

	float v[3];
	float *restrict s = state;
	float *restrict c = calibrated_measurement;

	v[0] = measurement[0] - s[0];
	v[1] = measurement[1] - s[1];
	v[2] = measurement[2] - s[2];

	/* 3x3 matrix multiply */
	c[0] = v[0] * (s[3] + 1.0f) + v[1] * s[4] + v[2] * s[5];
	c[1] = v[0] * s[6] + v[1] * (s[7] + 1.0f) + v[2] * s[8];
	c[2] = v[0] * s[9] + v[1] * s[10] + v[2] * (s[11] + 1.0f);
}

/*
_trical_filter_iterate
Generates a new calibration estimate for `instance` incorporating the raw
sensor readings in `measurement`.
*/
void _trical_filter_iterate(TRICAL_instance_t *instance,
			    float measurement[3], float field[3])
{
	unsigned int i, j, k, l, col;

	float *restrict covariance = instance->state_covariance;
	float *restrict state = instance->state;

	/*
	LLT decomposition on state covariance matrix, with result multiplied by
	TRICAL_DIM_PLUS_LAMBDA
	*/
	float covariance_llt[TRICAL_STATE_DIM * TRICAL_STATE_DIM];
	memset(covariance_llt, 0, sizeof(covariance_llt));
	matrix_cholesky_decomp_scale_f(
		TRICAL_STATE_DIM, covariance_llt, covariance, TRICAL_DIM_PLUS_LAMBDA);

	_print_matrix("LLT:\n", covariance_llt, TRICAL_STATE_DIM,
		      TRICAL_STATE_DIM);

	/*
	Generate the sigma points, and use them as the basis of the measurement
	estimates
	*/
	float temp_sigma[TRICAL_STATE_DIM], temp;
	float measurement_estimates[TRICAL_NUM_SIGMA], measurement_estimate_mean;

	measurement_estimate_mean = 0.0;

	/*
	Handle central sigma point -- process the measurement based on the current
	state vector
	*/
	measurement_estimates[0] = _trical_measurement_reduce(state, measurement,
				   field);

	__asm__ __volatile__("");

	for (i = 0, col = 0; i < TRICAL_STATE_DIM; i++, col += TRICAL_STATE_DIM) {
		/*
		Handle the positive sigma point -- perturb the state vector based on
		the current column of the covariance matrix, and process the
		measurement based on the resulting state estimate
		*/
		__asm__ __volatile__("");

		for (k = col, l = 0; l < TRICAL_STATE_DIM; k++, l++) {
			temp_sigma[l] = state[l] + covariance_llt[k];
		}

		measurement_estimates[i + 1] =
			_trical_measurement_reduce(temp_sigma, measurement, field);

		/* Handle the negative sigma point -- mirror of the above */
		__asm__ __volatile__("");

		for (k = col, l = 0; l < TRICAL_STATE_DIM; k++, l++) {
			temp_sigma[l] = state[l] - covariance_llt[k];
		}

		measurement_estimates[i + 1 + TRICAL_STATE_DIM] =
			_trical_measurement_reduce(temp_sigma, measurement, field);

		/* Calculate the measurement estimate sum as we go */
		temp = measurement_estimates[i + 1] +
		       measurement_estimates[i + 1 + TRICAL_STATE_DIM];
		measurement_estimate_mean += temp;
	}

	measurement_estimate_mean = measurement_estimate_mean * TRICAL_SIGMA_WMI +
				    measurement_estimates[0] * TRICAL_SIGMA_WM0;

	/*
	Convert estimates to deviation from mean (so measurement_estimates
	effectively becomes Z').

	While we're at it, calculate the measurement estimate covariance (which
	is a scalar quantity).
	*/
	float measurement_estimate_covariance = 0.0;

	__asm__ __volatile__("");

	for (i = 0; i < TRICAL_NUM_SIGMA; i++) {
		measurement_estimates[i] -= measurement_estimate_mean;

		temp = measurement_estimates[i] * measurement_estimates[i];
		measurement_estimate_covariance += temp;
	}

	_print_matrix("Measurement estimates:\n", measurement_estimates, 1,
		      TRICAL_NUM_SIGMA);

	/* Add the sensor noise to the measurement estimate covariance */
	temp = instance->measurement_noise * instance->measurement_noise;
	measurement_estimate_covariance += temp;

	/* Calculate cross-correlation matrix (1 x TRICAL_STATE_DIM) */
	float cross_correlation[TRICAL_STATE_DIM];
	memset(cross_correlation, 0, sizeof(cross_correlation));

	/*
	Calculate the innovation (difference between the expected value, i.e. the
	field norm, and the measurement estimate mean).
	*/
	float innovation;
	innovation = instance->field_norm - measurement_estimate_mean;

	/* Iterate over sigma points, two at a time */
	__asm__ __volatile__("");

	for (i = 0; i < TRICAL_STATE_DIM; i++) {
		/* Iterate over the cross-correlation matrix */
		__asm__ __volatile__("");

		for (j = 0; j < TRICAL_STATE_DIM; j++) {
			/*
			We're regenerating the sigma points as we go, so that we don't
			need to store W'.
			*/
			temp = measurement_estimates[i + 1] *
			       (state[j] + covariance_llt[i * TRICAL_STATE_DIM + j]);
			cross_correlation[j] += temp;

			temp = measurement_estimates[i + 1 + TRICAL_STATE_DIM] *
			       (state[j] - covariance_llt[i * TRICAL_STATE_DIM + j]);
			cross_correlation[j] += temp;
		}
	}

	/*
	Scale the results of the previous step, and add in the scaled central
	sigma point
	*/
	__asm__ __volatile__("");

	for (j = 0; j < TRICAL_STATE_DIM; j++) {
		temp = TRICAL_SIGMA_WC0 * measurement_estimates[0] * state[j];
		cross_correlation[j] = TRICAL_SIGMA_WCI * cross_correlation[j] + temp;
	}

	_print_matrix("Cross-correlation:\n", cross_correlation, 1,
		      TRICAL_STATE_DIM);

	/*
	Update the state -- since the measurement is a scalar, we can calculate
	the Kalman gain and update in a single pass.
	*/
	float kalman_gain;
	temp = recip(measurement_estimate_covariance);
	__asm__ __volatile__("");

	for (i = 0; i < TRICAL_STATE_DIM; i++) {
		kalman_gain = cross_correlation[i] * temp;
		state[i] += kalman_gain * innovation;
	}

	_print_matrix("State:\n", state, 1, TRICAL_STATE_DIM);

	/*
	Update the state covariance:
	covariance = covariance - kalman gain * measurement estimate covariance *
	             (transpose of kalman gain)

	Since kalman gain is a 1 x TRICAL_STATE_DIM matrix, multiplying by its
	transpose is just a vector outer product accumulating onto state
	covariance.

	And, of course, since kalman gain is cross correlation *
	(1 / measurement estimate covariance), and we multiply by measurement
	estimate covariance during the outer product, we can skip that whole step
	and just use cross correlation instead.
	*/
	__asm__ __volatile__("");

	for (i = 0; i < TRICAL_STATE_DIM; i++) {
		temp = -cross_correlation[i];

		__asm__ __volatile__("");

		for (j = 0; j < TRICAL_STATE_DIM; j++) {
			covariance[i * TRICAL_STATE_DIM + j] +=
				temp * cross_correlation[j];
		}
	}

	_print_matrix("State covariance:\n", covariance, TRICAL_STATE_DIM,
		      TRICAL_STATE_DIM);
}
