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

#ifndef _FILTER_H_
#define _FILTER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Unscented Kalman filter sigma point and scaling parameters. */
#define TRICAL_NUM_SIGMA (2 * TRICAL_STATE_DIM + 1)

#define TRICAL_ALPHA_2 (1.0f)
#define TRICAL_BETA (0.0f)
#define TRICAL_KAPPA (1.0f)
#define TRICAL_LAMBDA (TRICAL_ALPHA_2 * (TRICAL_STATE_DIM + TRICAL_KAPPA) - \
		       TRICAL_STATE_DIM)
#define TRICAL_DIM_PLUS_LAMBDA (TRICAL_ALPHA_2 * \
				(TRICAL_STATE_DIM + TRICAL_KAPPA))

#define TRICAL_SIGMA_WM0 (TRICAL_LAMBDA / TRICAL_DIM_PLUS_LAMBDA)
#define TRICAL_SIGMA_WC0 (TRICAL_SIGMA_WM0 + \
			  (1.0f - TRICAL_ALPHA_2 + TRICAL_BETA))
#define TRICAL_SIGMA_WMI (1.0f / (2.0f * TRICAL_DIM_PLUS_LAMBDA))
#define TRICAL_SIGMA_WCI (TRICAL_SIGMA_WMI)

/* Internal function prototypes */

/*
_trical_measurement_reduce
Reduces `measurement` to a scalar value based on the calibration estimate in
`state`.
*/
float _trical_measurement_reduce(float state[TRICAL_STATE_DIM], float
				 measurement[3], float field[3]);

/*
_trical_measurement_calibrate
Calibrates `measurement` based on the calibration estimate in `state` and
copies the result to `calibrated_measurement`. The `measurement` and
`calibrated_measurement` parameters may be pointers to the same vector.
*/
void _trical_measurement_calibrate(float state[TRICAL_STATE_DIM],
				   float measurement[3], float calibrated_measurement[3]);

/*
_trical_filter_iterate
Generates a new calibration estimate for `instance` incorporating the raw
sensor readings in `measurement`.
*/
void _trical_filter_iterate(TRICAL_instance_t *instance,
			    float measurement[3], float field[3]);

#ifdef __cplusplus
}
#endif

#endif
