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

#ifndef _TRICAL_H_
#define _TRICAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#define TRICAL_STATE_DIM 12

typedef struct {
	float field_norm;
	float measurement_noise;

	float state[TRICAL_STATE_DIM];
	float state_covariance[TRICAL_STATE_DIM * TRICAL_STATE_DIM];
	unsigned int measurement_count;
} TRICAL_instance_t;

/*
TRICAL_init:
Initializes `instance`. Must be called prior to any other TRICAL procedures
taking `instance` as a parameter.

If called on an instance that has already been initialized, TRICAL_init will
reset that instance to its default state.
*/
void TRICAL_init(TRICAL_instance_t *instance);

/*
TRICAL_reset:
Resets the state and state covariance of `instance`.
*/
void TRICAL_reset(TRICAL_instance_t *instance);

/*
TRICAL_norm_set:
Sets the expected field norm (magnitude) of `instance` to `norm`. If `norm`
differs from the instance's current field norm, all estimates are multiplied
by the ratio of new norm:old norm.
*/
void TRICAL_norm_set(TRICAL_instance_t *instance, float norm);

/*
TRICAL_norm_get:
Returns the expected field norm (magnitude) of `instance`.
*/
float TRICAL_norm_get(TRICAL_instance_t *instance);

/*
TRICAL_noise_set:
Sets the standard deviation in measurement supplied to `instance` to `noise`.
*/
void TRICAL_noise_set(TRICAL_instance_t *instance, float noise);

/*
TRICAL_noise_get:
Returns the standard deviation in measurements supplied to `instance`.
*/
float TRICAL_noise_get(TRICAL_instance_t *instance);

/*
TRICAL_measurement_count_get:
Returns the number of measurements previously provided to `instance` via
TRICAL_estimate_update.
*/
unsigned int TRICAL_measurement_count_get(TRICAL_instance_t *instance);

/*
TRICAL_estimate_update
Updates the calibration estimate of `instance` based on the new data in
`measurement`, and the current field direction estimate `reference_field`.
Call this function with each reading you receive from your sensor.
*/
void TRICAL_estimate_update(TRICAL_instance_t *instance,
			    float measurement[3], float reference_field[3]);

/*
TRICAL_estimate_get
Copies the calibration bias and scale esimates of `instance` to
`bias_estimate` and `scale_estimate` respectively. A new calibration estimate
will be available after every call to TRICAL_estimate_update.
*/
void TRICAL_estimate_get(TRICAL_instance_t *instance, float bias_estimate[3],
			 float scale_estimate[9]);

/*
TRICAL_estimate_get_ext
Same as TRICAL_estimate_get, but additionally copies the bias and scale
estimate variances to `bias_estimate_variance` and `scale_estimate_variance`.
*/
void TRICAL_estimate_get_ext(TRICAL_instance_t *instance,
			     float bias_estimate[3], float scale_estimate[9],
			     float bias_estimate_variance[3], float scale_estimate_variance[9]);

/*
TRICAL_measurement_calibrate
Calibrates `measurement` based on the current calibration estimates, and
copies the result to `calibrated_measurement`.

DO NOT pass the calibrated measurement into TRICAL_estimate_update, as it
needs the raw measurement values to work.
*/
void TRICAL_measurement_calibrate(TRICAL_instance_t *instance,
				  float measurement[3], float calibrated_measurement[3]);

#ifdef __cplusplus
}
#endif

#endif
