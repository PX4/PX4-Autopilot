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

#include <gtest/gtest.h>

/* C++ complains about the C99 'restrict' qualifier. Just ignore it. */
#define restrict

#include "TRICAL.h"


TEST(TRICAL, Initialisation)
{
	TRICAL_instance_t cal;

	TRICAL_init(&cal);
	EXPECT_FLOAT_EQ(1.0f, TRICAL_norm_get(&cal));
}

/* Check that the field norm can be set and read */
TEST(TRICAL, NormGetSet)
{
	TRICAL_instance_t cal;

	TRICAL_init(&cal);
	TRICAL_norm_set(&cal, 2.0f);
	EXPECT_FLOAT_EQ(2.0f, TRICAL_norm_get(&cal));
}

/* Check that the measurement noise can be set and read */
TEST(TRICAL, NoiseGetSet)
{
	TRICAL_instance_t cal;

	TRICAL_init(&cal);
	TRICAL_noise_set(&cal, 2.0f);
	EXPECT_FLOAT_EQ(2.0f, TRICAL_noise_get(&cal));
}

/* Check that the measurement count can be read */
TEST(TRICAL, MeasurementCountGet)
{
	TRICAL_instance_t cal;

	TRICAL_init(&cal);
	cal.measurement_count = 15;
	EXPECT_EQ(15, TRICAL_measurement_count_get(&cal));
}

/*
Check that estimate update works with a series of measurements that should
result in the identity matrix with zero bias
*/
TEST(TRICAL, EstimateUpdateIdentity)
{
	TRICAL_instance_t cal;

	TRICAL_init(&cal);

	float measurement[3];
	unsigned int i;

	for (i = 0; i < 100; i++) {
		measurement[0] = 1.0;
		measurement[1] = 0.0;
		measurement[2] = 0.0;
		TRICAL_estimate_update(&cal, measurement, measurement);

		measurement[0] = 0.0;
		measurement[1] = 1.0;
		measurement[2] = 0.0;
		TRICAL_estimate_update(&cal, measurement, measurement);

		measurement[0] = 0.0;
		measurement[1] = 0.0;
		measurement[2] = 1.0;
		TRICAL_estimate_update(&cal, measurement, measurement);

		measurement[0] = -1.0;
		measurement[1] = 0.0;
		measurement[2] = 0.0;
		TRICAL_estimate_update(&cal, measurement, measurement);

		measurement[0] = 0.0;
		measurement[1] = -1.0;
		measurement[2] = 0.0;
		TRICAL_estimate_update(&cal, measurement, measurement);

		measurement[0] = 0.0;
		measurement[1] = 0.0;
		measurement[2] = -1.0;
		TRICAL_estimate_update(&cal, measurement, measurement);
	}

	float bias_estimate[3], scale_estimate[9];
	TRICAL_estimate_get(&cal, bias_estimate, scale_estimate);
	EXPECT_NEAR(0.0, bias_estimate[0], 2e-2);
	EXPECT_NEAR(0.0, bias_estimate[1], 2e-2);
	EXPECT_NEAR(0.0, bias_estimate[2], 2e-2);

	EXPECT_NEAR(0.0, scale_estimate[0], 2e-2);
	EXPECT_NEAR(0.0, scale_estimate[1], 2e-2);
	EXPECT_NEAR(0.0, scale_estimate[2], 2e-2);

	EXPECT_NEAR(0.0, scale_estimate[3], 2e-2);
	EXPECT_NEAR(0.0, scale_estimate[4], 2e-2);
	EXPECT_NEAR(0.0, scale_estimate[5], 2e-2);

	EXPECT_NEAR(0.0, scale_estimate[6], 2e-2);
	EXPECT_NEAR(0.0, scale_estimate[7], 2e-2);
	EXPECT_NEAR(0.0, scale_estimate[8], 2e-2);
}

/*
Check that estimate update works with a series of measurements that should
result in the identity matrix with a bias in the positive X axis
*/
TEST(TRICAL, EstimateUpdateBias)
{
	TRICAL_instance_t cal;

	TRICAL_init(&cal);

	float measurement[3], ref[3];
	unsigned int i;

	for (i = 0; i < 200; i++) {
		measurement[0] = 2.0;
		measurement[1] = 0.0;
		measurement[2] = 0.0;
		ref[0] = 1.0;
		ref[1] = 0.0;
		ref[2] = 0.0;
		TRICAL_estimate_update(&cal, measurement, ref);

		measurement[0] = 1.0;
		measurement[1] = 1.0;
		measurement[2] = 0.0;
		ref[0] = 0.0;
		ref[1] = 1.0;
		ref[2] = 0.0;
		TRICAL_estimate_update(&cal, measurement, ref);

		measurement[0] = 1.0;
		measurement[1] = 0.0;
		measurement[2] = 1.0;
		ref[0] = 0.0;
		ref[1] = 0.0;
		ref[2] = 1.0;
		TRICAL_estimate_update(&cal, measurement, ref);

		measurement[0] = 0.0;
		measurement[1] = 0.0;
		measurement[2] = 0.0;
		ref[0] = -1.0;
		ref[1] = 0.0;
		ref[2] = 0.0;
		TRICAL_estimate_update(&cal, measurement, ref);

		measurement[0] = 1.0;
		measurement[1] = -1.0;
		measurement[2] = 0.0;
		ref[0] = 0.0;
		ref[1] = -1.0;
		ref[2] = 0.0;
		TRICAL_estimate_update(&cal, measurement, ref);

		measurement[0] = 1.0;
		measurement[1] = 0.0;
		measurement[2] = -1.0;
		ref[0] = 0.0;
		ref[1] = 0.0;
		ref[2] = -1.0;
		TRICAL_estimate_update(&cal, measurement, ref);
	}

	float bias_estimate[3], scale_estimate[9];
	TRICAL_estimate_get(&cal, bias_estimate, scale_estimate);
	EXPECT_NEAR(1.0, bias_estimate[0], 2e-2);
	EXPECT_NEAR(0.0, bias_estimate[1], 2e-2);
	EXPECT_NEAR(0.0, bias_estimate[2], 2e-2);

	EXPECT_NEAR(0.0, scale_estimate[0], 2e-2);
	EXPECT_NEAR(0.0, scale_estimate[1], 2e-2);
	EXPECT_NEAR(0.0, scale_estimate[2], 2e-2);

	EXPECT_NEAR(0.0, scale_estimate[3], 2e-2);
	EXPECT_NEAR(0.0, scale_estimate[4], 2e-2);
	EXPECT_NEAR(0.0, scale_estimate[5], 2e-2);

	EXPECT_NEAR(0.0, scale_estimate[6], 2e-2);
	EXPECT_NEAR(0.0, scale_estimate[7], 2e-2);
	EXPECT_NEAR(0.0, scale_estimate[8], 2e-2);
}

/* Check that state estimates can be accessed */
TEST(TRICAL, EstimateGet)
{
	TRICAL_instance_t cal;

	TRICAL_init(&cal);
	cal.state[0] = 3.0;
	cal.state[1] = 2.0;
	cal.state[2] = 1.0;
	cal.state[3] = 1.0;
	cal.state[4] = 0.5;
	cal.state[5] = 0.1;
	cal.state[6] = 0.5;
	cal.state[7] = 1.0;
	cal.state[8] = 0.5;
	cal.state[9] = 0.1;
	cal.state[10] = 0.5;
	cal.state[11] = 1.0;

	float bias_estimate[3], scale_estimate[9];
	TRICAL_estimate_get(&cal, bias_estimate, scale_estimate);
	EXPECT_FLOAT_EQ(3.0, bias_estimate[0]);
	EXPECT_FLOAT_EQ(2.0, bias_estimate[1]);
	EXPECT_FLOAT_EQ(1.0, bias_estimate[2]);

	EXPECT_FLOAT_EQ(1.0, scale_estimate[0]);
	EXPECT_FLOAT_EQ(0.5, scale_estimate[1]);
	EXPECT_FLOAT_EQ(0.1, scale_estimate[2]);

	EXPECT_FLOAT_EQ(0.5, scale_estimate[3]);
	EXPECT_FLOAT_EQ(1.0, scale_estimate[4]);
	EXPECT_FLOAT_EQ(0.5, scale_estimate[5]);

	EXPECT_FLOAT_EQ(0.1, scale_estimate[6]);
	EXPECT_FLOAT_EQ(0.5, scale_estimate[7]);
	EXPECT_FLOAT_EQ(1.0, scale_estimate[8]);
}

/* Check that state and covariance estimates can be accessed */
TEST(TRICAL, EstimateGetExt)
{
	TRICAL_instance_t cal;

	TRICAL_init(&cal);

	cal.state[0] = 3.0;
	cal.state[1] = 2.0;
	cal.state[2] = 1.0;
	cal.state[3] = 1.0;
	cal.state[4] = 0.5;
	cal.state[5] = 0.1;
	cal.state[6] = 0.5;
	cal.state[7] = 1.0;
	cal.state[8] = 0.5;
	cal.state[9] = 0.1;
	cal.state[10] = 0.5;
	cal.state[11] = 1.0;

	cal.state_covariance[0] = 10.0;
	cal.state_covariance[13] = 20.0;
	cal.state_covariance[26] = 30.0;
	cal.state_covariance[39] = 40.0;
	cal.state_covariance[52] = 50.0;
	cal.state_covariance[65] = 60.0;
	cal.state_covariance[78] = 70.0;
	cal.state_covariance[91] = 80.0;
	cal.state_covariance[104] = 90.0;
	cal.state_covariance[117] = 100.0;
	cal.state_covariance[130] = 110.0;
	cal.state_covariance[143] = 120.0;

	float bias_estimate[3], scale_estimate[9], bias_estimate_variance[3],
	      scale_estimate_variance[9];
	TRICAL_estimate_get_ext(&cal, bias_estimate, scale_estimate,
				bias_estimate_variance, scale_estimate_variance);

	EXPECT_FLOAT_EQ(3.0, bias_estimate[0]);
	EXPECT_FLOAT_EQ(2.0, bias_estimate[1]);
	EXPECT_FLOAT_EQ(1.0, bias_estimate[2]);

	EXPECT_FLOAT_EQ(1.0, scale_estimate[0]);
	EXPECT_FLOAT_EQ(0.5, scale_estimate[1]);
	EXPECT_FLOAT_EQ(0.1, scale_estimate[2]);

	EXPECT_FLOAT_EQ(0.5, scale_estimate[3]);
	EXPECT_FLOAT_EQ(1.0, scale_estimate[4]);
	EXPECT_FLOAT_EQ(0.5, scale_estimate[5]);

	EXPECT_FLOAT_EQ(0.1, scale_estimate[6]);
	EXPECT_FLOAT_EQ(0.5, scale_estimate[7]);
	EXPECT_FLOAT_EQ(1.0, scale_estimate[8]);

	EXPECT_FLOAT_EQ(10.0, bias_estimate_variance[0]);
	EXPECT_FLOAT_EQ(20.0, bias_estimate_variance[1]);
	EXPECT_FLOAT_EQ(30.0, bias_estimate_variance[2]);

	EXPECT_FLOAT_EQ(40.0, scale_estimate_variance[0]);
	EXPECT_FLOAT_EQ(50.0, scale_estimate_variance[1]);
	EXPECT_FLOAT_EQ(60.0, scale_estimate_variance[2]);

	EXPECT_FLOAT_EQ(70.0, scale_estimate_variance[3]);
	EXPECT_FLOAT_EQ(80.0, scale_estimate_variance[4]);
	EXPECT_FLOAT_EQ(90.0, scale_estimate_variance[5]);

	EXPECT_FLOAT_EQ(100.0, scale_estimate_variance[6]);
	EXPECT_FLOAT_EQ(110.0, scale_estimate_variance[7]);
	EXPECT_FLOAT_EQ(120.0, scale_estimate_variance[8]);
}

/*
Check that measurement calibration can be performed based on the instance
state.

This mirrors Filter.CalibrateFull.
*/
TEST(TRICAL, Calibrate)
{
	TRICAL_instance_t cal;

	TRICAL_init(&cal);
	cal.state[0] = 3.0;
	cal.state[1] = 2.0;
	cal.state[2] = 1.0;
	cal.state[3] = 1.0;
	cal.state[4] = 0.5;
	cal.state[5] = 0.1;
	cal.state[6] = 0.5;
	cal.state[7] = 1.0;
	cal.state[8] = 0.5;
	cal.state[9] = 0.1;
	cal.state[10] = 0.5;
	cal.state[11] = 1.0;

	float measurement[3] = { 1.0, 2.0, 3.0 }, result[3];
	TRICAL_measurement_calibrate(&cal, measurement, result);
	EXPECT_FLOAT_EQ(-3.8, result[0]);
	EXPECT_FLOAT_EQ(0.0, result[1]);
	EXPECT_FLOAT_EQ(3.8, result[2]);
}
