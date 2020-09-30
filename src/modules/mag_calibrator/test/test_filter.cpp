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
#include "filter.h"

/*
Test measurement calibration with the identity matrix -- all state values are
zero because the identity matrix is added to the symmetric scale factor matrix
during measurement calibration.
*/
TEST(Filter, CalibrateIdentity)
{
	float state[] = {
		0.0, 0.0, 0.0,
		0.0, 0.0, 0.0,
		0.0, 0.0, 0.0,
		0.0, 0.0, 0.0
	};
	float measurement[] = { 1.0, 2.0, 3.0 }, result[3];

	_trical_measurement_calibrate(state, measurement, result);
	EXPECT_FLOAT_EQ(measurement[0], result[0]);
	EXPECT_FLOAT_EQ(measurement[1], result[1]);
	EXPECT_FLOAT_EQ(measurement[2], result[2]);
}

/*
Test measurement calibration with a bias, but the identity matrix for scale
factors.

This test would permit calibration for zero-point bias and hard iron
distortion only.
*/
TEST(Filter, CalibrateBias)
{
	float state[] = {
		1.0, 2.0, 3.0,
		0.0, 0.0, 0.0,
		0.0, 0.0, 0.0,
		0.0, 0.0, 0.0
	};
	float measurement[] = { 1.0, 2.0, 3.0 }, result[3];

	_trical_measurement_calibrate(state, measurement, result);
	EXPECT_FLOAT_EQ(0.0, result[0]);
	EXPECT_FLOAT_EQ(0.0, result[1]);
	EXPECT_FLOAT_EQ(0.0, result[2]);
}

/*
Test measurement calibration with an orthogonal scale factor matrix -- this
is added to the identity matrix during the calculation, so the actual scale
factor matrix used will be:
2  0  0
0  2  0
0  0  2

This test would permit calibration for soft iron distortion of the magnetic
field, as well as scale factor error in the magnetometer.
*/
TEST(Filter, CalibrateOrthogonal)
{
	float state[] = {
		0.0, 0.0, 0.0,
		1.0, 0.0, 0.0,
		0.0, 1.0, 0.0,
		0.0, 0.0, 1.0
	};
	float measurement[] = { 1.0, 2.0, 3.0 }, result[3];

	_trical_measurement_calibrate(state, measurement, result);
	EXPECT_FLOAT_EQ(2.0, result[0]);
	EXPECT_FLOAT_EQ(4.0, result[1]);
	EXPECT_FLOAT_EQ(6.0, result[2]);
}

/*
Test measurement calibration with a full symmetric scale factor matrix. This
test would permit calibration for soft iron distortion, as well as scale
factor error and axis misalignment in the sensor.
*/
TEST(Filter, CalibrateSymmetric)
{
	float state[] = {
		0.0, 0.0, 0.0,
		1.0, 0.5, 0.1,
		0.5, 1.0, 0.5,
		0.1, 0.5, 1.0
	};
	float measurement[] = { 1.0, 2.0, 3.0 }, result[3];

	_trical_measurement_calibrate(state, measurement, result);
	EXPECT_FLOAT_EQ(3.3, result[0]);
	EXPECT_FLOAT_EQ(6.0, result[1]);
	EXPECT_FLOAT_EQ(7.1, result[2]);
}

/*
Test measurement calibration with a full symmetric scale factor matrix as well
as bias. This test would permit calibration for hard and soft iron distortion,
as well as bias, scale factor error and axis misalignment in the sensor.

(The general case, including body/sensor axis misalignment, would require
non-symmetric matrices as well as knowledge of the body attitude at each
calibration step, and is easier to do elsewhere -- e.g. by measurement.)
*/
TEST(Filter, CalibrateFull)
{
	float state[] = {
		3.0, 2.0, 1.0,
		1.0, 0.5, 0.1,
		0.5, 1.0, 0.5,
		0.1, 0.5, 1.0
	};
	float measurement[] = { 1.0, 2.0, 3.0 }, result[3];

	_trical_measurement_calibrate(state, measurement, result);
	EXPECT_FLOAT_EQ(-3.8, result[0]);
	EXPECT_FLOAT_EQ(0.0, result[1]);
	EXPECT_FLOAT_EQ(3.8, result[2]);
}

/*
Now test measurement reduction, from a raw 3-axis sensor reading to a scalar
output based on the current calibration estimate. This is used in the UKF
implementation to generate a measurement estimate for each sigma point.
*/
TEST(Filter, MeasurementReduction)
{
	float state[] = {
		0.0, 0.0, 0.0,
		1.0, 0.5, 0.1,
		0.5, 1.0, 0.5,
		0.1, 0.5, 1.0
	};
	float measurement[] = { 1.0, 2.0, 3.0 }, result;

	result = _trical_measurement_reduce(state, measurement, measurement);
	EXPECT_FLOAT_EQ(6.0497932, result);
}
