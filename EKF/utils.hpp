#include <matrix/math.hpp>

#pragma once

// converts Tait-Bryan 312 sequence of rotations from frame 1 to frame 2
// to the corresponding rotation matrix that rotates from frame 2 to frame 1
// rot312(0) - First rotation is a RH rotation about the Z axis (rad)
// rot312(1) - Second rotation is a RH rotation about the X axis (rad)
// rot312(2) - Third rotation is a RH rotation about the Y axis (rad)
// See http://www.atacolorado.com/eulersequences.doc
matrix::Dcmf taitBryan312ToRotMat(const matrix::Vector3f &rot312);

// Use Kahan summation algorithm to get the sum of "sum_previous" and "input".
// This function relies on the caller to be responsible for keeping a copy of
// "accumulator" and passing this value at the next iteration.
// Ref: https://en.wikipedia.org/wiki/Kahan_summation_algorithm
float kahanSummation(float sum_previous, float input, float &accumulator);

// calculate the inverse rotation matrix from a quaternion rotation
// this produces the inverse rotation to that produced by the math library quaternion to Dcmf operator
matrix::Dcmf quatToInverseRotMat(const matrix::Quatf &quat);

namespace ecl{
	inline float powf(float x, int exp)
	{
		float ret;
		if (exp > 0) {
			ret = x;
			for (int count = 1; count < exp; count++) {
				ret *= x;
			}
			return ret;
		} else if (exp < 0) {
			return 1.0f / ecl::powf(x, -exp);
		}
		return 1.0f;
	}
}
