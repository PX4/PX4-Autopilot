#include <matrix/math.hpp>

#ifndef EKF_UTILS_HPP
#define EKF_UTILS_HPP

// return the square of two floating point numbers - used in auto coded sections
static constexpr float sq(float var) { return var * var; }

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

// We should use a 3-2-1 Tait-Bryan (yaw-pitch-roll) rotation sequence
// when there is more roll than pitch tilt and a 3-1-2 rotation sequence
// when there is more pitch than roll tilt to avoid gimbal lock.
inline bool shouldUse321RotationSequence(const matrix::Dcmf &R) { return fabsf(R(2, 0)) < fabsf(R(2, 1)); }

inline float getEuler321Yaw(const matrix::Dcmf &R) { return atan2f(R(1, 0), R(0, 0)); }
inline float getEuler312Yaw(const matrix::Dcmf &R) { return atan2f(-R(0, 1), R(1, 1)); }

float getEuler321Yaw(const matrix::Quatf &q);
float getEuler312Yaw(const matrix::Quatf &q);

inline float getEulerYaw(const matrix::Dcmf &R)
{
	if (shouldUse321RotationSequence(R)) {
		return getEuler321Yaw(R);

	} else {
		return getEuler312Yaw(R);
	}
}

matrix::Dcmf updateEuler321YawInRotMat(float yaw, const matrix::Dcmf &rot_in);
matrix::Dcmf updateEuler312YawInRotMat(float yaw, const matrix::Dcmf &rot_in);

// Checks which euler rotation sequence to use and update yaw in rotation matrix
matrix::Dcmf updateYawInRotMat(float yaw, const matrix::Dcmf &rot_in);

namespace ecl
{
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
#endif // EKF_UTILS_HPP
