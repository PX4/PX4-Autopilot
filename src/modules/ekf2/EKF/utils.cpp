/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "utils.hpp"

matrix::Dcmf taitBryan312ToRotMat(const matrix::Vector3f &rot312)
{
	// Calculate the frame2 to frame 1 rotation matrix from a 312 Tait-Bryan rotation sequence
	const float c2 = cosf(rot312(2)); // third rotation is pitch
	const float s2 = sinf(rot312(2));
	const float s1 = sinf(rot312(1)); // second rotation is roll
	const float c1 = cosf(rot312(1));
	const float s0 = sinf(rot312(0)); // first rotation is yaw
	const float c0 = cosf(rot312(0));

	matrix::Dcmf R;
	R(0, 0) = c0 * c2 - s0 * s1 * s2;
	R(1, 1) = c0 * c1;
	R(2, 2) = c2 * c1;
	R(0, 1) = -c1 * s0;
	R(0, 2) = s2 * c0 + c2 * s1 * s0;
	R(1, 0) = c2 * s0 + s2 * s1 * c0;
	R(1, 2) = s0 * s2 - s1 * c0 * c2;
	R(2, 0) = -s2 * c1;
	R(2, 1) = s1;

	return R;
}

float kahanSummation(float sum_previous, float input, float &accumulator)
{
	const float y = input - accumulator;
	const float t = sum_previous + y;
	accumulator = (t - sum_previous) - y;
	return t;
}

matrix::Dcmf quatToInverseRotMat(const matrix::Quatf &quat)
{
	const float q00 = quat(0) * quat(0);
	const float q11 = quat(1) * quat(1);
	const float q22 = quat(2) * quat(2);
	const float q33 = quat(3) * quat(3);
	const float q01 = quat(0) * quat(1);
	const float q02 = quat(0) * quat(2);
	const float q03 = quat(0) * quat(3);
	const float q12 = quat(1) * quat(2);
	const float q13 = quat(1) * quat(3);
	const float q23 = quat(2) * quat(3);

	matrix::Dcmf dcm;
	dcm(0, 0) = q00 + q11 - q22 - q33;
	dcm(1, 1) = q00 - q11 + q22 - q33;
	dcm(2, 2) = q00 - q11 - q22 + q33;
	dcm(1, 0) = 2.0f * (q12 - q03);
	dcm(2, 0) = 2.0f * (q13 + q02);
	dcm(0, 1) = 2.0f * (q12 + q03);
	dcm(2, 1) = 2.0f * (q23 - q01);
	dcm(0, 2) = 2.0f * (q13 - q02);
	dcm(1, 2) = 2.0f * (q23 + q01);

	return dcm;
}

float getEuler321Yaw(const matrix::Quatf &q)
{
	// Values from yaw_input_321.c file produced by
	// https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/quat2yaw321.m
	const float a = 2.f * (q(0) * q(3) + q(1) * q(2));
	const float b = sq(q(0)) + sq(q(1)) - sq(q(2)) - sq(q(3));
	return atan2f(a, b);
}

float getEuler312Yaw(const matrix::Quatf &q)
{
	// Values from yaw_input_312.c file produced by
	// https://github.com/PX4/ecl/blob/master/matlab/scripts/Inertial%20Nav%20EKF/quat2yaw312.m
	const float a = 2.f * (q(0) * q(3) - q(1) * q(2));
	const float b = sq(q(0)) - sq(q(1)) + sq(q(2)) - sq(q(3));
	return atan2f(a, b);
}

matrix::Dcmf updateEuler321YawInRotMat(float yaw, const matrix::Dcmf &rot_in)
{
	matrix::Eulerf euler321(rot_in);
	euler321(2) = yaw;
	return matrix::Dcmf(euler321);
}

matrix::Dcmf updateEuler312YawInRotMat(float yaw, const matrix::Dcmf &rot_in)
{
	const matrix::Vector3f rotVec312(yaw,  // yaw
					 asinf(rot_in(2, 1)),  // roll
					 atan2f(-rot_in(2, 0), rot_in(2, 2)));  // pitch
	return taitBryan312ToRotMat(rotVec312);
}

matrix::Dcmf updateYawInRotMat(float yaw, const matrix::Dcmf &rot_in)
{
	return shouldUse321RotationSequence(rot_in) ?
	       updateEuler321YawInRotMat(yaw, rot_in) :
	       updateEuler312YawInRotMat(yaw, rot_in);
}
