/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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

/**
 * @file px4_eigen.h
 *
 * @brief Compatibility header to make Eigen compile on the PX4 stack
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Nuno Marques <n.marques21@hotmail.com>
 **/

#pragma once

#include <cmath>
#include <stdio.h>
#pragma GCC diagnostic push
#ifndef RAND_MAX
#define RAND_MAX __RAND_MAX
#endif
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wfloat-equal"
#ifndef _GLIBCXX_USE_C99_FP_MACROS_DYNAMIC
#define _GLIBCXX_USE_C99_FP_MACROS_DYNAMIC 1
#endif

#include <eigen/Eigen/Eigen>
#include <mathlib/mathlib.h>
// #include <eigen/unsupported/Eigen/CXX11/Tensor>

#pragma GCC diagnostic pop

/**
 * @brief
 *	Prints an Eigen::Matrix to stdout
 **/
template<typename T>
static void printEigen(const Eigen::MatrixBase<T> &b)
{
	for (int i = 0; i < b.rows(); ++i) {
		printf("(");

		for (int j = 0; j < b.cols(); ++j) {
			if (j > 0) {
				printf(",");
			}

			printf("%.3f", static_cast<double>(b(i, j)));
		}

		printf(")%s\n", i + 1 < b.rows() ? "," : "");
	}
}

/*
 * Custom Eigen methods
 */

/**
 * @brief
 *	Construct new Eigen::Quaternionf from euler angles
 *	Right order is YPR.
 */
static Eigen::Quaternionf quatFromEuler(const Eigen::Vector3f &rpy){
	return Eigen::Quaternionf(
		Eigen::AngleAxisf(rpy.z(), Eigen::Vector3f::UnitZ()) *
		Eigen::AngleAxisf(rpy.y(), Eigen::Vector3f::UnitY()) *
		Eigen::AngleAxisf(rpy.x(), Eigen::Vector3f::UnitX())
		);
}

/**
 * @brief
 *	Construct new Eigen::Matrix3f from euler angles
 *	Right order is YPR.
 */
static Eigen::Matrix3f matrixFromEuler(const Eigen::Vector3f &rpy){
	return Eigen::Matrix3f(
		Eigen::AngleAxisf(rpy.z(), Eigen::Vector3f::UnitZ()) *
		Eigen::AngleAxisf(rpy.y(), Eigen::Vector3f::UnitY()) *
		Eigen::AngleAxisf(rpy.x(), Eigen::Vector3f::UnitX())
		);
}

/**
 * @brief
 *	Construct new Eigen::Vector3f of euler angles from quaternion
 *	Right order is YPR.
 */
static Eigen::Vector3f eulerFromQuat(const Eigen::Quaternionf &q){
	return q.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
}


 /**
  * @brief
  *	Construct new Eigen::Vector3f of euler angles from rotation matrix
  *	Right order is YPR.
  */
static Eigen::Vector3f eulerFromRot(const Eigen::Matrix3f &rot){
	 return rot.eulerAngles(2, 1, 0).reverse();
}

/**
 * @brief
 *	Adjust PX4 math::quaternion to Eigen::Quaternionf
 */
static Eigen::Quaternionf eigenqFromPx4q(const math::Quaternion &q){
	return Eigen::Quaternionf(q.data[0], q.data[1], q.data[2], q.data[3]);
}

/**
 * @brief
 *	Adjust Eigen::Quaternionf to PX4 math::Quaternion
 */
static math::Quaternion px4qFromEigenq(const Eigen::Quaternionf &q){
	return math::Quaternion(q.w(), q.x(), q.y(), q.z());
}

/**
 * @brief
 *	Adjust PX4 math::Matrix<3,3> RotationMatrix to Eigen::Matrix3f RotationMatrix
 */
static Eigen::Matrix3f eigenrFromPx4r(const math::Matrix<3,3> &rot){
	math::Quaternion q;
	q.from_dcm(rot);
	return Eigen::Matrix3f(eigenqFromPx4q(q).toRotationMatrix());
}

/**
 * @brief
 *	Adjust Eigen::Matrix3f RotationMatrix to PX4 math::Matrix<3,3> RotationMatrix
 */
static math::Matrix<3,3> px4rFromEigenr(const Eigen::Matrix3f &rot){
	return math::Matrix<3,3>(px4qFromEigenq(Eigen::Quaternionf(rot)).to_dcm());
}

/**
 * @brief
 *	Create Eigen::Quaternionf from RotationMatrix through DCM.
 *  Commented since it is not required but kept as an alternative to q(R) constructor
 */
/*static Eigen::Quaternionf eigenqFromDcm(const Eigen::Matrix3f &dcm){
	math::Quaternion q;

	float tr = dcm(0,0) + dcm(1,1) + dcm(2,2);
	if (tr > 0.0f) {
		float s = sqrtf(tr + 1.0f);
		q.data[0] = s * 0.5f;
		s = 0.5f / s;
		q.data[1] = (dcm(2,1) - dcm(1,2)) * s;
		q.data[2] = (dcm(0,2) - dcm(2,0)) * s;
		q.data[3] = (dcm(1,0) - dcm(0,1)) * s;
	} else {
		// Find maximum diagonal element in dcm
		// store index in dcm_i //
		int dcm_i = 0;
		for (int i = 1; i < 3; i++) {
			if (dcm(i,i) > dcm(dcm_i,dcm_i)) {
				dcm_i = i;
			}
		}
		int dcm_j = (dcm_i + 1) % 3;
		int dcm_k = (dcm_i + 2) % 3;
		float s = sqrtf((dcm(dcm_i,dcm_i) - dcm(dcm_j,dcm_j) -
		dcm(dcm_k,dcm_k)) + 1.0f);
		q.data[dcm_i + 1] = s * 0.5f;
		s = 0.5f / s;
		q.data[dcm_j + 1] = (dcm(dcm_i,dcm_j) + dcm(dcm_j,dcm_i)) * s;
		q.data[dcm_k + 1] = (dcm(dcm_k,dcm_i) + dcm(dcm_i,dcm_k)) * s;
		q.data[0] = (dcm(dcm_k,dcm_j) - dcm(dcm_j,dcm_k)) * s;
	}
	return eigenqFromPx4q(q);
}*/
