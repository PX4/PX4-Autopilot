/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file quaternion.cpp
 *
 * @brief Custom Eigen functions with transformations and conversions for
 * 			quaternions, rotation matrices and euler angles
 * @author Nuno Marques <n.marques21@hotmail.com>
 */

#include <eigen/transform/quaternion.h>

namespace transform {
/**
 * @brief Construct new Eigen::Quaternionf from euler angles
 *
 * Right order is YPR.
 */
Eigen::Quaternionf quatFromEuler(const Eigen::Vector3f &rpy){
	return Eigen::Quaternionf(
		Eigen::AngleAxisf(rpy.z(), Eigen::Vector3f::UnitZ()) *
		Eigen::AngleAxisf(rpy.y(), Eigen::Vector3f::UnitY()) *
		Eigen::AngleAxisf(rpy.x(), Eigen::Vector3f::UnitX())
		);
}

/**
 * @brief Construct new Eigen::Matrix3f from Euler angles
 *
 * Right order is YPR.
 */
Eigen::Matrix3f matrixFromEuler(const Eigen::Vector3f &rpy){
	return Eigen::Matrix3f(
		Eigen::AngleAxisf(rpy.z(), Eigen::Vector3f::UnitZ()) *
		Eigen::AngleAxisf(rpy.y(), Eigen::Vector3f::UnitY()) *
		Eigen::AngleAxisf(rpy.x(), Eigen::Vector3f::UnitX())
		);
}

/**
 * @brief Construct new Eigen::Vector3f of Euler angles from quaternion
 *
 * Right order is YPR.
 */
Eigen::Vector3f eulerFromQuat(const Eigen::Quaternionf &q){
	return q.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
}


 /**
  * @brief Construct new Eigen::Vector3f of Euler angles from Rotation Matrix
  *
  *	Right order is YPR.
  */
Eigen::Vector3f eulerFromRot(const Eigen::Matrix3f &rot){
	 return rot.eulerAngles(2, 1, 0).reverse();
}

/**
 * @brief Adjust PX4 math::!uaternion to Eigen::Quaternionf
 */
Eigen::Quaternionf eigenqFromPx4q(const math::Quaternion &q){
	return Eigen::Quaternionf(q.data[0], q.data[1], q.data[2], q.data[3]);
}

/**
 * @brief Adjust Eigen::Quaternionf to PX4 math::Quaternion
 */
math::Quaternion px4qFromEigenq(const Eigen::Quaternionf &q){
	return math::Quaternion(q.w(), q.x(), q.y(), q.z());
}

/**
 * @brief Adjust PX4 math::Matrix<3,3> Rotation Matrix to Eigen::Matrix3f Rotation Matrix
 */
Eigen::Matrix3f eigenrFromPx4r(const math::Matrix<3,3> &rot){
	math::Quaternion q;
	q.from_dcm(rot);
	return Eigen::Matrix3f(eigenqFromPx4q(q).toRotationMatrix());
}

/**
 * @brief Adjust Eigen::Matrix3f Rotation Matrix to PX4 math::Matrix<3,3> Rotation Matrix
 */
math::Matrix<3,3> px4rFromEigenr(const Eigen::Matrix3f &rot){
	return math::Matrix<3,3>(px4qFromEigenq(Eigen::Quaternionf(rot)).to_dcm());
}

/**
 * @brief Create Eigen::Quaternionf from a Rotation Matrix through DCM
 * 
 * Alternative to Eigen::Quaternionf q(R) constructor.
 */
Eigen::Quaternionf eigenqFromDcm(const Eigen::Matrix3f &dcm){
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
}
};