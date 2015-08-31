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
 * @file quaternion.h
 *
 * @brief Header for custom Eigen functions with transformations and conversions for
 * 			quaternions, rotation matrices and euler angles
 * @author Nuno Marques <n.marques21@hotmail.com>
 */

#ifndef QUATERNION_H
#define QUATERNION_H

#include <eigen/px4_eigen.h>

namespace transform {

extern Eigen::Quaternionf quatFromEuler(const Eigen::Vector3f &rpy) __EXPORT;
extern Eigen::Matrix3f matrixFromEuler(const Eigen::Vector3f &rpy) __EXPORT;
extern Eigen::Vector3f eulerFromQuat(const Eigen::Quaternionf &q) __EXPORT;
extern Eigen::Vector3f eulerFromRot(const Eigen::Matrix3f &rot) __EXPORT;
extern Eigen::Quaternionf eigenqFromPx4q(const math::Quaternion &q) __EXPORT;
extern math::Quaternion px4qFromEigenq(const Eigen::Quaternionf &q) __EXPORT;
extern Eigen::Matrix3f eigenrFromPx4r(const math::Matrix<3,3> &rot) __EXPORT;
extern math::Matrix<3,3> px4rFromEigenr(const Eigen::Matrix3f &rot) __EXPORT;
extern Eigen::Quaternionf eigenqFromDcm(const Eigen::Matrix3f &dcm) __EXPORT;

}; // namespace transform

#endif // QUATERNION_H
