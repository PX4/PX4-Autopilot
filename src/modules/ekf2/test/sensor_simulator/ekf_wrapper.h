/****************************************************************************
 *
 *   Copyright (c) 2019 ECL Development Team. All rights reserved.
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
 * Wrapper class for Ekf object to set behavior or check status
 * @author Kamil Ritz <ka.ritz@hotmail.com>
 */
#ifndef EKF_EKF_WRAPPER_H
#define EKF_EKF_WRAPPER_H

#include <memory>
#include "EKF/ekf.h"
#include "EKF/estimator_interface.h"

class EkfWrapper
{
public:
	EkfWrapper(std::shared_ptr<Ekf> ekf);
	~EkfWrapper();


	void setBaroHeight();
	bool isIntendingBaroHeightFusion() const;

	void setGpsHeight();
	bool isIntendingGpsHeightFusion() const;

	void setRangeHeight();
	bool isIntendingRangeHeightFusion() const;

	void setVisionHeight();
	bool isIntendingVisionHeightFusion() const;

	void enableGpsFusion();
	void disableGpsFusion();
	bool isIntendingGpsFusion() const;

	void enableGpsHeadingFusion();
	void disableGpsHeadingFusion();
	bool isIntendingGpsHeadingFusion() const;

	void enableFlowFusion();
	void disableFlowFusion();
	bool isIntendingFlowFusion() const;
	void setFlowOffset(const matrix::Vector3f &offset);

	void enableExternalVisionPositionFusion();
	void disableExternalVisionPositionFusion();
	bool isIntendingExternalVisionPositionFusion() const;

	void enableExternalVisionVelocityFusion();
	void disableExternalVisionVelocityFusion();
	bool isIntendingExternalVisionVelocityFusion() const;

	void enableExternalVisionHeadingFusion();
	void disableExternalVisionHeadingFusion();
	bool isIntendingExternalVisionHeadingFusion() const;

	bool isIntendingMagHeadingFusion() const;
	bool isIntendingMag3DFusion() const;
	void setMagFuseTypeNone();

	void enableExternalVisionAlignment();
	void disableExternalVisionAlignment();

	bool isWindVelocityEstimated() const;

	void enableTerrainRngFusion();
	void disableTerrainRngFusion();
	bool isIntendingTerrainRngFusion() const;

	void enableTerrainFlowFusion();
	void disableTerrainFlowFusion();
	bool isIntendingTerrainFlowFusion() const;

	Eulerf getEulerAngles() const;
	float getYawAngle() const;
	matrix::Vector<float, 4> getQuaternionVariance() const;
	int getQuaternionResetCounter() const;

	matrix::Vector3f getDeltaVelBiasVariance() const;

private:
	std::shared_ptr<Ekf> _ekf;

	// Pointer to Ekf internal param struct
	parameters *_ekf_params;

};
#endif // !EKF_EKF_WRAPPER_H
