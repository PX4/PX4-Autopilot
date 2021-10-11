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
 * Feeds Ekf external vision data
 * @author Kamil Ritz <ka.ritz@hotmail.com>
 */
#ifndef EKF_VIO_H
#define EKF_VIO_H

#include "sensor.h"

namespace sensor_simulator
{
namespace sensor
{

class Vio: public Sensor
{
public:
	Vio(std::shared_ptr<Ekf> ekf);
	~Vio();

	void setData(const extVisionSample &vio_data);
	void setVelocityVariance(const Vector3f &velVar);
	void setVelocityCovariance(const Matrix3f &velCov);
	void setPositionVariance(const Vector3f &posVar);
	void setAngularVariance(float angVar);
	void setVelocity(const Vector3f &vel);
	void setPosition(const Vector3f &pos);
	void setOrientation(const Quatf &quat);
	void setVelocityFrameToLocal();
	void setVelocityFrameToBody();

	extVisionSample dataAtRest();

private:
	extVisionSample _vio_data;

	void send(uint64_t time) override;

};

} // namespace sensor
} // namespace sensor_simulator
#endif // !EKF_VIO_H
