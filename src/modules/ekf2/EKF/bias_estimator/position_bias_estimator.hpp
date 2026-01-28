/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file position_bias_estimator.hpp
 */

#ifndef EKF_POSITION_BIAS_ESTIMATOR_HPP
#define EKF_POSITION_BIAS_ESTIMATOR_HPP

#include "bias_estimator.hpp"

class PositionBiasEstimator
{
public:
	PositionBiasEstimator(PositionSensor sensor, const PositionSensor &sensor_ref):
		_sensor(sensor),
		_sensor_ref(sensor_ref)
	{}
	virtual ~PositionBiasEstimator() = default;

	bool fusionActive() const { return _is_sensor_fusion_active; }

	void setFusionActive() { _is_sensor_fusion_active = true; }
	void setFusionInactive() { _is_sensor_fusion_active = false; }

	void predict(float dt)
	{
		if ((_sensor_ref != _sensor) && _is_sensor_fusion_active) {
			_bias[0].predict(dt);
			_bias[1].predict(dt);
		}
	}

	void fuseBias(Vector2f bias, Vector2f bias_var)
	{
		if ((_sensor_ref != _sensor) && _is_sensor_fusion_active) {
			_bias[0].fuseBias(bias(0), bias_var(0));
			_bias[1].fuseBias(bias(1), bias_var(1));
		}
	}

	void setBias(const Vector2f &bias)
	{
		_bias[0].setBias(bias(0));
		_bias[1].setBias(bias(1));
	}

	void setProcessNoiseSpectralDensity(float nsd)
	{
		_bias[0].setProcessNoiseSpectralDensity(nsd);
		_bias[1].setProcessNoiseSpectralDensity(nsd);
	}
	// void setBiasStdDev(float state_noise) { _state_var = state_noise * state_noise; }
	// void setInnovGate(float gate_size) { _gate_size = gate_size; }

	void setMaxStateNoise(const Vector2f &max_noise)
	{
		_bias[0].setMaxStateNoise(max_noise(0));
		_bias[1].setMaxStateNoise(max_noise(1));
	}

	Vector2f getBias() const { return Vector2f(_bias[0].getBias(), _bias[1].getBias()); }
	float getBias(int index) const { return _bias[index].getBias(); }

	Vector2f getBiasVar() const { return Vector2f(_bias[0].getBiasVar(), _bias[1].getBiasVar()); }
	float getBiasVar(int index) const { return _bias[index].getBiasVar(); }

	const BiasEstimator::status &getStatus(int index) const { return _bias[index].getStatus(); }

	void reset()
	{
		_bias[0].reset();
		_bias[1].reset();
	}

private:
	BiasEstimator _bias[2] {};

	const PositionSensor _sensor;
	const PositionSensor &_sensor_ref;

	bool _is_sensor_fusion_active{false}; // TODO: replace by const ref and remove setter when migrating _control_status.flags from union to bool
};

#endif // !EKF_POSITION_BIAS_ESTIMATOR_HPP
