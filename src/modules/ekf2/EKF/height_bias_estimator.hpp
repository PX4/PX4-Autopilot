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
 * @file height_bias_estimator.hpp
 */

#pragma once

#include "bias_estimator.hpp"

class HeightBiasEstimator: public BiasEstimator
{
public:
	HeightBiasEstimator(uint8_t sensor, const uint8_t &sensor_ref):
		BiasEstimator(0.f, 0.f),
		_sensor(sensor),
		_sensor_ref(sensor_ref)
	{}
	virtual ~HeightBiasEstimator() = default;

	void setFusionActive() { _is_sensor_fusion_active = true; }
	void setFusionInactive() { _is_sensor_fusion_active = false; }

	virtual void predict(float dt) override
	{
		if ((_sensor_ref != _sensor) && _is_sensor_fusion_active) {
			BiasEstimator::predict(dt);
		}
	}

	virtual void fuseBias(float bias, float bias_var) override
	{
		if ((_sensor_ref != _sensor) && _is_sensor_fusion_active) {
			BiasEstimator::fuseBias(bias, bias_var);
		}
	}

private:
	const uint8_t _sensor;
	const uint8_t &_sensor_ref;

	bool _is_sensor_fusion_active{false}; // TODO: replace by const ref and remove setter when migrating _control_status.flags from union to bool
};
