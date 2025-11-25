/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file CombinedControllerConfigurationHandler.cpp
 */

#include "ControllerConfigurationHandler.hpp"
#include <drivers/drv_hrt.h>

using namespace time_literals;


void CombinedControllerConfigurationHandler::update(const hrt_abstime now)
{
	_longitudinal_updated = floatValueChanged(_longitudinal_configuration_current_cycle.pitch_min,
				_longitudinal_publisher.get().pitch_min);
	_longitudinal_updated |= floatValueChanged(_longitudinal_configuration_current_cycle.pitch_max,
				 _longitudinal_publisher.get().pitch_max);
	_longitudinal_updated |= floatValueChanged(_longitudinal_configuration_current_cycle.throttle_min,
				 _longitudinal_publisher.get().throttle_min);
	_longitudinal_updated |= floatValueChanged(_longitudinal_configuration_current_cycle.throttle_max,
				 _longitudinal_publisher.get().throttle_max);
	_longitudinal_updated |= floatValueChanged(_longitudinal_configuration_current_cycle.speed_weight,
				 _longitudinal_publisher.get().speed_weight);
	_longitudinal_updated |= floatValueChanged(_longitudinal_configuration_current_cycle.climb_rate_target,
				 _longitudinal_publisher.get().climb_rate_target);
	_longitudinal_updated |= floatValueChanged(_longitudinal_configuration_current_cycle.sink_rate_target,
				 _longitudinal_publisher.get().sink_rate_target);
	_longitudinal_updated |= booleanValueChanged(_longitudinal_configuration_current_cycle.enforce_low_height_condition,
				 _longitudinal_publisher.get().enforce_low_height_condition);
	_longitudinal_updated |= booleanValueChanged(_longitudinal_configuration_current_cycle.disable_underspeed_protection,
				 _longitudinal_publisher.get().disable_underspeed_protection);

	_lateral_updated |= floatValueChanged(_lateral_configuration_current_cycle.lateral_accel_max,
					      _lateral_publisher.get().lateral_accel_max);

	if (_longitudinal_updated || now - _time_last_longitudinal_publish > 1_s) {
		_longitudinal_configuration_current_cycle.timestamp = now;
		_longitudinal_publisher.update(_longitudinal_configuration_current_cycle);
		_time_last_longitudinal_publish = _longitudinal_configuration_current_cycle.timestamp;
	}

	if (_lateral_updated || now - _time_last_lateral_publish > 1_s) {
		_lateral_configuration_current_cycle.timestamp = now;
		_lateral_publisher.update(_lateral_configuration_current_cycle);
		_time_last_lateral_publish = _lateral_configuration_current_cycle.timestamp;
	}

	_longitudinal_updated = _lateral_updated = false;
	_longitudinal_configuration_current_cycle = empty_longitudinal_control_configuration;
	_lateral_configuration_current_cycle = empty_lateral_control_configuration;
}

void CombinedControllerConfigurationHandler::setThrottleMax(float throttle_max)
{
	_longitudinal_configuration_current_cycle.throttle_max = throttle_max;
}

void CombinedControllerConfigurationHandler::setThrottleMin(float throttle_min)
{
	_longitudinal_configuration_current_cycle.throttle_min = throttle_min;
}

void CombinedControllerConfigurationHandler::setSpeedWeight(float speed_weight)
{
	_longitudinal_configuration_current_cycle.speed_weight = speed_weight;
}

void CombinedControllerConfigurationHandler::setPitchMin(const float pitch_min)
{
	_longitudinal_configuration_current_cycle.pitch_min = pitch_min;
}

void CombinedControllerConfigurationHandler::setPitchMax(const float pitch_max)
{
	_longitudinal_configuration_current_cycle.pitch_max = pitch_max;
}

void CombinedControllerConfigurationHandler::setClimbRateTarget(float climbrate_target)
{
	_longitudinal_configuration_current_cycle.climb_rate_target = climbrate_target;
}

void CombinedControllerConfigurationHandler::setDisableUnderspeedProtection(bool disable)
{
	_longitudinal_configuration_current_cycle.disable_underspeed_protection = disable;
}

void CombinedControllerConfigurationHandler::setSinkRateTarget(const float sinkrate_target)
{
	_longitudinal_configuration_current_cycle.sink_rate_target = sinkrate_target;
}

void CombinedControllerConfigurationHandler::setLateralAccelMax(float lateral_accel_max)
{
	_lateral_configuration_current_cycle.lateral_accel_max = lateral_accel_max;
}

void CombinedControllerConfigurationHandler::setEnforceLowHeightCondition(bool low_height_condition)
{
	_longitudinal_configuration_current_cycle.enforce_low_height_condition = low_height_condition;
}

void CombinedControllerConfigurationHandler::resetLastPublishTime()
{
	_time_last_longitudinal_publish = _time_last_lateral_publish = 0;
}
