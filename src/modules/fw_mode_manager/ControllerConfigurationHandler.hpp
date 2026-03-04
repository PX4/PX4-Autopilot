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
 * @file CombinedControllerConfigurationHandler.hpp
 */
#ifndef PX4_CONTROLLERCONFIGURATIONHANDLER_HPP
#define PX4_CONTROLLERCONFIGURATIONHANDLER_HPP

#include <uORB/topics/longitudinal_control_configuration.h>
#include <uORB/topics/lateral_control_configuration.h>
#include <uORB/Publication.hpp>
#include <lib/mathlib/mathlib.h>

static constexpr longitudinal_control_configuration_s empty_longitudinal_control_configuration = {.timestamp = 0, .pitch_min = NAN, .pitch_max = NAN, .throttle_min = NAN, .throttle_max = NAN, .climb_rate_target = NAN, .sink_rate_target = NAN, .speed_weight = NAN, .enforce_low_height_condition = false, .disable_underspeed_protection = false };
static constexpr lateral_control_configuration_s empty_lateral_control_configuration = {.timestamp = 0, .lateral_accel_max = NAN};


class CombinedControllerConfigurationHandler
{
public:
	CombinedControllerConfigurationHandler() = default;
	~CombinedControllerConfigurationHandler() = default;

	void update(const hrt_abstime now);

	void setEnforceLowHeightCondition(bool low_height_condition);

	void setThrottleMax(float throttle_max);

	void setThrottleMin(float throttle_min);

	void setSpeedWeight(float speed_weight);

	void setPitchMin(const float pitch_min);

	void setPitchMax(const float pitch_max);

	void setClimbRateTarget(float climbrate_target);

	void setDisableUnderspeedProtection(bool disable);

	void setSinkRateTarget(const float sinkrate_target);

	void setLateralAccelMax(float lateral_accel_max);

	void resetLastPublishTime();

private:
	bool booleanValueChanged(bool new_value, bool current_value)
	{
		return current_value != new_value;
	}

	bool floatValueChanged(float new_value, float current_value)
	{
		bool  diff;

		if (PX4_ISFINITE(new_value)) {
			diff = PX4_ISFINITE(current_value) ? (fabsf(new_value - current_value) > FLT_EPSILON) : true;

		} else {
			diff = PX4_ISFINITE(current_value);
		}

		return diff;
	}

	bool _lateral_updated{false};
	bool _longitudinal_updated{false};

	hrt_abstime _time_last_longitudinal_publish{0};
	hrt_abstime _time_last_lateral_publish{0};

	uORB::PublicationData<lateral_control_configuration_s> _lateral_publisher{ORB_ID(lateral_control_configuration)};
	uORB::PublicationData<longitudinal_control_configuration_s> _longitudinal_publisher{ORB_ID(longitudinal_control_configuration)};

	lateral_control_configuration_s _lateral_configuration_current_cycle{empty_lateral_control_configuration};
	longitudinal_control_configuration_s _longitudinal_configuration_current_cycle {empty_longitudinal_control_configuration};
};

#endif //PX4_CONTROLLERCONFIGURATIONHANDLER_HPP
