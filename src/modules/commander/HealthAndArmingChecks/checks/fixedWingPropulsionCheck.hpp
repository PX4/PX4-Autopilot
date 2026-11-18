/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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


#pragma once

#include "../Common.hpp"

#include <lib/hysteresis/hysteresis.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>

/**
 * Fixed-wing propulsion failure: in fixed-wing flight, the battery current stays too low for a throttle above
 * COM_EF_THROT. It is latched until reboot or until the check is disabled.
 */
class FixedWingPropulsionChecks : public HealthAndArmingCheckBase
{
public:
	FixedWingPropulsionChecks() = default;
	~FixedWingPropulsionChecks() = default;

	void checkAndReport(const Context &context, Report &reporter) override;

private:
	bool isCurrentTooLow(const Context &context);

	uORB::SubscriptionMultiArray<battery_status_s, battery_status_s::MAX_INSTANCES> _battery_status_subs{ORB_ID::battery_status};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_thrust_setpoint_sub{ORB_ID(vehicle_thrust_setpoint), 0};

	systemlib::Hysteresis _low_current_hysteresis{false};
	hrt_abstime _throttle_high_since{0}; ///< battery samples from before show the current of a lower throttle
	bool _failure_detected{false};

	DEFINE_PARAMETERS_CUSTOM_PARENT(HealthAndArmingCheckBase,
					(ParamFloat<px4::params::COM_EF_CURR>) _param_com_ef_curr,
					(ParamFloat<px4::params::COM_EF_THROT>) _param_com_ef_throt,
					(ParamFloat<px4::params::COM_EF_TIME>) _param_com_ef_time
				       )
};
