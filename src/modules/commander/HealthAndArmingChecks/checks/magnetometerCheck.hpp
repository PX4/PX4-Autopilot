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

#pragma once

#include "../Common.hpp"

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/sensor_preflight_mag.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/estimator_status.h>
#include <lib/sensor_calibration/Magnetometer.hpp>

class MagnetometerChecks : public HealthAndArmingCheckBase
{
public:
	MagnetometerChecks() = default;
	~MagnetometerChecks() = default;

	void checkAndReport(const Context &context, Report &reporter) override;

private:
	bool isMagRequired(int instance, bool &mag_fault);
	void consistencyCheck(const Context &context, Report &reporter);

	uORB::SubscriptionMultiArray<sensor_mag_s, calibration::Magnetometer::MAX_SENSOR_COUNT> _sensor_mag_sub{ORB_ID::sensor_mag};
	uORB::SubscriptionMultiArray<estimator_status_s> _estimator_status_sub{ORB_ID::estimator_status};

	uORB::Subscription _sensor_preflight_mag_sub{ORB_ID(sensor_preflight_mag)};

	DEFINE_PARAMETERS_CUSTOM_PARENT(HealthAndArmingCheckBase,
					(ParamInt<px4::params::SYS_HAS_MAG>) _param_sys_has_mag,
					(ParamInt<px4::params::COM_ARM_MAG_ANG>) _param_com_arm_mag_ang
				       )
};
