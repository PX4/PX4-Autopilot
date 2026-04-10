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

#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/sensor_gps.h>

class GpsRedundancyChecks : public HealthAndArmingCheckBase
{
public:
	GpsRedundancyChecks() = default;
	~GpsRedundancyChecks() = default;

	void checkAndReport(const Context &context, Report &reporter) override;

private:
	static constexpr int GPS_MAX_INSTANCES = 2;

	uORB::SubscriptionMultiArray<sensor_gps_s, GPS_MAX_INSTANCES> _sensor_gps_sub{ORB_ID::sensor_gps};

	int _peak_fixed_count{0};

	hrt_abstime _divergence_since{0};

	DEFINE_PARAMETERS_CUSTOM_PARENT(HealthAndArmingCheckBase,
					(ParamInt<px4::params::SYS_HAS_NUM_GNSS>)   _param_sys_has_num_gnss,
					(ParamInt<px4::params::COM_GPS_LOSS_ACT>)  _param_com_gnss_loss_act,
					(ParamFloat<px4::params::SENS_GPS0_OFFX>)  _param_gps0_offx,
					(ParamFloat<px4::params::SENS_GPS0_OFFY>)  _param_gps0_offy,
					(ParamFloat<px4::params::SENS_GPS1_OFFX>)  _param_gps1_offx,
					(ParamFloat<px4::params::SENS_GPS1_OFFY>)  _param_gps1_offy
				       )
};
