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


#ifndef EKF_GNSS_HPP
#define EKF_GNSS_HPP

#include "../../common.h"

#if defined(CONFIG_EKF2_GNSS) && defined(MODULE_NAME)

#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <mathlib/math/filter/AlphaFilter.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/estimator_aid_source2d.h>
#include <uORB/topics/estimator_aid_source3d.h>
#include <uORB/topics/sensor_gps.h>

class Ekf;

class Gnss
{
public:
	Gnss()
	{
		for (uint8_t i = 0; i < estimator::MAX_GNSS_INSTANCES; i++) {
			_slots[i].sub = uORB::Subscription(ORB_ID(vehicle_gps_position), i);
		}
	}

	void initParameters(Ekf &ekf);
	void updateParameters(Ekf &ekf);

	// true if any receiver slot enables the given aiding, any aiding if bit is 0
	bool anySlotEnabled(const Ekf &ekf, int32_t bit = 0) const;

	void advertiseEnabledPublications(const Ekf &ekf);
	void updateSamples(Ekf &ekf, float yaw_offset_deg);
	void publishAidSourceStatus(const Ekf &ekf, const hrt_abstime &timestamp, uint8_t estimator_instance, bool replay_mode);

	// height offset between AMSL and ellipsoid
	float geoidHeight() const { return _geoid_height_lpf.getState(); }

private:
	struct Slot {
		uORB::Subscription sub{ORB_ID(vehicle_gps_position)};
		uORB::PublicationMulti<estimator_aid_source2d_s> aid_src_pos_pub{ORB_ID(estimator_aid_src_gnss_pos)};
		uORB::PublicationMulti<estimator_aid_source3d_s> aid_src_vel_pub{ORB_ID(estimator_aid_src_gnss_vel)};
		param_t ctrl_handle{PARAM_INVALID};
		hrt_abstime pos_pub_last{};
		hrt_abstime vel_pub_last{};
	};

	Slot _slots[estimator::MAX_GNSS_INSTANCES] {};

	hrt_abstime _last_geoid_height_update_us{0};
	static constexpr hrt_abstime kGeoidHeightLpfTimeConstant = 10000000; // 10 s
	AlphaFilter<float> _geoid_height_lpf;
};

#endif // CONFIG_EKF2_GNSS && MODULE_NAME

#endif // !EKF_GNSS_HPP
