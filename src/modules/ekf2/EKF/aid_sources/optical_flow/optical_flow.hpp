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

#ifndef EKF_OPTICAL_FLOW_HPP
#define EKF_OPTICAL_FLOW_HPP

#include "../../common.h"

#if defined(CONFIG_EKF2_OPTICAL_FLOW) && defined(MODULE_NAME)

#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/ekf2_timestamps.h>
#include <uORB/topics/estimator_aid_source2d.h>
#include <uORB/topics/vehicle_optical_flow.h>
#include <uORB/topics/vehicle_optical_flow_vel.h>

class Ekf;

class OpticalFlow
{
public:
	OpticalFlow()
	{
		for (uint8_t i = 0; i < MAX_OF_INSTANCES; i++) {
			_slots[i].sub = uORB::Subscription(ORB_ID(vehicle_optical_flow), i);
		}
	}

	void initParameters(Ekf &ekf);
	void updateParameters(Ekf &ekf);
	float maxEnabledDelayMs(Ekf &ekf) const;

	void advertiseEnabledPublications(const Ekf &ekf);
	void updateSamples(Ekf &ekf, ekf2_timestamps_s &ekf2_timestamps, const hrt_abstime &last_range_sensor_update);
	void publishAidSourceStatus(const Ekf &ekf, const hrt_abstime &timestamp, uint8_t estimator_instance,
				    bool replay_mode);
	void publishFlowVel(const Ekf &ekf, const hrt_abstime &timestamp, bool replay_mode);

private:
	void resolveTuningHandles(uint8_t i);

	struct ParamHandles {
		param_t ctrl{PARAM_INVALID};
		param_t delay{PARAM_INVALID};
		param_t gyr_src{PARAM_INVALID};
		param_t n_min{PARAM_INVALID};
		param_t n_max{PARAM_INVALID};
		param_t qmin{PARAM_INVALID};
		param_t qmin_gnd{PARAM_INVALID};
		param_t gate{PARAM_INVALID};
	};
	struct Slot {
		uORB::Subscription sub{ORB_ID(vehicle_optical_flow)};
		uORB::PublicationMulti<vehicle_optical_flow_vel_s> flow_vel_pub{ORB_ID(estimator_optical_flow_vel)};
		uORB::PublicationMulti<estimator_aid_source2d_s> aid_src_pub{ORB_ID(estimator_aid_src_optical_flow)};
		ParamHandles param_handles{};
		hrt_abstime status_pub_last{};
		hrt_abstime flow_vel_pub_last{};
	};

	Slot _slots[MAX_OF_INSTANCES] {};

#if defined(CONFIG_EKF2_RANGE_FINDER)
	int8_t _range_instance {-1}; ///< first instance providing a distance, used as range finder fallback
#endif // CONFIG_EKF2_RANGE_FINDER
};

#endif // CONFIG_EKF2_OPTICAL_FLOW && MODULE_NAME

#endif // !EKF_OPTICAL_FLOW_HPP
