/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#ifndef EKF_AIRSPEED2D_HPP
#define EKF_AIRSPEED2D_HPP

#if defined(CONFIG_EKF2_AIRSPEED2D) && defined(MODULE_NAME)

#include "EstimatorAidSource.hpp"

#include "../RingBuffer.h"

#include <px4_platform_common/module_params.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/estimator_aid_source2d.h>
#include <uORB/topics/sensor_airflow.h>

class Airspeed2D : public EstimatorAidSource, public ModuleParams
{
public:
	Airspeed2D() : ModuleParams(nullptr) {}
	virtual ~Airspeed2D() = default;

	void reset() override {};
	bool update(Ekf &ekf, const estimator::imuSample &imu_delayed) override;

private:
	struct AirflowSample {
		uint64_t time_us{};
		float speed{};
		float direction_rad{};
	};

	RingBuffer<AirflowSample> _airflow_buffer{20}; // TODO: size with _obs_buffer_length and actual publication rate
	uint64_t _time_last_buffer_push{0};

	estimator_aid_source2d_s _aid_src{};

	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_airspeed2d_pub {ORB_ID(estimator_aid_src_airspeed2d)};
	uORB::Subscription _sensor_airflow_sub{ORB_ID(sensor_airflow)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::EKF2_ASP2D_CTRL>) _param_ekf2_asp2d_ctrl,
		(ParamFloat<px4::params::EKF2_ASP2D_DELAY>) _param_ekf2_asp2d_delay,
		(ParamFloat<px4::params::EKF2_ASP2D_NOISE>) _param_ekf2_asp2d_noise,
		(ParamFloat<px4::params::EKF2_ASP2D_GATE>) _param_ekf2_asp2d_gate
	)

};

#endif // CONFIG_EKF2_AIRSPEED2D && MODULE_NAME

#endif // !EKF_AIRSPEED2D_HPP
