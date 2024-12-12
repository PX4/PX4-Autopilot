/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#ifndef EKF_AUX_GLOBAL_POSITION_HPP
#define EKF_AUX_GLOBAL_POSITION_HPP

// interface?
//  - ModuleParams
//  - Base class EKF
//  -  bool update(imu)
//   how to get delay?
//  WelfordMean for init?
//  WelfordMean for rate

#include "../../common.h"
#include "../../RingBuffer.h"

#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION) && defined(MODULE_NAME)

#if defined(MODULE_NAME)
# include <px4_platform_common/module_params.h>
# include <uORB/PublicationMulti.hpp>
# include <uORB/Subscription.hpp>
# include <uORB/topics/estimator_aid_source2d.h>
# include <uORB/topics/vehicle_global_position.h>
#endif // MODULE_NAME

class Ekf;

class AuxGlobalPosition : public ModuleParams
{
public:
	AuxGlobalPosition() : ModuleParams(nullptr)
	{
		_estimator_aid_src_aux_global_position_pub.advertise();
	}

	~AuxGlobalPosition() = default;

	void update(Ekf &ekf, const estimator::imuSample &imu_delayed);

	void updateParameters()
	{
		updateParams();
	}

	float test_ratio_filtered() const { return _test_ratio_filtered; }

private:
	bool isTimedOut(uint64_t last_sensor_timestamp, uint64_t time_delayed_us, uint64_t timeout_period) const
	{
		return (last_sensor_timestamp == 0) || (last_sensor_timestamp + timeout_period < time_delayed_us);
	}

	struct AuxGlobalPositionSample {
		uint64_t time_us{};     ///< timestamp of the measurement (uSec)
		double latitude{};
		double longitude{};
		float altitude_amsl{};
		float eph{};
		float epv{};
		uint8_t lat_lon_reset_counter{};
	};

	estimator_aid_source2d_s _aid_src_aux_global_position{};
	RingBuffer<AuxGlobalPositionSample> _aux_global_position_buffer{20}; // TODO: size with _obs_buffer_length and actual publication rate
	uint64_t _time_last_buffer_push{0};

	enum class Ctrl : uint8_t {
		HPOS  = (1 << 0),
		VPOS  = (1 << 1)
	};

	enum class State {
		stopped,
		starting,
		active,
	};

	State _state{State::stopped};

	float _test_ratio_filtered{INFINITY};

#if defined(MODULE_NAME)
	struct reset_counters_s {
		uint8_t lat_lon{};
	};
	reset_counters_s _reset_counters{};

	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_aux_global_position_pub{ORB_ID(estimator_aid_src_aux_global_position)};
	uORB::Subscription _aux_global_position_sub{ORB_ID(aux_global_position)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::EKF2_AGP_CTRL>) _param_ekf2_agp_ctrl,
		(ParamFloat<px4::params::EKF2_AGP_DELAY>) _param_ekf2_agp_delay,
		(ParamFloat<px4::params::EKF2_AGP_NOISE>) _param_ekf2_agp_noise,
		(ParamFloat<px4::params::EKF2_AGP_GATE>) _param_ekf2_agp_gate
	)

#endif // MODULE_NAME
};

#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION

#endif // !EKF_AUX_GLOBAL_POSITION_HPP
