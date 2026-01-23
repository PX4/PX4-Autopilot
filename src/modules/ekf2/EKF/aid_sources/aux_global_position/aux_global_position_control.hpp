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

#ifndef EKF_AUX_GLOBAL_POSITION_CONTROL_HPP
#define EKF_AUX_GLOBAL_POSITION_CONTROL_HPP

#include "../../common.h"

#if defined(CONFIG_EKF2_AUX_GLOBAL_POSITION) && defined(MODULE_NAME)

#include <lib/parameters/param.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/estimator_aid_source2d.h>
#include <uORB/topics/aux_global_position.h>

class Ekf;
class AuxGlobalPosition;

class AgpSource
{
public:
	AgpSource(int instance_id, AuxGlobalPosition *manager);
	~AgpSource() = default;

	void checkAndBufferData(const estimator::imuSample &imu_delayed);
	void update(Ekf &ekf, const estimator::imuSample &imu_delayed);
	void advertise() { _aid_src_pub.advertise(); }
	void initParams();
	void updateParams();

	float getTestRatioFiltered() const { return _test_ratio_filtered; }
	bool isFusing() const { return _state == State::kActive; }

private:
	struct AuxGlobalPositionSample {
		uint64_t time_us{};     ///< timestamp of the measurement (uSec)
		uint8_t id{};           ///< source identifier
		double latitude{};
		double longitude{};
		float altitude_amsl{};
		float eph{};
		float epv{};
		uint8_t lat_lon_reset_counter{};
	};

	enum class Ctrl : uint8_t {
		kHPos  = (1 << 0),
		kVPos  = (1 << 1)
	};

	enum class Mode : uint8_t {
		kAuto           = 0,   	///< Reset on fusion timeout if no other source of position is available
		kDeadReckoning = 1   	///< Reset on fusion timeout if no source of velocity is available
	};

	enum class State {
		kStopped,
		kStarting,
		kActive,
	};

	struct reset_counters_s {
		uint8_t lat_lon{};
	};

	uORB::Subscription _agp_sub;
	uORB::PublicationMulti<estimator_aid_source2d_s> _aid_src_pub{ORB_ID(estimator_aid_src_aux_global_position)};

	TimestampedRingBuffer<AuxGlobalPositionSample> _buffer{20};
	State _state{State::kStopped};
	estimator_aid_source2d_s _aid_src{};
	float _test_ratio_filtered{0.f};
	uint64_t _time_last_buffer_push{0};
	reset_counters_s _reset_counters{};

	AuxGlobalPosition *_manager;
	const int _instance_id;

	struct ParamHandles {
		param_t id{PARAM_INVALID};
		param_t ctrl{PARAM_INVALID};
		param_t mode{PARAM_INVALID};
		param_t delay{PARAM_INVALID};
		param_t noise{PARAM_INVALID};
		param_t gate{PARAM_INVALID};
	} _param_handles;

	struct Params {
		int32_t id{0};
		int32_t ctrl{0};
		int32_t mode{0};
		float delay{0.f};
		float noise{10.f};
		float gate{3.f};
	} _params;

	bool isResetAllowed(const Ekf &ekf) const;
	bool isTimedOut(uint64_t last_sensor_timestamp, uint64_t time_delayed_us, uint64_t timeout_period) const;
};

#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION && MODULE_NAME

#endif // !EKF_AUX_GLOBAL_POSITION_CONTROL_HPP
