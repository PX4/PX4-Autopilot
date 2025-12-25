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
# include <px4_platform_common/log.h>
# include <lib/parameters/param.h>
# include <uORB/PublicationMulti.hpp>
# include <uORB/Subscription.hpp>
# include <uORB/topics/estimator_aid_source2d.h>
# include <uORB/topics/aux_global_position.h>
#endif // MODULE_NAME

class Ekf;

class AuxGlobalPosition : public ModuleParams
{
public:
	static constexpr uint8_t MAX_AGP_IDS = 4;

	AuxGlobalPosition() : ModuleParams(nullptr)
	{
		for (int i = 0; i < MAX_AGP_IDS; i++) {
			_sources[i].aid_src_pub.advertise();

		}

		// check existing ID to initialize parameters
		for (int i = 0; i < MAX_AGP_IDS; i++) {
			if (getIdParam(i) != 0) {
				getCtrlParam(i);
				getModeParam(i);
				getDelayParam(i);
				getNoiseParam(i);
				getGateParam(i);

			} else {
				break;
			}
		}
	}

	~AuxGlobalPosition() = default;

	void update(Ekf &ekf, const estimator::imuSample &imu_delayed);

	void updateParameters()
	{
		updateParams();
	}

	float test_ratio_filtered(uint8_t id = 0) const
	{
		if (id < MAX_AGP_IDS) {
			return _sources[id].test_ratio_filtered;
		}

		return INFINITY;
	}

private:
	bool isTimedOut(uint64_t last_sensor_timestamp, uint64_t time_delayed_us, uint64_t timeout_period) const
	{
		return (last_sensor_timestamp == 0) || (last_sensor_timestamp + timeout_period < time_delayed_us);
	}

	bool isResetAllowed(const Ekf &ekf, int slot) const;

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
		kDeadReckoning = 1   	///< Reset on fusion timeout if no source of velocity is availabl
	};

	enum class State {
		kStopped,
		kStarting,
		kActive,
	};

#if defined(MODULE_NAME)
	struct reset_counters_s {
		uint8_t lat_lon{};
	};

	struct SourceData {
		estimator_aid_source2d_s aid_src{};
		RingBuffer<AuxGlobalPositionSample> buffer{20};
		uint64_t time_last_buffer_push{0};
		State state{State::kStopped};
		float test_ratio_filtered{0.f};
		reset_counters_s reset_counters{};
		uORB::PublicationMulti<estimator_aid_source2d_s> aid_src_pub{ORB_ID(estimator_aid_src_aux_global_position)};
	};

	SourceData _sources[MAX_AGP_IDS];

	uORB::Subscription _aux_global_position_subs[MAX_AGP_IDS] {
		{ORB_ID(aux_global_position), 0},
		{ORB_ID(aux_global_position), 1},
		{ORB_ID(aux_global_position), 2},
		{ORB_ID(aux_global_position), 3}
	};

	int32_t getAgpParamInt32(const char *param_suffix, int instance) const
	{
		char param_name[20] {};
		snprintf(param_name, sizeof(param_name), "EKF2_AGP%d_%s", instance, param_suffix);

		int32_t value = 0;

		if (param_get(param_find(param_name), &value) != 0) {
			PX4_ERR("failed to get %s", param_name);
		}

		return value;
	}

	float getAgpParamFloat(const char *param_suffix, int instance) const
	{
		char param_name[20] {};
		snprintf(param_name, sizeof(param_name), "EKF2_AGP%d_%s", instance, param_suffix);

		float value = NAN;

		if (param_get(param_find(param_name), &value) != 0) {
			PX4_ERR("failed to get %s", param_name);
		}

		return value;
	}

	bool setAgpParamInt32(const char *param_suffix, int instance, int32_t value)
	{
		char param_name[20] {};
		snprintf(param_name, sizeof(param_name), "EKF2_AGP%d_%s", instance, param_suffix);

		return param_set_no_notification(param_find(param_name), &value) == PX4_OK;
	}

	int32_t getCtrlParam(int instance) const
	{
		return getAgpParamInt32("CTRL", instance);
	}

	int32_t getModeParam(int instance) const
	{
		return getAgpParamInt32("MODE", instance);
	}

	float getDelayParam(int instance) const
	{
		return getAgpParamFloat("DELAY", instance);
	}

	float getNoiseParam(int instance) const
	{
		return getAgpParamFloat("NOISE", instance);
	}

	float getGateParam(int instance) const
	{
		return getAgpParamFloat("GATE", instance);
	}

	int32_t getIdParam(int instance) const
	{
		return getAgpParamInt32("ID", instance);
	}

	void setIdParam(int instance, int32_t sensor_id)
	{
		setAgpParamInt32("ID", instance, sensor_id);
	}

	int mapSensorIdToSlot(int32_t sensor_id)
	{
		for (int slot = 0; slot < MAX_AGP_IDS; slot++) {
			if (getIdParam(slot) == sensor_id) {
				return slot;
			}
		}

		// Not found, try to assign to first available slot
		for (int slot = 0; slot < MAX_AGP_IDS; slot++) {
			if (getIdParam(slot) == 0) {
				setIdParam(slot, sensor_id);
				return slot;
			}
		}

		// All slots are full
		return MAX_AGP_IDS;
	}

#endif // MODULE_NAME
};

#endif // CONFIG_EKF2_AUX_GLOBAL_POSITION

#endif // !EKF_AUX_GLOBAL_POSITION_HPP
