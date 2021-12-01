/****************************************************************************
 *
 *   Copyright (c) 2019-2021 PX4 Development Team. All rights reserved.
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
 * @file battery.h
 *
 * Library calls for battery functionality.
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Timothy Scott <timothy@auterion.com>
 */

#pragma once

#include <math.h>
#include <float.h>

#include <board_config.h>
#include <px4_platform_common/board_common.h>
#include <px4_platform_common/module_params.h>
#include <matrix/math.hpp>

#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>

/**
 * BatteryBase is a base class for any type of battery.
 *
 * You can use this class on its own. Or, if you need to implement a custom battery type,
 * you can inherit from this class. See, for example, src/modules/battery_status/AnalogBattery.h
 */
class Battery : public ModuleParams
{
public:
	Battery(int index, ModuleParams *parent, const int sample_interval_us, const uint8_t source);
	~Battery() = default;

	/**
	 * Get the battery cell count
	 */
	int cell_count() { return _params.n_cells; }

	/**
	 * Get the empty voltage per cell
	 */
	float empty_cell_voltage() { return _params.v_empty; }

	/**
	 * Get the full voltage per cell
	 */
	float full_cell_voltage() { return _params.v_charged; }

	void setPriority(const uint8_t priority) { _priority = priority; }
	void setConnected(const bool connected) { _connected = connected; }
	void updateVoltage(const float voltage_v);
	void updateCurrent(const float current_a);

	/**
	 * Update current battery status message.
	 *
	 * @param timestamp: Time at which the ADC was read (use hrt_absolute_time())
	 */
	void updateBatteryStatus(const hrt_abstime &timestamp);

	battery_status_s getBatteryStatus();
	void publishBatteryStatus(const battery_status_s &battery_status);

	/**
	 * Convenience function for combined update and publication
	 * @see updateBatteryStatus()
	 * @see publishBatteryStatus()
	 */
	void updateAndPublishBatteryStatus(const hrt_abstime &timestamp);

protected:
	struct {
		param_t v_empty;
		param_t v_charged;
		param_t n_cells;
		param_t capacity;
		param_t v_load_drop;
		param_t r_internal;
		param_t low_thr;
		param_t crit_thr;
		param_t emergen_thr;
		param_t source;
	} _param_handles{};

	struct {
		float v_empty;
		float v_charged;
		int32_t  n_cells;
		float capacity;
		float v_load_drop;
		float r_internal;
		float low_thr;
		float crit_thr;
		float emergen_thr;
		int32_t source;
	} _params{};

	const int _index;

	bool _first_parameter_update{true};
	void updateParams() override;

private:
	void sumDischarged(const hrt_abstime &timestamp, float current_a);
	void estimateStateOfCharge(const float voltage_v, const float current_a);
	uint8_t determineWarning(float state_of_charge);
	void computeScale();
	float computeRemainingTime(float current_a);

	uORB::Subscription _actuator_controls_0_sub{ORB_ID(actuator_controls_0)};
	uORB::PublicationMulti<battery_status_s> _battery_status_pub{ORB_ID(battery_status)};

	bool _connected{false};
	const uint8_t _source{};
	uint8_t _priority{0};
	bool _battery_initialized{false};
	float _voltage_v{0.f};
	AlphaFilter<float> _voltage_filter_v;
	float _current_a{-1};
	AlphaFilter<float> _current_filter_a;
	AlphaFilter<float> _current_average_filter_a;
	AlphaFilter<float> _throttle_filter;
	float _discharged_mah{0.f};
	float _discharged_mah_loop{0.f};
	float _state_of_charge_volt_based{-1.f}; // [0,1]
	float _state_of_charge{-1.f}; // [0,1]
	float _scale{1.f};
	uint8_t _warning{battery_status_s::BATTERY_WARNING_NONE};
	hrt_abstime _last_timestamp{0};
};
