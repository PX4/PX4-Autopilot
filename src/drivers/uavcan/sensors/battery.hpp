/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @author Jacob Dahl <dahl.jakejacob@gmail.com>
 * @author Alex Klimaj <alex@arkelectron.com>
 */

#pragma once

#include "sensor_bridge.hpp"
#include <uORB/topics/battery_status.h>
#include <uavcan/equipment/power/BatteryInfo.hpp>
#include <ardupilot/equipment/power/BatteryInfoAux.hpp>
#include <battery/battery.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>

using namespace time_literals;

class UavcanBatteryBridge : public UavcanSensorBridgeBase, public ModuleParams
{
public:
	static const char *const NAME;

	UavcanBatteryBridge(uavcan::INode &node);

	const char *get_name() const override { return NAME; }

	int init() override;

private:

	/* Different options to update the battery status */
	enum class BatteryDataType {
		Raw, // data from BatteryInfo message only
		RawAux, // data combination from BatteryInfo and BatteryInfoAux messages
		Filter, // filter data from BatteryInfo message with Battery library
	};

	void battery_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::power::BatteryInfo> &msg);
	void battery_aux_sub_cb(const uavcan::ReceivedDataStructure<ardupilot::equipment::power::BatteryInfoAux> &msg);
	void sumDischarged(hrt_abstime timestamp, float current_a);
	void determineWarning(float remaining);
	void filterData(const uavcan::ReceivedDataStructure<uavcan::equipment::power::BatteryInfo> &msg, uint8_t instance);

	typedef uavcan::MethodBinder < UavcanBatteryBridge *,
		void (UavcanBatteryBridge::*)
		(const uavcan::ReceivedDataStructure<uavcan::equipment::power::BatteryInfo> &) >
		BatteryInfoCbBinder;
	typedef uavcan::MethodBinder < UavcanBatteryBridge *,
		void (UavcanBatteryBridge::*)
		(const uavcan::ReceivedDataStructure<ardupilot::equipment::power::BatteryInfoAux> &) >
		BatteryInfoAuxCbBinder;

	uavcan::Subscriber<uavcan::equipment::power::BatteryInfo, BatteryInfoCbBinder> _sub_battery;
	uavcan::Subscriber<ardupilot::equipment::power::BatteryInfoAux, BatteryInfoAuxCbBinder> _sub_battery_aux;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::BAT_LOW_THR>) _param_bat_low_thr,
		(ParamFloat<px4::params::BAT_CRIT_THR>) _param_bat_crit_thr,
		(ParamFloat<px4::params::BAT_EMERGEN_THR>) _param_bat_emergen_thr
	)

	float _discharged_mah = 0.f;
	float _discharged_mah_loop = 0.f;
	uint8_t _warning;
	hrt_abstime _last_timestamp;
	battery_status_s _battery_status[battery_status_s::MAX_INSTANCES] {};
	BatteryDataType _batt_update_mod[battery_status_s::MAX_INSTANCES] {};

	static constexpr int FILTER_DATA = 2;
	static constexpr int BATTERY_INDEX_1 = 1;
	static constexpr int BATTERY_INDEX_2 = 2;
	static constexpr int BATTERY_INDEX_3 = 3;
	static constexpr int BATTERY_INDEX_4 = 4;
	static constexpr int SAMPLE_INTERVAL_US = 20_ms; // assume higher frequency UAVCAN feedback than 50Hz

	static_assert(battery_status_s::MAX_INSTANCES <= BATTERY_INDEX_4, "Battery array too big");

	Battery battery1 = {BATTERY_INDEX_1, this, SAMPLE_INTERVAL_US, battery_status_s::SOURCE_EXTERNAL};
	Battery battery2 = {BATTERY_INDEX_2, this, SAMPLE_INTERVAL_US, battery_status_s::SOURCE_EXTERNAL};
	Battery battery3 = {BATTERY_INDEX_3, this, SAMPLE_INTERVAL_US, battery_status_s::SOURCE_EXTERNAL};
	Battery battery4 = {BATTERY_INDEX_4, this, SAMPLE_INTERVAL_US, battery_status_s::SOURCE_EXTERNAL};

	Battery *_battery[battery_status_s::MAX_INSTANCES] = { &battery1, &battery2, &battery3, &battery4 };
};
