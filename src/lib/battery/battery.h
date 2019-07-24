/****************************************************************************
 *
 *   Copyright (c) 2016, 2017 PX4 Development Team. All rights reserved.
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
 */

#pragma once

#include <uORB/topics/battery_status.h>
#include <drivers/drv_hrt.h>
#include <px4_module_params.h>
#include <drivers/drv_adc.h>
#include <board_config.h>


class BatteryBase
{
public:
	BatteryBase();

	/**
	 * Reset all battery stats and report invalid/nothing.
	 */
	void reset(battery_status_s *battery_status);

	/**
	 * Get the battery cell count
	 */
	int cell_count() { return _get_bat_n_cells(); }

	/**
	 * Get the empty voltage per cell
	 */
	float empty_cell_voltage() { return _get_bat_v_empty(); }

	/**
	 * Get the full voltage per cell
	 */
	float full_cell_voltage() { return _get_bat_v_charged(); }

	/**
	 * Update current battery status message.
	 *
	 * @param voltage_v: current voltage in V
	 * @param current_a: current current in A
	 * @param connected: Battery is connected
	 * @param selected_source: This battery is on the brick that the selected source for selected_source
	 * @param priority: The brick number -1. The term priority refers to the Vn connection on the LTC4417
	 * @param throttle_normalized: throttle from 0 to 1
	 */
	void updateBatteryStatus(int32_t voltage_raw, int32_t current_raw, hrt_abstime timestamp,
				 bool valid_channel, bool selected_source, int priority,
				 float throttle_normalized,
				 bool armed, battery_status_s *status);

protected:
	virtual float _get_bat_v_empty() = 0;
	virtual float _get_bat_v_charged() = 0;
	virtual int _get_bat_n_cells() = 0;
	virtual float _get_bat_capacity() = 0;
	virtual float _get_bat_v_load_drop() = 0;
	virtual float _get_bat_r_internal() = 0;
	virtual float _get_bat_low_thr() = 0;
	virtual float _get_bat_crit_thr() = 0;
	virtual float _get_bat_emergen_thr() = 0;
	virtual float _get_cnt_v_volt() = 0;
	virtual float _get_cnt_v_curr() = 0;
	virtual float _get_v_offs_cur() = 0;
	virtual float _get_v_div() = 0;
	virtual float _get_a_per_v() = 0;
	virtual int _get_source() = 0;
	virtual int _get_adc_channel() = 0;

private:
	void filterVoltage(float voltage_v);
	void filterThrottle(float throttle);
	void filterCurrent(float current_a);
	void sumDischarged(hrt_abstime timestamp, float current_a);
	void estimateRemaining(float voltage_v, float current_a, float throttle, bool armed);
	void determineWarning(bool connected);
	void computeScale();

	bool _battery_initialized = false;
	float _voltage_filtered_v = -1.f;
	float _throttle_filtered = -1.f;
	float _current_filtered_a = -1.f;
	float _discharged_mah = 0.f;
	float _discharged_mah_loop = 0.f;
	float _remaining_voltage = -1.f;		///< normalized battery charge level remaining based on voltage
	float _remaining = -1.f;			///< normalized battery charge level, selected based on config param
	float _scale = 1.f;
	uint8_t _warning;
	hrt_abstime _last_timestamp;
};

class Battery0 : public BatteryBase, public ModuleParams
{
public:
	Battery0() : BatteryBase(), ModuleParams(nullptr) {}

private:

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::BAT_V_EMPTY>) _param_bat_v_empty,
		(ParamFloat<px4::params::BAT_V_CHARGED>) _param_bat_v_charged,
		(ParamInt<px4::params::BAT_N_CELLS>) _param_bat_n_cells,
		(ParamFloat<px4::params::BAT_CAPACITY>) _param_bat_capacity,
		(ParamFloat<px4::params::BAT_V_LOAD_DROP>) _param_bat_v_load_drop,
		(ParamFloat<px4::params::BAT_R_INTERNAL>) _param_bat_r_internal,
		(ParamFloat<px4::params::BAT_LOW_THR>) _param_bat_low_thr,
		(ParamFloat<px4::params::BAT_CRIT_THR>) _param_bat_crit_thr,
		(ParamFloat<px4::params::BAT_EMERGEN_THR>) _param_bat_emergen_thr,
		(ParamFloat<px4::params::BAT_CNT_V_VOLT>) _param_cnt_v_volt,
		(ParamFloat<px4::params::BAT_CNT_V_CURR>) _param_cnt_v_curr,
		(ParamFloat<px4::params::BAT_V_OFFS_CURR>) _param_v_offs_cur,
		(ParamFloat<px4::params::BAT_V_DIV>) _param_v_div,
		(ParamFloat<px4::params::BAT_A_PER_V>) _param_a_per_v,
		(ParamInt<px4::params::BAT_SOURCE>) _param_source,
		(ParamInt<px4::params::BAT_ADC_CHANNEL>) _param_adc_channel
	)

	float _get_bat_v_empty() override {return _param_bat_v_empty.get(); }
	float _get_bat_v_charged() override {return _param_bat_v_charged.get(); }
	int _get_bat_n_cells() override {return _param_bat_n_cells.get(); }
	float _get_bat_capacity() override {return _param_bat_capacity.get(); }
	float _get_bat_v_load_drop() override {return _param_bat_v_load_drop.get(); }
	float _get_bat_r_internal() override {return _param_bat_r_internal.get(); }
	float _get_bat_low_thr() override {return _param_bat_low_thr.get(); }
	float _get_bat_crit_thr() override {return _param_bat_crit_thr.get(); }
	float _get_bat_emergen_thr() override {return _param_bat_emergen_thr.get(); }
	float _get_cnt_v_volt() override {return _param_cnt_v_volt.get(); }
	float _get_cnt_v_curr() override {return _param_cnt_v_curr.get(); }
	float _get_v_offs_cur() override {return _param_v_offs_cur.get(); }
	float _get_v_div() override {return _param_v_div.get(); }
	float _get_a_per_v() override {return _param_a_per_v.get(); }
	int _get_source() override {return _param_source.get(); }
	int _get_adc_channel() override {return _param_adc_channel.get(); }
};
