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

#pragma once

#include <battery/battery_base.h>
#include <battery/battery.h>

class AnalogBattery : public ModuleParams
{
public:
	AnalogBattery();

	/**
	 * Update current battery status message.
	 *
	 * @param voltage_raw Battery voltage read from ADC, in raw ADC counts
	 * @param current_raw Voltage of current sense resistor, in raw ADC counts
	 * @param timestamp Time at which the ADC was read (use hrt_absolute_time())
	 * @param selected_source This battery is on the brick that the selected source for selected_source
	 * @param priority: The brick number -1. The term priority refers to the Vn connection on the LTC4417
	 * @param throttle_normalized Throttle of the vehicle, between 0 and 1
	 * @param armed Arming state of the vehicle
	 */
	void updateBatteryStatusRawADC(hrt_abstime timestamp, int32_t voltage_raw, int32_t current_raw,
				       bool selected_source, int priority, float throttle_normalized,
				       bool armed);

	/**
	 * Whether the ADC channel for the voltage of this battery is valid.
	 * Corresponds to BOARD_BRICK_VALID_LIST
	 */
	bool is_valid();

	/**
	 * Which ADC channel is used for voltage reading of this battery
	 */
	int getVoltageChannel();

	/**
	 * Which ADC channel is used for current reading of this battery
	 */
	int getCurrentChannel();

protected:
	virtual float _get_cnt_v_volt_raw() = 0;
	virtual float _get_cnt_v_curr_raw() = 0;
	virtual float _get_v_offs_cur() = 0;
	virtual float _get_v_div_raw() = 0;
	virtual float _get_a_per_v_raw() = 0;
	virtual int _get_adc_channel() = 0;

	virtual int _get_brick_index() = 0;

	virtual BatteryBase &_get_battery_base() = 0;

	float _get_cnt_v_volt();
	float _get_cnt_v_curr();
	float _get_v_div();
	float _get_a_per_v();
};

#if BOARD_NUMBER_BRICKS > 0
class AnalogBattery1 : public AnalogBattery
{
public:
	AnalogBattery1();
protected:

	Battery1 _base{};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::BAT_V_DIV>) _param_old_v_div,
		(ParamFloat<px4::params::BAT_A_PER_V>) _param_old_a_per_v,
		(ParamInt<px4::params::BAT_ADC_CHANNEL>) _param_old_adc_channel,

		(ParamFloat<px4::params::BAT1_V_DIV>) _param_v_div,
		(ParamFloat<px4::params::BAT1_A_PER_V>) _param_a_per_v,
		(ParamInt<px4::params::BAT1_ADC_CHANNEL>) _param_adc_channel,

		(ParamFloat<px4::params::BAT_CNT_V_VOLT>) _param_cnt_v_volt,
		(ParamFloat<px4::params::BAT_CNT_V_CURR>) _param_cnt_v_curr,
		(ParamFloat<px4::params::BAT_V_OFFS_CURR>) _param_v_offs_cur
	)

	float _get_v_div_raw() override {return _param_v_div.get(); }
	float _get_a_per_v_raw() override {return _param_a_per_v.get(); }
	int _get_adc_channel() override {return _param_adc_channel.get(); }
	float _get_cnt_v_volt_raw() override {return _param_cnt_v_volt.get(); }
	float _get_cnt_v_curr_raw() override {return _param_cnt_v_curr.get(); }
	float _get_v_offs_cur() override {return _param_v_offs_cur.get(); }

	int _get_brick_index() override {return 0; }

	BatteryBase &_get_battery_base() override {return _base;}
};
#endif // #if BOARD_NUMBER_BRICKS > 0

#if BOARD_NUMBER_BRICKS > 1
class AnalogBattery2 : public AnalogBattery
{
protected:

	Battery2 _base{};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::BAT2_V_DIV>) _param_v_div,
		(ParamFloat<px4::params::BAT2_A_PER_V>) _param_a_per_v,
		(ParamInt<px4::params::BAT2_ADC_CHANNEL>) _param_adc_channel,

		(ParamFloat<px4::params::BAT_CNT_V_VOLT>) _param_cnt_v_volt,
		(ParamFloat<px4::params::BAT_CNT_V_CURR>) _param_cnt_v_curr,
		(ParamFloat<px4::params::BAT_V_OFFS_CURR>) _param_v_offs_cur
	)


	float _get_v_div_raw() override {return _param_v_div.get(); }
	float _get_a_per_v_raw() override {return _param_a_per_v.get(); }
	int _get_adc_channel() override {return _param_adc_channel.get(); }
	float _get_cnt_v_volt_raw() override {return _param_cnt_v_volt.get(); }
	float _get_cnt_v_curr_raw() override {return _param_cnt_v_curr.get(); }
	float _get_v_offs_cur() override {return _param_v_offs_cur.get(); }

	int _get_brick_index() override {return 1; }

	BatteryBase &_get_battery_base() override {return _base;}
};
#endif // #if BOARD_NUMBER_BRICKS > 1