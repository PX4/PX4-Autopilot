#ifndef PX4_BATTERY_H
#define PX4_BATTERY_H

#include "battery_base.h"

/**
 * @file battery.h
 * Implementations of BatteryBase
 *
 * Because the ModuleParams class depends on macros which declare member variables,
 */

class Battery0 : public BatteryBase
{
public:
	Battery0() : BatteryBase()
	{
		// Can't do this in the constructor because virtual functions
		if (_get_adc_channel() >= 0) {
			vChannel = _get_adc_channel();

		} else {
			vChannel = DEFAULT_V_CHANNEL[0];
		}

		// TODO: Add parameter, like with V
		iChannel = DEFAULT_I_CHANNEL[0];
	}

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

	int _get_brick_index() override {return 0; }
};

class Battery1 : public BatteryBase
{
public:
	Battery1() : BatteryBase()
	{
		// Can't do this in the constructor because virtual functions
		if (_get_adc_channel() >= 0) {
			vChannel = _get_adc_channel();

		} else {
			vChannel = DEFAULT_V_CHANNEL[1];
		}

		// TODO: Add parameter, like with V
		iChannel = DEFAULT_I_CHANNEL[1];
	}

private:

	// TODO: Change to new parameters
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

	int _get_brick_index() override {return 1; }
};



#endif //PX4_BATTERY_H
