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

#include "battery_base.h"


/**
 * @file battery.h
 * Basic implementation of BatteryBase. This uses all BAT1_* parameters where applicable.
 *
 * The multiple batteries all share the same logic for calibration. The only difference is which parameters are used
 * (Battery 1 uses `BAT_*`, while Battery 2 uses `BAT2_*`). To avoid code duplication, inheritance is being used.
 * The problem is that the `ModuleParams` class depends on a macro which defines member variables. You can't override
 * member variables in C++, so we have to declare virtual getter functions in BatteryBase, and implement them here.
 *
 * The alternative would be to avoid ModuleParams entirely, and build parameter names dynamically, like so:
 * ```
 * char param_name[17]; //16 max length of parameter name, + null terminator
 * int battery_index = 1; // Or 2 or 3 or whatever
 * snprintf(param_name, 17, "BAT%d_N_CELLS", battery_index);
 * // A real implementation would have to handle the case where battery_index == 1 and there is no number in the param name.
 * param_find(param_name); // etc
 * ```
 *
 * This was decided against because the newer ModuleParams API provides more type safety and avoids code duplication.
 *
 * To add a new battery, just create a new implementation of BatteryBase and implement all of the _get_* methods,
 * then add all of the new parameters necessary for calibration.
 */
class Battery : public BatteryBase, public ModuleParams
{
public:
	Battery() : ModuleParams(nullptr) {}
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
		(ParamInt<px4::params::BAT_SOURCE>) _param_source
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
	int _get_source() override {return _param_source.get(); }
};
