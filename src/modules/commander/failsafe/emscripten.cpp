/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#define PARAM_IMPLEMENTATION
#include <parameters/param.h>

#include "failsafe.h"
#include "../ModeUtil/mode_requirements.hpp"
#include <uORB/topics/vehicle_status.h>

#include <emscripten/emscripten.h>
#include <emscripten/bind.h>
#include <emscripten/html5.h>
#include <cstdio>
#include <string>
#include <map>
#include <vector>

using namespace emscripten;

#include <generated_uorb_struct_field_mapping.h>

// parameter storage
struct Param {
	union param_value_u val;
};

class FailsafeTest : public ModuleParams
{
public:
	FailsafeTest() : ModuleParams(nullptr) {}

	void updateParams() override
	{
		ModuleParams::updateParams();
	}

	FailsafeBase &current() { return _failsafe; }
	std::map<param_t, Param> &params() { return _used_params; }
private:
	std::map<param_t, Param> _used_params;
	Failsafe _failsafe{this};
};

static FailsafeTest failsafe_instance;


int param_get(param_t param, void *val)
{
	std::map<param_t, Param> &used_params = failsafe_instance.params();
	auto param_iter = used_params.find(param);

	if (param_iter != used_params.end()) {
		memcpy(val, &param_iter->second.val, sizeof(param_iter->second.val));
		return 0;
	}

	printf("Error: param %i not found\n", param);
	return -1;
}

void param_set_used(param_t param)
{
	std::map<param_t, Param> &used_params = failsafe_instance.params();

	if (used_params.find(param) != used_params.end()) {
		return;
	}

	Param p;
	memcpy(&p.val, &px4::parameters[param].val, sizeof(p.val));
	used_params[param] = p;
}

std::vector<std::string> get_used_params()
{
	std::vector<std::string> ret;
	std::map<param_t, Param> &used_params = failsafe_instance.params();

	for (const auto &param_iter : used_params) {
		ret.push_back(px4::parameters[param_iter.first].name);
	}

	return ret;
}
param_value_u get_param_value(const std::string &name)
{
	std::map<param_t, Param> &used_params = failsafe_instance.params();

	for (const auto &param_iter : used_params) {
		if (name == px4::parameters[param_iter.first].name) {
			return param_iter.second.val;
		}
	}

	printf("Error: param %s not used\n", name.c_str());
	return {};
}

int get_param_value_int32(const std::string &name)
{
	return get_param_value(name).i;
}

float get_param_value_float(const std::string &name)
{
	return get_param_value(name).f;
}

void set_param_value(const std::string &name, const param_value_u value)
{
	std::map<param_t, Param> &used_params = failsafe_instance.params();

	for (auto &param_iter : used_params) {
		if (name == px4::parameters[param_iter.first].name) {
			param_iter.second.val = value;
			break;
		}
	}

	failsafe_instance.updateParams();
}

void set_param_value_int32(const std::string &name, int value)
{
	param_value_u param_value;
	param_value.i = value;
	set_param_value(name, param_value);
}

void set_param_value_float(const std::string &name, float value)
{
	param_value_u param_value;
	param_value.f = value;
	set_param_value(name, param_value);
}

int failsafe_update(bool armed, bool vtol_in_transition_mode, bool mission_finished,
		    bool user_override, uint8_t user_intended_mode, uint8_t vehicle_type,
		    failsafe_flags_s status_flags, bool defer_failsafes)
{
	uint64_t time_ms = emscripten_date_now();
	FailsafeBase::State state{};
	state.armed = armed;
	state.vtol_in_transition_mode = vtol_in_transition_mode;
	state.mission_finished = mission_finished;
	state.user_intended_mode = user_intended_mode;
	state.vehicle_type = vehicle_type;
	mode_util::getModeRequirements(vehicle_type, status_flags);
	failsafe_instance.current().deferFailsafes(defer_failsafes, 0);
	return failsafe_instance.current().update(time_ms * 1000, state, false, user_override, status_flags);
}

int selected_action()
{
	FailsafeBase::Action action = failsafe_instance.current().selectedAction();

	if (action == FailsafeBase::Action::Disarm) {
		printf("Disarming\n");
	}

	return (int)action;
}

bool user_takeover_active()
{
	return failsafe_instance.current().userTakeoverActive();
}

std::string action_str(int action)
{
	return FailsafeBase::actionStr((FailsafeBase::Action)action);
}

EMSCRIPTEN_BINDINGS(failsafe)
{
	class_<failsafe_flags_s>("state")
	.constructor<>()
	UORB_STRUCT_FIELD_MAPPING
	;

	function("failsafe_update", &failsafe_update);
	function("action_str", &action_str);
	function("get_used_params", &get_used_params);
	function("set_param_value_int32", &set_param_value_int32);
	function("set_param_value_float", &set_param_value_float);
	function("get_param_value_int32", &get_param_value_int32);
	function("get_param_value_float", &get_param_value_float);
	function("user_takeover_active", &user_takeover_active);
	function("selected_action", &selected_action);
	register_vector<std::string>("vector<string>");
}
