/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file ParamManager.cpp
 *
 * Implements basic functionality of UAVCAN parameter management class
 *
 * @author Jacob Crabill <jacob@flyvoly.com>
 */

#include "ParamManager.hpp"
#include <px4_platform_common/defines.h>

bool UavcanParamManager::GetParamByName(const char *param_name, uavcan_register_Value_1_0 &value)
{
	for (auto &param : _uavcan_params) {
		if (strcmp(param_name, param.uavcan_name) == 0) {
			param_t param_handle = param_find(param.px4_name);

			if (param_handle == PARAM_INVALID) {
				return false;
			}

			return param.px4_param_to_register_value(param_handle, value);
		}
	}

	for (auto &param : _type_registers) {
		if (strcmp(param_name, param.name) == 0) {
			uavcan_register_Value_1_0_select_string_(&value);
			value._string.value.count = strlen(param.value);
			memcpy(&value._string, param.value, value._string.value.count);
			return true;
		}
	}

	return false;
}

bool UavcanParamManager::GetParamByName(const uavcan_register_Name_1_0 &name, uavcan_register_Value_1_0 &value)
{
	for (auto &param : _uavcan_params) {
		if (strncmp((char *)name.name.elements, param.uavcan_name, name.name.count) == 0) {
			param_t param_handle = param_find(param.px4_name);

			if (param_handle == PARAM_INVALID) {
				return false;
			}

			return param.px4_param_to_register_value(param_handle, value);
		}
	}

	for (auto &param : _type_registers) {
		if (strncmp((char *)name.name.elements, param.name, name.name.count) == 0) {
			uavcan_register_Value_1_0_select_string_(&value);
			value._string.value.count = strlen(param.value);
			memcpy(&value._string, param.value, value._string.value.count);
			return true;
		}
	}

	return false;
}

bool UavcanParamManager::GetParamName(uint32_t id, uavcan_register_Name_1_0 &name)
{
	size_t number_of_integer_registers = sizeof(_uavcan_params) / sizeof(UavcanParamBinder);
	size_t number_of_type_registers = sizeof(_type_registers) / sizeof(CyphalTypeRegister);

	if (id < sizeof(_uavcan_params) / sizeof(UavcanParamBinder)) {
		strncpy((char *)name.name.elements, _uavcan_params[id].uavcan_name, uavcan_register_Name_1_0_name_ARRAY_CAPACITY_);
		name.name.count = strlen(_uavcan_params[id].uavcan_name);

	} else if (id < number_of_integer_registers + number_of_type_registers) {
		id -= number_of_integer_registers;
		strncpy((char *)name.name.elements, _type_registers[id].name, strlen(_type_registers[id].name));
		name.name.count = strlen(_type_registers[id].name);

	} else {
		return false;
	}

	return true;
}

bool UavcanParamManager::SetParamByName(const uavcan_register_Name_1_0 &name, const uavcan_register_Value_1_0 &value)
{
	for (auto &param : _uavcan_params) {
		if (strncmp((char *)name.name.elements, param.uavcan_name, name.name.count) == 0) {
			param_t param_handle = param_find(param.px4_name);

			if (param_handle == PARAM_INVALID) {
				return false;
			}

			return param.register_value_to_px4_param(value, param_handle);
		}
	}

	return false;
}
