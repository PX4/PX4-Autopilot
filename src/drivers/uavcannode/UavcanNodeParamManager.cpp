/****************************************************************************
 *
 *   Copyright (c) 2020 Volansi, Inc. All rights reserved.
 *
 ****************************************************************************/

#include <lib/parameters/param.h>
#include <px4_platform_common/defines.h>

#include "UavcanNodeParamManager.hpp"

UavcanNodeParamManager::UavcanNodeParamManager()
{
	init_parameters();
}

void UavcanNodeParamManager::getParamNameByIndex(Index index, Name &out_name) const
{
	if (index < _parameter_map.param_count) {
		param_t param_handle = get_param_handle(index);
		out_name = param_name(param_handle);
	}
}

void UavcanNodeParamManager::assignParamValue(const Name &name, const Value &value)
{
	int index = get_param_index(name.c_str());

	if (index < 0) {
		// Invalid parameter name
		return;
	}

	// Assign input value to parameter if types match
	param_t param_handle = get_param_handle(index);
	param_type_t value_type = param_type(param_handle);

	if (value.is(uavcan::protocol::param::Value::Tag::integer_value) && (value_type == PARAM_TYPE_INT32)) {
		auto val = *value.as<uavcan::protocol::param::Value::Tag::integer_value>();

		if ((val <= INT32_MAX) || (val >= INT32_MIN)) {
			int32_t in_param = val;
			param_set(param_handle, &in_param);
		}

	} else if (value.is(uavcan::protocol::param::Value::Tag::real_value) && (value_type == PARAM_TYPE_FLOAT)) {
		// TODO: min/max value checking for float
		float in_param = *value.as<uavcan::protocol::param::Value::Tag::real_value>();
		param_set(param_handle, &in_param);
	}
}

void UavcanNodeParamManager::readParamValue(const Name &name, Value &out_value) const
{
	int index = get_param_index(name.c_str());

	if (index < 0) {
		// Invalid parameter name
		return;
	}

	// Copy current parameter value to out_value
	param_t param_handle = get_param_handle(index);
	param_type_t value_type = param_type(param_handle);

	if (value_type == PARAM_TYPE_INT32) {
		int32_t current_value;
		param_get(param_handle, &current_value);
		out_value.to<uavcan::protocol::param::Value::Tag::integer_value>() = current_value;

	} else if (value_type == PARAM_TYPE_FLOAT) {
		float current_value;
		param_get(param_handle, &current_value);
		out_value.to<uavcan::protocol::param::Value::Tag::real_value>() = current_value;
	}
}

void UavcanNodeParamManager::readParamDefaultMaxMin(const Name &name, Value &out_default,
		NumericValue &out_max, NumericValue &out_min) const
{
	// TODO: get actual default value (will require a new function in param.h)

	int index = get_param_index(name.c_str());

	if (index < 0) {
		// Invalid parameter name
		return;
	}

	param_t param_handle = get_param_handle(index);
	param_type_t value_type = param_type(param_handle);

	if (value_type == PARAM_TYPE_INT32) {
		auto integer_max = INT32_MAX;
		auto integer_min = INT32_MIN;
		out_max.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = integer_max;
		out_min.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = integer_min;
		out_default.to<uavcan::protocol::param::Value::Tag::integer_value>() = 0;

	} else if (value_type == PARAM_TYPE_FLOAT) {
		auto real_max = uavcan::protocol::param::Value::FieldTypes::integer_value::max();
		auto real_min = uavcan::protocol::param::Value::FieldTypes::integer_value::min();
		out_max.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = real_max;
		out_min.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = real_min;
		out_default.to<uavcan::protocol::param::Value::Tag::real_value>() = 0.0;
	}
}

int UavcanNodeParamManager::saveAllParams()
{
	// Nothing to do here assuming autosave is turned on
	return 0;
}

int UavcanNodeParamManager::eraseAllParams()
{
	for (unsigned int i = 0; i < _parameter_map.param_count; ++i) {
		param_reset(get_param_handle(i));
	}

	return 0;
}

int UavcanNodeParamManager::get_param_index(const char *name) const
{
	int lhs = 0;
	int rhs = _parameter_map.param_count;
	int mid;

	// Find parameter using binary search
	while (lhs <= rhs) {
		mid = (rhs + lhs) / 2;
		const char *mid_name = param_name(get_param_handle(mid));
		int res = strcmp(name, mid_name);

		if (res == 0) {
			return mid;

		} else if (lhs == mid) {
			return -1;

		} else if (res < 0) {
			rhs = mid;

		} else {
			lhs = mid;
		}
	}

	return -1;
}

param_t UavcanNodeParamManager::get_param_handle(int index) const
{
	if (index < 0 || index >= (int)_parameter_map.param_count) {
		return -1;
	}

	const param_t *param_array = (const param_t *)&_parameter_map;

	return param_array[index];
}

/**
 * Initialize the parameter map. This is hard-coded for now.
 */
int UavcanNodeParamManager::init_parameters()
{
	_parameter_map.uavcan_baro_t = param_find("UAVCAN_BARO_T");
	_parameter_map.uavcan_mag_t = param_find("UAVCAN_MAG_T");
	_parameter_map.cannode_node_id = param_find("CANNODE_NODE_ID");
	_parameter_map.cannode_bitrate = param_find("CANNODE_BITRATE");
	_parameter_map.cannode_esc_en = param_find("CANNODE_ESC_EN");
	_parameter_map.cannode_esc_mask = param_find("CANNODE_ESC_MASK");
	_parameter_map.cannode_esc0 = param_find("CANNODE_ESC0");
	_parameter_map.cannode_esc1 = param_find("CANNODE_ESC1");
	_parameter_map.cannode_esc2 = param_find("CANNODE_ESC2");
	_parameter_map.cannode_esc3 = param_find("CANNODE_ESC3");
	_parameter_map.cannode_esc4 = param_find("CANNODE_ESC4");
	_parameter_map.cannode_esc5 = param_find("CANNODE_ESC5");
	_parameter_map.cannode_esc6 = param_find("CANNODE_ESC6");
	_parameter_map.cannode_esc7 = param_find("CANNODE_ESC7");
	_parameter_map.cannode_esc8 = param_find("CANNODE_ESC8");
	_parameter_map.cannode_esc9 = param_find("CANNODE_ESC9");
	_parameter_map.cannode_esc10 = param_find("CANNODE_ESC10");
	_parameter_map.cannode_esc11 = param_find("CANNODE_ESC11");
	_parameter_map.cannode_esc12 = param_find("CANNODE_ESC12");
	_parameter_map.cannode_esc13 = param_find("CANNODE_ESC13");
	_parameter_map.cannode_esc14 = param_find("CANNODE_ESC14");

	_parameter_map.param_count = 21;

	return PX4_OK;
}
