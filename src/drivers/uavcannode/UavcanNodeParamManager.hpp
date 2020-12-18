/****************************************************************************
 *
 *   Copyright (c) 2020 Volansi, Inc. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <parameters/param.h>

#include <uavcan/uavcan.hpp>
#include <uavcan/protocol/param_server.hpp>

class UavcanNodeParamManager : public uavcan::IParamManager
{
public:
	UavcanNodeParamManager();

	void getParamNameByIndex(Index index, Name &out_name) const override;
	void assignParamValue(const Name &name, const Value &value) override;
	void readParamValue(const Name &name, Value &out_value) const override;
	void readParamDefaultMaxMin(const Name &name, Value &out_default,
				    NumericValue &out_max, NumericValue &out_min) const override;
	int saveAllParams() override;
	int eraseAllParams() override;
private:

	/**
	 * Get parameter index in parameter map.
	 */
	int get_param_index(const char *name) const;

	/**
	 * Get the param_t handle for the mapping at index.
	 */
	param_t get_param_handle(int index) const;

	/**
	 * Initialize the parameter map.
	 */
	int init_parameters();

	/**
	 * Parameter map from UAVCAN indices to param handles.
	 * Must be in alphabetical order by parameter name.
	 */
	struct ParameterMap {
		param_t cannode_bitrate;
		param_t cannode_esc0;
		param_t cannode_esc1;
		param_t cannode_esc10;
		param_t cannode_esc11;
		param_t cannode_esc12;
		param_t cannode_esc13;
		param_t cannode_esc14;
		param_t cannode_esc2;
		param_t cannode_esc3;
		param_t cannode_esc4;
		param_t cannode_esc5;
		param_t cannode_esc6;
		param_t cannode_esc7;
		param_t cannode_esc8;
		param_t cannode_esc9;
		param_t cannode_esc_en;
		param_t cannode_esc_mask;
		param_t cannode_node_id;
		param_t uavcan_baro_t;
		param_t uavcan_mag_t;
		unsigned int param_count;
	} _parameter_map{0};
};
