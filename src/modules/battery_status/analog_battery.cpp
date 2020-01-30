#include <lib/battery/battery.h>
#include "analog_battery.h"

AnalogBattery::AnalogBattery(int index, ModuleParams *parent) :
	Battery(index, parent)
{
	char param_name[17];

	_analog_param_handles.v_offs_cur = param_find("BAT_V_OFFS_CURR");

	snprintf(param_name, sizeof(param_name), "BAT%d_V_DIV", index);
	_analog_param_handles.v_div = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_A_PER_V", index);
	_analog_param_handles.a_per_v = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_ADC_V_CH", index);
	_analog_param_handles.adc_v_channel = param_find(param_name);
	snprintf(param_name, sizeof(param_name), "BAT%d_ADC_V_DEV", index);
	_analog_param_handles.adc_v_devid = param_find(param_name);
	snprintf(param_name, sizeof(param_name), "BAT%d_ADC_C_CH", index);
	_analog_param_handles.adc_c_channel = param_find(param_name);
	snprintf(param_name, sizeof(param_name), "BAT%d_ADC_C_DEV", index);
	_analog_param_handles.adc_c_devid = param_find(param_name);

	_analog_param_handles.v_div_old = param_find("BAT_V_DIV");
	_analog_param_handles.a_per_v_old = param_find("BAT_A_PER_V");
	_analog_param_handles.adc_channel_old = param_find("BAT_ADC_CHANNEL");
}

void
AnalogBattery::updateBatteryStatusRawADC(hrt_abstime timestamp, int32_t voltage_raw, int32_t current_raw,
		bool selected_source, int priority, float throttle_normalized)
{
	float voltage_v = voltage_raw * _analog_params.v_div;
	float current_a = (current_raw - _analog_params.v_offs_cur) * _analog_params.a_per_v;

	bool connected = voltage_v > BOARD_ADC_OPEN_CIRCUIT_V &&
			 (BOARD_ADC_OPEN_CIRCUIT_V <= BOARD_VALID_UV);


	Battery::updateBatteryStatus(timestamp, voltage_v, current_a, connected,
				     selected_source, priority, throttle_normalized, _params.source == 0);
}


int AnalogBattery::get_voltage_channel()
{
	return _analog_params.adc_v_channel;
}

uint32_t AnalogBattery::get_voltage_deviceid()
{
	union {
		int32_t i;
		uint32_t u;
	} u2i;
	u2i.i = _analog_params.adc_v_devid;
	return u2i.u;
}

int AnalogBattery::get_current_channel()
{
	return _analog_params.adc_c_channel;
}

uint32_t AnalogBattery::get_current_deviceid()
{
	union {
		int32_t i;
		uint32_t u;
	} u2i;
	u2i.i = _analog_params.adc_c_devid;
	return u2i.u;
}

void
AnalogBattery::updateParams()
{
	if (_index == 1) {  // TODO: deprecate out-date parameters
		migrateParam<float>(_analog_param_handles.v_div_old, _analog_param_handles.v_div, &_analog_params.v_div_old,
				    &_analog_params.v_div, _first_parameter_update);
		migrateParam<float>(_analog_param_handles.a_per_v_old, _analog_param_handles.a_per_v, &_analog_params.a_per_v_old,
				    &_analog_params.a_per_v, _first_parameter_update);

	} else {
		param_get(_analog_param_handles.v_div, &_analog_params.v_div);
		param_get(_analog_param_handles.a_per_v, &_analog_params.a_per_v);
	}

	param_get(_analog_param_handles.adc_v_devid, &_analog_params.adc_v_devid);
	param_get(_analog_param_handles.adc_c_devid, &_analog_params.adc_c_devid);
	param_get(_analog_param_handles.adc_v_channel, &_analog_params.adc_v_channel);
	param_get(_analog_param_handles.adc_c_channel, &_analog_params.adc_c_channel);

	param_get(_analog_param_handles.v_offs_cur, &_analog_params.v_offs_cur);

	if (_analog_params.v_div <= 0.0f) {
		/* apply scaling according to defaults if set to default */
		_analog_params.v_div = BOARD_BATTERY1_V_DIV;
		param_set_no_notification(_analog_param_handles.v_div, &_analog_params.v_div);

		if (_index == 1) {
			_analog_params.v_div_old = BOARD_BATTERY1_V_DIV;
			param_set_no_notification(_analog_param_handles.v_div_old, &_analog_params.v_div_old);
		}
	}

	if (_analog_params.a_per_v <= 0.0f) {
		/* apply scaling according to defaults if set to default */

		_analog_params.a_per_v = BOARD_BATTERY1_A_PER_V;
		param_set_no_notification(_analog_param_handles.a_per_v, &_analog_params.a_per_v);

		if (_index == 1) {
			_analog_params.a_per_v_old = BOARD_BATTERY1_A_PER_V;
			param_set_no_notification(_analog_param_handles.a_per_v_old, &_analog_params.a_per_v_old);
		}
	}

	Battery::updateParams();
}
