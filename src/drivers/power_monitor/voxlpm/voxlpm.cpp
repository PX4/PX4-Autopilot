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
 * @file voxlpm.cpp
 * Driver for the VOXL Power Module unit
 */

#include "voxlpm.hpp"

/*
 * The VOXLPM v0 has two LTC2946 ICs on it.
 * Address 0x6A - measures battery voltage and current with a 0.0005 ohm sense resistor
 * Address 0x6B - measures 5VDC ouptut voltage and current
 *
 * The VOXLPM v1 has two INA231 ICs on it.
 * Address 0x44 - measures battery voltage and current with a 0.0005 ohm sense resistor
 * Address 0x45 - measures 5VDC/12VDC ouptut voltage and current
 */
VOXLPM::VOXLPM(I2CSPIBusOption bus_option, const int bus, int bus_frequency, uint8_t address, VOXLPM_CH_TYPE ch_type) :
	I2C(DRV_POWER_DEVTYPE_VOXLPM, MODULE_NAME, bus, address, bus_frequency),
	ModuleParams(nullptr),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus, address),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": sample")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms_errors")),
	_ch_type(ch_type),
	_battery(1, this)
{
}

VOXLPM::~VOXLPM()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
VOXLPM::init()
{
	int ret = PX4_ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		return ret;
	}

	uint8_t addr = get_device_address();

	if (addr == VOXLPM_LTC2946_ADDR_VBATT || addr == VOXLPM_LTC2946_ADDR_P5VD) {
		_pm_type = VOXLPM_TYPE_V0_LTC;
		_rsense = (_ch_type == VOXLPM_CH_TYPE_VBATT) ? VOXLPM_RSENSE_VBATT : VOXLPM_RSENSE_5VOUT;
		ret = init_ltc2946();

	} else if (addr == VOXLPM_INA231_ADDR_VBATT || addr == VOXLPM_INA231_ADDR_P5_12VDC) {
		_pm_type = VOXLPM_TYPE_V1_INA;
		_rsense = (_ch_type == VOXLPM_CH_TYPE_VBATT) ? VOXLPM_INA231_VBAT_SHUNT : VOXLPM_INA231_VREG_SHUNT;
		ret = init_ina231();
	}

	if (ret == PX4_OK) {
		_battery.reset();
		start();
	}

	return ret;
}

int
VOXLPM::init_ltc2946()
{
	write_reg(VOXLPM_LTC2946_CTRLA_REG, DEFAULT_LTC2946_CTRLA_REG_VAL);
	write_reg(VOXLPM_LTC2946_CTRLB_REG, DEFAULT_LTC2946_CTRLB_REG_VAL);
	return PX4_OK;
}

int
VOXLPM::init_ina231()
{
	/* Reset */
	uint16_t cmd = INA231_RST_BIT;
	write_reg_buf(INA231_REG_CONFIG, (uint8_t *)&cmd, sizeof(cmd));

	px4_usleep(100);

	/* Configure */
	cmd = INA231_CONFIG;
	write_reg_buf(INA231_REG_CONFIG, (uint8_t *)&cmd, sizeof(cmd));

	/* Set calibration */
	int16_t cal;

	if (_ch_type == VOXLPM_CH_TYPE_VBATT) {
		cal = (int16_t)VOXLPM_INA231_VBAT_CAL;

	} else {
		cal = (int16_t)VOXLPM_INA231_VREG_CAL;
	}

	write_reg_buf(INA231_REG_CALIBRATION, (uint8_t *)&cal, sizeof(cal));

	return PX4_OK;
}

void
VOXLPM::print_status()
{
	perf_print_counter(_sample_perf);

	switch (_pm_type) {
	case VOXLPM_TYPE_V0_LTC:
		printf("- V0 (LTC2964)\n");
		break;

	case VOXLPM_TYPE_V1_INA:
		printf("- V1 (INA231)\n");
		break;

	default:
		break;
	}

	switch (_ch_type) {
	case VOXLPM_CH_TYPE_VBATT:
		printf("- type: BATT\n");
		break;

	case VOXLPM_CH_TYPE_P5VDC:
		printf("- type: P5VDC\n");
		break;

	case VOXLPM_CH_TYPE_P12VDC:
		printf("- type: P12VDC\n");
		break;

	default:
		printf("- type: UNKOWN\n");
		break;
	}

	printf("  - voltage: %9.2f VDC \n", (double)_voltage);
	printf("  - current: %9.2f ADC \n", (double)_amperage);
	printf("  - rsense: %9.6f ohm \n", (double)_rsense);
	printf("  - meas interval:  %u us \n", _meas_interval);
}

void
VOXLPM::start()
{
	ScheduleOnInterval(_meas_interval, 1000);
}

void
VOXLPM::RunImpl()
{
	measure();
}

int
VOXLPM::measure()
{
	int ret = PX4_ERROR;
	parameter_update_s update;

	if (_parameter_sub.update(&update)) {
		updateParams();
	}

	perf_begin(_sample_perf);

	_voltage = 0.0f;
	_amperage = 0.0f;
	hrt_abstime tnow = hrt_absolute_time();

	switch (_pm_type) {
	case VOXLPM_TYPE_V0_LTC:
		ret = measure_ltc2946();
		break;

	case VOXLPM_TYPE_V1_INA:
		ret = measure_ina231();
		break;

	default:
		break;
	}

	if (ret == PX4_OK) {
		switch (_ch_type) {
		case VOXLPM_CH_TYPE_VBATT: {
				_actuators_sub.copy(&_actuator_controls);

				_battery.updateBatteryStatus(tnow,
							     _voltage,
							     _amperage,
							     true,
							     battery_status_s::BATTERY_SOURCE_POWER_MODULE,
							     0,
							     _actuator_controls.control[actuator_controls_s::INDEX_THROTTLE]);
			}

		// fallthrough
		case VOXLPM_CH_TYPE_P5VDC:
		case VOXLPM_CH_TYPE_P12VDC: {
				memset(&_pm_status, 0x00, sizeof(_pm_status));
				_pm_status.timestamp = tnow;
				_pm_status.voltage_v = (float) _voltage;
				_pm_status.current_a = (float) _amperage;

				//_pm_pub_topic.power_w   = (float) _power * _power_lsb;
				_pm_pub_topic.publish(_pm_status);
			}
			break;

		}

	} else {
		perf_count(_comms_errors);

		switch (_ch_type) {
		case VOXLPM_CH_TYPE_VBATT: {
				_battery.updateBatteryStatus(tnow,
							     0.0,
							     0.0,
							     true,
							     battery_status_s::BATTERY_SOURCE_POWER_MODULE,
							     0,
							     0.0);
			}
			break;

		default:
			break;
		}
	}

	perf_end(_sample_perf);

	return ret;
}

int
VOXLPM::measure_ltc2946()
{
	int ret = PX4_ERROR;
	uint8_t vraw[2];
	uint8_t iraw[2];

	int amp_ret = read_reg_buf(VOXLPM_LTC2946_DELTA_SENSE_MSB_REG, iraw, sizeof(iraw)); // 0x14
	int volt_ret = read_reg_buf(VOXLPM_LTC2946_VIN_MSB_REG, vraw, sizeof(vraw));         // 0x1E

	if ((amp_ret == 0) && (volt_ret == 0)) {
		uint16_t volt16 = (((uint16_t)vraw[0]) << 8) | vraw[1];   // MSB first
		volt16        >>= 4;                                      // data is 12 bit and left-aligned
		_voltage        = (volt16 / VOXLPM_LTC2946_RESOLUTION) * VOXLPM_LTC2946_VFS_SENSE;

		uint16_t curr16 = (((uint16_t)iraw[0]) << 8) | iraw[1];   // MSB first
		curr16        >>= 4;                                      // data is 12 bit and left-aligned
		_amperage       = curr16 / VOXLPM_LTC2946_RESOLUTION * VOXLPM_LTC2946_VFS_DELTA_SENSE / _rsense;
		ret = PX4_OK;
	}

	return ret;
}

int
VOXLPM::measure_ina231()
{
	int ret = PX4_ERROR;

	uint8_t raw_amps[2];
	uint8_t raw_vbus[2];
	int16_t vbus = -1;
	int16_t amps = -1;

	int amp_ret = read_reg_buf(INA231_REG_CURRENT, raw_amps, sizeof(raw_amps));
	int volt_ret = read_reg_buf(INA231_REG_BUSVOLTAGE, raw_vbus, sizeof(raw_vbus));

	if ((amp_ret == 0) && (volt_ret == 0)) {
		memcpy(&vbus, raw_vbus, sizeof(vbus));
		memcpy(&amps, raw_amps, sizeof(amps));

		_voltage = (float) vbus * INA231_VSCALE;

		if (_ch_type == VOXLPM_CH_TYPE_VBATT) {
			_amperage = (float) amps * VOXLPM_INA231_VBAT_I_LSB;

		} else {
			_amperage = (float) amps * VOXLPM_INA231_VREG_I_LSB;
		}

		ret = PX4_OK;
	}

	return ret;
}

uint8_t
VOXLPM::read_reg(uint8_t addr)
{
	uint8_t cmd[2] = { (uint8_t)(addr), 0};
	transfer(&cmd[0], 1, &cmd[1], 1);
	return cmd[1];
}

int
VOXLPM::read_reg_buf(uint8_t addr, uint8_t *buf, uint8_t len)
{
	const uint8_t cmd = (uint8_t)(addr);
	return transfer(&cmd, sizeof(cmd), buf, len);
}

int
VOXLPM::write_reg(uint8_t addr, uint8_t value)
{
	uint8_t cmd[2] = { (uint8_t)(addr), value};
	return transfer(cmd, sizeof(cmd), nullptr, 0);
}

int
VOXLPM::write_reg_buf(uint8_t addr, uint8_t *buf, uint8_t len)
{
	uint8_t cmd[len + 1] = {};
	cmd[0] = addr;
	memcpy(&cmd[1], buf, len);
	return transfer(cmd, sizeof(cmd), nullptr, 0);
}
