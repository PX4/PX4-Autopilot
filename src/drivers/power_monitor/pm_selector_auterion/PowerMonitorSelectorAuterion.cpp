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
 * Automatic handling power monitors
 */

#include "PowerMonitorSelectorAuterion.h"
#include "../ina226/ina226.h"

#include <builtin/builtin.h>
#include <sys/wait.h>

PowerMonitorSelectorAuterion::PowerMonitorSelectorAuterion() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

PowerMonitorSelectorAuterion::~PowerMonitorSelectorAuterion() = default;

bool PowerMonitorSelectorAuterion::init()
{
	ScheduleNow();
	return true;
}

void PowerMonitorSelectorAuterion::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	actuator_armed_s actuator_armed{};
	_actuator_armed_sub.copy(&actuator_armed);

	if (actuator_armed.armed) {
		exit_and_cleanup();
		return;
	}

	/* Always try to start with an EEPROM that contains PM information */
	try_eeprom_start();

	if (_eeprom_only_run >= EEPROM_ONLY_RUNS) {
		/* Give preference to the EEPROM-based initialization; use probing as a fallback */
		try_probe_start();

	} else {
		_eeprom_only_run++;
	}

	ScheduleDelayed(RUN_INTERVAL);
}

void PowerMonitorSelectorAuterion::try_eeprom_start()
{
	static_assert(sizeof(_buses) / sizeof(Buses) == BUSES_NUMBERS, "Unexpected number of buses");
	static_assert(sizeof(_eeprom_blocks_pm) / sizeof(EepromBlockPm) == EEPROM_MAX_BLOCKS,
		      "Unexpected number of EEPROM PM blocks");

	for (uint32_t i = 0U; i < BUSES_NUMBERS; i++) {
		if (!_buses[i].started) {
			if (eeprom_read(i) == PX4_OK) {
				for (int ii = 0U; ii < _eeprom_valid_blocks_pm && ii < EEPROM_MAX_BLOCKS; ii++) {
					EepromBlockPm eeprom_block_pm = _eeprom_blocks_pm[ii];

					uint16_t dev_type = eeprom_block_pm.dev_type;
					set_max_current(dev_type, eeprom_block_pm.max_current);
					set_shunt_value(dev_type, eeprom_block_pm.shunt_value);
					int ret_val = start_pm(_buses[i].bus_number, dev_type, eeprom_block_pm.i2c_addr, _buses[i].id);

					if (ret_val == PX4_OK) {
						_buses[i].started = true;
					}
				}
			}
		}
	}
}

void PowerMonitorSelectorAuterion::try_probe_start()
{
	static_assert(sizeof(_sensors) / sizeof(Sensor) == SENSORS_NUMBER, "Unexpected number of sensors");

	for (uint32_t i = 0U; i < SENSORS_NUMBER; ++i) {
		if (i >= BUSES_NUMBERS || !_buses[i].started) {
			if (!_sensors[i].started) {
				uint16_t dev_type = _sensors[i].dev_type;

				if (!is_user_configured(dev_type)) {
					if (ina226_probe(i) == PX4_OK) {
						set_shunt_value(dev_type, _sensors[i].shunt_value);
						int ret_val = start_pm(_sensors[i].bus_number, dev_type, _sensors[i].i2c_addr, _sensors[i].id);

						if (ret_val == PX4_OK) {
							_sensors[i].started = true;
						}
					}
				}
			}
		}
	}
}

int PowerMonitorSelectorAuterion::ina226_probe(const uint32_t instance) const
{
	I2CWrapper i2c{_sensors[instance].bus_number};
	int ret = PX4_ERROR;

	if (i2c.is_valid()) {
		struct i2c_msg_s msgv[2];

		uint8_t txdata[1] = {0};
		uint8_t rxdata[2] = {0};

		msgv[0].frequency = I2C_SPEED_STANDARD;
		msgv[0].addr = _sensors[instance].i2c_addr;
		msgv[0].flags = 0;
		msgv[0].buffer = txdata;
		msgv[0].length = sizeof(txdata);

		msgv[1].frequency = I2C_SPEED_STANDARD;
		msgv[1].addr = _sensors[instance].i2c_addr;
		msgv[1].flags = I2C_M_READ;
		msgv[1].buffer = rxdata;
		msgv[1].length = sizeof(rxdata);

		txdata[0] = {INA226_MFG_ID};
		ret = I2C_TRANSFER(i2c.get(), msgv, 2);
		uint16_t value = static_cast<uint16_t>(rxdata[1] | rxdata[0] << 8);

		if (ret != PX4_OK || value != INA226_MFG_ID_TI) {
			ret = PX4_ERROR;

		} else {
			txdata[0] = {INA226_MFG_DIEID};
			ret = I2C_TRANSFER(i2c.get(), msgv, 2);
			value = static_cast<uint16_t>(rxdata[1] | rxdata[0] << 8);

			if (ret != PX4_OK || value != INA226_MFG_DIE) {
				ret = PX4_ERROR;
			}
		}
	}

	return ret;
}

int PowerMonitorSelectorAuterion::eeprom_read(const uint32_t instance)
{
	I2CWrapper i2c{_buses[instance].bus_number};
	_eeprom_valid_blocks_pm = 0;
	EepromHeader eeprom_header;

	if (i2c.is_valid()) {
		struct i2c_msg_s msgv[2];
		uint8_t txdata[2] = {0};

		/* Sets EEPROM address pointer to 0 */
		msgv[0].frequency = I2C_SPEED_STANDARD;
		msgv[0].addr = _buses[instance].eeprom_i2c_addr;
		msgv[0].flags = 0;
		msgv[0].buffer = txdata;
		msgv[0].length = sizeof(txdata);

		/* Read the header from the EEPROM. */
		msgv[1].frequency = I2C_SPEED_STANDARD;
		msgv[1].addr = _buses[instance].eeprom_i2c_addr;
		msgv[1].flags = I2C_M_READ;
		/* Assumes that the EEPROM was also written on a little-endian platform */
		msgv[1].buffer = reinterpret_cast<uint8_t *>(&eeprom_header);
		msgv[1].length = sizeof(EepromHeader);

		int header_read_ret = I2C_TRANSFER(i2c.get(), msgv, 2);

		if (header_read_ret != PX4_OK || !is_eeprom_header_valid(&eeprom_header)) {
			return PX4_ERROR;
		}

		int transferred_blocks = 0;

		/* CRC computation starts from the 'flags' field and contains everything that follows (in the header and the blocks) */
		uint8_t *crc_start_ptr = reinterpret_cast<uint8_t *>(&eeprom_header) + offsetof(EepromHeader, flags);
		size_t crc_header_length = sizeof(EepromHeader) - offsetof(EepromHeader, flags);
		uint16_t crc = crc16_update(0, crc_start_ptr, crc_header_length);

		while (transferred_blocks < eeprom_header.num_blocks && transferred_blocks < EEPROM_MAX_BLOCKS) {
			int block_read_ret = eeprom_read_block(i2c.get(), instance, transferred_blocks, crc);
			transferred_blocks++;

			if (block_read_ret != PX4_OK) {
				return PX4_ERROR;
			}
		}

		if (crc != eeprom_header.crc) {
			return PX4_ERROR;
		}

		_eeprom_valid_blocks_pm = transferred_blocks;
		return PX4_OK;

	} else {
		return PX4_ERROR;
	}
}

bool PowerMonitorSelectorAuterion::is_eeprom_header_valid(EepromHeader *eeprom_header) const
{
	if (eeprom_header->magic != EEPROM_MAGIC
	    || eeprom_header->version != EEPROM_VERSION
	    || eeprom_header->num_blocks > EEPROM_MAX_BLOCKS) {
		return false;
	}

	return true;
}

int PowerMonitorSelectorAuterion::eeprom_read_block(struct i2c_master_s *i2c, const uint32_t instance,
		const uint16_t transferred_blocks, uint16_t &crc)
{
	int ret = PX4_ERROR;
	EepromBlockHeader eeprom_block_header;

	/* Read the block header from the EEPROM */
	struct i2c_msg_s msg;
	msg.frequency = I2C_SPEED_STANDARD;
	msg.addr = _buses[instance].eeprom_i2c_addr;
	msg.flags = I2C_M_READ;
	msg.buffer = reinterpret_cast<uint8_t *>(&eeprom_block_header);
	msg.length = sizeof(EepromBlockHeader);

	ret = I2C_TRANSFER(i2c, &msg, 1);

	if (ret == PX4_OK && is_eeprom_block_header_valid(&eeprom_block_header)) {
		EepromBlockPm &eeprom_block_pm = _eeprom_blocks_pm[transferred_blocks];
		eeprom_block_pm.block_header = eeprom_block_header;

		/* Already read the header, so just need to read the block itself now */
		uint8_t *data_ptr = reinterpret_cast<uint8_t *>(&eeprom_block_pm) + offsetof(EepromBlockPm, dev_type);
		size_t data_size = sizeof(EepromBlockPm) - offsetof(EepromBlockPm, dev_type);

		/* Read the actual block data from the EEPROM */
		msg.buffer = data_ptr;
		msg.length = data_size;

		ret = I2C_TRANSFER(i2c, &msg, 1);

		if (ret == PX4_OK) {
			crc = crc16_update(crc, reinterpret_cast<uint8_t *>(&eeprom_block_pm), sizeof(EepromBlockPm));
		}

	} else {
		ret = PX4_ERROR;
	}

	return ret;
}

bool PowerMonitorSelectorAuterion::is_eeprom_block_header_valid(EepromBlockHeader *eeprom_block_header) const
{
	if (eeprom_block_header->block_type != BlockType::TYPE_PM
	    || eeprom_block_header->block_type_version != EEPROM_BLOCK_TYPE_VERSION
	    || eeprom_block_header->block_length != sizeof(EepromBlockPm)) {
		return false;
	}

	return true;
}

int PowerMonitorSelectorAuterion::start_pm(const uint8_t bus_number, const uint16_t dev_type,
		const uint16_t i2c_addr, const uint16_t id) const
{
	char bus_number_str[BUS_MAX_LEN];
	snprintf(bus_number_str, sizeof(bus_number_str), "%u", bus_number);

	char i2c_addr_str[I2C_ADDR_MAX_LEN];
	snprintf(i2c_addr_str, sizeof(i2c_addr_str), "%u", i2c_addr);

	char id_str[ID_MAX_LEN];
	snprintf(id_str, sizeof(id_str), "%u", id);

	const char *start_command = get_start_command(dev_type);

	if (start_command == nullptr) {
		return PX4_ERROR;
	}

	const char *start_argv[] {
		start_command,
		"-X", "-b", bus_number_str, "-a", i2c_addr_str,
		"-t", id_str, "-q", "start", NULL
	};

	int status = PX4_ERROR;
	int pid = exec_builtin(start_command, (char **)start_argv, NULL, 0);

	if (pid != -1) {
		waitpid(pid, &status, WUNTRACED);
	}

	return status;
}

const char *PowerMonitorSelectorAuterion::get_start_command(const uint16_t dev_type) const
{
	switch (dev_type) {
	case DRV_POWER_DEVTYPE_INA220:
		return "ina220";

	case DRV_POWER_DEVTYPE_INA226:
		return "ina226";

	case DRV_POWER_DEVTYPE_INA228:
		return "ina228";

	case DRV_POWER_DEVTYPE_INA238:
		return "ina238";

	default:
		return nullptr;
	}
}

bool PowerMonitorSelectorAuterion::is_user_configured(const uint16_t dev_type) const
{
	const char *ina_type = get_ina_type(dev_type);

	if (ina_type == nullptr) {
		return false;
	}

	char param_name[PARAM_MAX_LEN];
	snprintf(param_name, sizeof(param_name), "SENS_EN_INA%s", ina_type);

	int32_t sens_en = 0;
	param_get(param_find(param_name), &sens_en);
	return sens_en != 0;
}

void PowerMonitorSelectorAuterion::set_max_current(const uint16_t dev_type, const float max_current) const
{
	const char *ina_type = get_ina_type(dev_type);

	if (ina_type == nullptr) {
		return;
	}

	char param_name[PARAM_MAX_LEN];
	snprintf(param_name, sizeof(param_name), "INA%s_CURRENT", ina_type);
	set_float_param(param_name, max_current);
}

void PowerMonitorSelectorAuterion::set_shunt_value(const uint16_t dev_type, const float shunt_value) const
{
	const char *ina_type = get_ina_type(dev_type);

	if (ina_type == nullptr) {
		return;
	}

	char param_name[PARAM_MAX_LEN];
	snprintf(param_name, sizeof(param_name), "INA%s_SHUNT", ina_type);
	set_float_param(param_name, shunt_value);
}

void PowerMonitorSelectorAuterion::set_float_param(const char *param_name, const float param_val) const
{
	float current_param_value = 0;
	param_get(param_find(param_name), &current_param_value);

	if (fabsf(current_param_value - param_val) > FLT_EPSILON) {
		param_set(param_find(param_name), &(param_val));
	}
}

const char *PowerMonitorSelectorAuterion::get_ina_type(const uint16_t dev_type) const
{
	switch (dev_type) {
	case DRV_POWER_DEVTYPE_INA220:
		return "220";

	case DRV_POWER_DEVTYPE_INA226:
		return "226";

	case DRV_POWER_DEVTYPE_INA228:
		return "228";

	case DRV_POWER_DEVTYPE_INA238:
		return "238";

	default:
		return nullptr;
	}
}

uint16_t PowerMonitorSelectorAuterion::crc16_update(const uint16_t current_crc, const uint8_t *data_p,
		size_t length) const
{
	uint8_t x;
	uint16_t crc = current_crc;

	while (length--) {
		x = crc >> 8 ^ *data_p++;
		x ^= x >> 4;
		crc = static_cast<uint16_t>((crc << 8) ^ (x << 12) ^ (x << 5) ^ x);
	}

	return crc;
}

int PowerMonitorSelectorAuterion::task_spawn(int argc, char *argv[])
{
	PowerMonitorSelectorAuterion *instance = new PowerMonitorSelectorAuterion();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int PowerMonitorSelectorAuterion::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int PowerMonitorSelectorAuterion::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for starting and auto-detecting different power monitors.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("pm_selector_auterion", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int pm_selector_auterion_main(int argc, char *argv[])
{
	return PowerMonitorSelectorAuterion::main(argc, argv);
}
