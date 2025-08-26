/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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

#include <stdarg.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_sensor.h>
#include <lib/crc/crc.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>

using namespace time_literals;

class I2CWrapper
{

public:
	explicit I2CWrapper(int bus_number)
	{
		_i2c = px4_i2cbus_initialize(bus_number);
	}

	~I2CWrapper()
	{
		if (_i2c != nullptr) {
			px4_i2cbus_uninitialize(_i2c);
		}
	}

	I2CWrapper(const I2CWrapper &) = delete;
	I2CWrapper &operator=(const I2CWrapper &) = delete;

	struct i2c_master_s *get() const { return _i2c; }
	bool is_valid() const { return _i2c != nullptr; }

private:
	struct i2c_master_s *_i2c {nullptr};
};

class PowerMonitorSelectorAuterion : public ModuleBase<PowerMonitorSelectorAuterion>, public px4::ScheduledWorkItem
{

public:
	PowerMonitorSelectorAuterion();
	virtual ~PowerMonitorSelectorAuterion();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

private:
	enum BlockType : uint8_t {
		TYPE_PM = 0
	};

	struct Buses {
		const uint16_t eeprom_i2c_addr;
		const uint8_t bus_number;
		bool started;
		const uint16_t id;
	};

	struct Sensor {
		const uint16_t dev_type;
		const uint16_t i2c_addr;
		const uint8_t bus_number;
		const float shunt_value;
		bool started;
		const uint16_t id;
	};

#pragma pack(push, 1)
	/* Careful when changing the layout. When doing so, adapt CRC calculation! */
	struct EepromHeader {
		uint16_t magic;	/**< offset 0 */
		uint16_t version; /**< offset 2 */
		uint16_t crc; /**< offset 4 */
		uint16_t flags; /**< offset 6 */
		uint16_t num_blocks; /**< offset 8 */
		uint8_t _reserved1[2]; /**< offset 10 */
	};
#pragma pack(pop)

#pragma pack(push, 1)
	struct EepromBlockHeader {
		uint8_t block_type; /**< offset 0 */
		uint8_t block_type_version; /**< offset 1 */
		uint16_t block_length; /**< offset 2 */
	};
#pragma pack(pop)

#pragma pack(push, 1)
	/* Block n starts at 12 + (n * 20) */
	struct EepromBlockPm {
		EepromBlockHeader block_header; /**< offset 0 */
		uint16_t dev_type; /**< offset 4 */
		uint16_t sensor_type; /**< offset 6 */
		uint16_t i2c_addr; /**< offset 8 */
		uint8_t _padding1[2]; /**< offset 10 */
		float max_current; /**< offset 12 */
		float shunt_value; /**< offset 16 */
	};
#pragma pack(pop)

	void Run() override;

	bool init();

	void try_eeprom_start();
	void try_probe_start();

	int ina226_probe(const uint32_t instance) const;
	int eeprom_read(const uint32_t instance);
	bool is_eeprom_header_valid(EepromHeader *eeprom_header) const;

	int eeprom_read_block(struct i2c_master_s *i2c, const uint32_t instance, const uint16_t transferred_blocks,
			      uint16_t &crc);
	bool is_eeprom_block_header_valid(EepromBlockHeader *eeprom_block_header) const;

	int start_pm(const uint8_t bus_number, const uint16_t dev_type, const uint16_t i2c_addr, const uint16_t id) const;
	const char *get_start_command(const uint16_t dev_type) const;

	bool is_user_configured(const uint16_t dev_type) const;
	void set_max_current(const uint16_t dev_type, const float max_current) const;
	void set_shunt_value(const uint16_t dev_type, const float shunt_value) const;
	void set_float_param(const char *param_name, const float param_val) const;
	const char *get_ina_type(const uint16_t dev_type) const;
	uint16_t crc16_update(const uint16_t current_crc, const uint8_t *data_p, size_t length) const;

	uORB::Subscription	_actuator_armed_sub{ORB_ID(actuator_armed)};		///< system armed control topic
	int _eeprom_only_run{0};

	static constexpr int RUN_INTERVAL{500_ms};
	static constexpr int EEPROM_ONLY_RUNS{2};
	static constexpr int BUSES_NUMBERS{2};
	static constexpr int SENSORS_NUMBER{4};

	static constexpr uint16_t EEPROM_MAGIC = 0xAFFA;
	static constexpr uint16_t EEPROM_VERSION = 1;
	static constexpr uint16_t EEPROM_MAX_BLOCKS = 5;
	static constexpr uint8_t EEPROM_BLOCK_TYPE_VERSION = 1;

	static constexpr size_t BUS_MAX_LEN = 4;
	static constexpr size_t I2C_ADDR_MAX_LEN = 4;
	static constexpr size_t ID_MAX_LEN = 6;
	static constexpr size_t PARAM_MAX_LEN = 17;

	Buses _buses[BUSES_NUMBERS] = {
		{
			.eeprom_i2c_addr = 0x50,
			.bus_number = 1,
			.started = false,
			.id = 1
		},
		{
			.eeprom_i2c_addr = 0x50,
			.bus_number = 2,
			.started = false,
			.id = 2
		}
	};

	Sensor _sensors[SENSORS_NUMBER] = {
		{
			.dev_type = DRV_POWER_DEVTYPE_INA226,
			.i2c_addr = 0x41,
			.bus_number = 1,
			.shunt_value = 0.0008f,
			.started = false,
			.id = 1
		},
		{
			.dev_type = DRV_POWER_DEVTYPE_INA226,
			.i2c_addr = 0x40,
			.bus_number = 1,
			.shunt_value = 0.0005f,
			.started = false,
			.id = 1
		},
		{
			.dev_type = DRV_POWER_DEVTYPE_INA226,
			.i2c_addr = 0x41,
			.bus_number = 2,
			.shunt_value = 0.0008f,
			.started = false,
			.id = 2
		},
		{
			.dev_type = DRV_POWER_DEVTYPE_INA226,
			.i2c_addr = 0x40,
			.bus_number = 2,
			.shunt_value = 0.0005f,
			.started = false,
			.id = 2
		}
	};

	EepromBlockPm _eeprom_blocks_pm[EEPROM_MAX_BLOCKS] = { 0 };
	uint16_t _eeprom_valid_blocks_pm = 0;
};
