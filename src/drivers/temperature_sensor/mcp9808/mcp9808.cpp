/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include "mcp9808.h"

MCP9808::MCP9808(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": single-sample")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms errors"))
{
	_sensor_temp.device_id = get_device_id();
}

MCP9808::~MCP9808()
{
	ScheduleClear();
	perf_free(_cycle_perf);
	perf_free(_comms_errors);
}

void MCP9808::RunImpl()
{
	perf_begin(_cycle_perf);

	float temperature = read_temperature();

	if (std::isnan(temperature)) {

		perf_count(_comms_errors);

	} else {
		_sensor_temp.timestamp = hrt_absolute_time();
		_sensor_temp.temperature = temperature;

		_sensor_temp_pub.publish(_sensor_temp);
	}

	perf_end(_cycle_perf);
}

int MCP9808::probe()
{
	uint16_t manuf_id, device_id;
	int ret = read_reg(MCP9808_REG_MANUF_ID, manuf_id);

	if (ret != PX4_OK) {
		return ret;
	}

	ret = read_reg(MCP9808_REG_DEVICE_ID, device_id);

	if (ret != PX4_OK) {
		return ret;
	}

	// Verify manufacturer and device ID
	if (manuf_id != 0x0054 || device_id != 0x0400) {
		PX4_ERR("MCP9808 not found (manuf_id: 0x%04X, device_id: 0x%04X)", manuf_id, device_id);
		return PX4_ERROR;
	}

	PX4_INFO("MCP9808 found (manuf_id: 0x%04X, device_id: 0x%04X)", manuf_id, device_id);
	return PX4_OK;
}

int MCP9808::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		PX4_ERR("I2C init failed");
		return ret;
	}

	PX4_DEBUG("I2C initialized successfully");

	ret = write_reg(MCP9808_REG_CONFIG, 0x0000); // Ensure default configuration

	if (ret != PX4_OK) {
		PX4_ERR("Configuration failed");
		return ret;
	}

	_sensor_temp_pub.advertise();

	ScheduleOnInterval(200_ms); // Sample at 5 Hz
	return PX4_OK;
}

float MCP9808::read_temperature()
{
	uint16_t temp_raw = 0;

	if (read_reg(MCP9808_REG_AMBIENT_TEMP, temp_raw) != PX4_OK) {
		return NAN;
	}

	float temp = (temp_raw & 0x0FFF) / 16.0f;

	if (temp_raw & 0x1000) {
		temp -= 256.0f;
	}

	return temp;
}

int MCP9808::read_reg(uint8_t address, uint16_t &data)
{
	uint8_t addr = address;
	uint8_t buffer[2] = {}; // Buffer to hold the raw data

	// Read 2 bytes from the register
	int ret = transfer(&addr, 1, buffer, 2);

	if (ret != PX4_OK) {
		return ret;
	}

	// Combine bytes (big-endian)
	data = (buffer[0] << 8) | buffer[1];

	return PX4_OK;
}

int MCP9808::write_reg(uint8_t address, uint16_t value)
{
	uint8_t buf[3] = {address, static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF)};
	return transfer(buf, sizeof(buf), nullptr, 0);
}
