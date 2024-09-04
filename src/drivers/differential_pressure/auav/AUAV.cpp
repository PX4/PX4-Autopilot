/****************************************************************************
 *
 *   Copyright (c) 2017-2024 PX4 Development Team. All rights reserved.
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

#include "AUAV.hpp"
#include "AUAV_Absolute.hpp"
#include "AUAV_Differential.hpp"

I2CSPIDriverBase *AUAV::instantiate(const I2CSPIDriverConfig &config, int runtime_instance)
{
	AUAV *instance = nullptr;

	if (config.devid_driver_index == DRV_DIFF_PRESS_DEVTYPE_AUAV) {
		instance = new AUAV_Differential(config);

	} else if (config.devid_driver_index == DRV_BARO_DEVTYPE_AUAV) {
		instance = new AUAV_Absolute(config);
	}

	if (!instance) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (instance->init() != OK) {
		delete instance;
		return nullptr;
	}

	return instance;
}

AUAV::AUAV(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms errors"))
{
}

AUAV::~AUAV()
{
	perf_free(_comms_errors);
}

int AUAV::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	ScheduleClear();
	ScheduleNow();
	return OK;
}

int AUAV::read_calibration_eeprom(uint8_t eeprom_address, uint16_t &data)
{
	uint8_t req_data[3] = {eeprom_address, 0x0, 0x0};
	int status = transfer(req_data, sizeof(req_data), nullptr, 0);

	/* Wait for the EEPROM read access. Worst case is 2000us */
	px4_usleep(2000);

	uint8_t res_data[3];
	status |= transfer(nullptr, 0, res_data, sizeof(res_data));

	/* If bit 5 is set to 1, the sensor is still busy. This read is considered invalid */
	if (res_data[0] & 0x20) {
		status = PX4_ERROR;
	}

	data = res_data[1] << 8 | res_data[2];
	return status;
}
