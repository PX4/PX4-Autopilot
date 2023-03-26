/****************************************************************************
 *
 *   Copyright (C) 2022 PX4 Development Team. All rights reserved.
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

#include "Inven_Sense_ICP201XX_registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/device/i2c.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_baro.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace Inven_Sense_ICP201XX;

class ICP201XX : public device::I2C, public I2CSPIDriver<ICP201XX>
{
public:
	ICP201XX(const I2CSPIDriverConfig &config);
	~ICP201XX() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	int probe() override;

	bool Reset();

	int read_reg(Register reg, uint8_t *val);
	int read_reg(Register reg, uint8_t *buf, uint8_t len);
	int write_reg(Register reg, uint8_t val);
	int read_otp_data(uint8_t addr, uint8_t cmd, uint8_t *dat);
	bool get_sensor_data(float *pressure, float *temperature);
	int mode_select(uint8_t mode);
	void dummy_reg();
	bool flush_fifo();
	bool configure();

	uORB::PublicationMulti<sensor_baro_s> _sensor_baro_pub{ORB_ID(sensor_baro)};

	perf_counter_t _reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": reset")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};

	hrt_abstime _reset_timestamp{0};
	int _failure_count{0};

	enum class STATE : uint8_t {
		SOFT_RESET = 0,
		OTP_BOOTUP_CFG,
		CONFIG,
		WAIT_READ,
		READ
	} _state{STATE::SOFT_RESET};

	enum class OP_MODE : uint8_t {
		OP_MODE0 = 0,   /* Mode 0: Bw:6.25 Hz ODR: 25Hz */
		OP_MODE1,       /* Mode 1: Bw:30 Hz ODR: 120Hz */
		OP_MODE2,       /* Mode 2: Bw:10 Hz ODR: 40Hz */
		OP_MODE3,       /* Mode 3: Bw:0.5 Hz ODR: 2Hz */
		OP_MODE4,       /* Mode 4: User configurable Mode */
	} _op_mode{OP_MODE::OP_MODE2};
	enum class FIFO_READOUT_MODE : uint8_t {
		FIFO_READOUT_MODE_PRES_TEMP = 0,   /* Pressure and temperature as pair and address wraps to the start address of the Pressure value ( pressure first ) */
		FIFO_READOUT_MODE_TEMP_ONLY = 1,   /* Temperature only reporting */
		FIFO_READOUT_MODE_TEMP_PRES = 2,   /* Pressure and temperature as pair and address wraps to the start address of the Temperature value ( Temperature first ) */
		FIFO_READOUT_MODE_PRES_ONLY = 3    /* Pressure only reporting */
	} _fifo_readout_mode{FIFO_READOUT_MODE::FIFO_READOUT_MODE_PRES_TEMP};
	enum class POWER_MODE : uint8_t {
		POWER_MODE_NORMAL = 0,  /* Normal Mode: Device is in standby and goes to active mode during the execution of a measurement */
		POWER_MODE_ACTIVE = 1   /* Active Mode: Power on DVDD and enable the high frequency clock */
	} _power_mode{POWER_MODE::POWER_MODE_NORMAL};
	enum MEAS_MODE : uint8_t {
		MEAS_MODE_FORCED_TRIGGER = 0, /* Force trigger mode based on icp201xx_forced_meas_trigger_t **/
		MEAS_MODE_CONTINUOUS = 1   /* Continuous measurements based on selected mode ODR settings*/
	} _meas_mode{MEAS_MODE::MEAS_MODE_CONTINUOUS};
	enum FORCED_MEAS_TRIGGER : uint8_t {
		FORCE_MEAS_STANDBY = 0,			/* Stay in Stand by */
		FORCE_MEAS_TRIGGER_FORCE_MEAS = 1	/* Trigger for forced measurements */
	} _forced_meas_trigger{FORCED_MEAS_TRIGGER::FORCE_MEAS_STANDBY};
};
