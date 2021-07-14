/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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

#include "MaierTek_MPC2520_registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/device/i2c.h>
#include <lib/drivers/barometer/PX4Barometer.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace MaierTek_MPC2520;

class MPC2520 : public device::I2C, public I2CSPIDriver<MPC2520>
{
public:
	MPC2520(const I2CSPIDriverConfig &config);
	~MPC2520() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	// Sensor Configuration
	struct register_config_t {
		Register reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	int probe() override;

	bool Reset();

	bool Configure();

	bool RegisterCheck(const register_config_t &reg_cfg);

	uint8_t RegisterRead(Register reg);
	void RegisterWrite(Register reg, uint8_t value);
	void RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits);

	PX4Barometer _px4_baro;

	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	perf_counter_t _reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": reset")};

	hrt_abstime _reset_timestamp{0};
	hrt_abstime _last_config_check_timestamp{0};
	int _failure_count{0};

#pragma pack(push,1)
	struct prom_s {
		int16_t c0;
		int16_t c1;
		int32_t c00;
		int32_t c10;
		int16_t c01;
		int16_t c11;
		int16_t c20;
		int16_t c21;
		int16_t c30;
	} _prom{};
#pragma pack(pop)

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		READ_PROM,
		CONFIGURE,
		READ,
	} _state{STATE::RESET};

	uint8_t _checked_register{0};
	static constexpr uint8_t size_register_cfg{4};
	register_config_t _register_cfg[size_register_cfg] {
		// Register               | Set bits, Clear bits
		{ Register::PRS_CFG,      PRS_CFG_BIT::PM_RATE_32_SET | PRS_CFG_BIT::PM_PRC_8_SET, PRS_CFG_BIT::PM_RATE_32_CLEAR | PRS_CFG_BIT::PM_PRC_8_CLEAR },
		{ Register::TMP_CFG,      TMP_CFG_BIT::TMP_EXT | TMP_CFG_BIT::TMP_RATE_32_SET | TMP_CFG_BIT::TMP_PRC_8_SET, TMP_CFG_BIT::TMP_RATE_32_CLEAR | TMP_CFG_BIT::TMP_PRC_8_CLEAR },
		{ Register::MEAS_CFG,     MEAS_CFG_BIT::MEAS_CTRL_CONT_PRES_TEMP, 0 },
		{ Register::CFG_REG,      0, CFG_REG_BIT::T_SHIFT | CFG_REG_BIT::P_SHIFT },
	};
};
