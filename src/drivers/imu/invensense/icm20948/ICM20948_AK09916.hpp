/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file ICM20948_AK09916.hpp
 *
 * Driver for the AKM AK09916 connected via I2C.
 *
 */

#pragma once

#include "AKM_AK09916_registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/device/i2c.h>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

class ICM20948;

namespace AKM_AK09916
{

class ICM20948_AK09916 : public px4::ScheduledWorkItem
{
public:
	ICM20948_AK09916(ICM20948 &icm20948, enum Rotation rotation = ROTATION_NONE);
	~ICM20948_AK09916() override;

	bool Init();
	bool Reset();
	void PrintInfo();

	void set_temperature(float temperature) { _px4_mag.set_temperature(temperature); }

private:

	struct TransferBuffer {
		uint8_t ST1;
		uint8_t HXL;
		uint8_t HXH;
		uint8_t HYL;
		uint8_t HYH;
		uint8_t HZL;
		uint8_t HZH;
		uint8_t TMPS;
		uint8_t ST2;
	};

	struct register_config_t {
		AKM_AK09916::Register reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	int probe();

	void Run() override;

	bool Configure();

	bool RegisterCheck(const register_config_t &reg_cfg, bool notify = false);

	uint8_t RegisterRead(AKM_AK09916::Register reg);
	void RegisterWrite(AKM_AK09916::Register reg, uint8_t value);
	void RegisterSetAndClearBits(AKM_AK09916::Register reg, uint8_t setbits, uint8_t clearbits);

	ICM20948 &_icm20948;

	PX4Magnetometer _px4_mag;

	perf_counter_t _transfer_perf{perf_alloc(PC_ELAPSED, MODULE_NAME"_ak09916: transfer")};
	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME"_ak09916: bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME"_ak09916: bad transfer")};
	perf_counter_t _duplicate_data_perf{perf_alloc(PC_COUNT, MODULE_NAME"_ak09916: duplicate data")};
	perf_counter_t _data_not_ready{perf_alloc(PC_COUNT, MODULE_NAME"_ak09916: data not ready")};

	hrt_abstime _reset_timestamp{0};
	hrt_abstime _last_config_check_timestamp{0};

	int16_t _last_measurement[3] {};

	uint8_t _checked_register{0};

	enum class STATE : uint8_t {
		RESET,
		READ_WHO_AM_I,
		WAIT_FOR_RESET,
		CONFIGURE,
		READ,
	} _state{STATE::RESET};;

	static constexpr uint8_t size_register_cfg{1};
	register_config_t _register_cfg[size_register_cfg] {
		// Register                       | Set bits, Clear bits
		{ AKM_AK09916::Register::CNTL2,   AKM_AK09916::CNTL2_BIT::MODE3, (uint8_t)~AKM_AK09916::CNTL2_BIT::MODE3 },
	};
};

} // namespace AKM_AK09916
