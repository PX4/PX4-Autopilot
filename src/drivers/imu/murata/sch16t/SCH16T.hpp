/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file SCH16T.hpp
 *
 * Driver for the Murata SCH16T connected via SPI.
 *
 */

#pragma once

#include "Murata_SCH16T_registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/geo/geo.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/i2c_spi_buses.h>

#define SPI48_DATA_INT32(a)         (((int32_t)(((a) << 4)  & 0xfffff000UL)) >> 12)
#define SPI48_DATA_UINT32(a)        ((uint32_t)(((a) >> 8)  & 0x000fffffUL))
#define SPI48_DATA_UINT16(a)        ((uint16_t)(((a) >> 8)  & 0x0000ffffUL))

using namespace Murata_SCH16T;

class SCH16T : public device::SPI, public I2CSPIDriver<SCH16T>
{
public:
	SCH16T(const I2CSPIDriverConfig &config);
	~SCH16T() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	struct SensorData {
		int32_t acc_x;
		int32_t acc_y;
		int32_t acc_z;
		int32_t gyro_x;
		int32_t gyro_y;
		int32_t gyro_z;
		int32_t temp;
		bool frame_error;
	};

	struct SensorStatus {
		uint16_t summary;
		uint16_t saturation;
		uint16_t common;
		uint16_t rate_common;
		uint16_t rate_x;
		uint16_t rate_y;
		uint16_t rate_z;
		uint16_t acc_x;
		uint16_t acc_y;
		uint16_t acc_z;
	};

	int probe() override;
	void exit_and_cleanup() override;

	bool ValidateSensorStatus();
	bool ValidateRegisterConfiguration();

	void Reset();
	void Configure();
	SensorData ReadData();
	uint8_t CalculateCRC8(uint64_t frame);

	void SoftwareReset();
	void ReadStatusRegisters();

	// Non-data registers are 16 bit or less
	uint64_t RegisterRead(uint8_t addr);
	void RegisterWrite(uint8_t addr, uint16_t value);

	uint64_t TransferSpiFrame(uint64_t frame);

	// Data Ready functions
	static int DataReadyInterruptCallback(int irq, void *context, void *arg);
	void DataReady();
	bool DataReadyInterruptConfigure();
	bool DataReadyInterruptDisable();
private:

	SensorStatus _sensor_status{};

	const spi_drdy_gpio_t _drdy_gpio;
	bool _hardware_reset_available{false};

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;

	hrt_abstime _reset_timestamp{0};
	hrt_abstime _last_config_check_timestamp{0};
	int _failure_count{0};

	px4::atomic<hrt_abstime> _drdy_timestamp_sample{0};

	enum class STATE : uint8_t {
		RESET_INIT,
		RESET_HARD,
		CONFIGURE,
		VALIDATE,
		READ,
	} _state{STATE::RESET_INIT};

	perf_counter_t _reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": reset")};
	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	perf_counter_t _perf_crc_bad{perf_counter_t(perf_alloc(PC_COUNT, MODULE_NAME": CRC8 bad"))};
	perf_counter_t _perf_frame_bad{perf_counter_t(perf_alloc(PC_COUNT, MODULE_NAME": Frame bad"))};

	perf_counter_t _drdy_missed_perf{nullptr};
};
