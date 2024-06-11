/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "Murata_SCH16T_registers.hpp"
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>

using namespace Murata_SCH16T;

class SCH16T : public device::SPI, public I2CSPIDriver<SCH16T>, public ModuleParams
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

	struct RegisterConfig {
		RegisterConfig(uint16_t a = 0, uint16_t v = 0)
			: addr(a)
			, value(v)
		{};
		uint8_t addr;
		uint16_t value;
	};

	int probe() override;
	void exit_and_cleanup() override;

	bool ValidateSensorStatus();
	bool ValidateRegisterConfiguration();
	void Reset();
	void ResetSpi6(bool reset);
	uint8_t CalculateCRC8(uint64_t frame);

	bool ReadData(SensorData *data);
	void ReadStatusRegisters();

	void Configure();
	void ConfigurationFromParameters();

	void SoftwareReset();

	void RegisterWrite(uint8_t addr, uint16_t value);
	uint64_t RegisterRead(uint8_t addr);
	uint64_t TransferSpiFrame(uint64_t frame);

	static int DataReadyInterruptCallback(int irq, void *context, void *arg);
	void DataReady();
	bool DataReadyInterruptConfigure();
	bool DataReadyInterruptDisable();
private:
	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;

	SensorStatus _sensor_status{};

	int _failure_count{0};

	px4::atomic<hrt_abstime> _drdy_timestamp_sample{0};
	const spi_drdy_gpio_t _drdy_gpio;
	bool _hardware_reset_available{false};

	enum class STATE : uint8_t {
		RESET_INIT,
		RESET_HARD,
		CONFIGURE,
		LOCK_CONFIGURATION,
		VALIDATE,
		READ,
	} _state{STATE::RESET_INIT};

	RegisterConfig _registers[6];

	uint32_t _sample_interval_us = 678;

	perf_counter_t _reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": reset")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	perf_counter_t _perf_crc_bad{perf_counter_t(perf_alloc(PC_COUNT, MODULE_NAME": CRC8 bad"))};
	perf_counter_t _drdy_missed_perf{nullptr};
	perf_counter_t _perf_general_error{perf_counter_t(perf_alloc(PC_COUNT, MODULE_NAME": general error"))};
	perf_counter_t _perf_command_error{perf_counter_t(perf_alloc(PC_COUNT, MODULE_NAME": command error"))};
	perf_counter_t _perf_saturation_error{perf_counter_t(perf_alloc(PC_COUNT, MODULE_NAME": saturation error"))};
	perf_counter_t _perf_doing_initialization{perf_counter_t(perf_alloc(PC_COUNT, MODULE_NAME": re-initializing"))};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SCH16T_GYRO_FILT>) _sch16t_gyro_filt,
		(ParamInt<px4::params::SCH16T_ACC_FILT>) _sch16t_acc_filt,
		(ParamInt<px4::params::SCH16T_DECIM>) _sch16t_decim
	)
};
