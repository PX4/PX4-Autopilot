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

#include "SCH16T_FPGA_driver.hpp"
#include <cstdint>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>

using namespace Murata_SCH16T_FPGA;

class ZEROONE_SCH16T : public device::SPI, public I2CSPIDriver<ZEROONE_SCH16T>, public ModuleParams,
	public Murata_SCH16T_FPGA::FpgaSpiInterface
{
	friend class Murata_SCH16T_FPGA::SCH16T_FPGA_driver;
public:
	ZEROONE_SCH16T(const I2CSPIDriverConfig &config);
	~ZEROONE_SCH16T() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
#define SPI_2Mhz 31//2Mhz = 1/(（31 - 1）*16.67ns)
#define CtrlMode_Direct 0
#define CtrlMode_FpgaRead 1
#define FIFO_RESET 1
#define FIFO_NRESET 0
	static constexpr float FIFO_SAMPLE_DT {1e6f / 1475.f};
	static constexpr float GYRO_RATE{1e6f / FIFO_SAMPLE_DT};   // 1475 Hz gyro

	uint8_t _direct_mode = CtrlMode_Direct;
	uint8_t _fifo_enable = 0;
	uint8_t _fifo_cmd_num = 17;
	uint8_t _fifo_baudrate = SPI_2Mhz;
	px4::atomic<hrt_abstime> _drdy_timestamp_sample{0};

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

	struct SensorData {
		int32_t acc_x;
		int32_t acc_y;
		int32_t acc_z;
		int32_t gyro_x;
		int32_t gyro_y;
		int32_t gyro_z;
		int32_t temp;
	};

	struct RegisterConfig {
		RegisterConfig(uint16_t a = 0, uint16_t v = 0)
			: addr(a)
			, value(v)
		{};
		uint8_t addr;
		uint16_t value;
	};

	// Store raw FIFO data for all packets to handle 20-bit rescale correctly
	struct RawFifoData {
		int32_t gyro_x;
		int32_t gyro_y;
		int32_t gyro_z;
		int32_t acc_x;
		int32_t acc_y;
		int32_t acc_z;
		int32_t temp;
		uint8_t time_stamp;
		uint8_t buff[24]; // Store raw buffer for 20-bit rescale
	};

	RegisterConfig _registers[6];
	SensorStatus _sensor_status;

	SCH16T_FPGA_driver *_fpga_driver{nullptr};

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;

	// FpgaSpiInterface implementation
	void transfer(uint8_t *send, uint8_t *recv, unsigned int len) override;

	int probe() override;
	void exit_and_cleanup() override;
	void fpga_init(void);
	bool read_data() ;
	void reset_chip();
	bool read_product_id();
	void configure_registers();
	bool validate_sensor_status();
	bool validate_register_configuration();
	void read_status_registers();
	uint8_t register_read(uint16_t addr, uint32_t *value);
	void register_write(uint16_t addr, uint32_t value);
	uint8_t wait_direct_busy(void);
	uint16_t _sch16t_read_status(uint16_t addr);

#if defined(DEBUG_BUILD)
	uint8_t fpga_test(void);
#endif

	perf_counter_t _reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": reset")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	perf_counter_t _perf_crc_bad{perf_counter_t(perf_alloc(PC_COUNT, MODULE_NAME": CRC8 bad"))};
	perf_counter_t _fifo_empty_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO empty")};
	perf_counter_t _perf_general_error{perf_counter_t(perf_alloc(PC_COUNT, MODULE_NAME": general error"))};
	perf_counter_t _perf_command_error{perf_counter_t(perf_alloc(PC_COUNT, MODULE_NAME": command error"))};
	perf_counter_t _perf_saturation_error{perf_counter_t(perf_alloc(PC_COUNT, MODULE_NAME": saturation error"))};
	perf_counter_t _perf_doing_initialization{perf_counter_t(perf_alloc(PC_COUNT, MODULE_NAME": re-initializing"))};

	uint16_t _fifo_empty_interval_us{678};// default 678 us
	int32_t _fifo_gyro_samples{static_cast<int32_t>(_fifo_empty_interval_us / (1000000 / GYRO_RATE))};

	enum class State : uint8_t {
		PowerOn,
		Reset,
		Configure,
		LockConfiguration,
		Validate,
		Read,
	} _state = State::PowerOn;

	int _failure_count {0};
};
