/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "ICM20602.hpp"

#include <drivers/boards/common/board_dma_alloc.h>

using namespace time_literals;
using namespace InvenSense_ICM20602;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb) { return (msb << 8u) | lsb; }

static constexpr uint32_t _fifo_interval{1000};						// 1000 us / 1000 Hz interval
static constexpr uint32_t _gyro_rate{8000}; 						// 8 kHz gyro
static constexpr uint32_t _gyro_readings_per_sample{_fifo_interval / (1000000 / _gyro_rate)};
static constexpr uint32_t _accel_rate{4000}; 						// 4 kHz accel
static constexpr uint32_t _accel_readings_per_sample{_fifo_interval / (1000000 / _accel_rate)};

ICM20602::ICM20602(int bus, uint32_t device, enum Rotation rotation) :
	SPI(MODULE_NAME, nullptr, bus, device, SPIDEV_MODE3, SPI_SPEED),
	ScheduledWorkItem(MODULE_NAME, px4::device_bus_to_wq(get_device_id())),
	_px4_accel(get_device_id(), ORB_PRIO_VERY_HIGH, rotation),
	_px4_gyro(get_device_id(), ORB_PRIO_VERY_HIGH, rotation)
{
	set_device_type(DRV_ACC_DEVTYPE_ICM20602);
	_px4_accel.set_device_type(DRV_ACC_DEVTYPE_ICM20602);
	_px4_gyro.set_device_type(DRV_GYR_DEVTYPE_ICM20602);

	_px4_accel.set_sample_rate(_accel_rate);
	_px4_gyro.set_sample_rate(_gyro_rate);

	_px4_accel.set_update_rate(1000000 / _fifo_interval);
	_px4_gyro.set_update_rate(1000000 / _fifo_interval);
}

ICM20602::~ICM20602()
{
	Stop();

	if (_dma_data_buffer != nullptr) {
		board_dma_free(_dma_data_buffer, FIFO::SIZE);
	}

	perf_free(_interval_perf);
	perf_free(_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_drdy_count_perf);
	perf_free(_drdy_interval_perf);
}

int
ICM20602::probe()
{
	uint8_t whoami = RegisterRead(Register::WHO_AM_I);

	if (whoami != ICM20602_WHOAMI) {
		PX4_WARN("unexpected WHO_AM_I 0x%02x", whoami);
		return PX4_ERROR;
	}

	return OK;
}

bool
ICM20602::Init()
{
	if (SPI::init() != PX4_OK) {
		PX4_ERR("SPI::init failed");
		return false;
	}

	if (!Reset()) {
		PX4_ERR("reset failed");
		return false;
	}

	// allocate DMA capable buffer
	_dma_data_buffer = (uint8_t *)board_dma_alloc(FIFO::SIZE);

	if (_dma_data_buffer == nullptr) {
		PX4_ERR("DMA alloc failed");
		return false;
	}

	Start();

	return true;
}

bool
ICM20602::Reset()
{
	for (int i = 0; i < 5; i++) {

		// PWR_MGMT_1: Device Reset
		// CLKSEL[2:0] must be set to 001 to achieve full gyroscope performance.
		RegisterWrite(Register::PWR_MGMT_1, PWR_MGMT_1_BIT::DEVICE_RESET);
		usleep(1000);

		// PWR_MGMT_1: CLKSEL[2:0] must be set to 001 to achieve full gyroscope performance.
		RegisterWrite(Register::PWR_MGMT_1, PWR_MGMT_1_BIT::CLKSEL_0);
		usleep(1000);

		// ACCEL_CONFIG: Accel 16 G range
		RegisterSetBits(Register::ACCEL_CONFIG, ACCEL_CONFIG_BIT::ACCEL_FS_SEL_16G);
		_px4_accel.set_scale(CONSTANTS_ONE_G / 2048);
		_px4_accel.set_range(16.0f * CONSTANTS_ONE_G);

		// GYRO_CONFIG: Gyro 2000 degrees/second
		RegisterSetBits(Register::GYRO_CONFIG, GYRO_CONFIG_BIT::FS_SEL_2000_DPS);
		_px4_gyro.set_scale(math::radians(1.0f / 16.4f));
		_px4_gyro.set_range(math::radians(2000.0f));

		const bool reset_done = !(RegisterRead(Register::PWR_MGMT_1) & PWR_MGMT_1_BIT::DEVICE_RESET);
		const bool clksel_done = (RegisterRead(Register::PWR_MGMT_1) & PWR_MGMT_1_BIT::CLKSEL_0);
		const bool data_ready = (RegisterRead(Register::INT_STATUS) & INT_STATUS_BIT::DATA_RDY_INT);

		// reset done once data is ready
		if (reset_done && clksel_done && data_ready) {
			return true;
		}
	}

	return false;
}

void
ICM20602::ResetFIFO()
{
	perf_count(_fifo_reset_perf);

	// ACCEL_CONFIG2: Accel DLPF disabled for full rate (4 kHz)
	RegisterSetBits(Register::ACCEL_CONFIG2, ACCEL_CONFIG2_BIT::ACCEL_FCHOICE_B_BYPASS_DLPF);

	// GYRO_CONFIG: Gyro DLPF disabled for full rate (8 kHz)
	RegisterClearBits(Register::GYRO_CONFIG, GYRO_CONFIG_BIT::FCHOICE_B_8KHZ_BYPASS_DLPF);

	// FIFO_EN: disable FIFO
	RegisterWrite(Register::FIFO_EN, 0);
	RegisterClearBits(Register::USER_CTRL, USER_CTRL_BIT::FIFO_EN | USER_CTRL_BIT::FIFO_RST);

	// USER_CTRL: reset FIFO then re-enable
	RegisterSetBits(Register::USER_CTRL, USER_CTRL_BIT::FIFO_RST);
	up_udelay(1); // bit auto clears after one clock cycle of the internal 20 MHz clock
	RegisterSetBits(Register::USER_CTRL, USER_CTRL_BIT::FIFO_EN);

	// CONFIG: should ensure that bit 7 of register 0x1A is set to 0 before using FIFO watermark feature
	RegisterSetBits(Register::CONFIG, CONFIG_BIT::FIFO_MODE);
	RegisterClearBits(Register::CONFIG, CONFIG_BIT::FIFO_WM);
	RegisterSetBits(Register::CONFIG, CONFIG_BIT::DLPF_CFG_BYPASS_DLPF_8KHZ);

	// FIFO_EN: enable both gyro and accel
	_data_ready_count = 0;
	RegisterWrite(Register::FIFO_EN, FIFO_EN_BIT::GYRO_FIFO_EN | FIFO_EN_BIT::ACCEL_FIFO_EN);
	up_udelay(10);
}

uint8_t
ICM20602::RegisterRead(Register reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

void
ICM20602::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	transfer(cmd, cmd, sizeof(cmd));
}

void
ICM20602::RegisterSetBits(Register reg, uint8_t setbits)
{
	uint8_t val = RegisterRead(reg);

	if (!(val & setbits)) {
		val |= setbits;
		RegisterWrite(reg, val);
	}
}

void
ICM20602::RegisterClearBits(Register reg, uint8_t clearbits)
{
	uint8_t val = RegisterRead(reg);

	if (val & clearbits) {
		val &= !clearbits;
		RegisterWrite(reg, val);
	}
}

int
ICM20602::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	ICM20602 *dev = reinterpret_cast<ICM20602 *>(arg);

	dev->DataReady();

	return 0;
}

void
ICM20602::DataReady()
{
	perf_count(_drdy_count_perf);
	perf_count(_drdy_interval_perf);

	_data_ready_count++;

	if (_data_ready_count >= 8) {
		_time_data_ready = hrt_absolute_time();

		_data_ready_count = 0;

		// make another measurement
		ScheduleNow();
	}
}

void
ICM20602::Start()
{
	Stop();

	ResetFIFO();

	// TODO: cleanup horrible DRDY define mess
#if defined(GPIO_DRDY_PORTC_PIN14)
	// Setup data ready on rising edge
	px4_arch_gpiosetevent(GPIO_DRDY_PORTC_PIN14, true, false, true, &ICM20602::DataReadyInterruptCallback, this);
	RegisterSetBits(Register::INT_ENABLE, INT_ENABLE_BIT::DATA_RDY_INT_EN);
#elif defined(GPIO_SPI1_DRDY1_ICM20602)
	// Setup data ready on rising edge
	px4_arch_gpiosetevent(GPIO_SPI1_DRDY1_ICM20602, true, false, true, &ICM20602::DataReadyInterruptCallback, this);
	RegisterSetBits(Register::INT_ENABLE, INT_ENABLE_BIT::DATA_RDY_INT_EN);
#elif defined(GPIO_SPI1_DRDY4_ICM20602)
	// Setup data ready on rising edge
	px4_arch_gpiosetevent(GPIO_SPI1_DRDY4_ICM20602, true, false, true, &ICM20602::DataReadyInterruptCallback, this);
	RegisterSetBits(Register::INT_ENABLE, INT_ENABLE_BIT::DATA_RDY_INT_EN);
#elif defined(GPIO_SPI1_DRDY1_ICM20602)
	// Setup data ready on rising edge
	px4_arch_gpiosetevent(GPIO_SPI1_DRDY1_ICM20602, true, false, true, &ICM20602::DataReadyInterruptCallback, this);
	RegisterSetBits(Register::INT_ENABLE, INT_ENABLE_BIT::DATA_RDY_INT_EN);
#elif defined(GPIO_DRDY_ICM_2060X)
	// Setup data ready on rising edge
	px4_arch_gpiosetevent(GPIO_DRDY_ICM_2060X, true, false, true, &ICM20602::DataReadyInterruptCallback, this);
	RegisterSetBits(Register::INT_ENABLE, INT_ENABLE_BIT::DATA_RDY_INT_EN);
#else
	ScheduleOnInterval(_fifo_interval, _fifo_interval);
#endif
}

void
ICM20602::Stop()
{
	// TODO: cleanup horrible DRDY define mess
#if defined(GPIO_DRDY_PORTC_PIN14)
	// Disable data ready callback
	px4_arch_gpiosetevent(GPIO_DRDY_PORTC_PIN14, false, false, false, nullptr, nullptr);
	RegisterClearBits(Register::INT_ENABLE, INT_ENABLE_BIT::DATA_RDY_INT_EN);
#elif defined(GPIO_SPI1_DRDY1_ICM20602)
	// Disable data ready callback
	px4_arch_gpiosetevent(GPIO_SPI1_DRDY1_ICM20602, false, false, false, nullptr, nullptr);
	RegisterClearBits(Register::INT_ENABLE, INT_ENABLE_BIT::DATA_RDY_INT_EN);
#elif defined(GPIO_SPI1_DRDY4_ICM20602)
	// Disable data ready callback
	px4_arch_gpiosetevent(GPIO_SPI1_DRDY4_ICM20602, false, false, false, nullptr, nullptr);
	RegisterClearBits(Register::INT_ENABLE, INT_ENABLE_BIT::DATA_RDY_INT_EN);
#elif defined(GPIO_SPI1_DRDY1_ICM20602)
	// Disable data ready callback
	px4_arch_gpiosetevent(GPIO_SPI1_DRDY1_ICM20602, false, false, false, nullptr, nullptr);
	RegisterClearBits(Register::INT_ENABLE, INT_ENABLE_BIT::DATA_RDY_INT_EN);
#elif defined(GPIO_DRDY_ICM_2060X)
	// Disable data ready callback
	px4_arch_gpiosetevent(GPIO_DRDY_ICM_2060X, false, false, false, nullptr, nullptr);
	RegisterClearBits(Register::INT_ENABLE, INT_ENABLE_BIT::DATA_RDY_INT_EN);
#else
	ScheduleClear();
#endif

}

void
ICM20602::Run()
{
	perf_count(_interval_perf);

	// check for FIFO overflow and reset
	const bool fifo_overflow = (RegisterRead(Register::INT_STATUS) & INT_STATUS_BIT::FIFO_OFLOW_INT);

	if (fifo_overflow) {
		perf_count(_fifo_overflow_perf);
		ResetFIFO();
		return;
	}

	// // check FIFO count
	// uint8_t fifo_count_buf[3] {};
	// fifo_count_buf[0] = static_cast<uint8_t>(Register::FIFO_COUNTH) | DIR_READ;
	// const hrt_abstime timestamp_fifo_check = hrt_absolute_time();

	// if (transfer(fifo_count_buf, fifo_count_buf, sizeof(fifo_count_buf)) != PX4_OK) {
	// 	return;
	// }

	// const size_t fifo_count = combine(fifo_count_buf[1], fifo_count_buf[2]);
	// const int samples = fifo_count / sizeof(FIFO::DATA);

	// if (samples < 1) {
	// 	perf_count(_fifo_empty_perf);
	// 	return;

	// } else if (samples > 32) {
	// 	// check for FIFO overflow and reset
	// 	const bool fifo_overflow = (RegisterRead(Register::INT_STATUS) & INT_STATUS_BIT::FIFO_OFLOW_INT);

	// 	if (fifo_overflow) {
	// 		perf_count(_fifo_overflow_perf);
	// 	}

	// 	ResetFIFO();
	// 	return;
	// }

	// only read 8 samples
	const int samples = 8;

	// Transfer data
	// 1 (cmd) + sizeof(FIFO_TRANSFER) * 32 = 449 bytes
	struct ICM_Report {
		uint8_t cmd;
		FIFO::DATA f[32]; // we never transfer more than 32 samples
	};
	const size_t transfer_size = math::min(samples * sizeof(FIFO::DATA) + 1, FIFO::SIZE);

	ICM_Report *report = (ICM_Report *)_dma_data_buffer;
	memset(report, 0, transfer_size);
	report->cmd = static_cast<uint8_t>(Register::FIFO_R_W) | DIR_READ;

	perf_begin(_transfer_perf);

	if (transfer(_dma_data_buffer, _dma_data_buffer, transfer_size) != PX4_OK) {
		perf_end(_transfer_perf);
		return;
	}

	perf_end(_transfer_perf);


	// Process data (only the latest 8 samples)
	const int num_samples = 8;
	const int first_sample = math::min(0, num_samples - 8);
	const int last_sample = num_samples;


	static constexpr uint32_t gyro_dt = _fifo_interval / _gyro_readings_per_sample;
	// estimate timestamp of first sample in the FIFO from number of samples and fill rate
	const hrt_abstime timestamp_sample = _time_data_ready - ((num_samples - 1) * gyro_dt);

	PX4Accelerometer::FIFOSample accel{};
	accel.timestamp_sample = timestamp_sample;
	accel.dt = _fifo_interval / _accel_readings_per_sample;

	PX4Gyroscope::FIFOSample gyro{};
	gyro.timestamp_sample = timestamp_sample;
	gyro.samples = num_samples;
	gyro.dt = _fifo_interval / _gyro_readings_per_sample;

	int accel_samples = 0;
	int16_t temperature[samples] {};

	for (int i = first_sample; i < last_sample; i++) {
		const FIFO::DATA &fifo_sample = report->f[i];

		// accel data is doubled
		if (i % 2) {
			accel.x[accel_samples] = combine(fifo_sample.ACCEL_XOUT_H, fifo_sample.ACCEL_XOUT_L);
			accel.y[accel_samples] = -combine(fifo_sample.ACCEL_YOUT_H, fifo_sample.ACCEL_YOUT_L);
			accel.z[accel_samples] = -combine(fifo_sample.ACCEL_ZOUT_H, fifo_sample.ACCEL_ZOUT_L);

			accel_samples++;
		}

		temperature[i] = combine(fifo_sample.TEMP_OUT_H, fifo_sample.TEMP_OUT_L);

		gyro.x[i] = combine(fifo_sample.GYRO_XOUT_H, fifo_sample.GYRO_XOUT_L);
		gyro.y[i] = -combine(fifo_sample.GYRO_YOUT_H, fifo_sample.GYRO_YOUT_L);
		gyro.z[i] = -combine(fifo_sample.GYRO_ZOUT_H, fifo_sample.GYRO_ZOUT_L);
	}

	accel.samples = accel_samples;

	// Temperature
	int32_t temperature_sum{0};

	for (auto t : temperature) {
		temperature_sum += t;
	}

	const int16_t temperature_avg = temperature_sum / samples;

	for (auto t : temperature) {
		// temperature changing wildly is likely a sign of a transfer error
		if (abs(t - temperature_avg) > 1000) {
			return;
		}
	}

	// use average temperature reading
	const float temperature_C = temperature_avg / 326.8f + 25.0f;	// 326.8 LSB/C
	_px4_accel.set_temperature(temperature_C);
	_px4_gyro.set_temperature(temperature_C);


	_px4_gyro.updateFIFO(gyro);
	_px4_accel.updateFIFO(accel);
}

void
ICM20602::PrintInfo()
{
	perf_print_counter(_interval_perf);
	perf_print_counter(_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_drdy_count_perf);
	perf_print_counter(_drdy_interval_perf);

	_px4_accel.print_status();
	_px4_gyro.print_status();
}
