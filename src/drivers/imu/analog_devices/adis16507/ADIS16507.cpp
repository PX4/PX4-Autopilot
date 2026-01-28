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

#include "ADIS16507.hpp"

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

ADIS16507::ADIS16507(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_drdy_gpio(config.drdy_gpio),
	_px4_accel(get_device_id(), config.rotation),
	_px4_gyro(get_device_id(), config.rotation)
{
	if (_drdy_gpio != 0) {
		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME": DRDY missed");
	}
}

ADIS16507::~ADIS16507()
{
	perf_free(_reset_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_bad_status_perf);
	perf_free(_bad_checksum_perf);
	perf_free(_bad_data_cntr_perf);
	perf_free(_drdy_missed_perf);
}

int ADIS16507::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		PX4_ERR("SPI::init failed (%i)", ret);
		return ret;
	}

	_state = STATE::RESET;
	ScheduleNow();
	return PX4_OK;
}

void ADIS16507::Reset()
{
	_state = STATE::RESET;
	DataReadyInterruptDisable();
	ScheduleClear();
}

void ADIS16507::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

int ADIS16507::probe()
{
	// Power-On Start-Up Time 310 ms
	if (hrt_absolute_time() < 310_ms) {
		PX4_WARN("Required Power-On Start-Up Time 310 ms");
	}

	const uint16_t id = RegisterRead(Register::PROD_ID);

	if (id != Register::PROD_ID_EXPECTED) {
		PX4_ERR("Unexpected PROD_ID 0x%02x", id);
		return PX4_ERROR;
	}

	const uint16_t serial = RegisterRead(Register::SERIAL_NUM);
	const uint16_t rev = RegisterRead(Register::FIRM_REV);
	const uint16_t daymonth = RegisterRead(Register::FIRM_DM);
	const uint16_t year = RegisterRead(Register::FIRM_Y);

	PX4_INFO("Serial Number: 0x%X, Firmware revision: 0x%X Date: Y %X DM %X", serial, rev, year, daymonth);

	return PX4_OK;
}

bool ADIS16507::Configure()
{
	struct {
		uint16_t reg;
		uint16_t val;
	} defaults[] = {
		// Default 0x00C1
		// - Change Data Ready polarity to active low
		{ Register::MSC_CTRL, 0x00C0 },
	};

	// Write default register configuration
	for (const auto &r : defaults) {
		RegisterWrite(r.reg, r.val);
	}

	// Wait for changes to apply
	px4_usleep(SPI_STALL_PERIOD);

	// Check that all are configured
	for (const auto &r : defaults) {
		if (!RegisterCheck(r.reg, r.val)) {
			return false;
		}
	}

	// Accelerometer only has a single measurement range and scale
	_px4_accel.set_range(392);
	_px4_accel.set_scale(392.f / 32'000.f); // 32,000 -> 392 m/sec^2

	// Check gyroscope measurement range
	const uint16_t rang_mdl = RegisterRead(Register::RANG_MDL);

	// sanity check RANG_MDL [1:0] Reserved, binary value = 11
	if (rang_mdl & (Bit1 | Bit0)) {
		const uint16_t gyro_range = (rang_mdl & (Bit3 | Bit2)) >> 2;

		if (gyro_range == 0b11) {
			PX4_DEBUG("Gyro Range ±2000°/sec");
			// 11 = ±2000°/sec (ADIS16507-3BMLZ)
			_px4_gyro.set_range(math::radians(2000.f));
			_px4_gyro.set_scale(math::radians(1.f / 10.f)); // scaling 10 LSB/°/sec -> rad/s per LSB

		} else if (gyro_range == 0b01) {
			PX4_DEBUG("Gyro Range ±500°/sec");
			// 01 = ±500°/sec (ADIS16507-2BMLZ)
			_px4_gyro.set_range(math::radians(500.f));
			_px4_gyro.set_scale(math::radians(1.f / 40.f)); // scaling 40 LSB/°/sec -> rad/s per LSB

		} else if (gyro_range == 0b00) {
			PX4_DEBUG("Gyro Range ±125°/sec");
			// 00 = ±125°/sec (ADIS16507-1BMLZ)
			_px4_gyro.set_range(math::radians(125.f));
			_px4_gyro.set_scale(math::radians(1.f / 160.f)); // scaling 160 LSB/°/sec -> rad/s per LSB
		}
	}

	return true;
}

void ADIS16507::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		PX4_DEBUG("Resetting");
		perf_count(_reset_perf);

		RegisterWrite(Register::GLOB_CMD, Register::GLOB_CMD_BIT::Software_reset);

#ifdef GPIO_ADIS16507_RESET
		PX4_DEBUG("Hardware reset");
		GPIO_ADIS16507_RESET(1);
		px4_usleep(15); // Minimum 10us
		GPIO_ADIS16507_RESET(0);
#endif
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(350_ms); // 255 ms Reset Recovery Time
		break;

	case STATE::WAIT_FOR_RESET:

		if (_self_test_passed) {
			if (RegisterRead(Register::PROD_ID) == Register::PROD_ID_EXPECTED) {
				_state = STATE::CONFIGURE;
				PX4_DEBUG("Reset complete, configuring");
				ScheduleNow();

			} else {
				PX4_DEBUG("Reset failed, retrying");
				_state = STATE::RESET;
				ScheduleDelayed(100_ms);
			}

		} else {
			PX4_DEBUG("Running self test");
			RegisterWrite(Register::GLOB_CMD, Register::GLOB_CMD_BIT::Sensor_self_test);
			_state = STATE::SELF_TEST_CHECK;
			ScheduleDelayed(50_ms); // Self Test Time 24ms typical
		}

		break;

	case STATE::SELF_TEST_CHECK: {
			const uint16_t diag_stat = RegisterRead(Register::DIAG_STAT);

			if (diag_stat != 0) {
				PX4_ERR("Self test failed");
				PrintErrorFlags(diag_stat);
				ScheduleDelayed(350_ms);

			} else {
				PX4_DEBUG("Self test passed");
				_self_test_passed = true;
				_state = STATE::RESET;
				ScheduleNow();
			}
		}
		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			_state = STATE::READ;

			if (DataReadyInterruptConfigure()) {
				PX4_DEBUG("Using data ready interrupt");
				_data_ready_interrupt_enabled = true;

				// backup schedule as a watchdog timeout
				ScheduleDelayed(100_ms);
				// Data ready should reschedule this almost immediately
				return;

			} else {
				PX4_DEBUG("Not using data ready interrupt");
				_data_ready_interrupt_enabled = false;
				ScheduleOnInterval(SAMPLE_INTERVAL_US, SAMPLE_INTERVAL_US);
			}

		} else {
			// CONFIGURE not complete
			PX4_WARN("Configure failed, resetting");
			_state = STATE::RESET;
			ScheduleDelayed(100_ms);
		}

		break;

	case STATE::READ: {
			hrt_abstime timestamp_sample = now;

			if (_data_ready_interrupt_enabled) {
				// scheduled from interrupt if _drdy_timestamp_sample was set as expected
				const hrt_abstime drdy_timestamp_sample = _drdy_timestamp_sample.fetch_and(0);

				if ((now - drdy_timestamp_sample) < SAMPLE_INTERVAL_US) {
					timestamp_sample = drdy_timestamp_sample;

				} else {
					perf_count(_drdy_missed_perf);
				}

				// push backup schedule back
				ScheduleDelayed(SAMPLE_INTERVAL_US * 2);
			}

			// struct __attribute__((packed)) BurstRead {
			struct BurstRead {
				uint16_t cmd;
				uint16_t diag_stat;
				int16_t x_gyro_out;
				int16_t y_gyro_out;
				int16_t z_gyro_out;
				int16_t x_accl_out;
				int16_t y_accl_out;
				int16_t z_accl_out;
				int16_t temp_out;
				uint16_t data_cntr;
				uint16_t checksum;
			} buffer{};

			// ADIS16507 burst report should be 176 bits
			static_assert(sizeof(BurstRead) == (176 / 8), "ADIS16507 report not 176 bits");

			// Pg 20 of Datasheet
			// 16-Bit Burst Mode with BURST_SEL = 0
			buffer.cmd = BURST_READ_CMD;
			set_frequency(SPI_SPEED_BURST);

			if (transferhword((uint16_t *)&buffer, (uint16_t *)&buffer, sizeof(buffer) / sizeof(uint16_t)) != PX4_OK) {
				perf_count(_bad_transfer_perf);
				_failure_count++;

				if (_failure_count > 10) {
					PX4_DEBUG("Consecutive failures!");
					Reset();
				}

				// Don't publish on a bad transfer
				return;
			}

			if (buffer.data_cntr == _last_data_cntr) {
				// Don't publish if data counter is not incrementing
				perf_count(_bad_data_cntr_perf);
				_last_data_cntr = buffer.data_cntr;
				_failure_count++;

				if (_failure_count > 10) {
					PX4_DEBUG("Consecutive failures!");
					Reset();
				}

				return;
			}

			_last_data_cntr = buffer.data_cntr;

			uint16_t checksum = 0;
			uint8_t *checksum_helper = (uint8_t *)&buffer.diag_stat;

			for (int i = 0; i < 18; i++) {
				checksum += checksum_helper[i];
			}

			if (buffer.checksum != checksum) {
				perf_count(_bad_checksum_perf);
				// Don't publish if checksum fails
				return;
			}

			// Check all Status/Error Flag Indicators (DIAG_STAT)
			if (buffer.diag_stat != 0) {
				perf_count(_bad_status_perf);
				PX4_DEBUG("Error: DIAG_STAT: 0x%02x", buffer.diag_stat);
				PrintErrorFlags(buffer.diag_stat);
				return;
			}

			const float temperature = buffer.temp_out * 0.1f; // 1 LSB = 0.1°C
			_px4_accel.set_temperature(temperature);
			_px4_gyro.set_temperature(temperature);

			// sensor frame is FLU, publish as FRD
			float accel_x = buffer.x_accl_out;
			float accel_y = -1.f * buffer.y_accl_out;
			float accel_z = -1.f * buffer.z_accl_out;
			float gyro_x = buffer.x_gyro_out;
			float gyro_y = -1.f * buffer.y_gyro_out;
			float gyro_z = -1.f * buffer.z_gyro_out;

			// Group Delay with No Filtering: Accelerometer 1.57 ms
			const uint64_t accel_group_delay_us = 1'570;
			_px4_accel.update(timestamp_sample - accel_group_delay_us, accel_x, accel_y, accel_z);

			// Group Delay with No Filtering:
			//  Gyroscope (X-Axis) 1.51 ms
			//  Gyroscope (Y-Axis) 1.51 ms
			//  Gyroscope (Z-Axis) 1.29 ms
			const uint64_t gyro_group_delay_us = (1'510 + 1'510 + 1'290) / 3;
			_px4_gyro.update(timestamp_sample - gyro_group_delay_us, gyro_x, gyro_y, gyro_z);

			if (_failure_count > 0) {
				_failure_count--;
			}
		}

		break;
	}
}

void ADIS16507::PrintErrorFlags(uint16_t flags)
{
	if (flags & (1 << 10)) {
		PX4_DEBUG("Accelerometer failure");
	}

	if (flags & (1 << 9)) {
		PX4_DEBUG("Gyro 2 failure");
	}

	if (flags & (1 << 8)) {
		PX4_DEBUG("Gyro 1 failure");
	}

	if (flags & (1 << 7)) {
		PX4_DEBUG("Clock error");
	}

	if (flags & (1 << 6)) {
		PX4_DEBUG("Memory failure");
	}

	if (flags & (1 << 5)) {
		PX4_DEBUG("Sensor failure");
	}

	if (flags & (1 << 4)) {
		PX4_DEBUG("Standby mode (VDD < 2.8V)");
	}

	if (flags & (1 << 3)) {
		PX4_DEBUG("SPI communication error");
	}

	if (flags & (1 << 2)) {
		PX4_DEBUG("Flash memory update failure");
	}

	if (flags & (1 << 1)) {
		PX4_DEBUG("Data path overrun");
	}

	// Bit 0 and 15:11 are reserved and not printed
}

int ADIS16507::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<ADIS16507 *>(arg)->DataReady();
	return 0;
}

void ADIS16507::DataReady()
{
	_drdy_timestamp_sample.store(hrt_absolute_time());
	ScheduleNow();
}

bool ADIS16507::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on falling edge
	return px4_arch_gpiosetevent(_drdy_gpio, false, true, false, &DataReadyInterruptCallback, this) == 0;
}

bool ADIS16507::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

uint16_t ADIS16507::RegisterRead(uint16_t reg)
{
	set_frequency(SPI_SPEED);

	uint16_t cmd[1];
	cmd[0] = (static_cast<uint16_t>(reg) << 8);

	transferhword(cmd, nullptr, 1);
	px4_udelay(SPI_STALL_PERIOD);
	transferhword(nullptr, cmd, 1);

	return cmd[0];
}

void ADIS16507::RegisterWrite(uint16_t reg, uint16_t val)
{
	set_frequency(SPI_SPEED);

	uint16_t cmd[2];
	cmd[0] = (((static_cast<uint16_t>(reg))     | DIR_WRITE) << 8) | ((0x00FF & val));
	cmd[1] = (((static_cast<uint16_t>(reg) + 1) | DIR_WRITE) << 8) | ((0xFF00 & val) >> 8);

	transferhword(cmd, nullptr, 1);
	px4_udelay(SPI_STALL_PERIOD);
	transferhword(cmd + 1, nullptr, 1);
}

bool ADIS16507::RegisterCheck(uint16_t reg, uint16_t val)
{
	const uint16_t actual = RegisterRead(reg);

	if (actual != val) {
		PX4_WARN("register 0x%02hhX: 0x%02hhX (should be 0x%02hhX)", reg, actual, val);
		return false;
	}

	return true;
}

void ADIS16507::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_reset_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_bad_status_perf);
	perf_print_counter(_bad_checksum_perf);
	perf_print_counter(_bad_data_cntr_perf);
	perf_print_counter(_drdy_missed_perf);
}
