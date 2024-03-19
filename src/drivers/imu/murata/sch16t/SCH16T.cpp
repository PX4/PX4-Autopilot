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

#include "SCH16T.hpp"

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

SCH16T::SCH16T(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_drdy_gpio(config.drdy_gpio),
	_px4_accel(get_device_id(), config.rotation),
	_px4_gyro(get_device_id(), config.rotation)
{
	if (_drdy_gpio != 0) {
		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME": DRDY missed");
	}

#ifdef GPIO_SPI6_RESET_SCH16T
	_hardware_reset_available = true;
	_reset_pin = GPIO_SPI6_RESET_SCH16T;
	px4_arch_configgpio(_reset_pin);
#endif
}

SCH16T::~SCH16T()
{
	perf_free(_reset_perf);
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_perf_crc_bad);
	perf_free(_drdy_missed_perf);
}

int SCH16T::init()
{
	PX4_INFO("init");

	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	// TODO:

	// During the startup sequence, the sensor performs a series of internal tests that will set various error
	// flags in the sensor status registers. To clear them it is necessary to read the status registers after the
	// start-up sequence is complete. When reading the status registers, the user must consider that the state
	// of status flags is not defined during LPM (Low Power Mode) and the 250 ms wait state after
	// EN_SENSOR

	return Reset() ? 0 : -1;
}

bool SCH16T::Reset()
{
	PX4_INFO("Reset()");

	_state = STATE::RESET_STAGE1;

	if (_drdy_gpio) {
		DataReadyInterruptDisable();
	}

	ScheduleClear();
	ScheduleNow();
	return true;
}

void SCH16T::exit_and_cleanup()
{
	if (_drdy_gpio) {
		DataReadyInterruptDisable();
	}

	I2CSPIDriverBase::exit_and_cleanup();
}

void SCH16T::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_reset_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_perf_crc_bad);
	perf_print_counter(_drdy_missed_perf);
}

int SCH16T::probe()
{
	PX4_INFO("probe");

	// Power-On Start-Up Time 310 ms
	if (hrt_absolute_time() < 250_ms) {
		PX4_WARN("required Power-On Start-Up Time 250 ms");
	}

	// SCH16 has COMP_ID-register that can be used to identify between versions and e.g. the measurement ranges.
	uint16_t comp_id = RegisterRead(COMP_ID);
	uint16_t asic_id = RegisterRead(ASIC_ID);

	RegisterRead(SN_ID1);
	uint16_t sn_id1 = RegisterRead(SN_ID2);
	uint16_t sn_id2 = RegisterRead(SN_ID3);
	uint16_t sn_id3 = RegisterRead(SN_ID3);

	char serial_num[14];
	snprintf(serial_num, 14, "%05d%01X%04X", sn_id2, sn_id1 & 0x000F, sn_id3);

	PX4_INFO("ASIC_ID: %u\tCOMP_ID: %u", asic_id, comp_id);
	PX4_INFO("Serial number: %s\r\n\r\n", serial_num);

	// TODO: check if asic/comp IDs match the expected version?

	return PX4_OK;
}

void SCH16T::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET_STAGE1:
		perf_count(_reset_perf);

		if (_hardware_reset_available) {
			px4_arch_gpiowrite(_reset_pin, 0);
			_state = STATE::RESET_STAGE2;
			ScheduleDelayed(2_ms);

		} else {
			SoftwareReset();
			_state = STATE::CONFIGURE;
			// wait 250_ms for POWER_ON
			ScheduleDelayed(250_ms);
		}

		_reset_timestamp = now;
		_failure_count = 0;
		break;

	case STATE::RESET_STAGE2:
		if (_hardware_reset_available) {
			px4_arch_gpiowrite(_reset_pin, 1);
		}

		_state = STATE::RESET_STAGE2;
		ScheduleDelayed(250_ms);
		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then start reading
			_state = STATE::READ;

			if (_drdy_gpio) {
				DataReadyInterruptConfigure();
				// backup schedule as a watchdog timeout
				ScheduleDelayed(100_ms);

			} else {
				ScheduleOnInterval(SAMPLE_INTERVAL_US, SAMPLE_INTERVAL_US);
			}

		} else {
			// CONFIGURE not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1_s) {
				PX4_DEBUG("Configure failed, resetting");
				_state = STATE::RESET_STAGE1;

			} else {
				PX4_DEBUG("Configure failed, retrying");
			}

			ScheduleDelayed(100_ms);
		}

		break;

	case STATE::READ: {
			hrt_abstime timestamp_sample = now;

			if (_drdy_gpio) {
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

			// TODO: read data impl and handle failures
			(void)timestamp_sample;
		}

		break;
	}
}

bool SCH16T::Configure()
{
	// Certain systems need to utilize every available sample and for example acquire samples from all axis at
	// the same time instant. As the natural output data rate with nominal primary frequency is 11.8 kHz, this
	// can create excessive load for the MCU. The purpose of decimation is to decrease the internal update
	// rate to give the host system enough time to read every sample.
	// During start-up, the user can select a suitable decimation from

	// now check that all are configured
	bool success = true;

	// TODO: check that registers are configured properly


	// SCH1600 settings and initialization
	//------------------------------------

	// // SCH1600 filter settings
	// Filter.Rate12 = FILTER_RATE;
	// Filter.Acc12  = FILTER_ACC12;
	// Filter.Acc3   = FILTER_ACC3;

	// // SCH1600 sensitivity settings
	// Sensitivity.Rate1 = SENSITIVITY_RATE1;
	// Sensitivity.Rate2 = SENSITIVITY_RATE2;
	// Sensitivity.Acc1  = SENSITIVITY_ACC1;
	// Sensitivity.Acc2  = SENSITIVITY_ACC2;
	// Sensitivity.Acc3  = SENSITIVITY_ACC3;

	// // SCH1600 decimation settings (for Rate2 and Acc2 channels).
	// Decimation.Rate2 = DECIMATION_RATE; // DEC4, F_PRIM/16
	// Decimation.Acc2  = DECIMATION_ACC;


	// TODO: set gyro range/scale
	// TODO: set accel range/scale


	return success;
}

int SCH16T::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<SCH16T *>(arg)->DataReady();
	return 0;
}

void SCH16T::DataReady()
{
	_drdy_timestamp_sample.store(hrt_absolute_time());
	ScheduleNow();
}

bool SCH16T::DataReadyInterruptConfigure()
{
	// Setup data ready on falling edge
	return px4_arch_gpiosetevent(_drdy_gpio, false, true, false, &DataReadyInterruptCallback, this) == 0;
}

bool SCH16T::DataReadyInterruptDisable()
{
	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

bool SCH16T::ValidateSensorStatus()
{
	RegisterRead(STAT_SUM);
	uint16_t summary 		= RegisterRead(STAT_SUM_SAT);
	uint16_t saturation 	= RegisterRead(STAT_COM);
	uint16_t common 		= RegisterRead(STAT_RATE_COM);
	uint16_t rate_common 	= RegisterRead(STAT_RATE_X);
	uint16_t rate_x 		= RegisterRead(STAT_RATE_Y);
	uint16_t rate_y 		= RegisterRead(STAT_RATE_Z);
	uint16_t rate_z 		= RegisterRead(STAT_ACC_X);
	uint16_t acc_x 			= RegisterRead(STAT_ACC_Y);
	uint16_t acc_y 			= RegisterRead(STAT_ACC_Z);
	uint16_t acc_z 			= RegisterRead(STAT_ACC_Z);

	uint16_t values[] = { summary, saturation, common, rate_common, rate_x, rate_y, rate_z, acc_x, acc_y, acc_z };

	for (auto v : values) {
		if (v != 0xFFFF) {
			return false;
		}
	}

	return true;
}

void SCH16T::SoftwareReset()
{
	// Writing 4b1010 to this field generates a SPI soft reset. SPI Communication is not allowed during 2ms after SPI SOFTRESET.
	RegisterWrite(CTRL_RESET, 0b1010);
}

int32_t SCH16T::DataRegisterRead(uint8_t addr)
{
	uint64_t data = {};
	data |= uint64_t(addr) << 38; // Target address offset
	data |= uint64_t(1) << 35; // FrameType: SPI48BF

	// Data registers are 2s complement
	return SPI48_DATA_INT32(TransferSpiFrame(data));
}

// The SPI protocol (SafeSPI) is 48bit out-of-frame. This means read return frames will be received on the next transfer.
uint16_t SCH16T::RegisterRead(uint8_t addr)
{
	uint64_t data = {};
	data |= uint64_t(addr) << 38; // Target address offset
	data |= uint64_t(1) << 35; // FrameType: SPI48BF

	return SPI48_DATA_UINT16(TransferSpiFrame(data));
}

// Non-data registers are the only writable ones and are 16 bit or less
void SCH16T::RegisterWrite(uint8_t addr, uint16_t value)
{
	uint64_t data = {};
	data |= uint64_t(addr) << 38; // Target address offset
	data |= uint64_t(1) << 37; // Write bit
	data |= uint64_t(1) << 35; // FrameType: SPI48BF
	data |= uint64_t(value) << 8;

	// We don't care about the return data on a write
	(void)TransferSpiFrame(data);
}

uint64_t SCH16T::TransferSpiFrame(uint64_t data)
{
	set_frequency(SPI_SPEED);

	uint16_t tx[3];
	uint16_t rx[3];

	// Swap 16-bit word order of data to send. By default the SPI_Transfer() sends
	// the lower 16 bit word first so the word order is swapped here to comply with
	// MSB first requirement.
	tx[0] = (data >> 32) & 0xffffUL;
	tx[1] = (data >> 16) & 0xffffUL;
	tx[2] = data & 0xffffUL;

	transferhword(tx, rx, 3);
	px4_udelay(SPI_STALL_PERIOD);

	uint64_t value = ((((uint64_t)rx[0]) << 32) & 0x0000ffff00000000) |
			 ((((uint64_t)rx[1]) << 16) & 0x00000000ffff0000) |
			 (((uint64_t)rx[2]) & 0x000000000000ffff);

	return value;
}
