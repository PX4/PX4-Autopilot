/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "ADIS1650x.hpp"

#include <endian.h>
#include <geo/geo.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/time.h>

using namespace time_literals;

int ADIS1650x::selected_dec_rate = 0;
int ADIS1650x::selected_filt_size = 0;

ADIS1650x::ADIS1650x(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_px4_accel(get_device_id(), config.rotation),
	_px4_gyro(get_device_id(), config.rotation),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_bad_transfers(perf_alloc(PC_COUNT, MODULE_NAME": bad transfers")),
	_reset_perf(perf_alloc(PC_COUNT, MODULE_NAME": reset")),
	_drdy_missed_perf(config.drdy_gpio ? perf_alloc(PC_COUNT, MODULE_NAME": DRDY missed") : nullptr),
	_drdy_gpio(config.drdy_gpio),
	_dec_rate(selected_dec_rate),
	_filt_size(selected_filt_size)
{
}

ADIS1650x::~ADIS1650x()
{
	perf_free(_sample_perf);
	perf_free(_bad_transfers);
	perf_free(_reset_perf);
	perf_free(_drdy_missed_perf);
}

// --- SPI register access ---

uint16_t ADIS1650x::read_reg16(Register reg)
{
	uint8_t addr = static_cast<uint8_t>(reg);
	uint8_t cmd[2]   = {addr, 0x00};
	uint8_t reply[2] = {};

	SPI::transfer(cmd, nullptr, sizeof(cmd));
	px4_udelay(SPI_STALL_US);

	SPI::transfer(nullptr, reply, sizeof(reply));
	px4_udelay(SPI_STALL_US);

	// Device sends MSB first
	return static_cast<uint16_t>(reply[0] << 8 | reply[1]);
}

bool ADIS1650x::write_reg16(Register reg, uint16_t val)
{
	uint8_t addr = static_cast<uint8_t>(reg);
	uint8_t cmd[2];

	// Write low byte first
	cmd[0] = addr | 0x80;
	cmd[1] = val & 0xFF;
	SPI::transfer(cmd, nullptr, sizeof(cmd));
	px4_udelay(SPI_STALL_US);

	// Write high byte
	cmd[0] = (addr + 1) | 0x80;
	cmd[1] = (val >> 8) & 0xFF;
	SPI::transfer(cmd, nullptr, sizeof(cmd));
	px4_udelay(SPI_STALL_US);

	return true;
}

bool ADIS1650x::write_reg16_verified(Register reg, uint16_t val, int retries)
{
	for (int i = 0; i < retries; i++) {
		write_reg16(reg, val);

		if (read_reg16(reg) == val) {
			return true;
		}
	}

	PX4_WARN("write_reg16_verified failed: reg=0x%02X val=0x%04X", static_cast<uint8_t>(reg), val);
	return false;
}

// --- Burst read ---

bool ADIS1650x::read_burst32(adis1650x_burst &burst)
{
	memset(&burst, 0, sizeof(burst));
	burst.cmd[0] = BURST_CMD_MSB;
	burst.cmd[1] = BURST_CMD_LSB;

	if (SPI::transfer((uint8_t *)&burst, (uint8_t *)&burst, sizeof(burst)) != PX4_OK) {
		return false;
	}

	return validate_checksum(burst);
}

bool ADIS1650x::validate_checksum(const adis1650x_burst &burst)
{
	// Byte-wise sum from diag_stat through data_cntr (inclusive), excludes spi_chksum
	const uint8_t *start = reinterpret_cast<const uint8_t *>(&burst.diag_stat);
	const uint8_t *end   = reinterpret_cast<const uint8_t *>(&burst.spi_chksum);
	uint16_t sum = 0;

	for (const uint8_t *p = start; p < end; p++) {
		sum += *p;
	}

	// Also reject all-zero frames: a burst of all zeros sums to zero and passes
	// the checksum (0 == 0), but is clearly invalid data.
	return sum != 0 && sum == be16toh(burst.spi_chksum);
}

// --- Probe ---

bool ADIS1650x::self_test()
{
	write_reg16(Register::GLOB_CMD, GLOB_CMD_SENSOR_SELF_TEST);
	px4_usleep(SELF_TEST_TIME_MS * 1000);

	const uint16_t diag_stat = read_reg16(Register::DIAG_STAT);

	if (diag_stat & DIAG_STAT_SELF_TEST_ERR) {
		PX4_ERR("self test failed, DIAG_STAT=0x%04X", diag_stat);
		return false;
	}

	return true;
}

int ADIS1650x::probe()
{
	_prod_id = read_reg16(Register::PROD_ID);

	switch (_prod_id) {
	case ADIS16500_PROD_ID:
	case ADIS16501_PROD_ID:
	case ADIS16505_PROD_ID:
	case ADIS16507_PROD_ID:
		break;

	default:
		PX4_ERR("Unknown PROD_ID: 0x%04X", _prod_id);
		return -EIO;
	}

	if (!self_test()) {
		return -EIO;
	}

	return PX4_OK;
}

// --- Configure ---

bool ADIS1650x::configure()
{
	switch (_prod_id) {
	case ADIS16500_PROD_ID: 	// Intentional fallthrough
	case ADIS16505_PROD_ID:
		// ±8g range
		_accel_scale = CONSTANTS_ONE_G / 262144000.0f;
		_px4_accel.set_range(8.0f * CONSTANTS_ONE_G);
		break;

	case ADIS16501_PROD_ID:
		// ±14g range
		_accel_scale = CONSTANTS_ONE_G / 149796571.0f;
		_px4_accel.set_range(14.0f * CONSTANTS_ONE_G);
		break;

	case ADIS16507_PROD_ID:
		// ±40g range
		_accel_scale = CONSTANTS_ONE_G / 52428800.0f;
		_px4_accel.set_range(40.0f * CONSTANTS_ONE_G);
		break;

	default:
		return false;
	}

	const uint16_t rng_mdl    = read_reg16(Register::RNG_MDL);
	const uint16_t gyro_range = rng_mdl & RNG_MDL_GYRO_MASK;

	switch (gyro_range) {
	case RNG_MDL_125DPS:
		_gyro_scale = math::radians(0.00625f) / 65536.0f;
		_px4_gyro.set_range(math::radians(125.0f));
		PX4_DEBUG("Gyro range: +-125 deg/s");
		break;

	case RNG_MDL_500DPS:
		_gyro_scale = math::radians(0.025f) / 65536.0f;
		_px4_gyro.set_range(math::radians(500.0f));
		PX4_DEBUG("Gyro range: +-500 deg/s");
		break;

	case RNG_MDL_2000DPS:
		_gyro_scale = math::radians(0.1f) / 65536.0f;
		_px4_gyro.set_range(math::radians(2000.0f));
		PX4_DEBUG("Gyro range: +-2000 deg/s");
		break;

	default:
		PX4_ERR("Unknown gyro range in RNG_MDL: 0x%04X", rng_mdl);
		return false;
	}

	_px4_accel.set_scale(_accel_scale);
	_px4_gyro.set_scale(_gyro_scale);

	const uint16_t msc_ctrl_val = MSC_CTRL_DR_POL | MSC_CTRL_GCOMP
				      | MSC_CTRL_PCOMP | MSC_CTRL_BURST_32;

	if (!write_reg16_verified(Register::MSC_CTRL, msc_ctrl_val)) {
		return false;
	}

	px4_udelay(MSC_CTRL_UPDATE_US);

	if (!write_reg16_verified(Register::DEC_RATE, static_cast<uint16_t>(_dec_rate))) {
		return false;
	}

	px4_udelay(DEC_RATE_UPDATE_US);

	_sample_rate_hz = static_cast<float>(INT_CLK_HZ) / static_cast<float>(_dec_rate + 1);
	PX4_DEBUG("PROD_ID=0x%04X, sample_rate=%.0f Hz, dec_rate=%d",
		  _prod_id, (double)_sample_rate_hz, _dec_rate);

	// Optional hardware Bartlett FIR filter
	float filt_delay_us = 0.0f;

	if (_filt_size > 0) {
		if (!write_reg16_verified(Register::FILT_CTRL, static_cast<uint16_t>(_filt_size))) {
			return false;
		}

		px4_udelay(FILT_UPDATE_US);

		const int flush_cycles = (1 << _filt_size) + 4;
		adis1650x_burst dummy{};

		for (int i = 0; i < flush_cycles; i++) {
			read_burst32(dummy);
			px4_usleep(FILT_FLUSH_DELAY_US);
		}

		filt_delay_us = static_cast<float>(_filt_size) * SAMPLE_PERIOD_US;
		PX4_DEBUG("Hardware filter: tap size=%d (flushed %d cycles)", _filt_size, flush_cycles);
	}

	// Group delay compensation (datasheet Table 8)
	float dec_delay_us = static_cast<float>(_dec_rate + 1) / 2.0f * SAMPLE_PERIOD_US;
	_group_delay_accel_us = GROUP_DELAY_ACCEL_BASE_US + filt_delay_us + dec_delay_us;
	_group_delay_gyro_us = GROUP_DELAY_GYRO_BASE_US + filt_delay_us + dec_delay_us;

	PX4_DEBUG("Group delay: accel=%.0f us, gyro=%.0f us",
		  (double)_group_delay_accel_us, (double)_group_delay_gyro_us);

	return true;
}

// --- DRDY interrupt handling ---

int ADIS1650x::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<ADIS1650x *>(arg)->DataReady();
	return 0;
}

void ADIS1650x::DataReady()
{
	_drdy_timestamp_sample.store(hrt_absolute_time());
	ScheduleNow();
}

bool ADIS1650x::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) { return false; }

	return px4_arch_gpiosetevent(_drdy_gpio, true, false, true,
				     &DataReadyInterruptCallback, this) == 0;
}

bool ADIS1650x::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) { return false; }

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false,
				     nullptr, nullptr) == 0;
}

// --- Start scheduling ---

void ADIS1650x::start()
{
	_last_data_cntr = 0;
	_drdy_timestamp_sample.store(0);

	if (DataReadyInterruptConfigure()) {
		_data_ready_interrupt_enabled = true;
		PX4_DEBUG("Using DRDY interrupt, watchdog at 10x period");
		ScheduleDelayed(10 * (1_s / static_cast<uint32_t>(_sample_rate_hz)));

	} else {
		_data_ready_interrupt_enabled = false;
		PX4_DEBUG("Polling at %.0f Hz", (double)_sample_rate_hz);
		ScheduleOnInterval(1_s / static_cast<uint32_t>(_sample_rate_hz));
	}
}

// --- Reset ---

void ADIS1650x::Reset()
{
	DataReadyInterruptDisable();
	ScheduleClear();
	perf_count(_reset_perf);
	_failure_count = 0;
	_configure_retries = 0;
	_state = STATE::RESET;
	ScheduleNow();
}

// --- Init ---

int ADIS1650x::init()
{
#ifdef GPIO_ADIS1650X_RESET
	// Hardware reset before SPI::init() so probe() sees a ready device on restarts.
	// Without this, probe() reads PROD_ID while the device is still in an unknown
	// state from a previous driver run, returning garbage values.
	px4_arch_configgpio(GPIO_ADIS1650X_RESET);
	px4_arch_gpiowrite(GPIO_ADIS1650X_RESET, 0);
	px4_udelay(RST_PULSE_US);
	px4_arch_gpiowrite(GPIO_ADIS1650X_RESET, 1);
	px4_usleep(SW_RESET_MS * 1000);
#endif

	int ret = SPI::init();

	if (ret != PX4_OK) {
		PX4_ERR("SPI::init failed (%i)", ret);
		return ret;
	}

	_state = STATE::RESET;
	ScheduleNow();
	return PX4_OK;
}

// --- Cleanup ---

void ADIS1650x::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

// --- State machine ---

void ADIS1650x::RunImpl()
{
	switch (_state) {
	case STATE::RESET:
		PX4_DEBUG("STATE::RESET");
#ifdef GPIO_ADIS1650X_RESET
		// Hardware reset: assert RST low, then release
		px4_arch_gpiowrite(GPIO_ADIS1650X_RESET, 0);
		px4_udelay(RST_PULSE_US);
		px4_arch_gpiowrite(GPIO_ADIS1650X_RESET, 1);
#else
		write_reg16(Register::GLOB_CMD, GLOB_CMD_SW_RESET);
#endif
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(SW_RESET_MS * 1000);
		break;

	case STATE::WAIT_FOR_RESET:
		PX4_DEBUG("STATE::WAIT_FOR_RESET");
		_prod_id = read_reg16(Register::PROD_ID);

		if (_prod_id == ADIS16500_PROD_ID || _prod_id == ADIS16501_PROD_ID ||
		    _prod_id == ADIS16505_PROD_ID || _prod_id == ADIS16507_PROD_ID) {
			PX4_INFO("Detected PROD_ID=0x%04X", _prod_id);

			const uint16_t diag_stat = read_reg16(Register::DIAG_STAT);

			if (diag_stat != 0) {
				PX4_WARN("DIAG_STAT=0x%04X after reset, retrying", diag_stat);
				_state = STATE::RESET;
				ScheduleDelayed(100000);
				break;
			}

			_state = STATE::CONFIGURE;
			ScheduleNow();

		} else {
			PX4_ERR("Invalid PROD_ID: 0x%04X, retrying reset", _prod_id);
			_state = STATE::RESET;
			ScheduleDelayed(100000);
		}

		break;

	case STATE::CONFIGURE:
		PX4_DEBUG("STATE::CONFIGURE");

		if (configure()) {
			_configure_retries = 0;
			_state = STATE::READ;
			start();

		} else {
			if (++_configure_retries >= 3) {
				PX4_ERR("Configure failed %d times, giving up", _configure_retries);
				return;
			}

			PX4_WARN("Configure failed, resetting (attempt %d/3)", _configure_retries);
			_state = STATE::RESET;
			ScheduleDelayed(100000);
		}

		break;

	case STATE::READ: {
			const hrt_abstime now = hrt_absolute_time();
			hrt_abstime timestamp_sample = now;

			if (_data_ready_interrupt_enabled) {
				const hrt_abstime drdy_ts = _drdy_timestamp_sample.fetch_and(0);

				if (drdy_ts != 0 && (now - drdy_ts) < (1_s / static_cast<uint32_t>(_sample_rate_hz))) {
					timestamp_sample = drdy_ts;

				} else {
					perf_count(_drdy_missed_perf);
				}

				// Push watchdog back
				ScheduleDelayed(2 * (1_s / static_cast<uint32_t>(_sample_rate_hz)));
			}

			if (measure(timestamp_sample) != PX4_OK) {
				_failure_count++;

				if (_failure_count > 10) {
					PX4_ERR("Too many consecutive failures, resetting");
					Reset();
				}

			} else {
				_failure_count = 0;
			}

			break;
		}
	}
}

// --- Measure ---

int ADIS1650x::measure(hrt_abstime timestamp_sample)
{
	perf_begin(_sample_perf);

	adis1650x_burst burst{};

	if (!read_burst32(burst)) {
		perf_count(_bad_transfers);
		perf_end(_sample_perf);
		return -EIO;
	}

	const uint16_t diag_stat = be16toh(burst.diag_stat);

	if (diag_stat != 0) {
		perf_count(_bad_transfers);
		print_error_flags(diag_stat);
		perf_end(_sample_perf);
		return -EIO;
	}

	// Discard stale samples — data_cntr increments with each new sensor output
	const uint16_t data_cntr = be16toh(burst.data_cntr);

	if (data_cntr == _last_data_cntr) {
		perf_end(_sample_perf);
		return PX4_OK;
	}

	_last_data_cntr = data_cntr;

	auto assemble32 = [](uint16_t lsb_field, uint16_t msb_field) -> int32_t {
		return static_cast<int32_t>(
			static_cast<uint32_t>(be16toh(msb_field)) << 16 |
			static_cast<uint16_t>(be16toh(lsb_field)));
	};

	const int32_t gx_raw = assemble32(burst.x_gyro_lsb,  burst.x_gyro_msb);
	const int32_t gy_raw = assemble32(burst.y_gyro_lsb,  burst.y_gyro_msb);
	const int32_t gz_raw = assemble32(burst.z_gyro_lsb,  burst.z_gyro_msb);
	const int32_t ax_raw = assemble32(burst.x_accel_lsb, burst.x_accel_msb);
	const int32_t ay_raw = assemble32(burst.y_accel_lsb, burst.y_accel_msb);
	const int32_t az_raw = assemble32(burst.z_accel_lsb, burst.z_accel_msb);

	const int16_t temp_raw    = static_cast<int16_t>(be16toh(burst.temp));
	const float   temperature = static_cast<float>(temp_raw) * TEMP_SCALE;

	const uint64_t error_count = perf_event_count(_bad_transfers);
	_px4_accel.set_error_count(error_count);
	_px4_gyro.set_error_count(error_count);

	_px4_accel.set_temperature(temperature);
	_px4_gyro.set_temperature(temperature);

	const hrt_abstime ts_gyro = timestamp_sample - static_cast<hrt_abstime>(_group_delay_gyro_us);
	const hrt_abstime ts_accel = timestamp_sample - static_cast<hrt_abstime>(_group_delay_accel_us);

	_px4_gyro.update(ts_gyro,
			 static_cast<float>(gx_raw),
			 static_cast<float>(gy_raw),
			 static_cast<float>(gz_raw));
	_px4_accel.update(ts_accel,
			  static_cast<float>(ax_raw),
			  static_cast<float>(ay_raw),
			  static_cast<float>(az_raw));

	perf_end(_sample_perf);
	return PX4_OK;
}

// --- Error flags ---

void ADIS1650x::print_error_flags(uint16_t diag_stat)
{
	if (diag_stat & DIAG_STAT_Z_ACCEL_FAILURE)      { PX4_ERR("Z-axis accelerometer failure"); }

	if (diag_stat & DIAG_STAT_Y_ACCEL_FAILURE)      { PX4_ERR("Y-axis accelerometer failure"); }

	if (diag_stat & DIAG_STAT_X_ACCEL_FAILURE)      { PX4_ERR("X-axis accelerometer failure"); }

	if (diag_stat & DIAG_STAT_Z_GYRO_FAILURE)       { PX4_ERR("Z-axis gyroscope failure"); }

	if (diag_stat & DIAG_STAT_Y_GYRO_FAILURE)       { PX4_ERR("Y-axis gyroscope failure"); }

	if (diag_stat & DIAG_STAT_X_GYRO_FAILURE)       { PX4_ERR("X-axis gyroscope failure"); }

	if (diag_stat & DIAG_STAT_SELF_TEST_ERR)         { PX4_ERR("Self-test diagnostics failure"); }

	if (diag_stat & DIAG_STAT_POWER_SUPPLY)          { PX4_ERR("Power supply failure"); }

	if (diag_stat & DIAG_STAT_SPI_COMM_ERROR)        { PX4_ERR("SPI communication error"); }

	if (diag_stat & DIAG_STAT_FLASH_UPDATE_FAILURE)  { PX4_ERR("Flash memory update failure"); }

	if (diag_stat & DIAG_STAT_DATAPATH_OVERRUN)      { PX4_ERR("Datapath processing overrun"); }

	if (diag_stat & DIAG_STAT_SENSOR_INIT_FAILURE)   { PX4_ERR("Sensor initialization failure"); }
}

// --- Status ---

void ADIS1650x::print_status()
{
	I2CSPIDriverBase::print_status();
	PX4_INFO("PROD_ID:     0x%04X", _prod_id);
	PX4_INFO("Sample rate: %.0f Hz (dec_rate=%d)", (double)_sample_rate_hz, _dec_rate);
	PX4_INFO("Filter:      %s (size=%d)", _filt_size > 0 ? "enabled" : "bypassed", _filt_size);
	PX4_INFO("DRDY: %s", _data_ready_interrupt_enabled ? "interrupt" : "polling");
	perf_print_counter(_sample_perf);
	perf_print_counter(_bad_transfers);
	perf_print_counter(_reset_perf);
	perf_print_counter(_drdy_missed_perf);
}
