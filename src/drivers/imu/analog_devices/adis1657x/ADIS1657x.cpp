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

#include "ADIS1657x.hpp"

#include <endian.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/time.h>


using namespace time_literals;

static inline int32_t assemble32(uint16_t lsb_field, uint16_t msb_field)
{
	uint32_t msb = static_cast<uint32_t>(be16toh(msb_field)) << 16;
	uint16_t lsb = static_cast<uint16_t>(be16toh(lsb_field));
	return static_cast<int32_t>(msb | lsb);
}

int ADIS1657x::selected_dec_rate = 0;
int ADIS1657x::selected_filt_size = 0;

ADIS1657x::ADIS1657x(const I2CSPIDriverConfig &config) :
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

ADIS1657x::~ADIS1657x()
{
	perf_free(_sample_perf);
	perf_free(_bad_transfers);
	perf_free(_reset_perf);
	perf_free(_drdy_missed_perf);
}

// --- SPI register access ---

uint16_t ADIS1657x::read_reg16(Register reg)
{
	uint8_t addr = static_cast<uint8_t>(reg);
	uint8_t cmd[2] = {addr, 0x00};
	uint8_t reply[2] = {};

	if (SPI::transfer(cmd, nullptr, sizeof(cmd)) != PX4_OK) {
		return 0;
	}

	px4_udelay(SPI_STALL_PERIOD);

	if (SPI::transfer(nullptr, reply, sizeof(reply)) != PX4_OK) {
		return 0;
	}

	px4_udelay(SPI_STALL_PERIOD);

	return static_cast<uint16_t>(reply[0] << 8 | reply[1]);
}

bool ADIS1657x::write_reg16(Register reg, uint16_t val)
{
	uint8_t addr = static_cast<uint8_t>(reg);

	uint8_t cmd[2];
	cmd[0] = addr | 0x80;
	cmd[1] = val & 0xFF;

	if (SPI::transfer(cmd, nullptr, sizeof(cmd)) != PX4_OK) {
		return false;
	}

	px4_udelay(SPI_STALL_PERIOD);

	cmd[0] = (addr + 1) | 0x80;
	cmd[1] = (val >> 8) & 0xFF;

	if (SPI::transfer(cmd, nullptr, sizeof(cmd)) != PX4_OK) {
		return false;
	}

	px4_udelay(SPI_STALL_PERIOD);

	return true;
}

bool ADIS1657x::write_reg16_verified(Register reg, uint16_t val, int retries)
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

bool ADIS1657x::read_burst32(adis1657x_burst &burst)
{
	memset(&burst, 0, sizeof(burst));
	burst.cmd[0] = BURST_CMD_MSB;
	burst.cmd[1] = BURST_CMD_LSB;

	// Full-duplex transfer: command goes out, response comes back in same buffer
	if (SPI::transfer((uint8_t *)&burst, (uint8_t *)&burst, sizeof(burst)) != PX4_OK) {
		return false;
	}

	// No stall needed after burst read (datasheet tSTALL = N/A for burst mode)
	return validate_checksum(burst);
}

bool ADIS1657x::validate_checksum(const adis1657x_burst &burst)
{
	// Checksum is byte-wise sum of all words from x_gyro_lsb through timestamp_upr
	// (excludes diag_stat and the checksum itself)
	const uint8_t *start = reinterpret_cast<const uint8_t *>(&burst.x_gyro_lsb);
	const uint8_t *end   = reinterpret_cast<const uint8_t *>(&burst.spi_chksum);
	uint16_t sum = 0;

	for (const uint8_t *p = start; p < end; p++) {
		sum += *p;
	}

	// Also reject all-zero frames: a burst of all zeros sums to zero and passes
	// the checksum (0 == 0), but is clearly invalid data.
	return sum != 0 && sum == be16toh(burst.spi_chksum);
}

// --- Probe: detect product ---

bool ADIS1657x::self_test()
{
	// DIN sequence: 0xE804, 0xE900 — sets GLOB_CMD bit 2 (sensor self test)
	write_reg16(Register::GLOB_CMD, GLOB_CMD_SENSOR_SELF_TEST);
	px4_usleep(SELF_TEST_TIME_MS * 1000);

	const uint16_t diag_stat = read_reg16(Register::DIAG_STAT);

	if (diag_stat & DIAG_STAT_SELF_TEST_ERR) {
		PX4_ERR("self test failed, DIAG_STAT=0x%04X", diag_stat);
		return false;
	}

	return true;
}

int ADIS1657x::probe()
{
	_prod_id = read_reg16(Register::PROD_ID);

	if (_prod_id != ADIS16575_PROD_ID
	    && _prod_id != ADIS16576_PROD_ID
	    && _prod_id != ADIS16577_PROD_ID) {
		PX4_ERR("Unknown PROD_ID: 0x%04X", _prod_id);
		return -EIO;
	}

	if (!self_test()) {
		return -EIO;
	}

	return PX4_OK;
}

// --- Init ---

int ADIS1657x::init()
{
#ifdef GPIO_ADIS1657X_RESET
	// Hardware reset before SPI::init() so probe() sees a ready device on restarts.
	// Without this, probe() reads PROD_ID while the device is still in an unknown
	// state from a previous driver run, returning garbage values.
	px4_arch_configgpio(GPIO_ADIS1657X_RESET);
	px4_arch_gpiowrite(GPIO_ADIS1657X_RESET, 0);
	px4_udelay(RST_PULSE_US);
	px4_arch_gpiowrite(GPIO_ADIS1657X_RESET, 1);
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

void ADIS1657x::Reset()
{
	DataReadyInterruptDisable();
	ScheduleClear();
	perf_count(_reset_perf);
	_failure_count = 0;
	_configure_retries = 0;
	_state = STATE::RESET;
	ScheduleNow();
}

bool ADIS1657x::Configure()
{
	// Determine accel scale and range from product ID
	switch (_prod_id) {
	case ADIS16575_PROD_ID:
		_accel_scale = CONSTANTS_ONE_G / 262144000.0f;
		_px4_accel.set_range(8.0f * CONSTANTS_ONE_G);
		break;

	case ADIS16576_PROD_ID:
	case ADIS16577_PROD_ID:
		_accel_scale = CONSTANTS_ONE_G / 52428800.0f;
		_px4_accel.set_range(40.0f * CONSTANTS_ONE_G);
		break;

	default:
		PX4_ERR("Unknown product, cannot set accel scale");
		return false;
	}

	// Determine gyro scale and range from RNG_MDL register
	uint16_t rng_mdl = read_reg16(Register::RNG_MDL);
	uint8_t gyro_range = rng_mdl & RNG_MDL_GYRO_MASK;

	switch (gyro_range) {
	case RNG_MDL_450DPS:
		_gyro_scale = math::radians(1.0f / 40.0f) / 65536.0f;
		_px4_gyro.set_range(math::radians(450.0f));
		PX4_DEBUG("Gyro range: +-450 deg/s");
		break;

	case RNG_MDL_2000DPS:
		_gyro_scale = math::radians(1.0f / 10.0f) / 65536.0f;
		_px4_gyro.set_range(math::radians(2000.0f));
		PX4_DEBUG("Gyro range: +-2000 deg/s");
		break;

	default:
		PX4_ERR("Unknown gyro range in RNG_MDL: 0x%04X", rng_mdl);
		return false;
	}

	_px4_accel.set_scale(_accel_scale);
	_px4_gyro.set_scale(_gyro_scale);

	PX4_DEBUG("Accel scale: %.6e m/s^2/LSB", (double)_accel_scale);
	PX4_DEBUG("Gyro scale: %.6e rad/s/LSB", (double)_gyro_scale);

	// Enable 32-bit burst mode; OUT_SEL=0 (gyro/accel); POP_EN and GSEN_EN use factory calibration
	if (!write_reg16_verified(Register::MSC_CTRL, MSC_CTRL_DEFAULT | MSC_CTRL_BURST_32)) {
		return false;
	}

	px4_udelay(MSC_CTRL_UPDATE_US);

	PX4_DEBUG("Setting DEC_RATE=%d (sample_rate=%.0f Hz)", _dec_rate,
		  (double)(INT_CLK_HZ / (_dec_rate + 1)));

	if (!write_reg16_verified(Register::DEC_RATE, static_cast<uint16_t>(_dec_rate))) {
		return false;
	}

	px4_udelay(DEC_RATE_UPDATE_US);

	_sample_rate_hz = static_cast<float>(INT_CLK_HZ) / static_cast<float>(_dec_rate + 1);

	// Set hardware Bartlett FIR filter
	if (_filt_size > 0) {
		PX4_DEBUG("Setting FILT_CTRL=%d", _filt_size);

		if (!write_reg16_verified(Register::FILT_CTRL, static_cast<uint16_t>(_filt_size))) {
			return false;
		}

		px4_udelay(FILT_UPDATE_US);

		// Flush filter pipeline
		const int flush_cycles = (1 << _filt_size) + 4;
		adis1657x_burst dummy{};

		for (int i = 0; i < flush_cycles; i++) {
			read_burst32(dummy);
			px4_usleep(FILT_FLUSH_DELAY_US);
		}

		PX4_DEBUG("Filter pipeline flushed (%d cycles)", flush_cycles);

	} else {
		PX4_DEBUG("Hardware filter bypassed");
	}

	return true;
}

// --- DRDY interrupt handling ---

int ADIS1657x::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<ADIS1657x *>(arg)->DataReady();
	return 0;
}

void ADIS1657x::DataReady()
{
	if (++_drdy_count >= (_dec_rate + 1)) {
		_drdy_timestamp_sample.store(hrt_absolute_time());
		_drdy_count = 0;
		ScheduleNow();
	}
}

bool ADIS1657x::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) { return false; }

	return px4_arch_gpiosetevent(_drdy_gpio, true, false, true,
				     &DataReadyInterruptCallback, this) == 0;
}

bool ADIS1657x::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) { return false; }

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false,
				     nullptr, nullptr) == 0;
}

// --- Start / scheduling ---

void ADIS1657x::start()
{
	_last_data_cntr = 0;
	_drdy_count = 0;
	_drdy_timestamp_sample.store(0);

	if (DataReadyInterruptConfigure()) {
		_data_ready_interrupt_enabled = true;
		PX4_DEBUG("Using DRDY interrupt, watchdog at 10x period");
		ScheduleDelayed(10 * (1_s / (uint32_t)_sample_rate_hz));

	} else {
		_data_ready_interrupt_enabled = false;
		PX4_DEBUG("Polling at %.0f Hz", (double)_sample_rate_hz);
		ScheduleDelayed(1_s / (uint32_t)_sample_rate_hz);
	}
}

void ADIS1657x::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

// --- RunImpl (state machine) ---

void ADIS1657x::RunImpl()
{
	switch (_state) {
	case STATE::RESET:
		PX4_DEBUG("STATE::RESET");
#ifdef GPIO_ADIS1657X_RESET
		px4_arch_gpiowrite(GPIO_ADIS1657X_RESET, 0);
		px4_udelay(RST_PULSE_US);
		px4_arch_gpiowrite(GPIO_ADIS1657X_RESET, 1);
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

		if (_prod_id == ADIS16575_PROD_ID
		    || _prod_id == ADIS16576_PROD_ID
		    || _prod_id == ADIS16577_PROD_ID) {
			PX4_INFO("Detected PROD_ID=0x%04X", _prod_id);

			const uint16_t diag_stat = read_reg16(Register::DIAG_STAT);

			if (diag_stat != 0) {
				PX4_WARN("DIAG_STAT=0x%04X after reset, retrying", diag_stat);
				_state = STATE::RESET;
				ScheduleDelayed(RETRY_DELAY_US);
				break;
			}

			_state = STATE::CONFIGURE;
			ScheduleNow();

		} else {
			PX4_ERR("Invalid PROD_ID: 0x%04X, retrying reset", _prod_id);
			_state = STATE::RESET;
			ScheduleDelayed(RETRY_DELAY_US);
		}

		break;

	case STATE::CONFIGURE:
		PX4_DEBUG("STATE::CONFIGURE");

		if (Configure()) {
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
			ScheduleDelayed(RETRY_DELAY_US);
		}

		break;

	case STATE::READ: {
			const hrt_abstime now = hrt_absolute_time();
			hrt_abstime timestamp_sample = now;

			if (_data_ready_interrupt_enabled) {
				const hrt_abstime drdy_ts = _drdy_timestamp_sample.fetch_and(0);

				if (drdy_ts != 0 && (now - drdy_ts) < (1_s / (uint32_t)_sample_rate_hz)) {
					timestamp_sample = drdy_ts;

				} else {
					perf_count(_drdy_missed_perf);
				}

				// Push watchdog back
				ScheduleDelayed(2 * (1_s / (uint32_t)_sample_rate_hz));

			} else {
				ScheduleDelayed(1_s / (uint32_t)_sample_rate_hz);
			}

			if (measure(timestamp_sample) != PX4_OK) {
				_failure_count++;

				if (_failure_count > 10) {
					PX4_ERR("Too many consecutive failures, resetting");
					Reset();
					return;
				}

			} else {
				_failure_count = 0;
			}

			break;
		}
	}
}

// --- Measure (single burst read) ---

int ADIS1657x::measure(hrt_abstime timestamp_sample)
{
	perf_begin(_sample_perf);

	adis1657x_burst burst{};

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

	_px4_gyro.update(timestamp_sample,
			 static_cast<float>(gx_raw),
			 static_cast<float>(gy_raw),
			 static_cast<float>(gz_raw));
	_px4_accel.update(timestamp_sample,
			  static_cast<float>(ax_raw),
			  static_cast<float>(ay_raw),
			  static_cast<float>(az_raw));

	perf_end(_sample_perf);
	return PX4_OK;
}

// --- Error flags ---

void ADIS1657x::print_error_flags(uint16_t diag_stat)
{
	if (diag_stat & DIAG_STAT_MICROCONTROLLER_FAULT) { PX4_ERR("Microcontroller fault"); }

	if (diag_stat & DIAG_STAT_Z_ACCEL_FAILURE)       { PX4_ERR("Z-Axis accelerometer failure"); }

	if (diag_stat & DIAG_STAT_Y_ACCEL_FAILURE)       { PX4_ERR("Y-Axis accelerometer failure"); }

	if (diag_stat & DIAG_STAT_X_ACCEL_FAILURE)       { PX4_ERR("X-Axis accelerometer failure"); }

	if (diag_stat & DIAG_STAT_Z_GYRO_FAILURE)        { PX4_ERR("Z-Axis gyroscope failure"); }

	if (diag_stat & DIAG_STAT_Y_GYRO_FAILURE)        { PX4_ERR("Y-Axis gyroscope failure"); }

	if (diag_stat & DIAG_STAT_X_GYRO_FAILURE)        { PX4_ERR("X-Axis gyroscope failure"); }

	if (diag_stat & DIAG_STAT_SYNC_DPLL_UNLOCK)      { PX4_ERR("Scaled sync DPLL unlock"); }

	if (diag_stat & DIAG_STAT_MEMORY_ERROR)           { PX4_ERR("Memory error (corrupted factory data)"); }

	if (diag_stat & DIAG_STAT_SELF_TEST_ERR)          { PX4_ERR("Self-test diagnostics failure"); }

	if (diag_stat & DIAG_STAT_POWER_SUPPLY_MON)       { PX4_ERR("Power supply monitor (VDD < 2.9V)"); }

	if (diag_stat & DIAG_STAT_SPI_COMM_ERROR)         { PX4_ERR("SPI communication error"); }

	if (diag_stat & DIAG_STAT_FLASH_UPDATE_FAILURE)   { PX4_ERR("Flash memory update failure"); }

	if (diag_stat & DIAG_STAT_DATAPATH_OVERRUN)       { PX4_ERR("Datapath processing overrun"); }

	if (diag_stat & DIAG_STAT_SENSOR_INIT_FAILURE)    { PX4_ERR("Sensor initialization failure"); }
}

// --- Status ---

void ADIS1657x::print_status()
{
	I2CSPIDriverBase::print_status();
	PX4_INFO("PROD_ID: 0x%04X", _prod_id);
	PX4_INFO("Sample rate: %.0f Hz", (double)_sample_rate_hz);
	PX4_INFO("DRDY: %s", _data_ready_interrupt_enabled ? "interrupt" : "polling");
	perf_print_counter(_sample_perf);
	perf_print_counter(_bad_transfers);
	perf_print_counter(_reset_perf);
	perf_print_counter(_drdy_missed_perf);
}
