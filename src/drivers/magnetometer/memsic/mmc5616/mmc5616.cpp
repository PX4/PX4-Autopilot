/**
 * @file mmc5616.cpp
 *
 * Implementation of Memsic MMC5616 Driver
 */

#include "mmc5616.hpp"

// define to a noop for quiet
#define MEMSIC_INFO(...)        {PX4_INFO(__VA_ARGS__);}

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

MMC5616::MMC5616(device::Device *interface, const I2CSPIDriverConfig &config) :
	I2CSPIDriver(config),
	_interface(interface),
	_px4_mag(interface->get_device_id(), config.rotation)
{
}

MMC5616::~MMC5616()
{
	perf_free(_comms_errors);
	perf_free(_reset_perf);
	perf_free(_timeout_perf);
	delete _interface;
}

int MMC5616::init()
{
  Reset();
	return 0;
}

void MMC5616::Reset()
{
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
}

void MMC5616::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_reset_perf);
	perf_print_counter(_timeout_perf);
}

bool MMC5616::CheckIDs()
{
	uint8_t chip_id, product_id;

	read(static_cast<uint8_t>(Register::R_CHIP_ID), &chip_id, sizeof(chip_id));
	read(static_cast<uint8_t>(Register::R_PRODUCT_ID), &product_id, sizeof(product_id));

	if (product_id != EXPECTED_PRODUCT_ID) {
    return false;
  }

  return true;
}

void MMC5616::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET: {
			perf_count(_reset_perf);
			RegisterWrite(Register::INTERNAL_CONTROL_1, CONTROL1_BIT::SW_RESET);
			_reset_timestamp = now;
			_failure_count = 0;
			_state = STATE::WAIT_FOR_RESET;
			ScheduleDelayed(25_ms); // reset occurs in 20ms, 25 is nice and "safe"
			break;
		}

	case STATE::WAIT_FOR_RESET: {
			bool chip_detected = CheckIDs();

			if (chip_detected) {
				_state = STATE::CONFIGURE;
				ScheduleDelayed(5_ms);

			} else {
				// RESET not complete
				if (hrt_elapsed_time(&_reset_timestamp) > 10_s) {
					PX4_ERR("Reset failed, retrying");
					Reset();

				} else {
					PX4_DEBUG("Reset not complete, check again in 100 ms");
					ScheduleDelayed(100_ms);
				}
			}

			break;
		}

	case STATE::CONFIGURE: {
			if (Configure()) {
				_state = STATE::READ;
				ScheduleOnInterval(20_ms, 20_ms); // 50 Hz

			} else {
				if (hrt_elapsed_time(&_reset_timestamp) > 30_s) {
					PX4_ERR("Configure failed, resetting");
					Reset();

				} else {
					PX4_DEBUG("Configure failed, retrying");
					ScheduleDelayed(100_ms);
				}
			}

			break;
		}

	case STATE::READ: {
			/* Note:
			 * This sensor has the capability to run freely
			 * and add measurements to an internal FIFO.
			 *
			 * This is currently the absolute most basic way
			 * of getting readings from it.
			 */
			mmc5616_measurement_t measurement = TakeSingleMeasurement();

			if (!measurement.valid) {
				MEMSIC_INFO("In read state, measurement invalid");
				_failure_count++;

				if (_failure_count > 10) {
					Reset();
					return;
				}
			}

			// no failure, decrease the count (if there was one)
			if (_failure_count > 0) {
				_failure_count--;
			}

			_px4_mag.set_error_count(perf_event_count(_timeout_perf) + perf_event_count(_reset_perf));
			_px4_mag.update(now, measurement.x, measurement.y, measurement.z);

			break;
		}
	}
}

MMC5616::mmc5616_measurement_t MMC5616::TakeSingleMeasurement()
{
	const uint8_t attempts = 50;
	const int32_t timeout_max = 10000; // uS
	const uint16_t delay = (timeout_max / attempts); // uS

	mmc5616_measurement_t measurement;
	int32_t timeout = timeout_max; // uS

	// returns void
	RegisterWrite(Register::INTERNAL_CONTROL_0,
		      CONTROL0_BIT::TAKE_MEAS_M | CONTROL0_BIT::AUTO_SR_EN);

	while (timeout > 0) {
		uint8_t reg_status = RegisterRead(Register::STATUS1);

		if (reg_status & STATUS1_BIT::MEAS_M_DONE) {
			break;
		}

		px4_usleep(delay);
		timeout -= delay;
	}

	if (timeout <= 0) {
		MEMSIC_INFO("Measurement timed out!");
		measurement.valid = false;
		perf_count(_timeout_perf);
		return measurement;
	}

	/* read out measurement(s) */
	/* register mapping: xx yy zz x y z */
	const uint8_t cmd = static_cast<uint8_t>(Register::XOUT0);
	uint8_t buffer[9] = {0};

	uint8_t res = _interface->read(cmd, &buffer, sizeof(buffer));

	if (res != PX4_OK) {
		MEMSIC_INFO("Error reading sensor data");
		measurement.valid = false;
		perf_count(_comms_errors);
		return measurement;
	}

	int32_t raw_measurement[3];
	/*
	 * combination of 8-bit values into one 20bit value
	 *
	 * trim off the last four bits (fifo status, unused data)
	 */
	raw_measurement[0] = (buffer[0] << 16 | buffer[1] << 8 | buffer[6]) >> 4;
	raw_measurement[1] = (buffer[2] << 16 | buffer[3] << 8 | buffer[7]) >> 4;
	raw_measurement[2] = (buffer[4] << 16 | buffer[5] << 8 | buffer[8]) >> 4;

	/* conversion to milliGauss */
	measurement.x = ((raw_measurement[0] - RAW_TO_SIGNED_OFFSET) * LSB_TO_MGAUSS_CONVERSION) - _offset.x;
	measurement.y = ((raw_measurement[1] - RAW_TO_SIGNED_OFFSET) * LSB_TO_MGAUSS_CONVERSION) - _offset.y;
	measurement.z = ((raw_measurement[2] - RAW_TO_SIGNED_OFFSET) * LSB_TO_MGAUSS_CONVERSION) - _offset.z;

	measurement.valid = true;

	return measurement;
}

bool MMC5616::Configure()
{
	const uint16_t wait_after_set = 10000; // uS, tmin 1ms
	/* identifies the initial sensor offset, runs the
	 * SET/RESET function for Null Field output temp compensation, and
	 * clears the sensor residual from strong external magnet exposure */

	_offset.x = 0.0f;
	_offset.y = 0.0f;
	_offset.z = 0.0f;

	// set
	RegisterWrite(Register::INTERNAL_CONTROL_0, CONTROL0_BIT::DO_SET);
	px4_usleep(wait_after_set);

	// read
	mmc5616_measurement_t read_after_set = TakeSingleMeasurement();

	if (!read_after_set.valid) {
		return false;
	}

	// reset
	RegisterWrite(Register::INTERNAL_CONTROL_0, CONTROL0_BIT::DO_RESET);
	px4_usleep(wait_after_set);

	// read
	mmc5616_measurement_t read_after_reset = TakeSingleMeasurement();

	if (!read_after_reset.valid) {
		return false;
	}

	// calculate offset
	_offset.x = (read_after_set.x + read_after_reset.x) / 2.0f;
	_offset.y = (read_after_set.y + read_after_reset.y) / 2.0f;
	_offset.z = (read_after_set.z + read_after_reset.z) / 2.0f;

	// set
	RegisterWrite(Register::INTERNAL_CONTROL_0, CONTROL0_BIT::DO_SET);
	px4_usleep(wait_after_set);

	// mag resolution is 0.0625mG per LSB at 20 bits
	_px4_mag.set_scale(0.0625e-3f);

	return true;
}

bool MMC5616::RegisterCheck(const register_config_t &reg_cfg)
{
	bool success = true;

	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) != 0)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
		success = false;
	}

	return success;
}

uint8_t MMC5616::RegisterRead(Register reg)
{
	const uint8_t cmd = static_cast<uint8_t>(reg);
	uint8_t buffer{};

	if (_interface->read(cmd, &buffer, sizeof(buffer)) != PX4_OK) {
		perf_count(_comms_errors);
	}

	return buffer;
}

void MMC5616::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t buffer[2] { (uint8_t)reg, value };

	if (_interface->write(buffer[0], &buffer[1], sizeof(buffer[1])) != PX4_OK) {
		perf_count(_comms_errors);
	}
}

void MMC5616::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);
	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}
