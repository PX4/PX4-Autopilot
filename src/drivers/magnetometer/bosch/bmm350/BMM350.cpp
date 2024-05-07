/****************************************************************************
 *
 *   Copyright (c) Technology Innovation Institute. All rights reserved.
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

#include <px4_platform_common/module.h>

#include "BMM350.hpp"

#include <debug.h>
using namespace time_literals;

// Performance settings; change both in sync if modified!

// Set ODR to 100 Hz, AVG to 4

#define BMM350_AGGR_SETTING	PMU_CMD_AGGR(PMU_CMD_AGGR_ODR::ODR_100HZ, PMU_CMD_AGGR_AVG::AVG_4);
#define BMM350_ODR_IN_US 10000

BMM350::BMM350(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_px4_mag(get_device_id(), config.rotation)
{
}

BMM350::~BMM350()
{
	perf_free(_bad_transfer_perf);
	perf_free(_reset_perf);
	perf_free(_overflow_perf);
	perf_free(_self_test_failed_perf);
}

int BMM350::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool BMM350::Reset()
{
	_state = STATE::RESET;
	_failure_count = 0;
	ScheduleClear();
	ScheduleDelayed(1_ms);
	return true;
}

void BMM350::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_reset_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_overflow_perf);
	perf_print_counter(_self_test_failed_perf);
}

int BMM350::probe()
{
	// 3 retries
	for (int i = 0; i < 3; i++) {
		uint8_t chip_id;
		int ret = RegisterRead(Register::CHIP_ID, &chip_id);

		if (ret == PX4_OK) {
			PX4_DEBUG("CHIP_ID: 0x%02hhX", chip_id);

			if (chip_id == DEFAULT_ID) {
				return PX4_OK;
			}
		}
	}

	return PX4_ERROR;
}

int BMM350::read_otp_word(uint8_t addr, uint16_t *lsb_msb)
{
	int ret;
	uint8_t otp_status;
	uint8_t otp_cmd;
	uint8_t lsb;
	uint8_t msb;

	// Set OTP command at specified address
	otp_cmd = OTP_CMD::DIR_READ | (addr & OTP_CMD::ADDR_MASK);
	ret = RegisterWrite(Register::OTP_CMD_REG, otp_cmd);

	if (ret == PX4_OK) {
		int timeout = 3;

		do {
			// Get OTP status
			ret = RegisterRead(Register::OTP_STATUS_REG, &otp_status);

			if (ret != PX4_OK || otp_status & OTP_STATUS_BIT::ERROR_MASK) {
				return PX4_ERROR;
			}
		} while ((!(otp_status & OTP_STATUS_BIT::OTP_CMD_DONE) || (otp_status & OTP_STATUS_BIT::ERROR_MASK))  && --timeout);

		if (timeout == 0) {
			return PX4_ERROR;
		}

		// Get OTP MSB data
		ret = RegisterRead(Register::OTP_DATA_MSB_REG, &msb);

		if (ret == PX4_OK) {
			// Get OTP LSB data
			ret = RegisterRead(Register::OTP_DATA_LSB_REG, &lsb);
			*lsb_msb = ((uint16_t)msb << 8) | lsb;
		}
	}

	return ret;
}

// Read compensation values from the OTP

int BMM350::update_mag_off_sens()
{
	uint16_t otp_data[32];
	uint16_t off_x_lsb_msb, off_y_lsb_msb, off_z_lsb_msb, off_t_lsb;
	uint8_t sens_x, sens_y, sens_z, sens_t;
	uint8_t tco_x, tco_y, tco_z;
	uint8_t tcs_x, tcs_y, tcs_z;
	uint8_t cross_x_y, cross_y_x, cross_z_x, cross_z_y;

	// Read all otp values
	for (size_t i = 0; i < sizeof(otp_data) / sizeof(otp_data[0]); i++) {
		if (read_otp_word(i, &otp_data[i]) != PX4_OK) {
			return PX4_ERROR;
		}
	}

	// Power off OTP

	if (RegisterWrite(Register::OTP_CMD_REG, OTP_CMD::PWR_OFF_OTP) != PX4_OK) {
		return PX4_ERROR;
	}

	// Set all internal compensation values
	off_x_lsb_msb = otp_data[BMM350_MAG_OFFSET_X] & 0x0FFF;
	off_y_lsb_msb = ((otp_data[BMM350_MAG_OFFSET_X] & 0xF000) >> 4) +
			(otp_data[BMM350_MAG_OFFSET_Y] & BMM350_LSB_MASK);
	off_z_lsb_msb = (otp_data[BMM350_MAG_OFFSET_Y] & 0x0F00) +
			(otp_data[BMM350_MAG_OFFSET_Z] & BMM350_LSB_MASK);
	off_t_lsb = otp_data[BMM350_TEMP_OFF_SENS] & BMM350_LSB_MASK;

	mag_comp.offset_coef[0] = fix_sign(off_x_lsb_msb, 12);
	mag_comp.offset_coef[1] = fix_sign(off_y_lsb_msb, 12);
	mag_comp.offset_coef[2] = fix_sign(off_z_lsb_msb, 12);
	mag_comp.offset_coef_t = fix_sign(off_t_lsb, 8) / 5.0f;

	sens_x = otp_data[BMM350_MAG_SENS_X] >> 8;
	sens_y = (uint8_t)otp_data[BMM350_MAG_SENS_Y];
	sens_z = otp_data[BMM350_MAG_SENS_Z] >> 8;
	sens_t = otp_data[BMM350_TEMP_OFF_SENS] >> 8;

	mag_comp.sensit_coef[0] = fix_sign(sens_x, 8) / 256.0f;
	mag_comp.sensit_coef[1] = (fix_sign(sens_y, 8) / 256.0f) + BMM350_SENS_CORR_Y;
	mag_comp.sensit_coef[2] = fix_sign(sens_z, 8) / 256.0f;
	mag_comp.sensit_coef_t = fix_sign(sens_t, 8) / 512.0f;

	tco_x = (otp_data[BMM350_MAG_TCO_X] & BMM350_LSB_MASK);
	tco_y = (otp_data[BMM350_MAG_TCO_Y] & BMM350_LSB_MASK);
	tco_z = (otp_data[BMM350_MAG_TCO_Z] & BMM350_LSB_MASK);

	mag_comp.tco[0] = fix_sign(tco_x, 8) / 32.0f;
	mag_comp.tco[1] = fix_sign(tco_y, 8) / 32.0f;
	mag_comp.tco[2] = fix_sign(tco_z, 8) / 32.0f;

	tcs_x = (otp_data[BMM350_MAG_TCS_X] & BMM350_MSB_MASK) >> 8;
	tcs_y = (otp_data[BMM350_MAG_TCS_Y] & BMM350_MSB_MASK) >> 8;
	tcs_z = (otp_data[BMM350_MAG_TCS_Z] & BMM350_MSB_MASK) >> 8;

	mag_comp.tcs[0] = fix_sign(tcs_x, 8) / 16384.0f;
	mag_comp.tcs[1] = fix_sign(tcs_y, 8) / 16384.0f;
	mag_comp.tcs[2] = (fix_sign(tcs_z, 8) / 16384.0f) - BMM350_TCS_CORR_Z;

	mag_comp.t0 = (fix_sign(otp_data[BMM350_MAG_T_0], 16) / 512.0f) + 23.0f;

	cross_x_y = (otp_data[BMM350_CROSS_X_Y] & BMM350_LSB_MASK);
	cross_y_x = (otp_data[BMM350_CROSS_Y_X] & BMM350_MSB_MASK) >> 8;
	cross_z_x = (otp_data[BMM350_CROSS_Z_X] & BMM350_LSB_MASK);
	cross_z_y = (otp_data[BMM350_CROSS_Z_Y] & BMM350_MSB_MASK) >> 8;

	mag_comp.cross_axis.cross_x_y = fix_sign(cross_x_y, 8) / 800.0f;
	mag_comp.cross_axis.cross_y_x = fix_sign(cross_y_x, 8) / 800.0f;
	mag_comp.cross_axis.cross_z_x = fix_sign(cross_z_x, 8) / 800.0f;
	mag_comp.cross_axis.cross_z_y = fix_sign(cross_z_y, 8) / 800.0f;

	return PX4_OK;
}

void BMM350::compensate_xyzt(float *xyzt)
{
	// Apply compensation to temperature */

	xyzt[3] = (1 + mag_comp.sensit_coef_t) * xyzt[3] + mag_comp.offset_coef_t;

	// Apply compensation to x, y, z

	for (int i = 0; i < 3; i++) {
		xyzt[i] *= 1 + mag_comp.sensit_coef[i];
		xyzt[i] += mag_comp.offset_coef[i];
		xyzt[i] += mag_comp.tco[i] * (xyzt[3] - mag_comp.t0);
		xyzt[i] /= 1 + mag_comp.tcs[i] * (xyzt[3] - mag_comp.t0);
	}

	float cr_ax_comp_x = (xyzt[0] - mag_comp.cross_axis.cross_x_y * xyzt[1]) /
			     (1 - mag_comp.cross_axis.cross_y_x * mag_comp.cross_axis.cross_x_y);
	float cr_ax_comp_y = (xyzt[1] - mag_comp.cross_axis.cross_y_x * xyzt[0]) /
			     (1 - mag_comp.cross_axis.cross_y_x * mag_comp.cross_axis.cross_x_y);
	float cr_ax_comp_z = (xyzt[2] +
			      (xyzt[0] * (mag_comp.cross_axis.cross_y_x * mag_comp.cross_axis.cross_z_y -
					  mag_comp.cross_axis.cross_z_x) - xyzt[1] *
			       (mag_comp.cross_axis.cross_z_y - mag_comp.cross_axis.cross_x_y *
				mag_comp.cross_axis.cross_z_x)) /
			      (1 - mag_comp.cross_axis.cross_y_x * mag_comp.cross_axis.cross_x_y));

	xyzt[0] = cr_ax_comp_x;
	xyzt[1] = cr_ax_comp_y;
	xyzt[2] = cr_ax_comp_z;
}

enum BMM350::BMM350_measure_res BMM350::measure(float *raw_xyzt, bool ignore_time = false)
{
	enum BMM350::BMM350_measure_res ret;

	struct TransferBuffer {
		uint8_t DUMMY1;
		uint8_t DUMMY2;
		uint8_t MAG_X_XLSB;
		uint8_t MAG_X_LSB;
		uint8_t MAG_X_MSB;
		uint8_t MAG_Y_XLSB;
		uint8_t MAG_Y_LSB;
		uint8_t MAG_Y_MSB;
		uint8_t MAG_Z_XLSB;
		uint8_t MAG_Z_LSB;
		uint8_t MAG_Z_MSB;
		uint8_t TEMP_XLSB;
		uint8_t TEMP_LSB;
		uint8_t TEMP_MSB;
		uint8_t SENSORTIME_XLSB;
		uint8_t SENSORTIME_LSB;
		uint8_t SENSORTIME_MSB;
	} buffer{};

	uint8_t cmd = static_cast<uint8_t>(Register::MAG_X_XLSB);
	uint32_t sampletime;

	if (transfer(&cmd, 1, (uint8_t *)&buffer, sizeof(buffer)) == PX4_OK) {

		// Check sensortime to determine if we got new data. 1 LSB is 39.0625us.

		sampletime = (uint32_t)(buffer.SENSORTIME_XLSB +
					((uint32_t)buffer.SENSORTIME_LSB << 8) +
					((uint32_t)buffer.SENSORTIME_MSB << 16));

		if (sampletime != _prev_sensortime || ignore_time) {
			// New data produced

			_prev_sensortime = sampletime;

			// Combine bits to form x,y,z,t & extend sign

			raw_xyzt[0] = fix_sign(buffer.MAG_X_XLSB + ((uint32_t)buffer.MAG_X_LSB << 8) + ((uint32_t)buffer.MAG_X_MSB << 16), 24);
			raw_xyzt[1] = fix_sign(buffer.MAG_Y_XLSB + ((uint32_t)buffer.MAG_Y_LSB << 8) + ((uint32_t)buffer.MAG_Y_MSB << 16), 24);
			raw_xyzt[2] = fix_sign(buffer.MAG_Z_XLSB + ((uint32_t)buffer.MAG_Z_LSB << 8) + ((uint32_t)buffer.MAG_Z_MSB << 16), 24);
			raw_xyzt[3] = fix_sign(buffer.TEMP_XLSB + ((uint32_t)buffer.TEMP_LSB << 8) + ((uint32_t)buffer.TEMP_MSB << 16), 24);

			// Convert mag lsb to uT and temp lsb to degC

			for (int i = 0; i < 4; i++) {
				raw_xyzt[i] = raw_xyzt[i] * lsb_to_ut_degc[i];
			}

			if (raw_xyzt[3] > 0.0f) {
				raw_xyzt[3] = raw_xyzt[3] - 25.49f;

			} else if (raw_xyzt[3] < 0.0f) {
				raw_xyzt[3] = raw_xyzt[3] + 25.49f;
			}

			ret = BMM350_MEAS_DONE;

		} else {
			ret = BMM350_MEAS_NODATA;
		}

	} else {
		ret = BMM350_MEAS_IO_ERR;
	}

	return ret;
}

void BMM350::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET: {
			if (RegisterWrite(Register::CMD, CMD::softreset) == PX4_OK) {
				_failure_count = 0;
				_state = STATE::WAIT_FOR_RESET;
				perf_count(_reset_perf);
			}

			ScheduleDelayed(24_ms); // 24 ms from reset to suspend
		}
		break;

	case STATE::WAIT_FOR_RESET: {
			int ret = probe();

			if (ret == PX4_OK) {
				uint8_t status0;
				ret = RegisterRead(Register::PMU_CMD_STATUS_0, &status0);

				// Check that we are not in normal mode (i.e. we are in suspend after reset)
				if (ret == PX4_OK && (status0 & PMU_CMD_STATUS_0_BIT::PWR_MODE_IS_NORMAL) != 0) {
					ret = PX4_ERROR;
				}

				if (ret == PX4_OK) {
					_state = STATE::CONFIGURE;
					ScheduleDelayed(1_ms);

				} else {
					// RESET not complete
					PX4_DEBUG("Reset failed, retrying");
					ScheduleDelayed(100_ms);
				}
			}

			break;
		}

	case STATE::CONFIGURE: {
			if (Configure()) {
				// if configure succeeded we need to wait for minimum 1ms for ODR set to take effect. Then start magnetic reset procedure
				_state = STATE::FGR;
				ScheduleDelayed(1_ms);

			} else {
				PX4_DEBUG("Configure failed, resetting");
				_state = STATE::RESET;
				ScheduleNow();
			}
		}
		break;

	case STATE::FGR: {
			// Issue flux-guide reset with full CRST recharge
			int ret = RegisterWrite(Register::PMU_CMD, PMU_CMD::FGR);

			if (ret == PX4_OK) {
				_state = STATE::BR;

			} else {
				_state = STATE::RESET;
			}

			ScheduleDelayed(30_ms);
		}
		break;

	case STATE::BR: {
			uint8_t status0;
			int ret = RegisterRead(Register::PMU_CMD_STATUS_0, &status0);
			_state = STATE::RESET;

			// Check that FGR was executed and issue bit reset with full CRST recharge
			if (ret == PX4_OK && (PMU_CMD_STATUS_0_VAL(status0) == BMM350_PMU_CMD_STATUS_0_FGR)) {
				ret = RegisterWrite(Register::PMU_CMD, PMU_CMD::BR_FAST);

				if (ret == PX4_OK) {
					_state = STATE::MAG_RESET_DONE;
				}
			}

			ScheduleDelayed(4_ms);
		}
		break;

	case STATE::MAG_RESET_DONE: {
			uint8_t status0;
			int ret = RegisterRead(Register::PMU_CMD_STATUS_0, &status0);

			// Check that BR command was executed and start initial measurement for self test
			if (ret == PX4_OK && (PMU_CMD_STATUS_0_VAL(status0) == BMM350_PMU_CMD_STATUS_0_BR_FAST)) {
				_selftest_state = SELFTEST_STATE::ST_INIT;
				_state = STATE::MEASURE_FORCED;

			} else {
				_state = STATE::RESET;
			}

			ScheduleNow();
		}
		break;

	case STATE::MEASURE_FORCED: {
			// Start meausrement in forced mode
			int ret = RegisterWrite(Register::PMU_CMD, PMU_CMD::FM_FAST);

			if (ret == PX4_OK) {
				_state = STATE::SELF_TEST;

			} else {
				_state = STATE::RESET;
			}

			ScheduleDelayed(BMM350_ODR_IN_US);
		}
		break;

	case STATE::SELF_TEST: {
			// Read the measuremet
			float xyzt[4];

			// If anything fails, next state will be reset
			_state = STATE::RESET;

			if (measure(xyzt, true) != PX4_OK) {
				perf_count(_self_test_failed_perf);
				ScheduleNow();
				break;
			}

			switch (_selftest_state) {
			case SELFTEST_STATE::ST_INIT:
				// Store initial measurement
				memcpy(_initial_xyzt, xyzt, sizeof(_initial_xyzt));

				// Enable self test on positive X
				if (RegisterWrite(Register::TMR_SELFTEST_USER, TMR_SELFTEST_USER::POS_X) == PX4_OK) {
					_selftest_state = SELFTEST_STATE::ST_POS_X;
					_state = STATE::MEASURE_FORCED;
				}

				break;

			case SELFTEST_STATE::ST_POS_X:

				// Check the result and enable self test on negative X
				if (xyzt[0] - _initial_xyzt[0] >= 130.0f &&
				    RegisterWrite(Register::TMR_SELFTEST_USER, TMR_SELFTEST_USER::NEG_X) == PX4_OK) {
					_selftest_state = SELFTEST_STATE::ST_NEG_X;
					_state = STATE::MEASURE_FORCED;
				}

				break;

			case SELFTEST_STATE::ST_NEG_X:

				// Check the result and enable self test on positive Y
				if (xyzt[0] - _initial_xyzt[0] <= -130.0f &&
				    RegisterWrite(Register::TMR_SELFTEST_USER, TMR_SELFTEST_USER::POS_Y) == PX4_OK) {
					_selftest_state = SELFTEST_STATE::ST_POS_Y;
					_state = STATE::MEASURE_FORCED;
				}

				break;

			case SELFTEST_STATE::ST_POS_Y:

				// Check the result and enable self test on negative Y
				if (xyzt[1] - _initial_xyzt[1] >= 130.0f &&
				    RegisterWrite(Register::TMR_SELFTEST_USER, TMR_SELFTEST_USER::NEG_Y) == PX4_OK) {
					_selftest_state = SELFTEST_STATE::ST_NEG_Y;
					_state = STATE::MEASURE_FORCED;
				}

				break;

			case SELFTEST_STATE::ST_NEG_Y:

				// Check the result and disable self test
				if (xyzt[1] - _initial_xyzt[1] <= -130.0f &&
				    RegisterWrite(Register::TMR_SELFTEST_USER, TMR_SELFTEST_USER::DISABLE) == PX4_OK) {
					_state = STATE::SET_NORMAL_MODE;
				}

				break;
			}

			if (_state == STATE::RESET) {
				perf_count(_self_test_failed_perf);
			}

			ScheduleDelayed(1_ms);
		}
		break;

	case STATE::SET_NORMAL_MODE: {
			// Switch from suspend to normal mode. After this we need 38ms delay

			int ret = RegisterWrite(Register::PMU_CMD, PMU_CMD::NM);

			if (ret == PX4_OK) {
				// Start reading every 50 ms (20 Hz)
				_state = STATE::READ;
				ScheduleOnInterval(50_ms, 50_ms);

			} else {
				_state = STATE::RESET;
			}
		}
		break;

	case STATE::READ: {
			float xyzt[4];

			enum BMM350_measure_res res = measure(xyzt);

			switch (res) {
			case BMM350_MEAS_DONE: {
					// Check for overflow (SQRT(Hx^2 + Hy^2 + Hz^2) < 2000µT)
					if (xyzt[0] * xyzt[0] + xyzt[1] * xyzt[1] + xyzt[2] * xyzt[2] < 4000000.0f) {
						// Get compensated x,y,z and temperature
						compensate_xyzt(xyzt);

						// Set temperature, update and reset failure count
						_px4_mag.set_temperature(xyzt[3]);
						_px4_mag.update(now, xyzt[0], xyzt[1], xyzt[2]);
						_failure_count = 0;

					} else {
						_failure_count++;
						perf_count(_overflow_perf);
					}

					_px4_mag.set_error_count(perf_event_count(_overflow_perf) +
								 perf_event_count(_bad_transfer_perf) +
								 perf_event_count(_self_test_failed_perf));
				}
				break;

			case BMM350_MEAS_NODATA:
				_failure_count++;
				break;

			case BMM350_MEAS_IO_ERR:
				perf_count(_bad_transfer_perf);
				Reset();
				break;
			}

			if (_failure_count > 10) {
				Reset();
			}
		}
		break;
	}
}

int BMM350::SetODR_AVG(uint8_t aggr)
{
	// Set ODR, AVG

	int ret = RegisterWrite(Register::PMU_CMD_AGGR_SET, aggr);

	if (ret == PX4_OK) {
		ret = RegisterWrite(Register::PMU_CMD, PMU_CMD::UPD_OAE);
	}

	return ret;
}

bool BMM350::Configure()
{
	// Read OTP data
	int ret = update_mag_off_sens();

	if (ret != PX4_OK) {
		return false;
	}

	uint8_t aggr = BMM350_AGGR_SETTING;
	ret = SetODR_AVG(aggr);

	if (ret != PX4_OK) {
		return false;
	}

	// uT to Gauss

	_px4_mag.set_scale(0.01f);

	return true;
}

int BMM350::RegisterRead(Register reg, uint8_t *val)
{
	const uint8_t cmd = static_cast<uint8_t>(reg);
	uint8_t data[3];
	int ret = transfer(&cmd, 1, data, sizeof(data));

	if (ret != PX4_OK) {
		PX4_DEBUG("register read 0x%02hhX failed, ret = %d", cmd, ret);

	} else {
		// Each read produces 2 dummy bytes as per spec. Disacard them
		*val = data[2];
	}

	return ret;
}

int BMM350::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t buffer[2] { static_cast<uint8_t>(reg), value };
	int ret = transfer(buffer, sizeof(buffer), nullptr, 0);

	if (ret != PX4_OK) {
		PX4_DEBUG("register write 0x%02hhX failed, ret = %d", (uint8_t)reg, ret);
	}

	return ret;
}

int BMM350::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	uint8_t orig_val;
	int ret = RegisterRead(reg, &orig_val);

	if (ret == PX4_OK) {
		uint8_t val = (orig_val & ~clearbits) | setbits;

		if (orig_val != val) {
			ret = RegisterWrite(reg, val);
		}
	}

	return ret;
}

void BMM350::print_usage()
{
	PRINT_MODULE_USAGE_NAME("bmm350", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("magnetometer");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x10);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, 35, "Rotation", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}
