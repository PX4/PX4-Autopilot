/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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

#include "BMM350.hpp"
using namespace time_literals;

BMM350::BMM350(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	ModuleParams(nullptr),
	_px4_mag(get_device_id(), config.rotation)

{
}

BMM350::~BMM350()
{
	perf_free(_reset_perf);
	perf_free(_bad_read_perf);
	perf_free(_self_test_failed_perf);
}

int BMM350::init()
{
	ModuleParams::updateParams();
	ParametersUpdate(true);
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool BMM350::Reset()
{
	RegisterWrite(Register::CMD, SOFT_RESET);
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleDelayed(1_ms);
	return true;
}

void BMM350::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_reset_perf);
	perf_print_counter(_bad_read_perf);
	perf_print_counter(_self_test_failed_perf);
}

void BMM350::ParametersUpdate(bool force)
{
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
		UpdateMagParams();
	}
}

void BMM350::UpdateMagParams()
{
	uint8_t odr = GetODR(_param_bmm350_odr.get());
	uint8_t avg = GetAVG(_param_bmm350_avg.get());

	_mag_odr_mode = odr;
	_mag_avg_mode = avg;
	_mag_pad_drive = static_cast<uint8_t>(_param_bmm350_drive.get());
	PX4_DEBUG("Set params odr = %d, avg = %d, drive = %d", _mag_odr_mode, _mag_avg_mode, _mag_pad_drive);
}

uint8_t BMM350::GetODR(int value)
{
	switch (value) {
	case 0: return ODR_400HZ;

	case 1: return ODR_200HZ;

	case 2: return ODR_100HZ;

	case 3: return ODR_50HZ;

	case 4: return ODR_25HZ;

	case 5: return ODR_12_5HZ;

	case 6: return ODR_6_25HZ;

	case 7: return ODR_3_125HZ;

	case 8: return ODR_1_5625HZ;

	default: return ODR_200HZ;
	}
}

hrt_abstime BMM350::OdrToUs(uint8_t odr)
{
	switch (odr) {
	case ODR_400HZ:
		return 2500_us;

	case ODR_200HZ:
		return 5000_us;

	case ODR_100HZ:
		return 10000_us;

	case ODR_50HZ:
		return 20000_us;

	case ODR_25HZ:
		return 40000_us;

	case ODR_12_5HZ:
		return 80000_us;

	case ODR_6_25HZ:
		return 160000_us;

	case ODR_3_125HZ:
		return 320000_us;

	case ODR_1_5625HZ:
		return 640000_us;

	default:
		return 5000_us;
	}
}

uint8_t BMM350::GetAVG(int value)
{
	switch (value) {
	case 0: return AVG_NO_AVG;

	case 1: return AVG_2;

	case 2: return AVG_4;

	case 3: return AVG_8;

	default: return AVG_2;
	}
}


int BMM350::probe()
{
	for (int i = 0; i < 3; i++) {
		uint8_t chip_id;

		if (PX4_OK == RegisterRead(Register::CHIP_ID, &chip_id)) {
			PX4_DEBUG("CHIP_ID: 0x%02hhX", chip_id);

			if (chip_id == chip_identification_number) {
				return PX4_OK;
			}
		}
	}

	return PX4_ERROR;
}

void BMM350::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();
	int ret = PX4_OK;
	ParametersUpdate();

	switch (_state) {
	case STATE::RESET: {
			if (RegisterWrite(Register::CMD, SOFT_RESET) == PX4_OK) {
				_reset_timestamp = now;
				_state = STATE::WAIT_FOR_RESET;
				perf_count(_reset_perf);
			}

			ScheduleDelayed(10_ms);
		}
		break;

	case STATE::WAIT_FOR_RESET: {
			ret = probe();

			if (ret == PX4_OK) {
				_state = STATE::RESET;
				uint8_t status_0;
				ret = RegisterRead(Register::PMU_STATUS_0, &status_0);

				if (ret == PX4_OK && (status_0 & PWR_NORMAL) != 0) {
					ret = PX4_ERROR;
				}

				if (ret == PX4_OK) {
					ret = UpdateMagOffsets();
				}

				if (ret == PX4_OK) {
					PX4_DEBUG("Going to FGR");
					_state = STATE::FGR;

				}

				ScheduleDelayed(10_ms);

			} else {
				if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
					PX4_DEBUG("Reset failed, retrying");
					_state = STATE::RESET;
					ScheduleDelayed(100_ms);

				} else {
					PX4_DEBUG("Reset not complete, check again in 10 ms");
					ScheduleDelayed(10_ms);
				}
			}
		}
		break;

	case STATE::FGR: {
			_state = STATE::RESET;
			uint8_t odr_reg_data = (ODR_100HZ & 0xf);
			odr_reg_data = ((odr_reg_data & ~(0x30)) | ((AVG_2 << 0x4) & 0x30));
			ret = RegisterWrite(Register::PMU_CMD_AGGR_SET, odr_reg_data);

			if (ret == PX4_OK) {
				ret = RegisterWrite(Register::PMU_CMD_AXIS_EN, EN_XYZ);
			}

			if (ret == PX4_OK) {
				ret = RegisterWrite(Register::PMU_CMD, PMU_CMD_FGR);
			}

			if (ret == PX4_OK) {
				PX4_DEBUG("Going to BR");
				_state = STATE::BR;
			}

			ScheduleDelayed(30_ms);
		}
		break;

	case STATE::BR: {
			uint8_t status_0;
			_state = STATE::RESET;
			ret = RegisterRead(Register::PMU_STATUS_0, &status_0);

			if (ret == PX4_OK && PMU_CMD_STATUS_0_RES(status_0) == PMU_STATUS_FGR) {
				ret = RegisterWrite(Register::PMU_CMD, PMU_CMD_BR_FAST);

				if (ret == PX4_OK) {
					PX4_DEBUG("Going to after reset");
					_state = STATE::AFTER_RESET;
				}
			}

			ScheduleDelayed(4_ms);
		}
		break;

	case STATE::AFTER_RESET: {
			uint8_t status_0;
			_state = STATE::RESET;
			ret = RegisterRead(Register::PMU_STATUS_0, &status_0);

			if (ret == PX4_OK && PMU_CMD_STATUS_0_RES(status_0) == PMU_STATUS_BR_FAST) {
				_state = STATE::MEASURE_FORCED;
				_self_test_state = SELF_TEST_STATE::INIT;
				PX4_DEBUG("Going to fast FM");
			}

			ScheduleNow();
		}
		break;

	case STATE::MEASURE_FORCED: {
			ret = RegisterWrite(Register::PMU_CMD, PMU_CMD_FAST_FM);

			if (ret == PX4_OK) {
				_state = STATE::SELF_TEST_CHECK;

			} else {
				_state = STATE::RESET;
			}

			ScheduleDelayed(OdrToUs(_mag_odr_mode));
			break;
		}

	case STATE::SELF_TEST_CHECK: {

			float xyzt[4];
			_state = STATE::RESET;

			if (ReadOutRawData(xyzt) != PX4_OK) {
				perf_count(_self_test_failed_perf);
				ScheduleNow();
				break;
			}

			switch (_self_test_state) {
			case SELF_TEST_STATE::INIT:
				memcpy(_initial_self_test_values, xyzt, sizeof(_initial_self_test_values));

				if (RegisterWrite(Register::TMR_SELF_TEST_USER, SELF_TEST_POS_X) == PX4_OK) {
					_self_test_state = SELF_TEST_STATE::POS_X;
					_state = STATE::MEASURE_FORCED;
				}

				break;

			case SELF_TEST_STATE::POS_X:
				if (xyzt[0] - _initial_self_test_values[0] >= 130.0f &&
				    RegisterWrite(Register::TMR_SELF_TEST_USER, SELF_TEST_NEG_X) == PX4_OK) {
					_self_test_state = SELF_TEST_STATE::NEG_X;
					_state = STATE::MEASURE_FORCED;
				}

				break;

			case SELF_TEST_STATE::NEG_X:
				if (xyzt[0] - _initial_self_test_values[0] <= -130.0f &&
				    RegisterWrite(Register::TMR_SELF_TEST_USER, SELF_TEST_POS_Y) == PX4_OK) {
					_self_test_state = SELF_TEST_STATE::POS_Y;
					_state = STATE::MEASURE_FORCED;
				}

				break;

			case SELF_TEST_STATE::POS_Y:
				if (xyzt[1] - _initial_self_test_values[1] >= 130.0f &&
				    RegisterWrite(Register::TMR_SELF_TEST_USER, SELF_TEST_NEG_Y) == PX4_OK) {
					_self_test_state = SELF_TEST_STATE::NEG_Y;
					_state = STATE::MEASURE_FORCED;
				}

				break;

			case SELF_TEST_STATE::NEG_Y:
				if (xyzt[1] - _initial_self_test_values[1] <= -130.0f &&
				    RegisterWrite(Register::TMR_SELF_TEST_USER, SELF_TEST_DISABLE) == PX4_OK) {
					PX4_DEBUG("Self test good, going to configure");
					_state = STATE::CONFIGURE;
				}

				break;

			}

			if (_state == STATE::RESET) {
				PX4_DEBUG("Self test failed");
				perf_count(_self_test_failed_perf);
			}

			ScheduleDelayed(1_ms);
		}
		break;


	case STATE::CONFIGURE:
		if (Configure() == PX4_OK) {
			_state = STATE::READ;
			PX4_DEBUG("Configure went fine");
			ScheduleOnInterval(OdrToUs(_mag_odr_mode), 50_ms);

		} else {
			// CONFIGURE not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Configure failed, resetting");
				_state = STATE::RESET;

			} else {
				PX4_DEBUG("Configure failed, retrying");
			}

			ScheduleDelayed(100_ms);
		}

		break;

	case STATE::READ: {
			// -- start get_compensated_mag_xyz_temp_data
			float out_data[4] = {0.0f};
			float dut_offset_coeff[3], dut_sensit_coeff[3], dut_tcos[3], dut_tcss[3];
			float cr_ax_comp_x, cr_ax_comp_y, cr_ax_comp_z;

			ret = ReadOutRawData(out_data);

			if (ret == PX4_OK) {
				// Apply compensation to temperature reading
				out_data[3] = (1 + _mag_comp_vals.dut_sensit_coef.t_sens) * out_data[3] +
					      _mag_comp_vals.dut_offset_coef.t_offs;

				// Store magnetic compensation structure to an array
				dut_offset_coeff[0] = _mag_comp_vals.dut_offset_coef.offset_x;
				dut_offset_coeff[1] = _mag_comp_vals.dut_offset_coef.offset_y;
				dut_offset_coeff[2] = _mag_comp_vals.dut_offset_coef.offset_z;

				dut_sensit_coeff[0] = _mag_comp_vals.dut_sensit_coef.sens_x;
				dut_sensit_coeff[1] = _mag_comp_vals.dut_sensit_coef.sens_y;
				dut_sensit_coeff[2] = _mag_comp_vals.dut_sensit_coef.sens_z;

				dut_tcos[0] = _mag_comp_vals.dut_tco.tco_x;
				dut_tcos[1] = _mag_comp_vals.dut_tco.tco_y;
				dut_tcos[2] = _mag_comp_vals.dut_tco.tco_z;

				dut_tcss[0] = _mag_comp_vals.dut_tcs.tcs_x;
				dut_tcss[1] = _mag_comp_vals.dut_tcs.tcs_y;
				dut_tcss[2] = _mag_comp_vals.dut_tcs.tcs_z;

				for (int idx = 0; idx < 3; idx++) {
					out_data[idx] *= 1 + dut_sensit_coeff[idx];
					out_data[idx] += dut_offset_coeff[idx];
					out_data[idx] += dut_tcos[idx] * (out_data[3] - _mag_comp_vals.dut_t0);
					out_data[idx] /= 1 + dut_tcss[idx] * (out_data[3] - _mag_comp_vals.dut_t0);
				}

				cr_ax_comp_x = (out_data[0] - _mag_comp_vals.cross_axis.cross_x_y * out_data[1]) /
					       (1 - _mag_comp_vals.cross_axis.cross_y_x * _mag_comp_vals.cross_axis.cross_x_y);
				cr_ax_comp_y = (out_data[1] - _mag_comp_vals.cross_axis.cross_y_x * out_data[0]) /
					       (1 - _mag_comp_vals.cross_axis.cross_y_x * _mag_comp_vals.cross_axis.cross_x_y);
				cr_ax_comp_z =
					(out_data[2] +
					 (out_data[0] *
					  (_mag_comp_vals.cross_axis.cross_y_x * _mag_comp_vals.cross_axis.cross_z_y -
					   _mag_comp_vals.cross_axis.cross_z_x) -
					  out_data[1] *
					  (_mag_comp_vals.cross_axis.cross_z_y - _mag_comp_vals.cross_axis.cross_x_y *
					   _mag_comp_vals.cross_axis.cross_z_x)) /
					 (1 - _mag_comp_vals.cross_axis.cross_y_x * _mag_comp_vals.cross_axis.cross_x_y));

				out_data[0] = cr_ax_comp_x;
				out_data[1] = cr_ax_comp_y;
				out_data[2] = cr_ax_comp_z;
				_px4_mag.set_error_count(perf_event_count(_bad_read_perf) + perf_event_count(_self_test_failed_perf));
				_px4_mag.set_temperature(out_data[3]);
				_px4_mag.update(now, out_data[0], out_data[1], out_data[2]);

			} else {
				perf_count(_bad_read_perf);
			}
		}

		break;
	}
}

int BMM350::Configure()
{
	PX4_DEBUG("Configuring");
	uint8_t readData = 0;
	int ret;

	// Set pad drive
	ret = RegisterWrite(Register::PAD_CTRL, (_mag_pad_drive & 0x7));

	if (ret != PX4_OK) {
		PX4_DEBUG("Couldn't set pad drive, defaults to 7");
		return ret;
	}

	// Set PMU data aggregation
	uint8_t odr = _mag_odr_mode;
	uint8_t avg = _mag_avg_mode;

	if (odr == ODR_400HZ && avg >= AVG_2) {
		avg = AVG_NO_AVG;

	} else if (odr == ODR_200HZ && avg >= AVG_4) {
		avg = AVG_2;

	} else if (odr == ODR_100HZ && avg >= AVG_8) {
		avg = AVG_4;
	}

	uint8_t odr_reg_data = (odr & 0xf);
	odr_reg_data = ((odr_reg_data & ~(0x30)) | ((avg << 0x4) & 0x30));

	ret = RegisterWrite(Register::PMU_CMD_AGGR_SET, odr_reg_data);

	if (ret != PX4_OK) {
		PX4_DEBUG("Cannot set PMU AGG REG");
		return ret;
	}

	ret = RegisterRead(Register::PMU_CMD_AGGR_SET, &readData);

	if (ret != PX4_OK || readData != odr_reg_data) {
		PX4_DEBUG("Couldn't check PMU AGGR REG");
		return ret;
	}

	odr_reg_data = PMU_CMD_UPDATE_OAE;
	ret = RegisterWrite(Register::PMU_CMD, odr_reg_data);

	if (ret != PX4_OK) {
		PX4_DEBUG("Couldn't write PMU CMD REG");
		return ret;
	}

	ret = RegisterRead(Register::PMU_CMD, &readData);

	if (ret != PX4_OK || readData != odr_reg_data) {
		PX4_DEBUG("Couldn't check PMU CMD REG");
		return ret;
	}

	// Enable AXIS
	uint8_t axis_data = EN_X | EN_Y | EN_Z;
	// PMU_CMD_AXIS_EN
	ret = RegisterWrite(Register::PMU_CMD_AXIS_EN, axis_data);

	if (ret != PX4_OK) {
		PX4_DEBUG("Couldn't write AXIS data");
		return ret;
	}

	ret = RegisterRead(Register::PMU_CMD_AXIS_EN, &readData);

	if (ret != PX4_OK || readData != axis_data) {
		PX4_DEBUG("Couldn't cross check the set AXIS");
		return ret;
	}

	ret = RegisterWrite(Register::PMU_CMD, PMU_CMD_NM);

	if (ret != PX4_OK) {
		PX4_DEBUG("Failed to start mag in normal mode");
		return ret;
	}

	// microTesla -> Gauss
	_px4_mag.set_scale(0.01f);

	return ret;
}

int32_t BMM350::FixSign(uint32_t inval, int8_t num_bits)
{
	int32_t power = 1 << (num_bits - 1); // Calculate 2^(num_bits - 1)
	int32_t retval = static_cast<int32_t>(inval);

	if (retval >= power) {
		retval -= (power << 1); // Equivalent to retval = retval - (power * 2)
	}

	return retval;
}

int BMM350::ReadOutRawData(float *out_data)
{
	if (out_data == NULL) {
		return -1;
	}

	float temp = 0.0;
	struct BMM350::raw_mag_data raw_data = {0};

	// --- Start read_uncomp_mag_temp_data
	uint8_t mag_data[14] = {0};

	uint32_t raw_mag_x, raw_mag_y, raw_mag_z, raw_temp;
	uint8_t cmd = static_cast<uint8_t>(Register::DATAX_XLSB);

	uint8_t res = transfer(&cmd, 1, (uint8_t *)&mag_data, sizeof(mag_data));

	if (res != PX4_OK) {
		return -1;
	}

	raw_mag_x = mag_data[2] + ((uint32_t)mag_data[3] << 8) + ((uint32_t)mag_data[4] << 16);
	raw_mag_y = mag_data[5] + ((uint32_t)mag_data[6] << 8) + ((uint32_t)mag_data[7] << 16);
	raw_mag_z = mag_data[8] + ((uint32_t)mag_data[9] << 8) + ((uint32_t)mag_data[10] << 16);
	raw_temp = mag_data[11] + ((uint32_t)mag_data[12] << 8) + ((uint32_t)mag_data[13] << 16);

	raw_data.raw_x = FixSign(raw_mag_x, 24);
	raw_data.raw_y = FixSign(raw_mag_y, 24);
	raw_data.raw_z = FixSign(raw_mag_z, 24);
	raw_data.raw_t = FixSign(raw_temp, 24);
	// --- End read_uncomp_mag_temp_data

	// --- Start read_out_raw_data
	out_data[0] = (float)raw_data.raw_x * lsb_to_utc_degc[0];
	out_data[1] = (float)raw_data.raw_y * lsb_to_utc_degc[1];
	out_data[2] = (float)raw_data.raw_z * lsb_to_utc_degc[2];
	out_data[3] = (float)raw_data.raw_t * lsb_to_utc_degc[3];

	// Adjust temperature
	if (out_data[3] > 0.0f) {
		temp = (float)(out_data[3] - (1 * 25.49f));

	} else if (out_data[3] < 0.0f) {
		temp = (float)(out_data[3] - (-1 * 25.49f));

	} else {
		temp = (float)(out_data[3]);
	}

	out_data[3] = temp;

	return res;
}

int BMM350::ReadOTPWord(uint8_t addr, uint16_t *lsb_msb)
{
	uint8_t otp_cmd = OTP_DIR_READ | (addr & OTP_WORD_MSK);
	int ret = RegisterWrite(Register::OTP_CMD, otp_cmd);
	uint8_t otp_status = 0;

	if (ret == PX4_OK) {
		do {
			ret = RegisterRead(Register::OTP_STATUS, &otp_status);

			if (ret != PX4_OK) {
				return PX4_ERROR;
			}
		} while (!(otp_status & 0x01));

		uint8_t msb = 0, lsb = 0;
		ret = RegisterRead(Register::OTP_DATA_MSB, &msb);

		if (ret == PX4_OK) {
			ret  = RegisterRead(Register::OTP_DATA_LSB, &lsb);

			if (ret == PX4_OK) {
				*lsb_msb = ((msb << 8) | lsb) & 0xffff;
			}
		}
	}

	return ret;
}

int BMM350::UpdateMagOffsets()
{
	PX4_DEBUG("DUMPING OTP");
	uint16_t otp_data[32] = {0};

	for (int idx = 0; idx < 32; idx++) {
		if (ReadOTPWord(idx, &otp_data[idx]) != PX4_OK) {
			return PX4_ERROR;
		}

		PX4_DEBUG("i: %i, val = %i", idx, otp_data[idx]);
	}

	if (RegisterWrite(Register::OTP_CMD, PWR_OFF_OTP) != PX4_OK) {
		return PX4_ERROR;
	}

	PX4_DEBUG("var_id: %i", (otp_data[30] & 0x7f00) >> 9);

	PX4_DEBUG("UPDATING OFFSETS");
	uint16_t off_x_lsb_msb, off_y_lsb_msb, off_z_lsb_msb, t_off;
	uint8_t sens_x, sens_y, sens_z, t_sens;
	uint8_t tco_x, tco_y, tco_z;
	uint8_t tcs_x, tcs_y, tcs_z;
	uint8_t cross_x_y, cross_y_x, cross_z_x, cross_z_y;

	off_x_lsb_msb = otp_data[0x0E] & 0x0FFF;
	off_y_lsb_msb = ((otp_data[0x0E] & 0xF000) >> 4) +
			(otp_data[0x0F] & 0x00FF);
	off_z_lsb_msb = (otp_data[0x0F] & 0x0F00) +
			(otp_data[0x10] & 0x00FF);
	t_off = otp_data[0x0D] & 0x00FF;

	_mag_comp_vals.dut_offset_coef.offset_x = FixSign(off_x_lsb_msb, 12);
	_mag_comp_vals.dut_offset_coef.offset_y = FixSign(off_y_lsb_msb, 12);
	_mag_comp_vals.dut_offset_coef.offset_z = FixSign(off_z_lsb_msb, 12);
	_mag_comp_vals.dut_offset_coef.t_offs = FixSign(t_off, 8) / 5.0f;

	sens_x = (otp_data[0x10] & 0xFF00) >> 8;
	sens_y = (otp_data[0x11] & 0x00FF);
	sens_z = (otp_data[0x11] & 0xFF00) >> 8;
	t_sens = (otp_data[0x0D] & 0xFF00) >> 8;

	_mag_comp_vals.dut_sensit_coef.sens_x = FixSign(sens_x, 8) / 256.0f;
	_mag_comp_vals.dut_sensit_coef.sens_y = (FixSign(sens_y, 8) / 256.0f) + 0.01f;
	_mag_comp_vals.dut_sensit_coef.sens_z = FixSign(sens_z, 8) / 256.0f;
	_mag_comp_vals.dut_sensit_coef.t_sens = FixSign(t_sens, 8) / 512.0f;

	tco_x = (otp_data[0x12] & 0x00FF);
	tco_y = (otp_data[0x13] & 0x00FF);
	tco_z = (otp_data[0x14] & 0x00FF);

	_mag_comp_vals.dut_tco.tco_x = FixSign(tco_x, 8) / 32.0f;
	_mag_comp_vals.dut_tco.tco_y = FixSign(tco_y, 8) / 32.0f;
	_mag_comp_vals.dut_tco.tco_z = FixSign(tco_z, 8) / 32.0f;

	tcs_x = (otp_data[0x12] & 0xFF00) >> 8;
	tcs_y = (otp_data[0x13] & 0xFF00) >> 8;
	tcs_z = (otp_data[0x14] & 0xFF00) >> 8;

	_mag_comp_vals.dut_tcs.tcs_x = FixSign(tcs_x, 8) / 16384.0f;
	_mag_comp_vals.dut_tcs.tcs_y = FixSign(tcs_y, 8) / 16384.0f;
	_mag_comp_vals.dut_tcs.tcs_z = (FixSign(tcs_z, 8) / 16384.0f) - 0.0001f;

	_mag_comp_vals.dut_t0 = (FixSign(otp_data[0x18], 16) / 512.0f) + 23.0f;

	cross_x_y = (otp_data[0x15] & 0x00FF);
	cross_y_x = (otp_data[0x15] & 0xFF00) >> 8;
	cross_z_x = (otp_data[0x16] & 0x00FF);
	cross_z_y = (otp_data[0x16] & 0xFF00) >> 8;

	_mag_comp_vals.cross_axis.cross_x_y = FixSign(cross_x_y, 8) / 800.0f;
	_mag_comp_vals.cross_axis.cross_y_x = FixSign(cross_y_x, 8) / 800.0f;
	_mag_comp_vals.cross_axis.cross_z_x = FixSign(cross_z_x, 8) / 800.0f;
	_mag_comp_vals.cross_axis.cross_z_y = FixSign(cross_z_y, 8) / 800.0f;
	return PX4_OK;
}

int BMM350::RegisterRead(Register reg, uint8_t *value)
{
	const uint8_t cmd = static_cast<uint8_t>(reg);
	uint8_t buffer[3] = {0};
	int ret = transfer(&cmd, 1, buffer, 3);

	if (ret != PX4_OK) {
		PX4_DEBUG("register read 0x%02hhX failed, ret = %d", cmd, ret);

	} else {
		*value = buffer[2];
	}

	return ret;
}

int BMM350::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t buffer[2] {(uint8_t)reg, value};
	int ret = transfer(buffer, sizeof(buffer), nullptr, 0);

	if (ret != PX4_OK) {
		PX4_DEBUG("register write 0x%02hhX failed, ret = %d", (uint8_t)reg, ret);
	}

	return ret;
}
