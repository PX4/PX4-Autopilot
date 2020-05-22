/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "ICM20948_AK09916.hpp"

#include "ICM20948.hpp"

using namespace time_literals;

namespace AKM_AK09916
{

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

ICM20948_AK09916::ICM20948_AK09916(ICM20948 &icm20948, enum Rotation rotation) :
	ScheduledWorkItem("icm20948_ak09916", px4::device_bus_to_wq(icm20948.get_device_id())),
	_icm20948(icm20948),
	_px4_mag(icm20948.get_device_id(), ORB_PRIO_DEFAULT, rotation)
{
	_px4_mag.set_device_type(DRV_MAG_DEVTYPE_AK09916);
	_px4_mag.set_external(icm20948.external());
}

ICM20948_AK09916::~ICM20948_AK09916()
{
	ScheduleClear();

	perf_free(_transfer_perf);
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_duplicate_data_perf);
	perf_free(_data_not_ready);
}

bool ICM20948_AK09916::Init()
{
	return Reset();
}

bool ICM20948_AK09916::Reset()
{
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void ICM20948_AK09916::PrintInfo()
{
	perf_print_counter(_transfer_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_duplicate_data_perf);
	perf_print_counter(_data_not_ready);

	_px4_mag.print_status();
}

void ICM20948_AK09916::Run()
{
	switch (_state) {
	case STATE::RESET:
		// CNTL3 SRST: Soft reset
		RegisterWrite(Register::CNTL3, CNTL3_BIT::SRST);
		_reset_timestamp = hrt_absolute_time();
		_state = STATE::READ_WHO_AM_I;
		ScheduleDelayed(100_ms);
		break;

	case STATE::READ_WHO_AM_I:
		_icm20948.I2CSlaveRegisterStartRead(I2C_ADDRESS_DEFAULT, (uint8_t)Register::WIA);
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(10_ms);
		break;

	case STATE::WAIT_FOR_RESET: {

			uint8_t WIA = 0;
			_icm20948.I2CSlaveExternalSensorDataRead(&WIA, 1);

			if (WIA == WHOAMI) {
				// if reset succeeded then configure
				PX4_DEBUG("AK09916 reset successful, configuring");
				_state = STATE::CONFIGURE;
				ScheduleDelayed(10_ms);

			} else {
				// RESET not complete
				if (hrt_elapsed_time(&_reset_timestamp) > 100_ms) {
					PX4_DEBUG("Reset failed, retrying");
					_state = STATE::RESET;
					ScheduleDelayed(100_ms);

				} else {
					PX4_DEBUG("Reset not complete, check again in 100 ms");
					ScheduleDelayed(100_ms);
				}
			}
		}

		break;

	// TODO: read FUSE ROM (to get ASA corrections)

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then start reading
			PX4_DEBUG("AK09916 configure successful, reading");
			_icm20948.I2CSlaveExternalSensorDataEnable(I2C_ADDRESS_DEFAULT, (uint8_t)Register::ST1, sizeof(TransferBuffer));
			_state = STATE::READ;
			ScheduleOnInterval(20_ms, 20_ms); // 50 Hz

		} else {
			PX4_DEBUG("Configure failed, retrying");
			// try again in 100 ms
			ScheduleDelayed(100_ms);
		}

		break;

	case STATE::READ: {
			perf_begin(_transfer_perf);

			TransferBuffer buffer{};
			const hrt_abstime timestamp_sample = hrt_absolute_time();
			bool success = _icm20948.I2CSlaveExternalSensorDataRead((uint8_t *)&buffer, sizeof(TransferBuffer));

			perf_end(_transfer_perf);

			if (success && !(buffer.ST2 & ST2_BIT::HOFL) && (buffer.ST1 & ST1_BIT::DRDY)) {
				// sensor's frame is +y forward (x), -x right, +z down
				int16_t x = combine(buffer.HYH, buffer.HYL); // +Y
				int16_t y = combine(buffer.HXH, buffer.HXL); // +X
				y = (y == INT16_MIN) ? INT16_MAX : -y; // flip y
				int16_t z = combine(buffer.HZH, buffer.HZL);

				const bool all_zero = (x == 0 && y == 0 && z == 0);
				const bool new_data = (_last_measurement[0] != x || _last_measurement[1] != y || _last_measurement[2] != z);

				if (!new_data) {
					perf_count(_duplicate_data_perf);
				}

				if (!all_zero && new_data) {
					_px4_mag.update(timestamp_sample, x, y, z);

					_last_measurement[0] = x;
					_last_measurement[1] = y;
					_last_measurement[2] = z;

				} else {
					success = false;
				}

			} else {
				perf_count(_data_not_ready);
			}

			if (!success) {
				perf_count(_bad_transfer_perf);
			}
		}

		break;
	}
}

bool ICM20948_AK09916::Configure()
{
	bool success = true;

	for (const auto &reg : _register_cfg) {
		if (!RegisterCheck(reg)) {
			success = false;
		}
	}

	// TODO: read ASA and set sensitivity

	//const uint8_t ASAX = RegisterRead(Register::ASAX);
	//const uint8_t ASAY = RegisterRead(Register::ASAY);
	//const uint8_t ASAZ = RegisterRead(Register::ASAZ);

	// float ak8963_ASA[3] {};

	// for (int i = 0; i < 3; i++) {
	// 	if (0 != response[i] && 0xff != response[i]) {
	// 		ak8963_ASA[i] = ((float)(response[i] - 128) / 256.0f) + 1.0f;

	// 	} else {
	// 		return false;
	// 	}
	// }

	// _px4_mag.set_sensitivity(ak8963_ASA[0], ak8963_ASA[1], ak8963_ASA[2]);


	// in 16-bit sampling mode the mag resolution is 1.5 milli Gauss per bit */
	_px4_mag.set_scale(1.5e-3f);

	return success;
}

bool ICM20948_AK09916::RegisterCheck(const register_config_t &reg_cfg, bool notify)
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

	if (!success) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);

		if (notify) {
			perf_count(_bad_register_perf);
			_px4_mag.increase_error_count();
		}
	}

	return success;
}

uint8_t ICM20948_AK09916::RegisterRead(Register reg)
{
	// TODO: use slave 4 and check register
	_icm20948.I2CSlaveRegisterStartRead(I2C_ADDRESS_DEFAULT, static_cast<uint8_t>(reg));
	usleep(1000);

	uint8_t buffer{};
	_icm20948.I2CSlaveExternalSensorDataRead(&buffer, 1);

	return buffer;
}

void ICM20948_AK09916::RegisterWrite(Register reg, uint8_t value)
{
	return _icm20948.I2CSlaveRegisterWrite(I2C_ADDRESS_DEFAULT, static_cast<uint8_t>(reg), value);
}

void ICM20948_AK09916::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);
	uint8_t val = orig_val;

	if (setbits) {
		val |= setbits;
	}

	if (clearbits) {
		val &= ~clearbits;
	}

	RegisterWrite(reg, val);
}

} // namespace AKM_AK09916
