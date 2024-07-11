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

/**
 * @file BMM350.hpp
 *
 * Driver for the Bosch BMM350 connected via I2C.
 *
 */

#pragma once

#include "Bosch_BMM350_registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/device/i2c.h>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module_params.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/SubscriptionInterval.hpp>

using namespace Bosch_BMM350;
using namespace time_literals;

class BMM350 : public device::I2C, public I2CSPIDriver<BMM350>, public ModuleParams
{
public:
	BMM350(const I2CSPIDriverConfig &config);
	~BMM350() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	struct mag_temp_data {
		float x;
		float y;
		float z;
		float temp;
	};

	struct raw_mag_data {
		int32_t raw_x;
		int32_t raw_y;
		int32_t raw_z;
		int32_t raw_t;
	};

	struct register_config_t {
		Register reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	struct dut_offset_coef {
		float t_offs;
		float offset_x;
		float offset_y;
		float offset_z;
	};
	struct dut_sensit_coef {
		float t_sens;
		float sens_x;
		float sens_y;
		float sens_z;
	};

	struct dut_tco {
		float tco_x;
		float tco_y;
		float tco_z;
	};

	struct dut_tcs {
		float tcs_x;
		float tcs_y;
		float tcs_z;
	};

	struct cross_axis {
		float cross_x_y;
		float cross_y_x;
		float cross_z_x;
		float cross_z_y;
	};

	struct mag_compensate_vals {
		struct dut_offset_coef dut_offset_coef;
		struct dut_sensit_coef dut_sensit_coef;
		struct dut_tco dut_tco;
		struct dut_tcs dut_tcs;
		float dut_t0;
		struct cross_axis cross_axis;
	};

	int probe() override;
	bool Reset();
	int Configure();

	int RegisterRead(Register reg, uint8_t *value);
	int RegisterWrite(Register reg, uint8_t value);

	int8_t CompensateAxisAndTemp();
	int ReadOutRawData(float *out_data);
	int ReadOTPWord(uint8_t addr, uint16_t *lsb_msb);
	int32_t FixSign(uint32_t inval, int8_t num_bits);

	int UpdateMagOffsets();
	void ParametersUpdate(bool force = false);
	void UpdateMagParams();
	uint8_t GetODR(int value);
	hrt_abstime OdrToUs(uint8_t value);
	uint8_t GetAVG(int value);

	PX4Magnetometer _px4_mag;

	perf_counter_t _reset_perf{perf_alloc(PC_COUNT, MODULE_NAME ": reset")};
	perf_counter_t _bad_read_perf{perf_alloc(PC_COUNT, MODULE_NAME ": bad read")};
	perf_counter_t _self_test_failed_perf{perf_alloc(PC_COUNT, MODULE_NAME ": self test failed")};

	hrt_abstime _reset_timestamp{0};
	hrt_abstime _last_config_check_timestamp{0};

	mag_compensate_vals _mag_comp_vals{0};

	float _initial_self_test_values[4];

	uint8_t _mag_odr_mode = ODR_200HZ;
	uint8_t _mag_avg_mode = AVG_2;
	uint8_t _mag_pad_drive = 7;


	static constexpr float BXY_SENS = 14.55f;
	static constexpr float BZ_SENS = 9.0f;
	static constexpr float TEMP_SENS = 0.00204f;
	static constexpr float INA_XY_GAIN_TRT = 19.46f;
	static constexpr float INA_Z_GAIN_TRT = 31.0f;
	static constexpr float ADC_GAIN = 1 / 1.5f;
	static constexpr float LUT_GAIN = 0.714607238769531f;
	static constexpr float POWER = 1000000.0f / 1048576.0f;
	float lsb_to_utc_degc[4] = {
		(POWER / (BXY_SENS *INA_XY_GAIN_TRT *ADC_GAIN * LUT_GAIN)),
		(POWER / (BXY_SENS *INA_XY_GAIN_TRT *ADC_GAIN * LUT_GAIN)),
		(POWER / (BZ_SENS *INA_Z_GAIN_TRT *ADC_GAIN * LUT_GAIN)),
		1 / (TEMP_SENS *ADC_GAIN *LUT_GAIN * 1048576)
	};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		FGR,
		BR,
		AFTER_RESET,
		MEASURE_FORCED,
		SELF_TEST_CHECK,
		CONFIGURE,
		READ,
	} _state{STATE::RESET};

	enum class SELF_TEST_STATE : uint8_t {
		INIT,
		POS_X,
		NEG_X,
		POS_Y,
		NEG_Y
	} _self_test_state{SELF_TEST_STATE::INIT};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::BMM350_ODR>) _param_bmm350_odr,
		(ParamInt<px4::params::BMM350_AVG>) _param_bmm350_avg,
		(ParamInt<px4::params::BMM350_DRIVE>) _param_bmm350_drive
	)
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
};
