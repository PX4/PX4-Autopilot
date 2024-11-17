/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#pragma once

#include "spa06.h"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/perf/perf_counter.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_baro.h>

class SPA06 : public I2CSPIDriver<SPA06>
{
public:
	SPA06(const I2CSPIDriverConfig &config, spa06::ISPA06 *interface);
	virtual ~SPA06();

	static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);
	static void print_usage();

	int			init();
	void			print_status();

	void			RunImpl();
private:
	void			Start();
	// float scale_factor(int oversampling_rate);

	int			collect(); //get results and publish
	int			calibrate();

	uORB::PublicationMulti<sensor_baro_s> _sensor_baro_pub{ORB_ID(sensor_baro)};

	spa06::ISPA06		*_interface;
	spa06::data_s		_data;
	spa06::calibration_s	_cal{};

	// set config, recommended settings
	//
	// oversampling rate  : single | 2       | 4       | 8       | 16     | 32     | 64      | 128
	// scale factor(KP/KT): 524288 | 1572864 | 3670016 | 7864320 | 253952 | 516096 | 1040384 | 2088960

	// configuration of pressure measurement rate (PM_RATE) and resolution (PM_PRC)
	//
	// bit[7]: reserved
	//
	// PM_RATE[6:4]    : 0 | 1 | 2 | 3 | 4  | 5  | 6  | 7
	// measurement rate: 1 | 2 | 4 | 8 | 16 | 32 | 64 | 128
	// note: applicable for measurements in background mode only
	//
	// PM_PRC[3:0]         : 0      | 1   | 2   | 3    | 4    | 5    | 6     | 7
	// oversampling (times): single | 2   | 4   | 8    | 16   | 32   | 64    | 128
	// measurement time(ms): 3.6    | 5.2 | 8.4 | 14.8 | 27.6 | 53.2 | 104.4 | 206.8
	// precision(PaRMS)    : 5.0    |     | 2.5 |      | 1.2  | 0.9  | 0.5   |
	// note: use in combination with a bit shift when the oversampling rate is > 8 times. see CFG_REG(0x19) register
	static constexpr uint8_t	_curr_prs_cfg{4 << 4 | 4};

	// configuration of temperature measurment rate (TMP_RATE) and resolution (TMP_PRC)
	//
	// temperature measurement: internal sensor (in ASIC) | external sensor (in pressure sensor MEMS element)
	// TMP_EXT[7]             : 0                         | 1
	// note: it is highly recommended to use the same temperature sensor as the source of the calibration coefficients wihch can be read from reg 0x28
	//
	// TMP_RATE[6:4]   : 0 | 1 | 2 | 3 | 4  | 5  | 6  | 7
	// measurement rate: 1 | 2 | 4 | 8 | 16 | 32 | 64 | 128
	// note: applicable for measurements in background mode only
	//
	// bit[3]: reserved
	//
	// TMP_PRC[2:0]        : 0      | 1 | 2 | 3 | 4  | 5  | 6  | 7
	// oversampling (times): single | 2 | 4 | 8 | 16 | 32 | 64 | 128
	// note: single(default) measurement time 3.6ms, other settings are optional, and may not be relevant
	// note: use in combination with a bit shift when the oversampling rate is > 8 times. see CFG_REG(0x19) register
	static constexpr uint8_t	_curr_tmp_cfg{1 << 7 | 4 << 4 | 0};

	bool			_collect_phase{false};
	float kp;
	float kt;

	perf_counter_t		_sample_perf;
	perf_counter_t		_measure_perf;
	perf_counter_t		_comms_errors;

	static constexpr uint32_t _sample_rate{16};
	static constexpr uint32_t _measure_interval{1000000 / _sample_rate / 2};
};
