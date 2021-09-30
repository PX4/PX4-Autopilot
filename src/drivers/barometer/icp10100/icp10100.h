/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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
 * @file icp10100.h
 *
 * Shared defines for the icp10100 driver.
 */
#pragma once

#include <math.h>
#include <drivers/drv_hrt.h>
#include <lib/cdev/CDev.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/barometer/PX4Barometer.hpp>

#include "board_config.h"

#define ICP10xxx_CHIP_ID 0x80
#define ICP_I2C_ID 0x63

#define ICP_CMD_READ_ID 0xefc8
#define ICP_CMD_SET_ADDR 0xc595
#define ICP_CMD_READ_OTP 0xc7f7
#define ICP_CMD_MEAS_LP 0x609c
#define ICP_CMD_MEAS_N 0x6825
#define ICP_CMD_MEAS_LN 0x70df
#define ICP_CMD_MEAS_ULN 0x7866
#define ICP_CMD_SOFT_RESET 0x805d

/*
 * ICP10100 internal constants and data structures.
 */
class IICP10100
{
public:
	virtual ~IICP10100() = default;

	virtual int init() = 0;

	virtual int read_measurement_results(uint8_t *buf, uint8_t len) = 0;
	virtual int read_response(uint16_t cmd, uint8_t *buf, uint8_t len) = 0;
	virtual int send_command(uint16_t cmd) = 0;
	virtual int send_command(uint16_t cmd, uint8_t *data, uint8_t len) = 0;

	virtual uint32_t get_device_id() const = 0;

	virtual uint8_t get_device_address() const = 0;
};

class ICP10100 : public I2CSPIDriver<ICP10100>
{
public:
	ICP10100(const I2CSPIDriverConfig &config, IICP10100 *interface);
	virtual ~ICP10100();

	static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);
	static void print_usage();

	virtual int		init();

	void			print_status();

	void 			RunImpl();
private:
	PX4Barometer		_px4_baro;
	IICP10100		*_interface{nullptr};

	unsigned		_measure_interval{0};			// interval in microseconds needed to measure

	perf_counter_t		_sample_perf;
	perf_counter_t		_measure_perf;
	perf_counter_t		_comms_errors;

	bool			_collect_phase{false};

	void 			start();
	int 			measure();
	int			collect(); //get results and publish

	bool 			set_sensor_settings();

	bool 			get_sensor_data(uint8_t *comp_data);

	enum mmode { FAST, NORMAL, ACCURATE, VERY_ACCURATE };
	bool get_calibration();
	bool is_connected();
	uint32_t get_measurement_time(ICP10100::mmode mode = ICP10100::NORMAL);
	void _calculate(void);
	bool dataReady();
	float getTemperatureC(void);
	float getTemperatureF(void);
	float getPressurePa(void);
	short _scal[4];
	uint16_t _raw_t;
	uint32_t _raw_p;
	float _temperature_C;
	float _pressure_Pa;
	unsigned long _meas_start;
	uint8_t _meas_duration;
	bool _data_ready;
};


/* interface factories */
extern IICP10100 *icp10100_i2c_interface(uint8_t busnum, uint32_t device, int bus_frequency);
