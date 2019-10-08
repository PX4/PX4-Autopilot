/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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
 * @file bmp388.h
 *
 * Shared defines for the bmp388 driver.
 */
#pragma once

#include <math.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_hrt.h>
#include <lib/cdev/CDev.hpp>
#include <perf/perf_counter.h>
//TODO #include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/barometer/PX4Barometer.hpp>

#include "board_config.h"
#include "bmp3_defs.h"

// From https://github.com/BoschSensortec/BMP3-Sensor-API/blob/master/self-test/bmp3_selftest.c
#define BMP3_POST_SLEEP_WAIT_TIME         5000
#define BMP3_POST_RESET_WAIT_TIME         2000
#define BMP3_POST_INIT_WAIT_TIME          40000
#define BMP3_TRIM_CRC_DATA_ADDR           0x30
#define BPM3_CMD_SOFT_RESET               0xB6
#define BMP3_ODR_ADDR                     0x1D
#define BMP3_IIR_ADDR                     0x1F

// https://github.com/BoschSensortec/BMP3-Sensor-API/blob/master/bmp3.c
/*! Power control settings */
#define POWER_CNTL            (0x0006)
/*! Odr and filter settings */
#define ODR_FILTER            (0x00F0)
/*! Interrupt control settings */
#define INT_CTRL              (0x0708)
/*! Advance settings */
#define ADV_SETT              (0x1800)

#pragma pack(push,1)
struct calibration_s {
	uint16_t par_t1;
	uint16_t par_t2;
	int8_t   par_t3;

	int16_t  par_p1;
	int16_t  par_p2;
	int8_t   par_p3;
	int8_t   par_p4;
	uint16_t par_p5;
	uint16_t par_p6;
	int8_t   par_p7;
	int8_t   par_p8;
	int16_t  par_p9;
	int8_t   par_p10;
	int8_t   par_p11;

}; //calibration data

struct data_s {
	uint8_t p_msb;
	uint8_t p_lsb;
	uint8_t p_xlsb;

	uint8_t t_msb;
	uint8_t t_lsb;
	uint8_t t_xlsb;
}; // data
#pragma pack(pop)

struct fcalibration_s {
	float t1;
	float t2;
	float t3;

	float p1;
	float p2;
	float p3;
	float p4;
	float p5;
	float p6;
	float p7;
	float p8;
	float p9;
};

/*
 * BMP388 internal constants and data structures.
 */


class IBMP388
{
public:
	virtual ~IBMP388() = default;

	virtual bool is_external() = 0;
	virtual int init() = 0;

	// read reg value
	virtual uint8_t get_reg(uint8_t addr) = 0;

	// bulk read reg value
	virtual int get_reg_buf(uint8_t addr, uint8_t *buf, uint8_t len) = 0;

	// write reg value
	virtual int set_reg(uint8_t value, uint8_t addr) = 0;

	// bulk read of data into buffer, return same pointer
	virtual data_s *get_data(uint8_t addr) = 0;

	// bulk read of calibration data into buffer, return same pointer
	virtual calibration_s *get_calibration(uint8_t addr) = 0;

	virtual uint32_t get_device_id() const = 0;

};

class BMP388 : public cdev::CDev, public px4::ScheduledWorkItem
{
public:
	BMP388(IBMP388 *interface, const char *path);
	virtual ~BMP388();

	virtual int		init();

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

private:
	PX4Barometer		_px4_baro;
	IBMP388			*_interface;

	unsigned		_measure_interval{0}; // interval in microseconds needed to measure
	uint8_t			_osr_t;               // oversampling rate, temperature
	uint8_t			_osr_p;               // oversampling rate, pressure
	uint8_t			_odr;                 // output data rate
	uint8_t			_iir_coef;            // IIR coefficient

	perf_counter_t		_sample_perf;
	perf_counter_t		_measure_perf;
	perf_counter_t		_comms_errors;

	struct calibration_s 	*_cal; // stored calibration constants

	bool			_collect_phase;

	void 			Run() override;
	void 			start();
	void 			stop();
	int 			measure();
	int			collect(); //get results and publish
	uint32_t		get_measurement_time(uint8_t osr_t, uint8_t osr_p);

	bool			soft_reset();
	bool			get_calib_data();
	bool			validate_trimming_param();
	bool 			set_sensor_settings();
	bool			set_op_mode(uint8_t op_mode);

	bool 			get_sensor_data(uint8_t sensor_comp, struct bmp3_data *comp_data);
	bool			compensate_data(uint8_t sensor_comp, const struct bmp3_uncomp_data *uncomp_data, struct bmp3_data *comp_data);
};


/* interface factories */
extern IBMP388 *bmp388_spi_interface(uint8_t busnum, uint32_t device, bool external);
extern IBMP388 *bmp388_i2c_interface(uint8_t busnum, uint32_t device, bool external);
typedef IBMP388 *(*BMP388_constructor)(uint8_t, uint32_t, bool);
