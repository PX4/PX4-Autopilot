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
#include <string.h>

#include <drivers/drv_baro.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/i2c.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/spi.h>
#include <lib/cdev/CDev.hpp>
#include <perf/perf_counter.h>
#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_work_queue/ScheduledWorkItem.hpp>

#include "board_config.h"
#include "bmp3_defs.h"

// From https://github.com/BoschSensortec/BMP3-Sensor-API/blob/master/self-test/bmp3_selftest.c
#define BMP3_POST_SLEEP_WAIT_TIME         5000
#define BMP3_POST_RESET_WAIT_TIME         2000
#define BMP3_POST_INIT_WAIT_TIME          40000
#define BMP3_TRIM_CRC_DATA_ADDR           0x30
#define BPM3_CMD_SOFT_RESET               0xB6

// https://github.com/BoschSensortec/BMP3-Sensor-API/blob/master/bmp3.c
/*! Power control settings */
#define POWER_CNTL            (0x0006)
/*! Odr and filter settings */
#define ODR_FILTER            (0x00F0)
/*! Interrupt control settings */
#define INT_CTRL              (0x0708)
/*! Advance settings */
#define ADV_SETT              (0x1800)

namespace bmp388
{

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
	virtual bmp388::data_s *get_data(uint8_t addr) = 0;

	// bulk read of calibration data into buffer, return same pointer
	virtual bmp388::calibration_s *get_calibration(uint8_t addr) = 0;

	virtual uint32_t get_device_id() const = 0;

};

} /* namespace */


/* interface factories */
extern bmp388::IBMP388 *bmp388_spi_interface(uint8_t busnum, uint32_t device, bool external);
extern bmp388::IBMP388 *bmp388_i2c_interface(uint8_t busnum, uint32_t device, bool external);
typedef bmp388::IBMP388 *(*BMP388_constructor)(uint8_t, uint32_t, bool);
