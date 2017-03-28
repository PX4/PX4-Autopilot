/****************************************************************************
 *
 *   Copyright (C) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file bmp285.h
 *
 * Shared defines for the bmp285 driver.
 */

#ifndef BMP285_HPP_
#define BMP285_HPP_

#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <getopt.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <arch/board/board.h>
#include <board_config.h>


#include <drivers/device/device.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/i2c.h>
#include <systemlib/perf_counter.h>
#include <systemlib/err.h>


#pragma once


#define BMP285_DEVICE_PATH_PRESSURE         "/dev/bmp285_i2c_int"

#define BMP285_DEVICE_PATH_PRESSURE_EXT     "/dev/bmp285_i2c_ext"

#define BMP285_SLAVE_ADDRESS                 PX4_I2C_OBDEV_BMP285

#define BMP285_BUS_SPEED                     1000*100

#define BPM285_ADDR_CAL		0x88	/* address of 12x 2 bytes calibration data */
#define BPM285_ADDR_DATA	0xF7	/* address of 2x 3 bytes p-t data */

#define BPM285_ADDR_CONFIG	0xF5	/* configuration */
#define BPM285_ADDR_CTRL	0xF4	/* controll */
#define BPM285_ADDR_STATUS	0xF3	/* state */
#define BPM285_ADDR_RESET	0xE0	/* reset */
#define BPM285_ADDR_ID		0xD0	/* id */

#define BPM285_VALUE_ID		0x58	/* chip id */
#define BPM285_VALUE_RESET	0xB6	/* reset */

#define BPM285_STATUS_MEASURING	1<<3	/* if in process of measure */
#define BPM285_STATUS_COPING	1<<0	/* if in process of data copy */

#define BPM285_CTRL_P0		0x0<<2		/* no p measure */
#define BPM285_CTRL_P1		0x1<<2
#define BPM285_CTRL_P2		0x2<<2
#define BPM285_CTRL_P4		0x3<<2
#define BPM285_CTRL_P8		0x4<<2
#define BPM285_CTRL_P16		0x5<<2

#define BPM285_CTRL_T0		0x0<<5		/* no t measure */
#define BPM285_CTRL_T1		0x1<<5
#define BPM285_CTRL_T2		0x2<<5
#define BPM285_CTRL_T4		0x3<<5
#define BPM285_CTRL_T8		0x4<<5
#define BPM285_CTRL_T16		0x5<<5

#define BPM285_CONFIG_F0		0x0<<2		/* no filter */
#define BPM285_CONFIG_F2		0x1<<2
#define BPM285_CONFIG_F4		0x2<<2
#define BPM285_CONFIG_F8		0x3<<2
#define BPM285_CONFIG_F16		0x4<<2

#define BMP285_CTRL_T_SB0       0x0<<5
#define BMP285_CTRL_T_SB1       0x1<<5
#define BMP285_CTRL_T_SB2       0x2<<5
#define BMP285_CTRL_T_SB3       0x3<<5
#define BMP285_CTRL_T_SB4       0x4<<5
#define BMP285_CTRL_T_SB5       0x5<<5
#define BMP285_CTRL_T_SB6       0x6<<5
#define BMP285_CTRL_T_SB7       0x7<<5


#define BPM285_CTRL_MODE_SLEEP	0x0
#define BPM285_CTRL_MODE_FORCE	0x1		/* on demand, goes to sleep after */
#define BPM285_CTRL_MODE_NORMAL	0x3

#define BPM285_MT_INIT		6400	/* max measure time of initial p + t in us */
#define BPM285_MT			2300	/* max measure time of p or t in us */



namespace bmp285
{

#pragma pack(push,1)
struct calibration_s {
	uint16_t t1;
	int16_t t2;
	int16_t t3;

	uint16_t p1;
	int16_t p2;
	int16_t p3;
	int16_t p4;
	int16_t p5;
	int16_t p6;
	int16_t p7;
	int16_t p8;
	int16_t p9;
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

} /* namespace */


/*
 * BMP285 internal constants and data structures.
 */



class BMP285 : public device::I2C
{
public:
	BMP285(int bus, uint16_t address, const char *path, bool external);
	~BMP285();

	bool is_external();
	virtual int     init();

	virtual ssize_t read(struct file *filp, char *buffer, size_t buflen);
	virtual int     ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void            print_info();
	bmp285::data_s *get_data(uint8_t reg);
	bmp285::calibration_s *get_calibration(uint8_t addr);

private:
	uint8_t             _curr_ctrl;
	struct work_s       _work;
	bool _external;

	unsigned            _report_ticks; // 0 - no cycling, otherwise period of sending a report
	unsigned            _max_mesure_ticks; //ticks needed to measure

	ringbuffer::RingBuffer  *_reports;

	bool            _collect_phase;

	/* altitude conversion calibration */
	unsigned        _msl_pressure;  /* in Pa */

	orb_advert_t        _baro_topic;
	int                 _orb_class_instance;
	int                 _class_instance;

	perf_counter_t      _sample_perf;
	perf_counter_t      _measure_perf;
	perf_counter_t      _comms_errors;
	perf_counter_t      _buffer_overflows;

	struct bmp285::calibration_s _cal;
	struct bmp285::data_s _data;

	struct bmp285::fcalibration_s _fcal; //pre processed calibration constants

	float           _P; /* in Pa */
	float           _T; /* in K */


	/* periodic execution helpers */
	void            start_cycle();
	void            stop_cycle();
	void            cycle(); //main execution
	static void     cycle_trampoline(void *arg);
	/**
	 * Read a register from the BMP285
	 *
	 * @param       The register to read.
	 * @return      The value that was read.
	 */
	uint8_t         read_reg(unsigned reg);

	/**
	 * Write a register in the BMP285
	 *
	 * @param reg       The register to write.
	 * @param value     The new value to write.
	 * @return          The value returned after transfer of data.
	 */
	int            write_reg(unsigned reg, uint8_t value);

	int     measure(); //start measure
	int     collect(); //get results and publish

};

#endif

