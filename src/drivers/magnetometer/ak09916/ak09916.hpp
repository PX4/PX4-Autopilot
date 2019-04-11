/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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


#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <px4_log.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>
#include <nuttx/wqueue.h>
#include <systemlib/conversions.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>
#include <drivers/device/i2c.h>
#include <drivers/drv_mag.h>


#define AK09916_SAMPLE_RATE					100
#define AK09916_DEVICE_PATH_MAG              "/dev/ak09916_i2c_int"

#define AK09916_DEVICE_PATH_MAG_EXT          "/dev/ak09916_i2c_ext"

/* in 16-bit sampling mode the mag resolution is 1.5 milli Gauss per bit */

#define AK09916_MAG_RANGE_GA        1.5e-3f;

/* ak09916 deviating register addresses and bit definitions */
#define AK09916_I2C_ADDR         0x0C

#define AK09916_DEVICE_ID_A		0x48
#define AK09916_DEVICE_ID_B		0x09	// additional ID byte ("INFO" on AK9063 without content specification.)

#define AK09916REG_WIA           0x00

#define AK09916REG_HXL        0x11
#define AK09916REG_HXH        0x12
#define AK09916REG_HYL        0x13
#define AK09916REG_HYH        0x14
#define AK09916REG_HZL        0x15
#define AK09916REG_HZH        0x16
#define AK09916REG_ST1        0x10
#define AK09916REG_ST2        0x18
#define AK09916REG_CNTL2          0x31
#define AK09916REG_CNTL3          0x32


#define AK09916_CNTL2_POWERDOWN_MODE            0x00
#define AK09916_RESET							0x01
#define AK09916_CNTL2_SINGLE_MODE               0x01 /* default */
#define AK09916_CNTL2_CONTINOUS_MODE_10HZ       0x02
#define AK09916_CNTL2_CONTINOUS_MODE_20HZ       0x04
#define AK09916_CNTL2_CONTINOUS_MODE_50HZ       0x06
#define AK09916_CNTL2_CONTINOUS_MODE_100HZ      0x08
#define AK09916_CNTL2_SELFTEST_MODE             0x10
#define AK09916_CNTL3_SRST                      0x01
#define AK09916_ST1_DRDY                        0x01
#define AK09916_ST1_DOR                         0x02


#pragma pack(push, 1)
struct ak09916_regs {
	uint8_t st1;
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t tmps;
	uint8_t st2;
};
#pragma pack(pop)



/**
 * Helper class implementing the magnetometer driver node.
 */
class AK09916 : public device::I2C
{
public:
	AK09916(int bus, const char *path, enum Rotation rotation);
	~AK09916();

	virtual int ioctl(struct file *filp, int cmd, unsigned long arg);
	virtual int init();
	virtual ssize_t       read(struct file *filp, char *buffer, size_t buflen);

	void read_block(uint8_t reg, uint8_t *val, uint8_t count);

	int reset(void);
	int setup(void);
	void print_info(void);
	int setup_master_i2c(void);
	bool check_id(uint8_t &id);


	static void cycle_trampoline(void *arg);
	void start(void);
	void stop(void);
	void cycle(void);

protected:

	friend class ICM20948;

	/* Directly measure from the _interface if possible */
	void measure();

	/* Update the state with prefetched data (internally called by the regular measure() )*/
	void _measure(struct ak09916_regs data);

	uint8_t read_reg(uint8_t reg);
	void write_reg(uint8_t reg, uint8_t value);

private:
	work_s			_work{};
	unsigned		_measure_ticks;

	enum Rotation _rotation;
	orb_advert_t _mag_topic;
	int _mag_orb_class_instance;
	int _mag_class_instance;
	bool _mag_reading_data;
	ringbuffer::RingBuffer *_mag_reports;
	mag_report   _last_report {};          /**< used for info() */
	struct mag_calibration_s _mag_scale;
	float _mag_range_scale;
	perf_counter_t _mag_reads;
	perf_counter_t _mag_errors;
	perf_counter_t _mag_overruns;
	perf_counter_t _mag_overflows;
	perf_counter_t _mag_duplicates;
	float _mag_asa_x;
	float _mag_asa_y;
	float _mag_asa_z;

	bool check_duplicate(uint8_t *mag_data);

	// keep last mag reading for duplicate detection
	uint8_t			_last_mag_data[6];

	/* do not allow to copy this class due to pointer data members */
	AK09916(const AK09916 &);
	AK09916 operator=(const AK09916 &);
};
