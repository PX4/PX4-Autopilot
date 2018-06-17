/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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

#include "drivers/device/ringbuffer.h"	// ringbuffer::RingBuffer
#include "drivers/drv_mag.h"		// mag_calibration_s
#include <perf/perf_counter.h>

/* in 16-bit sampling mode the mag resolution is 1.5 milli Gauss per bit */

#define MPU9250_MAG_RANGE_GA        1.5e-3f;

/* we are using the continuous fixed sampling rate of 100Hz */

#define MPU9250_AK8963_SAMPLE_RATE 100

/* ak8963 register address and bit definitions */

#define AK8963_I2C_ADDR         0x0C
#define AK8963_DEVICE_ID        0x48

#define AK8963REG_WIA           0x00
#define AK8963REG_ST1           0x02
#define AK8963REG_HXL           0x03
#define AK8963REG_ASAX          0x10
#define AK8963REG_CNTL1         0x0A
#define AK8963REG_CNTL2         0x0B

#define AK8963_SINGLE_MEAS_MODE 0x01
#define AK8963_CONTINUOUS_MODE1 0x02
#define AK8963_CONTINUOUS_MODE2 0x06
#define AK8963_POWERDOWN_MODE   0x00
#define AK8963_SELFTEST_MODE    0x08
#define AK8963_FUZE_MODE        0x0F
#define AK8963_16BIT_ADC        0x10
#define AK8963_14BIT_ADC        0x00
#define AK8963_RESET            0x01



class MPU9250;

#pragma pack(push, 1)
struct ak8963_regs {
	uint8_t st1;
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t st2;
};
#pragma pack(pop)

extern device::Device *AK8963_I2C_interface(int bus, bool external_bus);

typedef device::Device *(*MPU9250_mag_constructor)(int, bool);


/**
 * Helper class implementing the magnetometer driver node.
 */
class MPU9250_mag : public device::CDev
{
public:
	MPU9250_mag(MPU9250 *parent, device::Device *interface, const char *path);
	~MPU9250_mag();

	virtual ssize_t read(struct file *filp, char *buffer, size_t buflen);
	virtual int ioctl(struct file *filp, int cmd, unsigned long arg);
	virtual int init();

	void set_passthrough(uint8_t reg, uint8_t size, uint8_t *out = NULL);
	void passthrough_read(uint8_t reg, uint8_t *buf, uint8_t size);
	void passthrough_write(uint8_t reg, uint8_t val);
	void read_block(uint8_t reg, uint8_t *val, uint8_t count);

	int ak8963_reset(void);
	int ak8963_setup(void);
	int ak8963_setup_master_i2c(void);
	bool ak8963_check_id(uint8_t &id);
	bool ak8963_read_adjustments(void);

protected:
	Device			*_interface;

	friend class MPU9250;

	/* Directly measure from the _interface if possible */
	void measure();

	/* Update the state with prefetched data (internally called by the regular measure() )*/
	void _measure(struct ak8963_regs data);


	uint8_t read_reg(unsigned reg);
	void write_reg(unsigned reg, uint8_t value);


	bool is_passthrough() { return _interface == nullptr; }

	int self_test(void);

private:
	MPU9250 *_parent;
	orb_advert_t _mag_topic;
	int _mag_orb_class_instance;
	int _mag_class_instance;
	bool _mag_reading_data;
	ringbuffer::RingBuffer *_mag_reports;
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
	MPU9250_mag(const MPU9250_mag &);
	MPU9250_mag operator=(const MPU9250_mag &);
};
