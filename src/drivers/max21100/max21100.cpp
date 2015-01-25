/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file max21100.cpp
 *
 * Driver for the Maxim MAX21100 connected via SPI.
 *
 * @author Matthew Coleman
 */

#include <nuttx/config.h>

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
#include <getopt.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/conversions.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/spi.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>

#include "max21100.h"

#define DIR_READ			0x80
#define DIR_WRITE			0x00

#define MAX21100_DEVICE_PATH_ACCEL		"/dev/max21100_accel"
#define MAX21100_DEVICE_PATH_GYRO		"/dev/max21100_gyro"



#define MAX21100_ONE_G					9.80665f

#define MAX21100_BUS_SPEED				10*1000*1000 /* will be rounded */

class MAX21100_gyro;

class MAX21100 : public device::SPI
{
public:
	MAX21100(int bus, const char *path_accel, const char *path_gyro, spi_dev_e device, enum Rotation rotation);
	virtual ~MAX21100();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

	void			print_registers();

	/**
	 * Test behaviour against factory offsets
	 *
	 * @return 0 on success, 1 on failure
	 */
	int 			factory_self_test();

	// deliberately cause a sensor error
	void 			test_error();

protected:
	virtual int		probe();

	friend class MAX21100_gyro;

	virtual ssize_t		gyro_read(struct file *filp, char *buffer, size_t buflen);
	virtual int		gyro_ioctl(struct file *filp, int cmd, unsigned long arg);

private:
	MAX21100_gyro		*_gyro;
	uint8_t			_product;	/** product code */

	struct hrt_call		_call;
	unsigned		_call_interval;

	RingBuffer		*_accel_reports;

	struct accel_scale	_accel_scale;
	float			_accel_range_scale;
	float			_accel_range_m_s2;
	orb_advert_t		_accel_topic;
	orb_id_t		_accel_orb_id;
	int			_accel_class_instance;

	RingBuffer		*_gyro_reports;

	struct gyro_scale	_gyro_scale;
	float			_gyro_range_scale;
	float			_gyro_range_rad_s;

	unsigned		_sample_rate;
	perf_counter_t		_accel_reads;
	perf_counter_t		_gyro_reads;
	perf_counter_t		_sample_perf;
	perf_counter_t		_bad_transfers;
	perf_counter_t		_bad_registers;
	perf_counter_t		_good_transfers;
	perf_counter_t		_reset_retries;
	perf_counter_t		_system_latency_perf;
	perf_counter_t		_controller_latency_perf;

	uint8_t			_register_wait;
	uint64_t		_reset_wait;

	math::LowPassFilter2p	_accel_filter_x;
	math::LowPassFilter2p	_accel_filter_y;
	math::LowPassFilter2p	_accel_filter_z;
	math::LowPassFilter2p	_gyro_filter_x;
	math::LowPassFilter2p	_gyro_filter_y;
	math::LowPassFilter2p	_gyro_filter_z;

	enum Rotation		_rotation;

	// this is used to support runtime checking of key
	// configuration registers to detect SPI bus errors and sensor
	// reset
#define MAX21100_NUM_CHECKED_REGISTERS 9
	static const uint8_t	_checked_registers[MAX21100_NUM_CHECKED_REGISTERS];
	uint8_t			_checked_values[MAX21100_NUM_CHECKED_REGISTERS];
	uint8_t			_checked_next;

	// use this to avoid processing measurements when in factory
	// self test
	volatile bool		_in_factory_test;

	/**
	 * Start automatic measurement.
	 */
	void			start();

	/**
	 * Stop automatic measurement.
	 */
	void			stop();

	/**
	 * Reset chip.
	 *
	 * Resets the chip and measurements ranges, but not scale and offset.
	 */
	int			reset();

	/**
	 * Static trampoline from the hrt_call context; because we don't have a
	 * generic hrt wrapper yet.
	 *
	 * Called by the HRT in interrupt context at the specified rate if
	 * automatic polling is enabled.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		measure_trampoline(void *arg);

	/**
	 * Fetch measurements from the sensor and update the report buffers.
	 */
	void			measure();

	/**
	 * Read a register
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint8_t			read_reg(unsigned reg, uint32_t speed=MAX21100_LOW_BUS_SPEED);
	uint16_t		read_reg16(unsigned reg);

	/**
	 * Write a register
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_reg(unsigned reg, uint8_t value);

	/**
	 * Modify a register
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Write a register, updating _checked_values
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_checked_reg(unsigned reg, uint8_t value);

	/**
	 * Set the measurement range.
	 *
	 * @param max_g		The maximum G value the range must support.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			set_range(unsigned max_g);

	/**
	 * Swap a 16-bit value read from the MAX21100 to native byte order.
	 */
	uint16_t		swap16(uint16_t val) { return (val >> 8) | (val << 8);	}

	/**
	 * Measurement self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	 int 			self_test();

	/**
	 * Accel self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int 			accel_self_test();

	/**
	 * Gyro self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	 int 			gyro_self_test();

	/*
	  set low pass filter frequency
	 */
	void _set_dlpf_filter(uint16_t frequency_hz);

	/*
	  set sample rate (approximate) - 1kHz to 5Hz
	*/
	void _set_sample_rate(uint16_t desired_sample_rate_hz);

	/*
	  check that key registers still have the right value
	 */
	void check_registers(void);

	/* do not allow to copy this class due to pointer data members */
	MAX21100(const MAX21100&);
	MAX21100 operator=(const MAX21100&);

#pragma pack(push, 1)
	/**
	 * Report conversation within the MAX21100, including command byte and
	 * interrupt status.
	 */
	struct MPUReport {
		uint8_t		cmd;
		uint8_t		status;
		uint8_t		accel_x[2];
		uint8_t		accel_y[2];
		uint8_t		accel_z[2];
		uint8_t		temp[2];
		uint8_t		gyro_x[2];
		uint8_t		gyro_y[2];
		uint8_t		gyro_z[2];
	};
#pragma pack(pop)
};

/*
  list of registers that will be checked in check_registers(). Note
  that MPUREG_PRODUCT_ID must be first in the list.
 */
const uint8_t MAX21100::_checked_registers[MAX21100_NUM_CHECKED_REGISTERS] = { MPUREG_PRODUCT_ID,
									     MPUREG_PWR_MGMT_1,
									     MPUREG_USER_CTRL,
									     MPUREG_SMPLRT_DIV,
                                                                             MPUREG_CONFIG,
									     MPUREG_GYRO_CONFIG,
									     MPUREG_ACCEL_CONFIG,
									     MPUREG_INT_ENABLE,
									     MPUREG_INT_PIN_CFG };

					   

/**
 * Helper class implementing the gyro driver node.
 */
class MAX21100_gyro : public device::CDev
{
public:MAX21100
	MAX21100_gyro(MAX21100 *parent, const char *path);
	~MAX21100_gyro();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	virtual int		init();

protected:
	friend class MAX21100;

	void			parent_poll_notify();

private:
	MAX21100			*_parent;
	orb_advert_t		_gyro_topic;
	orb_id_t		_gyro_orb_id;
	int			_gyro_class_instance;

	/* do not allow to copy this class due to pointer data members */
	MAX21100_gyro(const MAX21100_gyro&);
	MAX21100_gyro operator=(const MAX21100_gyro&);
};

/** driver 'main' command */
extern "C" { __EXPORT int mpu6000_main(int argc, char *argv[]); }

MAX21100::MAX21100(int bus, const char *path_accel, const char *path_gyro, spi_dev_e device, enum Rotation rotation) :
	SPI("MAX21100", path_accel, bus, device, SPIDEV_MODE3, MAX21100_BUS_SPEED),
	_gyro(new MAX21100_gyro(this, path_gyro)),
	_product(0),
	_call{},
	_call_interval(0),
	_accel_reports(nullptr),
	_accel_scale{},
	_accel_range_scale(0.0f),
	_accel_range_m_s2(0.0f),
	_accel_topic(-1),
	_accel_orb_id(nullptr),
	_accel_class_instance(-1),
	_gyro_reports(nullptr),
	_gyro_scale{},
	_gyro_range_scale(0.0f),
	_gyro_range_rad_s(0.0f),
	_sample_rate(1000),
	_accel_reads(perf_alloc(PC_COUNT, "mpu6000_accel_read")),
	_gyro_reads(perf_alloc(PC_COUNT, "mpu6000_gyro_read")),
	_sample_perf(perf_alloc(PC_ELAPSED, "mpu6000_read")),
	_bad_transfers(perf_alloc(PC_COUNT, "mpu6000_bad_transfers")),
	_bad_registers(perf_alloc(PC_COUNT, "mpu6000_bad_registers")),
	_good_transfers(perf_alloc(PC_COUNT, "mpu6000_good_transfers")),
	_reset_retries(perf_alloc(PC_COUNT, "mpu6000_reset_retries")),
	_system_latency_perf(perf_alloc_once(PC_ELAPSED, "sys_latency")),
	_controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency")),
	_register_wait(0),
	_reset_wait(0),
	_accel_filter_x(MAX21100_ACCEL_DEFAULT_RATE, MAX21100_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_y(MAX21100_ACCEL_DEFAULT_RATE, MAX21100_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_z(MAX21100_ACCEL_DEFAULT_RATE, MAX21100_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_x(MAX21100_GYRO_DEFAULT_RATE, MAX21100_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_y(MAX21100_GYRO_DEFAULT_RATE, MAX21100_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_z(MAX21100_GYRO_DEFAULT_RATE, MAX21100_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_rotation(rotation),
	_checked_next(0),
	_in_factory_test(false)
{
	// disable debug() calls
	_debug_enabled = false;

	// default accel scale factors
	_accel_scale.x_offset = 0;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0;
	_accel_scale.z_scale  = 1.0f;

	// default gyro scale factors
	_gyro_scale.x_offset = 0;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0;
	_gyro_scale.z_scale  = 1.0f;

	memset(&_call, 0, sizeof(_call));
}

MAX21100::~MAX21100()
{
	/* make sure we are truly inactive */
	stop();

	/* delete the gyro subdriver */
	delete _gyro;

	/* free any existing reports */
	if (_accel_reports != nullptr)
		delete _accel_reports;
	if (_gyro_reports != nullptr)
		delete _gyro_reports;

	if (_accel_class_instance != -1)
		unregister_class_devname(ACCEL_DEVICE_PATH, _accel_class_instance);

	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_accel_reads);
	perf_free(_gyro_reads);
	perf_free(_bad_transfers);
	perf_free(_bad_registers);
	perf_free(_good_transfers);
}

int
MAX21100::init()
{
	int ret;

	/* do SPI init (and probe) first */
	ret = SPI::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		debug("SPI setup failed");
		return ret;
	}

	/* allocate basic report buffers */
	_accel_reports = new RingBuffer(2, sizeof(accel_report));
	if (_accel_reports == nullptr)
		goto out;

	_gyro_reports = new RingBuffer(2, sizeof(gyro_report));
	if (_gyro_reports == nullptr)
		goto out;

	if (reset() != OK)
		goto out;

	/* Initialize offsets and scales */
	_accel_scale.x_offset = 0;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0;
	_accel_scale.z_scale  = 1.0f;

	_gyro_scale.x_offset = 0;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0;
	_gyro_scale.z_scale  = 1.0f;

	/* do CDev init for the gyro device node, keep it optional */
	ret = _gyro->init();
	/* if probe/setup failed, bail now */
	if (ret != OK) {
		debug("gyro init failed");
		return ret;
	}

	_accel_class_instance = register_class_devname(ACCEL_DEVICE_PATH);

	measure();

	/* advertise sensor topic, measure manually to initialize valid report */
	struct accel_report arp;
	_accel_reports->get(&arp);

	/* measurement will have generated a report, publish */
	switch (_accel_class_instance) {
		case CLASS_DEVICE_PRIMARY:
			_accel_orb_id = ORB_ID(sensor_accel0);
			break;
		
		case CLASS_DEVICE_SECONDARY:
			_accel_orb_id = ORB_ID(sensor_accel1);
			break;

		case CLASS_DEVICE_TERTIARY:
			_accel_orb_id = ORB_ID(sensor_accel2);
			break;

	}

	_accel_topic = orb_advertise(_accel_orb_id, &arp);

	if (_accel_topic < 0) {
		warnx("ADVERT FAIL");
	}


	/* advertise sensor topic, measure manually to initialize valid report */
	struct gyro_report grp;
	_gyro_reports->get(&grp);

	switch (_gyro->_gyro_class_instance) {
		case CLASS_DEVICE_PRIMARY:
			_gyro->_gyro_orb_id = ORB_ID(sensor_gyro0);
			break;

		case CLASS_DEVICE_SECONDARY:
			_gyro->_gyro_orb_id = ORB_ID(sensor_gyro1);
			break;

		case CLASS_DEVICE_TERTIARY:
			_gyro->_gyro_orb_id = ORB_ID(sensor_gyro2);
			break;

	}

	_gyro->_gyro_topic = orb_advertise(_gyro->_gyro_orb_id, &grp);

	if (_gyro->_gyro_topic < 0) {
		warnx("ADVERT FAIL");
	}

out:
	return ret;
}

int MAX21100::reset()
{
	// if the mpu6000 is initialised after the l3gd20 and lsm303d
	// then if we don't do an irqsave/irqrestore here the mpu6000
	// frequenctly comes up in a bad state where all transfers
	// come as zero
	uint8_t tries = 5;
	while (--tries != 0) {
		irqstate_t state;
		state = irqsave();

		write_reg(MPUREG_PWR_MGMT_1, BIT_H_RESET);
		up_udelay(10000);

		// Wake up device and select GyroZ clock. Note that the
		// MAX21100 starts up in sleep mode, and it can take some time
		// for it to come out of sleep
		write_checked_reg(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
		up_udelay(1000);
		
		// Disable I2C bus (recommended on datasheet)
		write_checked_reg(MPUREG_USER_CTRL, BIT_I2C_IF_DIS);
		irqrestore(state);

		if (read_reg(MPUREG_PWR_MGMT_1) == MPU_CLK_SEL_PLLGYROZ) {
			break;
		}
		perf_count(_reset_retries);
		usleep(2000);
	}
	if (read_reg(MPUREG_PWR_MGMT_1) != MPU_CLK_SEL_PLLGYROZ) {
		return -EIO;
	}

	usleep(1000);

	// SAMPLE RATE
	_set_sample_rate(_sample_rate);
	usleep(1000);

	// FS & DLPF   FS=2000 deg/s, DLPF = 20Hz (low pass filter)
	// was 90 Hz, but this ruins quality and does not improve the
	// system response
	_set_dlpf_filter(MAX21100_DEFAULT_ONCHIP_FILTER_FREQ);
	usleep(1000);
	// Gyro scale 2000 deg/s ()
	write_checked_reg(MPUREG_GYRO_CONFIG, BITS_FS_2000DPS);
	usleep(1000);

	// correct gyro scale factors
	// scale to rad/s in SI units
	// 2000 deg/s = (2000/180)*PI = 34.906585 rad/s
	// scaling factor:
	// 1/(2^15)*(2000/180)*PI
	_gyro_range_scale = (0.0174532 / 16.4);//1.0f / (32768.0f * (2000.0f / 180.0f) * M_PI_F);
	_gyro_range_rad_s = (2000.0f / 180.0f) * M_PI_F;

	// product-specific scaling
	switch (_product) {
	case MAX21100ES_REV_C4:
	case MAX21100ES_REV_C5:
	case MAX21100_REV_C4:
	case MAX21100_REV_C5:
		// Accel scale 8g (4096 LSB/g)
		// Rev C has different scaling than rev D
		write_checked_reg(MPUREG_ACCEL_CONFIG, 1 << 3);
		break;

	case MAX21100ES_REV_D6:
	case MAX21100ES_REV_D7:
	case MAX21100ES_REV_D8:
	case MAX21100_REV_D6:
	case MAX21100_REV_D7:
	case MAX21100_REV_D8:
	case MAX21100_REV_D9:
	case MAX21100_REV_D10:
	// default case to cope with new chip revisions, which
	// presumably won't have the accel scaling bug		
	default:
		// Accel scale 8g (4096 LSB/g)
		write_checked_reg(MPUREG_ACCEL_CONFIG, 2 << 3);
		break;
	}

	// Correct accel scale factors of 4096 LSB/g
	// scale to m/s^2 ( 1g = 9.81 m/s^2)
	_accel_range_scale = (MAX21100_ONE_G / 4096.0f);
	_accel_range_m_s2 = 8.0f * MAX21100_ONE_G;

	usleep(1000);

	// INT CFG => Interrupt on Data Ready
	write_checked_reg(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);        // INT: Raw data ready
	usleep(1000);
	write_checked_reg(MPUREG_INT_PIN_CFG, BIT_INT_ANYRD_2CLEAR); // INT: Clear on any read
	usleep(1000);

	// Oscillator set
	// write_reg(MPUREG_PWR_MGMT_1,MPU_CLK_SEL_PLLGYROZ);
	usleep(1000);

	return OK;
}

int
MAX21100::probe()
{

	/* look for a product ID we recognise */
	_product = read_reg(MPUREG_PRODUCT_ID);

	// verify product revision
	switch (_product) {
	case MAX21100ES_REV_C4:
	case MAX21100ES_REV_C5:
	case MAX21100_REV_C4:
	case MAX21100_REV_C5:
	case MAX21100ES_REV_D6:
	case MAX21100ES_REV_D7:
	case MAX21100ES_REV_D8:
	case MAX21100_REV_D6:
	case MAX21100_REV_D7:
	case MAX21100_REV_D8:
	case MAX21100_REV_D9:
	case MAX21100_REV_D10:
		debug("ID 0x%02x", _product);
		_checked_values[0] = _product;
		return OK;
	}

	debug("unexpected ID 0x%02x", _product);
	return -EIO;
}

/*
  set sample rate (approximate) - 1kHz to 5Hz, for both accel and gyro
*/
void
MAX21100::_set_sample_rate(uint16_t desired_sample_rate_hz)
{
  uint8_t div = 1000 / desired_sample_rate_hz;
  if(div>200) div=200;
  if(div<1) div=1;
  write_checked_reg(MPUREG_SMPLRT_DIV, div-1);
  _sample_rate = 1000 / div;
}

/*
  set the DLPF filter frequency. This affects both accel and gyro.
 */
void
MAX21100::_set_dlpf_filter(uint16_t frequency_hz)
{
	uint8_t filter;

	/* 
	   choose next highest filter frequency available
	 */
        if (frequency_hz == 0) {
		filter = BITS_DLPF_CFG_2100HZ_NOLPF;
        } else if (frequency_hz <= 5) {
		filter = BITS_DLPF_CFG_5HZ;
	} else if (frequency_hz <= 10) {
		filter = BITS_DLPF_CFG_10HZ;
	} else if (frequency_hz <= 20) {
		filter = BITS_DLPF_CFG_20HZ;
	} else if (frequency_hz <= 42) {
		filter = BITS_DLPF_CFG_42HZ;
	} else if (frequency_hz <= 98) {
		filter = BITS_DLPF_CFG_98HZ;
	} else if (frequency_hz <= 188) {
		filter = BITS_DLPF_CFG_188HZ;
	} else if (frequency_hz <= 256) {
		filter = BITS_DLPF_CFG_256HZ_NOLPF2;
	} else {
		filter = BITS_DLPF_CFG_2100HZ_NOLPF;
	}
	write_checked_reg(MPUREG_CONFIG, filter);
}

ssize_t
MAX21100::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(accel_report);

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
	if (_call_interval == 0) {
		_accel_reports->flush();
		measure();
	}

	/* if no data, error (we could block here) */
	if (_accel_reports->empty())
		return -EAGAIN;

	perf_count(_accel_reads);

	/* copy reports out of our buffer to the caller */
	accel_report *arp = reinterpret_cast<accel_report *>(buffer);
	int transferred = 0;
	while (count--) {
		if (!_accel_reports->get(arp))
			break;
		transferred++;
		arp++;
	}

	/* return the number of bytes transferred */
	return (transferred * sizeof(accel_report));
}

int
MAX21100::self_test()
{
	if (perf_event_count(_sample_perf) == 0) {
		measure();
	}

	/* return 0 on success, 1 else */
	return (perf_event_count(_sample_perf) > 0) ? 0 : 1;
}

int
MAX21100::accel_self_test()
{
	if (self_test())
		return 1;

	/* inspect accel offsets */
	if (fabsf(_accel_scale.x_offset) < 0.000001f)
		return 1;
	if (fabsf(_accel_scale.x_scale - 1.0f) > 0.4f || fabsf(_accel_scale.x_scale - 1.0f) < 0.000001f)
		return 1;

	if (fabsf(_accel_scale.y_offset) < 0.000001f)
		return 1;
	if (fabsf(_accel_scale.y_scale - 1.0f) > 0.4f || fabsf(_accel_scale.y_scale - 1.0f) < 0.000001f)
		return 1;

	if (fabsf(_accel_scale.z_offset) < 0.000001f)
		return 1;
	if (fabsf(_accel_scale.z_scale - 1.0f) > 0.4f || fabsf(_accel_scale.z_scale - 1.0f) < 0.000001f)
		return 1;

	return 0;
}

int
MAX21100::gyro_self_test()
{
	if (self_test())
		return 1;

	/* evaluate gyro offsets, complain if offset -> zero or larger than 6 dps */
	if (fabsf(_gyro_scale.x_offset) > 0.1f || fabsf(_gyro_scale.x_offset) < 0.000001f)
		return 1;
	if (fabsf(_gyro_scale.x_scale - 1.0f) > 0.3f)
		return 1;

	if (fabsf(_gyro_scale.y_offset) > 0.1f || fabsf(_gyro_scale.y_offset) < 0.000001f)
		return 1;
	if (fabsf(_gyro_scale.y_scale - 1.0f) > 0.3f)
		return 1;

	if (fabsf(_gyro_scale.z_offset) > 0.1f || fabsf(_gyro_scale.z_offset) < 0.000001f)
		return 1;
	if (fabsf(_gyro_scale.z_scale - 1.0f) > 0.3f)
		return 1;

	return 0;
}


/*
  perform a self-test comparison to factory trim values. This takes
  about 200ms and will return OK if the current values are within 14%
  of the expected values (as per datasheet)
 */
int
MAX21100::factory_self_test()
{
	_in_factory_test = true;
	uint8_t saved_gyro_config = read_reg(MPUREG_GYRO_CONFIG);
	uint8_t saved_accel_config = read_reg(MPUREG_ACCEL_CONFIG);
	const uint16_t repeats = 100;
	int ret = OK;

	// gyro self test has to be done at 250DPS
	write_reg(MPUREG_GYRO_CONFIG, BITS_FS_250DPS);

	struct MPUReport mpu_report;	
	float accel_baseline[3];
	float gyro_baseline[3];
	float accel[3];
	float gyro[3];
	float accel_ftrim[3];
	float gyro_ftrim[3];

	// get baseline values without self-test enabled
        set_frequency(MAX21100_HIGH_BUS_SPEED);

	memset(accel_baseline, 0, sizeof(accel_baseline));
	memset(gyro_baseline, 0, sizeof(gyro_baseline));
	memset(accel, 0, sizeof(accel));
	memset(gyro, 0, sizeof(gyro));

	for (uint8_t i=0; i<repeats; i++) {
		up_udelay(1000);
		mpu_report.cmd = DIR_READ | MPUREG_INT_STATUS;
		transfer((uint8_t *)&mpu_report, ((uint8_t *)&mpu_report), sizeof(mpu_report));

		accel_baseline[0] += int16_t_from_bytes(mpu_report.accel_x);
		accel_baseline[1] += int16_t_from_bytes(mpu_report.accel_y);
		accel_baseline[2] += int16_t_from_bytes(mpu_report.accel_z);
		gyro_baseline[0] += int16_t_from_bytes(mpu_report.gyro_x);
		gyro_baseline[1] += int16_t_from_bytes(mpu_report.gyro_y);
		gyro_baseline[2] += int16_t_from_bytes(mpu_report.gyro_z);
	}

#if 1
	write_reg(MPUREG_GYRO_CONFIG, 
		  BITS_FS_250DPS | 
		  BITS_GYRO_ST_X | 
		  BITS_GYRO_ST_Y | 
		  BITS_GYRO_ST_Z);

	// accel 8g, self-test enabled all axes
	write_reg(MPUREG_ACCEL_CONFIG, saved_accel_config | 0xE0);
#endif

	up_udelay(20000);

	// get values with self-test enabled
        set_frequency(MAX21100_HIGH_BUS_SPEED);


	for (uint8_t i=0; i<repeats; i++) {
		up_udelay(1000);
		mpu_report.cmd = DIR_READ | MPUREG_INT_STATUS;
		transfer((uint8_t *)&mpu_report, ((uint8_t *)&mpu_report), sizeof(mpu_report));
		accel[0] += int16_t_from_bytes(mpu_report.accel_x);
		accel[1] += int16_t_from_bytes(mpu_report.accel_y);
		accel[2] += int16_t_from_bytes(mpu_report.accel_z);
		gyro[0] += int16_t_from_bytes(mpu_report.gyro_x);
		gyro[1] += int16_t_from_bytes(mpu_report.gyro_y);
		gyro[2] += int16_t_from_bytes(mpu_report.gyro_z);
	}

	for (uint8_t i=0; i<3; i++) {
		accel_baseline[i] /= repeats;
		gyro_baseline[i] /= repeats;
		accel[i] /= repeats;
		gyro[i] /= repeats;
	}

	// extract factory trim values
	uint8_t trims[4];
	trims[0] = read_reg(MPUREG_TRIM1);
	trims[1] = read_reg(MPUREG_TRIM2);
	trims[2] = read_reg(MPUREG_TRIM3);
	trims[3] = read_reg(MPUREG_TRIM4);
	uint8_t atrim[3];
	uint8_t gtrim[3];

	atrim[0] = ((trims[0]>>3)&0x1C) | ((trims[3]>>4)&0x03);
	atrim[1] = ((trims[1]>>3)&0x1C) | ((trims[3]>>2)&0x03);
	atrim[2] = ((trims[2]>>3)&0x1C) | ((trims[3]>>0)&0x03);
	gtrim[0] = trims[0] & 0x1F;
	gtrim[1] = trims[1] & 0x1F;
	gtrim[2] = trims[2] & 0x1F;

	// convert factory trims to right units
	for (uint8_t i=0; i<3; i++) {
		accel_ftrim[i] = 4096 * 0.34f * powf(0.92f/0.34f, (atrim[i]-1)/30.0f);
		gyro_ftrim[i] = 25 * 131.0f * powf(1.046f, gtrim[i]-1);
	}
	// Y gyro trim is negative
	gyro_ftrim[1] *= -1;

	for (uint8_t i=0; i<3; i++) {
		float diff = accel[i]-accel_baseline[i];
		float err = 100*(diff - accel_ftrim[i]) / accel_ftrim[i];
		::printf("ACCEL[%u] baseline=%d accel=%d diff=%d ftrim=%d err=%d\n",
			 (unsigned)i,
			 (int)(1000*accel_baseline[i]),
			 (int)(1000*accel[i]),
			 (int)(1000*diff),
			 (int)(1000*accel_ftrim[i]),
			 (int)err);
		if (fabsf(err) > 14) {
			::printf("FAIL\n");
			ret = -EIO;
		}
	}	
	for (uint8_t i=0; i<3; i++) {
		float diff = gyro[i]-gyro_baseline[i];
		float err = 100*(diff - gyro_ftrim[i]) / gyro_ftrim[i];
		::printf("GYRO[%u] baseline=%d gyro=%d diff=%d ftrim=%d err=%d\n",
			 (unsigned)i,
			 (int)(1000*gyro_baseline[i]),
			 (int)(1000*gyro[i]),
			 (int)(1000*(gyro[i]-gyro_baseline[i])),
			 (int)(1000*gyro_ftrim[i]),
			 (int)err);
		if (fabsf(err) > 14) {
			::printf("FAIL\n");
			ret = -EIO;
		}
	}	

	write_reg(MPUREG_GYRO_CONFIG, saved_gyro_config);
	write_reg(MPUREG_ACCEL_CONFIG, saved_accel_config);

	_in_factory_test = false;
	if (ret == OK) {
		::printf("PASSED\n");
	}

	return ret;
}


/*
  deliberately trigger an error in the sensor to trigger recovery
 */
void
MAX21100::test_error()
{
	_in_factory_test = true;
	// deliberately trigger an error. This was noticed during
	// development as a handy way to test the reset logic
	uint8_t data[16];
	memset(data, 0, sizeof(data));
	transfer(data, data, sizeof(data));
	::printf("error triggered\n");
	print_registers();
	_in_factory_test = false;
}

ssize_t
MAX21100::gyro_read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(gyro_report);

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
	if (_call_interval == 0) {
		_gyro_reports->flush();
		measure();
	}

	/* if no data, error (we could block here) */
	if (_gyro_reports->empty())
		return -EAGAIN;

	perf_count(_gyro_reads);

	/* copy reports out of our buffer to the caller */
	gyro_report *grp = reinterpret_cast<gyro_report *>(buffer);
	int transferred = 0;
	while (count--) {
		if (!_gyro_reports->get(grp))
			break;
		transferred++;
		grp++;
	}

	/* return the number of bytes transferred */
	return (transferred * sizeof(gyro_report));
}

int
MAX21100::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCRESET:
		return reset();

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

				/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_call_interval = 0;
				return OK;

				/* external signalling not supported */
			case SENSOR_POLLRATE_EXTERNAL:

				/* zero would be bad */
			case 0:
				return -EINVAL;

				/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
				return ioctl(filp, SENSORIOCSPOLLRATE, 1000);

			case SENSOR_POLLRATE_DEFAULT:
				return ioctl(filp, SENSORIOCSPOLLRATE, MAX21100_ACCEL_DEFAULT_RATE);

				/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_call_interval == 0);

					/* convert hz to hrt interval via microseconds */
					unsigned ticks = 1000000 / arg;

					/* check against maximum sane rate */
					if (ticks < 1000)
						return -EINVAL;

					// adjust filters
					float cutoff_freq_hz = _accel_filter_x.get_cutoff_freq();
					float sample_rate = 1.0e6f/ticks;
					_set_dlpf_filter(cutoff_freq_hz);
					_accel_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz);
					_accel_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz);
					_accel_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz);


					float cutoff_freq_hz_gyro = _gyro_filter_x.get_cutoff_freq();
					_set_dlpf_filter(cutoff_freq_hz_gyro);
					_gyro_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
					_gyro_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
					_gyro_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);

					/* update interval for next measurement */
					/* XXX this is a bit shady, but no other way to adjust... */
					_call.period = _call_interval = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start)
						start();

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_call_interval == 0)
			return SENSOR_POLLRATE_MANUAL;

		return 1000000 / _call_interval;

	case SENSORIOCSQUEUEDEPTH: {
		/* lower bound is mandatory, upper bound is a sanity check */
		if ((arg < 1) || (arg > 100))
			return -EINVAL;
		
		irqstate_t flags = irqsave();
		if (!_accel_reports->resize(arg)) {
			irqrestore(flags);
			return -ENOMEM;
		}
		irqrestore(flags);
		
		return OK;
	}

	case SENSORIOCGQUEUEDEPTH:
		return _accel_reports->size();

	case ACCELIOCGSAMPLERATE:
		return _sample_rate;

	case ACCELIOCSSAMPLERATE:
		_set_sample_rate(arg);
		return OK;

	case ACCELIOCGLOWPASS:
		return _accel_filter_x.get_cutoff_freq();

	case ACCELIOCSLOWPASS:
		// set hardware filtering
		_set_dlpf_filter(arg);
		// set software filtering
		_accel_filter_x.set_cutoff_frequency(1.0e6f / _call_interval, arg);
		_accel_filter_y.set_cutoff_frequency(1.0e6f / _call_interval, arg);
		_accel_filter_z.set_cutoff_frequency(1.0e6f / _call_interval, arg);
		return OK;

	case ACCELIOCSSCALE:
		{
			/* copy scale, but only if off by a few percent */
			struct accel_scale *s = (struct accel_scale *) arg;
			float sum = s->x_scale + s->y_scale + s->z_scale;
			if (sum > 2.0f && sum < 4.0f) {
				memcpy(&_accel_scale, s, sizeof(_accel_scale));
				return OK;
			} else {
				return -EINVAL;
			}
		}

	case ACCELIOCGSCALE:
		/* copy scale out */
		memcpy((struct accel_scale *) arg, &_accel_scale, sizeof(_accel_scale));
		return OK;

	case ACCELIOCSRANGE:
		/* XXX not implemented */
		// XXX change these two values on set:
		// _accel_range_scale = (9.81f / 4096.0f);
		// _accel_range_m_s2 = 8.0f * 9.81f;
		return -EINVAL;
	case ACCELIOCGRANGE:
		return (unsigned long)((_accel_range_m_s2)/MAX21100_ONE_G + 0.5f);

	case ACCELIOCSELFTEST:
		return accel_self_test();

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

int
MAX21100::gyro_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

		/* these are shared with the accel side */
	case SENSORIOCSPOLLRATE:
	case SENSORIOCGPOLLRATE:
	case SENSORIOCRESET:
		return ioctl(filp, cmd, arg);

	case SENSORIOCSQUEUEDEPTH: {
		/* lower bound is mandatory, upper bound is a sanity check */
		if ((arg < 1) || (arg > 100))
			return -EINVAL;

		irqstate_t flags = irqsave();
		if (!_gyro_reports->resize(arg)) {
			irqrestore(flags);
			return -ENOMEM;
		}
		irqrestore(flags);

		return OK;
	}

	case SENSORIOCGQUEUEDEPTH:
		return _gyro_reports->size();

	case GYROIOCGSAMPLERATE:
		return _sample_rate;

	case GYROIOCSSAMPLERATE:
		_set_sample_rate(arg);
		return OK;

	case GYROIOCGLOWPASS:
		return _gyro_filter_x.get_cutoff_freq();
	case GYROIOCSLOWPASS:
		// set hardware filtering
		_set_dlpf_filter(arg);
		_gyro_filter_x.set_cutoff_frequency(1.0e6f / _call_interval, arg);
		_gyro_filter_y.set_cutoff_frequency(1.0e6f / _call_interval, arg);
		_gyro_filter_z.set_cutoff_frequency(1.0e6f / _call_interval, arg);
		return OK;

	case GYROIOCSSCALE:
		/* copy scale in */
		memcpy(&_gyro_scale, (struct gyro_scale *) arg, sizeof(_gyro_scale));
		return OK;

	case GYROIOCGSCALE:
		/* copy scale out */
		memcpy((struct gyro_scale *) arg, &_gyro_scale, sizeof(_gyro_scale));
		return OK;

	case GYROIOCSRANGE:
		/* XXX not implemented */
		// XXX change these two values on set:
		// _gyro_range_scale = xx
		// _gyro_range_rad_s = xx
		return -EINVAL;
	case GYROIOCGRANGE:
		return (unsigned long)(_gyro_range_rad_s * 180.0f / M_PI_F + 0.5f);

	case GYROIOCSELFTEST:
		return gyro_self_test();

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

uint8_t
MAX21100::read_reg(unsigned reg, uint32_t speed)
{
	uint8_t cmd[2] = { (uint8_t)(reg | DIR_READ), 0};

        // general register transfer at low clock speed
        set_frequency(speed);

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[1];
}

uint16_t
MAX21100::read_reg16(unsigned reg)
{
	uint8_t cmd[3] = { (uint8_t)(reg | DIR_READ), 0, 0 };

        // general register transfer at low clock speed
        set_frequency(MAX21100_LOW_BUS_SPEED);

	transfer(cmd, cmd, sizeof(cmd));

	return (uint16_t)(cmd[1] << 8) | cmd[2];
}

void
MAX21100::write_reg(unsigned reg, uint8_t value)
{
	uint8_t	cmd[2];

	cmd[0] = reg | DIR_WRITE;
	cmd[1] = value;

        // general register transfer at low clock speed
        set_frequency(MAX21100_LOW_BUS_SPEED);

	transfer(cmd, nullptr, sizeof(cmd));
}

void
MAX21100::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val;

	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_reg(reg, val);
}

void
MAX21100::write_checked_reg(unsigned reg, uint8_t value)
{
	write_reg(reg, value);
	for (uint8_t i=0; i<MAX21100_NUM_CHECKED_REGISTERS; i++) {
		if (reg == _checked_registers[i]) {
			_checked_values[i] = value;
		}
	}
}

int
MAX21100::set_range(unsigned max_g)
{
#if 0
	uint8_t rangebits;
	float rangescale;

	if (max_g > 16) {
		return -ERANGE;

	} else if (max_g > 8) {		/* 16G */
		rangebits = OFFSET_LSB1_RANGE_16G;
		rangescale = 1.98;

	} else if (max_g > 4) {		/* 8G */
		rangebits = OFFSET_LSB1_RANGE_8G;
		rangescale = 0.99;

	} else if (max_g > 3) {		/* 4G */
		rangebits = OFFSET_LSB1_RANGE_4G;
		rangescale = 0.5;

	} else if (max_g > 2) {		/* 3G */
		rangebits = OFFSET_LSB1_RANGE_3G;
		rangescale = 0.38;

	} else if (max_g > 1) {		/* 2G */
		rangebits = OFFSET_LSB1_RANGE_2G;
		rangescale = 0.25;

	} else {			/* 1G */
		rangebits = OFFSET_LSB1_RANGE_1G;
		rangescale = 0.13;
	}

	/* adjust sensor configuration */
	modify_reg(ADDR_OFFSET_LSB1, OFFSET_LSB1_RANGE_MASK, rangebits);
	_range_scale = rangescale;
#endif
	return OK;
}

void
MAX21100::start()
{
	/* make sure we are stopped first */
	stop();

	/* discard any stale data in the buffers */
	_accel_reports->flush();
	_gyro_reports->flush();

	/* start polling at the specified rate */
	hrt_call_every(&_call, 1000, _call_interval, (hrt_callout)&MAX21100::measure_trampoline, this);
}

void
MAX21100::stop()
{
	hrt_cancel(&_call);
}

void
MAX21100::measure_trampoline(void *arg)
{
	MAX21100 *dev = reinterpret_cast<MAX21100 *>(arg);

	/* make another measurement */
	dev->measure();
}

void
MAX21100::check_registers(void)
{
	/*
	  we read the register at full speed, even though it isn't
	  listed as a high speed register. The low speed requirement
	  for some registers seems to be a propgation delay
	  requirement for changing sensor configuration, which should
	  not apply to reading a single register. It is also a better
	  test of SPI bus health to read at the same speed as we read
	  the data registers.
	*/
	uint8_t v;
	if ((v=read_reg(_checked_registers[_checked_next], MAX21100_HIGH_BUS_SPEED)) !=
	    _checked_values[_checked_next]) {
		/*
		  if we get the wrong value then we know the SPI bus
		  or sensor is very sick. We set _register_wait to 20
		  and wait until we have seen 20 good values in a row
		  before we consider the sensor to be OK again. 
		 */
		perf_count(_bad_registers);

		/*
		  try to fix the bad register value. We only try to
		  fix one per loop to prevent a bad sensor hogging the
		  bus.
		 */
		if (_register_wait == 0 || _checked_next == 0) {
			// if the product_id is wrong then reset the
			// sensor completely
			write_reg(MPUREG_PWR_MGMT_1, BIT_H_RESET);
			// after doing a reset we need to wait a long
			// time before we do any other register writes
			// or we will end up with the mpu6000 in a
			// bizarre state where it has all correct
			// register values but large offsets on the
			// accel axes
			_reset_wait = hrt_absolute_time() + 10000;
                        _checked_next = 0;
		} else {
			write_reg(_checked_registers[_checked_next], _checked_values[_checked_next]);
			// waiting 3ms between register writes seems
			// to raise the chance of the sensor
			// recovering considerably
			_reset_wait = hrt_absolute_time() + 3000;
		}
		_register_wait = 20;
	}
	_checked_next = (_checked_next+1) % MAX21100_NUM_CHECKED_REGISTERS;
}

void
MAX21100::measure()
{
	if (_in_factory_test) {
		// don't publish any data while in factory test mode
		return;
	}

	if (hrt_absolute_time() < _reset_wait) {
		// we're waiting for a reset to complete
		return;
	}

	struct MPUReport mpu_report;
	struct Report {
		int16_t		accel_x;
		int16_t		accel_y;
		int16_t		accel_z;
		int16_t		temp;
		int16_t		gyro_x;
		int16_t		gyro_y;
		int16_t		gyro_z;
	} report;

	/* start measuring */
	perf_begin(_sample_perf);

	/*
	 * Fetch the full set of measurements from the MAX21100 in one pass.
	 */
	mpu_report.cmd = DIR_READ | MPUREG_INT_STATUS;

	check_registers();

        // sensor transfer at high clock speed
        set_frequency(MAX21100_HIGH_BUS_SPEED);

	if (OK != transfer((uint8_t *)&mpu_report, ((uint8_t *)&mpu_report), sizeof(mpu_report)))
		return;

	/*
	 * Convert from big to little endian
	 */

	report.accel_x = int16_t_from_bytes(mpu_report.accel_x);
	report.accel_y = int16_t_from_bytes(mpu_report.accel_y);
	report.accel_z = int16_t_from_bytes(mpu_report.accel_z);

	report.temp = int16_t_from_bytes(mpu_report.temp);

	report.gyro_x = int16_t_from_bytes(mpu_report.gyro_x);
	report.gyro_y = int16_t_from_bytes(mpu_report.gyro_y);
	report.gyro_z = int16_t_from_bytes(mpu_report.gyro_z);

	if (report.accel_x == 0 &&
	    report.accel_y == 0 &&
	    report.accel_z == 0 &&
	    report.temp == 0 &&
	    report.gyro_x == 0 &&
	    report.gyro_y == 0 &&
	    report.gyro_z == 0) {
		// all zero data - probably a SPI bus error
		perf_count(_bad_transfers);
		perf_end(_sample_perf);
                // note that we don't call reset() here as a reset()
                // costs 20ms with interrupts disabled. That means if
                // the mpu6k does go bad it would cause a FMU failure,
                // regardless of whether another sensor is available,
		return;
	}

	perf_count(_good_transfers);

	if (_register_wait != 0) {
		// we are waiting for some good transfers before using
		// the sensor again. We still increment
		// _good_transfers, but don't return any data yet
		_register_wait--;
		return;
	}
	    

	/*
	 * Swap axes and negate y
	 */
	int16_t accel_xt = report.accel_y;
	int16_t accel_yt = ((report.accel_x == -32768) ? 32767 : -report.accel_x);

	int16_t gyro_xt = report.gyro_y;
	int16_t gyro_yt = ((report.gyro_x == -32768) ? 32767 : -report.gyro_x);

	/*
	 * Apply the swap
	 */
	report.accel_x = accel_xt;
	report.accel_y = accel_yt;
	report.gyro_x = gyro_xt;
	report.gyro_y = gyro_yt;

	/*
	 * Report buffers.
	 */
	accel_report		arb;
	gyro_report		grb;

	/*
	 * Adjust and scale results to m/s^2.
	 */
	grb.timestamp = arb.timestamp = hrt_absolute_time();

	// report the error count as the sum of the number of bad
	// transfers and bad register reads. This allows the higher
	// level code to decide if it should use this sensor based on
	// whether it has had failures
        grb.error_count = arb.error_count = perf_event_count(_bad_transfers) + perf_event_count(_bad_registers);

	/*
	 * 1) Scale raw value to SI units using scaling from datasheet.
	 * 2) Subtract static offset (in SI units)
	 * 3) Scale the statically calibrated values with a linear
	 *    dynamically obtained factor
	 *
	 * Note: the static sensor offset is the number the sensor outputs
	 * 	 at a nominally 'zero' input. Therefore the offset has to
	 * 	 be subtracted.
	 *
	 *	 Example: A gyro outputs a value of 74 at zero angular rate
	 *	 	  the offset is 74 from the origin and subtracting
	 *		  74 from all measurements centers them around zero.
	 */


	/* NOTE: Axes have been swapped to match the board a few lines above. */

	arb.x_raw = report.accel_x;
	arb.y_raw = report.accel_y;
	arb.z_raw = report.accel_z;

	float x_in_new = ((report.accel_x * _accel_range_scale) - _accel_scale.x_offset) * _accel_scale.x_scale;
	float y_in_new = ((report.accel_y * _accel_range_scale) - _accel_scale.y_offset) * _accel_scale.y_scale;
	float z_in_new = ((report.accel_z * _accel_range_scale) - _accel_scale.z_offset) * _accel_scale.z_scale;
	
	arb.x = _accel_filter_x.apply(x_in_new);
	arb.y = _accel_filter_y.apply(y_in_new);
	arb.z = _accel_filter_z.apply(z_in_new);

	// apply user specified rotation
	rotate_3f(_rotation, arb.x, arb.y, arb.z);

	arb.scaling = _accel_range_scale;
	arb.range_m_s2 = _accel_range_m_s2;

	arb.temperature_raw = report.temp;
	arb.temperature = (report.temp) / 361.0f + 35.0f;

	grb.x_raw = report.gyro_x;
	grb.y_raw = report.gyro_y;
	grb.z_raw = report.gyro_z;

	float x_gyro_in_new = ((report.gyro_x * _gyro_range_scale) - _gyro_scale.x_offset) * _gyro_scale.x_scale;
	float y_gyro_in_new = ((report.gyro_y * _gyro_range_scale) - _gyro_scale.y_offset) * _gyro_scale.y_scale;
	float z_gyro_in_new = ((report.gyro_z * _gyro_range_scale) - _gyro_scale.z_offset) * _gyro_scale.z_scale;
	
	grb.x = _gyro_filter_x.apply(x_gyro_in_new);
	grb.y = _gyro_filter_y.apply(y_gyro_in_new);
	grb.z = _gyro_filter_z.apply(z_gyro_in_new);

	// apply user specified rotation
	rotate_3f(_rotation, grb.x, grb.y, grb.z);

	grb.scaling = _gyro_range_scale;
	grb.range_rad_s = _gyro_range_rad_s;

	grb.temperature_raw = report.temp;
	grb.temperature = (report.temp) / 361.0f + 35.0f;

	_accel_reports->force(&arb);
	_gyro_reports->force(&grb);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);
	_gyro->parent_poll_notify();

	if (!(_pub_blocked)) {
		/* log the time of this report */
		perf_begin(_controller_latency_perf);
		perf_begin(_system_latency_perf);
		/* publish it */
		orb_publish(_accel_orb_id, _accel_topic, &arb);
	}

	if (!(_pub_blocked)) {
		/* publish it */
		orb_publish(_gyro->_gyro_orb_id, _gyro->_gyro_topic, &grb);
	}

	/* stop measuring */
	perf_end(_sample_perf);
}

void
MAX21100::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_accel_reads);
	perf_print_counter(_gyro_reads);
	perf_print_counter(_bad_transfers);
	perf_print_counter(_bad_registers);
	perf_print_counter(_good_transfers);
	perf_print_counter(_reset_retries);
	_accel_reports->print_info("accel queue");
	_gyro_reports->print_info("gyro queue");
        ::printf("checked_next: %u\n", _checked_next);
        for (uint8_t i=0; i<MAX21100_NUM_CHECKED_REGISTERS; i++) {
            uint8_t v = read_reg(_checked_registers[i], MAX21100_HIGH_BUS_SPEED);
            if (v != _checked_values[i]) {
                ::printf("reg %02x:%02x should be %02x\n", 
                         (unsigned)_checked_registers[i],
                         (unsigned)v,
                         (unsigned)_checked_values[i]);
            }
        }
}

void
MAX21100::print_registers()
{
	printf("MAX21100 registers\n");
	for (uint8_t reg=MPUREG_PRODUCT_ID; reg<=108; reg++) {
		uint8_t v = read_reg(reg);
		printf("%02x:%02x ",(unsigned)reg, (unsigned)v);
		if ((reg - (MPUREG_PRODUCT_ID-1)) % 13 == 0) {
			printf("\n");
		}
	}
	printf("\n");
}


MAX21100_gyro::MAX21100_gyro(MAX21100 *parent, const char *path) :
	CDev("MAX21100_gyro", path),
	_parent(parent),
	_gyro_topic(-1),
	_gyro_orb_id(nullptr),
	_gyro_class_instance(-1)
{
}

MAX21100_gyro::~MAX21100_gyro()
{
	if (_gyro_class_instance != -1)
		unregister_class_devname(GYRO_DEVICE_PATH, _gyro_class_instance);
}

int
MAX21100_gyro::init()
{
	int ret;

	// do base class init
	ret = CDev::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		debug("gyro init failed");
		return ret;
	}

	_gyro_class_instance = register_class_devname(GYRO_DEVICE_PATH);

	return ret;
}

void
MAX21100_gyro::parent_poll_notify()
{
	poll_notify(POLLIN);
}

ssize_t
MAX21100_gyro::read(struct file *filp, char *buffer, size_t buflen)
{
	return _parent->gyro_read(filp, buffer, buflen);
}

int
MAX21100_gyro::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	return _parent->gyro_ioctl(filp, cmd, arg);
}

/**
 * Local functions in support of the shell command.
 */
namespace mpu6000
{

MAX21100	*g_dev_int; // on internal bus
MAX21100	*g_dev_ext; // on external bus

void	start(bool, enum Rotation);
void	test(bool);
void	reset(bool);
void	info(bool);
void	regdump(bool);
void	testerror(bool);
void	factorytest(bool);
void	usage();

/**
 * Start the driver.
 *
 * This function only returns if the driver is up and running
 * or failed to detect the sensor.
 */
void
start(bool external_bus, enum Rotation rotation)
{
	int fd;
	MAX21100 **g_dev_ptr = g_dev_int;
	const char *path_accel = MAX21100_DEVICE_PATH_ACCEL;
	const char *path_gyro  = MAX21100_DEVICE_PATH_GYRO;

	if (*g_dev_ptr != nullptr)
		/* if already started, the still command succeeded */
		errx(0, "already started");

	/* create the driver */
	*g_dev_ptr = new MAX21100(PX4_SPI_BUS_SENSORS, path_accel, path_gyro, (spi_dev_e)PX4_SPIDEV_GYRIO, rotation);

	if (*g_dev_ptr == nullptr)
		goto fail;

	if (OK != (*g_dev_ptr)->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	fd = open(path_accel, O_RDONLY);

	if (fd < 0)
		goto fail;

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		goto fail;

        close(fd);

	exit(0);
fail:

	if (*g_dev_ptr != nullptr) {
            delete (*g_dev_ptr);
            *g_dev_ptr = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test(bool external_bus)
{
	const char *path_accel = MAX21100_DEVICE_PATH_ACCEL;
	const char *path_gyro  = MAX21100_DEVICE_PATH_GYRO;
	accel_report a_report;
	gyro_report g_report;
	ssize_t sz;

	/* get the driver */
	int fd = open(path_accel, O_RDONLY);

	if (fd < 0)
		err(1, "%s open failed (try 'mpu6000 start')",
		    path_accel);

	/* get the driver */
	int fd_gyro = open(path_gyro, O_RDONLY);

	if (fd_gyro < 0)
		err(1, "%s open failed", path_gyro);

	/* reset to manual polling */
	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MANUAL) < 0)
		err(1, "reset to manual polling");

	/* do a simple demand read */
	sz = read(fd, &a_report, sizeof(a_report));

	if (sz != sizeof(a_report)) {
		warnx("ret: %d, expected: %d", sz, sizeof(a_report));
		err(1, "immediate acc read failed");
	}

	warnx("single read");
	warnx("time:     %lld", a_report.timestamp);
	warnx("acc  x:  \t%8.4f\tm/s^2", (double)a_report.x);
	warnx("acc  y:  \t%8.4f\tm/s^2", (double)a_report.y);
	warnx("acc  z:  \t%8.4f\tm/s^2", (double)a_report.z);
	warnx("acc  x:  \t%d\traw 0x%0x", (short)a_report.x_raw, (unsigned short)a_report.x_raw);
	warnx("acc  y:  \t%d\traw 0x%0x", (short)a_report.y_raw, (unsigned short)a_report.y_raw);
	warnx("acc  z:  \t%d\traw 0x%0x", (short)a_report.z_raw, (unsigned short)a_report.z_raw);
	warnx("acc range: %8.4f m/s^2 (%8.4f g)", (double)a_report.range_m_s2,
	      (double)(a_report.range_m_s2 / MAX21100_ONE_G));

	/* do a simple demand read */
	sz = read(fd_gyro, &g_report, sizeof(g_report));

	if (sz != sizeof(g_report)) {
		warnx("ret: %d, expected: %d", sz, sizeof(g_report));
		err(1, "immediate gyro read failed");
	}

	warnx("gyro x: \t% 9.5f\trad/s", (double)g_report.x);
	warnx("gyro y: \t% 9.5f\trad/s", (double)g_report.y);
	warnx("gyro z: \t% 9.5f\trad/s", (double)g_report.z);
	warnx("gyro x: \t%d\traw", (int)g_report.x_raw);
	warnx("gyro y: \t%d\traw", (int)g_report.y_raw);
	warnx("gyro z: \t%d\traw", (int)g_report.z_raw);
	warnx("gyro range: %8.4f rad/s (%d deg/s)", (double)g_report.range_rad_s,
	      (int)((g_report.range_rad_s / M_PI_F) * 180.0f + 0.5f));

	warnx("temp:  \t%8.4f\tdeg celsius", (double)a_report.temperature);
	warnx("temp:  \t%d\traw 0x%0x", (short)a_report.temperature_raw, (unsigned short)a_report.temperature_raw);


	/* XXX add poll-rate tests here too */

	reset(external_bus);
	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset(bool external_bus)
{
	const char *path_accel = MAX21100_DEVICE_PATH_ACCEL;
	int fd = open(path_accel, O_RDONLY);

	if (fd < 0)
		err(1, "failed ");

	if (ioctl(fd, SENSORIOCRESET, 0) < 0)
		err(1, "driver reset failed");

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		err(1, "driver poll restart failed");

        close(fd);

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info(bool external_bus)
{
	MAX21100 **g_dev_ptr = g_dev_int;
	if (*g_dev_ptr == nullptr)
		errx(1, "driver not running");

	printf("state @ %p\n", *g_dev_ptr);
	(*g_dev_ptr)->print_info();

	exit(0);
}

/**
 * Dump the register information
 */
void
regdump(bool external_bus)
{
	MAX21100 **g_dev_ptr = &g_dev_int;
	if (*g_dev_ptr == nullptr)
		errx(1, "driver not running");

	printf("regdump @ %p\n", *g_dev_ptr);
	(*g_dev_ptr)->print_registers();

	exit(0);
}

/**
 * deliberately produce an error to test recovery
 */
void
testerror(bool external_bus)
{
	MAX21100 **g_dev_ptr = external_bus?&g_dev_ext:&g_dev_int;
	if (*g_dev_ptr == nullptr)
		errx(1, "driver not running");

	(*g_dev_ptr)->test_error();

	exit(0);
}

/**
 * Dump the register information
 */
void
factorytest(bool external_bus)
{
	MAX21100 **g_dev_ptr = external_bus?&g_dev_ext:&g_dev_int;
	if (*g_dev_ptr == nullptr)
		errx(1, "driver not running");

	(*g_dev_ptr)->factory_self_test();

	exit(0);
}

void
usage()
{
	warnx("missing command: try 'start', 'info', 'test', 'reset', 'regdump', 'factorytest', 'testerror'");
	warnx("options:");
	warnx("    -X    (external bus)");
	warnx("    -R rotation");
}

} // namespace

int
max21100_main(int argc, char *argv[])
{
	bool external_bus = false;
	int ch;
	enum Rotation rotation = ROTATION_NONE;

	/* jump over start/off/etc and look at options first */
	while ((ch = getopt(argc, argv, "XR:")) != EOF) {
		switch (ch) {
		case 'X':
			external_bus = true;
			break;
		case 'R':
			rotation = (enum Rotation)atoi(optarg);
			break;
		default:
			MAX21100::usage();
			exit(0);
		}
	}

	const char *verb = argv[optind];

	/*
	 * Start/load the driver.

	 */
	if (!strcmp(verb, "start"))
		MAX21100::start(external_bus, rotation);

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test"))
		MAX21100::test(external_bus);

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset"))
		MAX21100::reset(external_bus);

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info"))
		MAX21100::info(external_bus);

	/*
	 * Print register information.
	 */
	if (!strcmp(verb, "regdump"))
		MAX21100::regdump(external_bus);

	if (!strcmp(verb, "factorytest"))
		MAX21100::factorytest(external_bus);

	if (!strcmp(verb, "testerror"))
		MAX21100::testerror(external_bus);

	errx(1, "unrecognized command, try 'start', 'test', 'reset', 'info', 'regdump', 'factorytest' or 'testerror'");
}
