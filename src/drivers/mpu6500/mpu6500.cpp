/****************************************************************************
 *
 *   Copyright (c) 2015, 2016 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name Airmind nor the names of its contributors may be
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
 * @file mpu6500.cpp
 *
 * Driver for the Invensense MPU6500 connected via SPI.
 *
 * @author Andrew Tridgell
 * @author Pat Hickey
 */

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
#include <drivers/device/integrator.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>


#define DIR_READ			0x80
#define DIR_WRITE			0x00

#define MPU_DEVICE_PATH_ACCEL		"/dev/mpu6500_accel"
#define MPU_DEVICE_PATH_GYRO		"/dev/mpu6500_gyro"
#define MPU_DEVICE_PATH_ACCEL_EXT	"/dev/mpu6500_accel_ext"
#define MPU_DEVICE_PATH_GYRO_EXT	"/dev/mpu6500_gyro_ext"

// MPU 6000 registers
#define MPUREG_WHOAMI			0x75
#define MPUREG_SMPLRT_DIV		0x19
#define MPUREG_CONFIG			0x1A
#define MPUREG_GYRO_CONFIG		0x1B
#define MPUREG_ACCEL_CONFIG		0x1C
#define MPUREG_ACCEL_CONFIG2		0x1D

#define MPUREG_FIFO_EN			0x23
#define MPUREG_INT_PIN_CFG		0x37
#define MPUREG_INT_ENABLE		0x38
#define MPUREG_INT_STATUS		0x3A
#define MPUREG_ACCEL_XOUT_H		0x3B
#define MPUREG_ACCEL_XOUT_L		0x3C
#define MPUREG_ACCEL_YOUT_H		0x3D
#define MPUREG_ACCEL_YOUT_L		0x3E
#define MPUREG_ACCEL_ZOUT_H		0x3F
#define MPUREG_ACCEL_ZOUT_L		0x40
#define MPUREG_TEMP_OUT_H		0x41
#define MPUREG_TEMP_OUT_L		0x42
#define MPUREG_GYRO_XOUT_H		0x43
#define MPUREG_GYRO_XOUT_L		0x44
#define MPUREG_GYRO_YOUT_H		0x45
#define MPUREG_GYRO_YOUT_L		0x46
#define MPUREG_GYRO_ZOUT_H		0x47
#define MPUREG_GYRO_ZOUT_L		0x48
#define MPUREG_USER_CTRL		0x6A
#define MPUREG_PWR_MGMT_1		0x6B
#define MPUREG_PWR_MGMT_2		0x6C
#define MPUREG_FIFO_COUNTH		0x72
#define MPUREG_FIFO_COUNTL		0x73
#define MPUREG_FIFO_R_W			0x74
//#define MPUREG_PRODUCT_ID		0x0C
//#define MPUREG_TRIM1			0x0D
//#define MPUREG_TRIM2			0x0E
//#define MPUREG_TRIM3			0x0F
#define MPUREG_TRIM4			0x10
#define MPUREG_GYRO_SELFTEST_X	0x00
#define MPUREG_GYRO_SELFTEST_Y	0x01
#define MPUREG_GYRO_SELFTEST_Z	0x02
#define MPUREG_ACCEL_SELFTEST_X	0x0D
#define MPUREG_ACCEL_SELFTEST_Y	0x0E
#define MPUREG_ACCEL_SELFTEST_Z	0x0F


// Configuration bits MPU 3000 and MPU 6000 (not revised)?
#define BIT_SLEEP			0x40
#define BIT_H_RESET			0x80
#define BITS_CLKSEL			0x07
#define MPU_CLK_SEL_PLLGYROX		0x01
#define MPU_CLK_SEL_PLLGYROZ		0x03
#define MPU_EXT_SYNC_GYROX		0x02
#define BITS_GYRO_ST_X			0x80
#define BITS_GYRO_ST_Y			0x40
#define BITS_GYRO_ST_Z			0x20
#define BITS_FS_250DPS			0x00
#define BITS_FS_500DPS			0x08
#define BITS_FS_1000DPS			0x10
#define BITS_FS_2000DPS			0x18
#define BITS_FS_MASK			0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2	0x00
#define BITS_DLPF_CFG_188HZ		0x01
#define BITS_DLPF_CFG_98HZ		0x02
#define BITS_DLPF_CFG_42HZ		0x03
#define BITS_DLPF_CFG_20HZ		0x04
#define BITS_DLPF_CFG_10HZ		0x05
#define BITS_DLPF_CFG_5HZ		0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF	0x07
#define BITS_DLPF_CFG_MASK		0x07
#define BIT_INT_ANYRD_2CLEAR		0x10
#define BIT_RAW_RDY_EN			0x01
#define BIT_I2C_IF_DIS			0x10
#define BIT_INT_STATUS_DATA		0x01


#define MPU6500_ACCEL_DEFAULT_RANGE_G			16
#define MPU6500_ACCEL_DEFAULT_RATE				1000
#define MPU6500_ACCEL_MAX_OUTPUT_RATE			280
#define MPU6500_ACCEL_DEFAULT_DRIVER_FILTER_FREQ	30

#define MPU6500_GYRO_DEFAULT_RANGE_G			8
#define MPU6500_GYRO_DEFAULT_RATE				1000
/* rates need to be the same between accel and gyro */
#define MPU6500_GYRO_MAX_OUTPUT_RATE			MPU6500_ACCEL_MAX_OUTPUT_RATE
#define MPU6500_GYRO_DEFAULT_DRIVER_FILTER_FREQ		30

#define MPU6500_DEFAULT_ONCHIP_FILTER_FREQ		42

#define MPU6500_ONE_G					9.80665f

#ifdef PX4_SPI_BUS_EXT
#define EXTERNAL_BUS PX4_SPI_BUS_EXT
#else
#define EXTERNAL_BUS 0
#endif

/*
  the MPU6500 can only handle high SPI bus speeds on the sensor and
  interrupt status registers. All other registers have a maximum 1MHz
  SPI speed
 */
#define MPU6500_LOW_BUS_SPEED				1000*1000
#define MPU6500_HIGH_BUS_SPEED				11*1000*1000 /* will be rounded to 10.4 MHz, within margins for MPU6K */

/*
  we set the timer interrupt to run a bit faster than the desired
  sample rate and then throw away duplicates by comparing
  accelerometer values. This time reduction is enough to cope with
  worst case timing jitter due to other timers
 */
#define MPU6500_TIMER_REDUCTION				200

class MPU6500_gyro;

class MPU6500 : public device::SPI
{
public:
	MPU6500(int bus, const char *path_accel, const char *path_gyro, spi_dev_e device, enum Rotation rotation);
	virtual ~MPU6500();

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

	friend class MPU6500_gyro;

	virtual ssize_t		gyro_read(struct file *filp, char *buffer, size_t buflen);
	virtual int		gyro_ioctl(struct file *filp, int cmd, unsigned long arg);

private:
	MPU6500_gyro		*_gyro;
	uint8_t			_product;	/** product code */

	struct hrt_call		_call;
	unsigned		_call_interval;

	ringbuffer::RingBuffer *_accel_reports;

	struct accel_calibration_s	_accel_scale;
	float			_accel_range_scale;
	float			_accel_range_m_s2;
	orb_advert_t		_accel_topic;
	int			_accel_orb_class_instance;
	int			_accel_class_instance;

	ringbuffer::RingBuffer *_gyro_reports;

	struct gyro_calibration_s	_gyro_scale;
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
	perf_counter_t		_duplicates;
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

	Integrator		_accel_int;
	Integrator		_gyro_int;

	enum Rotation		_rotation;

	// this is used to support runtime checking of key
	// configuration registers to detect SPI bus errors and sensor
	// reset
#define MPU6500_NUM_CHECKED_REGISTERS 9
	static const uint8_t	_checked_registers[MPU6500_NUM_CHECKED_REGISTERS];
	uint8_t			_checked_values[MPU6500_NUM_CHECKED_REGISTERS];
	uint8_t			_checked_next;

	// use this to avoid processing measurements when in factory
	// self test
	volatile bool		_in_factory_test;

	// last temperature reading for print_info()
	float			_last_temperature;

	// keep last accel reading for duplicate detection
	uint16_t		_last_accel[3];
	bool			_got_duplicate;

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
	 * Read a register from the MPU6500
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint8_t			read_reg(unsigned reg, uint32_t speed = MPU6500_LOW_BUS_SPEED);
	uint16_t		read_reg16(unsigned reg);

	/**
	 * Write a register in the MPU6500
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_reg(unsigned reg, uint8_t value);

	/**
	 * Modify a register in the MPU6500
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Write a register in the MPU6500, updating _checked_values
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_checked_reg(unsigned reg, uint8_t value);

	/**
	 * Set the MPU6500 measurement range.
	 *
	 * @param max_g		The maximum G value the range must support.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			set_accel_range(unsigned max_g);

	/**
	 * Swap a 16-bit value read from the MPU6500 to native byte order.
	 */
	uint16_t		swap16(uint16_t val) { return (val >> 8) | (val << 8);	}

	/**
	 * Get the internal / external state
	 *
	 * @return true if the sensor is not on the main MCU board
	 */
	bool			is_external() { return (_bus == EXTERNAL_BUS); }

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
	void _set_sample_rate(unsigned desired_sample_rate_hz);

	/*
	  check that key registers still have the right value
	 */
	void check_registers(void);

	/* do not allow to copy this class due to pointer data members */
	MPU6500(const MPU6500 &);
	MPU6500 operator=(const MPU6500 &);

#pragma pack(push, 1)
	/**
	 * Report conversation within the MPU6500, including command byte and
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
const uint8_t MPU6500::_checked_registers[MPU6500_NUM_CHECKED_REGISTERS] = {
	MPUREG_PWR_MGMT_1,
	MPUREG_USER_CTRL,
	MPUREG_SMPLRT_DIV,
	MPUREG_CONFIG,
	MPUREG_GYRO_CONFIG,
	MPUREG_ACCEL_CONFIG,
	MPUREG_INT_ENABLE,
	MPUREG_INT_PIN_CFG
};



/**
 * Helper class implementing the gyro driver node.
 */
class MPU6500_gyro : public device::CDev
{
public:
	MPU6500_gyro(MPU6500 *parent, const char *path);
	~MPU6500_gyro();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	virtual int		init();

protected:
	friend class MPU6500;

	void			parent_poll_notify();

private:
	MPU6500			*_parent;
	orb_advert_t		_gyro_topic;
	int			_gyro_orb_class_instance;
	int			_gyro_class_instance;

	/* do not allow to copy this class due to pointer data members */
	MPU6500_gyro(const MPU6500_gyro &);
	MPU6500_gyro operator=(const MPU6500_gyro &);
};

/** driver 'main' command */
extern "C" { __EXPORT int mpu6500_main(int argc, char *argv[]); }

MPU6500::MPU6500(int bus, const char *path_accel, const char *path_gyro, spi_dev_e device, enum Rotation rotation) :
	SPI("MPU6500", path_accel, bus, device, SPIDEV_MODE3, MPU6500_LOW_BUS_SPEED),
	_gyro(new MPU6500_gyro(this, path_gyro)),
	_product(0),
	_call{},
	_call_interval(0),
	_accel_reports(nullptr),
	_accel_scale{},
	_accel_range_scale(0.0f),
	_accel_range_m_s2(0.0f),
	_accel_topic(nullptr),
	_accel_orb_class_instance(-1),
	_accel_class_instance(-1),
	_gyro_reports(nullptr),
	_gyro_scale{},
	_gyro_range_scale(0.0f),
	_gyro_range_rad_s(0.0f),
	_sample_rate(1000),
	_accel_reads(perf_alloc(PC_COUNT, "mpu6500_accel_read")),
	_gyro_reads(perf_alloc(PC_COUNT, "mpu6500_gyro_read")),
	_sample_perf(perf_alloc(PC_ELAPSED, "mpu6500_read")),
	_bad_transfers(perf_alloc(PC_COUNT, "mpu6500_bad_transfers")),
	_bad_registers(perf_alloc(PC_COUNT, "mpu6500_bad_registers")),
	_good_transfers(perf_alloc(PC_COUNT, "mpu6500_good_transfers")),
	_reset_retries(perf_alloc(PC_COUNT, "mpu6500_reset_retries")),
	_duplicates(perf_alloc(PC_COUNT, "mpu6500_duplicates")),
	_system_latency_perf(perf_alloc_once(PC_ELAPSED, "sys_latency")),
	_controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency")),
	_register_wait(0),
	_reset_wait(0),
	_accel_filter_x(MPU6500_ACCEL_DEFAULT_RATE, MPU6500_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_y(MPU6500_ACCEL_DEFAULT_RATE, MPU6500_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_z(MPU6500_ACCEL_DEFAULT_RATE, MPU6500_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_x(MPU6500_GYRO_DEFAULT_RATE, MPU6500_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_y(MPU6500_GYRO_DEFAULT_RATE, MPU6500_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_z(MPU6500_GYRO_DEFAULT_RATE, MPU6500_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_int(1000000 / MPU6500_ACCEL_MAX_OUTPUT_RATE),
	_gyro_int(1000000 / MPU6500_GYRO_MAX_OUTPUT_RATE, true),
	_rotation(rotation),
	_checked_next(0),
	_in_factory_test(false),
	_last_temperature(0),
	_last_accel{},
	_got_duplicate(false)
{
	// disable debug() calls
	_debug_enabled = false;

	_device_id.devid_s.devtype = DRV_ACC_DEVTYPE_MPU6500;

	/* Prime _gyro with parents devid. */
	_gyro->_device_id.devid = _device_id.devid;
	_gyro->_device_id.devid_s.devtype = DRV_GYR_DEVTYPE_MPU6500;

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

MPU6500::~MPU6500()
{
	/* make sure we are truly inactive */
	stop();

	/* delete the gyro subdriver */
	delete _gyro;

	/* free any existing reports */
	if (_accel_reports != nullptr) {
		delete _accel_reports;
	}

	if (_gyro_reports != nullptr) {
		delete _gyro_reports;
	}

	if (_accel_class_instance != -1) {
		unregister_class_devname(ACCEL_BASE_DEVICE_PATH, _accel_class_instance);
	}

	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_accel_reads);
	perf_free(_gyro_reads);
	perf_free(_bad_transfers);
	perf_free(_bad_registers);
	perf_free(_good_transfers);
	perf_free(_reset_retries);
	perf_free(_duplicates);
}

int
MPU6500::init()
{
	int ret;

	/* do SPI init (and probe) first */
	ret = SPI::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("SPI setup failed");
		return ret;
	}

	/* allocate basic report buffers */
	_accel_reports = new ringbuffer::RingBuffer(2, sizeof(accel_report));

	if (_accel_reports == nullptr) {
		goto out;
	}

	_gyro_reports = new ringbuffer::RingBuffer(2, sizeof(gyro_report));

	if (_gyro_reports == nullptr) {
		goto out;
	}

	if (reset() != OK) {
		goto out;
	}

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
		DEVICE_DEBUG("gyro init failed");
		return ret;
	}

	_accel_class_instance = register_class_devname(ACCEL_BASE_DEVICE_PATH);

	measure();

	/* advertise sensor topic, measure manually to initialize valid report */
	struct accel_report arp;
	_accel_reports->get(&arp);

	/* measurement will have generated a report, publish */
	_accel_topic = orb_advertise_multi(ORB_ID(sensor_accel), &arp,
					   &_accel_orb_class_instance, (is_external()) ? ORB_PRIO_MAX : ORB_PRIO_HIGH);

	if (_accel_topic == nullptr) {
		warnx("ADVERT FAIL");
	}


	/* advertise sensor topic, measure manually to initialize valid report */
	struct gyro_report grp;
	_gyro_reports->get(&grp);

	_gyro->_gyro_topic = orb_advertise_multi(ORB_ID(sensor_gyro), &grp,
			     &_gyro->_gyro_orb_class_instance, (is_external()) ? ORB_PRIO_MAX : ORB_PRIO_HIGH);

	if (_gyro->_gyro_topic == nullptr) {
		warnx("ADVERT FAIL");
	}

out:
	return ret;
}

int MPU6500::reset()
{
	// if the mpu6500 is initialised after the l3gd20 and lsm303d
	// then if we don't do an irqsave/irqrestore here the mpu6500
	// frequenctly comes up in a bad state where all transfers
	// come as zero
	uint8_t tries = 5;

	while (--tries != 0) {
		irqstate_t state;
		state = px4_enter_critical_section();

		write_reg(MPUREG_PWR_MGMT_1, BIT_H_RESET);
		up_udelay(10000);

		// Wake up device and select GyroZ clock. Note that the
		// MPU6500 starts up in sleep mode, and it can take some time
		// for it to come out of sleep
		write_checked_reg(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
		up_udelay(1000);

		// Disable I2C bus (recommended on datasheet)
		write_checked_reg(MPUREG_USER_CTRL, BIT_I2C_IF_DIS);
		px4_leave_critical_section(state);

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
	_set_dlpf_filter(MPU6500_DEFAULT_ONCHIP_FILTER_FREQ);
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

	set_accel_range(MPU6500_ACCEL_DEFAULT_RANGE_G);

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
MPU6500::probe()
{

	/* look for a product ID we recognise */
	uint8_t who = read_reg(MPUREG_WHOAMI);

//        log("WHOAMI  0x%02x", who);
	if (who != 0x70) {return -EIO;}

	return (OK);
}

/*
  set sample rate (approximate) - 1kHz to 5Hz, for both accel and gyro
*/
void
MPU6500::_set_sample_rate(unsigned desired_sample_rate_hz)
{
	if (desired_sample_rate_hz == 0 ||
	    desired_sample_rate_hz == GYRO_SAMPLERATE_DEFAULT ||
	    desired_sample_rate_hz == ACCEL_SAMPLERATE_DEFAULT) {
		desired_sample_rate_hz = MPU6500_GYRO_DEFAULT_RATE;
	}

	uint8_t div = 1000 / desired_sample_rate_hz;

	if (div > 200) { div = 200; }

	if (div < 1) { div = 1; }

	write_checked_reg(MPUREG_SMPLRT_DIV, div - 1);
	_sample_rate = 1000 / div;
}

/*
  set the DLPF filter frequency. This affects both accel and gyro.
 */
void
MPU6500::_set_dlpf_filter(uint16_t frequency_hz)
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
MPU6500::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(accel_report);

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
	if (_call_interval == 0) {
		_accel_reports->flush();
		measure();
	}

	/* if no data, error (we could block here) */
	if (_accel_reports->empty()) {
		return -EAGAIN;
	}

	perf_count(_accel_reads);

	/* copy reports out of our buffer to the caller */
	accel_report *arp = reinterpret_cast<accel_report *>(buffer);
	int transferred = 0;

	while (count--) {
		if (!_accel_reports->get(arp)) {
			break;
		}

		transferred++;
		arp++;
	}

	/* return the number of bytes transferred */
	return (transferred * sizeof(accel_report));
}

int
MPU6500::self_test()
{
	if (perf_event_count(_sample_perf) == 0) {
		measure();
	}

	/* return 0 on success, 1 else */
	return (perf_event_count(_sample_perf) > 0) ? 0 : 1;
}

int
MPU6500::accel_self_test()
{
	if (self_test()) {
		return 1;
	}

	return 0;
}

int
MPU6500::gyro_self_test()
{
	if (self_test()) {
		return 1;
	}

	/*
	 * Maximum deviation of 20 degrees, according to
	 * http://www.invensense.com/mems/gyro/documents/PS-MPU-6000A-00v3.4.pdf
	 * Section 6.1, initial ZRO tolerance
	 */
	const float max_offset = 0.34f;
	/* 30% scale error is chosen to catch completely faulty units but
	 * to let some slight scale error pass. Requires a rate table or correlation
	 * with mag rotations + data fit to
	 * calibrate properly and is not done by default.
	 */
	const float max_scale = 0.3f;

	/* evaluate gyro offsets, complain if offset -> zero or larger than 20 dps. */
	if (fabsf(_gyro_scale.x_offset) > max_offset) {
		return 1;
	}

	/* evaluate gyro scale, complain if off by more than 30% */
	if (fabsf(_gyro_scale.x_scale - 1.0f) > max_scale) {
		return 1;
	}

	if (fabsf(_gyro_scale.y_offset) > max_offset) {
		return 1;
	}

	if (fabsf(_gyro_scale.y_scale - 1.0f) > max_scale) {
		return 1;
	}

	if (fabsf(_gyro_scale.z_offset) > max_offset) {
		return 1;
	}

	if (fabsf(_gyro_scale.z_scale - 1.0f) > max_scale) {
		return 1;
	}

	return 0;
}



/*
  perform a self-test comparison to factory trim values. This takes
  about 200ms and will return OK if the current values are within 14%
  of the expected values (as per datasheet)
 */
int
MPU6500::factory_self_test()
{
//	_in_factory_test = true;
//	uint8_t saved_gyro_config = read_reg(MPUREG_GYRO_CONFIG);
//	uint8_t saved_accel_config = read_reg(MPUREG_ACCEL_CONFIG);
//	const uint16_t repeats = 100;
	int ret = OK;

	// gyro self test has to be done at 250DPS
//	write_reg(MPUREG_GYRO_CONFIG, BITS_FS_250DPS);

//	struct MPUReport mpu_report;
//	float accel_baseline[3];
//	float gyro_baseline[3];
//	float accel[3];
//	float gyro[3];
//	float accel_ftrim[3];
//	float gyro_ftrim[3];
//
//	// get baseline values without self-test enabled
//      set_frequency(MPU6500_HIGH_BUS_SPEED);

//	memset(accel_baseline, 0, sizeof(accel_baseline));
//	memset(gyro_baseline, 0, sizeof(gyro_baseline));
//	memset(accel, 0, sizeof(accel));
//	memset(gyro, 0, sizeof(gyro));
//
//	for (uint8_t i=0; i<repeats; i++) {
//		up_udelay(1000);
//		mpu_report.cmd = DIR_READ | MPUREG_INT_STATUS;
//		transfer((uint8_t *)&mpu_report, ((uint8_t *)&mpu_report), sizeof(mpu_report));
//
//		accel_baseline[0] += int16_t_from_bytes(mpu_report.accel_x);
//		accel_baseline[1] += int16_t_from_bytes(mpu_report.accel_y);
//		accel_baseline[2] += int16_t_from_bytes(mpu_report.accel_z);
//		gyro_baseline[0] += int16_t_from_bytes(mpu_report.gyro_x);
//		gyro_baseline[1] += int16_t_from_bytes(mpu_report.gyro_y);
//		gyro_baseline[2] += int16_t_from_bytes(mpu_report.gyro_z);
//	}
//
//#if 1
//	write_reg(MPUREG_GYRO_CONFIG,
//		  BITS_FS_250DPS |
//		  BITS_GYRO_ST_X |
//		  BITS_GYRO_ST_Y |
//		  BITS_GYRO_ST_Z);
//
//	// accel 8g, self-test enabled all axes
//	write_reg(MPUREG_ACCEL_CONFIG, saved_accel_config | 0xE0);
//#endif

//	up_udelay(20000);

//	// get values with self-test enabled
//        set_frequency(MPU6500_HIGH_BUS_SPEED);


//	for (uint8_t i=0; i<repeats; i++) {
//		up_udelay(1000);
//		mpu_report.cmd = DIR_READ | MPUREG_INT_STATUS;
//		transfer((uint8_t *)&mpu_report, ((uint8_t *)&mpu_report), sizeof(mpu_report));
//		accel[0] += int16_t_from_bytes(mpu_report.accel_x);
//		accel[1] += int16_t_from_bytes(mpu_report.accel_y);
//		accel[2] += int16_t_from_bytes(mpu_report.accel_z);
//		gyro[0] += int16_t_from_bytes(mpu_report.gyro_x);
//		gyro[1] += int16_t_from_bytes(mpu_report.gyro_y);
//		gyro[2] += int16_t_from_bytes(mpu_report.gyro_z);
//	}
//
//	for (uint8_t i=0; i<3; i++) {
//		accel_baseline[i] /= repeats;
//		gyro_baseline[i] /= repeats;
//		accel[i] /= repeats;
//		gyro[i] /= repeats;
//	}
//
//	// extract factory trim values
//	uint8_t trims[4];
//	trims[0] = read_reg(MPUREG_TRIM1);
//	trims[1] = read_reg(MPUREG_TRIM2);
//	trims[2] = read_reg(MPUREG_TRIM3);
//	trims[3] = read_reg(MPUREG_TRIM4);
//	uint8_t atrim[3];
//	uint8_t gtrim[3];

//	atrim[0] = ((trims[0]>>3)&0x1C) | ((trims[3]>>4)&0x03);
//	atrim[1] = ((trims[1]>>3)&0x1C) | ((trims[3]>>2)&0x03);
//	atrim[2] = ((trims[2]>>3)&0x1C) | ((trims[3]>>0)&0x03);
//	gtrim[0] = trims[0] & 0x1F;
//	gtrim[1] = trims[1] & 0x1F;
//	gtrim[2] = trims[2] & 0x1F;
//
	// convert factory trims to right units
//	for (uint8_t i=0; i<3; i++) {
//		accel_ftrim[i] = 4096 * 0.34f * powf(0.92f/0.34f, (atrim[i]-1)/30.0f);
//		gyro_ftrim[i] = 25 * 131.0f * powf(1.046f, gtrim[i]-1);
//	}
	// Y gyro trim is negative
//	gyro_ftrim[1] *= -1;

//	for (uint8_t i=0; i<3; i++) {
//		float diff = accel[i]-accel_baseline[i];
//		float err = 100*(diff - accel_ftrim[i]) / accel_ftrim[i];
//		::printf("ACCEL[%u] baseline=%d accel=%d diff=%d ftrim=%d err=%d\n",
//			 (unsigned)i,
//			 (int)(1000*accel_baseline[i]),
//			 (int)(1000*accel[i]),
//			 (int)(1000*diff),
//			 (int)(1000*accel_ftrim[i]),
//			 (int)err);
//		if (fabsf(err) > 14) {
//			::printf("FAIL\n");
//			ret = -EIO;
//		}
//	}
//	for (uint8_t i=0; i<3; i++) {
//		float diff = gyro[i]-gyro_baseline[i];
//		float err = 100*(diff - gyro_ftrim[i]) / gyro_ftrim[i];
//		::printf("GYRO[%u] baseline=%d gyro=%d diff=%d ftrim=%d err=%d\n",
//			 (unsigned)i,
//			 (int)(1000*gyro_baseline[i]),
//			 (int)(1000*gyro[i]),
//			 (int)(1000*(gyro[i]-gyro_baseline[i])),
//			 (int)(1000*gyro_ftrim[i]),
//			 (int)err);
//		if (fabsf(err) > 14) {
//			::printf("FAIL\n");
//			ret = -EIO;
//		}
//	}

//	write_reg(MPUREG_GYRO_CONFIG, saved_gyro_config);
//	write_reg(MPUREG_ACCEL_CONFIG, saved_accel_config);
//
//	_in_factory_test = false;
//	if (ret == OK) {
//		::printf("PASSED\n");
//	}

	return ret;
}


/*
  deliberately trigger an error in the sensor to trigger recovery
 */
void
MPU6500::test_error()
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
MPU6500::gyro_read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(gyro_report);

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
	if (_call_interval == 0) {
		_gyro_reports->flush();
		measure();
	}

	/* if no data, error (we could block here) */
	if (_gyro_reports->empty()) {
		return -EAGAIN;
	}

	perf_count(_gyro_reads);

	/* copy reports out of our buffer to the caller */
	gyro_report *grp = reinterpret_cast<gyro_report *>(buffer);
	int transferred = 0;

	while (count--) {
		if (!_gyro_reports->get(grp)) {
			break;
		}

		transferred++;
		grp++;
	}

	/* return the number of bytes transferred */
	return (transferred * sizeof(gyro_report));
}

int
MPU6500::ioctl(struct file *filp, int cmd, unsigned long arg)
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
				return ioctl(filp, SENSORIOCSPOLLRATE, MPU6500_ACCEL_DEFAULT_RATE);

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_call_interval == 0);

					/* convert hz to hrt interval via microseconds */
					unsigned ticks = 1000000 / arg;

					/* check against maximum sane rate */
					if (ticks < 1000) {
						return -EINVAL;
					}

					// adjust filters
					float cutoff_freq_hz = _accel_filter_x.get_cutoff_freq();
					float sample_rate = 1.0e6f / ticks;
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
					_call_interval = ticks;

					/*
					  set call interval faster then the sample time. We
					  then detect when we have duplicate samples and reject
					  them. This prevents aliasing due to a beat between the
					  stm32 clock and the mpu6500 clock
					 */
					_call.period = _call_interval - MPU6500_TIMER_REDUCTION;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_call_interval == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return 1000000 / _call_interval;

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = px4_enter_critical_section();

			if (!_accel_reports->resize(arg)) {
				px4_leave_critical_section(flags);
				return -ENOMEM;
			}

			px4_leave_critical_section(flags);

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

	case ACCELIOCSSCALE: {
			/* copy scale, but only if off by a few percent */
			struct accel_calibration_s *s = (struct accel_calibration_s *) arg;
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
		memcpy((struct accel_calibration_s *) arg, &_accel_scale, sizeof(_accel_scale));
		return OK;

	case ACCELIOCSRANGE:
		return set_accel_range(arg);

	case ACCELIOCGRANGE:
		return (unsigned long)((_accel_range_m_s2) / MPU6500_ONE_G + 0.5f);

	case ACCELIOCSELFTEST:
		return accel_self_test();

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

int
MPU6500::gyro_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	/* these are shared with the accel side */
	case SENSORIOCSPOLLRATE:
	case SENSORIOCGPOLLRATE:
	case SENSORIOCRESET:
		return ioctl(filp, cmd, arg);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = px4_enter_critical_section();

			if (!_gyro_reports->resize(arg)) {
				px4_leave_critical_section(flags);
				return -ENOMEM;
			}

			px4_leave_critical_section(flags);

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
		memcpy(&_gyro_scale, (struct gyro_calibration_s *) arg, sizeof(_gyro_scale));
		return OK;

	case GYROIOCGSCALE:
		/* copy scale out */
		memcpy((struct gyro_calibration_s *) arg, &_gyro_scale, sizeof(_gyro_scale));
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
MPU6500::read_reg(unsigned reg, uint32_t speed)
{
	uint8_t cmd[2] = { (uint8_t)(reg | DIR_READ), 0};

	// general register transfer at low clock speed
	set_frequency(speed);

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[1];
}

uint16_t
MPU6500::read_reg16(unsigned reg)
{
	uint8_t cmd[3] = { (uint8_t)(reg | DIR_READ), 0, 0 };

	// general register transfer at low clock speed
	set_frequency(MPU6500_LOW_BUS_SPEED);

	transfer(cmd, cmd, sizeof(cmd));

	return (uint16_t)(cmd[1] << 8) | cmd[2];
}

void
MPU6500::write_reg(unsigned reg, uint8_t value)
{
	uint8_t	cmd[2];

	cmd[0] = reg | DIR_WRITE;
	cmd[1] = value;

	// general register transfer at low clock speed
	set_frequency(MPU6500_LOW_BUS_SPEED);

	transfer(cmd, nullptr, sizeof(cmd));
}

void
MPU6500::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val;

	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_reg(reg, val);
}

void
MPU6500::write_checked_reg(unsigned reg, uint8_t value)
{
	write_reg(reg, value);

	for (uint8_t i = 0; i < MPU6500_NUM_CHECKED_REGISTERS; i++) {
		if (reg == _checked_registers[i]) {
			_checked_values[i] = value;
		}
	}
}

int
MPU6500::set_accel_range(unsigned max_g_in)
{
	// workaround for bugged versions of MPU6k (rev C)
//	switch (_product) {
//		case MPU6500ES_REV_C4:
//		case MPU6500ES_REV_C5:
//		case MPU6500_REV_C4:
//		case MPU6500_REV_C5:
//			write_checked_reg(MPUREG_ACCEL_CONFIG, 1 << 3);
//			_accel_range_scale = (MPU6500_ONE_G / 4096.0f);
//			_accel_range_m_s2 = 8.0f * MPU6500_ONE_G;
//			return OK;
//	}

	uint8_t afs_sel;
	float lsb_per_g;
	float max_accel_g;

	if (max_g_in > 8) { // 16g - AFS_SEL = 3
		afs_sel = 3;
		lsb_per_g = 2048;
		max_accel_g = 16;

	} else if (max_g_in > 4) { //  8g - AFS_SEL = 2
		afs_sel = 2;
		lsb_per_g = 4096;
		max_accel_g = 8;

	} else if (max_g_in > 2) { //  4g - AFS_SEL = 1
		afs_sel = 1;
		lsb_per_g = 8192;
		max_accel_g = 4;

	} else {                //  2g - AFS_SEL = 0
		afs_sel = 0;
		lsb_per_g = 16384;
		max_accel_g = 2;
	}

	write_checked_reg(MPUREG_ACCEL_CONFIG, afs_sel << 3);
	_accel_range_scale = (MPU6500_ONE_G / lsb_per_g);
	_accel_range_m_s2 = max_accel_g * MPU6500_ONE_G;

	return OK;
}

void
MPU6500::start()
{
	/* make sure we are stopped first */
	stop();

	/* discard any stale data in the buffers */
	_accel_reports->flush();
	_gyro_reports->flush();

	/* start polling at the specified rate */
	hrt_call_every(&_call,
		       1000,
		       _call_interval - MPU6500_TIMER_REDUCTION,
		       (hrt_callout)&MPU6500::measure_trampoline, this);
}

void
MPU6500::stop()
{
	hrt_cancel(&_call);

	/* reset internal states */
	memset(_last_accel, 0, sizeof(_last_accel));

	/* discard unread data in the buffers */
	_accel_reports->flush();
	_gyro_reports->flush();
}

void
MPU6500::measure_trampoline(void *arg)
{
	MPU6500 *dev = reinterpret_cast<MPU6500 *>(arg);

	/* make another measurement */
	dev->measure();
}

void
MPU6500::check_registers(void)
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

	if ((v = read_reg(_checked_registers[_checked_next], MPU6500_HIGH_BUS_SPEED)) !=
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
			// or we will end up with the mpu6500 in a
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

	_checked_next = (_checked_next + 1) % MPU6500_NUM_CHECKED_REGISTERS;
}

void
MPU6500::measure()
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
	 * Fetch the full set of measurements from the MPU6500 in one pass.
	 */
	mpu_report.cmd = DIR_READ | MPUREG_INT_STATUS;

	// sensor transfer at high clock speed
	set_frequency(MPU6500_HIGH_BUS_SPEED);

	if (OK != transfer((uint8_t *)&mpu_report, ((uint8_t *)&mpu_report), sizeof(mpu_report))) {
		return;
	}

	//      check_registers();
	/*
	   see if this is duplicate accelerometer data. Note that we
	   can't use the data ready interrupt status bit in the status
	   register as that also goes high on new gyro data, and when
	   we run with BITS_DLPF_CFG_256HZ_NOLPF2 the gyro is being
	   sampled at 8kHz, so we would incorrectly think we have new
	   data when we are in fact getting duplicate accelerometer data.
	*/
	if (!_got_duplicate && memcmp(&mpu_report.accel_x[0], &_last_accel[0], 6) == 0) {
		// it isn't new data - wait for next timer
		perf_end(_sample_perf);
		perf_count(_duplicates);
		_got_duplicate = true;
		return;
	}

	memcpy(&_last_accel[0], &mpu_report.accel_x[0], 6);
	_got_duplicate = false;

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

	float xraw_f = report.accel_x;
	float yraw_f = report.accel_y;
	float zraw_f = report.accel_z;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	float x_in_new = ((xraw_f * _accel_range_scale) - _accel_scale.x_offset) * _accel_scale.x_scale;
	float y_in_new = ((yraw_f * _accel_range_scale) - _accel_scale.y_offset) * _accel_scale.y_scale;
	float z_in_new = ((zraw_f * _accel_range_scale) - _accel_scale.z_offset) * _accel_scale.z_scale;

	arb.x = _accel_filter_x.apply(x_in_new);
	arb.y = _accel_filter_y.apply(y_in_new);
	arb.z = _accel_filter_z.apply(z_in_new);

	math::Vector<3> aval(x_in_new, y_in_new, z_in_new);
	math::Vector<3> aval_integrated;

	bool accel_notify = _accel_int.put(arb.timestamp, aval, aval_integrated, arb.integral_dt);
	arb.x_integral = aval_integrated(0);
	arb.y_integral = aval_integrated(1);
	arb.z_integral = aval_integrated(2);

	arb.scaling = _accel_range_scale;
	arb.range_m_s2 = _accel_range_m_s2;

	_last_temperature = (report.temp) / 361.0f + 35.0f;

	arb.temperature_raw = report.temp;
	arb.temperature = _last_temperature;

	/* return device ID */
	arb.device_id = _device_id.devid;

	grb.x_raw = report.gyro_x;
	grb.y_raw = report.gyro_y;
	grb.z_raw = report.gyro_z;

	xraw_f = report.gyro_x;
	yraw_f = report.gyro_y;
	zraw_f = report.gyro_z;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	float x_gyro_in_new = ((xraw_f * _gyro_range_scale) - _gyro_scale.x_offset) * _gyro_scale.x_scale;
	float y_gyro_in_new = ((yraw_f * _gyro_range_scale) - _gyro_scale.y_offset) * _gyro_scale.y_scale;
	float z_gyro_in_new = ((zraw_f * _gyro_range_scale) - _gyro_scale.z_offset) * _gyro_scale.z_scale;

	grb.x = _gyro_filter_x.apply(x_gyro_in_new);
	grb.y = _gyro_filter_y.apply(y_gyro_in_new);
	grb.z = _gyro_filter_z.apply(z_gyro_in_new);

	math::Vector<3> gval(x_gyro_in_new, y_gyro_in_new, z_gyro_in_new);
	math::Vector<3> gval_integrated;

	bool gyro_notify = _gyro_int.put(arb.timestamp, gval, gval_integrated, grb.integral_dt);
	grb.x_integral = gval_integrated(0);
	grb.y_integral = gval_integrated(1);
	grb.z_integral = gval_integrated(2);

	grb.scaling = _gyro_range_scale;
	grb.range_rad_s = _gyro_range_rad_s;

	grb.temperature_raw = report.temp;
	grb.temperature = _last_temperature;

	/* return device ID */
	grb.device_id = _gyro->_device_id.devid;

	_accel_reports->force(&arb);
	_gyro_reports->force(&grb);

	/* notify anyone waiting for data */
	if (accel_notify) {
		poll_notify(POLLIN);
	}

	if (gyro_notify) {
		_gyro->parent_poll_notify();
	}

	if (accel_notify && !(_pub_blocked)) {
		/* log the time of this report */
		perf_begin(_controller_latency_perf);
		perf_begin(_system_latency_perf);
		/* publish it */
		orb_publish(ORB_ID(sensor_accel), _accel_topic, &arb);
	}

	if (gyro_notify && !(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(sensor_gyro), _gyro->_gyro_topic, &grb);
	}

	/* stop measuring */
	perf_end(_sample_perf);
}

void
MPU6500::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_accel_reads);
	perf_print_counter(_gyro_reads);
	perf_print_counter(_bad_transfers);
	perf_print_counter(_bad_registers);
	perf_print_counter(_good_transfers);
	perf_print_counter(_reset_retries);
	perf_print_counter(_duplicates);
	_accel_reports->print_info("accel queue");
	_gyro_reports->print_info("gyro queue");
	::printf("checked_next: %u\n", _checked_next);

	for (uint8_t i = 0; i < MPU6500_NUM_CHECKED_REGISTERS; i++) {
		uint8_t v = read_reg(_checked_registers[i], MPU6500_HIGH_BUS_SPEED);

		if (v != _checked_values[i]) {
			::printf("reg %02x:%02x should be %02x\n",
				 (unsigned)_checked_registers[i],
				 (unsigned)v,
				 (unsigned)_checked_values[i]);
		}
	}

	::printf("temperature: %.1f\n", (double)_last_temperature);
}

void
MPU6500::print_registers()
{
	printf("MPU6500 registers\n");

	for (uint8_t reg = 0; reg <= 108; reg++) {
		uint8_t v = read_reg(reg);
		printf("%02x:%02x ", (unsigned)reg, (unsigned)v);

		if (reg  % 16 == 0) {
			printf("\n");
		}
	}

	printf("\n");
}


MPU6500_gyro::MPU6500_gyro(MPU6500 *parent, const char *path) :
	CDev("MPU6500_gyro", path),
	_parent(parent),
	_gyro_topic(nullptr),
	_gyro_orb_class_instance(-1),
	_gyro_class_instance(-1)
{
}

MPU6500_gyro::~MPU6500_gyro()
{
	if (_gyro_class_instance != -1) {
		unregister_class_devname(GYRO_BASE_DEVICE_PATH, _gyro_class_instance);
	}
}

int
MPU6500_gyro::init()
{
	int ret;

	// do base class init
	ret = CDev::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("gyro init failed");
		return ret;
	}

	_gyro_class_instance = register_class_devname(GYRO_BASE_DEVICE_PATH);

	return ret;
}

void
MPU6500_gyro::parent_poll_notify()
{
	poll_notify(POLLIN);
}

ssize_t
MPU6500_gyro::read(struct file *filp, char *buffer, size_t buflen)
{
	return _parent->gyro_read(filp, buffer, buflen);
}

int
MPU6500_gyro::ioctl(struct file *filp, int cmd, unsigned long arg)
{

	switch (cmd) {
	case DEVIOCGDEVICEID:
		return (int)CDev::ioctl(filp, cmd, arg);
		break;

	default:
		return _parent->gyro_ioctl(filp, cmd, arg);
	}
}

/**
 * Local functions in support of the shell command.
 */
namespace mpu6500
{


MPU6500	*g_dev_int; // on internal bus
MPU6500	*g_dev_ext; // on external bus

void	start(bool, enum Rotation, int range);
void	stop(bool);
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
start(bool external_bus, enum Rotation rotation, int range)
{
	int fd;
	MPU6500 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;
	const char *path_accel = external_bus ? MPU_DEVICE_PATH_ACCEL_EXT : MPU_DEVICE_PATH_ACCEL;
	const char *path_gyro  = external_bus ? MPU_DEVICE_PATH_GYRO_EXT : MPU_DEVICE_PATH_GYRO;

	if (*g_dev_ptr != nullptr)
		/* if already started, the still command succeeded */
	{
		errx(0, "already started");
	}

	/* create the driver */
	if (external_bus) {
#if defined(PX4_SPI_BUS_EXT) && defined(PX4_SPIDEV_EXT_MPU)
		*g_dev_ptr = new MPU6500(PX4_SPI_BUS_EXT, path_accel, path_gyro, (spi_dev_e)PX4_SPIDEV_EXT_MPU, rotation);
#else
		errx(0, "External SPI not available");
#endif

	} else {
		*g_dev_ptr = new MPU6500(PX4_SPI_BUS_SENSORS, path_accel, path_gyro, (spi_dev_e)PX4_SPIDEV_MPU, rotation);
	}

	if (*g_dev_ptr == nullptr) {
		goto fail;
	}

	if (OK != (*g_dev_ptr)->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(path_accel, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

	if (ioctl(fd, ACCELIOCSRANGE, range) < 0) {
		goto fail;
	}

	close(fd);

	exit(0);
fail:

	if (*g_dev_ptr != nullptr) {
		delete *g_dev_ptr;
		*g_dev_ptr = nullptr;
	}

	errx(1, "driver start failed");
}

void
stop(bool external_bus)
{
	MPU6500 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

	if (*g_dev_ptr != nullptr) {
		delete *g_dev_ptr;
		*g_dev_ptr = nullptr;

	} else {
		/* warn, but not an error */
		warnx("already stopped.");
	}

	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test(bool external_bus)
{
	const char *path_accel = external_bus ? MPU_DEVICE_PATH_ACCEL_EXT : MPU_DEVICE_PATH_ACCEL;
	const char *path_gyro  = external_bus ? MPU_DEVICE_PATH_GYRO_EXT : MPU_DEVICE_PATH_GYRO;
	accel_report a_report;
	gyro_report g_report;
	ssize_t sz;

	/* get the driver */
	int fd = open(path_accel, O_RDONLY);

	if (fd < 0)
		err(1, "%s open failed (try 'mpu6500 start')",
		    path_accel);

	/* get the driver */
	int fd_gyro = open(path_gyro, O_RDONLY);

	if (fd_gyro < 0) {
		err(1, "%s open failed", path_gyro);
	}

	/* reset to manual polling */
	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MANUAL) < 0) {
		err(1, "reset to manual polling");
	}

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
	      (double)(a_report.range_m_s2 / MPU6500_ONE_G));

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

	/* reset to default polling */
	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "reset to default polling");
	}

	close(fd);
	close(fd_gyro);

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
	const char *path_accel = external_bus ? MPU_DEVICE_PATH_ACCEL_EXT : MPU_DEVICE_PATH_ACCEL;
	int fd = open(path_accel, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	close(fd);

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info(bool external_bus)
{
	MPU6500 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

	if (*g_dev_ptr == nullptr) {
		errx(1, "driver not running");
	}

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
	MPU6500 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

	if (*g_dev_ptr == nullptr) {
		errx(1, "driver not running");
	}

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
	MPU6500 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

	if (*g_dev_ptr == nullptr) {
		errx(1, "driver not running");
	}

	(*g_dev_ptr)->test_error();

	exit(0);
}

/**
 * Dump the register information
 */
void
factorytest(bool external_bus)
{
	MPU6500 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

	if (*g_dev_ptr == nullptr) {
		errx(1, "driver not running");
	}

	(*g_dev_ptr)->factory_self_test();

	exit(0);
}

void
usage()
{
	warnx("missing command: try 'start', 'info', 'test', 'stop',\n'reset', 'regdump', 'factorytest', 'testerror'");
	warnx("options:");
	warnx("    -X    (external bus)");
	warnx("    -R rotation");
	warnx("    -a accel range (in g)");
}

} // namespace

int
mpu6500_main(int argc, char *argv[])
{
	bool external_bus = false;
	int ch;
	enum Rotation rotation = ROTATION_NONE;
	int accel_range = 8;

	/* jump over start/off/etc and look at options first */
	while ((ch = getopt(argc, argv, "XR:a:")) != EOF) {
		switch (ch) {
		case 'X':
			external_bus = true;
			break;

		case 'R':
			rotation = (enum Rotation)atoi(optarg);
			break;

		case 'a':
			accel_range = atoi(optarg);
			break;

		default:
			mpu6500::usage();
			exit(0);
		}
	}

	const char *verb = argv[optind];

	/*
	 * Start/load the driver.

	 */
	if (!strcmp(verb, "start")) {
		mpu6500::start(external_bus, rotation, accel_range);
	}

	if (!strcmp(verb, "stop")) {
		mpu6500::stop(external_bus);
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		mpu6500::test(external_bus);
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		mpu6500::reset(external_bus);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		mpu6500::info(external_bus);
	}

	/*
	 * Print register information.
	 */
	if (!strcmp(verb, "regdump")) {
		mpu6500::regdump(external_bus);
	}

	if (!strcmp(verb, "factorytest")) {
		mpu6500::factorytest(external_bus);
	}

	if (!strcmp(verb, "testerror")) {
		mpu6500::testerror(external_bus);
	}

	mpu6500::usage();
	exit(1);
}
