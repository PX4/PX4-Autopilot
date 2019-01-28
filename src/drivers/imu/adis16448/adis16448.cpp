/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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
 * @file adis16448.cpp
 *
 * Driver for the Analog device ADIS16448 connected via SPI.
 *
 * @author Amir Melzer
 * @author Andrew Tridgell
 * @author Pat Hickey
 * @author Lorenz Meier <lm@inf.ethz.ch>
 *
 */

#include <px4_config.h>
#include <ecl/geo/geo.h>
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

#include <perf/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/conversions.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include <board_config.h>

#include <drivers/device/spi.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>

#include <board_config.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>
#include <px4_work_queue/ScheduledWorkItem.hpp>

#define DIR_READ				0x00
#define DIR_WRITE				0x80

#define ADIS16448_DEVICE_PATH_ACCEL		"/dev/adis16448_accel"
#define ADIS16448_DEVICE_PATH_GYRO		"/dev/adis16448_gyro"
#define ADIS16448_DEVICE_PATH_MAG		"/dev/adis16448_mag"

//  ADIS16448 registers
#define ADIS16448_FLASH_CNT  	0x00  /* Flash memory write count */
#define ADIS16448_SUPPLY_OUT 	0x02  /* Power supply measurement */
#define ADIS16448_XGYRO_OUT 	0x04  /* X-axis gyroscope output */
#define ADIS16448_YGYRO_OUT 	0x06  /* Y-axis gyroscope output */
#define ADIS16448_ZGYRO_OUT 	0x08  /* Z-axis gyroscope output */
#define ADIS16448_XACCL_OUT 	0x0A  /* X-axis accelerometer output */
#define ADIS16448_YACCL_OUT 	0x0C  /* Y-axis accelerometer output */
#define ADIS16448_ZACCL_OUT 	0x0E  /* Z-axis accelerometer output */
#define ADIS16448_XMAGN_OUT 	0x10  /* X-axis magnetometer measurement */
#define ADIS16448_YMAGN_OUT 	0x12  /* Y-axis magnetometer measurement */
#define ADIS16448_ZMAGN_OUT 	0x14  /* Z-axis magnetometer measurement */
#define ADIS16448_BARO_OUT  	0x16  /* Barometric pressure output */
#define ADIS16448_TEMP_OUT  	0x18  /* Temperature output */

/* Calibration parameters */
#define ADIS16448_XGYRO_OFF 	0x1A  /* X-axis gyroscope bias offset factor */
#define ADIS16448_YGYRO_OFF 	0x1C  /* Y-axis gyroscope bias offset factor */
#define ADIS16448_ZGYRO_OFF 	0x1E  /* Z-axis gyroscope bias offset factor */
#define ADIS16448_XACCL_OFF 	0x20  /* X-axis acceleration bias offset factor */
#define ADIS16448_YACCL_OFF 	0x22  /* Y-axis acceleration bias offset factor */
#define ADIS16448_ZACCL_OFF 	0x24  /* Z-axis acceleration bias offset factor */
#define ADIS16448_XMAGN_HIC 	0x26  /* X-axis magnetometer, hard-iron factor */
#define ADIS16448_YMAGN_HIC 	0x28  /* Y-axis magnetometer, hard-iron factor */
#define ADIS16448_ZMAGN_HIC 	0x2A  /* Z-axis magnetometer, hard-iron factor */
#define ADIS16448_XMAGN_SIC	 	0x2C  /* X-axis magnetometer, soft-iron factor */
#define ADIS16448_YMAGN_SIC 	0x2E  /* Y-axis magnetometer, soft-iron factor */
#define ADIS16448_ZMAGN_SIC 	0x30  /* Z-axis magnetometer, soft-iron factor */

#define ADIS16448_GPIO_CTRL 	0x32  /* Auxiliary digital input/output control */
#define ADIS16448_MSC_CTRL  	0x34  /* Miscellaneous control */
#define ADIS16448_SMPL_PRD  	0x36  /* Internal sample period (rate) control */
#define ADIS16448_SENS_AVG  	0x38  /* Dynamic range and digital filter control */
#define ADIS16448_SLP_CNT   	0x3A  /* Sleep mode control */
#define ADIS16448_DIAG_STAT 	0x3C  /* System status */

/* Alarm functions */
#define ADIS16448_GLOB_CMD  	0x3E  /* System command */
#define ADIS16448_ALM_MAG1  	0x40  /* Alarm 1 amplitude threshold */
#define ADIS16448_ALM_MAG2  	0x42  /* Alarm 2 amplitude threshold */
#define ADIS16448_ALM_SMPL1 	0x44  /* Alarm 1 sample size */
#define ADIS16448_ALM_SMPL2 	0x46  /* Alarm 2 sample size */
#define ADIS16448_ALM_CTRL  	0x48  /* Alarm control */

#define ADIS16334_LOT_ID1   	0x52  /* Lot identification code 1 */
#define ADIS16334_LOT_ID2   	0x54  /* Lot identification code 2 */
#define ADIS16448_PRODUCT_ID 	0x56  /* Product identifier */
#define ADIS16334_SERIAL_NUMBER 0x58  /* Serial number, lot specific */

#define ADIS16448_Product		0x4040/* Product ID Description for ADIS16448 */

#define BITS_SMPL_PRD_NO_TAP_CFG 	(0<<8)
#define BITS_SMPL_PRD_2_TAP_CFG	 	(1<<8)
#define BITS_SMPL_PRD_4_TAP_CFG	 	(2<<8)
#define BITS_SMPL_PRD_8_TAP_CFG	 	(3<<8)
#define BITS_SMPL_PRD_16_TAP_CFG	(4<<8)

#define BITS_GYRO_DYN_RANGE_1000_CFG (4<<8)
#define BITS_GYRO_DYN_RANGE_500_CFG	 (2<<8)
#define BITS_GYRO_DYN_RANGE_250_CFG	 (1<<8)

#define BITS_FIR_NO_TAP_CFG		(0<<0)
#define BITS_FIR_2_TAP_CFG		(1<<0)
#define BITS_FIR_4_TAP_CFG		(2<<0)
#define BITS_FIR_8_TAP_CFG		(3<<0)
#define BITS_FIR_16_TAP_CFG		(4<<0)
#define BITS_FIR_32_TAP_CFG		(5<<0)
#define BITS_FIR_64_TAP_CFG		(6<<0)
#define BITS_FIR_128_TAP_CFG	(7<<0)


#define ADIS16448_GYRO_DEFAULT_RATE					100
#define ADIS16448_GYRO_DEFAULT_DRIVER_FILTER_FREQ	30

#define ADIS16448_ACCEL_DEFAULT_RATE				100
#define ADIS16448_ACCEL_DEFAULT_DRIVER_FILTER_FREQ	30

#define ADIS16448_MAG_DEFAULT_RATE					100
#define ADIS16448_MAG_DEFAULT_DRIVER_FILTER_FREQ	30

#define ADIS16448_ACCEL_MAX_OUTPUT_RATE              1221
#define ADIS16448_GYRO_MAX_OUTPUT_RATE               1221

#define FW_FILTER									false

#define SPI_BUS_SPEED								1000000
#define T_STALL										9

#define GYROINITIALSENSITIVITY						250
#define ACCELINITIALSENSITIVITY						(1.0f / 1200.0f)
#define MAGINITIALSENSITIVITY						(1.0f / 7.0f)
#define ACCELDYNAMICRANGE							18.0f
#define MAGDYNAMICRANGE								1900.0f

class ADIS16448_gyro;
class ADIS16448_mag;

class ADIS16448 : public device::SPI, public px4::ScheduledWorkItem
{
public:
	ADIS16448(int bus, const char *path_accel, const char *path_gyro, const char *path_mag, uint32_t device,
		  enum Rotation rotation);
	virtual ~ADIS16448();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver and sensor.
	 */
	void			print_info();

	void 			print_calibration_data();

protected:
	virtual int		probe();

	friend class ADIS16448_gyro;
	friend class ADIS16448_mag;


	virtual ssize_t		gyro_read(struct file *filp, char *buffer, size_t buflen);
	virtual int		gyro_ioctl(struct file *filp, int cmd, unsigned long arg);

	virtual ssize_t		mag_read(struct file *filp, char *buffer, size_t buflen);
	virtual int		mag_ioctl(struct file *filp, int cmd, unsigned long arg);


private:
	ADIS16448_gyro		*_gyro;
	ADIS16448_mag		*_mag;

	uint16_t			_product;	/** product code */

	unsigned			_call_interval;

	ringbuffer::RingBuffer			*_gyro_reports;

	struct gyro_calibration_s	_gyro_scale;
	float				_gyro_range_scale;
	float				_gyro_range_rad_s;

	ringbuffer::RingBuffer			*_accel_reports;

	struct accel_calibration_s	_accel_scale;
	float				_accel_range_scale;
	float				_accel_range_m_s2;
	orb_advert_t		_accel_topic;
	int					_accel_orb_class_instance;
	int					_accel_class_instance;

	ringbuffer::RingBuffer			*_mag_reports;

	struct mag_calibration_s	_mag_scale;
	float				_mag_range_scale;
	float				_mag_range_mgauss;

	unsigned			_sample_rate;
	perf_counter_t		_accel_reads;
	perf_counter_t		_gyro_reads;
	perf_counter_t		_mag_reads;
	perf_counter_t		_sample_perf;
	perf_counter_t		_bad_transfers;


	math::LowPassFilter2p	_gyro_filter_x;
	math::LowPassFilter2p	_gyro_filter_y;
	math::LowPassFilter2p	_gyro_filter_z;
	math::LowPassFilter2p	_accel_filter_x;
	math::LowPassFilter2p	_accel_filter_y;
	math::LowPassFilter2p	_accel_filter_z;
	math::LowPassFilter2p	_mag_filter_x;
	math::LowPassFilter2p	_mag_filter_y;
	math::LowPassFilter2p	_mag_filter_z;

	Integrator			_accel_int;
	Integrator			_gyro_int;

	enum Rotation		_rotation;

#pragma pack(push, 1)
	/**
	 * Report conversation with in the ADIS16448, including command byte and interrupt status.
	 */
	struct ADISReport {
		uint16_t		cmd;
		uint16_t		status;
		uint16_t		gyro_x;
		uint16_t		gyro_y;
		uint16_t		gyro_z;
		uint16_t		accel_x;
		uint16_t		accel_y;
		uint16_t		accel_z;
		uint16_t		mag_x;
		uint16_t		mag_y;
		uint16_t		mag_z;
		uint16_t		baro;
		uint16_t		temp;

		ADISReport():
			cmd(0),
			status(0),
			gyro_x(0),
			gyro_y(0),
			gyro_z(0),
			accel_x(0),
			accel_y(0),
			accel_z(0),
			mag_x(0),
			mag_y(0),
			mag_z(0),
			baro(0),
			temp(0) {}
	};
#pragma pack(pop)

	/**
	 * Start automatic measurement.
	 */
	void		start();

	/**
	 * Stop automatic measurement.
	 */
	void		stop();

	/**
	 * Reset chip.
	 *
	 * Resets the chip and measurements ranges, but not scale and offset.
	 */
	int			reset();

	void		Run() override;

	/**
	 * Fetch measurements from the sensor and update the report buffers.
	 */
	int			measure();

	/**
	 * Read a register from the ADIS16448
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint16_t		read_reg16(unsigned reg);

	/**
	 * Write a register in the ADIS16448
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void 			write_reg16(unsigned reg, uint16_t value);

	/**
	 * Modify a register in the ADIS16448
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_reg16(unsigned reg, uint16_t clearbits, uint16_t setbits);

	/**
	 * Swap a 16-bit value read from the ADIS16448 to native byte order.
	 */
	uint16_t		swap16(uint16_t val) { return (val >> 8) | (val << 8);	}

	/**
	 * convert 12 bit integer format to int16.
	 */
	int16_t			convert12BitToINT16(int16_t word);

	/**
	 * Measurement self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int 			self_test();

	/*
	  set low pass filter frequency
	 */
	void _set_dlpf_filter(uint16_t frequency_hz);

	/*
	  set IMU to factory default
	 */
	void _set_factory_default();

	/*
	  set sample rate (approximate) - 1kHz to 5Hz
	*/
	void _set_sample_rate(uint16_t desired_sample_rate_hz);

	/*
	  set the gyroscope dynamic range
	*/
	void _set_gyro_dyn_range(uint16_t desired_gyro_dyn_range);

	ADIS16448(const ADIS16448 &);
	ADIS16448 operator=(const ADIS16448 &);
};

/**
 * Helper class implementing the gyro driver node.
 */
class ADIS16448_gyro : public device::CDev
{
public:
	ADIS16448_gyro(ADIS16448 *parent, const char *path);
	virtual ~ADIS16448_gyro();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	virtual int		init();

protected:
	friend class ADIS16448;

	void			parent_poll_notify();
private:
	ADIS16448			*_parent;
	orb_advert_t		_gyro_topic;
	int					_gyro_orb_class_instance;
	int					_gyro_class_instance;

	/* do not allow to copy this class due to pointer data members */
	ADIS16448_gyro(const ADIS16448_gyro &);
	ADIS16448_gyro operator=(const ADIS16448_gyro &);

};
/**
 * Helper class implementing the mag driver node.
 */
class ADIS16448_mag : public device::CDev
{
public:
	ADIS16448_mag(ADIS16448 *parent, const char *path);
	virtual ~ADIS16448_mag();
	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	virtual int		init();

protected:
	friend class ADIS16448;

	void			parent_poll_notify();
private:
	ADIS16448			*_parent;
	orb_advert_t		_mag_topic;
	int					_mag_orb_class_instance;
	int					_mag_class_instance;

	/* do not allow to copy this class due to pointer data members */
	ADIS16448_mag(const ADIS16448_mag &);
	ADIS16448_mag operator=(const ADIS16448_mag &);

};
/** driver 'main' command */
extern "C" { __EXPORT int adis16448_main(int argc, char *argv[]); }

ADIS16448::ADIS16448(int bus, const char *path_accel, const char *path_gyro, const char *path_mag, uint32_t device,
		     enum Rotation rotation) :
	SPI("ADIS16448", path_accel, bus, device, SPIDEV_MODE3, SPI_BUS_SPEED),
	ScheduledWorkItem(px4::device_bus_to_wq(this->get_device_id())),
	_gyro(new ADIS16448_gyro(this, path_gyro)),
	_mag(new ADIS16448_mag(this, path_mag)),
	_product(0),
	_call_interval(0),
	_gyro_reports(nullptr),
	_gyro_scale{},
	_gyro_range_scale(0.0f),
	_gyro_range_rad_s(0.0f),
	_accel_reports(nullptr),
	_accel_scale{},
	_accel_range_scale(0.0f),
	_accel_range_m_s2(0.0f),
	_accel_topic(nullptr),
	_accel_orb_class_instance(-1),
	_accel_class_instance(-1),
	_mag_reports(nullptr),
	_mag_scale{},
	_mag_range_scale(0.0f),
	_mag_range_mgauss(0.0f),
	_sample_rate(100),																			/* Init sampling frequency set to 100Hz */
	_accel_reads(perf_alloc(PC_COUNT, "adis16448_accel_read")),
	_gyro_reads(perf_alloc(PC_COUNT, "adis16448_gyro_read")),
	_mag_reads(perf_alloc(PC_COUNT, "adis16448_mag_read")),
	_sample_perf(perf_alloc(PC_ELAPSED, "adis16448_read")),
	_bad_transfers(perf_alloc(PC_COUNT, "adis16448_bad_transfers")),
	_gyro_filter_x(ADIS16448_GYRO_DEFAULT_RATE, ADIS16448_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_y(ADIS16448_GYRO_DEFAULT_RATE, ADIS16448_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_z(ADIS16448_GYRO_DEFAULT_RATE, ADIS16448_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_x(ADIS16448_ACCEL_DEFAULT_RATE, ADIS16448_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_y(ADIS16448_ACCEL_DEFAULT_RATE, ADIS16448_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_z(ADIS16448_ACCEL_DEFAULT_RATE, ADIS16448_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_mag_filter_x(ADIS16448_MAG_DEFAULT_RATE, ADIS16448_MAG_DEFAULT_DRIVER_FILTER_FREQ),
	_mag_filter_y(ADIS16448_MAG_DEFAULT_RATE, ADIS16448_MAG_DEFAULT_DRIVER_FILTER_FREQ),
	_mag_filter_z(ADIS16448_MAG_DEFAULT_RATE, ADIS16448_MAG_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_int(1000000 / ADIS16448_ACCEL_MAX_OUTPUT_RATE, false),
	_gyro_int(1000000 / ADIS16448_GYRO_MAX_OUTPUT_RATE, true),
	_rotation(rotation)
{
	// disable debug() calls
	_debug_enabled = false;

	_device_id.devid_s.devtype = DRV_ACC_DEVTYPE_ADIS16448;

	_gyro->_device_id.devid = _device_id.devid;
	_gyro->_device_id.devid_s.devtype = DRV_GYR_DEVTYPE_ADIS16448;

	_mag->_device_id.devid = _device_id.devid;
	_mag->_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_ADIS16448;

	// default gyro scale factors
	_gyro_scale.x_offset = 0;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0;
	_gyro_scale.z_scale  = 1.0f;

	// default accel scale factors
	_accel_scale.x_offset = 0;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0;
	_accel_scale.z_scale  = 1.0f;

	// default mag scale factors
	_mag_scale.x_offset = 0;
	_mag_scale.x_scale  = 1.0f;
	_mag_scale.y_offset = 0;
	_mag_scale.y_scale  = 1.0f;
	_mag_scale.z_offset = 0;
	_mag_scale.z_scale  = 1.0f;
}

ADIS16448::~ADIS16448()
{
	/* make sure we are truly inactive */
	stop();

	/* delete the gyro subdriver */
	delete _gyro;
	delete _mag;

	/* free any existing reports */
	if (_gyro_reports != nullptr) {
		delete _gyro_reports;
	}

	if (_accel_reports != nullptr) {
		delete _accel_reports;
	}

	if (_mag_reports != nullptr) {
		delete _mag_reports;
	}

	if (_accel_class_instance != -1) {
		unregister_class_devname(ACCEL_BASE_DEVICE_PATH, _accel_class_instance);
	}

	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_accel_reads);
	perf_free(_gyro_reads);
	perf_free(_mag_reads);
	perf_free(_bad_transfers);
}

int
ADIS16448::init()
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
	_gyro_reports = new ringbuffer::RingBuffer(2, sizeof(sensor_gyro_s));

	if (_gyro_reports == nullptr) {
		goto out;
	}

	_accel_reports = new ringbuffer::RingBuffer(2, sizeof(sensor_accel_s));

	if (_accel_reports == nullptr) {
		goto out;
	}

	_mag_reports = new ringbuffer::RingBuffer(2, sizeof(mag_report));

	if (_mag_reports == nullptr) {
		goto out;
	}

	if (reset() != OK) {
		goto out;
	}

	/* Initialize offsets and scales */
	_gyro_scale.x_offset = 0;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0;
	_gyro_scale.z_scale  = 1.0f;

	_accel_scale.x_offset = 0;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0;
	_accel_scale.z_scale  = 1.0f;

	_mag_scale.x_offset   = 0;
	_mag_scale.x_scale    = 1.0f;
	_mag_scale.y_offset   = 0;
	_mag_scale.y_scale    = 1.0f;
	_mag_scale.z_offset   = 0;
	_mag_scale.z_scale    = 1.0f;

	/* do CDev init for the gyro device node, keep it optional */
	ret = _gyro->init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("gyro init failed");
		return ret;
	}

	/* do CDev init for the gyro device node, keep it optional */
	ret = _mag->init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("mag init failed");
		return ret;
	}

	_accel_class_instance = register_class_devname(ACCEL_BASE_DEVICE_PATH);

	/* fetch an initial set of measurements for advertisement */
	measure();

	/* advertise sensor topic, measure manually to initialize valid report */
	sensor_accel_s arp;
	_accel_reports->get(&arp);

	/* measurement will have generated a report, publish */
	_accel_topic = orb_advertise_multi(ORB_ID(sensor_accel), &arp,
					   &_accel_orb_class_instance, ORB_PRIO_MAX);

	if (_accel_topic == nullptr) {
		warnx("ADVERT FAIL");
	}

	sensor_gyro_s grp;

	_gyro_reports->get(&grp);

	_gyro->_gyro_topic = orb_advertise_multi(ORB_ID(sensor_gyro), &grp,
			     &_gyro->_gyro_orb_class_instance, ORB_PRIO_MAX);

	if (_gyro->_gyro_topic == nullptr) {
		warnx("ADVERT FAIL");
	}

	struct mag_report mrp;

	_mag_reports->get(&mrp);

	_mag->_mag_topic = orb_advertise_multi(ORB_ID(sensor_mag), &mrp,
					       &_mag->_mag_orb_class_instance, ORB_PRIO_MAX);

	if (_mag->_mag_topic == nullptr) {
		warnx("ADVERT FAIL");
	}

out:
	return ret;
}

int ADIS16448::reset()
{

	/* Set gyroscope scale to default value */
	_set_gyro_dyn_range(GYROINITIALSENSITIVITY);

	/* Set digital FIR filter tap */
	_set_dlpf_filter(BITS_FIR_16_TAP_CFG);

	/* Set IMU sample rate */
	_set_sample_rate(_sample_rate);

	_accel_range_scale = CONSTANTS_ONE_G * ACCELINITIALSENSITIVITY;
	_accel_range_m_s2  = CONSTANTS_ONE_G * ACCELDYNAMICRANGE;
	_mag_range_scale   = MAGINITIALSENSITIVITY;
	_mag_range_mgauss  = MAGDYNAMICRANGE;

	/* settling time */
	up_udelay(50000);

	return OK;

}

int
ADIS16448::probe()
{
	uint16_t serial_number;

	/* retry 5 time to get the ADIS16448 PRODUCT ID number */
	for (int i = 0; i < 5; i++) {
		/* recognize product ID */
		_product = read_reg16(ADIS16448_PRODUCT_ID);

		if (_product != 0) {
			break;
		}
	}

	/* recognize product serial number */
	serial_number = (read_reg16(ADIS16334_SERIAL_NUMBER) & 0xfff);

	/* verify product ID */
	switch (_product) {
	case ADIS16448_Product:
		DEVICE_DEBUG("ADIS16448 is detected ID: 0x%02x, Serial: 0x%02x", _product, serial_number);
		modify_reg16(ADIS16448_GPIO_CTRL, 0x0200, 0x0002);			/* Turn on ADIS16448 adaptor board led */
		return OK;
	}

	DEVICE_DEBUG("unexpected ID 0x%02x", _product);
	return -EIO;
}

/* set sample rate for both accel and gyro */
void
ADIS16448::_set_sample_rate(uint16_t desired_sample_rate_hz)
{
	uint16_t smpl_prd = 0;

	if (desired_sample_rate_hz <= 51) {
		smpl_prd = BITS_SMPL_PRD_16_TAP_CFG;

	} else if (desired_sample_rate_hz <= 102) {
		smpl_prd = BITS_SMPL_PRD_8_TAP_CFG;

	} else if (desired_sample_rate_hz <= 204) {
		smpl_prd = BITS_SMPL_PRD_4_TAP_CFG;

	} else if (desired_sample_rate_hz <= 409) {
		smpl_prd = BITS_SMPL_PRD_2_TAP_CFG;

	} else {
		smpl_prd = BITS_SMPL_PRD_NO_TAP_CFG;
	}

	modify_reg16(ADIS16448_SMPL_PRD, 0x1f00, smpl_prd);

	if ((read_reg16(ADIS16448_SMPL_PRD) & 0x1f00) != smpl_prd) {
		DEVICE_DEBUG("failed to set IMU sample rate");
	}

}

/* set the DLPF FIR filter tap. This affects both accelerometer and gyroscope. */
void
ADIS16448::_set_dlpf_filter(uint16_t desired_filter_tap)
{
	modify_reg16(ADIS16448_SENS_AVG, 0x0007, desired_filter_tap);

	/* Verify data write on the IMU */

	if ((read_reg16(ADIS16448_SENS_AVG) & 0x0007) != desired_filter_tap) {
		DEVICE_DEBUG("failed to set IMU filter");
	}

}

/* set IMU to factory defaults. */
void
ADIS16448::_set_factory_default()
{
	write_reg16(ADIS16448_GLOB_CMD, 0x02);
}

/* set the gyroscope dynamic range */
void
ADIS16448::_set_gyro_dyn_range(uint16_t desired_gyro_dyn_range)
{

	uint16_t gyro_range_selection = 0;

	if (desired_gyro_dyn_range <= 250) {
		gyro_range_selection = BITS_GYRO_DYN_RANGE_250_CFG;

	} else if (desired_gyro_dyn_range <= 500) {
		gyro_range_selection = BITS_GYRO_DYN_RANGE_500_CFG;

	} else {
		gyro_range_selection = BITS_GYRO_DYN_RANGE_1000_CFG;
	}

	modify_reg16(ADIS16448_SENS_AVG, 0x0700, gyro_range_selection);

	/* Verify data write on the IMU */

	if ((read_reg16(ADIS16448_SENS_AVG) & 0x0700) != gyro_range_selection) {
		DEVICE_DEBUG("failed to set gyro range");

	} else {
		_gyro_range_rad_s  = ((float)(gyro_range_selection >> 8) * 250.0f / 180.0f) * M_PI_F;
		_gyro_range_scale  = (float)(gyro_range_selection >> 8) / 100.0f;
	}
}


ssize_t
ADIS16448::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(sensor_accel_s);

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
	sensor_accel_s *arp = reinterpret_cast<sensor_accel_s *>(buffer);
	int transferred = 0;

	while (count--) {
		if (!_accel_reports->get(arp)) {
			break;
		}

		transferred++;
		arp++;
	}

	/* return the number of bytes transferred */
	return (transferred * sizeof(sensor_accel_s));
}

int
ADIS16448::self_test()
{
	if (perf_event_count(_sample_perf) == 0) {
		measure();
	}

	/* return 0 on success, 1 else */
	return (perf_event_count(_sample_perf) > 0) ? 0 : 1;
}

ssize_t
ADIS16448::gyro_read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(sensor_gyro_s);

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
	sensor_gyro_s *grp = reinterpret_cast<sensor_gyro_s *>(buffer);
	int transferred = 0;

	while (count--) {
		if (!_gyro_reports->get(grp)) {
			break;
		}

		transferred++;
		grp++;
	}

	/* return the number of bytes transferred */
	return (transferred * sizeof(sensor_gyro_s));
}

ssize_t
ADIS16448::mag_read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(mag_report);

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
	if (_call_interval == 0) {
		_mag_reports->flush();
		measure();
	}

	/* if no data, error (we could block here) */
	if (_mag_reports->empty()) {
		return -EAGAIN;
	}

	perf_count(_mag_reads);

	/* copy reports out of our buffer to the caller */
	mag_report *mrp = reinterpret_cast<mag_report *>(buffer);
	int transferred = 0;

	while (count--) {
		if (!_mag_reports->get(mrp)) {
			break;
		}

		transferred++;
		mrp++;
	}

	/* return the number of bytes transferred */
	return (transferred * sizeof(mag_report));
}

int
ADIS16448::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCRESET:
		return reset();

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default polling rate */
			case SENSOR_POLLRATE_DEFAULT:
				return ioctl(filp, SENSORIOCSPOLLRATE, ADIS16448_ACCEL_DEFAULT_RATE);

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
					_accel_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz);
					_accel_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz);
					_accel_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz);


					float cutoff_freq_hz_gyro = _gyro_filter_x.get_cutoff_freq();
					_gyro_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
					_gyro_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
					_gyro_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);


					float cutoff_freq_hz_mag = _mag_filter_x.get_cutoff_freq();
					_mag_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz_mag);
					_mag_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz_mag);
					_mag_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz_mag);

					/* update interval for next measurement */
					/* XXX this is a bit shady, but no other way to adjust... */
					_call_interval = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

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

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

int
ADIS16448::gyro_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	/* these are shared with the accel side */
	case SENSORIOCSPOLLRATE:
	case SENSORIOCRESET:
		return ioctl(filp, cmd, arg);

	case GYROIOCSSCALE:
		/* copy scale in */
		memcpy(&_gyro_scale, (struct gyro_calibration_s *) arg, sizeof(_gyro_scale));
		return OK;

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

int
ADIS16448::mag_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	/* these are shared with the accel side */
	case SENSORIOCSPOLLRATE:
	case SENSORIOCRESET:
		return ioctl(filp, cmd, arg);

	case MAGIOCSSCALE:
		/* copy scale in */
		memcpy(&_mag_scale, (struct mag_calibration_s *) arg, sizeof(_mag_scale));
		return OK;

	case MAGIOCGSCALE:
		/* copy scale out */
		memcpy((struct mag_calibration_s *) arg, &_mag_scale, sizeof(_mag_scale));
		return OK;

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}

}

uint16_t
ADIS16448::read_reg16(unsigned reg)
{
	uint16_t cmd[1];

	cmd[0] = ((reg | DIR_READ) << 8) & 0xff00;
	transferhword(cmd, nullptr, 1);
	up_udelay(T_STALL);
	transferhword(nullptr, cmd, 1);
	up_udelay(T_STALL);

	return cmd[0];
}

void
ADIS16448::write_reg16(unsigned reg, uint16_t value)
{
	uint16_t	cmd[2];

	cmd[0] = ((reg | DIR_WRITE) << 8) | (0x00ff & value);
	cmd[1] = (((reg + 0x1) | DIR_WRITE) << 8) | ((0xff00 & value) >> 8);

	transferhword(cmd, nullptr, 1);
	up_udelay(T_STALL);
	transferhword(cmd + 1, nullptr, 1);
	up_udelay(T_STALL);
}

void
ADIS16448::modify_reg16(unsigned reg, uint16_t clearbits, uint16_t setbits)
{
	uint16_t	val;

	val = read_reg16(reg);
	val &= ~clearbits;
	val |= setbits;
	write_reg16(reg, val);
}

int16_t
ADIS16448::convert12BitToINT16(int16_t word)
{

	int16_t outputbuffer = 0;

	if ((word >> 11) & 0x1) {
		outputbuffer = (word & 0xfff) | 0xf000;

	} else {
		outputbuffer = (word & 0x0fff);
	}

	return (outputbuffer);
}

void
ADIS16448::start()
{
	/* make sure we are stopped first */
	uint32_t last_call_interval = _call_interval;
	stop();
	_call_interval = last_call_interval;

	/* discard any stale data in the buffers */
	_gyro_reports->flush();
	_accel_reports->flush();
	_mag_reports->flush();

	/* start polling at the specified rate */
	ScheduleOnInterval(_call_interval, 10000);
}

void
ADIS16448::stop()
{
	ScheduleClear();
}

void
ADIS16448::Run()
{
	measure();
}

int
ADIS16448::measure()
{
	struct ADISReport adis_report;

	struct Report {
		int16_t		gyro_x;
		int16_t		gyro_y;
		int16_t		gyro_z;
		int16_t		accel_x;
		int16_t		accel_y;
		int16_t		accel_z;
		int16_t		mag_x;
		int16_t		mag_y;
		int16_t		mag_z;
		int16_t		baro;
		int16_t		temp;
	} report;

	/* start measuring */
	perf_begin(_sample_perf);

	/*
	 * Fetch the full set of measurements from the ADIS16448 in one pass (burst read).
	 */

	adis_report.cmd = ((ADIS16448_GLOB_CMD | DIR_READ) << 8) & 0xff00;

	if (OK != transferhword((uint16_t *)&adis_report, ((uint16_t *)&adis_report), sizeof(adis_report) / sizeof(uint16_t))) {
		return -EIO;
	}

	report.gyro_x  = (int16_t) adis_report.gyro_x;
	report.gyro_y  = (int16_t) adis_report.gyro_y;
	report.gyro_z  = (int16_t) adis_report.gyro_z;
	report.accel_x = (int16_t) adis_report.accel_x;
	report.accel_y = (int16_t) adis_report.accel_y;
	report.accel_z = (int16_t) adis_report.accel_z;
	report.mag_x   = (int16_t) adis_report.mag_x;
	report.mag_y   = (int16_t) adis_report.mag_y;
	report.mag_z   = (int16_t) adis_report.mag_z;
	report.baro    = (int16_t) adis_report.baro;
	report.temp    = convert12BitToINT16(adis_report.temp);

	if (report.gyro_x == 0 && report.gyro_y == 0 && report.gyro_z == 0 &&
	    report.accel_x == 0 && report.accel_y == 0 && report.accel_z == 0 &&
	    report.mag_x == 0 && report.mag_y == 0 && report.mag_z == 0 &&
	    report.baro == 0 && report.temp == 0) {
		perf_count(_bad_transfers);
		perf_end(_sample_perf);
		return -EIO;
	}

	/*
	 * Report buffers.
	 */
	sensor_accel_s	arb;
	sensor_gyro_s	grb;
	mag_report		mrb;

	grb.timestamp = arb.timestamp = mrb.timestamp = hrt_absolute_time();
	grb.error_count = arb.error_count = mrb.error_count = perf_event_count(_bad_transfers);

	/* Gyro report: */
	grb.x_raw = report.gyro_x;
	grb.y_raw = report.gyro_y;
	grb.z_raw = report.gyro_z;

	float xraw_f = report.gyro_x;
	float yraw_f = report.gyro_y;
	float zraw_f = report.gyro_z;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	float x_gyro_in_new = ((xraw_f * _gyro_range_scale) * M_PI_F / 180.0f - _gyro_scale.x_offset) * _gyro_scale.x_scale;
	float y_gyro_in_new = ((yraw_f * _gyro_range_scale) * M_PI_F / 180.0f - _gyro_scale.y_offset) * _gyro_scale.y_scale;
	float z_gyro_in_new = ((zraw_f * _gyro_range_scale) * M_PI_F / 180.0f - _gyro_scale.z_offset) * _gyro_scale.z_scale;

	if (FW_FILTER) {
		grb.x = _gyro_filter_x.apply(x_gyro_in_new);
		grb.y = _gyro_filter_y.apply(y_gyro_in_new);
		grb.z = _gyro_filter_z.apply(z_gyro_in_new);

	} else {
		grb.x = x_gyro_in_new;
		grb.y = y_gyro_in_new;
		grb.z = z_gyro_in_new;
	}

	grb.scaling = _gyro_range_scale * M_PI_F / 180.0f;

	/* Accel report: */
	arb.x_raw = report.accel_x;
	arb.y_raw = report.accel_y;
	arb.z_raw = report.accel_z;

	xraw_f = report.accel_x;
	yraw_f = report.accel_y;
	zraw_f = report.accel_z;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	float x_in_new = ((xraw_f * _accel_range_scale) - _accel_scale.x_offset) * _accel_scale.x_scale;
	float y_in_new = ((yraw_f * _accel_range_scale) - _accel_scale.y_offset) * _accel_scale.y_scale;
	float z_in_new = ((zraw_f * _accel_range_scale) - _accel_scale.z_offset) * _accel_scale.z_scale;

	if (FW_FILTER) {
		arb.x = _accel_filter_x.apply(x_in_new);
		arb.y = _accel_filter_y.apply(y_in_new);
		arb.z = _accel_filter_z.apply(z_in_new);

	} else {
		arb.x = x_in_new;
		arb.y = y_in_new;
		arb.z = z_in_new;
	}

	arb.scaling = _accel_range_scale;

	/* Mag report: */
	mrb.x_raw = report.mag_x;
	mrb.y_raw = report.mag_y;
	mrb.z_raw = report.mag_z;

	xraw_f = report.mag_x;
	yraw_f = report.mag_y;
	zraw_f = report.mag_z;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	float x_mag_new = ((xraw_f * _mag_range_scale) - _mag_scale.x_offset) * _mag_scale.x_scale;
	float y_mag_new = ((yraw_f * _mag_range_scale) - _mag_scale.y_offset) * _mag_scale.y_scale;
	float z_mag_new = ((zraw_f * _mag_range_scale) - _mag_scale.z_offset) * _mag_scale.z_scale;

	if (FW_FILTER) {
		mrb.x = _mag_filter_x.apply(x_mag_new) / 1000.0f;
		mrb.y = _mag_filter_y.apply(y_mag_new) / 1000.0f;
		mrb.z = _mag_filter_z.apply(z_mag_new) / 1000.0f;

	} else {
		mrb.x = x_mag_new / 1000.0f;
		mrb.y = y_mag_new / 1000.0f;
		mrb.z = z_mag_new / 1000.0f;
	}

	mrb.scaling  = _mag_range_scale / 1000.0f;

	/* Temperature report: */
	grb.temperature 	= (report.temp * 0.07386f) + 31.0f;
	arb.temperature 	= grb.temperature;

	matrix::Vector3f aval(x_in_new, y_in_new, z_in_new);
	matrix::Vector3f aval_integrated;

	bool accel_notify = _accel_int.put(arb.timestamp, aval, aval_integrated, arb.integral_dt);
	arb.x_integral = aval_integrated(0);
	arb.y_integral = aval_integrated(1);
	arb.z_integral = aval_integrated(2);

	matrix::Vector3f gval(x_gyro_in_new, y_gyro_in_new, z_gyro_in_new);
	matrix::Vector3f gval_integrated;

	bool gyro_notify = _gyro_int.put(grb.timestamp, gval, gval_integrated, grb.integral_dt);
	grb.x_integral = gval_integrated(0);
	grb.y_integral = gval_integrated(1);
	grb.z_integral = gval_integrated(2);

	/* return device ID */
	arb.device_id = _device_id.devid;
	grb.device_id = _gyro->_device_id.devid;
	mrb.device_id = _mag->_device_id.devid;

	_gyro_reports ->force(&grb);
	_accel_reports->force(&arb);
	_mag_reports  ->force(&mrb);

	/* notify anyone waiting for data */
	if (accel_notify) {
		poll_notify(POLLIN);
	}

	if (gyro_notify) {
		_gyro->parent_poll_notify();
	}

	_mag->parent_poll_notify();

	if (accel_notify && !(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(sensor_accel), _accel_topic, &arb);
	}

	if (gyro_notify && !(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(sensor_gyro), _gyro->_gyro_topic, &grb);
	}

	if (!(_pub_blocked) && ((adis_report.status >> 7) & 0x1)) {			/* Mag data validity bit (bit 8 DIAG_STAT) */
		/* publish it */
		orb_publish(ORB_ID(sensor_mag), _mag->_mag_topic, &mrb);
	}

	/* stop measuring */
	perf_end(_sample_perf);
	return OK;
}

void
ADIS16448::print_calibration_data()
{
	uint16_t XGYRO_OFF = read_reg16(ADIS16448_XGYRO_OFF);
	uint16_t YGYRO_OFF = read_reg16(ADIS16448_YGYRO_OFF);
	uint16_t ZGYRO_OFF = read_reg16(ADIS16448_ZGYRO_OFF);
	uint16_t XACCL_OFF = read_reg16(ADIS16448_XACCL_OFF);
	uint16_t YACCL_OFF = read_reg16(ADIS16448_YACCL_OFF);
	uint16_t ZACCL_OFF = read_reg16(ADIS16448_ZACCL_OFF);
	uint16_t XMAGN_HIC = read_reg16(ADIS16448_XMAGN_HIC);
	uint16_t YMAGN_HIC = read_reg16(ADIS16448_YMAGN_HIC);
	uint16_t ZMAGN_HIC = read_reg16(ADIS16448_ZMAGN_HIC);
	uint16_t XMAGN_SIC = read_reg16(ADIS16448_XMAGN_SIC);
	uint16_t YMAGN_SIC = read_reg16(ADIS16448_YMAGN_SIC);
	uint16_t ZMAGN_SIC = read_reg16(ADIS16448_ZMAGN_SIC);

	warnx("single calibration value read:");
	warnx("XGYRO_OFF =:  \t%8.4x\t", XGYRO_OFF);
	warnx("YGYRO_OFF =:  \t%8.4x\t", YGYRO_OFF);
	warnx("ZGYRO_OFF =:  \t%8.4x\t", ZGYRO_OFF);
	warnx("XACCL_OFF =:  \t%8.4x\t", XACCL_OFF);
	warnx("YACCL_OFF =:  \t%8.4x\t", YACCL_OFF);
	warnx("ZACCL_OFF =:  \t%8.4x\t", ZACCL_OFF);
	warnx("XMAGN_HIC =:  \t%8.4x\t", XMAGN_HIC);
	warnx("YMAGN_HIC =:  \t%8.4x\t", YMAGN_HIC);
	warnx("ZMAGN_HIC =:  \t%8.4x\t", ZMAGN_HIC);
	warnx("XMAGN_SIC =:  \t%8.4x\t", XMAGN_SIC);
	warnx("YMAGN_SIC =:  \t%8.4x\t", YMAGN_SIC);
	warnx("ZMAGN_SIC =:  \t%8.4x\t", ZMAGN_SIC);
}

void
ADIS16448::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_accel_reads);
	perf_print_counter(_gyro_reads);
	perf_print_counter(_mag_reads);
	perf_print_counter(_bad_transfers);
	_accel_reports->print_info("accel queue");
	_gyro_reports->print_info("gyro queue");
	_mag_reports->print_info("mag queue");

	printf("DEVICE ID:\nACCEL:\t%d\nGYRO:\t%d\nMAG:\t%d\n", _device_id.devid, _gyro->_device_id.devid,
	       _mag->_device_id.devid);
}

ADIS16448_gyro::ADIS16448_gyro(ADIS16448 *parent, const char *path) :
	CDev("ADIS16448_gyro", path),
	_parent(parent),
	_gyro_topic(nullptr),
	_gyro_orb_class_instance(-1),
	_gyro_class_instance(-1)
{
}

ADIS16448_gyro::~ADIS16448_gyro()
{
	if (_gyro_class_instance != -1) {
		unregister_class_devname(GYRO_BASE_DEVICE_PATH, _gyro_class_instance);
	}
}

int
ADIS16448_gyro::init()
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
ADIS16448_gyro::parent_poll_notify()
{
	poll_notify(POLLIN);
}

ssize_t
ADIS16448_gyro::read(struct file *filp, char *buffer, size_t buflen)
{
	return _parent->gyro_read(filp, buffer, buflen);
}

int
ADIS16448_gyro::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case DEVIOCGDEVICEID:
		return (int)CDev::ioctl(filp, cmd, arg);
		break;

	default:
		return _parent->gyro_ioctl(filp, cmd, arg);
	}
}

ADIS16448_mag::ADIS16448_mag(ADIS16448 *parent, const char *path) :
	CDev("ADIS16448_mag", path),
	_parent(parent),
	_mag_topic(nullptr),
	_mag_orb_class_instance(-1),
	_mag_class_instance(-1)
{
}

ADIS16448_mag::~ADIS16448_mag()
{
	if (_mag_class_instance != -1) {
		unregister_class_devname(MAG_BASE_DEVICE_PATH, _mag_class_instance);
	}
}

int
ADIS16448_mag::init()
{
	int ret;

	// do base class init
	ret = CDev::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("mag init failed");
		return ret;
	}

	_mag_class_instance = register_class_devname(MAG_BASE_DEVICE_PATH);

	return ret;
}

void
ADIS16448_mag::parent_poll_notify()
{
	poll_notify(POLLIN);
}

ssize_t
ADIS16448_mag::read(struct file *filp, char *buffer, size_t buflen)
{
	return _parent->mag_read(filp, buffer, buflen);
}

int
ADIS16448_mag::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case DEVIOCGDEVICEID:
		return (int)CDev::ioctl(filp, cmd, arg);
		break;

	default:
		return _parent->mag_ioctl(filp, cmd, arg);
	}
}

/**
 * Local functions in support of the shell command.
 */
namespace adis16448
{

ADIS16448	*g_dev;

void	start(enum Rotation rotation);
void	test();
void	reset();
void	info();
void 	info_cal();
void	usage();
/**
 * Start the driver.
 */
void
start(enum Rotation rotation)
{
	int fd;

	if (g_dev != nullptr)
		/* if already started, the still command succeeded */
	{
		errx(0, "already started");
	}

	/* create the driver */
#if defined(PX4_SPI_BUS_EXT)
	g_dev = new ADIS16448(PX4_SPI_BUS_EXT, ADIS16448_DEVICE_PATH_ACCEL, ADIS16448_DEVICE_PATH_GYRO,
			      ADIS16448_DEVICE_PATH_MAG, PX4_SPIDEV_EXT_MPU, rotation);
#else
	PX4_ERR("External SPI not available");
	exit(0);
#endif

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != (g_dev)->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(ADIS16448_DEVICE_PATH_ACCEL, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

	close(fd);
	exit(0);
fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	sensor_accel_s a_report{};
	sensor_gyro_s g_report{};
	mag_report 	 m_report;

	ssize_t sz;

	/* get the driver */
	int fd = open(ADIS16448_DEVICE_PATH_ACCEL, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed", ADIS16448_DEVICE_PATH_ACCEL);
	}

	/* get the mag driver */
	int fd_mag = open(ADIS16448_DEVICE_PATH_MAG, O_RDONLY);

	if (fd_mag < 0) {
		err(1, "%s open failed", ADIS16448_DEVICE_PATH_MAG);
	}

	/* get the gyro driver */
	int fd_gyro = open(ADIS16448_DEVICE_PATH_GYRO, O_RDONLY);

	if (fd_gyro < 0) {
		err(1, "%s open failed", ADIS16448_DEVICE_PATH_GYRO);
	}

	/* do a simple demand read */
	sz = read(fd, &a_report, sizeof(a_report));

	if (sz != sizeof(a_report)) {
		warnx("ret: %d, expected: %d", sz, sizeof(a_report));
		err(1, "immediate acc read failed");
	}

	print_message(a_report);

	/* do a simple demand read */
	sz = read(fd_gyro, &g_report, sizeof(g_report));

	if (sz != sizeof(g_report)) {
		warnx("ret: %d, expected: %d", sz, sizeof(g_report));
		err(1, "immediate gyro read failed");
	}

	print_message(g_report);

	/* do a simple mag demand read */
	sz = read(fd_mag, &m_report, sizeof(m_report));

	if (sz != sizeof(m_report)) {
		warnx("ret: %d, expected: %d", sz, sizeof(m_report));
		err(1, "immediate mag read failed");
	}

	print_message(m_report);

	/* XXX add poll-rate tests here too */
	close(fd_mag);
	close(fd_gyro);
	close(fd);

	reset();
	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(ADIS16448_DEVICE_PATH_ACCEL, O_RDONLY);

	if (fd < 0) {
		err(1, "open failed");
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
info()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

/**
 * Print a sensor calibration info.
 */
void
info_cal()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	g_dev->print_calibration_data();

	exit(0);
}

void
usage()
{
	warnx("missing command: try 'start', 'test', 'info', 'info_cal', 'reset',\n");
	warnx("options:");
	warnx("    -R rotation");
}

}
// namespace

int
adis16448_main(int argc, char *argv[])
{
	enum Rotation rotation = ROTATION_NONE;
	int ch;

	/* start options */
	while ((ch = getopt(argc, argv, "R:")) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(optarg);
			break;

		default:
			adis16448::usage();
			exit(0);
		}
	}

	const char *verb = argv[optind];

	/*
	 * Start/load the driver.

	 */
	if (!strcmp(verb, "start")) {
		adis16448::start(rotation);
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		adis16448::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		adis16448::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		adis16448::info();
	}

	/*
	 * Print sensor calibration information.
	 */
	if (!strcmp(verb, "info_cal")) {
		adis16448::info_cal();
	}

	adis16448::usage();
	exit(1);
}
