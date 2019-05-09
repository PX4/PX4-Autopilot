/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file gyrosim.cpp
 *
 * Driver for the simulated gyro
 *
 * @author Andrew Tridgell
 * @author Pat Hickey
 * @author Mark Charlebois
 */

#include <inttypes.h>

#include <px4_config.h>
#include <px4_getopt.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <simulator/simulator.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/conversions.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/device.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>

#include "VirtDevObj.hpp"

using namespace DriverFramework;

#define DIR_READ			0x80
#define DIR_WRITE			0x00

#define MPU_DEVICE_PATH_ACCEL		"/dev/gyrosim_accel"
#define MPU_DEVICE_PATH_GYRO		"/dev/gyrosim_gyro"

// MPU 6000 registers
#define MPUREG_SMPLRT_DIV		0x19
#define MPUREG_CONFIG			0x1A
#define MPUREG_GYRO_CONFIG		0x1B
#define MPUREG_ACCEL_CONFIG		0x1C
#define MPUREG_INT_STATUS		0x3A
#define MPUREG_PRODUCT_ID		0x0C

// Product ID Description for GYROSIM
// high 4 bits 	low 4 bits
// Product Name	Product Revision
#define GYROSIMES_REV_C4		0x14

#define GYROSIM_ACCEL_DEFAULT_RATE	400

#define GYROSIM_GYRO_DEFAULT_RATE	400

#ifdef PX4_SPI_BUS_EXT
#define EXTERNAL_BUS PX4_SPI_BUS_EXT
#else
#define EXTERNAL_BUS 0
#endif

/*
  the GYROSIM can only handle high SPI bus speeds on the sensor and
  interrupt status registers. All other registers have a maximum 1MHz
  SPI speed
 */
class GYROSIM_gyro;

class GYROSIM : public VirtDevObj
{
public:
	GYROSIM(const char *path_accel, const char *path_gyro, enum Rotation rotation);
	virtual ~GYROSIM();

	int             	init();
	virtual int		start();

	virtual ssize_t		devRead(void *buffer, size_t buflen);
	virtual int		devIOCTL(unsigned long cmd, unsigned long arg);
	int			transfer(uint8_t *send, uint8_t *recv, unsigned len);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

	void			print_registers();

protected:
	friend class GYROSIM_gyro;

	virtual ssize_t		gyro_read(void *buffer, size_t buflen);
	virtual int		gyro_ioctl(unsigned long cmd, unsigned long arg);

private:
	GYROSIM_gyro		*_gyro;
	uint8_t			_product;	/** product code */

	unsigned		_call_interval;

	ringbuffer::RingBuffer	*_accel_reports;

	struct accel_calibration_s	_accel_scale;
	float			_accel_range_scale;
	float			_accel_range_m_s2;
	orb_advert_t		_accel_topic;
	int			_accel_orb_class_instance;

	ringbuffer::RingBuffer	*_gyro_reports;

	struct gyro_calibration_s	_gyro_scale;
	float			_gyro_range_scale;

	perf_counter_t		_accel_reads;
	perf_counter_t		_gyro_reads;
	perf_counter_t		_sample_perf;
	perf_counter_t		_good_transfers;
	perf_counter_t		_reset_retries;

	Integrator _accel_int;
	Integrator _gyro_int;

	// last temperature reading for print_info()
	float			_last_temperature;

	/**
	 * Reset chip.
	 *
	 * Resets the chip and measurements ranges, but not scale and offset.
	 */
	int			reset();

	/**
	 * Fetch measurements from the sensor and update the report buffers.
	 */
	virtual void		_measure();

	/**
	 * Read a register from the GYROSIM
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint8_t			read_reg(unsigned reg);

	/**
	 * Write a register in the GYROSIM
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_reg(unsigned reg, uint8_t value);

	/**
	 * Set the GYROSIM measurement range.
	 *
	 * @param max_g		The maximum G value the range must support.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			set_accel_range(unsigned max_g);

	/**
	 * Swap a 16-bit value read from the GYROSIM to native byte order.
	 */
	uint16_t		swap16(uint16_t val) { return (val >> 8) | (val << 8);	}

	/**
	 * Measurement self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int 			self_test();

	/*
	  set sample rate (approximate) - 1kHz to 5Hz
	*/
	void _set_sample_rate(unsigned desired_sample_rate_hz);

	/* do not allow to copy this class due to pointer data members */
	GYROSIM(const GYROSIM &) = delete;
	GYROSIM operator=(const GYROSIM &) = delete;

#pragma pack(push, 1)
	/**
	 * Report conversation within the GYROSIM, including command byte and
	 * interrupt status.
	 */
	struct MPUReport {
		uint8_t		cmd;
		uint8_t		status;
		float		accel_x;
		float		accel_y;
		float		accel_z;
		float		temp;
		float		gyro_x;
		float		gyro_y;
		float		gyro_z;
	};
#pragma pack(pop)

	uint8_t _regdata[108];

};

/**
 * Helper class implementing the gyro driver node.
 */
class GYROSIM_gyro  : public VirtDevObj
{
public:
	GYROSIM_gyro(GYROSIM *parent, const char *path);
	virtual ~GYROSIM_gyro() = default;

	virtual ssize_t		devRead(void *buffer, size_t buflen);
	virtual int		devIOCTL(unsigned long cmd, unsigned long arg);

	virtual int		init();

protected:
	friend class GYROSIM;

	virtual void 		_measure() {}
private:
	GYROSIM			*_parent;
	orb_advert_t		_gyro_topic;
	int			_gyro_orb_class_instance;

	/* do not allow to copy this class due to pointer data members */
	GYROSIM_gyro(const GYROSIM_gyro &) = delete;
	GYROSIM_gyro operator=(const GYROSIM_gyro &) = delete;
};

/** driver 'main' command */
extern "C" { __EXPORT int gyrosim_main(int argc, char *argv[]); }

GYROSIM::GYROSIM(const char *path_accel, const char *path_gyro, enum Rotation rotation) :
	VirtDevObj("GYROSIM", path_accel, ACCEL_BASE_DEVICE_PATH, 1e6 / 400),
	_gyro(new GYROSIM_gyro(this, path_gyro)),
	_product(GYROSIMES_REV_C4),
	_accel_reports(nullptr),
	_accel_scale{},
	_accel_range_scale(0.0f),
	_accel_range_m_s2(0.0f),
	_accel_topic(nullptr),
	_accel_orb_class_instance(-1),
	_gyro_reports(nullptr),
	_gyro_scale{},
	_gyro_range_scale(0.0f),
	_accel_reads(perf_alloc(PC_COUNT, "gyrosim_accel_read")),
	_gyro_reads(perf_alloc(PC_COUNT, "gyrosim_gyro_read")),
	_sample_perf(perf_alloc(PC_ELAPSED, "gyrosim_read")),
	_good_transfers(perf_alloc(PC_COUNT, "gyrosim_good_transfers")),
	_reset_retries(perf_alloc(PC_COUNT, "gyrosim_reset_retries")),
	_accel_int(1000000 / GYROSIM_ACCEL_DEFAULT_RATE, true),
	_gyro_int(1000000 / GYROSIM_GYRO_DEFAULT_RATE, true),
	_last_temperature(0)
{

	m_id.dev_id_s.bus = 1;
	m_id.dev_id_s.devtype = DRV_ACC_DEVTYPE_GYROSIM;

	/* Prime _gyro with parents devid. */
	_gyro->m_id.dev_id = m_id.dev_id;
	_gyro->m_id.dev_id_s.devtype = DRV_GYR_DEVTYPE_GYROSIM;

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
}

GYROSIM::~GYROSIM()
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

	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_accel_reads);
	perf_free(_gyro_reads);
	perf_free(_good_transfers);
}

int
GYROSIM::init()
{
	int ret = VirtDevObj::init();

	if (ret != 0) {
		PX4_WARN("Base class init failed");
		return ret;
	}

	/* allocate basic report buffers */
	_accel_reports = new ringbuffer::RingBuffer(2, sizeof(sensor_accel_s));

	if (_accel_reports == nullptr) {
		PX4_WARN("_accel_reports creation failed");
		return -ENOMEM;
	}

	_gyro_reports = new ringbuffer::RingBuffer(2, sizeof(sensor_gyro_s));

	if (_gyro_reports == nullptr) {
		PX4_WARN("_gyro_reports creation failed");
		return -ENOMEM;
	}

	if (reset() != OK) {
		PX4_WARN("reset failed");
		return PX4_ERROR;
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

	/* do init for the gyro device node, keep it optional */
	ret = _gyro->init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		PX4_ERR("gyro init failed");
		return ret;
	}

	ret = start();

	if (ret != OK) {
		PX4_ERR("gyro accel start failed (%d)", ret);
		return ret;
	}

	return PX4_OK;
}

int GYROSIM::reset()
{
	return OK;
}

int
GYROSIM::transfer(uint8_t *send, uint8_t *recv, unsigned len)
{
	uint8_t cmd = send[0];
	uint8_t reg = cmd & 0x7F;
	const uint8_t MPUREAD = MPUREG_INT_STATUS | DIR_READ;

	if (cmd == MPUREAD) {
		// Get data from the simulator
		Simulator *sim = Simulator::getInstance();

		if (sim == nullptr) {
			PX4_WARN("failed accessing simulator");
			return ENODEV;
		}

		// FIXME - not sure what interrupt status should be
		recv[1] = 0;

		// skip cmd and status bytes
		if (len > 2) {
			sim->getMPUReport(&recv[2], len - 2);
		}

	} else if (cmd & DIR_READ) {
		PX4_DEBUG("Reading %u bytes from register %u", len - 1, reg);
		memcpy(&_regdata[reg - MPUREG_PRODUCT_ID], &send[1], len - 1);

	} else {
		PX4_DEBUG("Writing %u bytes to register %u", len - 1, reg);

		if (recv) {
			memcpy(&recv[1], &_regdata[reg - MPUREG_PRODUCT_ID], len - 1);
		}
	}

	return PX4_OK;
}

/*
  set sample rate (approximate) - 1kHz to 5Hz, for both accel and gyro
*/
void
GYROSIM::_set_sample_rate(unsigned desired_sample_rate_hz)
{
	PX4_DEBUG("_set_sample_rate %u Hz", desired_sample_rate_hz);

	if (desired_sample_rate_hz == 0) {
		desired_sample_rate_hz = GYROSIM_GYRO_DEFAULT_RATE;
	}

	uint8_t div = 1000 / desired_sample_rate_hz;

	if (div > 200) { div = 200; }

	if (div < 1) { div = 1; }

	// This does nothing in the simulator but writes the value in the "register" so
	// register dumps look correct
	write_reg(MPUREG_SMPLRT_DIV, div - 1);

	unsigned sample_rate = 1000 / div;
	PX4_DEBUG("Changed sample rate to %uHz", sample_rate);
	setSampleInterval(1000000 / sample_rate);
	_gyro->setSampleInterval(1000000 / sample_rate);
}

ssize_t
GYROSIM::devRead(void *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(sensor_accel_s);

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
	if (_call_interval == 0) {
		_accel_reports->flush();
		_measure();
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
GYROSIM::self_test()
{
	if (perf_event_count(_sample_perf) == 0) {
		_measure();
	}

	/* return 0 on success, 1 else */
	return (perf_event_count(_sample_perf) > 0) ? 0 : 1;
}

ssize_t
GYROSIM::gyro_read(void *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(sensor_gyro_s);

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
	if (_call_interval == 0) {
		_gyro_reports->flush();
		_measure();
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

int
GYROSIM::devIOCTL(unsigned long cmd, unsigned long arg)
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
				return devIOCTL(SENSORIOCSPOLLRATE, GYROSIM_ACCEL_DEFAULT_RATE);

			/* adjust to a legal polling interval in Hz */
			default: {
					/* convert hz to hrt interval via microseconds */
					unsigned ticks = 1000000 / arg;

					/* check against maximum sane rate */
					if (ticks < 1000) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_call_interval = ticks;

					/* do we need to start internal polling? */
					bool want_start = (_call_interval == 0);

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
		return VirtDevObj::devIOCTL(cmd, arg);
	}
}

int
GYROSIM::gyro_ioctl(unsigned long cmd, unsigned long arg)
{
	switch (cmd) {

	/* these are shared with the accel side */
	case SENSORIOCSPOLLRATE:
	case SENSORIOCRESET:
		return devIOCTL(cmd, arg);

	case GYROIOCSSCALE:
		/* copy scale in */
		memcpy(&_gyro_scale, (struct gyro_calibration_s *) arg, sizeof(_gyro_scale));
		return OK;

	default:
		/* give it to the superclass */
		return VirtDevObj::devIOCTL(cmd, arg);
	}
}

uint8_t
GYROSIM::read_reg(unsigned reg)
{
	uint8_t cmd[2] = { (uint8_t)(reg | DIR_READ), 0};

	// general register transfer at low clock speed
	transfer(cmd, cmd, sizeof(cmd));

	return cmd[1];
}

void
GYROSIM::write_reg(unsigned reg, uint8_t value)
{
	uint8_t	cmd[2];

	cmd[0] = reg | DIR_WRITE;
	cmd[1] = value;

	// general register transfer at low clock speed
	transfer(cmd, nullptr, sizeof(cmd));
}

int
GYROSIM::set_accel_range(unsigned max_g_in)
{
	// workaround for bugged versions of MPU6k (rev C)
	switch (_product) {
	case GYROSIMES_REV_C4:
		write_reg(MPUREG_ACCEL_CONFIG, 1 << 3);
		_accel_range_scale = (CONSTANTS_ONE_G / 4096.0f);
		_accel_range_m_s2 = 8.0f * CONSTANTS_ONE_G;
		return OK;
	}

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

	write_reg(MPUREG_ACCEL_CONFIG, afs_sel << 3);
	_accel_range_scale = (CONSTANTS_ONE_G / lsb_per_g);
	_accel_range_m_s2 = max_accel_g * CONSTANTS_ONE_G;

	return OK;
}

int
GYROSIM::start()
{
	/* make sure we are stopped first */
	stop();

	/* discard any stale data in the buffers */
	_accel_reports->flush();
	_gyro_reports->flush();

	/* start polling at the specified rate */
	return DevObj::start();
}

void
GYROSIM::_measure()
{

#if 0
	static int x = 0;

	// Verify the samples are being taken at the expected rate
	if (x == 99) {
		x = 0;
		PX4_INFO("GYROSIM::measure %" PRIu64, hrt_absolute_time());

	} else {
		x++;
	}

#endif
	struct MPUReport mpu_report = {};

	/* start measuring */
	perf_begin(_sample_perf);

	/*
	 * Fetch the full set of measurements from the GYROSIM in one pass.
	 */
	mpu_report.cmd = DIR_READ | MPUREG_INT_STATUS;

	// sensor transfer at high clock speed
	//set_frequency(GYROSIM_HIGH_BUS_SPEED);
	if (OK != transfer((uint8_t *)&mpu_report, ((uint8_t *)&mpu_report), sizeof(mpu_report))) {
		return;
	}

	/*
	 * Report buffers.
	 */
	sensor_accel_s	arb = {};
	sensor_gyro_s	grb = {};

	// for now use local time but this should be the timestamp of the simulator
	grb.timestamp = hrt_absolute_time();
	arb.timestamp = grb.timestamp;
	// report the error count as the sum of the number of bad
	// transfers and bad register reads. This allows the higher
	// level code to decide if it should use this sensor based on
	// whether it has had failures
	grb.error_count = arb.error_count = 0;	// FIXME

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

	if (math::isZero(_accel_range_scale)) {
		_accel_range_scale = FLT_EPSILON;
	}

	arb.x_raw = math::constrainFloatToInt16(mpu_report.accel_x / _accel_range_scale);
	arb.y_raw = math::constrainFloatToInt16(mpu_report.accel_y / _accel_range_scale);
	arb.z_raw = math::constrainFloatToInt16(mpu_report.accel_z / _accel_range_scale);

	arb.scaling = _accel_range_scale;

	_last_temperature = mpu_report.temp;

	arb.temperature = _last_temperature;

	arb.x = mpu_report.accel_x;
	arb.y = mpu_report.accel_y;
	arb.z = mpu_report.accel_z;

	matrix::Vector3f aval(mpu_report.accel_x, mpu_report.accel_y, mpu_report.accel_z);
	matrix::Vector3f aval_integrated;

	bool accel_notify = _accel_int.put(arb.timestamp, aval, aval_integrated, arb.integral_dt);
	arb.x_integral = aval_integrated(0);
	arb.y_integral = aval_integrated(1);
	arb.z_integral = aval_integrated(2);

	/* fake device ID */
	arb.device_id = 1376264;

	if (math::isZero(_gyro_range_scale)) {
		_gyro_range_scale = FLT_EPSILON;
	}

	grb.x_raw = math::constrainFloatToInt16(mpu_report.gyro_x / _gyro_range_scale);
	grb.y_raw = math::constrainFloatToInt16(mpu_report.gyro_y / _gyro_range_scale);
	grb.z_raw = math::constrainFloatToInt16(mpu_report.gyro_z / _gyro_range_scale);

	grb.scaling = _gyro_range_scale;

	grb.temperature = _last_temperature;

	grb.x = mpu_report.gyro_x;
	grb.y = mpu_report.gyro_y;
	grb.z = mpu_report.gyro_z;

	matrix::Vector3f gval(mpu_report.gyro_x, mpu_report.gyro_y, mpu_report.gyro_z);
	matrix::Vector3f gval_integrated;

	bool gyro_notify = _gyro_int.put(grb.timestamp, gval, gval_integrated, grb.integral_dt);
	grb.x_integral = gval_integrated(0);
	grb.y_integral = gval_integrated(1);
	grb.z_integral = gval_integrated(2);

	/* fake device ID */
	grb.device_id = 2293768;

	_accel_reports->force(&arb);
	_gyro_reports->force(&grb);

	if (accel_notify) {
		orb_publish_auto(ORB_ID(sensor_accel), &_accel_topic, &arb, &_accel_orb_class_instance, ORB_PRIO_HIGH);
	}

	if (gyro_notify) {
		orb_publish_auto(ORB_ID(sensor_gyro), &_gyro->_gyro_topic, &grb, &_gyro->_gyro_orb_class_instance, ORB_PRIO_HIGH);
	}

	/* stop measuring */
	perf_end(_sample_perf);
}

void
GYROSIM::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_accel_reads);
	perf_print_counter(_gyro_reads);
	perf_print_counter(_good_transfers);
	perf_print_counter(_reset_retries);
	_accel_reports->print_info("accel queue");
	_gyro_reports->print_info("gyro queue");
	PX4_INFO("temperature: %.1f", (double)_last_temperature);
}

void
GYROSIM::print_registers()
{
	char buf[6 * 13 + 1];
	int i = 0;

	buf[0] = '\0';
	PX4_INFO("GYROSIM registers");

	for (uint8_t reg = MPUREG_PRODUCT_ID; reg <= 108; reg++) {
		uint8_t v = read_reg(reg);
		sprintf(&buf[i * 6], "%02x:%02x ", (unsigned)reg, (unsigned)v);
		i++;

		if ((i + 1) % 13 == 0) {
			PX4_INFO("%s", buf);
			i = 0;
			buf[i] = '\0';
		}
	}

	PX4_INFO("%s", buf);
}


GYROSIM_gyro::GYROSIM_gyro(GYROSIM *parent, const char *path) :
	// Set sample interval to 0 since device is read by parent
	VirtDevObj("GYROSIM_gyro", path, GYRO_BASE_DEVICE_PATH, 0),
	_parent(parent),
	_gyro_topic(nullptr),
	_gyro_orb_class_instance(-1)
{
}


int
GYROSIM_gyro::init()
{
	int ret = VirtDevObj::init();
	PX4_DEBUG("GYROSIM_gyro::init base class ret: %d", ret);
	return ret;
}

ssize_t
GYROSIM_gyro::devRead(void *buffer, size_t buflen)
{
	return _parent->gyro_read(buffer, buflen);
}

int
GYROSIM_gyro::devIOCTL(unsigned long cmd, unsigned long arg)
{

	switch (cmd) {
	case DEVIOCGDEVICEID:
		return (int)VirtDevObj::devIOCTL(cmd, arg);
		break;

	default:
		return _parent->gyro_ioctl(cmd, arg);
	}
}

/**
 * Local functions in support of the shell command.
 */
namespace gyrosim
{

GYROSIM	*g_dev_sim; // on simulated bus

int	start(enum Rotation /*rotation*/);
int	stop();
int	test();
int	reset();
int	info();
int	regdump();
void	usage();

/**
 * Start the driver.
 *
 * This function only returns if the driver is up and running
 * or failed to detect the sensor.
 */
int
start(enum Rotation rotation)
{
	GYROSIM **g_dev_ptr = &g_dev_sim;
	const char *path_accel = MPU_DEVICE_PATH_ACCEL;
	const char *path_gyro  = MPU_DEVICE_PATH_GYRO;
	DevHandle h;

	if (*g_dev_ptr != nullptr) {
		/* if already started, the still command succeeded */
		PX4_WARN("already started");
		return 0;
	}

	/* create the driver */
	*g_dev_ptr = new GYROSIM(path_accel, path_gyro, rotation);

	if (*g_dev_ptr == nullptr) {
		goto fail;
	}

	if (OK != (*g_dev_ptr)->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	DevMgr::getHandle(path_accel, h);

	if (!h.isValid()) {
		goto fail;
	}

	if (h.ioctl(SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		DevMgr::releaseHandle(h);
		goto fail;
	}

	DevMgr::releaseHandle(h);
	return 0;
fail:

	if (*g_dev_ptr != nullptr) {
		delete *g_dev_ptr;
		*g_dev_ptr = nullptr;
	}

	PX4_WARN("driver start failed");
	return 1;
}

int
stop()
{
	GYROSIM **g_dev_ptr = &g_dev_sim;

	if (*g_dev_ptr != nullptr) {
		delete *g_dev_ptr;
		*g_dev_ptr = nullptr;

	} else {
		/* warn, but not an error */
		PX4_WARN("already stopped.");
	}

	return 0;
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
int
test()
{
	const char *path_accel = MPU_DEVICE_PATH_ACCEL;
	const char *path_gyro  = MPU_DEVICE_PATH_GYRO;
	sensor_accel_s a_report;
	sensor_gyro_s g_report;
	ssize_t sz;

	/* get the driver */
	DevHandle h_accel;
	DevMgr::getHandle(path_accel, h_accel);

	if (!h_accel.isValid()) {
		PX4_ERR("%s open failed (try 'gyrosim start')", path_accel);
		return 1;
	}

	/* get the driver */
	DevHandle h_gyro;
	DevMgr::getHandle(path_gyro, h_gyro);

	if (!h_gyro.isValid()) {
		PX4_ERR("%s open failed", path_gyro);
		return 1;
	}

	/* do a simple demand read */
	sz = h_accel.read(&a_report, sizeof(a_report));

	if (sz != sizeof(a_report)) {
		PX4_WARN("ret: %zd, expected: %zd", sz, sizeof(a_report));
		PX4_ERR("immediate acc read failed");
		return 1;
	}

	PX4_INFO("single read");
	PX4_INFO("time:     %lld", (long long)a_report.timestamp);
	PX4_INFO("acc  x:  \t%8.4f\tm/s^2", (double)a_report.x);
	PX4_INFO("acc  y:  \t%8.4f\tm/s^2", (double)a_report.y);
	PX4_INFO("acc  z:  \t%8.4f\tm/s^2", (double)a_report.z);
	PX4_INFO("acc  x:  \t%d\traw 0x%0x", (short)a_report.x_raw, (unsigned short)a_report.x_raw);
	PX4_INFO("acc  y:  \t%d\traw 0x%0x", (short)a_report.y_raw, (unsigned short)a_report.y_raw);
	PX4_INFO("acc  z:  \t%d\traw 0x%0x", (short)a_report.z_raw, (unsigned short)a_report.z_raw);

	/* do a simple demand read */
	sz = h_gyro.read(&g_report, sizeof(g_report));

	if (sz != sizeof(g_report)) {
		PX4_WARN("ret: %zd, expected: %zd", sz, sizeof(g_report));
		PX4_ERR("immediate gyro read failed");
		return 1;
	}

	PX4_INFO("gyro x: \t% 9.5f\trad/s", (double)g_report.x);
	PX4_INFO("gyro y: \t% 9.5f\trad/s", (double)g_report.y);
	PX4_INFO("gyro z: \t% 9.5f\trad/s", (double)g_report.z);
	PX4_INFO("gyro x: \t%d\traw", (int)g_report.x_raw);
	PX4_INFO("gyro y: \t%d\traw", (int)g_report.y_raw);
	PX4_INFO("gyro z: \t%d\traw", (int)g_report.z_raw);

	PX4_INFO("temp:  \t%8.4f\tdeg celsius", (double)a_report.temperature);


	/* XXX add poll-rate tests here too */

	// Destructor would clean the up too
	DevMgr::releaseHandle(h_accel);
	DevMgr::releaseHandle(h_gyro);
	reset();
	PX4_INFO("PASS");

	return 0;
}

/**
 * Reset the driver.
 */
int
reset()
{
	DevHandle h;
	DevMgr::getHandle(MPU_DEVICE_PATH_ACCEL, h);

	if (!h.isValid()) {
		PX4_ERR("reset failed");
		return 1;
	}


	if (h.ioctl(SENSORIOCRESET, 0) < 0) {
		PX4_ERR("driver reset failed");
		goto reset_fail;
	}

	if (h.ioctl(SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("driver poll restart failed");
		goto reset_fail;
	}

	return 0;

reset_fail:
	return 1;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	GYROSIM **g_dev_ptr = &g_dev_sim;

	if (*g_dev_ptr == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	PX4_INFO("state @ %p", *g_dev_ptr);
	(*g_dev_ptr)->print_info();
	unsigned dummy = 0;
	PX4_INFO("device_id: %u", (unsigned int)(*g_dev_ptr)->devIOCTL(DEVIOCGDEVICEID, dummy));

	return 0;
}

/**
 * Dump the register information
 */
int
regdump()
{
	GYROSIM **g_dev_ptr = &g_dev_sim;

	if (*g_dev_ptr == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	PX4_INFO("regdump @ %p", *g_dev_ptr);
	(*g_dev_ptr)->print_registers();

	return 0;
}

void
usage()
{
	PX4_INFO("missing command: try 'start', 'info', 'test', 'stop', 'reset', 'regdump'");
	PX4_INFO("options:");
	PX4_INFO("    -R rotation");
}

} // namespace

int
gyrosim_main(int argc, char *argv[])
{
	int ch;
	enum Rotation rotation = ROTATION_NONE;
	int ret;

	/* jump over start/off/etc and look at options first */
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "R:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;

		default:
			gyrosim::usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		gyrosim::usage();
		return 1;
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.

	 */
	if (!strcmp(verb, "start")) {
		ret = gyrosim::start(rotation);
	}

	else if (!strcmp(verb, "stop")) {
		ret = gyrosim::stop();
	}

	/*
	 * Test the driver/device.
	 */
	else if (!strcmp(verb, "test")) {
		ret = gyrosim::test();
	}

	/*
	 * Reset the driver.
	 */
	else if (!strcmp(verb, "reset")) {
		ret = gyrosim::reset();
	}

	/*
	 * Print driver information.
	 */
	else if (!strcmp(verb, "info")) {
		ret = gyrosim::info();
	}

	/*
	 * Print register information.
	 */
	else if (!strcmp(verb, "regdump")) {
		ret = gyrosim::regdump();
	}

	else  {
		gyrosim::usage();
		ret = 1;
	}

	return ret;
}
