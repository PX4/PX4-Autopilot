/****************************************************************************
 *
 * Copyright (c) 2014-2015 PX4 Development Team. All rights reserved.
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
 * @file accelsim.cpp
 * Driver for a simulated accelerometer / magnetometer.
 */

#include <px4_config.h>

#include <sys/types.h>
#include <sys/stat.h>
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
#include <px4_getopt.h>

#include <simulator/simulator.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/device/device.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_tone_alarm.h>

#include <board_config.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#define ACCELSIM_DEVICE_PATH_ACCEL	"/dev/sim_accel"
#define ACCELSIM_DEVICE_PATH_ACCEL_EXT	"/dev/sim_accel_ext"
#define ACCELSIM_DEVICE_PATH_MAG	"/dev/sim_mag"

#define ADDR_WHO_AM_I			0x0F

#define ACCELSIM_ACCEL_DEFAULT_RATE			800
#define ACCELSIM_ACCEL_DEFAULT_DRIVER_FILTER_FREQ	30
#define ACCELSIM_ONE_G					9.80665f

#define DIR_READ				(1<<7)
#define DIR_WRITE				(0<<7)
#define ACC_READ 				(0<<6)
#define MAG_READ 				(1<<6)

extern "C" { __EXPORT int accelsim_main(int argc, char *argv[]); }


class ACCELSIM_mag;

class ACCELSIM : public device::VDev
{
public:
	ACCELSIM(const char* path, enum Rotation rotation);
	virtual ~ACCELSIM();

	virtual int		init();

	virtual ssize_t		read(device::file_t *filp, char *buffer, size_t buflen);
	virtual int		ioctl(device::file_t *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	//void			print_info();

	/**
	 * dump register values
	 */
	void			print_registers();

protected:
	friend class 		ACCELSIM_mag;

	virtual ssize_t		mag_read(device::file_t *filp, char *buffer, size_t buflen);
	virtual int		mag_ioctl(device::file_t *filp, int cmd, unsigned long arg);

	int			transfer(uint8_t *send, uint8_t *recv, unsigned len);
private:

	ACCELSIM_mag		*_mag;

	struct hrt_call		_accel_call;
	struct hrt_call		_mag_call;

	unsigned		_call_accel_interval;
	unsigned		_call_mag_interval;

	ringbuffer::RingBuffer		*_accel_reports;
	ringbuffer::RingBuffer		*_mag_reports;

	struct accel_scale	_accel_scale;
	unsigned		_accel_range_m_s2;
	float			_accel_range_scale;
	unsigned		_accel_samplerate;
	unsigned		_accel_onchip_filter_bandwith;

	struct mag_scale	_mag_scale;
	unsigned		_mag_range_ga;
	float			_mag_range_scale;
	unsigned		_mag_samplerate;

	orb_advert_t		_accel_topic;
	int			_accel_orb_class_instance;
	int			_accel_class_instance;

	unsigned		_accel_read;
	unsigned		_mag_read;

	perf_counter_t		_accel_sample_perf;
	perf_counter_t		_mag_sample_perf;
	perf_counter_t		_accel_reschedules;
	perf_counter_t		_bad_registers;
	perf_counter_t		_bad_values;

	math::LowPassFilter2p	_accel_filter_x;
	math::LowPassFilter2p	_accel_filter_y;
	math::LowPassFilter2p	_accel_filter_z;

	enum Rotation		_rotation;

	// values used to
	float			_last_accel[3];
	uint8_t			_constant_accel_count;

	// last temperature value
	float			_last_temperature;

	// this is used to support runtime checking of key
	// configuration registers to detect SPI bus errors and sensor
	// reset
#define ACCELSIM_NUM_CHECKED_REGISTERS 1
	static const uint8_t	_checked_registers[ACCELSIM_NUM_CHECKED_REGISTERS];
	uint8_t			_checked_values[ACCELSIM_NUM_CHECKED_REGISTERS];
	uint8_t			_checked_next;

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
	void			reset();

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
	 * Static trampoline for the mag because it runs at a lower rate
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		mag_measure_trampoline(void *arg);

	/**
	 * Fetch accel measurements from the sensor and update the report ring.
	 */
	void			measure();

	/**
	 * Fetch mag measurements from the sensor and update the report ring.
	 */
	void			mag_measure();

	/**
	 * Read a register from the ACCELSIM
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint8_t			read_reg(unsigned reg);

	/**
	 * Write a register in the ACCELSIM
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_reg(unsigned reg, uint8_t value);

	/**
	 * Modify a register in the ACCELSIM
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Write a register in the ACCELSIM, updating _checked_values
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_checked_reg(unsigned reg, uint8_t value);

	/**
	 * Set the ACCELSIM accel measurement range.
	 *
	 * @param max_g	The measurement range of the accel is in g (9.81m/s^2)
	 *			Zero selects the maximum supported range.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			accel_set_range(unsigned max_g);

	/**
	 * Set the ACCELSIM mag measurement range.
	 *
	 * @param max_ga	The measurement range of the mag is in Ga
	 *			Zero selects the maximum supported range.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			mag_set_range(unsigned max_g);

	/**
	 * Set the driver lowpass filter bandwidth.
	 *
	 * @param bandwidth The anti-alias filter bandwidth in Hz
	 * 			Zero selects the highest bandwidth
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			accel_set_driver_lowpass_filter(float samplerate, float bandwidth);

	/**
	 * Set the ACCELSIM internal accel sampling frequency.
	 *
	 * @param frequency	The internal accel sampling frequency is set to not less than
	 *			this value.
	 *			Zero selects the maximum rate supported.
	 * @return		OK if the value can be supported.
	 */
	int			accel_set_samplerate(unsigned frequency);

	/**
	 * Set the ACCELSIM internal mag sampling frequency.
	 *
	 * @param frequency	The internal mag sampling frequency is set to not less than
	 *			this value.
	 *			Zero selects the maximum rate supported.
	 * @return		OK if the value can be supported.
	 */
	int			mag_set_samplerate(unsigned frequency);

	/* this class cannot be copied */
	ACCELSIM(const ACCELSIM&);
	ACCELSIM operator=(const ACCELSIM&);
};

/*
  list of registers that will be checked in check_registers(). Note
  that ADDR_WHO_AM_I must be first in the list.
 */
const uint8_t ACCELSIM::_checked_registers[ACCELSIM_NUM_CHECKED_REGISTERS] = { ADDR_WHO_AM_I };

/**
 * Helper class implementing the mag driver node.
 */
class ACCELSIM_mag : public device::VDev
{
public:
	ACCELSIM_mag(ACCELSIM *parent);
	~ACCELSIM_mag();

	virtual ssize_t			read(device::file_t *filp, char *buffer, size_t buflen);
	virtual int			ioctl(device::file_t *filp, int cmd, unsigned long arg);

	virtual int			init();

protected:
	friend class ACCELSIM;

	void				parent_poll_notify();
private:
	ACCELSIM				*_parent;

	orb_advert_t			_mag_topic;
	int				_mag_orb_class_instance;
	int				_mag_class_instance;

	void				measure();

	void				measure_trampoline(void *arg);

	/* this class does not allow copying due to ptr data members */
	ACCELSIM_mag(const ACCELSIM_mag&);
	ACCELSIM_mag operator=(const ACCELSIM_mag&);
};


ACCELSIM::ACCELSIM(const char* path, enum Rotation rotation) :
	VDev("ACCELSIM", path),
	_mag(new ACCELSIM_mag(this)),
	_accel_call{},
	_mag_call{},
	_call_accel_interval(0),
	_call_mag_interval(0),
	_accel_reports(nullptr),
	_mag_reports(nullptr),
	_accel_scale{},
	_accel_range_m_s2(0.0f),
	_accel_range_scale(0.0f),
	_accel_samplerate(0),
	_accel_onchip_filter_bandwith(0),
	_mag_scale{},
	_mag_range_ga(0.0f),
	_mag_range_scale(0.0f),
	_mag_samplerate(0),
	_accel_topic(nullptr),
	_accel_orb_class_instance(-1),
	_accel_class_instance(-1),
	_accel_read(0),
	_mag_read(0),
	_accel_sample_perf(perf_alloc(PC_ELAPSED, "sim_accel_read")),
	_mag_sample_perf(perf_alloc(PC_ELAPSED, "sim_mag_read")),
	_accel_reschedules(perf_alloc(PC_COUNT, "sim_accel_resched")),
	_bad_registers(perf_alloc(PC_COUNT, "sim_bad_registers")),
	_bad_values(perf_alloc(PC_COUNT, "sim_bad_values")),
	_accel_filter_x(ACCELSIM_ACCEL_DEFAULT_RATE, ACCELSIM_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_y(ACCELSIM_ACCEL_DEFAULT_RATE, ACCELSIM_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_z(ACCELSIM_ACCEL_DEFAULT_RATE, ACCELSIM_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_rotation(rotation),
	_constant_accel_count(0),
	_last_temperature(0),
	_checked_next(0)
{


	// enable debug() calls
	_debug_enabled = false;

	_device_id.devid_s.devtype = DRV_ACC_DEVTYPE_ACCELSIM;
	
	/* Prime _mag with parents devid. */
	_mag->_device_id.devid = _device_id.devid;
	_mag->_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_ACCELSIM;


	// default scale factors
	_accel_scale.x_offset = 0.0f;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0.0f;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0.0f;
	_accel_scale.z_scale  = 1.0f;

	_mag_scale.x_offset = 0.0f;
	_mag_scale.x_scale = 1.0f;
	_mag_scale.y_offset = 0.0f;
	_mag_scale.y_scale = 1.0f;
	_mag_scale.z_offset = 0.0f;
	_mag_scale.z_scale = 1.0f;
}

ACCELSIM::~ACCELSIM()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_accel_reports != nullptr)
		delete _accel_reports;
	if (_mag_reports != nullptr)
		delete _mag_reports;

	if (_accel_class_instance != -1)
		unregister_class_devname(ACCEL_BASE_DEVICE_PATH, _accel_class_instance);

	delete _mag;

	/* delete the perf counter */
	perf_free(_accel_sample_perf);
	perf_free(_mag_sample_perf);
	perf_free(_bad_registers);
	perf_free(_bad_values);
	perf_free(_accel_reschedules);
}

int
ACCELSIM::init()
{
	int ret = ERROR;

	/* do SIM init first */
	if (VDev::init() != OK) {
		PX4_WARN("SIM init failed");
		goto out;
	}

	/* allocate basic report buffers */
	_accel_reports = new ringbuffer::RingBuffer(2, sizeof(accel_report));

	if (_accel_reports == nullptr)
		goto out;

	_mag_reports = new ringbuffer::RingBuffer(2, sizeof(mag_report));

	if (_mag_reports == nullptr)
		goto out;

	reset();

	/* do VDev init for the mag device node */
	ret = _mag->init();
	if (ret != OK) {
		PX4_WARN("MAG init failed");
		goto out;
	}

	/* fill report structures */
	measure();

	/* advertise sensor topic, measure manually to initialize valid report */
	struct mag_report mrp;
	_mag_reports->get(&mrp);

	/* measurement will have generated a report, publish */
	_mag->_mag_topic = orb_advertise_multi(ORB_ID(sensor_mag), &mrp,
		&_mag->_mag_orb_class_instance, ORB_PRIO_LOW);

	if (_mag->_mag_topic == nullptr) {
		PX4_WARN("ADVERT ERR");
	}


	_accel_class_instance = register_class_devname(ACCEL_BASE_DEVICE_PATH);

	/* advertise sensor topic, measure manually to initialize valid report */
	struct accel_report arp;
	_accel_reports->get(&arp);

	/* measurement will have generated a report, publish */
	_accel_topic = orb_advertise_multi(ORB_ID(sensor_accel), &arp,
		&_accel_orb_class_instance, ORB_PRIO_DEFAULT);

	if (_accel_topic == nullptr) {
		PX4_WARN("ADVERT ERR");
	}

out:
	return ret;
}

void
ACCELSIM::reset()
{
}

int
ACCELSIM::transfer(uint8_t *send, uint8_t *recv, unsigned len)
{
	uint8_t cmd = send[0];

	if (cmd & DIR_READ) {
		// Get data from the simulator
		Simulator *sim = Simulator::getInstance();
		if (sim == NULL)
			return ENODEV;

		// FIXME - not sure what interrupt status should be
		recv[1] = 0;
		// skip cmd and status bytes
		if (cmd & ACC_READ) {
			sim->getRawAccelReport(&recv[2], len-2);
		} else if (cmd & MAG_READ) {
			sim->getMagReport(&recv[2], len-2);
		}
	}

	return PX4_OK;
}

ssize_t
ACCELSIM::read(device::file_t *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct accel_report);
	accel_report *arb = reinterpret_cast<accel_report *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	/* if automatic measurement is enabled */
	if (_call_accel_interval > 0) {
		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 */
		while (count--) {
			if (_accel_reports->get(arb)) {
				ret += sizeof(*arb);
				arb++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement */
	measure();

	/* measurement will have generated a report, copy it out */
	if (_accel_reports->get(arb))
		ret = sizeof(*arb);

	return ret;
}

ssize_t
ACCELSIM::mag_read(device::file_t *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct mag_report);
	mag_report *mrb = reinterpret_cast<mag_report *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	/* if automatic measurement is enabled */
	if (_call_mag_interval > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 */
		while (count--) {
			if (_mag_reports->get(mrb)) {
				ret += sizeof(*mrb);
				mrb++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement */
	_mag_reports->flush();
	_mag->measure();

	/* measurement will have generated a report, copy it out */
	if (_mag_reports->get(mrb))
		ret = sizeof(*mrb);

	return ret;
}

int
ACCELSIM::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
		switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_call_accel_interval = 0;
				return OK;

			/* external signalling not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
				return ioctl(filp, SENSORIOCSPOLLRATE, 1600);

			case SENSOR_POLLRATE_DEFAULT:
				return ioctl(filp, SENSORIOCSPOLLRATE, ACCELSIM_ACCEL_DEFAULT_RATE);

				/* adjust to a legal polling interval in Hz */
			default: {
				/* do we need to start internal polling? */
				bool want_start = (_call_accel_interval == 0);

				/* convert hz to hrt interval via microseconds */
				unsigned period = 1000000 / arg;

				/* check against maximum sane rate */
				if (period < 500)
					return -EINVAL;

				/* adjust filters */
				accel_set_driver_lowpass_filter((float)arg, _accel_filter_x.get_cutoff_freq());

				/* update interval for next measurement */
				/* XXX this is a bit shady, but no other way to adjust... */
				_accel_call.period = _call_accel_interval = period;

				/* if we need to start the poll state machine, do it */
				if (want_start)
					start();

				return OK;
			}
		}
	}

	case SENSORIOCGPOLLRATE:
		if (_call_accel_interval == 0)
			return SENSOR_POLLRATE_MANUAL;

		return 1000000 / _call_accel_interval;

	case SENSORIOCSQUEUEDEPTH: {
		/* lower bound is mandatory, upper bound is a sanity check */
		if ((arg < 1) || (arg > 100))
			return -EINVAL;

		if (!_accel_reports->resize(arg)) {
			return -ENOMEM;
		}

		return OK;
	}

	case SENSORIOCGQUEUEDEPTH:
		return _accel_reports->size();

	case SENSORIOCRESET:
		reset();
		return OK;

	case ACCELIOCSSAMPLERATE:
		return accel_set_samplerate(arg);

	case ACCELIOCGSAMPLERATE:
		return _accel_samplerate;

	case ACCELIOCSLOWPASS: {
		return accel_set_driver_lowpass_filter((float)_accel_samplerate, (float)arg);
	}

	case ACCELIOCSSCALE: {
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

	case ACCELIOCSRANGE:
		/* arg needs to be in G */
		return accel_set_range(arg);

	case ACCELIOCGRANGE:
		/* convert to m/s^2 and return rounded in G */
		return (unsigned long)((_accel_range_m_s2)/ACCELSIM_ONE_G + 0.5f);

	case ACCELIOCGSCALE:
		/* copy scale out */
		memcpy((struct accel_scale *) arg, &_accel_scale, sizeof(_accel_scale));
		return OK;

	case ACCELIOCSELFTEST:
		return OK;

	default:
		/* give it to the superclass */
		return VDev::ioctl(filp, cmd, arg);
	}
}

int
ACCELSIM::mag_ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
		switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_call_mag_interval = 0;
				return OK;

			/* external signalling not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT:
				/* 100 Hz is max for mag */
				return mag_ioctl(filp, SENSORIOCSPOLLRATE, 100);

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_call_mag_interval == 0);

					/* convert hz to hrt interval via microseconds */
					unsigned period = 1000000 / arg;

					/* check against maximum sane rate (1ms) */
					if (period < 10000)
						return -EINVAL;

					/* update interval for next measurement */
					/* XXX this is a bit shady, but no other way to adjust... */
					_mag_call.period = _call_mag_interval = period;

					//PX4_INFO("SET _call_mag_interval=%u", _call_mag_interval);

					/* if we need to start the poll state machine, do it */
					if (want_start)
						start();

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_call_mag_interval == 0)
			return SENSOR_POLLRATE_MANUAL;

		return 1000000 / _call_mag_interval;

	case SENSORIOCSQUEUEDEPTH: {
		/* lower bound is mandatory, upper bound is a sanity check */
		if ((arg < 1) || (arg > 100))
			return -EINVAL;

		if (!_mag_reports->resize(arg)) {
			return -ENOMEM;
		}

		return OK;
	}

	case SENSORIOCGQUEUEDEPTH:
		return _mag_reports->size();

	case SENSORIOCRESET:
		reset();
		return OK;

	case MAGIOCSSAMPLERATE:
		return mag_set_samplerate(arg);

	case MAGIOCGSAMPLERATE:
		return _mag_samplerate;

	case MAGIOCSLOWPASS:
	case MAGIOCGLOWPASS:
		/* not supported, no internal filtering */
		return -EINVAL;

	case MAGIOCSSCALE:
		/* copy scale in */
		memcpy(&_mag_scale, (struct mag_scale *) arg, sizeof(_mag_scale));
		return OK;

	case MAGIOCGSCALE:
		/* copy scale out */
		memcpy((struct mag_scale *) arg, &_mag_scale, sizeof(_mag_scale));
		return OK;

	case MAGIOCSRANGE:
		return mag_set_range(arg);

	case MAGIOCGRANGE:
		return _mag_range_ga;

	case MAGIOCGEXTERNAL:
		/* Even if this sensor is on the "external" SPI bus
		 * it is still fixed to the autopilot assembly,
		 * so always return 0.
		 */
		return 0;

	case MAGIOCSELFTEST:
		return OK;
	default:
		/* give it to the superclass */
		return VDev::ioctl(filp, cmd, arg);
	}
}

uint8_t
ACCELSIM::read_reg(unsigned reg)
{
	uint8_t cmd[2];

	cmd[0] = reg | DIR_READ;
	cmd[1] = 0;

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[1];
}

void
ACCELSIM::write_reg(unsigned reg, uint8_t value)
{
	uint8_t	cmd[2];

	cmd[0] = reg | DIR_WRITE;
	cmd[1] = value;

	transfer(cmd, nullptr, sizeof(cmd));
}

void
ACCELSIM::write_checked_reg(unsigned reg, uint8_t value)
{
	write_reg(reg, value);
	for (uint8_t i=0; i<ACCELSIM_NUM_CHECKED_REGISTERS; i++) {
		if (reg == _checked_registers[i]) {
			_checked_values[i] = value;
		}
	}
}

void
ACCELSIM::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val;

	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_checked_reg(reg, val);
}

int
ACCELSIM::accel_set_range(unsigned max_g)
{
	float new_scale_g_digit = 0.732e-3f;

	_accel_range_scale = new_scale_g_digit * ACCELSIM_ONE_G;

	return OK;
}

int
ACCELSIM::mag_set_range(unsigned max_ga)
{
	float new_scale_ga_digit = 0.479e-3f;

	_mag_range_scale = new_scale_ga_digit;

	return OK;
}

int
ACCELSIM::accel_set_driver_lowpass_filter(float samplerate, float bandwidth)
{
	_accel_filter_x.set_cutoff_frequency(samplerate, bandwidth);
	_accel_filter_y.set_cutoff_frequency(samplerate, bandwidth);
	_accel_filter_z.set_cutoff_frequency(samplerate, bandwidth);

	return OK;
}

int
ACCELSIM::accel_set_samplerate(unsigned frequency)
{
	return OK;
}

int
ACCELSIM::mag_set_samplerate(unsigned frequency)
{
	return OK;
}

void
ACCELSIM::start()
{
	/* make sure we are stopped first */
	stop();

	/* reset the report ring */
	_accel_reports->flush();
	_mag_reports->flush();

	/* start polling at the specified rate */
	//PX4_INFO("ACCELSIM::start accel %u", _call_accel_interval);
	hrt_call_every(&_accel_call, 1000, _call_accel_interval, (hrt_callout)&ACCELSIM::measure_trampoline, this);

	// There is a race here where SENSORIOCSPOLLRATE on the accel starts polling of mag but _call_mag_interval is 0
	if (_call_mag_interval == 0) {
		//PX4_INFO("_call_mag_interval uninitilized - would have set period delay of 0");
		_call_mag_interval = 10000; // Max 100Hz
	}

	//PX4_INFO("ACCELSIM::start mag %u", _call_mag_interval);
	hrt_call_every(&_mag_call, 1000, _call_mag_interval, (hrt_callout)&ACCELSIM::mag_measure_trampoline, this);
}

void
ACCELSIM::stop()
{
	hrt_cancel(&_accel_call);
	hrt_cancel(&_mag_call);
}

void
ACCELSIM::measure_trampoline(void *arg)
{
	//PX4_INFO("ACCELSIM::measure_trampoline");
	ACCELSIM *dev = (ACCELSIM *)arg;

	/* make another measurement */
	dev->measure();
}

void
ACCELSIM::mag_measure_trampoline(void *arg)
{
	//PX4_INFO("ACCELSIM::mag_measure_trampoline");
	ACCELSIM *dev = (ACCELSIM *)arg;

	/* make another measurement */
	dev->mag_measure();
}

void
ACCELSIM::measure()
{
	/* status register and data as read back from the device */

#pragma pack(push, 1)
	struct {
		uint8_t		cmd;
		uint8_t		status;
		float temperature;
		float		x;
		float		y;
		float		z;
	} raw_accel_report;
#pragma pack(pop)

	accel_report accel_report;

	/* start the performance counter */
	perf_begin(_accel_sample_perf);

	/* fetch data from the sensor */
	memset(&raw_accel_report, 0, sizeof(raw_accel_report));
	raw_accel_report.cmd = DIR_READ | ACC_READ;

	if(OK != transfer((uint8_t *)&raw_accel_report, (uint8_t *)&raw_accel_report, sizeof(raw_accel_report))) {
		return;
	}

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


	accel_report.timestamp = hrt_absolute_time();

	// use the temperature from the last mag reading
	accel_report.temperature = _last_temperature;

	// report the error count as the sum of the number of bad
	// register reads and bad values. This allows the higher level
	// code to decide if it should use this sensor based on
	// whether it has had failures
        accel_report.error_count = perf_event_count(_bad_registers) + perf_event_count(_bad_values);

	accel_report.x_raw = (int16_t)(raw_accel_report.x/_accel_range_scale);
	accel_report.y_raw = (int16_t)(raw_accel_report.y / _accel_range_scale);
	accel_report.z_raw = (int16_t)(raw_accel_report.z / _accel_range_scale);

	// float xraw_f = (int16_t)(raw_accel_report.x/_accel_range_scale);
	// float yraw_f = (int16_t)(raw_accel_report.y / _accel_range_scale);
	// float zraw_f = (int16_t)(raw_accel_report.z / _accel_range_scale);

	// // apply user specified rotation
	// rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	// float x_in_new = ((xraw_f * _accel_range_scale) - _accel_scale.x_offset) * _accel_scale.x_scale;
	// float y_in_new = ((yraw_f * _accel_range_scale) - _accel_scale.y_offset) * _accel_scale.y_scale;
	// float z_in_new = ((zraw_f * _accel_range_scale) - _accel_scale.z_offset) * _accel_scale.z_scale;

	/*
	  we have logs where the accelerometers get stuck at a fixed
	  large value. We want to detect this and mark the sensor as
	  being faulty
	 */
	// if (fabsf(_last_accel[0] - x_in_new) < 0.001f &&
	//     fabsf(_last_accel[1] - y_in_new) < 0.001f &&
	//     fabsf(_last_accel[2] - z_in_new) < 0.001f &&
	//     fabsf(x_in_new) > 20 &&
	//     fabsf(y_in_new) > 20 &&
	//     fabsf(z_in_new) > 20) {
	// 	_constant_accel_count += 1;
	// } else {
	// 	_constant_accel_count = 0;
	// }
	// if (_constant_accel_count > 100) {
	// 	// we've had 100 constant accel readings with large
	// 	// values. The sensor is almost certainly dead. We
	// 	// will raise the error_count so that the top level
	// 	// flight code will know to avoid this sensor, but
	// 	// we'll still give the data so that it can be logged
	// 	// and viewed
	// 	perf_count(_bad_values);
	// 	_constant_accel_count = 0;
	// }

	// _last_accel[0] = x_in_new;
	// _last_accel[1] = y_in_new;
	// _last_accel[2] = z_in_new;

	// accel_report.x = _accel_filter_x.apply(x_in_new);
	// accel_report.y = _accel_filter_y.apply(y_in_new);
	// accel_report.z = _accel_filter_z.apply(z_in_new);

	accel_report.x = raw_accel_report.x;
	accel_report.y = raw_accel_report.y;
	accel_report.z = raw_accel_report.z;

	accel_report.scaling = _accel_range_scale;
	accel_report.range_m_s2 = _accel_range_m_s2;

	_accel_reports->force(&accel_report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	if (!(_pub_blocked)) {
		/* publish it */

		// The first call to measure() is from init() and _accel_topic is not
		// yet initialized
		if (_accel_topic != nullptr) {
			orb_publish(ORB_ID(sensor_accel), _accel_topic, &accel_report);
		}
	}

	_accel_read++;

	/* stop the perf counter */
	perf_end(_accel_sample_perf);
}

void
ACCELSIM::mag_measure()
{
	/* status register and data as read back from the device */
#pragma pack(push, 1)
	struct {
		uint8_t		cmd;
		uint8_t		status;
		float		temperature;
		float		x;
		float		y;
		float		z;
	} raw_mag_report;
#pragma pack(pop)

	mag_report mag_report;
	memset(&mag_report, 0, sizeof(mag_report));

	/* start the performance counter */
	perf_begin(_mag_sample_perf);

	/* fetch data from the sensor */
	memset(&raw_mag_report, 0, sizeof(raw_mag_report));
	raw_mag_report.cmd = DIR_READ | MAG_READ;

	if(OK != transfer((uint8_t *)&raw_mag_report, (uint8_t *)&raw_mag_report, sizeof(raw_mag_report))) {
		return;
	}

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


	mag_report.timestamp = hrt_absolute_time();

	mag_report.x_raw = (int16_t)(raw_mag_report.x / _mag_range_scale);
	mag_report.y_raw = (int16_t)(raw_mag_report.y / _mag_range_scale);
	mag_report.z_raw = (int16_t)(raw_mag_report.z / _mag_range_scale);

	float xraw_f = (int16_t)(raw_mag_report.x / _mag_range_scale);
	float yraw_f = (int16_t)(raw_mag_report.y / _mag_range_scale);
	float zraw_f = (int16_t)(raw_mag_report.z / _mag_range_scale);


	/* apply user specified rotation */
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	// mag_report.x = ((xraw_f * _mag_range_scale) - _mag_scale.x_offset) * _mag_scale.x_scale;
	// mag_report.y = ((yraw_f * _mag_range_scale) - _mag_scale.y_offset) * _mag_scale.y_scale;
	// mag_report.z = ((zraw_f * _mag_range_scale) - _mag_scale.z_offset) * _mag_scale.z_scale;
	// mag_report.scaling = _mag_range_scale;
	// mag_report.range_ga = (float)_mag_range_ga;
	// mag_report.error_count = perf_event_count(_bad_registers) + perf_event_count(_bad_values);

	/* remember the temperature. The datasheet isn't clear, but it
	 * seems to be a signed offset from 25 degrees C in units of 0.125C
	 */
	_last_temperature = raw_mag_report.temperature;
	mag_report.temperature = _last_temperature;
	mag_report.x = raw_mag_report.x;
	mag_report.y = raw_mag_report.y;
	mag_report.z = raw_mag_report.z;

	_mag_reports->force(&mag_report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	if (!(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(sensor_mag), _mag->_mag_topic, &mag_report);
	}

	_mag_read++;

	/* stop the perf counter */
	perf_end(_mag_sample_perf);
}

ACCELSIM_mag::ACCELSIM_mag(ACCELSIM *parent) :
	VDev("ACCELSIM_mag", ACCELSIM_DEVICE_PATH_MAG),
	_parent(parent),
	_mag_topic(nullptr),
	_mag_orb_class_instance(-1),
	_mag_class_instance(-1)
{
}

ACCELSIM_mag::~ACCELSIM_mag()
{
	if (_mag_class_instance != -1)
		unregister_class_devname(MAG_BASE_DEVICE_PATH, _mag_class_instance);
}

int
ACCELSIM_mag::init()
{
	int ret;

	ret = VDev::init();
	if (ret != OK)
		goto out;

	_mag_class_instance = register_class_devname(MAG_BASE_DEVICE_PATH);

out:
	return ret;
}

void
ACCELSIM_mag::parent_poll_notify()
{
	poll_notify(POLLIN);
}

ssize_t
ACCELSIM_mag::read(device::file_t *filp, char *buffer, size_t buflen)
{
	return _parent->mag_read(filp, buffer, buflen);
}

int
ACCELSIM_mag::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
		case DEVIOCGDEVICEID:
			return (int)VDev::ioctl(filp, cmd, arg);
			break;
		default:
			return _parent->mag_ioctl(filp, cmd, arg);
	}
}

void
ACCELSIM_mag::measure()
{
	_parent->mag_measure();
}

void
ACCELSIM_mag::measure_trampoline(void *arg)
{
	_parent->mag_measure_trampoline(arg);
}

/**
 * Local functions in support of the shell command.
 */
namespace accelsim
{

ACCELSIM	*g_dev;

int	start(enum Rotation rotation);
int	info();
void	usage();

/**
 * Start the driver.
 *
 * This function call only returns once the driver is
 * up and running or failed to detect the sensor.
 */
int
start(enum Rotation rotation)
{
	int fd, fd_mag;
	if (g_dev != nullptr) {
		PX4_WARN( "already started");
		return 0;
	}

	/* create the driver */
	g_dev = new ACCELSIM(ACCELSIM_DEVICE_PATH_ACCEL, rotation);

	if (g_dev == nullptr) {
		PX4_ERR("failed instantiating ACCELSIM obj");
		goto fail;
	}

	if (OK != g_dev->init()) {
		PX4_ERR("failed init of ACCELSIM obj");
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = px4_open(ACCELSIM_DEVICE_PATH_ACCEL, O_RDONLY);

	if (fd < 0) {
		PX4_WARN("open %s failed", ACCELSIM_DEVICE_PATH_ACCEL);
		goto fail;
	}

	if (px4_ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("ioctl SENSORIOCSPOLLRATE %s failed", ACCELSIM_DEVICE_PATH_ACCEL);
        	px4_close(fd);
		goto fail;
	}

	fd_mag = px4_open(ACCELSIM_DEVICE_PATH_MAG, O_RDONLY);

	/* don't fail if mag dev cannot be opened */
	if (0 <= fd_mag) {
		if (px4_ioctl(fd_mag, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
			PX4_ERR("ioctl SENSORIOCSPOLLRATE %s failed", ACCELSIM_DEVICE_PATH_ACCEL);
		}
	}
	else
	{
		PX4_ERR("ioctl SENSORIOCSPOLLRATE %s failed", ACCELSIM_DEVICE_PATH_ACCEL);
	}

        px4_close(fd);
        px4_close(fd_mag);

	return 0;
fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	PX4_ERR("driver start failed");
	return 1;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	PX4_DEBUG("state @ %p", g_dev);
	//g_dev->print_info();

	return 0;
}

void
usage()
{
	PX4_WARN("Usage: accelsim 'start', 'info'");
	PX4_WARN("options:");
	PX4_WARN("    -R rotation");
}

} // namespace

int
accelsim_main(int argc, char *argv[])
{
	int ch;
	enum Rotation rotation = ROTATION_NONE;
	int ret;
	int myoptind = 1;
	const char * myoptarg = NULL;

	/* jump over start/off/etc and look at options first */
	while ((ch = px4_getopt(argc, argv, "R:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;
		default:
			accelsim::usage();
			return 0;
		}
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start"))
		ret = accelsim::start(rotation);

	/*
	 * Print driver information.
	 */
	else if (!strcmp(verb, "info"))
		ret = accelsim::info();

	else {
		accelsim::usage();
		return 1;
	}
	return ret;
}
