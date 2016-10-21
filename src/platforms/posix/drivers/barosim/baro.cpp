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
 * @file baro.cpp
 * Driver for the simulated barometric pressure sensor
 */

#include <inttypes.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_time.h>
#include <px4_getopt.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <arch/board/board.h>
#include <board_config.h>

#include <drivers/device/device.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>

#include <simulator/simulator.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include "barosim.h"
#include "VirtDevObj.hpp"

/* helper macro for handling report buffer indices */
#define INCREMENT(_x, _lim)	do { __typeof__(_x) _tmp = _x+1; if (_tmp >= _lim) _tmp = 0; _x = _tmp; } while(0)

/* helper macro for arithmetic - returns the square of the argument */
#define POW2(_x)		((_x) * (_x))

#define BAROSIM_DEV_PATH "/dev/barosim"
/*
 * BAROSIM internal constants and data structures.
 */

/* internal conversion time: 9.17 ms, so should not be read at rates higher than 100 Hz */
#define BAROSIM_CONVERSION_INTERVAL	10000	/* microseconds */
#define BAROSIM_MEASUREMENT_RATIO	3	/* pressure measurements per temperature measurement */

using namespace DriverFramework;

class BAROSIM : public VirtDevObj
{
public:
	BAROSIM(const char *path);
	~BAROSIM();

	virtual int		init();

	virtual ssize_t		devRead(void *buffer, size_t buflen);
	virtual int		devIOCTL(unsigned long cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:

	ringbuffer::RingBuffer	*_reports;

	bool			_collect_phase;
	unsigned		_measure_phase;

	/* last report */
	struct baro_report	report;

	/* altitude conversion calibration */
	unsigned		_msl_pressure;	/* in Pa */

	orb_advert_t		_baro_topic;
	int			_orb_class_instance;

	perf_counter_t		_sample_perf;
	perf_counter_t		_measure_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;

	/**
	 * Initialize the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void			start_cycle();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void			stop_cycle();

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the heart of the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
	 * When the interval between measurements is greater than the minimum
	 * measurement interval, a gap is inserted between collection
	 * and measurement to provide the most recent measurement possible
	 * at the next interval.
	 */
	void			cycle();

	int			transfer(const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len);

	/**
	 * Get the internal / external state
	 *
	 * @return true if the sensor is not on the main MCU board
	 */
	bool			is_external() { return (_orb_class_instance == 0); /* XXX put this into the interface class */ }

	/**
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		cycle_trampoline(void *arg);

	/**
	 * Issue a measurement command for the current state.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			measure();

	// Unused
	virtual void _measure();

	/**
	 * Collect the result of the most recent measurement.
	 */
	virtual int		collect();
};

static BAROSIM *g_barosim = nullptr;

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int barosim_main(int argc, char *argv[]);

BAROSIM::BAROSIM(const char *path) :
	VirtDevObj("BAROSIM", path, BARO_BASE_DEVICE_PATH, 1e6 / 100),
	_reports(nullptr),
	_collect_phase(false),
	_measure_phase(0),
	report{},
	_msl_pressure(101325),
	_baro_topic(nullptr),
	_orb_class_instance(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "barosim_read")),
	_measure_perf(perf_alloc(PC_ELAPSED, "barosim_measure")),
	_comms_errors(perf_alloc(PC_COUNT, "barosim_comms_errors")),
	_buffer_overflows(perf_alloc(PC_COUNT, "barosim_buffer_overflows"))
{

}

BAROSIM::~BAROSIM()
{
	/* make sure we are truly inactive */
	stop_cycle();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_measure_perf);
	perf_free(_comms_errors);
	perf_free(_buffer_overflows);

}

int
BAROSIM::init()
{
	int ret;
	struct baro_report brp = {};
	//PX4_DEBUG("BAROSIM::init");

	ret = VirtDevObj::init();

	if (ret != OK) {
		PX4_ERR("VirtDevObj init failed");
		goto out;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(baro_report));

	if (_reports == nullptr) {
		PX4_ERR("can't get memory for reports");
		ret = -ENOMEM;
		goto out;
	}

	/* do a first measurement cycle to populate reports with valid data */
	_measure_phase = 0;

	_reports->flush();

	_baro_topic = orb_advertise_multi(ORB_ID(sensor_baro), &brp,
					  &_orb_class_instance, (is_external()) ? ORB_PRIO_HIGH : ORB_PRIO_DEFAULT);

	if (_baro_topic == nullptr) {
		PX4_ERR("failed to create sensor_baro publication");
	}

	/* this do..while is goto without goto */
	do {
		/* do temperature first */
		if (OK != measure()) {
			ret = -EIO;
			PX4_ERR("temp measure failed");
			break;
		}

		usleep(BAROSIM_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			PX4_ERR("temp collect failed");
			break;
		}

		/* now do a pressure measurement */
		if (OK != measure()) {
			ret = -EIO;
			PX4_ERR("pressure collect failed");
			break;
		}

		usleep(BAROSIM_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			PX4_ERR("pressure collect failed");
			break;
		}

		/* state machine will have generated a report, copy it out */
		_reports->get(&brp);

		ret = OK;

		//PX4_WARN("sensor_baro publication %ld", _baro_topic);

	} while (0);

out:
	return ret;
}

ssize_t
BAROSIM::devRead(void *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct baro_report);
	struct baro_report *brp = reinterpret_cast<struct baro_report *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (m_sample_interval_usecs > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(brp)) {
				ret += sizeof(*brp);
				brp++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	do {
		_measure_phase = 0;
		_reports->flush();

		/* do temperature first */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		usleep(BAROSIM_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* now do a pressure measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		usleep(BAROSIM_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		if (_reports->get(brp)) {
			ret = sizeof(*brp);
		}

	} while (0);

	return ret;
}

int
BAROSIM::devIOCTL(unsigned long cmd, unsigned long arg)
{
	//PX4_WARN("baro IOCTL %" PRIu64 , hrt_absolute_time());

	switch (cmd) {

	case SENSORIOCSPOLLRATE:
		switch (arg) {

		/* switching to manual polling */
		case SENSOR_POLLRATE_MANUAL:
			stop_cycle();
			setSampleInterval(0);
			m_sample_interval_usecs = 0;
			return OK;

		/* external signalling not supported */
		case SENSOR_POLLRATE_EXTERNAL:

		/* zero would be bad */
		case 0:
			return -EINVAL;

		/* set default/max polling rate */
		case SENSOR_POLLRATE_MAX:
		case SENSOR_POLLRATE_DEFAULT: {
				setSampleInterval(BAROSIM_CONVERSION_INTERVAL);
				return OK;
			}

		/* adjust to a legal polling interval in Hz */
		default: {
				/* convert hz to tick interval via microseconds */
				unsigned long interval = 1000000 / arg;

				/* check against maximum rate */
				if (interval < BAROSIM_CONVERSION_INTERVAL) {
					return -EINVAL;
				}

				bool want_start = (m_sample_interval_usecs == 0);

				/* update interval for next measurement */
				setSampleInterval(interval);

				if (want_start) {
					start();
				}

				return OK;
			}
		}

	case SENSORIOCGPOLLRATE:
		if (m_sample_interval_usecs == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return (1000000 / m_sample_interval_usecs);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			if (!_reports->resize(arg)) {
				return -ENOMEM;
			}

			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _reports->size();

	case SENSORIOCRESET:
		/*
		 * Since we are initialized, we do not need to do anything, since the
		 * PROM is correctly read and the part does not need to be configured.
		 */
		return OK;

	case BAROIOCSMSLPRESSURE:

		/* range-check for sanity */
		if ((arg < 80000) || (arg > 120000)) {
			return -EINVAL;
		}

		_msl_pressure = arg;
		return OK;

	case BAROIOCGMSLPRESSURE:
		return _msl_pressure;

	default:
		break;
	}

	/* give it to the bus-specific superclass */
	// return bus_ioctl(filp, cmd, arg);
	return VirtDevObj::devIOCTL(cmd, arg);
}

void
BAROSIM::start_cycle()
{

	/* reset the report ring and state machine */
	_collect_phase = false;
	_measure_phase = 0;
	_reports->flush();

	/* schedule a cycle to start things */
	setSampleInterval(1000);
	start();
}

void
BAROSIM::stop_cycle()
{
	stop();
	setSampleInterval(0);
}

void
BAROSIM::_measure()
{
	cycle();
}

void
BAROSIM::cycle()
{
	int ret;

	//PX4_WARN("baro cycle %llu", hrt_absolute_time());

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		ret = collect();

		if (ret != OK) {
			uint8_t		cmd = ADDR_RESET_CMD;

			/* bump the retry count */
			(void)transfer(&cmd, 1, nullptr, 0);

			/* reset the collection state machine and try again */
			//start_cycle();

			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 * Don't inject one after temperature measurements, so we can keep
		 * doing pressure measurements at something close to the desired rate.
		 */
		if (_measure_phase != 0) {

			//setSampleInterval(BAROSIM_CONVERSION_INTERVAL);

			return;
		}
	}

	/* measurement phase */
	ret = measure();

	if (ret != OK) {
		//DEVICE_LOG("measure error %d", ret);
		/* issue a reset command to the sensor */
		//_interface->devIOCTL(IOCTL_RESET, dummy);

		/* reset the collection state machine and try again */
		//start_cycle();
		return;
	}

	/* next phase is collection */
	_collect_phase = true;

	//setSampleInterval(BAROSIM_CONVERSION_INTERVAL);
}

int
BAROSIM::measure()
{
	int ret;

	//PX4_WARN("baro measure %llu", hrt_absolute_time());

	perf_begin(_measure_perf);

	/*
	 * In phase zero, request temperature; in other phases, request pressure.
	 */
	unsigned long addr = (_measure_phase == 0) ? ADDR_CMD_CONVERT_D2 : ADDR_CMD_CONVERT_D1;

	/*
	 * Send the command to begin measuring.
	 */
	uint8_t cmd = addr;
	ret = transfer(&cmd, 1, nullptr, 0);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	perf_end(_measure_perf);

	return ret;
}

int
BAROSIM::transfer(const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len)
{
	if (send_len == 1 && send[0] == ADDR_RESET_CMD) {
		/* reset command */
		return 0;

	} else if (send_len == 1 && (send[0] == ADDR_CMD_CONVERT_D2 || send[0] == ADDR_CMD_CONVERT_D1)) {
		/* measure command */
		if (send[0] == ADDR_CMD_CONVERT_D2) {
		} else {
		}

		return 0;

	} else if (send[0] == 0 && send_len == 1) {
		/* read requested */
		Simulator *sim = Simulator::getInstance();

		if (sim == NULL) {
			PX4_ERR("Error BAROSIM_DEV::transfer no simulator");
			return -ENODEV;
		}

		PX4_DEBUG("BAROSIM_DEV::transfer getting sample");
		sim->getBaroSample(recv, recv_len);
		return recv_len;

	} else {
		PX4_WARN("BAROSIM_DEV::transfer invalid param %u %u %u", send_len, send[0], recv_len);
		return 1;
	}


	return 0;
}

int
BAROSIM::collect()
{
	//PX4_WARN("baro collect %llu", hrt_absolute_time());

	int ret;

#pragma pack(push, 1)
	struct raw_baro_s {
		float		pressure;
		float		altitude;
		float		temperature;
	} raw_baro;
#pragma pack(pop)

	perf_begin(_sample_perf);

	/* this should be fairly close to the end of the conversion, so the best approximation of the time */
	report.timestamp = hrt_absolute_time();
	report.error_count = perf_event_count(_comms_errors);

	/* read the most recent measurement - read offset/size are hardcoded in the interface */
	uint8_t cmd = 0;
	ret = transfer(&cmd, 1, (uint8_t *)(&raw_baro), sizeof(raw_baro));

	if (ret < 0) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	/* handle a measurement */
	if (_measure_phase == 0) {
		report.pressure = raw_baro.pressure;
		report.altitude = raw_baro.altitude;
		report.temperature = raw_baro.temperature;

	} else {
		report.pressure = raw_baro.pressure;
		report.altitude = raw_baro.altitude;
		report.temperature = raw_baro.temperature;

		/* publish it */
		if (!(m_pub_blocked)) {
			if (_baro_topic != nullptr) {
				/* publish it */
				orb_publish(ORB_ID(sensor_baro), _baro_topic, &report);

			} else {
				PX4_WARN("BAROSIM::collect _baro_topic not initialized");
			}
		}

		if (_reports->force(&report)) {
			perf_count(_buffer_overflows);
		}

		/* notify anyone waiting for data */
		//DevMgr::updateNotify(*this);
		updateNotify();
	}

	/* update the measurement state machine */
	INCREMENT(_measure_phase, BAROSIM_MEASUREMENT_RATIO + 1);

	perf_end(_sample_perf);

	return OK;
}

void
BAROSIM::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	PX4_INFO("poll interval:  %u usec", m_sample_interval_usecs);
	_reports->print_info("report queue");
	PX4_INFO("TEMP:           %f", (double)report.temperature);
	PX4_INFO("P:              %.3f", (double)report.pressure);
}

namespace barosim
{

/**
 * BAROSIM crc4 cribbed from the datasheet
 */
bool
crc4(uint16_t *n_prom)
{
	int16_t cnt;
	uint16_t n_rem;
	uint16_t crc_read;
	uint8_t n_bit;

	n_rem = 0x00;

	/* save the read crc */
	crc_read = n_prom[7];

	/* remove CRC byte */
	n_prom[7] = (0xFF00 & (n_prom[7]));

	for (cnt = 0; cnt < 16; cnt++) {
		/* uneven bytes */
		if (cnt & 1) {
			n_rem ^= (uint8_t)((n_prom[cnt >> 1]) & 0x00FF);

		} else {
			n_rem ^= (uint8_t)(n_prom[cnt >> 1] >> 8);
		}

		for (n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & 0x8000) {
				n_rem = (n_rem << 1) ^ 0x3000;

			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	/* final 4 bit remainder is CRC value */
	n_rem = (0x000F & (n_rem >> 12));
	n_prom[7] = crc_read;

	/* return true if CRCs match */
	return (0x000F & crc_read) == (n_rem ^ 0x00);
}

/**
 * Start the driver.
 *
 * This function call only returns once the driver
 * is either successfully up and running or failed to start.
 */
static int
start()
{
	g_barosim = new BAROSIM(BAROSIM_DEV_PATH);

	if (g_barosim != nullptr && OK != g_barosim->init()) {
		delete g_barosim;
		g_barosim = NULL;
		PX4_ERR("bus init failed");
		return false;
	}

	DevHandle h;
	DevMgr::getHandle(BAROSIM_DEV_PATH, h);

	/* set the poll rate to default, starts automatic data collection */
	if (!h.isValid()) {
		PX4_ERR("can't open baro device");
		return false;
	}

	if (h.ioctl(SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		DevMgr::releaseHandle(h);
		PX4_ERR("failed setting default poll rate");
		return false;
	}

	DevMgr::releaseHandle(h);
	return 0;
}


/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
static int
test()
{
	struct baro_report report;
	ssize_t sz;
	int ret;

	DevHandle h;
	DevMgr::getHandle(BAROSIM_DEV_PATH, h);

	if (!h.isValid()) {
		PX4_ERR("getHandle failed (try 'barosim start' if the driver is not running)");
		return 1;
	}

	/* do a simple demand read */
	sz = h.read(&report, sizeof(report));

	if (sz != sizeof(report)) {
		PX4_ERR("immediate read failed");
		return 1;
	}

	PX4_INFO("single read");
	PX4_INFO("pressure:    %10.4f", (double)report.pressure);
	PX4_INFO("altitude:    %11.4f", (double)report.altitude);
	PX4_INFO("temperature: %8.4f", (double)report.temperature);
	PX4_INFO("time:        %lld", (long long)report.timestamp);

	/* set the queue depth to 10 */
	if (OK != h.ioctl(SENSORIOCSQUEUEDEPTH, 10UL)) {
		PX4_ERR("failed to set queue depth");
		DevMgr::releaseHandle(h);
		return 1;
	}

	/* start the sensor polling at 2Hz */
	if (OK != h.ioctl(SENSORIOCSPOLLRATE, 2UL)) {
		PX4_ERR("failed to set 2Hz poll rate");
		DevMgr::releaseHandle(h);
		return 1;
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		UpdateList in_set, out_set;
		in_set.pushBack(&h);

		/* wait for data to be ready */
		ret = DevMgr::waitForUpdate(in_set, out_set, 1000);

		if (ret != 1) {
			PX4_WARN("timed out waiting for sensor data");
		}

		/* now go get it */
		sz = h.read(&report, sizeof(report));

		if (sz != sizeof(report)) {
			PX4_ERR("periodic read failed");
			DevMgr::releaseHandle(h);
			return 1;
		}

		PX4_INFO("periodic read %u", i);
		PX4_INFO("pressure:    %10.4f", (double)report.pressure);
		PX4_INFO("altitude:    %11.4f", (double)report.altitude);
		PX4_INFO("temperature: %8.4f", (double)report.temperature);
		PX4_INFO("time:        %lld", (long long)report.timestamp);
	}

	DevMgr::releaseHandle(h);
	PX4_INFO("PASS");
	return 0;
}

/**
 * Reset the driver.
 */
static int
reset()
{
	DevHandle h;
	DevMgr::getHandle(BAROSIM_DEV_PATH, h);

	if (!h.isValid()) {
		PX4_ERR("failed ");
		return 1;
	}

	if (h.ioctl(SENSORIOCRESET, 0) < 0) {
		PX4_ERR("driver reset failed");
		return 1;
	}

	if (h.ioctl(SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("driver poll restart failed");
		return 1;
	}

	return 0;
}

/**
 * Print a little info about the driver.
 */
static int
info()
{
	if (g_barosim != nullptr) {
		PX4_INFO("%s", BAROSIM_DEV_PATH);
		g_barosim->print_info();
	}

	return 0;
}

/**
 * Calculate actual MSL pressure given current altitude
 */
static int
calibrate(unsigned altitude)
{
	struct baro_report report;
	float  pressure;
	float  p1;

	DevHandle h;
	DevMgr::getHandle(BAROSIM_DEV_PATH, h);

	if (!h.isValid()) {
		PX4_ERR("open failed (try 'barosim start' if the driver is not running)");
		return 1;
	}

	/* start the sensor polling at max */
	if (OK != h.ioctl(SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MAX)) {
		PX4_ERR("failed to set poll rate");
		return 1;
	}

	/* average a few measurements */
	pressure = 0.0f;

	for (unsigned i = 0; i < 20; i++) {
		int ret;
		ssize_t sz;
		UpdateList in_set, out_set;
		in_set.pushBack(&h);

		/* wait for data to be ready */
		ret = DevMgr::waitForUpdate(in_set, out_set, 1000);

		if (ret != 1) {
			PX4_ERR("timed out waiting for sensor data");
			return 1;
		}

		/* now go get it */
		sz = h.read(&report, sizeof(report));

		if (sz != sizeof(report)) {
			PX4_ERR("sensor read failed");
			return 1;
		}

		pressure += report.pressure;
	}

	pressure /= 20;		/* average */
	pressure /= 10;		/* scale from millibar to kPa */

	/* tropospheric properties (0-11km) for standard atmosphere */
	const float T1 = 15.0 + 273.15;	/* temperature at base height in Kelvin */
	const float a  = -6.5 / 1000;	/* temperature gradient in degrees per metre */
	const float g  = 9.80665f;	/* gravity constant in m/s/s */
	const float R  = 287.05f;	/* ideal gas constant in J/kg/K */

	PX4_INFO("averaged pressure %10.4fkPa at %um", (double)pressure, altitude);

	p1 = pressure * (powf(((T1 + (a * (float)altitude)) / T1), (g / (a * R))));

	PX4_INFO("calculated MSL pressure %10.4fkPa", (double)p1);

	/* save as integer Pa */
	p1 *= 1000.0f;

	if (h.ioctl(BAROIOCSMSLPRESSURE, (unsigned long)(p1)) != OK) {
		PX4_WARN("BAROIOCSMSLPRESSURE");
		return 1;
	}

	DevMgr::releaseHandle(h);
	return 0;
}

static void
usage()
{
	PX4_WARN("missing command: try 'start', 'info', 'test', 'test2', 'reset', 'calibrate <altitude>'");
}

}; // namespace barosim

int
barosim_main(int argc, char *argv[])
{
	int ret;

	if (argc < 2) {
		barosim::usage();
		return 1;
	}

	const char *verb = argv[1];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		ret = barosim::start();
	}

	/*
	 * Test the driver/device.
	 */
	else if (!strcmp(verb, "test")) {
		ret = barosim::test();
	}

	/*
	 * Reset the driver.
	 */
	else if (!strcmp(verb, "reset")) {
		ret = barosim::reset();
	}

	/*
	 * Print driver information.
	 */
	else if (!strcmp(verb, "info")) {
		ret = barosim::info();
	}

	/*
	 * Perform MSL pressure calibration given an altitude in metres
	 */
	else if (!strcmp(verb, "calibrate")) {
		if (argc < 3) {
			PX4_WARN("missing altitude");
			barosim::usage();
			return 1;
		}

		long altitude = strtol(argv[2], nullptr, 10);

		ret = barosim::calibrate(altitude);

	} else {
		barosim::usage();
		return 1;
	}

	return ret;
}
