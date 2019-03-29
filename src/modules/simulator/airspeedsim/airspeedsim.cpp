/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file ets_airspeed.cpp
 * @author Simon Wilks
 * @author Mark Charlebois (simulator)
 * @author Roman Bapst (simulator)
 * Driver for the Simulated AirspeedSim Sensor based on Eagle Tree AirspeedSim V3.
 */

#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <systemlib/err.h>
#include <parameters/param.h>
#include <perf/perf_counter.h>

#include <drivers/drv_airspeed.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/differential_pressure.h>

#include <simulator/simulator.h>

#include "airspeedsim.h"

AirspeedSim::AirspeedSim(int bus, int address, unsigned conversion_interval, const char *path) :
	CDev(path),
	_reports(nullptr),
	_retries(0),
	_sensor_ok(false),
	_measure_ticks(0),
	_collect_phase(false),
	_diff_pres_offset(0.0f),
	_airspeed_pub(nullptr),
	_class_instance(-1),
	_conversion_interval(conversion_interval),
	_sample_perf(perf_alloc(PC_ELAPSED, "airspeed_read")),
	_comms_errors(perf_alloc(PC_COUNT, "airspeed_comms_errors"))
{
}

AirspeedSim::~AirspeedSim()
{
	/* make sure we are truly inactive */
	stop();

	if (_class_instance != -1) {
		unregister_class_devname(AIRSPEED_BASE_DEVICE_PATH, _class_instance);
	}

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
AirspeedSim::init()
{
	int ret = ERROR;

	/* init base class */
	if (CDev::init() != OK) {
		PX4_ERR("CDev init failed");
		goto out;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(differential_pressure_s));

	if (_reports == nullptr) {
		goto out;
	}

	/* register alternate interfaces if we have to */
	_class_instance = register_class_devname(AIRSPEED_BASE_DEVICE_PATH);

	/* publication init */
	if (_class_instance == 0) {

		/* advertise sensor topic, measure manually to initialize valid report */
		struct differential_pressure_s arp;
		measure();
		_reports->get(&arp);

		/* measurement will have generated a report, publish */
		_airspeed_pub = orb_advertise(ORB_ID(differential_pressure), &arp);

		if (_airspeed_pub == nullptr) {
			PX4_WARN("uORB started?");
		}
	}

	ret = OK;

out:
	return ret;
}

int
AirspeedSim::probe()
{
	/* on initial power up the device may need more than one retry
	   for detection. Once it is running the number of retries can
	   be reduced
	*/
	_retries = 4;
	int ret = measure();

	// drop back to 2 retries once initialised
	_retries = 2;
	return ret;
}

int
AirspeedSim::ioctl(cdev::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default polling rate */
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(_conversion_interval);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					int ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(_conversion_interval)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case AIRSPEEDIOCSSCALE: {
			struct airspeed_scale *s = (struct airspeed_scale *)arg;
			_diff_pres_offset = s->offset_pa;
			return OK;
		}

	default:
		/* give it to the superclass */
		//return I2C::ioctl(filp, cmd, arg);	XXX SIM should be super class
		return 0;
	}
}

ssize_t
AirspeedSim::read(cdev::file_t *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(differential_pressure_s);
	differential_pressure_s *abuf = reinterpret_cast<differential_pressure_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(abuf)) {
				ret += sizeof(*abuf);
				abuf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	do {
		_reports->flush();

		/* trigger a measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* wait for it to complete */
		px4_usleep(_conversion_interval);

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		if (_reports->get(abuf)) {
			ret = sizeof(*abuf);
		}

	} while (0);

	return ret;
}

void
AirspeedSim::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&AirspeedSim::cycle_trampoline, this, 1);
}

void
AirspeedSim::stop()
{
	work_cancel(HPWORK, &_work);
}

void
AirspeedSim::cycle_trampoline(void *arg)
{
	AirspeedSim *dev = (AirspeedSim *)arg;

	dev->cycle();
}

void
AirspeedSim::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	PX4_INFO("poll interval:  %u ticks", _measure_ticks);
	_reports->print_info("report queue");
}

void
AirspeedSim::new_report(const differential_pressure_s &report)
{
	_reports->force(&report);
}

int
AirspeedSim::transfer(const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len)
{
	if (recv_len > 0) {
		// this is equivalent to the collect phase
		Simulator *sim = Simulator::getInstance();

		if (sim == nullptr) {
			PX4_ERR("Error BARO_SIM::transfer no simulator");
			return -ENODEV;
		}

		PX4_DEBUG("BARO_SIM::transfer getting sample");
		sim->getAirspeedSample(recv, recv_len);

	} else {
		// we don't need measure phase
	}

	return 0;
}
