/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
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
 * @file gps.cpp
 * Driver for the GPS on a serial port
 */

#include <nuttx/clock.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdio.h>
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
#include <fcntl.h>
#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/i2c.h>
#include <systemlib/perf_counter.h>
#include <systemlib/scheduling_priorities.h>
#include <systemlib/err.h>
#include <drivers/drv_gps.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>

#include "ubx.h"
#include "mtk.h"


#define TIMEOUT_5HZ 500
#define RATE_MEASUREMENT_PERIOD 5000000

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif



class GPS : public device::CDev
{
public:
	GPS(const char *uart_path);
	virtual ~GPS();

	virtual int			init();

	virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void				print_info();

private:

	bool				_task_should_exit;				///< flag to make the main worker task exit
	int				_serial_fd;					///< serial interface to GPS
	unsigned			_baudrate;					///< current baudrate
	char				_port[20];					///< device / serial port path
	volatile int			_task;						//< worker task
	bool				_healthy;					///< flag to signal if the GPS is ok
	bool 				_baudrate_changed;				///< flag to signal that the baudrate with the GPS has changed
	bool				_mode_changed;					///< flag that the GPS mode has changed
	gps_driver_mode_t		_mode;						///< current mode
	GPS_Helper			*_Helper;					///< instance of GPS parser
	struct vehicle_gps_position_s 	_report;					///< uORB topic for gps position
	orb_advert_t			_report_pub;					///< uORB pub for gps position
	float				_rate;						///< position update rate


	/**
	 * Try to configure the GPS, handle outgoing communication to the GPS
	 */
	void				config();

	/**
	 * Trampoline to the worker task
	 */
	static void			task_main_trampoline(void *arg);


	/**
	 * Worker task: main GPS thread that configures the GPS and parses incoming data, always running
	 */
	void				task_main(void);

	/**
	 * Set the baudrate of the UART to the GPS
	 */
	int				set_baudrate(unsigned baud);

	/**
	 * Send a reset command to the GPS
	 */
	void				cmd_reset();

};


/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int gps_main(int argc, char *argv[]);

namespace
{

GPS	*g_dev;

}


GPS::GPS(const char *uart_path) :
	CDev("gps", GPS_DEVICE_PATH),
	_task_should_exit(false),
	_healthy(false),
	_mode_changed(false),
	_mode(GPS_DRIVER_MODE_UBX),
	_Helper(nullptr),
	_report_pub(-1),
	_rate(0.0f)
{
	/* store port name */
	strncpy(_port, uart_path, sizeof(_port));
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	/* we need this potentially before it could be set in task_main */
	g_dev = this;
	memset(&_report, 0, sizeof(_report));

	_debug_enabled = true;
}

GPS::~GPS()
{
	/* tell the task we want it to go away */
	_task_should_exit = true;

	/* spin waiting for the task to stop */
	for (unsigned i = 0; (i < 10) && (_task != -1); i++) {
		/* give it another 100ms */
		usleep(100000);
	}

	/* well, kill it anyway, though this will probably crash */
	if (_task != -1)
		task_delete(_task);

	g_dev = nullptr;

}

int
GPS::init()
{
	int ret = ERROR;

	/* do regular cdev init */
	if (CDev::init() != OK)
		goto out;

	/* start the GPS driver worker task */
	_task = task_create("gps", SCHED_PRIORITY_SLOW_DRIVER, 2048, (main_t)&GPS::task_main_trampoline, nullptr);

	if (_task < 0) {
		warnx("task start failed: %d", errno);
		return -errno;
	}

	ret = OK;
out:
	return ret;
}

int
GPS::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	lock();

	int ret = OK;

	switch (cmd) {
	case SENSORIOCRESET:
		cmd_reset();
		break;
	}

	unlock();

	return ret;
}

void
GPS::task_main_trampoline(void *arg)
{
	g_dev->task_main();
}

void
GPS::task_main()
{
	log("starting");

	/* open the serial port */
	_serial_fd = ::open(_port, O_RDWR);

	if (_serial_fd < 0) {
		log("failed to open serial port: %s err: %d", _port, errno);
		/* tell the dtor that we are exiting, set error code */
		_task = -1;
		_exit(1);
	}

	uint64_t last_rate_measurement = hrt_absolute_time();
	unsigned last_rate_count = 0;

	/* loop handling received serial bytes and also configuring in between */
	while (!_task_should_exit) {

		if (_Helper != nullptr) {
			delete(_Helper);
			/* set to zero to ensure parser is not used while not instantiated */
			_Helper = nullptr;
		}

		switch (_mode) {
		case GPS_DRIVER_MODE_UBX:
			_Helper = new UBX(_serial_fd, &_report);
			break;

		case GPS_DRIVER_MODE_MTK:
			_Helper = new MTK(_serial_fd, &_report);
			break;

		default:
			break;
		}

		unlock();

		if (_Helper->configure(_baudrate) == 0) {
			unlock();

			// GPS is obviously detected successfully, reset statistics
			_Helper->reset_update_rates();

			while (_Helper->receive(TIMEOUT_5HZ) > 0 && !_task_should_exit) {
//				lock();
				/* opportunistic publishing - else invalid data would end up on the bus */
				if (_report_pub > 0) {
					orb_publish(ORB_ID(vehicle_gps_position), _report_pub, &_report);

				} else {
					_report_pub = orb_advertise(ORB_ID(vehicle_gps_position), &_report);
				}

				last_rate_count++;

				/* measure update rate every 5 seconds */
				if (hrt_absolute_time() - last_rate_measurement > RATE_MEASUREMENT_PERIOD) {
					_rate = last_rate_count / ((float)((hrt_absolute_time() - last_rate_measurement)) / 1000000.0f);
					last_rate_measurement = hrt_absolute_time();
					last_rate_count = 0;
					_Helper->store_update_rates();
					_Helper->reset_update_rates();
				}

				if (!_healthy) {
					char *mode_str = "unknown";

					switch (_mode) {
					case GPS_DRIVER_MODE_UBX:
						mode_str = "UBX";
						break;

					case GPS_DRIVER_MODE_MTK:
						mode_str = "MTK";
						break;

					default:
						break;
					}

					warnx("module found: %s", mode_str);
					_healthy = true;
				}
			}

			if (_healthy) {
				warnx("module lost");
				_healthy = false;
				_rate = 0.0f;
			}

			lock();
		}

		lock();

		/* select next mode */
		switch (_mode) {
		case GPS_DRIVER_MODE_UBX:
			_mode = GPS_DRIVER_MODE_MTK;
			break;

		case GPS_DRIVER_MODE_MTK:
			_mode = GPS_DRIVER_MODE_UBX;
			break;

		default:
			break;
		}

	}

	warnx("exiting");

	::close(_serial_fd);

	/* tell the dtor that we are exiting */
	_task = -1;
	_exit(0);
}



void
GPS::cmd_reset()
{
	//XXX add reset?
}

void
GPS::print_info()
{
	switch (_mode) {
	case GPS_DRIVER_MODE_UBX:
		warnx("protocol: UBX");
		break;

	case GPS_DRIVER_MODE_MTK:
		warnx("protocol: MTK");
		break;

	default:
		break;
	}

	warnx("port: %s, baudrate: %d, status: %s", _port, _baudrate, (_healthy) ? "OK" : "NOT OK");

	if (_report.timestamp_position != 0) {
		warnx("position lock: %dD, satellites: %d, last update: %fms ago", (int)_report.fix_type,
				_report.satellites_visible, (hrt_absolute_time() - _report.timestamp_position) / 1000.0f);
		warnx("lat: %d, lon: %d, alt: %d", _report.lat, _report.lon, _report.alt);
		warnx("eph: %.2fm, epv: %.2fm", _report.eph_m, _report.epv_m);
		warnx("rate position: \t%6.2f Hz", (double)_Helper->get_position_update_rate());
		warnx("rate velocity: \t%6.2f Hz", (double)_Helper->get_velocity_update_rate());
		warnx("rate publication:\t%6.2f Hz", (double)_rate);

	}

	usleep(100000);
}

/**
 * Local functions in support of the shell command.
 */
namespace gps
{

GPS	*g_dev;

void	start(const char *uart_path);
void	stop();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start(const char *uart_path)
{
	int fd;

	if (g_dev != nullptr)
		errx(1, "already started");

	/* create the driver */
	g_dev = new GPS(uart_path);

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	fd = open(GPS_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		errx(1, "Could not open device path: %s\n", GPS_DEVICE_PATH);
		goto fail;
	}

	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Stop the driver.
 */
void
stop()
{
	delete g_dev;
	g_dev = nullptr;

	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(GPS_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "failed ");

	if (ioctl(fd, SENSORIOCRESET, 0) < 0)
		err(1, "driver reset failed");

	exit(0);
}

/**
 * Print the status of the driver.
 */
void
info()
{
	if (g_dev == nullptr)
		errx(1, "driver not running");

	g_dev->print_info();

	exit(0);
}

} // namespace


int
gps_main(int argc, char *argv[])
{

	/* set to default */
	char *device_name = GPS_DEFAULT_UART_PORT;

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		/* work around getopt unreliability */
		if (argc > 3) {
			if (!strcmp(argv[2], "-d")) {
				device_name = argv[3];

			} else {
				goto out;
			}
		}

		gps::start(device_name);
	}

	if (!strcmp(argv[1], "stop"))
		gps::stop();

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
		gps::test();

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset"))
		gps::reset();

	/*
	 * Print driver status.
	 */
	if (!strcmp(argv[1], "status"))
		gps::info();

out:
	errx(1, "unrecognized command, try 'start', 'stop', 'test', 'reset' or 'status' [-d /dev/ttyS0-n]");
}
