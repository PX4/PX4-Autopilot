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
 * Driver for the GPS on UART6
 */

#include <nuttx/config.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
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
#include <termios.h>
#include <fcntl.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include <arch/board/board.h>

#include <drivers/drv_hrt.h>

#include <systemlib/perf_counter.h>
#include <systemlib/scheduling_priorities.h>
#include <systemlib/err.h>

#include <drivers/drv_gps.h>

#include <uORB/topics/vehicle_gps_position.h>

#include "ubx.h"

#define SEND_BUFFER_LENGTH 100
#define TIMEOUT 1000000 //1s

#define NUMBER_OF_BAUDRATES 4
#define CONFIG_TIMEOUT 2000000

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
	GPS();
	~GPS();

	virtual int			init();

	virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void				print_info();

private:

	int				_task_should_exit;
	int				_serial_fd;					///< serial interface to GPS
	unsigned			_baudrate;
	unsigned			_baudrates_to_try[NUMBER_OF_BAUDRATES];
	uint8_t				_send_buffer[SEND_BUFFER_LENGTH];
	perf_counter_t			_sample_perf;
	perf_counter_t			_comms_errors;
	perf_counter_t			_buffer_overflows;
	volatile int			_task;						///< worker task

	bool				_response_received;
	bool				_config_needed;
	bool 				_baudrate_changed;
	bool				_mode_changed;
	bool				_healthy;
	gps_driver_mode_t		_mode;
	unsigned 			_messages_received;
	float				_rate;						///< position update rate

	GPS_Helper			*_Helper;					///< class for either UBX, MTK or NMEA helper

	struct vehicle_gps_position_s	_report;					///< the current GPS report
	orb_advert_t			_report_pub;					///< the publication topic, only valid on first valid GPS module message

	void				recv();

	void				config();

	static void			task_main_trampoline(void *arg);


	/**
	 * Worker task.
	 */
	void				task_main(void);

	/**
	 * Set the module baud rate
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


GPS::GPS() :
	CDev("gps", GPS_DEVICE_PATH),
	_task_should_exit(false),
	_baudrates_to_try({9600, 38400, 57600, 115200}),
	_config_needed(true),
	_baudrate_changed(false),
	_mode_changed(true),
	_healthy(false),
	_mode(GPS_DRIVER_MODE_UBX),
	_messages_received(0),
	_Helper(nullptr),
	_rate(0.0f),
	_report_pub(-1)
{
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

	/* start the IO interface task */
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
	case GPS_CONFIGURE_UBX:
		if (_mode != GPS_DRIVER_MODE_UBX) {
			_mode = GPS_DRIVER_MODE_UBX;
			_mode_changed = true;
		}
		break;
	case GPS_CONFIGURE_MTK19:
		if (_mode != GPS_DRIVER_MODE_MTK19) {
			_mode = GPS_DRIVER_MODE_MTK19;
			_mode_changed = true;
		}
		break;
	case GPS_CONFIGURE_MTK16:
		if (_mode != GPS_DRIVER_MODE_MTK16) {
			_mode = GPS_DRIVER_MODE_MTK16;
			_mode_changed = true;
		}
		break;
	case GPS_CONFIGURE_NMEA:
		if (_mode != GPS_DRIVER_MODE_NMEA) {
			_mode = GPS_DRIVER_MODE_NMEA;
			_mode_changed = true;
		}
		break;
	case SENSORIOCRESET:
		cmd_reset();
		break;
	}

	unlock();

	return ret;
}

void
GPS::config()
{
	int length = 0;

	_Helper->configure(_config_needed, _baudrate_changed, _baudrate, _send_buffer, length, SEND_BUFFER_LENGTH);

	/* the message needs to be sent at the old baudrate */
	if (length > 0) {
		if (length != ::write(_serial_fd, _send_buffer, length)) {
			debug("write config failed");
			return;
		}
	}

	if (_baudrate_changed) {
		set_baudrate(_baudrate);
		_baudrate_changed = false;
	}
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
	_serial_fd = ::open("/dev/ttyS3", O_RDWR);

	uint8_t buf[256];
	int baud_i = 0;
	uint64_t time_before_configuration;

	if (_serial_fd < 0) {
		log("failed to open serial port: %d", errno);
		/* tell the dtor that we are exiting, set error code */
		_task = -1;
		_exit(1);
	}

	/* poll descriptor */
	pollfd fds[1];
	fds[0].fd = _serial_fd;
	fds[0].events = POLLIN;
	debug("ready");

	/* lock against the ioctl handler */
	lock();


	_baudrate = _baudrates_to_try[baud_i];
	set_baudrate(_baudrate);

	time_before_configuration = hrt_absolute_time();

	uint64_t last_rate_measurement = hrt_absolute_time();
	unsigned last_rate_count = 0;

	/* loop handling received serial bytes */
	while (!_task_should_exit) {

		if (_mode_changed) {
			if (_Helper != nullptr) {
				delete(_Helper);
				/* set to zero to ensure parser is not used while not instantiated */
				_Helper = nullptr;
			}

			switch (_mode) {
			case GPS_DRIVER_MODE_UBX:
				_Helper = new UBX();
				break;
			case GPS_DRIVER_MODE_MTK19:
				//_Helper = new MTK19();
				break;
			case GPS_DRIVER_MODE_MTK16:
				//_Helper = new MTK16();
				break;
			case GPS_DRIVER_MODE_NMEA:
				//_Helper = new NMEA();
				break;
			default:
				break;
			}
			_mode_changed = false;
		}

		if (_config_needed && time_before_configuration + CONFIG_TIMEOUT < hrt_absolute_time()) {
			baud_i = (baud_i+1)%NUMBER_OF_BAUDRATES;
			_baudrate = _baudrates_to_try[baud_i];
			set_baudrate(_baudrate);
			time_before_configuration = hrt_absolute_time();
		}


		int poll_timeout;
		if (_config_needed) {
			poll_timeout = 50;
		} else {
			poll_timeout = 400;
		}
		/* sleep waiting for data, but no more than the poll timeout */
		unlock();
		int ret = ::poll(fds, sizeof(fds) / sizeof(fds[0]), poll_timeout);
		lock();




		/* this would be bad... */
		if (ret < 0) {
			log("poll error %d", errno);
		} else if (ret == 0) {
			config();
			if (_config_needed == false) {
				_config_needed = true;
				warnx("lost GPS module");
			}
		} else if (ret > 0) {
			/* if we have new data from GPS, go handle it */
			if (fds[0].revents & POLLIN) {
				int count;

				/*
				 * We are here because poll says there is some data, so this
				 * won't block even on a blocking device.  If more bytes are
				 * available, we'll go back to poll() again...
				 */
				count = ::read(_serial_fd, buf, sizeof(buf));

				/* pass received bytes to the packet decoder */
				int j;
				for (j = 0; j < count; j++) {
					_messages_received += _Helper->parse(buf[j], &_report);
				}
				if (_messages_received > 0) {
					if (_config_needed == true) {
						_config_needed = false;
						debug("Found GPS module");
					}

					/* opportunistic publishing - else invalid data would end up on the bus */
					if (_report_pub > 0) {
						orb_publish(ORB_ID(vehicle_gps_position), _report_pub, &_report);
					} else {
						_report_pub = orb_advertise(ORB_ID(vehicle_gps_position), &_report);
					}

					_messages_received = 0;
					last_rate_count++;

					/* measure update rate every 5 seconds */
					if (hrt_absolute_time() - last_rate_measurement > 5000000) {
						_rate = last_rate_count / ((float)((hrt_absolute_time() - last_rate_measurement)) / 1000000.0f);
						last_rate_measurement = hrt_absolute_time();
						last_rate_count = 0;
					}
				}
			}
		}
	}
out:
	debug("exiting");

	::close(_serial_fd);

	/* tell the dtor that we are exiting */
	_task = -1;
	_exit(0);
}

int
GPS::set_baudrate(unsigned baud)
{
	/* process baud rate */
	int speed;

	switch (baud) {
	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	default:
		warnx("ERROR: Unsupported baudrate: %d\n", baud);
		return -EINVAL;
	}

	struct termios uart_config;
	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(_serial_fd, &uart_config);

	/* clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;
	/* no parity, one stop bit */
	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	/* set baud rate */
	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		warnx("ERROR setting config: %d (cfsetispeed, cfsetospeed)\n", termios_state);
		return -1;
	}


	if ((termios_state = tcsetattr(_serial_fd, TCSANOW, &uart_config)) < 0) {
		warnx("ERROR setting baudrate (tcsetattr)\n");
		return -1;
	}

	/* XXX if resetting the parser here, ensure it does exist (check for null pointer) */
}

void
GPS::cmd_reset()
{
	_healthy = false;
}

void
GPS::print_info()
{
	switch (_mode) {
		case GPS_DRIVER_MODE_UBX:
			warnx("protocol: UBX");
			break;
		case GPS_DRIVER_MODE_MTK19:
			warnx("protocol: MTK 1.9");
			break;
		case GPS_DRIVER_MODE_MTK16:
			warnx("protocol: MTK 1.6");
			break;
		case GPS_DRIVER_MODE_NMEA:
			warnx("protocol: NMEA");
			break;
		default:
			break;
	}
	warnx("baudrate: %d, status: %s", _baudrate, (_config_needed) ? "NOT OK" : "OK");
	if (_report.timestamp != 0) {
		warnx("position lock: %dD, last update %4.2f seconds ago", (int)_report.fix_type,
			((float)(hrt_absolute_time() - _report.timestamp) / 1000000.0f));
		warnx("lat: %d, lon: %d, alt: %d", _report.lat, _report.lon, _report.alt);
		warnx("update rate: %6.2f Hz", (double)_rate);
	}
}

/**
 * Local functions in support of the shell command.
 */
namespace gps
{

GPS	*g_dev;

void	start();
void	stop();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start()
{
	int fd;

	if (g_dev != nullptr)
		errx(1, "already started");

	/* create the driver */
	g_dev = new GPS;

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	fd = open(GPS_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		printf("Could not open device path: %s\n", GPS_DEVICE_PATH);
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

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

} // namespace


int
gps_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start"))
		gps::start();

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

	errx(1, "unrecognized command, try 'start', 'stop', 'test', 'reset' or 'status'");
}
