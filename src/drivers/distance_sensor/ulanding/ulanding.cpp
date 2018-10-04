/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file ulanding.cpp
 * @author Jessica Stockham <jessica@aerotenna.com>
 * @author Roman Bapst <roman@uaventure.com>
 *
 * Driver for the uLanding radar from Aerotenna
 */

#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_workqueue.h>
#include <px4_defines.h>
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
#include <vector>
#include <perf/perf_counter.h>
#include <systemlib/err.h>
#include <drivers/device/ringbuffer.h>
#include <stdio.h>
#include <uORB/uORB.h>
#include <termios.h>


#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/device.h>

#include <uORB/topics/distance_sensor.h>

namespace ulanding
{

#define ULANDING_MIN_DISTANCE		0.315f
#define ULANDING_MAX_DISTANCE		50.0f
#define ULANDING_VERSION	1

#if defined(__PX4_POSIX_OCPOC)
#define RADAR_DEFAULT_PORT		"/dev/ttyS6"	// default uLanding port on OCPOC
#else
#define RADAR_DEFAULT_PORT		"/dev/ttyS2"	// telem2 on Pixhawk
#endif

#if ULANDING_VERSION == 1
#define ULANDING_HDR		254
#define BUF_LEN 		18
#else
#define ULANDING_HDR		72
#define BUF_LEN 		9
#endif

/* assume standard deviation to be equal to sensor resolution.
   Static bench tests have shown that the sensor ouput does
   not vary if the unit is not moved. */
#define SENS_VARIANCE 		0.045f * 0.045f


extern "C" __EXPORT int ulanding_radar_main(int argc, char *argv[]);

class Radar : public cdev::CDev
{
public:
	Radar(uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING, const char *port = RADAR_DEFAULT_PORT);
	virtual ~Radar();

	virtual int 			init();

	int				start();

private:
	uint8_t _rotation;
	bool				_task_should_exit;
	int 				_task_handle;
	char 				_port[20];
	int				_class_instance;
	int				_orb_class_instance;
	orb_advert_t			_distance_sensor_topic;

	unsigned 	_head;
	unsigned 	_tail;
	uint8_t 	_buf[BUF_LEN] {};

	static int task_main_trampoline(int argc, char *argv[]);
	void task_main();

	bool read_and_parse(uint8_t *buf, int len, float *range);

	bool is_header_byte(uint8_t c) {return (c == ULANDING_HDR);};
};

namespace radar
{
Radar	*g_dev;
}

Radar::Radar(uint8_t rotation, const char *port) :
	CDev(RANGE_FINDER0_DEVICE_PATH),
	_rotation(rotation),
	_task_should_exit(false),
	_task_handle(-1),
	_class_instance(-1),
	_orb_class_instance(-1),
	_distance_sensor_topic(nullptr),
	_head(0),
	_tail(0)

{
	/* store port name */
	strncpy(_port, port, sizeof(_port));
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';
}

Radar::~Radar()
{

	if (_class_instance != -1) {
		unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
	}

	if (_task_handle != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			px4_usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_task_handle);
				break;
			}
		} while (_task_handle != -1);
	}

	if (_class_instance != -1) {
		unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
	}

	orb_unadvertise(_distance_sensor_topic);
}

int
Radar::init()
{
	/* status */
	int ret = 0;

	do { /* create a scope to handle exit conditions using break */

		ret = CDev::init();

		if (ret != OK) {
			PX4_WARN("vdev init failed");
			break;
		}

		/* open fd */
		int fd = ::open(_port, O_RDWR | O_NOCTTY);
		PX4_INFO("passed open port");

		if (fd < 0) {
			PX4_WARN("failed to open serial device");
			ret = 1;
			break;
		}

		struct termios uart_config;

		int termios_state;

		/* fill the struct for the new configuration */
		tcgetattr(fd, &uart_config);

		/** Input flags - Turn off input processing
		 *
		 * convert break to null byte, no CR to NL translation,
		 * no NL to CR translation, don't mark parity errors or breaks
		 * no input parity check, don't strip high bit off,
		 * no XON/XOFF software flow control
		 *
		 */

		uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
					 INLCR | PARMRK | INPCK | ISTRIP | IXON);

		/** No line processing
		 *
		 * echo off, echo newline off, canonical mode off,
		 * extended input processing off, signal chars off
		 *
		 */

		uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

		/* clear ONLCR flag (which appends a CR for every LF) */
		uart_config.c_oflag &= ~ONLCR;

		/* no parity, one stop bit */
		uart_config.c_cflag &= ~(CSTOPB | PARENB);



		unsigned speed = B115200;

		/* set baud rate */
		if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
			PX4_WARN("ERR CFG: %d ISPD", termios_state);
			ret = 1;
			break;
		}

		if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
			PX4_WARN("ERR CFG: %d OSPD\n", termios_state);
			ret = 1;
			break;
		}

		if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
			PX4_WARN("ERR baud %d ATTR", termios_state);
			ret = 1;
			break;
		}

		::close(fd);

		_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

		struct distance_sensor_s ds_report = {};
		ds_report.timestamp = hrt_absolute_time();
		ds_report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR;
		ds_report.orientation = _rotation;
		ds_report.id = 0;
		ds_report.current_distance = -1.0f;	// make evident that this range sample is invalid
		ds_report.covariance = SENS_VARIANCE;

		_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &ds_report,
					 &_orb_class_instance, ORB_PRIO_HIGH);

		if (_distance_sensor_topic == nullptr) {
			PX4_ERR("failed to create distance_sensor object");
			ret = 1;
			break;
		}

	} while (0);

	return ret;
}

int
Radar::task_main_trampoline(int argc, char *argv[])
{
	radar::g_dev->task_main();
	return 0;
}

int
Radar::start()
{
	ASSERT(_task_handle == -1);

	/* start the task */
	_task_handle = px4_task_spawn_cmd("ulanding_radar",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_MAX - 30,
					  800,
					  (px4_main_t)&Radar::task_main_trampoline,
					  nullptr);

	if (_task_handle < 0) {
		PX4_WARN("task start failed");
		return -errno;
	}

	return OK;
}

bool Radar::read_and_parse(uint8_t *buf, int len, float *range)
{
	bool ret = false;

	// write new data into a ring buffer
	for (int i = 0; i < len; i++) {

		_head++;

		if (_head >= BUF_LEN) {
			_head = 0;
		}

		if (_tail == _head) {
			_tail = (_tail == BUF_LEN - 1) ? 0 : _head + 1;
		}

		_buf[_head] = buf[i];
	}

	// check how many bytes are in the buffer, return if it's lower than the size of one package
	int num_bytes = _head >= _tail ? (_head - _tail + 1) : (_head + 1 + BUF_LEN - _tail);

	if (num_bytes < BUF_LEN / 3) {
		PX4_DEBUG("not enough bytes");
		return false;
	}

	int index = _head;
	uint8_t no_header_counter = 0;	// counter for bytes which are non header bytes

	// go through the buffer backwards starting from the newest byte
	// if we find a header byte and the previous two bytes weren't header bytes
	// then we found the newest package.
	for (int i = 0; i < num_bytes; i++) {
		if (is_header_byte(_buf[index])) {
			if (no_header_counter >= BUF_LEN / 3 - 1) {
				if (ULANDING_VERSION == 1) {
					if (((_buf[index + 1] + _buf[index + 2] + _buf[index + 3] + _buf[index + 4])) != (_buf[index + 5])
					    || (_buf[index + 1] <= 0)) {
						ret = false;
						break;
					}

					int raw = _buf[index + 3] * 256 + _buf[index + 2];

					*range = ((float)(raw / 100.)); // raw is in cm, converts range to meters;

				} else {
					int raw = (_buf[index + 1] & 0x7F);
					raw += ((_buf[index + 2] & 0x7F) << 7);

					*range = ((float)raw) * 0.045f;
				}

				// set the tail to one after the index because we neglect
				// any data before the one we just read
				_tail = index == BUF_LEN - 1 ? 0 : index++;
				ret = true;
				break;

			}

			no_header_counter = 0;

		} else {
			no_header_counter++;
		}

		index--;

		if (index < 0) {
			index = BUF_LEN - 1;
		}

	}

	return ret;
}

void
Radar::task_main()
{
	int fd = ::open(_port, O_RDWR | O_NOCTTY);

	if (fd < 0) {
		PX4_WARN("serial port not open");
	}

	if (!isatty(fd)) {
		PX4_WARN("not a serial device");
	}


	// we poll on data from the serial port
	pollfd fds[1];
	fds[0].fd = fd;
	fds[0].events = POLLIN;

	// read buffer
	uint8_t buf[BUF_LEN];

	while (!_task_should_exit) {
		// wait for up to 100ms for data
		int pret = ::poll(fds, (sizeof(fds) / sizeof(fds[0])), 100);

		// timed out
		if (pret == 0) {
			continue;
		}

		if (pret < 0) {
			PX4_DEBUG("radar serial port poll error");
			// sleep a bit before next try
			px4_usleep(100000);
			continue;
		}

		if (fds[0].revents & POLLIN) {
			memset(&buf[0], 0, sizeof(buf));
			int len = ::read(fd, &buf[0], sizeof(buf));

			if (len <= 0) {
				PX4_DEBUG("error reading radar");
			}

			float range = 0;

			if (read_and_parse(&buf[0], len, &range)) {

				struct distance_sensor_s report = {};
				report.timestamp = hrt_absolute_time();
				report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
				report.orientation = _rotation;
				report.current_distance = range;
				report.current_distance = report.current_distance > ULANDING_MAX_DISTANCE ? ULANDING_MAX_DISTANCE :
							  report.current_distance;
				report.current_distance = report.current_distance < ULANDING_MIN_DISTANCE ? ULANDING_MIN_DISTANCE :
							  report.current_distance;
				report.min_distance = ULANDING_MIN_DISTANCE;
				report.max_distance = ULANDING_MAX_DISTANCE;
				report.covariance = SENS_VARIANCE;
				report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR;
				report.id = 0;

				// publish radar data
				orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);
			}

		}
	}

	::close(fd);
}

int ulanding_radar_main(int argc, char *argv[])
{
	if (argc <= 1) {
		PX4_WARN("not enough arguments, usage: start [device_path], stop, info ");
		return 1;
	}

	// check for optional arguments
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;


	while ((ch = px4_getopt(argc, argv, "R:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (uint8_t)atoi(myoptarg);
			PX4_INFO("Setting distance sensor orientation to %d", (int)rotation);
			break;

		default:
			PX4_WARN("Unknown option!");
		}
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[myoptind], "start")) {
		if (radar::g_dev != nullptr) {
			PX4_WARN("driver already started");
			return 0;
		}

		if (argc > myoptind + 1) {
			radar::g_dev = new Radar(rotation, argv[myoptind + 1]);

		} else {
			radar::g_dev = new Radar(rotation, RADAR_DEFAULT_PORT);
		}

		if (radar::g_dev == nullptr) {
			PX4_ERR("failed to create instance of Radar");
			return 1;
		}

		if (PX4_OK != radar::g_dev->init()) {
			delete radar::g_dev;
			radar::g_dev = nullptr;
			return 1;
		}

		if (OK != radar::g_dev->start()) {
			delete radar::g_dev;
			radar::g_dev = nullptr;
			return 1;
		}

		return 0;
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[myoptind], "stop")) {
		if (radar::g_dev != nullptr) {
			delete radar::g_dev;
			radar::g_dev = nullptr;

		} else {
			PX4_WARN("driver not running");
		}

		return 0;
	}

	if (!strcmp(argv[myoptind], "info")) {
		PX4_INFO("ulanding radar from Aerotenna");
		PX4_INFO("min distance %.2f m", (double)ULANDING_MIN_DISTANCE);
		PX4_INFO("max distance %.2f m", (double)ULANDING_MAX_DISTANCE);
		PX4_INFO("update rate: 500 Hz");

		return 0;

	}

	PX4_WARN("unrecognized arguments, try: start [device_path], stop, info ");
	return 1;
}
}; //namespace
