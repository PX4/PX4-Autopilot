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
 * @author Roman Bapst <roman@uaventure.com>
 *
 * Driver for the uLanding radar from Aerotenna
 */

#include <termios.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/device.h>

#include <uORB/topics/distance_sensor.h>

#define ULANDING_MIN_DISTANCE		0.315f
#define ULANDING_MAX_DISTANCE		50.0f
#define RADAR_RANGE_DATA 		0x48

#define RADAR_DEFAULT_PORT		"/dev/ttyS2"	// telem2 on Pixhawk
#define BUF_LEN 		9
#define SENS_VARIANCE 		0.045f * 0.045f		// assume standard deviation to be equal to sensor resolution. Static bench tests have shown that
// the sensor ouput does not vary if the unit is not moved.

extern "C" __EXPORT int ulanding_radar_main(int argc, char *argv[]);

class Radar : public device::CDev
{
public:
	Radar(const char *port = RADAR_DEFAULT_PORT);
	virtual ~Radar();

	virtual int 			init();

	int				start();

private:
	bool				_task_should_exit;
	int 				_task_handle;
	char 				_port[20];
	int				_class_instance;
	int				_orb_class_instance;
	orb_advert_t			_distance_sensor_topic;

	unsigned 	_head;
	unsigned 	_tail;
	uint8_t 	_buf[BUF_LEN];

	static void task_main_trampoline(int argc, char *argv[]);
	void task_main();

	bool read_and_parse(uint8_t *buf, int len, float *range);
	bool is_header_byte(uint8_t c) {return ((c & 0x80) == 0x00 && (c & 0x7F) == RADAR_RANGE_DATA);};
};

namespace radar
{
Radar	*g_dev;
}

Radar::Radar(const char *port) :
	CDev("Radar", RANGE_FINDER0_DEVICE_PATH),
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

	// disable debug() calls
	_debug_enabled = false;

	memset(&_buf[0], 0, sizeof(_buf));
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
			usleep(20000);

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

		/* do regular cdev init */
		ret = CDev::init();

		if (ret != OK) {
			PX4_WARN("cdev init failed");
			break;
		}

		int fd = px4_open(RANGE_FINDER0_DEVICE_PATH, 0);

		if (fd < 0) {
			PX4_WARN("failed to open range finder device");
			ret = 1;
			break;
		}

		px4_close(fd);

		/* open fd */
		fd = px4_open(_port, O_RDWR | O_NOCTTY);

		if (fd < 0) {
			PX4_WARN("failed to open serial device");
			ret = 1;
			break;
		}

		struct termios uart_config;

		int termios_state;

		/* fill the struct for the new configuration */
		tcgetattr(fd, &uart_config);

		/* clear ONLCR flag (which appends a CR for every LF) */
		uart_config.c_oflag &= ~ONLCR;

		/* no parity, one stop bit */
		uart_config.c_cflag	 &= ~(CSTOPB | PARENB);

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

		px4_close(fd);

		_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

		struct distance_sensor_s ds_report = {};
		ds_report.timestamp = hrt_absolute_time();
		ds_report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR;
		ds_report.orientation = 8;
		ds_report.id = 0;
		ds_report.current_distance = -1.0f;	// make evident that this range sample is invalid
		ds_report.covariance = SENS_VARIANCE;

		_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &ds_report,
					 &_orb_class_instance, ORB_PRIO_HIGH);

		if (_distance_sensor_topic == nullptr) {
			DEVICE_LOG("failed to create distance_sensor object. Did you start uOrb?");
			ret = 1;
			break;
		}

	} while (0);

	return ret;
}

void
Radar::task_main_trampoline(int argc, char *argv[])
{
	radar::g_dev->task_main();
}

int
Radar::start()
{
	ASSERT(_task_handle == -1);

	/* start the task */
	_task_handle = px4_task_spawn_cmd("radar",
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
	for (unsigned i = 0; i < len; i++) {

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

	if (num_bytes < 3) {
		return false;
	}

	int index = _head;
	uint8_t no_header_counter = 0;	// counter for bytes which are non header bytes
	uint8_t byte1 = _buf[_head];
	uint8_t byte2 = _buf[_head];

	// go through the buffer backwards starting from the newest byte
	// if we find a header byte and the previous two bytes weren't header bytes
	// then we found the newest package.
	for (unsigned i = 0; i < num_bytes; i++) {
		if (is_header_byte(_buf[index])) {
			if (no_header_counter >= 2) {
				int raw = (byte1 & 0x7F);
				raw += ((byte2 & 0x7F) << 7);

				*range = ((float)raw) * 0.045f;

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

		byte2 = byte1;
		byte1 = _buf[index];

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

	int fd = px4_open(_port, O_RDWR | O_NOCTTY);

	// we poll on data from the serial port
	px4_pollfd_struct_t fds[1];
	fds[0].fd = fd;
	fds[0].events = POLLIN;

	// read buffer, one measurement consists of three bytes
	uint8_t buf[BUF_LEN];

	while (!_task_should_exit) {
		// wait for up to 100ms for data
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		// timed out
		if (pret == 0) {
			continue;
		}

		if (pret < 0) {
			PX4_DEBUG("radar serial port poll error");
			// sleep a bit before next try
			usleep(100000);
			continue;
		}

		if (fds[0].revents & POLLIN) {
			memset(&buf[0], 0, sizeof(buf));
			int len = px4_read(fd, &buf[0], sizeof(buf));

			if (len <= 0) {
				PX4_DEBUG("error reading radar");
			}

			float range = 0;

			if (read_and_parse(&buf[0], len, &range)) {

				struct distance_sensor_s report = {};
				report.timestamp = hrt_absolute_time();
				report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
				report.orientation = 8;
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

				// publish it
				orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);
			}

		}
	}

	px4_close(fd);
}

int ulanding_radar_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		if (radar::g_dev != nullptr) {
			PX4_WARN("driver already started");
			return 0;
		}

		if (argc > 2) {
			radar::g_dev = new Radar(argv[2]);

		} else {
			radar::g_dev = new Radar(RADAR_DEFAULT_PORT);
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
	if (!strcmp(argv[1], "stop")) {
		if (radar::g_dev != nullptr) {
			delete radar::g_dev;
			radar::g_dev = nullptr;

		} else {
			PX4_WARN("driver not running");
		}

		return 0;
	}

	if (!strcmp(argv[1], "info")) {
		PX4_INFO("ulanding radar from Aerotenna");
		PX4_INFO("min distance %.2f m", (double)ULANDING_MIN_DISTANCE);
		PX4_INFO("max distance %.2f m", (double)ULANDING_MAX_DISTANCE);
		PX4_INFO("update rate: 500 Hz");
		return 0;

	}

	PX4_WARN("unrecognized arguments, try: start [device_name], stop, info ");
	return 1;
}
