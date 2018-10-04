/****************************************************************************
 *
 *   Copyright (C) 2017  Intel Corporation. All rights reserved.
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

#include <px4_config.h>
#include <px4_workqueue.h>
#include <px4_getopt.h>
#include <px4_defines.h>

#include <string.h>
#include <termios.h>

#include <arch/board/board.h>

#include <drivers/device/device.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_hrt.h>

#include <perf/perf_counter.h>

#include <uORB/topics/distance_sensor.h>

#define MIN_DISTANCE 0.01f
#define MAX_DISTANCE 40.0f

#define NAME "leddar_one"
#define DEVICE_PATH "/dev/" NAME

#define SENSOR_READING_FREQ 10.0f
#define READING_USEC_PERIOD (unsigned long)(1000000.0f / SENSOR_READING_FREQ)
#define OVERSAMPLE 6
#define WORK_USEC_INTERVAL READING_USEC_PERIOD / OVERSAMPLE
#define COLLECT_USEC_TIMEOUT READING_USEC_PERIOD / (OVERSAMPLE / 2)

/* 0.5sec */
#define PROBE_USEC_TIMEOUT 500000

#define MODBUS_SLAVE_ADDRESS 0x01
#define MODBUS_READING_FUNCTION 0x04
#define READING_START_ADDR 0x14
#define READING_LEN 0xA

static const uint8_t request_reading_msg[] = {
	MODBUS_SLAVE_ADDRESS,
	MODBUS_READING_FUNCTION,
	0, /* starting addr high byte */
	READING_START_ADDR,
	0, /* number of bytes to read high byte */
	READING_LEN,
	0x30, /* CRC low */
	0x09 /* CRC high */
};

struct __attribute__((__packed__)) reading_msg {
	uint8_t slave_addr;
	uint8_t function;
	uint8_t len;
	uint8_t low_timestamp_high_byte;
	uint8_t low_timestamp_low_byte;
	uint8_t high_timestamp_high_byte;
	uint8_t high_timestamp_low_byte;
	uint8_t temp_high;
	uint8_t temp_low;
	uint8_t num_detections_high_byte;
	uint8_t num_detections_low_byte;
	uint8_t first_dist_high_byte;
	uint8_t first_dist_low_byte;
	uint8_t first_amplitude_high_byte;
	uint8_t first_amplitude_low_byte;
	uint8_t second_dist_high_byte;
	uint8_t second_dist_low_byte;
	uint8_t second_amplitude_high_byte;
	uint8_t second_amplitude_low_byte;
	uint8_t third_dist_high_byte;
	uint8_t third_dist_low_byte;
	uint8_t third_amplitude_high_byte;
	uint8_t third_amplitude_low_byte;
	uint16_t crc; /* little-endian */
};

class leddar_one : public cdev::CDev
{
public:
	leddar_one(const char *device_path, const char *serial_port, uint8_t rotation);
	virtual ~leddar_one();

	virtual int init();
	void start();
	void stop();

	virtual ssize_t	read(struct file *filp, char *buffer, size_t buflen);

private:
	work_s _work;

	int _fd = -1;
	const char *_serial_port;

	uint8_t _rotation;

	uint8_t _buffer[sizeof(struct reading_msg)];
	uint8_t _buffer_len = 0;

	orb_advert_t _topic = nullptr;
	ringbuffer::RingBuffer *_reports = nullptr;

	enum {
		state_waiting_reading = 0,
		state_reading_requested
	} _state = state_waiting_reading;
	hrt_abstime _timeout_usec = 0;

	perf_counter_t _collect_timeout_perf;
	perf_counter_t _comm_error;
	perf_counter_t _sample_perf;

	int _fd_open();
	bool _request();
	int _collect();
	void _publish(uint16_t distance_cm);
	int _cycle();

	static void cycle_trampoline(void *arg);
};

extern "C" __EXPORT int leddar_one_main(int argc, char *argv[]);

static void help()
{
	printf("missing command: try 'start', 'stop' or 'test'\n");
	printf("options:\n");
	printf("    -d <serial port> to set the serial port were " NAME " is connected\n");
	printf("    -r rotation\n");
}

int leddar_one_main(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	const char *serial_port = "/dev/ttyS3";
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	static leddar_one *inst = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:r", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			serial_port = myoptarg;
			break;

		case 'r':
			rotation = (uint8_t)atoi(myoptarg);
			break;

		default:
			help();
			return PX4_ERROR;
		}
	}

	if (myoptind >= argc) {
		help();
		return PX4_ERROR;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		inst = new leddar_one(DEVICE_PATH, serial_port, rotation);

		if (!inst) {
			PX4_ERR("No memory to allocate " NAME);
			return PX4_ERROR;
		}

		if (inst->init() != PX4_OK) {
			delete inst;
			return PX4_ERROR;
		}

		inst->start();

	} else if (!strcmp(verb, "stop")) {
		delete inst;
		inst = nullptr;

	} else if (!strcmp(verb, "test")) {
		int fd = open(DEVICE_PATH, O_RDONLY);
		ssize_t sz;
		struct distance_sensor_s report;

		if (fd < 0) {
			PX4_ERR("Unable to open %s", DEVICE_PATH);
			return PX4_ERROR;
		}

		sz = read(fd, &report, sizeof(report));
		close(fd);

		if (sz != sizeof(report)) {
			PX4_ERR("No sample available in %s", DEVICE_PATH);
			return PX4_ERROR;
		}

		print_message(report);

	} else {
		help();
		return PX4_ERROR;
	}

	return PX4_OK;
}

leddar_one::leddar_one(const char *device_path, const char *serial_port, uint8_t rotation):
	CDev(device_path),
	_rotation(rotation),
	_collect_timeout_perf(perf_alloc(PC_COUNT, "leddar_one_collect_timeout")),
	_comm_error(perf_alloc(PC_COUNT, "leddar_one_comm_errors")),
	_sample_perf(perf_alloc(PC_ELAPSED, "leddar_one_sample"))
{
	memset(&_work, 0, sizeof(_work));
	_serial_port = strdup(serial_port);
}

leddar_one::~leddar_one()
{
	stop();

	free((char *)_serial_port);

	if (_fd > -1) {
		::close(_fd);
	}

	if (_reports) {
		delete _reports;
	}

	if (_topic) {
		orb_unadvertise(_topic);
	}

	perf_free(_sample_perf);
	perf_free(_collect_timeout_perf);
	perf_free(_comm_error);
}

int leddar_one::_fd_open()
{
	if (!_serial_port) {
		return -1;
	}

	_fd = ::open(_serial_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_fd < 0) {
		return -1;
	}

	struct termios config;

	int r = tcgetattr(_fd, &config);

	if (r) {
		PX4_ERR("Unable to get termios from %s.", _serial_port);
		goto error;
	}

	/* clear: data bit size, two stop bits, parity, hardware flow control */
	config.c_cflag &= ~(CSIZE | CSTOPB | PARENB | CRTSCTS);
	/* set: 8 data bits, enable receiver, ignore modem status lines */
	config.c_cflag |= (CS8 | CREAD | CLOCAL);
	/* turn off output processing */
	config.c_oflag = 0;
	/* clear: echo, echo new line, canonical input and extended input */
	config.c_lflag &= (ECHO | ECHONL | ICANON | IEXTEN);

	r = cfsetispeed(&config, B115200);
	r |= cfsetospeed(&config, B115200);

	if (r) {
		PX4_ERR("Unable to set baudrate");
		goto error;
	}

	r = tcsetattr(_fd, TCSANOW, &config);

	if (r) {
		PX4_ERR("Unable to set termios to %s", _serial_port);
		goto error;
	}

	return PX4_OK;

error:
	::close(_fd);
	_fd = -1;
	return PX4_ERROR;
}

int leddar_one::init()
{
	if (CDev::init()) {
		PX4_ERR("Unable to initialize device\n");
		return -1;
	}

	_reports = new ringbuffer::RingBuffer(2, sizeof(distance_sensor_s));

	if (!_reports) {
		PX4_ERR("No memory to allocate RingBuffer");
		return -1;
	}

	if (_fd_open()) {
		return PX4_ERROR;
	}

	hrt_abstime timeout_usec, now;

	for (now = hrt_absolute_time(), timeout_usec = now + PROBE_USEC_TIMEOUT;
	     now < timeout_usec;
	     now = hrt_absolute_time()) {

		if (_cycle() > 0) {
			printf(NAME " found\n");
			return PX4_OK;
		}

		px4_usleep(1000);
	}

	PX4_ERR("No readings from " NAME);

	return PX4_ERROR;
}

void
leddar_one::stop()
{
	work_cancel(HPWORK, &_work);
}

void leddar_one::start()
{
	/*
	 * file descriptor can only be accessed by the process that opened it
	 * so closing here and it will be opened from the High priority kernel
	 * process
	 */
	::close(_fd);
	_fd = -1;

	work_queue(HPWORK, &_work, (worker_t)&leddar_one::cycle_trampoline, this, USEC2TICK(WORK_USEC_INTERVAL));
}

void
leddar_one::cycle_trampoline(void *arg)
{
	leddar_one *dev = reinterpret_cast<leddar_one * >(arg);

	if (dev->_fd != -1) {
		dev->_cycle();

	} else {
		dev->_fd_open();
	}

	work_queue(HPWORK, &(dev->_work), (worker_t)&leddar_one::cycle_trampoline,
		   dev, USEC2TICK(WORK_USEC_INTERVAL));
}

void leddar_one::_publish(uint16_t distance_mm)
{
	struct distance_sensor_s report;

	report.timestamp = hrt_absolute_time();
	report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
	report.orientation = _rotation;
	report.current_distance = ((float)distance_mm / 1000.0f);
	report.min_distance = MIN_DISTANCE;
	report.max_distance = MAX_DISTANCE;
	report.covariance = 0.0f;
	report.signal_quality = -1;
	report.id = 0;

	_reports->force(&report);

	if (_topic == nullptr) {
		_topic = orb_advertise(ORB_ID(distance_sensor), &report);

	} else {
		orb_publish(ORB_ID(distance_sensor), _topic, &report);
	}
}

bool leddar_one::_request()
{
	/* flush anything in RX buffer */
	tcflush(_fd, TCIFLUSH);

	int r = ::write(_fd, request_reading_msg, sizeof(request_reading_msg));
	return r == sizeof(request_reading_msg);
}

static uint16_t _calc_crc16(const uint8_t *buffer, uint8_t len)
{
	uint16_t crc = 0xFFFF;

	for (uint8_t i = 0; i < len; i++) {
		crc ^= buffer[i];

		for (uint8_t j = 0; j < 8; j++) {
			if (crc & 1) {
				crc = (crc >> 1) ^ 0xA001;

			} else {
				crc >>= 1;
			}
		}
	}

	return crc;
}

/*
 * returns 0 when still waiting for reading_msg, 1 when frame was read or
 * -1 in case of error
 */
int leddar_one::_collect()
{
	struct reading_msg *msg;

	int r = ::read(_fd, _buffer + _buffer_len, sizeof(_buffer) - _buffer_len);

	if (r < 1) {
		return 0;
	}

	_buffer_len += r;

	if (_buffer_len < sizeof(struct reading_msg)) {
		return 0;
	}

	msg = (struct reading_msg *)_buffer;

	if (msg->slave_addr != MODBUS_SLAVE_ADDRESS || msg->function != MODBUS_READING_FUNCTION) {
		return -1;
	}

	const uint16_t crc16_calc = _calc_crc16(_buffer, _buffer_len - 2);

	if (crc16_calc != msg->crc) {
		return -1;
	}

	/* NOTE: little-endian support only */
	_publish(msg->first_dist_high_byte << 8 | msg->first_dist_low_byte);
	return 1;
}

int leddar_one::_cycle()
{
	int ret = 0;
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case state_waiting_reading:
		if (now > _timeout_usec) {
			if (_request()) {
				perf_begin(_sample_perf);
				_buffer_len = 0;
				_state = state_reading_requested;
				_timeout_usec = now + COLLECT_USEC_TIMEOUT;
			}
		}

		break;

	case state_reading_requested:
		ret = _collect();

		if (ret == 1) {
			perf_end(_sample_perf);
			_state = state_waiting_reading;
			_timeout_usec = now + READING_USEC_PERIOD;

		} else {
			if (ret == 0 && now < _timeout_usec) {
				/* still waiting for reading */
				break;
			}

			if (ret == -1) {
				perf_count(_comm_error);

			} else {
				perf_count(_collect_timeout_perf);
			}

			perf_cancel(_sample_perf);
			_state = state_waiting_reading;
			_timeout_usec = 0;
			return _cycle();
		}
	}

	return ret;
}

ssize_t leddar_one::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct distance_sensor_s);
	struct distance_sensor_s *rbuf = reinterpret_cast<struct distance_sensor_s *>(buffer);
	int ret = 0;

	if (count < 1) {
		return -ENOSPC;
	}

	while (count--) {
		if (_reports->get(rbuf)) {
			ret += sizeof(*rbuf);
			rbuf++;
		}
	}

	return ret ? ret : -EAGAIN;
}
