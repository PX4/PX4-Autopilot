/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file cm8jl65.cpp
 * @author Claudio Micheli <claudio@auterion.com>
 *
 * Driver for the Lanbao PSK-CM8JL65-CC5 distance sensor.
 * Make sure to disable MAVLINK messages (MAV_0_CONFIG PARAMETER)
 * on the serial port you connect the sensor,i.e TELEM2.
 *
 */

#include <poll.h>
#include <px4_cli.h>
#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <drivers/device/device.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <perf/perf_counter.h>
#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_workqueue.h>
#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>

using namespace time_literals;

/* Configuration Constants */
#define CM8JL65_TAKE_RANGE_REG     'd'
#define CM8JL65_DEFAULT_PORT       "/dev/ttyS2" // Default serial port on Pixhawk (TELEM2), baudrate 115200
#define CM8JL65_MEASURE_INTERVAL   50_ms        // 50ms default sensor conversion time.

/* Frame start delimiter */
#define START_FRAME_DIGIT1          0xA5
#define START_FRAME_DIGIT2          0x5A

/**
 * Frame format definition
 *   1B     1B      1B              1B            2B
 * | 0xA5 | 0x5A | distance-MSB | distance-LSB | crc-16 |
 *
 * Frame data saved for CRC calculation
 */
#define DISTANCE_MSB_POS            2
#define DISTANCE_LSB_POS            3
#define PARSER_BUF_LENGTH           4

static const unsigned char crc_msb_vector[] = {
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40
};

static const unsigned char crc_lsb_vector[] = {
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
	0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
	0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
	0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
	0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
	0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
	0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
	0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
	0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
	0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
	0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
	0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
	0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB,
	0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
	0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
	0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
	0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
	0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
	0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
	0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
	0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
	0x41, 0x81, 0x80, 0x40
};


class CM8JL65 : public cdev::CDev, public px4::ScheduledWorkItem
{
public:
	/**
	 * Default Constructor
	 * @param port The serial port to open for communicating with the sensor.
	 * @param rotation The sensor rotation relative to the vehicle body.
	 */
	CM8JL65(const char *port = CM8JL65_DEFAULT_PORT, uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);

	/** Virtual destructor */
	virtual ~CM8JL65() override;

	/**
	 * Method : init()
	 * This method initializes the general driver for a range finder sensor.
	 */
	virtual int  init() override;

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_info();

private:

	enum CM8JL65_PARSE_STATE {
		WAITING_FRAME = 0,
		DIGIT_1,
		DIGIT_2,
		MSB_DATA,
		LSB_DATA,
		CHECKSUM
	};

	/**
	 * Calculates the 16 byte crc value for the data frame.
	 * @param data_frame The data frame to compute a checksum for.
	 * @param crc16_length The length of the data frame.
	 */
	uint16_t crc16_calc(const unsigned char *data_frame, uint8_t crc16_length);

	/**
	 * Reads data from serial UART and places it into a buffer.
	 */
	int collect();

	int data_parser(const uint8_t check_byte, uint8_t parserbuf[PARSER_BUF_LENGTH], CM8JL65_PARSE_STATE &state,
			uint16_t &crc16, int &distance);

	/**
	 * Opens and configures the UART serial communications port.
	 * @param speed The baudrate (speed) to configure the serial UART port.
	 */
	int open_serial_port(const speed_t speed = B115200);

	/**
	 * Perform a reading cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void Run() override;

	/**
	 * Initialise the automatic measurement state machine and start it.
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void start();

	/**
	 * Stops the automatic measurement state machine.
	 */
	void stop();

	char _port[20] {};

	unsigned char _frame_data[PARSER_BUF_LENGTH] {START_FRAME_DIGIT1, START_FRAME_DIGIT2, 0, 0};

	int _measure_interval{CM8JL65_MEASURE_INTERVAL};
	int _file_descriptor{-1};
	int _orb_class_instance{-1};

	uint8_t _cycle_counter{0};
	uint8_t _linebuf[25] {};
	uint8_t _rotation{0};

	uint16_t _crc16{0};

	// Use conservative distance bounds, to make sure we don't fuse garbage data
	float _max_distance{7.9f}; // Datasheet: 8.0m
	float _min_distance{0.2f}; // Datasheet: 0.17m

	CM8JL65_PARSE_STATE _parse_state{WAITING_FRAME};

	orb_advert_t _distance_sensor_topic{nullptr};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, "cm8jl65_com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, "cm8jl65_read")};
};


CM8JL65::CM8JL65(const char *port, uint8_t rotation) :
	CDev(RANGE_FINDER_BASE_DEVICE_PATH),
	ScheduledWorkItem(px4::wq_configurations::hp_default),
	_rotation(rotation)
{
	// Store the port name.
	strncpy(_port, port, sizeof(_port) - 1);

	// Enforce null termination.
	_port[sizeof(_port) - 1] = '\0';
}

CM8JL65::~CM8JL65()
{
	// Ensure we are truly inactive.
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

uint16_t
CM8JL65::crc16_calc(const unsigned char *data_frame, uint8_t crc16_length)
{
	// compute CRC16 IBM 8005
	unsigned char crc_high_byte = 0xFF;
	unsigned char crc_low_byte = 0xFF;
	size_t index = 0;

	while (crc16_length--) {
		index = crc_low_byte ^ *(data_frame++);
		crc_low_byte = (unsigned char)(crc_high_byte ^ crc_msb_vector[index]);
		crc_high_byte = crc_lsb_vector[index];
	}

	uint16_t crc16 = (crc_high_byte << 8 | crc_low_byte);
	crc16 = (crc16 >> 8) | (crc16 << 8); // Convert endian

	return crc16;
}

int
CM8JL65::collect()
{
	perf_begin(_sample_perf);

	int bytes_processed = 0;
	int distance_mm = -1;
	int index = 0;

	bool crc_valid = false;

	// Read from the sensor UART buffer.
	int bytes_read = ::read(_file_descriptor, &_linebuf[0], sizeof(_linebuf));

	if (bytes_read > 0) {
		index = bytes_read - 6 ;

		while (index >= 0 && !crc_valid) {
			if (_linebuf[index] == START_FRAME_DIGIT1) {
				bytes_processed = index;

				while (bytes_processed < bytes_read && !crc_valid) {
					if (data_parser(_linebuf[bytes_processed], _frame_data, _parse_state, _crc16, distance_mm) == PX4_OK) {
						crc_valid = true;
					}

					bytes_processed++;
				}

				_parse_state = WAITING_FRAME;
			}

			index--;
		}

	} else {
		PX4_INFO("read error: %d", bytes_read);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return PX4_ERROR;
	}

	if (!crc_valid) {
		return -EAGAIN;
	}

	bytes_read = OK;

	distance_sensor_s report = {};
	report.current_distance = static_cast<float>(distance_mm) / 1000.0f;
	report.id               = 0;	// TODO: set proper ID.
	report.h_fov            = 0.0488692f;
	report.max_distance     = _max_distance;
	report.min_distance     = _min_distance;
	report.orientation      = _rotation;
	report.signal_quality   = report.current_distance < _max_distance && report.current_distance > _min_distance ? -1 : 0;
	report.timestamp        = hrt_absolute_time();
	report.type             = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
	report.variance         = 0.0f;

	// Publish the new report.
	orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);

	// Notify anyone waiting for data.
	poll_notify(POLLIN);
	perf_end(_sample_perf);

	return PX4_OK;
}

int
CM8JL65::data_parser(const uint8_t check_byte, uint8_t parserbuf[PARSER_BUF_LENGTH],
		     CM8JL65_PARSE_STATE &state, uint16_t &crc16, int &distance)
{
	switch (state) {
	case WAITING_FRAME:
		if (check_byte == START_FRAME_DIGIT1) {
			state = DIGIT_1;
		}

		break;

	case DIGIT_1:
		if (check_byte == START_FRAME_DIGIT1) {
			state = DIGIT_1;

		} else if (check_byte == START_FRAME_DIGIT2) {
			state = DIGIT_2;

		} else {
			state = WAITING_FRAME;
		}

		break;

	case DIGIT_2:
		state = MSB_DATA;
		parserbuf[DISTANCE_MSB_POS] = check_byte; // MSB Data
		break;

	case MSB_DATA:
		state = LSB_DATA;
		parserbuf[DISTANCE_LSB_POS] = check_byte; // LSB Data

		// Calculate CRC.
		crc16 = crc16_calc(parserbuf, PARSER_BUF_LENGTH);
		break;

	case LSB_DATA:
		if (check_byte == (crc16 >> 8)) {
			state = CHECKSUM;

		} else {
			state = WAITING_FRAME;

		}

		break;

	case CHECKSUM:
		// Here, reset state to `NOT-STARTED` no matter crc ok or not
		state = WAITING_FRAME;

		if (check_byte == (crc16 & 0xFF)) {
			// printf("Checksum verified \n");
			distance = (parserbuf[DISTANCE_MSB_POS] << 8) | parserbuf[DISTANCE_LSB_POS];
			return PX4_OK;
		}

		break;

	default:
		// Nothing todo
		break;
	}

	return PX4_ERROR;
}

int
CM8JL65::init()
{
	// Intitialize the character device.
	if (CDev::init() != OK) {
		return PX4_ERROR;
	}

	// Get a publish handle on the range finder topic.
	distance_sensor_s ds_report = {};
	_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &ds_report,
				 &_orb_class_instance, ORB_PRIO_HIGH);

	if (_distance_sensor_topic == nullptr) {
		PX4_ERR("failed to create distance_sensor object");
	}

	start();
	return PX4_OK;
}

int
CM8JL65::open_serial_port(const speed_t speed)
{
	// File descriptor initialized?
	if (_file_descriptor > 0) {
		// PX4_INFO("serial port already open");
		return PX4_OK;
	}

	// Configure port flags for read/write, non-controlling, non-blocking.
	int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

	// Open the serial port.
	_file_descriptor = ::open(_port, flags);

	if (_file_descriptor < 0) {
		PX4_ERR("open failed (%i)", errno);
		return PX4_ERROR;
	}

	termios uart_config = {};

	// Store the current port configuration. attributes.
	tcgetattr(_file_descriptor, &uart_config);

	// Clear ONLCR flag (which appends a CR for every LF).
	uart_config.c_oflag &= ~ONLCR;

	// No parity, one stop bit.
	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	// Set the input baud rate in the uart_config struct.
	int termios_state = cfsetispeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d ISPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Set the output baud rate in the uart_config struct.
	termios_state = cfsetospeed(&uart_config, speed);

	if (termios_state < 0) {
		PX4_ERR("CFG: %d OSPD", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	// Apply the modified port attributes.
	termios_state = tcsetattr(_file_descriptor, TCSANOW, &uart_config);

	if (termios_state < 0) {
		PX4_ERR("baud %d ATTR", termios_state);
		::close(_file_descriptor);
		return PX4_ERROR;
	}

	PX4_INFO("successfully opened UART port %s", _port);
	return PX4_OK;
}

void
CM8JL65::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}

void
CM8JL65::Run()
{
	// Ensure the serial port is open.
	open_serial_port();

	// Perform collection.
	if (collect() == -EAGAIN) {
		_cycle_counter++;
	}

	_cycle_counter = 0;
}

void
CM8JL65::start()
{
	// Schedule the driver at regular intervals.
	ScheduleOnInterval(CM8JL65_MEASURE_INTERVAL, 0);
	PX4_INFO("driver started");
}

void
CM8JL65::stop()
{
	// Ensure the serial port is closed.
	::close(_file_descriptor);

	// Clear the work queue schedule.
	ScheduleClear();
}


/**
 * Local functions in support of the shell command.
 */
namespace cm8jl65
{

CM8JL65	*g_dev;

int reset(const char *port = CM8JL65_DEFAULT_PORT);
int start(const char *port = CM8JL65_DEFAULT_PORT,
	  const uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
int status();
int stop();
int test(const char *port = CM8JL65_DEFAULT_PORT);
int usage();

/**
 * Reset the driver.
 */
int
reset(const char *port)
{
	if (stop() == PX4_OK) {
		return start(port);
	}

	return PX4_ERROR;
}

/**
 * Start the driver.
 */
int
start(const char *port, const uint8_t rotation)
{
	if (g_dev != nullptr) {
		PX4_INFO("already started");
		return PX4_OK;
	}

	// Instantiate the driver.
	g_dev = new CM8JL65(port, rotation);

	if (g_dev == nullptr) {
		PX4_ERR("object instantiate failed");
		return PX4_ERROR;
	}

	if (g_dev->init() != PX4_OK) {
		PX4_ERR("driver start failed");
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

	return PX4_OK;
}

/**
 * Print the driver status.
 */
int
status()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	return PX4_OK;
}

/**
 * Stop the driver
 */
int stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	return PX4_ERROR;
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
int
test(const char *port)
{
	int fd = open(port, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("%s open failed (try 'cm8jl65 start' if the driver is not running", port);
		return PX4_ERROR;
	}

	// Perform a simple demand read.
	distance_sensor_s report;
	ssize_t bytes_read = read(fd, &report, sizeof(report));

	if (bytes_read != sizeof(report)) {
		PX4_ERR("immediate read failed");
		return PX4_ERROR;
	}

	print_message(report);
	return PX4_OK;
}

int
usage()
{
	PX4_INFO("usage: cm8jl65 command [options]");
	PX4_INFO("command:");
	PX4_INFO("\treset|start|status|stop|test");
	PX4_INFO("options:");
	PX4_INFO("\t-R --rotation (%d)", distance_sensor_s::ROTATION_DOWNWARD_FACING);
	PX4_INFO("\t-d --device_path");
	return PX4_OK;
}

} // namespace cm8jl65


/**
 * Driver 'main' command.
 */
extern "C" __EXPORT int cm8jl65_main(int argc, char *argv[])
{
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	const char *device_path = CM8JL65_DEFAULT_PORT;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "R:d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R': {
				int rot = -1;

				if (px4_get_parameter_value(myoptarg, rot) != 0) {
					PX4_ERR("rotation parsing failed");
					return -1;
				}

				rotation = (uint8_t)rot;
				break;
			}

		case 'd':
			device_path = myoptarg;
			break;

		default:
			PX4_WARN("Unknown option!");
			return cm8jl65::usage();
		}
	}

	if (myoptind >= argc) {
		return cm8jl65::usage();
	}

	// Reset the driver.
	if (!strcmp(argv[myoptind], "reset")) {
		return cm8jl65::reset(device_path);
	}

	// Start/load the driver.
	if (!strcmp(argv[myoptind], "start")) {
		return cm8jl65::start(device_path, rotation);
	}

	// Print driver information.
	if (!strcmp(argv[myoptind], "status")) {
		return cm8jl65::status();
	}

	// Stop the driver
	if (!strcmp(argv[myoptind], "stop")) {
		return cm8jl65::stop();
	}

	// Test the driver/device.
	if (!strcmp(argv[myoptind], "test")) {
		return cm8jl65::test();
	}

	return cm8jl65::usage();
}
