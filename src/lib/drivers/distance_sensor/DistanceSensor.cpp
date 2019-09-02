/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file DistanceSensor.cpp
 * Distance Sensor driver base class.
 */

#include "DistanceSensor.h"

DistanceSensor::DistanceSensor() :
	ScheduledWorkItem(px4::wq_configurations::hp_default)
{
}

DistanceSensor::~DistanceSensor()
{
	// Ensure we are truly inactive.
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int
DistanceSensor::collect()
{
	perf_begin(_sample_perf);

	if (!_collect_phase) {
		return measure();
	}

	distance_sensor_s report;
	report.current_distance = get_distance();
	report.id               = get_sensor_id();
	report.max_distance     = _max_distance;
	report.min_distance     = _min_distance;
	report.orientation      = get_sensor_orientation();
	report.signal_quality   = -1;
	report.timestamp        = hrt_absolute_time();
	report.type             = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
	report.variance         = get_sensor_variance();

	// Publish the new report.
	orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);
	perf_end(_sample_perf);
	return PX4_OK;
}

float
DistanceSensor::get_field_of_view()
{
	return _field_of_view;
}

uint8_t
DistanceSensor::get_sensor_id()
{
	return 0;
}

uint8_t
DistanceSensor::get_sensor_orientation()
{
	return _orientation;
}

uint8_t
DistanceSensor::get_sensor_variance()
{
	return _variance;
}

int
DistanceSensor::init()
{
	// Intitialize the device.
	if (dev_init() != OK) {
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
DistanceSensor::measure()
{
	return PX4_OK;
}

int
DistanceSensor::open_serial_port(const speed_t speed)
{
	// File descriptor initialized?
	if (_file_descriptor > 0) {
		// PX4_INFO("serial port already open");
		return PX4_OK;
	}

	// Configure port flags for read/write, non-controlling, non-blocking.
	int flags = (O_RDWR | O_NOCTTY | O_NONBLOCK);

	// Open the serial port.
	_file_descriptor = ::open(_serial_port, flags);

	if (_file_descriptor < 0) {
		PX4_ERR("open failed (%i)", errno);
		return PX4_ERROR;
	}

	if (!isatty(_file_descriptor)) {
		PX4_WARN("not a serial device");
		return PX4_ERROR;
	}

	termios uart_config = {};

	// Store the current port configuration. attributes.
	tcgetattr(_file_descriptor, &uart_config);

	// Input flags - Turn off input processing, convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks, no input parity check,
	// don't strip high bit off, no XON/XOFF software flow control
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

	// Clear ONLCR flag (which appends a CR for every LF).
	uart_config.c_oflag &= ~ONLCR;

	// No parity, one stop bit.
	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	// No line processing - echo off, echo newline off, canonical mode off, extended input processing off, signal chars off
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

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

	PX4_INFO("successfully opened UART port %s", _serial_port);
	return PX4_OK;
}

void
DistanceSensor::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	PX4_INFO("max distance %.2f m", static_cast<double>(_max_distance));
	PX4_INFO("min distance %.2f m", static_cast<double>(_min_distance));
	PX4_INFO("measure interval:  %u msec", _measure_interval / 1000);
}

void
DistanceSensor::Run()
{
	// Perform collection.
	collect();
}

void
DistanceSensor::set_sensor_orientation(const uint8_t orientation)
{
	_orientation = orientation;
}

void
DistanceSensor::start()
{
	// Schedule the driver at regular intervals.
	ScheduleOnInterval(_measure_interval);
	PX4_INFO("driver started");
}

void
DistanceSensor::stop()
{
	// Clear the work queue schedule.
	ScheduleClear();
}
