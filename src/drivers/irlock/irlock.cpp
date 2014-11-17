/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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

/*
 * @file irlock.cpp
 * @author Michael Landes
 *
 * Driver for the IRLock sensor connected via I2C.
 *
 *  Created on: Nov 12, 2014
 */

#include <fcntl.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

#include "drivers/boards/px4fmu-v2/board_config.h"
#include "drivers/device/i2c.h"
#include "drivers/device/ringbuffer.h"
#include "drivers/drv_irlock.h"
#include "drivers/drv_hrt.h"

#include "nuttx/clock.h"
#include "nuttx/wqueue.h"
#include "systemlib/err.h"

#include "uORB/topics/irlock.h"
#include "uORB/topics/subsystem_info.h"
#include "uORB/uORB.h"

/* Configuration Constants */
#define IRLOCK_BUS            PX4_I2C_BUS_EXPANSION
#define I2C_IRLOCK_ADDRESS    0x65 //* 7-bit address (non shifted)
#define IRLOCK_CONVERSION_INTERVAL 20000 /* us = 20ms = 50Hz */

#define IRLOCK_SYNC			0xAA55
#define IRLOCK_RESYNC		0x5500
#define IRLOCK_ADJUST		0xAA

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class IRLOCK : public device::I2C
{
public:
	IRLOCK(int bus = IRLOCK_BUS, int address = I2C_IRLOCK_ADDRESS);
	virtual ~IRLOCK();

	virtual int init();

	void print_diagnostics();

	virtual int ioctl(struct file *filp, int cmd, unsigned long arg);
	virtual ssize_t read(struct file *filp, char *buffer, size_t buflen);

protected:
	virtual int probe();

private:
	// static callback for the work queue, arg will be pointer to instance of this class
	static void cycle_trampoline(void *arg);

	RingBuffer *_reports;
	orb_advert_t _irlock_topic;
	bool _sensor_ok;
	int _measure_ticks;
	work_s _work;

	// irlock device io
	int read_device();
	bool sync_device();
	int read_device_word(uint16_t *word);
	int read_device_block(struct irlock_s *block);

	// driver control
	void start();
	void stop();
	void cycle();
};

extern "C" __EXPORT int irlock_main(int argc, char *argv[]);

IRLOCK::IRLOCK(int bus, int address) :
		I2C("IRLOCK", IRLOCK_DEVICE_PATH, bus, address, 400000), // 400 khz - supposedly pointless number...
		_reports(nullptr),
		_irlock_topic(-1),
		_sensor_ok(false),
		_measure_ticks(0)
{
	memset(&_work, 0, sizeof(_work));
}

IRLOCK::~IRLOCK()
{
	stop();

	if (_reports != nullptr) {
		delete _reports;
	}
}

int IRLOCK::init()
{
	int status = ERROR;

	if (I2C::init() != OK) {
		goto finish;
	}

	_reports = new RingBuffer(135, sizeof(struct irlock_s));
	if (_reports == nullptr)
		goto finish;

	status = OK;
	_sensor_ok = true;

finish:
	return status;
}

bool IRLOCK::sync_device()
{
	uint8_t sync_byte;
	uint16_t sync_word;

	if (read_device_word(&sync_word) != OK)
		return false;

	if (sync_word == IRLOCK_RESYNC) {
		transfer(nullptr, 0, &sync_byte, 1);
		if (sync_byte == IRLOCK_ADJUST)
			return true;
	} else if (sync_word == IRLOCK_SYNC) {
		return true;
	}

	return false;
}

int IRLOCK::probe()
{
	/*
	 * IRLock defaults to sending 0x00 when there is no block
	 * data to return, so really all we can do is check to make
	 * sure a transfer completes successfully.
	 */
	uint8_t byte;
	if (transfer(nullptr, 0, &byte, 1) != OK) {
		return -EIO;
	}

	return OK;
}

int IRLOCK::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch(cmd) {
	case SENSORIOCSPOLLRATE: // set poll rate (and start polling)
		switch (arg) {
		case SENSOR_POLLRATE_MAX:
		case SENSOR_POLLRATE_DEFAULT: {
			bool stopped = (_measure_ticks == 0);

			_measure_ticks = USEC2TICK(IRLOCK_CONVERSION_INTERVAL);
			if (stopped) {
				start();
			}

			return OK;
		}
		// not yet supporting manual, custom or external polling
		case SENSOR_POLLRATE_MANUAL:
			warnx("manual polling not currently supported");
			return -EINVAL;
		case SENSOR_POLLRATE_EXTERNAL:
			warnx("external polling not currently supported");
			return -EINVAL;
		default:
			warnx("custom poll-rates not currently supported");
			return -EINVAL;
		}
		warnx("set poll rate");
		return OK;
	case SENSORIOCRESET:
		warnx("reset not implemented, use stop and then start");
		return -EINVAL;
	default:
		warnx("default ioctl call");
		return OK; // I2C::ioctl(filp, cmd, arg); - causes errors for some reason
	}
}

ssize_t IRLOCK::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct irlock_s);
	struct irlock_s *rbuf = reinterpret_cast<struct irlock_s *>(buffer);
	int ret = 0;

	if (count < 1)
		return -ENOSPC;

	// only try to read if we are using automatic polling in the driver
	if (_measure_ticks > 0) {
		while (count--) {
			if (_reports->get(rbuf)) {
				ret += sizeof(*rbuf);
				++rbuf;
			}
		}

		return ret ? ret : -EAGAIN;
	}

	return ret;
}

int IRLOCK::read_device()
{
	// if we sync, then we are starting a new frame, else fail
	if (!sync_device())
		return ERROR;

	// now read blocks until sync stops, first flush stale queue data
	_reports->flush();
	while (sync_device()) {
		struct irlock_s block;
		if (read_device_block(&block) != OK)
			break;

		_reports->force(&block);
	}

	return OK;
}

int IRLOCK::read_device_word(uint16_t *word)
{
	uint8_t bytes[2];
	memset(bytes, 0, sizeof bytes);

	int status = transfer(nullptr, 0, &bytes[0], 2);
	*word = bytes[1] << 8 | bytes[0];

	return status;
}

int IRLOCK::read_device_block(struct irlock_s *block)
{
	uint8_t bytes[12];
	memset(bytes, 0, sizeof bytes);

	int status = transfer(nullptr, 0, &bytes[0], 12);
	uint16_t checksum = bytes[1] << 8 | bytes[0];
	block->signature = bytes[3] << 8 | bytes[2];
	block->center_x = bytes[5] << 8 | bytes[4];
	block->center_y = bytes[7] << 8 | bytes[6];
	block->width = bytes[9] << 8 | bytes[8];
	block->height = bytes[11] << 8 | bytes[10];

	if (block->signature + block->center_x + block->center_y + block->width + block->height != checksum)
		return -EIO;

	 block->timestamp = hrt_absolute_time();
	 return status;
}

void IRLOCK::start()
{
	// flush ring and reset state machine
	_reports->flush();

	// start work queue cycle
	work_queue(HPWORK, &_work, (worker_t)&IRLOCK::cycle_trampoline, this, 1);
}

void IRLOCK::stop()
{
	warnx("stopping queue work");
	work_cancel(HPWORK, &_work);
}

void IRLOCK::cycle_trampoline(void *arg)
{
	IRLOCK *device = (IRLOCK*)arg;
	device->cycle();
}

void IRLOCK::cycle()
{
	// ignoring failure, if we do, we will be back again right away...
	read_device();

	// schedule the next cycle
	work_queue(HPWORK, &_work, (worker_t)&IRLOCK::cycle_trampoline, this, USEC2TICK(IRLOCK_CONVERSION_INTERVAL));
}

void IRLOCK::print_diagnostics()
{
	if (!_sensor_ok)
		warnx("sensor is not ok");

	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue: ");
}

namespace irlock
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;

IRLOCK *g_dev = nullptr;

void start();
void stop();
void test();
void reset();
void info();
void usage();

void start()
{
	int fd;

	if (g_dev != nullptr) {
		warnx("irlock device driver has already been started");
		exit(OK);
	}

	g_dev = new IRLOCK(IRLOCK_BUS, I2C_IRLOCK_ADDRESS);
	if (g_dev == nullptr) {
		warnx("failed to instantiate device reference");
		goto fail;
	}

	if (g_dev->init() != OK) {
		warnx("failed to initialize device");
		goto fail;
	}

	fd = open(IRLOCK_DEVICE_PATH, O_RDONLY);
	if (fd < 0) {
		warnx("could not open device file");
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MAX) < 0) {
		warnx("set poll rate on device failed");
		goto fail;
	}

	exit(OK);

fail:
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	exit(ERROR);
}

void stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	} else {
		warnx("irlock device driver was not running");
	}

	exit(OK);
}

void reset()
{
	int fd = open(IRLOCK_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		errx(ERROR, "driver access failed ");

	if (ioctl(fd, SENSORIOCRESET, 0) < 0)
		errx(ERROR, "driver reset failed");

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		errx(ERROR, "driver poll restart failed");

	exit(OK);
}

void test()
{
	errx(1, "test - not implmented");
}

void info()
{
	if (g_dev == nullptr) {
		errx(1, "irlock device driver is not running");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_diagnostics();

	exit(0);
}

void usage()
{
	warnx("missing command: try 'start', 'stop', 'reset', 'test', 'info'");
}

} // namespace irlock

int irlock_main(int argc, char *argv[])
{
	const char *command = argv[1];

	/*
	 * Start/load the driver
	 */
	if (!strcmp(command, "start")) {
		irlock::start();
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(command, "stop")) {
		irlock::stop();
	}

	/*
	 * Reset the driver
	 */
	if (!strcmp(command, "reset")) {
		irlock::reset();
	}

	/*
	 * Test the driver/device
	 */
	if (!strcmp(command, "test")) {
		irlock::test();
	}

	/*
	 * Print driver information
	 */
	if (!strcmp(command, "info") || !strcmp(command, "status")) {
		irlock::info();
	}

	irlock::usage();
}
