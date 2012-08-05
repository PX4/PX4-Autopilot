/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file px4io.cpp
 * Driver for the PX4IO board.
 *
 * PX4IO is connected via serial (or possibly some other interface at a later
 * point).
 *
 * XXX current design is racy as all hell; need a locking strategy.
 */

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

#include <arch/board/board.h>

#include <drivers/device/device.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_pwm_output.h>

#include <systemlib/perf_counter.h>
#include <systemlib/hx_stream.h>

#include "../protocol.h"
#include "uploader.h"

class PX4IO;


namespace 
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;

PX4IO	*g_dev;

}


class PX4IO_RC : public device::CDev
{
public:
	PX4IO_RC();
	~PX4IO_RC();

	virtual int		init();
	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);

	friend class PX4IO;
protected:
	void			set_channels(unsigned count, const servo_position_t *data);

private:
	int			_publication;
	struct rc_input_values	_input;
};

/* XXX this may conflict with the onboard PPM input */
PX4IO_RC::PX4IO_RC() :
	CDev("px4io_rc", RC_INPUT_DEVICE_PATH),
	_publication(-1)
{
	for (unsigned i = 0; i < RC_INPUT_MAX_CHANNELS; i++) {
		_input.values[i] = 0;
	}
	_input.channel_count = 0;
}

PX4IO_RC::~PX4IO_RC()
{
	if (_publication != -1)
		::close(_publication);
}

int
PX4IO_RC::init()
{
	int ret;

	ret = CDev::init();

	/* advertise ourselves as the RC input controller */
	if (ret == OK) {
		_publication = orb_advertise(ORB_ID(input_rc), &_input);
		if (_publication < 0)
			ret = -errno;
	}

	return ret;
}

ssize_t
PX4IO_RC::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned channels = buflen / sizeof(rc_input_t);
	rc_input_t *pdata = (rc_input_t *)buffer;
	unsigned i;

	if (channels > PX4IO_INPUT_CHANNELS)
		return -EIO;

	lock();
	for (i = 0; i < channels; i++)
		pdata[i] = _input.values[i];
	unlock();

	return i * sizeof(servo_position_t);
}

void
PX4IO_RC::set_channels(unsigned count, const servo_position_t *data)
{

	ASSERT(count <= PX4IO_INPUT_CHANNELS);

	/* convert incoming servo position values into 0-100 range */
	lock();
	for (unsigned i = 0; i < count; i++) {
		rc_input_t chn;

		if (data[i] < 1000) {
			chn = 0;
		} else if (data[i] > 2000) {
			chn = 100;
		} else {
			chn = (data[i] - 1000) / 10;
		}

		_input.values[i] = chn;
	}
	_input.channel_count = count;
	unlock();

	/* publish to anyone that might be listening */
	if (_publication != -1)
		orb_publish(ORB_ID(input_rc), _publication, &_input);

}

class PX4IO : public device::CDev
{
public:
	PX4IO();
	~PX4IO();

	virtual int		init();

	virtual ssize_t		write(struct file *filp, const char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

private:
	int			_fd;
	int			_task;
	PX4IO_RC		*_rc;

	/** command to be sent to IO */
	struct px4io_command	_next_command;

	/** RC channel input from IO */
	servo_position_t	_rc_channel[PX4IO_INPUT_CHANNELS];
	int			_rc_channel_count;

	volatile bool		_armed;
	volatile bool		_task_should_exit;

	bool			_send_needed;

	hx_stream_t		_io_stream;

	static void		task_main_trampoline(int argc, char *argv[]);
	void			task_main();
	void			io_recv();

	static void		rx_callback_trampoline(void *arg, const void *buffer, size_t bytes_received);
	void			rx_callback(const uint8_t *buffer, size_t bytes_received);

	void			io_send();
};

PX4IO::PX4IO() :
	CDev("px4io", "/dev/px4io"),
	_fd(-1),
	_task(-1),
	_rc(new PX4IO_RC),
	_rc_channel_count(0),
	_armed(false),
	_task_should_exit(false),
	_send_needed(false),
	_io_stream(nullptr)
{
	/* set up the command we will use */
	_next_command.f2i_magic = F2I_MAGIC;

	/* we need this potentially before it could be set in px4io_main */
	g_dev = this;

	_debug_enabled = true;
}

PX4IO::~PX4IO()
{
	if (_rc != nullptr)
		delete _rc;
	if (_task != -1) {
		/* task should wake up every 100ms or so at least */
		_task_should_exit = true;

		unsigned i = 0;
		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 10) {
				task_delete(_task);
				break;
			}

		} while (_task != -1);
	}

	g_dev = nullptr;
}

int
PX4IO::init()
{
	int ret;

	ASSERT(_task == -1);

	/* XXX send a who-are-you request */

	/* XXX verify firmware/protocol version */

	/* do regular cdev init */
	ret = CDev::init();
	if (ret != OK)
		return ret;

	/* start the IO interface task */
	_task = task_create("px4io", SCHED_PRIORITY_DEFAULT, 1024, (main_t)&PX4IO::task_main_trampoline, nullptr);
	if (_task < 0) {
		debug("task start failed: %d", errno);
		return -errno;
	}

	return OK;
}

void
PX4IO::task_main_trampoline(int argc, char *argv[])
{
	g_dev->task_main();
}

void
PX4IO::task_main()
{
	ASSERT(_fd == -1);

	log("ready");

	/* open the serial port */
	_fd = ::open("/dev/ttyS2", O_RDWR | O_NONBLOCK);
	if (_fd < 0) {
		debug("failed to open serial port for IO: %d", errno);
		_task = -1;
		_exit(errno);
	}

	/* protocol stream */
	_io_stream = hx_stream_init(_fd, &PX4IO::rx_callback_trampoline, this);

	perf_counter_t pc_tx_bytes = perf_alloc(PC_COUNT, "PX4IO frames transmitted");
	perf_counter_t pc_rx_bytes = perf_alloc(PC_COUNT, "PX4IO frames received");
	perf_counter_t pc_rx_errors = perf_alloc(PC_COUNT, "PX4IO receive errors");
	hx_stream_set_counters(_io_stream, pc_tx_bytes, pc_rx_bytes, pc_rx_errors);

	/* poll descriptor(s) */
	struct pollfd fds[1];
	fds[0].fd = _fd;
	fds[0].events = POLLIN;

	/* loop handling received serial bytes */
	while (!_task_should_exit) {

		/* sleep waiting for data, but no more than 100ms */
		int ret = ::poll(&fds[0], 1, 100);

		/* this would be bad... */
		if (ret < 0) {
			log("poll error %d", errno);
			usleep(1000000);
			continue;
		}

		/* if we timed out waiting, we should send an update */
		if (ret == 0)
			_send_needed = true;

		/* if we have new data from IO, go handle it */
		if ((ret > 0) && (fds[0].revents & POLLIN))
			io_recv();

		/* send an update to IO if required */
		if (_send_needed) {
			_send_needed = false;
			io_send();
		}
	}
	if (_io_stream != nullptr)
		hx_stream_free(_io_stream);
	::close(_fd);

	/* tell the dtor that we are exiting */
	_task = -1;
	_exit(0);
}

void
PX4IO::io_recv()
{
	uint8_t c;

	/* handle bytes from IO */
	while (::read(_fd, &c, 1) == 1)
		hx_stream_rx(_io_stream, c);
}

void
PX4IO::rx_callback_trampoline(void *arg, const void *buffer, size_t bytes_received)
{
	g_dev->rx_callback((const uint8_t *)buffer, bytes_received);
}

void
PX4IO::rx_callback(const uint8_t *buffer, size_t bytes_received)
{
	const struct px4io_report *rep = (const struct px4io_report *)buffer;

	/* sanity-check the received frame size */
	if (bytes_received != sizeof(struct px4io_report))
		return;

	lock();

	/* pass RC input data to the driver */
	if (_rc != nullptr)
		_rc->set_channels(rep->channel_count, &rep->rc_channel[0]);
	_armed = rep->armed;

	/* send an update frame */
	_send_needed = true;

	unlock();

}

void
PX4IO::io_send()
{
	lock();

	/* send packet to IO while we're guaranteed it won't change */
	hx_stream_send(_io_stream, &_next_command, sizeof(_next_command));

	unlock();
}

ssize_t
PX4IO::write(struct file *filp, const char *buffer, size_t len)
{
	unsigned channels = len / sizeof(servo_position_t);
	servo_position_t *pdata = (servo_position_t *)buffer;
	unsigned i;

	if (channels > PX4IO_OUTPUT_CHANNELS)
		return -EIO;

	lock();
	for (i = 0; i < channels; i++)
		_next_command.servo_command[i] = pdata[i];
	unlock();

	return i * sizeof(servo_position_t);
}

int
PX4IO::ioctl(struct file *filep, int cmd, unsigned long arg)
{
	int ret = -ENOTTY;

	lock();

	/* regular ioctl? */
	switch (cmd) {
	case PWM_SERVO_ARM:
		_next_command.arm_ok = true;
		ret = 0;
		break;

	case PWM_SERVO_DISARM:
		_next_command.arm_ok = false;
		ret = 0;
		break;

	default:
		/* channel set? */
		if ((cmd >= PWM_SERVO_SET(0)) && (cmd < PWM_SERVO_SET(PX4IO_OUTPUT_CHANNELS))) {
			/* XXX sanity-check value? */
			_next_command.servo_command[cmd - PWM_SERVO_SET(0)] = arg;
			ret = 0;
			break;
		}

		/* channel get? */
		if ((cmd >= PWM_SERVO_GET(0)) && (cmd < PWM_SERVO_GET(PX4IO_INPUT_CHANNELS))) {
			int channel = cmd - PWM_SERVO_GET(0);

			/* currently no data for this channel */
			if (channel >= _rc_channel_count) {
				ret = -ERANGE;
				break;
			}

			*(servo_position_t *)arg = _rc_channel[channel];
			ret = 0;
			break;
		}

		/* not a recognised value */
		ret = -ENOTTY;
	}
	unlock();

	return ret;
}

extern "C" __EXPORT int px4io_main(int argc, char *argv[]);

int
px4io_main(int argc, char *argv[])
{
	if (!strcmp(argv[1], "start")) {

		if (g_dev != nullptr) {
			fprintf(stderr, "PX4IO: already loaded\n");
			return -EBUSY;
		}

		/* create the driver - it will set g_dev */
		(void)new PX4IO;

		if (g_dev == nullptr) {
			fprintf(stderr, "PX4IO: driver alloc failed\n");
			return -ENOMEM;
		}

		if (OK != g_dev->init()) {
			fprintf(stderr, "PX4IO: driver init failed\n");
			delete g_dev;
			return -EIO;
		}

		return OK;
	}

	if (!strcmp(argv[1], "update")) {
		PX4IO_Uploader *up;
		const char *fn[3];

		/* work out what we're uploading... */
		if (argc > 2) {
			fn[0] = argv[2];
			fn[1] = nullptr;
		} else {
			fn[0] = "/fs/microsd/px4io.bin";
			fn[1] =	"/etc/px4io.bin";
			fn[2] =	nullptr;
		}

		up = new PX4IO_Uploader;
		int ret = up->upload(&fn[0]);
		delete up;

		switch (ret) {
		case OK:
			break;
		case -ENOENT:
			fprintf(stderr, "PX4IO firmware file '%s' not found\n", fn);
			break;
		case -EEXIST:
		case -EIO:
			fprintf(stderr, "error updating PX4IO - check that bootloader mode is enabled\n");
			break;
		case -EINVAL:
			fprintf(stderr, "verify failed - retry the update\n");
			break;
		case -ETIMEDOUT:
			fprintf(stderr, "timed out waiting for bootloader - power-cycle and try again\n");
		default:
			fprintf(stderr, "unexpected error %d\n", ret);
			break;
		}
		return ret;
	}



	printf("need a verb, only support 'start' and 'update'\n");
	return ERROR;
}
