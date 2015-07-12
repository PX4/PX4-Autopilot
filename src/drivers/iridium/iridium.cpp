/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file iridium.cpp
 *
 * Driver for the Iridium modem AT command set
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <nuttx/config.h>

#include <drivers/drv_hrt.h>
#include <drivers/device/device.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <ctype.h>
#include <termios.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>

#include <board_config.h>

#include <uORB/topics/iridium_message.h>

#define IRIDIUM_DESTINATION_EMAIL		"lorenz@px4.io"
#define IRIDIUM_UPDATE_INTERVAL_US		(500 * 1000)	///< update every 500 ms
#define IRIDIUM_COMMAND_WAIT_INTERVAL_US	(20 * 1000)	///< wait 20 ms after command
#define IRIDIUM_DEVICE_PATH			"/dev/iridium"
#define IRIDIUM_DEFAULT_SERIAL			"/dev/ttyS2"	///< telem 2
#define IRIDIUM_DEFAULT_BAUDRATE		19200

class Iridium : public device::CDev
{
public:
	Iridium(const char* port, unsigned baud);
	virtual ~Iridium();

	virtual int		init();
	virtual int		info();
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Send a string to the default channel
	 */
	int			send_message(const char* message);

private:

	enum IRIDIUM_STATE {
		IRIDIUM_STATE_INIT = 0,
		IRIDIUM_STATE_SYNC = 1,
	};

	/**
	 * Start periodic updates to the LEDs
	 */
	void			start();

	/**
	 * Stop periodic updates to the LEDs
	 */
	void			stop();

	/**
	 * static function that is called by worker queue
	 */
	static void		cycle_trampoline(void *arg);

	/**
	 * update the colours displayed by the LEDs
	 */
	void			cycle();

	/**
	 * set the port baud rate
	 */
	int			set_baudrate(const int &fd, unsigned baud);

	/* internal variables */
	work_s			_work;		///< work queue for scheduling reads
	hrt_abstime		_start_time;	///< start time of this driver
	enum IRIDIUM_STATE	_proto_state;
	int			_fd;		///< UART file descriptor
	unsigned		_baud;		///< UART baud rate
	char			_message[51];	///< next message to be send
	char			_port[32];		///< port name
};

/* for now, we only support one Iridium */
namespace
{
Iridium *g_iridium = nullptr;
}

void iridium_usage();

extern "C" __EXPORT int iridium_main(int argc, char *argv[]);

/* constructor */
Iridium::Iridium(const char* port, unsigned baud) :
	CDev("iridium", IRIDIUM_DEVICE_PATH),
	_work{},
	_start_time(hrt_absolute_time()),
	_fd(-1),
	_baud(baud),
	_message{}
{
	/* copy port name */
	strncpy(_port, port, sizeof(_port));
}

/* destructor */
Iridium::~Iridium()
{
	/* make sure we are truly inactive */
	stop();
}

int
Iridium::init()
{
	/* start work queue */
	start();

	return OK;
}

int
Iridium::info()
{
	// XXX output modem status

	return OK;
}

void
Iridium::start()
{
	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&Iridium::cycle_trampoline, this, 1);
}

void
Iridium::stop()
{
	work_cancel(HPWORK, &_work);
}

void
Iridium::cycle_trampoline(void *arg)
{
	Iridium *dev = (Iridium *)arg;

	/* check global oreoled and cycle */
	if (g_iridium != nullptr) {
		dev->cycle();
	}
}

int
Iridium::send_message(const char* message)
{
	strncpy(_message, message, sizeof(_message));

	return OK;
}

void
Iridium::cycle()
{
	/* check time since startup */
	//uint64_t now = hrt_absolute_time();

	if (_fd < 0) {
		_fd = ::open(_port, O_RDWR);

		if (_fd < 0) {
			warnx("start failed, giving up.");
			return;
		}

		if (set_baudrate(_fd, _baud)) {
			warnx("baud rate failed, giving up.");
			return;
		}
	}

	unsigned sched_interval = IRIDIUM_UPDATE_INTERVAL_US;

	switch (_proto_state) {
		case IRIDIUM_STATE_INIT:
			sched_interval = IRIDIUM_COMMAND_WAIT_INTERVAL_US;
			break;

		case IRIDIUM_STATE_SYNC:
			sched_interval = IRIDIUM_COMMAND_WAIT_INTERVAL_US;
			break;
	}

	/* schedule a fresh cycle call when the processing is done */
	work_queue(HPWORK, &_work, (worker_t)&Iridium::cycle_trampoline, this,
		   USEC2TICK(sched_interval));
}

int
Iridium::set_baudrate(const int &fd, unsigned baud)
{
	/* process baud rate */
	int speed;

	switch (baud) {

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	default:
		warnx("ERR: baudrate: %d\n", baud);
		return -EINVAL;
	}

	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(fd, &uart_config);

	/* clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;
	/* no parity, one stop bit */
	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		warnx("ERR: %d (cfsetispeed)\n", termios_state);
		return -1;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		warnx("ERR: %d (cfsetospeed)\n", termios_state);
		return -1;
	}

	if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
		warnx("ERR: %d (tcsetattr)\n", termios_state);
		return -1;
	}

	return 0;
}

int
Iridium::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int ret = -ENODEV;

	switch (cmd) {
	default:
		/* see if the parent class can make any use of it */
		ret = CDev::ioctl(filp, cmd, arg);
		break;
	}

	return ret;
}

void
iridium_usage()
{
	warnx("missing command: try 'start', 'test', 'info'");
	warnx("options:");
	warnx("    -d %s", IRIDIUM_DEFAULT_SERIAL);
	warnx("    -b %d", IRIDIUM_DEFAULT_BAUDRATE);
}

int
iridium_main(int argc, char *argv[])
{
	char *port = (char *)IRIDIUM_DEFAULT_SERIAL;
	int baudrate = IRIDIUM_DEFAULT_BAUDRATE;
	int ch;

	/* jump over start/off/etc and look at options first */
	while ((ch = getopt(argc, argv, "d:b:")) != EOF) {
		switch (ch) {
		case 'd':
			port = optarg;
			break;

		case 'b':
			baudrate = (int)strtol(optarg, NULL, 0);
			break;

		default:
			iridium_usage();
			return 0;
		}
	}

	if (optind >= argc) {
		iridium_usage();
		return 1;
	}

	const char *verb = argv[optind];

	/* start driver */
	if (!strcmp(verb, "start")) {
		if (g_iridium != nullptr) {
			warnx("already started");
			return 1;
		}

		/* instantiate driver */
		g_iridium = new Iridium(port, baudrate);

		/* check if object was created */
		if (g_iridium == nullptr) {
			warnx("failed to allocated memory for driver");
			return 1;
		}

		/* check object was created successfully */
		if (g_iridium->init() != OK) {
			delete g_iridium;
			g_iridium = nullptr;
			warnx("failed to start driver");
			return 1;
		}

		return 0;
	}

	/* need the driver past this point */
	if (g_iridium == nullptr) {
		warnx("not started");
		iridium_usage();
		return 1;
	}

	if (!strcmp(verb, "test")) {
		int fd = open(IRIDIUM_DEVICE_PATH, O_RDWR);

		if (fd == -1) {
			warnx("Unable to open " IRIDIUM_DEVICE_PATH);
			return 1;
		}

		close(fd);
		return 0;
	}

	/* display driver status */
	if (!strcmp(verb, "info")) {
		g_iridium->info();
		return 0;
	}

	/* send message */
	if (!strcmp(verb, "message")) {

		if (argc > (optind + 1)) {
			warnx("sending: %s", argv[optind + 1]);
			return g_iridium->send_message(argv[optind + 1]);
		} else {
			warnx("need a message argument (in quotes)");
			return 1;
		}
	}

	if (!strcmp(verb, "stop")) {
		int fd = open(IRIDIUM_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " IRIDIUM_DEVICE_PATH);
		}

		close(fd);

		Iridium *tmp_iridium = g_iridium;
		g_iridium = nullptr;
		delete tmp_iridium;
		return 0;
	}

	iridium_usage();
	return 0;
}
