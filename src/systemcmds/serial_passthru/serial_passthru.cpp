/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 * @file serial_passthru.cpp
 */

#include <px4_platform_common/px4_config.h>
#include <termios.h>


#include <px4_platform_common/atomic.h>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>

static constexpr int TASK_STACK_SIZE   = PX4_STACK_ADJUSTED(1224);
static constexpr int THREAD_STACK_SIZE = PX4_STACK_ADJUSTED(1224);

class SERIALPASSTHRU : public ModuleBase<SERIALPASSTHRU>
{
public:

	SERIALPASSTHRU(const char *external_path, const char *internal_path, unsigned baudrate, bool trackbaud);
	~SERIALPASSTHRU() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static SERIALPASSTHRU *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]) { return print_usage("unknown command"); }

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;


	void thread_run();

	void thread_start();
	void thread_stop();
	static void *trampoline(void *context);


private:
	int        _fd_ext{-1};       ///< the connection to the outside device
	int        _fd_int{-1};       ///< the connection to the inside device
	unsigned   _baudrate{0};      ///< baudrate passed
	char       _ext_path[20] {}; ///< external device / serial port path
	char       _int_path[20] {}; ///< internal device / serial port path
	bool       _track_baud{false}; ///< track baudrate from external and dynmaicaly reconfigure internal

#if defined(DEBUG_BUILD)
	enum dbg_t {
		NONE = 0,
		INT   = 1,
		EXT  = 2,
		BAUD = 4,
	};

	enum dbg_t _debug_level {BAUD};

#endif

	pthread_t _thread{0};      ///< worker task id
	px4::atomic_bool _thread_should_exit{false};

	int setBaudrate(int fd, unsigned baud);
	void dump(const char *dirin, const char *dirout, int read, int written,
		  char *buffer);
};



void SERIALPASSTHRU::dump(const char *dirin, const char *dirout, int read, int written,
			  char *buffer)
{
#if defined(DEBUG_BUILD)
	enum dbg_t mgtype =  dirin[0] == 'i' ? INT : EXT;

	if ((_debug_level & mgtype) && read > 0) {
		fprintf(stderr, "%s %d bytes read\n", dirin, read);
		fprintf(stderr, "%s %d bytes written\n", dirout, written);

		for (int i = 0; i < read; i++) {
			fprintf(stderr, "|%X", buffer[i]);
		}

		fprintf(stderr, "\n");
	}

#endif
}

void SERIALPASSTHRU::thread_run()
{
	px4_prctl(PR_SET_NAME, "serial_passthru-ext->int", px4_getpid());

	struct termios uart_config;
	struct termios last_config = {};
	tcgetattr(_fd_ext, &uart_config);

	do {

		if (_track_baud) {
			// Get the current extenral settings
			tcgetattr(_fd_ext, &uart_config);

			speed_t baudrate = cfgetspeed(&uart_config);
			speed_t last_baudrate = cfgetspeed(&last_config);
			// Has setting baud rate

			if (baudrate != last_baudrate) {
				struct termios int_config;
				// Get tht internal settings
				tcgetattr(_fd_int, &int_config);
				// configure the new baudrate
				cfsetspeed(&int_config, baudrate);

				// set the new baudrate
				tcsetattr(_fd_int, TCSANOW, &int_config);
				last_config = uart_config;
				tcflush(_fd_int, TCIOFLUSH);

#if defined(DEBUG_BUILD)

				if (_debug_level & BAUD) {
					fprintf(stderr, "Baudrate change was:%d is:%d\n", last_baudrate, baudrate);
				}

#endif
			}
		}

		pollfd fds[1];
		fds[0].fd = _fd_ext;
		fds[0].events = POLLIN;

		int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), 10);

		if (ret > 0) {
			if (fds[0].revents & POLLIN) {

				char buf[80];
				int nread = read(_fd_ext, &buf, sizeof(buf));

				if (nread > 0) {
					int nwrite = write(_fd_int, &buf, nread);
					dump("ext", "int", nread, nwrite, buf);
				}
			}
		}

	} while (!_thread_should_exit.load());
}

void SERIALPASSTHRU::thread_start()
{
	pthread_attr_t loop_attr;
	pthread_attr_init(&loop_attr);

	struct sched_param param;
	(void)pthread_attr_getschedparam(&loop_attr, &param);
	param.sched_priority = SCHED_PRIORITY_SLOW_DRIVER - 1;
	(void)pthread_attr_setschedparam(&loop_attr, &param);

	pthread_attr_setstacksize(&loop_attr, THREAD_STACK_SIZE);
	pthread_create(&_thread, &loop_attr, SERIALPASSTHRU::trampoline, (void *)this);
	pthread_attr_destroy(&loop_attr);
}

void *SERIALPASSTHRU::trampoline(void *context)
{
	SERIALPASSTHRU *self = reinterpret_cast<SERIALPASSTHRU *>(context);
	self->thread_run();
	return nullptr;
}

void SERIALPASSTHRU::thread_stop()
{
	_thread_should_exit.store(true);
	pthread_join(_thread, nullptr);
}

SERIALPASSTHRU::SERIALPASSTHRU(const char *path1, const char *path2, unsigned baudrate, bool trackbaud) :
	_baudrate(baudrate),
	_track_baud(trackbaud)
{
	strncpy(_ext_path, path1, sizeof(_ext_path) - 1);
	_ext_path[sizeof(_ext_path) - 1] = '\0';

	strncpy(_int_path, path2, sizeof(_int_path) - 1);
	_int_path[sizeof(_int_path) - 1] = '\0';
}

SERIALPASSTHRU::~SERIALPASSTHRU()
{
}


int SERIALPASSTHRU::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION("Pass data from one device to another.\n"
				 "\n"
				 "This can be used to use u-center connected to USB with a GPS on a serial port.\n"
				);

	PRINT_MODULE_USAGE_NAME_SIMPLE("serial_passthru", "command");
	PRINT_MODULE_USAGE_PARAM_STRING('e', nullptr, "<file:dev>", "External device path", false);
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<file:dev>", "Internal device path", false);
	PRINT_MODULE_USAGE_PARAM_INT('b', 115200, 0, 3000000, "Baudrate", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('t', "Track the External devices baudrate on internal device", true);
	return 0;
}

void
SERIALPASSTHRU::run()
{
	px4_prctl(PR_SET_NAME, "serial_passthru-int->ext", px4_getpid());

	while (!should_exit()) {

		if (_fd_ext < 0) {
			/* open the serial port */
			_fd_ext = ::open(_ext_path, O_RDWR | O_NOCTTY);

			if (_fd_ext < 0) {
				PX4_ERR("failed to open %s err: %d", _ext_path, errno);

			} else {
				setBaudrate(_fd_ext, _baudrate);
			}
		}

		if (_fd_int < 0) {
			/* open the serial port */
			_fd_int = ::open(_int_path, O_RDWR | O_NOCTTY);

			if (_fd_ext < 0) {
				PX4_ERR("failed to open %s err: %d", _int_path, errno);

			} else {
				setBaudrate(_fd_int, _baudrate);
			}
		}

		if (_fd_ext < 0 || _fd_int < 0) {
			px4_sleep(1);
			continue;

		} else if (_thread == 0) {
			thread_start();
		}

		char buf[80];
		int nread = read(_fd_int, &buf, sizeof(buf));

		if (nread > 0) {
			int nwrite = write(_fd_ext, &buf, nread);
			dump("int", "ext", nread, nwrite, buf);
		}
	}

	thread_stop();
	close(_fd_ext);
	close(_fd_int);
}

int SERIALPASSTHRU::setBaudrate(int fd, unsigned baud)
{
	/* process baud rate */
	int speed;

	switch (baud) {
	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	case 230400: speed = B230400; break;

#ifndef B460800
#define B460800 460800
#endif

	case 460800: speed = B460800; break;

#ifndef B921600
#define B921600 921600
#endif

	case 921600: speed = B921600; break;

	default:
		PX4_ERR("ERR: unknown baudrate: %d", baud);
		return -EINVAL;
	}

	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(fd, &uart_config);

	/* properly configure the terminal (see also https://en.wikibooks.org/wiki/Serial_Programming/termios ) */

	//
	// Input flags - Turn off input processing
	//
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
				 INLCR | PARMRK | INPCK | ISTRIP | IXON);
	//
	// Output flags - Turn off output processing
	//
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	// config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	//                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
	uart_config.c_oflag = 0;

	//
	// No line processing
	//
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	//
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	/* no parity, one stop bit, disable flow control */
	uart_config.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		PX4_ERR("%d (cfsetispeed)", termios_state);
		return -1;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		PX4_ERR("%d (cfsetospeed)", termios_state);
		return -1;
	}

	if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
		PX4_ERR("%d (tcsetattr)", termios_state);
		return -1;
	}

	return 0;
}

int SERIALPASSTHRU::task_spawn(int argc, char *argv[])
{
	int task_id = px4_task_spawn_cmd("passthru", SCHED_DEFAULT,
					 SCHED_PRIORITY_SLOW_DRIVER, TASK_STACK_SIZE,
					 run_trampoline, (char *const *)argv);

	if (task_id < 0) {
		task_id = -1;
		return -errno;
	}

	_task_id = task_id;
	return 0;
}

SERIALPASSTHRU *SERIALPASSTHRU::instantiate(int argc, char *argv[])
{
	const char *ext_device = nullptr;
	const char *int_device = nullptr;
	int baudrate = 115200;
	bool trackbaud = false;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "b:d:e:t", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {

		case 't':
			trackbaud = true;
			break;

		case 'b':
			if (px4_get_parameter_value(myoptarg, baudrate) != 0) {
				PX4_ERR("baudrate parsing failed");
				return nullptr;
			}

			break;

		case 'e':
			ext_device = myoptarg;
			break;

		case 'd':
			int_device = myoptarg;
			break;

		}
	}

	SERIALPASSTHRU *serial_passthru = nullptr;
	bool lok =  ext_device && (access(ext_device, R_OK | W_OK) == 0);
	bool rok =  int_device && (access(int_device, R_OK | W_OK) == 0);

	if (rok && lok) {
		serial_passthru = new SERIALPASSTHRU(ext_device, int_device, baudrate, trackbaud);

	} else {
		if (!lok) {
			PX4_ERR("Invalid external device (-e) %s", ext_device ? ext_device  : "");
		}

		if (rok) {
			PX4_ERR("Invalid internal device (-r) %s", int_device ? int_device  : "");
		}
	}

	return serial_passthru;
}


extern "C" __EXPORT int serial_passthru_main(int argc, char *argv[]);

int serial_passthru_main(int argc, char *argv[])
{
	return SERIALPASSTHRU::main(argc, argv);
}
