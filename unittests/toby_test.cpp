#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <toby/toby.h>

#include "gtest/gtest.h"

// Not all POSIX systems define B921600, but those
// which do not accept a plain integer with the same value
#ifndef B921600
#define B921600 921600
#endif

TEST(TobyTest, Toby)
{
	const char *uart_name = "/dev/tty.usbserialABC";

	warnx("connecting to port: %s", uart_name);

	int ret = 0;

	int fd = open(uart_name, O_RDWR | O_NOCTTY);

	ASSERT_GT(fd, -1) << "ERR OPENING " << uart_name;

	int default_speed = B57600;
	int target_speed = B921600;

	int speed = default_speed;

	struct termios uart_config;
	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* Set baud rate */
	ret = cfsetispeed(&uart_config, speed);
	ASSERT_GT(ret, -1) << "ERR SET cfsetispeed " << uart_name;

	ret = cfsetospeed(&uart_config, speed);
	ASSERT_GT(ret, -1) << "ERR SET cfsetospeed " << uart_name;

	ret = tcsetattr(fd, TCSANOW, &uart_config);
	ASSERT_EQ(0, ret) << "ERR SET CONF";

	// TODO configure modem and read status
	TobyLTE toby(fd);
	bool connected = toby.init();

	ret = close(fd);

	ASSERT_EQ(0, ret) << "Failed closing port";
}
