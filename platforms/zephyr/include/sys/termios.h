/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file termios.h
 *
 * Terminal interface. Zephyr has no terminal devices: tcgetattr(),
 * tcsetattr(), tcflush() and tcdrain() fail with ENOTTY.
 */

#pragma once
#pragma GCC system_header

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t tcflag_t;
typedef uint8_t cc_t;
typedef uint32_t speed_t;

#define NCCS 12

struct termios {
	tcflag_t c_iflag;
	tcflag_t c_oflag;
	tcflag_t c_cflag;
	tcflag_t c_lflag;
	cc_t c_cc[NCCS];
	speed_t c_ispeed;
	speed_t c_ospeed;
};

/* c_cc */
#define VMIN     0
#define VTIME    1

/* c_iflag */
#define IGNBRK   (1 << 0)
#define BRKINT   (1 << 1)
#define IGNPAR   (1 << 2)
#define PARMRK   (1 << 3)
#define INPCK    (1 << 4)
#define ISTRIP   (1 << 5)
#define INLCR    (1 << 6)
#define IGNCR    (1 << 7)
#define ICRNL    (1 << 8)
#define IXON     (1 << 9)
#define IXOFF    (1 << 10)

/* c_oflag */
#define OPOST    (1 << 0)
#define ONLCR    (1 << 1)

/* c_cflag */
#define CSIZE    (3 << 0)
#define CS5      (0 << 0)
#define CS6      (1 << 0)
#define CS7      (2 << 0)
#define CS8      (3 << 0)
#define CSTOPB   (1 << 2)
#define CREAD    (1 << 3)
#define PARENB   (1 << 4)
#define PARODD   (1 << 5)
#define HUPCL    (1 << 6)
#define CLOCAL   (1 << 7)
#define CRTSCTS  (1 << 8)

/* c_lflag */
#define ISIG     (1 << 0)
#define ICANON   (1 << 1)
#define ECHO     (1 << 2)
#define ECHONL   (1 << 3)
#define IEXTEN   (1 << 4)

/* tcsetattr() */
#define TCSANOW   0
#define TCSADRAIN 1
#define TCSAFLUSH 2

/* tcflush() */
#define TCIFLUSH  0
#define TCOFLUSH  1
#define TCIOFLUSH 2

/* the speeds are the baud rates */
#define B0       0
#define B50      50
#define B75      75
#define B110     110
#define B134     134
#define B150     150
#define B200     200
#define B300     300
#define B600     600
#define B1200    1200
#define B1800    1800
#define B2400    2400
#define B4800    4800
#define B9600    9600
#define B19200   19200
#define B38400   38400
#define B57600   57600
#define B115200  115200
#define B230400  230400

speed_t cfgetispeed(const struct termios *termios_p);
speed_t cfgetospeed(const struct termios *termios_p);
int cfsetispeed(struct termios *termios_p, speed_t speed);
int cfsetospeed(struct termios *termios_p, speed_t speed);
int tcdrain(int fd);
int tcflush(int fd, int queue_selector);
int tcgetattr(int fd, struct termios *termios_p);
int tcsetattr(int fd, int optional_actions, const struct termios *termios_p);

#ifdef __cplusplus
}
#endif
