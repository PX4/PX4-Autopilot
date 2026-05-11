/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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
 * Windows termios.h shim for PX4 SITL.
 *
 * MinGW-w64 does not ship a termios header. SITL drivers that touch
 * serial UARTs (GPS, RC, telemetry radios, etc.) include <termios.h>
 * unconditionally. We expose the full IEEE Std 1003.1-2017 structure
 * and constants so the code compiles, and route the handful of termios
 * functions to stubs that return -ENOSYS. Real serial I/O on Windows
 * would go through CreateFileA("\\\\.\\COMx", ...) + SetCommState;
 * that is out of scope for the base SITL port but this stub surface
 * leaves room to add it later without touching any driver code.
 */
#pragma once

#include <sys/types.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef unsigned int  tcflag_t;
typedef unsigned int  speed_t;
typedef unsigned char cc_t;

#define NCCS 32

/**
 * @brief POSIX terminal attribute container.
 *
 * Native Windows SITL currently exposes this structure for driver source
 * compatibility only; tc* functions return -1/ENOSYS until a COM-port backed
 * implementation is added.
 */
struct termios {
	tcflag_t c_iflag;
	tcflag_t c_oflag;
	tcflag_t c_cflag;
	tcflag_t c_lflag;
	cc_t     c_line;
	cc_t     c_cc[NCCS];
	speed_t  c_ispeed;
	speed_t  c_ospeed;
};

/* c_iflag - input modes */
#define IGNBRK   0000001
#define BRKINT   0000002
#define IGNPAR   0000004
#define PARMRK   0000010
#define INPCK    0000020
#define ISTRIP   0000040
#define INLCR    0000100
#define IGNCR    0000200
#define ICRNL    0000400
#define IUCLC    0001000
#define IXON     0002000
#define IXANY    0004000
#define IXOFF    0010000
#define IMAXBEL  0020000
#define IUTF8    0040000

/* c_oflag - output modes */
#define OPOST    0000001
#define OLCUC    0000002
#define ONLCR    0000004
#define OCRNL    0000010
#define ONOCR    0000020
#define ONLRET   0000040
#define OFILL    0000100
#define OFDEL    0000200
#define NLDLY    0000400
#define NL0      0000000
#define NL1      0000400
#define CRDLY    0003000
#define CR0      0000000
#define CR1      0001000
#define CR2      0002000
#define CR3      0003000
#define TABDLY   0014000
#define TAB0     0000000
#define TAB1     0004000
#define TAB2     0010000
#define TAB3     0014000
#define BSDLY    0020000
#define BS0      0000000
#define BS1      0020000
#define VTDLY    0040000
#define VT0      0000000
#define VT1      0040000
#define FFDLY    0100000
#define FF0      0000000
#define FF1      0100000

/* c_cflag - control modes */
#define CSIZE    0000060
#define CS5      0000000
#define CS6      0000020
#define CS7      0000040
#define CS8      0000060
#define CSTOPB   0000100
#define CREAD    0000200
#define PARENB   0000400
#define PARODD   0001000
#define HUPCL    0002000
#define CLOCAL   0004000
#define CMSPAR   010000000000
#define CRTSCTS  020000000000

/* c_lflag - local modes */
#define ISIG     0000001
#define ICANON   0000002
#define XCASE    0000004
#define ECHO     0000010
#define ECHOE    0000020
#define ECHOK    0000040
#define ECHONL   0000100
#define NOFLSH   0000200
#define TOSTOP   0000400
#define ECHOCTL  0001000
#define ECHOPRT  0002000
#define ECHOKE   0004000
#define FLUSHO   0010000
#define PENDIN   0040000
#define IEXTEN   0100000
#define EXTPROC  0200000

/* c_cc indices */
#define VINTR     0
#define VQUIT     1
#define VERASE    2
#define VKILL     3
#define VEOF      4
#define VTIME     5
#define VMIN      6
#define VSWTC     7
#define VSTART    8
#define VSTOP     9
#define VSUSP     10
#define VEOL      11
#define VREPRINT  12
#define VDISCARD  13
#define VWERASE   14
#define VLNEXT    15
#define VEOL2     16

/* optional_actions for tcsetattr */
#define TCSANOW    0
#define TCSADRAIN  1
#define TCSAFLUSH  2

/* queue_selector for tcflush */
#define TCIFLUSH   0
#define TCOFLUSH   1
#define TCIOFLUSH  2

/* action for tcflow */
#define TCOOFF     0
#define TCOON      1
#define TCIOFF     2
#define TCION      3

/* Baud rates - numeric values (Linux encodes as bitmask in c_cflag;
 * PX4 never inspects these beyond round-tripping through the struct). */
#define B0        0
#define B50       50
#define B75       75
#define B110      110
#define B134      134
#define B150      150
#define B200      200
#define B300      300
#define B600      600
#define B1200     1200
#define B1800     1800
#define B2400     2400
#define B4800     4800
#define B9600     9600
#define B19200    19200
#define B38400    38400
#define B57600    57600
#define B115200   115200
#define B230400   230400
#define B460800   460800
#define B500000   500000
#define B576000   576000
#define B921600   921600
#define B1000000  1000000
#define B1152000  1152000
#define B1500000  1500000
#define B2000000  2000000
#define B2500000  2500000
#define B3000000  3000000
#define B3500000  3500000
#define B4000000  4000000

/** @name Terminal attribute operations
 *
 * Declared for POSIX source compatibility. The Windows backend stubs these
 * functions today because PX4 SITL does not talk to native serial devices
 * through termios.
 *
 * @{
 */
int     tcgetattr(int fd, struct termios *termios_p);
int     tcsetattr(int fd, int optional_actions, const struct termios *termios_p);
int     tcflush(int fd, int queue_selector);
int     tcdrain(int fd);
int     tcflow(int fd, int action);
int     tcsendbreak(int fd, int duration);
pid_t   tcgetsid(int fd);
int     cfsetispeed(struct termios *termios_p, speed_t speed);
int     cfsetospeed(struct termios *termios_p, speed_t speed);
int     cfsetspeed(struct termios *termios_p, speed_t speed);
speed_t cfgetispeed(const struct termios *termios_p);
speed_t cfgetospeed(const struct termios *termios_p);
void    cfmakeraw(struct termios *termios_p);
/** @} */

#ifdef __cplusplus
}
#endif
