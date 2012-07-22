/****************************************************************************
 * include/sys/str_tty.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

#ifndef __INCLUDE_SYS_STR_TTY_H
#define __INCLUDE_SYS_STR_TTY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* These are non-standard AIX-like interfaces to simplify TTY baud rate
 * operations.
 *
 * The baud rate functions set_speed() and get_speed() are provided to
 * allow the applications to program any value of the baud rate that is
 * supported by the serial driver, but that cannot be expressed using
 * the termios subroutines cfsetospeed, cfsetispeed, cfgetospeed,
 * and cfsgetispeed. Those subroutines are limited to the set of values
 * defined termios.h.
 *
 * Normal mode: This is the default mode, in which a termios supported
 * speed is in use.
 *
 * Speed-extended mode: This mode is entered by calling set_speed() with
 * a non-termios supported speed at the configuration of the line. In this
 * mode, all the calls to tcgetattr subroutine or TCGETS ioctl subroutine
 * will have B50 in the returned termios structure.
 *
 * If tcsetattr() or TCSETS, TCSETAF, or TCSETAW ioctl calls the driver
 * and attempts to set B50, the actual baud rate is not changed. If it
 * attempts to set any other termios-supported speed, will switch back
 * to the normal mode and the requested baud rate is set.  Calling
 * reset_speed subroutine is another way to switch back to the
 * normal mode.
 */

EXTERN int32_t get_speed(int fd);
EXTERN int set_speed(int fd, int32_t speed);
EXTERN int reset_speed(int fd);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_SYS_STR_TTY_H */
