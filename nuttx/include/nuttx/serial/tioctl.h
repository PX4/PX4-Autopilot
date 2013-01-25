/********************************************************************************************
 * include/nuttx/serial/tioctl.h
 *
 *   Copyright (C) 2011-2013 Gregory Nutt. All rights reserved.
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
 ********************************************************************************************/
/* This function should not be included directly.  Rather, it should be included indirectly
 * via include/nuttx/fs/ioctl.h.
 */

#ifndef __INCLUDE_NUTTX_SERIAL_TIOCTL_H
#define __INCLUDE_NUTTX_SERIAL_TIOCTL_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

#include <stdint.h>

/********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/

/* Get and Set Terminal Attributes (see termios.h) */

#define TCGETS          _TIOC(0x0001)  /* Get serial port settings: FAR struct termios* */
#define TCSETS          _TIOC(0x0002)  /* Set serial port settings: FAR const struct termios* */
#define TCSETSW         _TIOC(0x0003)  /* Drain output and set serial port settings: FAR const struct termios* */
#define TCSETSF         _TIOC(0x0004)  /* Drain output, discard intput, and set serial port settings: FAR const struct termios* */
#define TCGETA          _TIOC(0x0005)  /* See TCGETS: FAR struct termio* */
#define TCSETA          _TIOC(0x0006)  /* See TCSETS: FAR const struct termio* */
#define TCSETAW         _TIOC(0x0007)  /* See TCSETSF: FAR const struct termio* */
#define TCSETAF         _TIOC(0x0008)  /* See TCSETSF: FAR const struct termio* */

/* Locking the termios structure */

#define TIOCGLCKTRMIOS  _TIOC(0x0009) /* Get termios lock status: FAR struct termios* */
#define TIOCSLCKTRMIOS  _TIOC(0x000a) /* Set termios lock status: FAR const struct termios* */

/* Get and Set Window Size */

#define TIOCGWINSZ      _TIOC(0x000b) /* Get window size: FAR struct winsize */
#define TIOCSWINSZ      _TIOC(0x000c) /* Set window size: FAR const struct winsize */

/* Send a break */

#define TCSBRK          _TIOC(0x000d)  /* Send a break: int */
#define TCSBRKP         _TIOC(0x000e)  /* Send a POSIX break: int */
#define TIOCSBRK        _TIOC(0x000f)  /* Turn break on: void */
#define TIOCCBRK        _TIOC(0x0010)  /* Turn break off: void */

/* Software flow control */

#define TCXONC          _TIOC(0x0011)  /* Control flow control: int */

/* Buffer count and flushing */

#define TIOCINQ         _TIOC(0x0012)  /* Bytes in input buffer: int */
#define TIOCOUTQ        _TIOC(0x0013)  /* Bytes in output buffer: int */
#define TCFLSH          _TIOC(0x0014)  /* Flush: int */

/* Faking input */

#define TIOCSTI         _TIOC(0x0015)  /* Insert into input: const char */

/* Re-directing console output */

#define TIOCCONS        _TIOC(0x0016)  /* Re-direct console output to device: void */

/* Controlling TTY */

#define TIOCSCTTY       _TIOC(0x0017)  /* Make controlling TTY: int */
#define TIOCNOTTY       _TIOC(0x0018)  /* Give up controllinog TTY: void */

/* Exclusive mode */

#define TIOCEXCL        _TIOC(0x0019)  /* Put TTY in exclusive mode: void */
#define TIOCNXCL        _TIOC(0x001a)  /* Disable exclusive mode: void */

/* Line discipline */

#define TIOCGETD        _TIOC(0x001b)  /* Get line discipline: FAR int */
#define TIOCSETD        _TIOC(0x001c)  /* Set line discipline: FAR const int */

/* Packet mode */

#define TIOCPKT         _TIOC(0x001d)  /* Control packet mode: FAR const int */

#  define TIOCPKT_FLUSHREAD  (1 << 0)  /* The read queue for the terminal is flushed */
#  define TIOCPKT_FLUSHWRITE (1 << 1)  /* The write queue for the terminal is flushed */
#  define TIOCPKT_STOP       (1 << 2)  /* Output to the terminal is stopped */
#  define TIOCPKT_START      (1 << 3)  /* Output to the terminal is restarted */
#  define TIOCPKT_DOSTOP     (1 << 4)  /* t_stopc is '^S' and t_startc is '^Q' */
#  define TIOCPKT_NOSTOP     (1 << 5)  /* The start and stop characters are not '^S/^Q' */

/* Modem control */

#define TIOCMGET        _TIOC(0x001e)  /* Get modem status bits: FAR int */
#define TIOCMSET        _TIOC(0x001f)  /* Set modem status bits: FAR const int */
#define TIOCMBIC        _TIOC(0x0020)  /* Clear modem bits: FAR const int */
#define TIOCMBIS        _TIOC(0x0021)  /* Set modem bits: FAR const int */

#  define TIOCM_LE      (1 << 0)       /* DSR (data set ready/line enable) */
#  define TIOCM_DTR     (1 << 1)       /* DTR (data terminal ready) */
#  define TIOCM_RTS     (1 << 2)       /* RTS (request to send) */
#  define TIOCM_ST      (1 << 3)       /* Secondary TXD (transmit) */
#  define TIOCM_SR      (1 << 4)       /* Secondary RXD (receive) */
#  define TIOCM_CTS     (1 << 5)       /* CTS (clear to send) */
#  define TIOCM_CAR     (1 << 6)       /* DCD (data carrier detect) */
#  define TIOCM_CD      TIOCM_CAR
#  define TIOCM_RNG     (1 << 7)       /* RNG (ring) */
#  define TIOCM_RI      TIOCM_RNG
#  define TIOCM_DSR     (1 << 8)       /* DSR (data set ready) */

/* TTY shutdown */

#define TIOCVHANGUP     _TIOC(0x0022)  /* Shutdown TTY: void */

/* Marking a line as local */

#define TIOCGSOFTCAR    _TIOC(0x0023)  /* Get software carrier flag: FAR int */
#define TIOCSSOFTCAR    _TIOC(0x0024)  /* Set software carrier flag: FAR const int */

/* Get/set serial line info */

#define TIOCGSERIAL     _TIOC(0x0025)  /* Get serial line info: FAR struct serial_struct */
#define TIOCSSERIAL     _TIOC(0x0026)  /* Set serial line info: FAR const struct serial_struct */
#define TIOCSERGETLSR   _TIOC(0x0027)  /* Get line status register: FAR int */

/* Serial events  */

#define TIOCMIWAIT      _TIOC(0x0028)  /* Wait for a change on serial input line(s): void */
#define TIOCGICOUNT     _TIOC(0x0029)  /* Read serial port interrupt count: FAR  struct serial_icounter_struct */

/* RS-485 Support */

#define TIOCSRS485      _TIOC(0x002a)  /* Set RS485 mode, arg: pointer to struct serial_rs485 */
#define TIOCGRS485      _TIOC(0x002b)  /* Get RS485 mode, arg: pointer to struct serial_rs485 */

/* Definitions for flags used in struct serial_rs485 (Linux compatible) */

#  define SER_RS485_ENABLED        (1 << 0) /* Enable/disble RS-485 support */
#  define SER_RS485_RTS_ON_SEND    (1 << 1) /* Logic level for RTS pin when sending */
#  define SER_RS485_RTS_AFTER_SEND (1 << 2) /* Logic level for RTS pin after sent */
#  define SER_RS485_RX_DURING_TX   (1 << 4)

/* Single-wire UART support */

#define TIOCSSINGLEWIRE _TIOC(0x002c)  /* Set single-wire mode */
#define TIOCGSINGLEWIRE _TIOC(0x002d)  /* Get single-wire mode */

#  define SER_SINGLEWIRE_ENABLED   (1 << 0) /* Enable/disable single-wire support */

/* Debugging */

#define TIOCSERGSTRUCT  _TIOC(0x002e) /* Get device TTY structure */

/********************************************************************************************
 * Public Type Definitions
 ********************************************************************************************/

/* Used with TTY ioctls */

struct winsize
{
  uint16_t ws_row;
  uint16_t ws_col;
/* uint16_t ws_xpixel;    unused */
/* uint16_t ws_ypixel;    unused */
};

/* Structure used with TIOCSRS485 and TIOCGRS485 (Linux compatible) */

struct serial_rs485
{
  uint32_t flags;                  /* See SER_RS485_* definitions */
  uint32_t delay_rts_before_send;  /* Delay before send (milliseconds) */
  uint32_t delay_rts_after_send;   /* Delay after send (milliseconds) */
};

/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SERIAL_TIOCTL_H */
