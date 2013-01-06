/****************************************************************************
 * include/termios.h
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_TERMIOS_H
#define __INCLUDE_TERMIOS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdint.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Terminal input modes (c_iflag in the termios structure) */

#define BRKINT    (1 << 0)  /* Bit 0:  Signal interrupt on break */
#define ICRNL     (1 << 1)  /* Bit 1:  Map CR to NL on input */
#define IGNBRK    (1 << 2)  /* Bit 2:  Ignore break condition */
#define IGNCR     (1 << 3)  /* Bit 3:  Ignore CR */
#define IGNPAR    (1 << 4)  /* Bit 4:  Ignore characters with parity errors */
#define INLCR     (1 << 5)  /* Bit 5:  Map NL to CR on input */
#define INPCK     (1 << 6)  /* Bit 6:  Enable input parity check */
#define ISTRIP    (1 << 7)  /* Bit 7:  Strip character */
#define IUCLC     (1 << 8)  /* Bit 8:  Map upper-case to lower-case on input (LEGACY) */
#define IXANY     (1 << 9)  /* Bit 9:  Enable any character to restart output */
#define IXOFF     (1 << 10) /* Bit 10: Enable start/stop input control */
#define IXON      (1 << 11) /* Bit 11: Enable start/stop output control */
#define PARMRK    (1 << 12) /* Bit 12: Mark parity errors */

/* Terminal output modes (c_oflag in the termios structure) */

#define OPOST     (1 << 0)  /* Bit 0:  Post-process output */
#define OLCUC     (1 << 1)  /* Bit 1:  Map lower-case to upper-case on output (LEGACY) */
#define ONLCR     (1 << 2)  /* Bit 2:  Map NL to CR-NL on output */
#define OCRNL     (1 << 3)  /* Bit 3:  Map CR to NL on output */
#define ONOCR     (1 << 4)  /* Bit 4:  No CR output at column 0 */
#define ONLRET    (1 << 5)  /* Bit 5:  NL performs CR function */
#define OFILL     (1 << 6)  /* Bit 6:  Use fill characters for delay */
#define NLDLY     (1 << 7)  /* Bit 7:  Select newline delays: */
#  define NL0     (0 << 7)  /*   Newline character type 0 */
#  define NL1     (1 << 7)  /*   Newline character type 1 */
#define CRDLY     (3 << 8)  /* Bits 8-9:  Select carriage-return delays: */
#  define CR0     (0 << 8)  /*   Carriage-return delay type 0 */
#  define CR1     (1 << 8)  /*   Carriage-return delay type 1 */
#  define CR2     (2 << 8)  /*   Carriage-return delay type 2 */
#  define CR3     (3 << 8)  /*   Carriage-return delay type 3 */
#define TABDLY    (3 << 10) /* Bit 10-11:  Select horizontal-tab delays: */
#  define TAB0    (0 << 10) /*   Horizontal-tab delay type 0 */
#  define TAB1    (1 << 10) /*   Horizontal-tab delay type 1 */
#  define TAB2    (2 << 10) /*   Horizontal-tab delay type 2 */
#  define TAB3    (3 << 10) /*   Expand tabs to spaces */
#define BSDLY     (1 << 12) /* Bit 12:  Select backspace delays: */
#  define BS0     (0 << 12) /*   Backspace-delay type 0 */
#  define BS1     (1 << 12) /*   Backspace-delay type 1 */
#define VTDLY     (1 << 13) /* Bit 13:  Select vertical-tab delays: */
#  define VT0     (0 << 13) /*   Vertical-tab delay type 0 */
#  define VT1     (1 << 13) /*   Vertical-tab delay type 1 */
#define FFDLY     (1 << 14) /* Bit 14:  Select form-feed delays: */
#  define FF0     (0 << 14) /*   Form-feed delay type 0 */
#  define FF1     (1 << 14) /*   Form-feed delay type 1 */

/* Control Modes (c_cflag in the termios structure) */

#define CSIZE     (3 << 0)  /* Bits 0-1: Character size: */
#  define CS5     (0 << 0)  /*   5 bits */
#  define CS6     (1 << 0)  /*   6 bits */
#  define CS7     (2 << 0)  /*   7 bits */
#  define CS8     (3 << 0)  /*   8 bits */
#define CSTOPB    (1 << 2)  /* Bit 2: Send two stop bits, else one */
#define CREAD     (1 << 3)  /* Bit 3: Enable receiver */
#define PARENB    (1 << 4)  /* Bit 4: Parity enable */
#define PARODD    (1 << 5)  /* Bit 5: Odd parity, else even */
#define HUPCL     (1 << 6)  /* Bit 6: Hang up on last close */
#define CLOCAL    (1 << 7)  /* Bit 7: Ignore modem status lines */
#define CCTS_OFLOW (1 << 8) /* Bit 8: CTS flow control of output */
#define CRTSCTS   CCTS_OFLOW
#define CRTS_IFLOW (1 << 9) /* Bit 9: RTS flow control of input */

/* Local Modes (c_lflag in the termios structure) */

#define ECHO      (1 << 0)  /* Bit 0:  Enable echo */
#define ECHOE     (1 << 1)  /* Bit 1:  Echo erase character as error-correcting backspace */
#define ECHOK     (1 << 2)  /* Bit 2:  Echo KILL */
#define ECHONL    (1 << 3)  /* Bit 3:  Echo NL */
#define ICANON    (1 << 4)  /* Bit 4:  Canonical input (erase and kill processing) */
#define IEXTEN    (1 << 5)  /* Bit 5:  Enable extended input character processing */
#define ISIG      (1 << 6)  /* Bit 6:  Enable signals */
#define NOFLSH    (1 << 7)  /* Bit 7:  Disable flush after interrupt or quit */
#define TOSTOP    (1 << 8)  /* Bit 8:  Send SIGTTOU for background output */
#define XCASE     (1 << 9)  /* Bit 9:  Canonical upper/lower presentation (LEGACY) */

/* The following are subscript names for the termios c_cc array */

#define VEOF      0         /* Bit 0:  EOF character (canonical mode) */
#define VMIN      VEOF      /* Bit 0:  MIN value (Non-canonical mode) */
#define VEOL      1         /* Bit 1:  EOL character (canonical mode) */
#define VTIME     VEOL      /* Bit 1:  TIME value (Non-canonical mode) */
#define VERASE    2         /* Bit 2:  ERASE character (canonical mode) */
#define VINTR     3         /* Bit 3:  INTR character */
#define VKILL     4         /* Bit 4:  KILL character (canonical mode) */
#define VQUIT     5         /* Bit 5:  QUIT character */
#define VSTART    6         /* Bit 6:  START character */
#define VSTOP     7         /* Bit 7:  STOP character */
#define VSUSP     8         /* Bit 8:  SUSP character */
#define NCCS      9         /* Bit 9:  Size of the array c_cc for control characters */

/* Baud Rate Selection.  These are instances of type speed_t.  Values of 38400
 * and below are specified by POSIX; values above 38400 are sometimes referred
 * to as extended values and most values appear in most termios.h implementations.
 *
 * NOTE that is NuttX that the encoding of the speed_t values is simply the
 * value of the baud itself.  So this opens a window for non-portable abuse
 * of the speed-related interfaces:  The defined values should be used where-
 * ever possible for reasons of portability.
 */

#define B0        0         /* Hang up */
#define B50       50        /* 50 baud */
#define B75       75        /* 75 baud */
#define B110      110       /* 110 baud */
#define B134      134       /* 134.5 baud */
#define B150      150       /* 150 baud */
#define B200      200       /* 200 baud */
#define B300      300       /* 300 baud */
#define B600      600       /* 600 baud */
#define B1200     1200      /* 1,200 baud */
#define B1800     1800      /* 1,800 baud */
#define B2400     2400      /* 2,400 baud */
#define B4800     4800      /* 4,800 baud */
#define B9600     9600      /* 9,600 baud */
#define B19200    19200     /* 19,200 baud */
#define B38400    38400     /* 38,400 baud */

#define B57600    57600     /* 57,600 baud */
#define B115200   115200    /* 115,200 baud */
#define B128000   128000    /* 128,000 baud */
#define B230400   230400    /* 230,400 baud */
#define B256000   256000    /* 256,000 baud */
#define B460800   460800    /* 460,800 baud */
#define B500000   500000    /* 500,000 baud */
#define B576000   576000    /* 576,000 baud */
#define B921600   921600    /* 921,600 baud */
#define B1000000  1000000   /* 1,000,000 baud */
#define B1152000  1152000   /* 1,152,000 baud */
#define B1500000  1500000   /* 1,500,000 baud */
#define B2000000  2000000   /* 2,000,000 baud */
#define B2500000  2500000   /* 2,500,000 baud */
#define B3000000  3000000   /* 3,000,000 baud */

/* Attribute Selection (used with tcsetattr()) */

#define TCSANOW   0         /* Change attributes immediately */
#define TCSADRAIN 1         /* Change attributes when output has drained */
#define TCSAFLUSH 2         /* Change attributes when output has drained; also flush pending input */

/* Line Control (used with tcflush()) */

#define TCIFLUSH  0         /* Flush pending input. Flush untransmitted output */
#define TCIOFLUSH 1         /* Flush both pending input and untransmitted output */
#define TCOFLUSH  2         /* Flush untransmitted output */

/* Constants for use with tcflow() */

#define TCIOFF    0         /* Transmit a STOP character, intended to suspend input data */
#define TCION     1         /* Transmit a START character, intended to restart input data */
#define TCOOFF    2         /* Suspend output */
#define TCOON     3         /* Restart output */

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Baud rate selection */

typedef uint32_t speed_t;   /* Used for terminal baud rates */

/* Types used within the termios structure */

typedef uint16_t tcflag_t;  /* Used for terminal modes */
typedef int      cc_t;      /* Used for terminal special characters */

/* The termios structure */

struct termios
{
  /* Exposed fields defined by POSIX */

  tcflag_t  c_iflag;        /* Input modes */
  tcflag_t  c_oflag;        /* Output modes */
  tcflag_t  c_cflag;        /* Control modes */
  tcflag_t  c_lflag;        /* Local modes */
  cc_t      c_cc[NCCS];     /* Control chars */

  /* Implementation specific fields.  For portability reasons, these fields
   * should not be accessed directly, but rather through only through the
   * cf[set|get][o|i]speed() POSIX interfaces.
   */

  speed_t c_speed;          /* Input/output speed (non-POSIX)*/
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* The cfgetspeed() function is a non-POSIX function will extract the baud
 * from the termios structure to which the termiosp argument points. NuttX
 * does not control input/output baud independently.  Both must be the same. 
 * The POSIX standard interfaces, cfigetispeed() and cfigetospeed() are
 * supported by simply defining them to be cfgetspeed().
 */

EXTERN speed_t cfgetspeed(FAR const struct termios *termiosp);
#define cfgetispeed(termiosp) cfgetspeed(termiosp)
#define cfgetospeed(termiosp) cfgetspeed(termiosp)

/* The cfsetspeed() function is a non-POSIX function that sets the baud
 * stored in the structure pointed to by termiosp to speed. NuttX does
 * not control input/output baud independently.  Both must be the same. 
 * The POSIX standard interfaces, cfigetispeed() and cfigetospeed() are
 * supported by simply defining them to be cfsetspeed().
 */

EXTERN int cfsetspeed(FAR struct termios *termiosp, speed_t speed);
#define cfsetispeed(termiosp,speed) cfsetspeed(termiosp,speed)
#define cfsetospeed(termiosp,speed) cfsetspeed(termiosp,speed)

/* Wait for transmission of output */

EXTERN int tcdrain(int fd);

/* Suspend or restart the transmission or reception of data */

EXTERN int tcflow(int fd, int action);

/* Flush non-transmitted output data, non-read input data or both */

EXTERN int tcflush(int fd, int cmd);

/* Get the parameters associated with the terminal */

EXTERN int tcgetattr(int fd, FAR struct termios *termiosp);

/* Get process group ID for session leader for controlling terminal */

EXTERN pid_t tcgetsid(int fd);

/* Send a "break" for a specific duration */

EXTERN int tcsendbreak(int fd, int duration);

/* Set the parameters associated with the terminal */

EXTERN int tcsetattr(int fd, int options, FAR const struct termios *termiosp);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_TERMIOS_H */
