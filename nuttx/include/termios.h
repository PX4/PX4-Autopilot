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

#define CBAUD     (0x0f)    /* Bits 0-3:  baud */
#define CBAUDEX   (1 << 4)  /* Bit 4: Extended baud */
#define CSIZE     (3 << 5)  /* Bits 5-6: Character size: */
#  define CS5     (0 << 5)  /*   5 bits */
#  define CS6     (1 << 5)  /*   6 bits */
#  define CS7     (2 << 5)  /*   7 bits */
#  define CS8     (3 << 5)  /*   8 bits */
#define CSTOPB    (1 << 7)  /* Bit 7: Send two stop bits, else one */
#define CREAD     (1 << 8)  /* Bit 8: Enable receiver */
#define PARENB    (1 << 9)  /* Bit 9: Parity enable */
#define PARODD    (1 << 10) /* Bit 10: Odd parity, else even */
#define HUPCL     (1 << 11) /* Bit 11: Hang up on last close */
#define CLOCAL    (1 << 12) /* Bit 12: Ignore modem status lines */

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

/* Baud Rate Selection. */

#define B0        (0x00)    /* Hang up */
#define B50       (0x01)    /* 50 baud */
#define B75       (0x02)    /* 75 baud */
#define B110      (0x03)    /* 110 baud */
#define B134      (0x04)    /* 134.5 baud */
#define B150      (0x05)    /* 150 baud */
#define B200      (0x06)    /* 200 baud */
#define B300      (0x07)    /* 300 baud */
#define B600      (0x08)    /* 600 baud */
#define B1200     (0x09)    /* 1,200 baud */
#define B1800     (0x0a)    /* 1,800 baud */
#define B2400     (0x0b)    /* 2,400 baud */
#define B4800     (0x0c)    /* 4,800 baud */
#define B9600     (0x0d)    /* 9,600 baud */
#define B19200    (0x0e)    /* 19,200 baud */
#define B38400    (0x0f)    /* 38,400 baud */

/* "Extended" baud rates above 37K include the CBAUDEX bit */

#define BOTHER    (CBAUDEX | 0x00) /* Use baud values in c_ispeed and c_ospeed */
#define B57600    (CBAUDEX | 0x01) /* 57,600 baud */
#define B115200   (CBAUDEX | 0x02) /* 115,200 baud */
#define B128000   (CBAUDEX | 0x03) /* 128,000 baud */
#define B230400   (CBAUDEX | 0x04) /* 230,400 baud */
#define B256000   (CBAUDEX | 0x05) /* 256,000 baud */
#define B460800   (CBAUDEX | 0x06) /* 460,800 baud */
#define B500000   (CBAUDEX | 0x07) /* 500,000 baud */
#define B576000   (CBAUDEX | 0x08) /* 576,000 baud */
#define B921600   (CBAUDEX | 0x09) /* 921,600 baud */
#define B1000000  (CBAUDEX | 0x0a) /* 1,000,000 baud */
#define B1152000  (CBAUDEX | 0x0b) /* 1,152,000 baud */
#define B1500000  (CBAUDEX | 0x0c) /* 1,500,000 baud */
#define B2000000  (CBAUDEX | 0x0d) /* 2,000,000 baud */
#define B2500000  (CBAUDEX | 0x0e) /* 2,500,000 baud */
#define B3000000  (CBAUDEX | 0x0f) /* 3,000,000 baud */

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

  /* If CBAUD == BOTHER, then these fields hold the input/output BAUD. */

  speed_t   c_ispeed;       /* Input speed (non-POSIX)*/
  speed_t   c_ospeed;       /* Output speed (non-POSIX) */
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

EXTERN speed_t cfgetispeed(FAR const struct termios *);
EXTERN speed_t cfgetospeed(FAR const struct termios *);
EXTERN int     cfsetispeed(FAR struct termios *, speed_t);
EXTERN int     cfsetospeed(FAR struct termios *, speed_t);
EXTERN int     tcdrain(int);
EXTERN int     tcflow(int, int);
EXTERN int     tcflush(int, int);
EXTERN int     tcgetattr(int, FAR struct termios *);
EXTERN pid_t   tcgetsid(int);
EXTERN int     tcsendbreak(int, int);
EXTERN int     tcsetattr(int, int, FAR const struct termios *);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_TERMIOS_H */
