/****************************************************************************
 * include/termios.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_TERMIOS_H
#define __INCLUDE_TERMIOS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Terminal input modes (c_iflag in the termios structure) */

#define IGNBRK    (1 << 0)  /* Bit 0:  Ignore break condition */
#define BRKINT    (1 << 1)  /* Bit 1:  Signal interrupt on break */
#define IGNPAR    (1 << 2)  /* Bit 2:  Ignore characters with parity errors */
#define PARMRK    (1 << 3)  /* Bit 3:  Mark parity errors */
#define INPCK     (1 << 4)  /* Bit 4:  Enable input parity check */
#define ISTRIP    (1 << 5)  /* Bit 5:  Strip character */
#define INLCR     (1 << 6)  /* Bit 6:  Map NL to CR on input */
#define IGNCR     (1 << 7)  /* Bit 7:  Ignore CR */
#define ICRNL     (1 << 8)  /* Bit 8:  Map CR to NL on input */
#define IUCLC     (1 << 9)  /* Bit 9:  Map upper-case to lower-case on input (LEGACY) */
#define IXON      (1 << 10) /* Bit 10: Enable start/stop output control */
#define IXANY     (1 << 11) /* Bit 11: Enable any character to restart output */
#define IXOFF     (1 << 12) /* Bit 12: Enable start/stop input control */

/* Terminal output modes (c_oflag in the termios structure) */

#define OPOST     (1 << 0)  /* Bit 0:  Post-process output */
#define OLCUC     (1 << 1)  /* Bit 1:  Map lower-case to upper-case on output (LEGACY) */
#define ONLCR     (1 << 2)  /* Bit 2:  Map NL to CR-NL on output */
#define OCRNL     (1 << 3)  /* Bit 3:  Map CR to NL on output */
#define ONOCR     (1 << 4)  /* Bit 4:  No CR output at column 0 */
#define ONLRET    (1 << 5)  /* Bit 5:  NL performs CR function */
#define OFILL     (1 << 6)  /* Bit 6:  Use fill characters for delay */
#define NLDLY     (1 << 8)  /* Bit 8:  Select newline delays: */
#  define NL0     (0 << 8)  /*   Newline character type 0 */
#  define NL1     (1 << 8)  /*   Newline character type 1 */
#define CRDLY     (3 << 9)  /* Bits 9-10: Select carriage-return delays: */
#  define CR0     (0 << 9)  /*   Carriage-return delay type 0 */
#  define CR1     (1 << 9)  /*   Carriage-return delay type 1 */
#  define CR2     (2 << 9)  /*   Carriage-return delay type 2 */
#  define CR3     (3 << 9)  /*   Carriage-return delay type 3 */
#define TABDLY    (3 << 11) /* Bits 11-12: Select horizontal-tab delays: */
#  define TAB0    (0 << 11) /*   Horizontal-tab delay type 0 */
#  define TAB1    (1 << 11) /*   Horizontal-tab delay type 1 */
#  define TAB2    (2 << 11) /*   Horizontal-tab delay type 2 */
#  define TAB3    (3 << 11) /*   Expand tabs to spaces */
#define BSDLY     (1 << 13) /* Bit 13: Select backspace delays: */
#  define BS0     (0 << 13) /*   Backspace-delay type 0 */
#  define BS1     (1 << 13) /*   Backspace-delay type 1 */
#define VTDLY     (1 << 14) /* Bit 14: Select vertical-tab delays: */
#  define VT0     (0 << 14) /*   Vertical-tab delay type 0 */
#  define VT1     (1 << 14) /*   Vertical-tab delay type 1 */
#define FFDLY     (1 << 15) /* Bit 15: Select form-feed delays: */
#  define FF0     (0 << 15) /*   Form-feed delay type 0 */
#  define FF1     (1 << 15) /*   Form-feed delay type 1 */

/* Control Modes (c_cflag in the termios structure) */

#define CSIZE     (3 << 4)    /* Bits 4-5: Character size: */
#  define CS5     (0 << 4)    /*   5 bits */
#  define CS6     (1 << 4)    /*   6 bits */
#  define CS7     (2 << 4)    /*   7 bits */
#  define CS8     (3 << 4)    /*   8 bits */
#define CSTOPB    (1 << 6)    /* Bit 6:  Send two stop bits, else one */
#define CREAD     (1 << 7)    /* Bit 7:  Enable receiver */
#define PARENB    (1 << 8)    /* Bit 8: Parity enable */
#define PARODD    (1 << 9)    /* Bit 9: Odd parity, else even */
#define HUPCL     (1 << 10)   /* Bit 10: Hang up on last close */
#define CLOCAL    (1 << 11)   /* Bit 11: Ignore modem status lines */
#define CCTS_OFLOW (1 << 29)  /* Bit 29: CTS flow control of output */
#define CRTS_IFLOW (1u << 31) /* Bit 31: RTS flow control of input */
#define CRTSCTS   (CCTS_OFLOW | CRTS_IFLOW)

/* Local Modes (c_lflag in the termios structure) */

#define ISIG      (1 << 0)  /* Bit 0:  Enable signals */
#define ICANON    (1 << 1)  /* Bit 1:  Canonical input (erase and kill processing) */
#define XCASE     (1 << 2)  /* Bit 2:  Canonical upper/lower presentation (LEGACY) */
#define ECHO      (1 << 3)  /* Bit 3:  Enable echo */
#define ECHOE     (1 << 4)  /* Bit 4:  Echo erase character as error correcting backspace */
#define ECHOK     (1 << 5)  /* Bit 5:  Echo KILL */
#define ECHONL    (1 << 6)  /* Bit 6:  Echo NL */
#define NOFLSH    (1 << 7)  /* Bit 7:  Disable flush after interrupt or quit */
#define TOSTOP    (1 << 8)  /* Bit 8:  Send SIGTTOU for background output */
#define IEXTEN    (1 << 15) /* Bit 15: Enable extended input character processing */

/* The following are subscript names for the termios c_cc array.
 *
 * Common characters:  VINTR, VQUIT, VSTART, VSTOP, VSUSP
 *
 *   VINTR:  Interrupt character   (Default ETX, Control-C)
 *   VQUIT:  Quit character        (Default FS,  Control-\)
 *   VSTART: Start character       (Default DC1, Control-Q)
 *   VSTOP:  Stop character        (Default DC3, Control-S)
 *   VSUSP:  Suspend character     (Default SUB, Control-Z)
 *
 * Canonical mode:     Adds VEOF, VEOL, VERASE, VKILL
 *
 *   VEOL:   End-of-file character (Default SUB, Control-Z)
 *   VEOF:   End-of-line character (Default NUL)
 *   VERASE: Erase character       (Default DEL or BS, Control-H)
 *   VKILL:  Kill character        (Default NAK or BS, Control-U)
 *
 * Non-canonical mode: Adds VMIN, VTIME
 *
 *   VMIN:   Minimum number of characters for non-canonical read
 *   VTIME:  Timeout in deciseconds for non-canonical read
 */

#define VINTR     0         /* Bit 0:  INTR character */
#define VQUIT     1         /* Bit 1:  QUIT character */
#define VERASE    2         /* Bit 2:  ERASE character (canonical mode) */
#define VKILL     3         /* Bit 3:  KILL character (canonical mode) */
#define VEOF      4         /* Bit 4:  EOF character (canonical mode) */
#define VTIME     5         /* Bit 5:  TIME value (non-canonical mode) */
#define VMIN      6         /* Bit 6:  MIN value (non-canonical mode) */
#define VSTART    8         /* Bit 8:  START character */
#define VSTOP     9         /* Bit 9:  STOP character */
#define VSUSP     10        /* Bit 10: SUSP character */
#define VEOL      11        /* Bit 11: EOL character (canonical mode) */
#define NCCS      12        /* Bit 12: Size of the array c_cc for control characters */

/* Baud Rate Selection.  These are instances of type speed_t.  Values of
 * 38400 and below are specified by POSIX; values above 38400 are sometimes
 * referred to as extended values and most values appear in most termios.h
 * implementations.
 */

#define B0        0000000   /* Hang up */
#define B50       0000001   /* 50 baud */
#define B75       0000002   /* 75 baud */
#define B110      0000003   /* 110 baud */
#define B134      0000004   /* 134.5 baud */
#define B150      0000005   /* 150 baud */
#define B200      0000006   /* 200 baud */
#define B300      0000007   /* 300 baud */
#define B600      0000010   /* 600 baud */
#define B1200     0000011   /* 1,200 baud */
#define B1800     0000012   /* 1,800 baud */
#define B2400     0000013   /* 2,400 baud */
#define B4800     0000014   /* 4,800 baud */
#define B9600     0000015   /* 9,600 baud */
#define B19200    0000016   /* 19,200 baud */
#define B38400    0000017   /* 38,400 baud */

#define B57600    0010001   /* 57,600 baud */
#define B115200   0010002   /* 115,200 baud */
#define B230400   0010003   /* 230,400 baud */
#define B460800   0010004   /* 460,800 baud */
#define B500000   0010005   /* 500,000 baud */
#define B576000   0010006   /* 576,000 baud */
#define B921600   0010007   /* 921,600 baud */
#define B1000000  0010010   /* 1,000,000 baud */
#define B1152000  0010011   /* 1,152,000 baud */
#define B1500000  0010012   /* 1,500,000 baud */
#define B2000000  0010013   /* 2,000,000 baud */
#define B2500000  0010014   /* 2,500,000 baud */
#define B3000000  0010015   /* 3,000,000 baud */
#define B3500000  0010016   /* 3,500,000 baud */
#define B4000000  0010017   /* 4,000,000 baud */

/* Attribute Selection (used with tcsetattr()) */

#define TCSANOW   0         /* Change attributes immediately */
#define TCSADRAIN 1         /* Change attributes when output has drained */
#define TCSAFLUSH 2         /* Change attributes when output has drained;
                             * also flush pending input */

/* Line Control (used with tcflush()) */

#define TCIFLUSH  0         /* Flush pending input */
#define TCOFLUSH  1         /* Flush untransmitted output */
#define TCIOFLUSH 2         /* Flush both pending input and untransmitted
                             * output */

/* Constants for use with tcflow() */

#define TCOOFF    0         /* Suspend output */
#define TCOON     1         /* Restart output */
#define TCIOFF    2         /* Transmit a STOP character, intended to
                             * suspend input data */
#define TCION     3         /* Transmit a START character, intended to
                             * restart input data */

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Baud rate selection */

typedef unsigned long speed_t;   /* Used for terminal baud rates */

/* Types used within the termios structure */

typedef unsigned int  tcflag_t;  /* Used for terminal modes */
typedef unsigned char cc_t;      /* Used for terminal special characters */

/* The termios structure */

struct termios {
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

	speed_t c_speed;          /* Input/output speed (non-POSIX) */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* The cfgetspeed() function is a non-POSIX function will extract the baud
 * from the termios structure to which the termiosp argument points. NuttX
 * does not control input/output baud independently.  Both must be the same.
 * The POSIX standard interfaces, cfigetispeed() and cfigetospeed() are
 * supported by simply defining them to be cfgetspeed().
 * The return value is baud value(9600).
 */

speed_t cfgetspeed(const struct termios *termiosp);
#define cfgetispeed(termiosp) cfgetspeed(termiosp)
#define cfgetospeed(termiosp) cfgetspeed(termiosp)

/* The cfsetspeed() function is a non-POSIX function that sets the baud
 * stored in the structure pointed to by termiosp to speed. NuttX does
 * not control input/output baud independently.  Both must be the same.
 * The POSIX standard interfaces, cfigetispeed() and cfigetospeed() are
 * supported by simply defining them to be cfsetspeed().
 * Speed could be baud value(9600) or could be baud mask(B9600).
 */

int cfsetspeed(struct termios *termiosp, speed_t speed);
#define cfsetispeed(termiosp,speed) cfsetspeed(termiosp,speed)
#define cfsetospeed(termiosp,speed) cfsetspeed(termiosp,speed)

/* The cfmakeraw() function is a non-POSIX function that sets the terminal
 * to something like the "raw" mode.
 */

void cfmakeraw(struct termios *termiosp);

/* Wait for transmission of output */

int tcdrain(int fd);

/* Suspend or restart the transmission or reception of data */

int tcflow(int fd, int action);

/* Flush non-transmitted output data, non-read input data or both */

int tcflush(int fd, int cmd);

/* Get the parameters associated with the terminal */

int tcgetattr(int fd, struct termios *termiosp);

/* Get process group ID for session leader for controlling terminal */

pid_t tcgetsid(int fd);

/* Send a "break" for a specific duration */

int tcsendbreak(int fd, int duration);

/* Set the parameters associated with the terminal */

int tcsetattr(int fd, int options, const struct termios *termiosp);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_TERMIOS_H */
