/****************************************************************************
 * arch/arm/src/lpc43xx/lpc43_serial.c
 *
 *   Copyright (C) 2010-2012 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>

#include <arch/serial.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "os_internal.h"
#include "up_internal.h"

#include "lpc43_internal.h"
#include "lpc43_uart.h"
#include "lpc43_serial.h"

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#if defined(USE_SERIALDRIVER) && defined(HAVE_UART)

/* We cannot allow the DLM/DLL divisor to become to small or will will lose too
 * much accuracy.  This following is a "fudge factor" that represents the minimum
 * value of the divisor that we will permit.
 */

#define UART_MINDL 32

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uint32_t uartbase;  /* Base address of UART registers */
  uint32_t baud;      /* Configured baud */
  uint32_t ier;       /* Saved IER value */
  uint8_t  irq;       /* IRQ associated with this UART */
  uint8_t  parity;    /* 0=none, 1=odd, 2=even */
  uint8_t  bits;      /* Number of bits (7 or 8) */
  uint8_t  cclkdiv;   /* Divisor needed to get PCLK from CCLK */
  bool     stopbits2; /* true: Configure with 2 stop bits instead of 1 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_interrupt(int irq, void *context);
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  up_receive(struct uart_dev_s *dev, uint32_t *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);
static bool up_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Variables
 ****************************************************************************/

struct uart_ops_s g_uart_ops =
{
  .setup          = up_setup,
  .shutdown       = up_shutdown,
  .attach         = up_attach,
  .detach         = up_detach,
  .ioctl          = up_ioctl,
  .receive        = up_receive,
  .rxint          = up_rxint,
  .rxavailable    = up_rxavailable,
  .send           = up_send,
  .txint          = up_txint,
  .txready        = up_txready,
  .txempty        = up_txempty,
};

/* I/O buffers */

#ifdef CONFIG_LPC43_USART0
static char g_uart0rxbuffer[CONFIG_USART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_USART0_TXBUFSIZE];
#endif
#ifdef CONFIG_LPC43_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif
#ifdef CONFIG_LPC43_USART2
static char g_uart2rxbuffer[CONFIG_USART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_USART2_TXBUFSIZE];
#endif
#ifdef CONFIG_LPC43_USART3
static char g_uart3rxbuffer[CONFIG_USART3_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_USART3_TXBUFSIZE];
#endif

/* This describes the state of the LPC43xx uart0 port. */

#ifdef CONFIG_LPC43_USART0
static struct up_dev_s g_uart0priv =
{
  .uartbase       = LPC43_USART0_BASE,
  .baud           = CONFIG_USART0_BAUD,
  .irq            = LPC43_IRQ_USART0,
  .parity         = CONFIG_USART0_PARITY,
  .bits           = CONFIG_USART0_BITS,
  .stopbits2      = CONFIG_USART0_2STOP,
};

static uart_dev_t g_uart0port =
{
  .recv     =
  {
    .size   = CONFIG_USART0_RXBUFSIZE,
    .buffer = g_uart0rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART0_TXBUFSIZE,
    .buffer = g_uart0txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart0priv,
};
#endif

/* This describes the state of the LPC43xx uart1 port. */

#ifdef CONFIG_LPC43_UART1
static struct up_dev_s g_uart1priv =
{
  .uartbase       = LPC43_UART1_BASE,
  .baud           = CONFIG_UART1_BAUD,
  .irq            = LPC43_IRQ_UART1,
  .parity         = CONFIG_UART1_PARITY,
  .bits           = CONFIG_UART1_BITS,
  .stopbits2      = CONFIG_UART1_2STOP,
};

static uart_dev_t g_uart1port =
{
  .recv     =
  {
    .size   = CONFIG_UART1_RXBUFSIZE,
    .buffer = g_uart1rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART1_TXBUFSIZE,
    .buffer = g_uart1txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart1priv,
};
#endif

/* This describes the state of the LPC43xx uart1 port. */

#ifdef CONFIG_LPC43_USART2
static struct up_dev_s g_uart2priv =
{
  .uartbase       = LPC43_USART2_BASE,
  .baud           = CONFIG_USART2_BAUD,
  .irq            = LPC43_IRQ_USART2,
  .parity         = CONFIG_USART2_PARITY,
  .bits           = CONFIG_USART2_BITS,
  .stopbits2      = CONFIG_USART2_2STOP,
};

static uart_dev_t g_uart2port =
{
  .recv     =
  {
    .size   = CONFIG_USART2_RXBUFSIZE,
    .buffer = g_uart2rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART2_TXBUFSIZE,
    .buffer = g_uart2txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart2priv,
};
#endif

/* This describes the state of the LPC43xx uart1 port. */

#ifdef CONFIG_LPC43_USART3
static struct up_dev_s g_uart3priv =
{
  .uartbase       = LPC43_USART3_BASE,
  .baud           = CONFIG_USART3_BAUD,
  .irq            = LPC43_IRQ_USART3,
  .parity         = CONFIG_USART3_PARITY,
  .bits           = CONFIG_USART3_BITS,
  .stopbits2      = CONFIG_USART3_2STOP,
};

static uart_dev_t g_uart3port =
{
  .recv     =
  {
    .size   = CONFIG_USART3_RXBUFSIZE,
    .buffer = g_uart3rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_USART3_TXBUFSIZE,
    .buffer = g_uart3txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart3priv,
};
#endif

/* Which UART with be tty0/console and which tty1? tty2? tty3? */

#ifdef HAVE_CONSOLE
#  if defined(CONFIG_USART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart0port      /* USART0=console */
#    define TTYS0_DEV       g_uart0port      /* USART0=ttyS0 */
#    ifdef CONFIG_LPC43_UART1
#      define TTYS1_DEV     g_uart1port      /* USART0=ttyS0;UART1=ttyS1 */
#      ifdef CONFIG_LPC43_USART2
#        define TTYS2_DEV   g_uart2port      /* USART0=ttyS0;UART1=ttyS1;USART2=ttyS2 */
#        ifdef CONFIG_LPC43_USART3
#          define TTYS3_DEV g_uart3port      /* USART0=ttyS0;UART1=ttyS1;USART2=ttyS2;USART3=ttyS3 */
#        else
#          undef TTYS3_DEV                   /* USART0=ttyS0;UART1=ttyS1;USART2=ttyS;No ttyS3 */
#        endif
#      else
#        ifdef CONFIG_LPC43_USART3
#          define TTYS2_DEV g_uart3port     /* USART0=ttyS0;UART1=ttyS1;USART3=ttys2;No ttyS3 */
#        else
#          undef TTYS2_DEV                  /* USART0=ttyS0;UART1=ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS3_DEV                    /* No ttyS3 */
#      endif
#    else
#      ifdef CONFIG_LPC43_USART2
#        define TTYS1_DEV   g_uart2port    /* USART0=ttyS0;USART2=ttyS1;No ttyS3 */
#        ifdef CONFIG_LPC43_USART3
#          define TTYS2_DEV g_uart3port    /* USART0=ttyS0;USART2=ttyS1;USART3=ttyS2;No ttyS3 */
#        else
#          undef TTYS2_DEV                 /* USART0=ttyS0;USART2=ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS3_DEV                   /* No ttyS3 */
#      else
#        ifdef CONFIG_LPC43_USART3
#          define TTYS1_DEV g_uart3port    /* USART0=ttyS0;USART3=ttyS1;No ttyS2;No ttyS3 */
#        else
#          undef TTYS1_DEV                 /* USART0=ttyS0;No ttyS1;No ttyS2;No ttyS3 */
#        endif
#          undef TTYS2_DEV                  /* No ttyS2 */
#        undef TTYS3_DEV                    /* No ttyS3 */
#      endif
#    endif
#  elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart1port     /* UART1=console */
#    define TTYS0_DEV       g_uart1port     /* UART1=ttyS0 */
#    ifdef CONFIG_LPC43_USART0
#      define TTYS1_DEV     g_uart0port     /* UART1=ttyS0;USART0=ttyS1 */
#      ifdef CONFIG_LPC43_USART2
#        define TTYS2_DEV   g_uart2port     /* UART1=ttyS0;USART0=ttyS1;USART2=ttyS2 */
#        ifdef CONFIG_LPC43_USART3
#          define TTYS3_DEV g_uart3port     /* UART1=ttyS0;USART0=ttyS1;USART2=ttyS2;USART3=ttyS3 */
#        else
#          undef TTYS3_DEV                  /* UART1=ttyS0;USART0=ttyS1;USART2=ttyS;No ttyS3 */
#        endif
#      else
#        ifdef CONFIG_LPC43_USART3
#          define TTYS2_DEV g_uart3port     /* UART1=ttyS0;USART0=ttyS1;USART3=ttys2;No ttyS3 */
#        else
#          undef TTYS2_DEV                  /* UART1=ttyS0;USART0=ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS3_DEV                    /* No ttyS3 */
#      endif
#    else
#      ifdef CONFIG_LPC43_USART2
#        define TTYS1_DEV   g_uart2port     /* UART1=ttyS0;USART2=ttyS1 */
#        ifdef CONFIG_LPC43_USART3
#          define TTYS2_DEV g_uart3port     /* UART1=ttyS0;USART2=ttyS1;USART3=ttyS2;No ttyS3 */
#        else
#          undef TTYS2_DEV                  /* UART1=ttyS0;USART2=ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS3_DEV                    /* No ttyS3 */
#      else
#        ifdef CONFIG_LPC43_USART3
#          define TTYS1_DEV   g_uart3port   /* UART1=ttyS0;USART3=ttyS1;No ttyS2;No ttyS3 */
#        else
#          undef TTYS1_DEV                  /* UART1=ttyS0;No ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS2_DEV                    /* No ttyS2 */
#        undef TTYS3_DEV                    /* No ttyS3 */
#      endif
#    endif
#  elif defined(CONFIG_USART2_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart2port     /* USART2=console */
#    define TTYS0_DEV       g_uart2port     /* USART2=ttyS0 */
#    ifdef CONFIG_LPC43_USART2
#      define TTYS1_DEV     g_uart0port     /* USART2=ttyS0;USART0=ttyS1 */
#      ifdef CONFIG_LPC43_UART1
#        define TTYS2_DEV   g_uart1port     /* USART2=ttyS0;USART0=ttyS1;UART1=ttyS2 */
#        ifdef CONFIG_LPC43_USART3
#          define TTYS3_DEV g_uart3port     /* USART2=ttyS0;USART0=ttyS1;UART1=ttyS2;USART3=ttyS3 */
#        else
#          undef TTYS3_DEV                  /* USART2=ttyS0;USART0=ttyS1;UART1=ttyS;No ttyS3 */
#        endif
#      else
#        ifdef CONFIG_LPC43_USART3
#          define TTYS2_DEV g_uart3port     /* USART2=ttyS0;USART0=ttyS1;USART3=ttys2;No ttyS3 */
#        else
#          undef TTYS2_DEV                  /* USART2=ttyS0;USART0=ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS3_DEV                    /* No ttyS3 */
#      endif
#    else
#      ifdef CONFIG_LPC43_UART1
#        define TTYS1_DEV   g_uart1port    /* USART2=ttyS0;UART1=ttyS1 */
#        ifdef CONFIG_LPC43_USART3
#          define TTYS2_DEV g_uart3port    /* USART2=ttyS0;UART1=ttyS1;USART3=ttyS2 */
#        else
#          undef TTYS2_DEV                 /* USART2=ttyS0;UART1=ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS3_DEV                   /* No ttyS3 */
#      else
#        ifdef CONFIG_LPC43_USART3
#          define TTYS1_DEV g_uart3port    /* USART2=ttyS0;USART3=ttyS1;No ttyS3 */
#        else
#          undef TTYS1_DEV                 /* USART2=ttyS0;No ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS2_DEV                   /* No ttyS2 */
#        undef TTYS3_DEV                   /* No ttyS3 */
#      endif
#    endif
#  elif defined(CONFIG_USART3_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart3port    /* USART3=console */
#    define TTYS0_DEV       g_uart3port    /* USART3=ttyS0 */
#    ifdef CONFIG_LPC43_USART0
#      define TTYS1_DEV     g_uart0port    /* USART3=ttyS0;USART0=ttyS1 */
#      ifdef CONFIG_LPC43_UART1
#        define TTYS2_DEV   g_uart1port    /* USART3=ttyS0;USART0=ttyS1;UART1=ttyS2 */
#        ifdef CONFIG_LPC43_USART2
#          define TTYS3_DEV g_uart2port    /* USART3=ttyS0;USART0=ttyS1;UART1=ttyS2;USART2=ttyS3 */
#        else
#          undef TTYS3_DEV                 /* USART3=ttyS0;USART0=ttyS1;UART1=ttyS;No ttyS3 */
#        endif
#      else
#        ifdef CONFIG_LPC43_USART2
#          define TTYS2_DEV g_uart2port    /* USART3=ttyS0;USART0=ttyS1;USART2=ttys2;No ttyS3 */
#        else
#          undef TTYS2_DEV                 /* USART3=ttyS0;USART0=ttyS1;No ttyS2;No ttyS3 */
#        endif
#          undef TTYS3_DEV                 /* No ttyS3 */
#      endif
#    else
#      ifdef CONFIG_LPC43_UART1
#        define TTYS1_DEV   g_uart1port    /* USART3=ttyS0;UART1=ttyS1 */
#        ifdef CONFIG_LPC43_USART2
#          define TTYS2_DEV g_uart2port    /* USART3=ttyS0;UART1=ttyS1;USART2=ttyS2;No ttyS3 */
#        else
#          undef TTYS2_DEV                 /* USART3=ttyS0;UART1=ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS3_DEV                   /* No ttyS3 */
#      else
#        ifdef CONFIG_LPC43_USART2
#          define TTYS1_DEV   g_uart2port  /* USART3=ttyS0;USART2=ttyS1;No ttyS3;No ttyS3 */
#          undef TTYS3_DEV                 /* USART3=ttyS0;USART2=ttyS1;No ttyS2;No ttyS3 */
#        else
#          undef TTYS1_DEV                 /* USART3=ttyS0;No ttyS1;No ttyS2;No ttyS3 */
#        endif
#        undef TTYS2_DEV                   /* No ttyS2 */
#        undef TTYS3_DEV                   /* No ttyS3 */
#      endif
#    endif
#  endif
#else /* No console */
#  define TTYS0_DEV       g_uart0port      /* USART0=ttyS0 */
#  ifdef CONFIG_LPC43_UART1
#    define TTYS1_DEV     g_uart1port      /* USART0=ttyS0;UART1=ttyS1 */
#    ifdef CONFIG_LPC43_USART2
#      define TTYS2_DEV   g_uart2port      /* USART0=ttyS0;UART1=ttyS1;USART2=ttyS2 */
#      ifdef CONFIG_LPC43_USART3
#        define TTYS3_DEV g_uart3port      /* USART0=ttyS0;UART1=ttyS1;USART2=ttyS2;USART3=ttyS3 */
#      else
#        undef TTYS3_DEV                   /* USART0=ttyS0;UART1=ttyS1;USART2=ttyS;No ttyS3 */
#      endif
#    else
#      ifdef CONFIG_LPC43_USART3
#        define TTYS2_DEV g_uart3port      /* USART0=ttyS0;UART1=ttyS1;USART3=ttys2;No ttyS3 */
#      else
#        undef TTYS2_DEV                   /* USART0=ttyS0;UART1=ttyS1;No ttyS2;No ttyS3 */
#      endif
#      undef TTYS3_DEV                     /* No ttyS3 */
#    endif
#  else
#    ifdef CONFIG_LPC43_USART2
#      define TTYS1_DEV   g_uart2port     /* USART0=ttyS0;USART2=ttyS1;No ttyS3 */
#      ifdef CONFIG_LPC43_USART3
#        define TTYS2_DEV g_uart3port     /* USART0=ttyS0;USART2=ttyS1;USART3=ttyS2;No ttyS3 */
#      else
#        undef TTYS2_DEV                  /* USART0=ttyS0;USART2=ttyS1;No ttyS2;No ttyS3 */
#      endif
#      undef TTYS3_DEV                    /* No ttyS3 */
#    else
#      ifdef CONFIG_LPC43_USART3
#        define TTYS1_DEV g_uart3port     /* USART0=ttyS0;USART3=ttyS1;No ttyS2;No ttyS3 */
#      else
#        undef TTYS1_DEV                  /* USART0=ttyS0;No ttyS1;No ttyS2;No ttyS3 */
#      endif
#        undef TTYS2_DEV                  /* No ttyS2 */
#      undef TTYS3_DEV                    /* No ttyS3 */
#    endif
#  endif
#endif /*HAVE_CONSOLE*/

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

/****************************************************************************
 * Name: up_serialin
 ****************************************************************************/

static inline uint32_t up_serialin(struct up_dev_s *priv, int offset)
{
  return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static inline void up_serialout(struct up_dev_s *priv, int offset, uint32_t value)
{
  putreg32(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_disableuartint
 ****************************************************************************/

static inline void up_disableuartint(struct up_dev_s *priv, uint32_t *ier)
{
  if (ier)
    {
      *ier = priv->ier & UART_IER_ALLIE;
    }

  priv->ier &= ~UART_IER_ALLIE;
  up_serialout(priv, LPC43_UART_IER_OFFSET, priv->ier);
}

/****************************************************************************
 * Name: up_restoreuartint
 ****************************************************************************/

static inline void up_restoreuartint(struct up_dev_s *priv, uint32_t ier)
{
  priv->ier |= ier & UART_IER_ALLIE;
  up_serialout(priv, LPC43_UART_IER_OFFSET, priv->ier);
}

/****************************************************************************
 * Name: up_enablebreaks
 ****************************************************************************/

static inline void up_enablebreaks(struct up_dev_s *priv, bool enable)
{
  uint32_t lcr = up_serialin(priv, LPC43_UART_LCR_OFFSET);
  if (enable)
    {
      lcr |= UART_LCR_BRK;
    }
  else
    {
      lcr &= ~UART_LCR_BRK;
    }
  up_serialout(priv, LPC43_UART_LCR_OFFSET, lcr);
}

/************************************************************************************
 * Name: lpc43_uartcclkdiv
 *
 * Descrption:
 *   Select a CCLK divider to produce the UART PCLK.  The stratey is to select the
 *   smallest divisor that results in an solution within range of the 16-bit
 *   DLM and DLL divisor:
 *
 *     PCLK = CCLK / divisor
 *     BAUD = PCLK / (16 * DL)
 *
 *   Ignoring the fractional divider for now.
 *
 *   NOTE:  This is an inline function.  If a typical optimization level is used and
 *   a constant is provided for the desired frequency, then most of the following
 *   logic will be optimized away.
 *
 ************************************************************************************/

static inline uint32_t lpc43_uartcclkdiv(uint32_t baud)
{
  /* Ignoring the fractional divider, the BAUD is given by:
   *
   *   BAUD = PCLK / (16 * DL), or
   *   DL   = PCLK / BAUD / 16
   *
   * Where:
   *
   *   PCLK = CCLK / divisor.
   *
   * Check divisor == 1.  This works if the upper limit is met	
   *
   *   DL < 0xffff, or
   *   PCLK / BAUD / 16 < 0xffff, or
   *   CCLK / BAUD / 16 < 0xffff, or
   *   CCLK < BAUD * 0xffff * 16
   *   BAUD > CCLK / 0xffff / 16
   *
   * And the lower limit is met (we can't allow DL to get very close to one).
   *
   *   DL >= MinDL
   *   CCLK / BAUD / 16 >= MinDL, or
   *   BAUD <= CCLK / 16 / MinDL
   */

  if (baud < (LPC43_CCLK / 16 / UART_MINDL ))
    {
      return SYSCON_PCLKSEL_CCLK;
    }
   
  /* Check divisor == 2.  This works if:
   *
   *   2 * CCLK / BAUD / 16 < 0xffff, or
   *   BAUD > CCLK / 0xffff / 8
   *
   * And
   *
   *   2 * CCLK / BAUD / 16 >= MinDL, or
   *   BAUD <= CCLK / 8 / MinDL
   */

  else if (baud < (LPC43_CCLK / 8 / UART_MINDL ))
    {
      return SYSCON_PCLKSEL_CCLK2;
    }

  /* Check divisor == 4.  This works if:
   *
   *   4 * CCLK / BAUD / 16 < 0xffff, or
   *   BAUD > CCLK / 0xffff / 4
   *
   * And
   *
   *   4 * CCLK / BAUD / 16 >= MinDL, or
   *   BAUD <= CCLK / 4 / MinDL 
   */

  else if (baud < (LPC43_CCLK / 4 / UART_MINDL ))
    {
      return SYSCON_PCLKSEL_CCLK4;
    }

  /* Check divisor == 8.  This works if:
   *
   *   8 * CCLK / BAUD / 16 < 0xffff, or
   *   BAUD > CCLK / 0xffff / 2
   *
   * And
   *
   *   8 * CCLK / BAUD / 16 >= MinDL, or
   *   BAUD <= CCLK / 2 / MinDL 
   */

  else /* if (baud < (LPC43_CCLK / 2 / UART_MINDL )) */
    {
      return SYSCON_PCLKSEL_CCLK8;
    }
}

/************************************************************************************
 * Name: lpc43_uart0config, uart1config, uart2config, and uart3config
 *
 * Descrption:
 *   Configure the UART.  USART0/2/3 and UART1 peripherals are configured using the following
 *   registers:
 *
 *   1. Power: In the PCONP register, set bits PCUSART0/1/2/3.
 *      On reset, USART0 and UART 1 are enabled (PCUSART0 = 1 and PCUART1 = 1)
 *      and USART2/3 are disabled (PCUART1 = 0 and PCUSART3 = 0).
 *   2. Peripheral clock: In the PCLKSEL0 register, select PCLK_USART0 and
 *      PCLK_UART1; in the PCLKSEL1 register, select PCLK_USART2 and PCLK_USART3.
 *   3. Pins: Select UART pins through the PINSEL registers and pin modes
 *      through the PINMODE registers. UART receive pins should not have
 *      pull-down resistors enabled.
 *
 ************************************************************************************/

#ifdef CONFIG_LPC43_USART0
static inline void lpc43_uart0config(uint32_t clkdiv)
{
  uint32_t   regval;
  irqstate_t flags;

  /* Step 1: Enable power on USART0 */

  flags   = irqsave();
  regval  = getreg32(LPC43_SYSCON_PCONP);
  regval |= SYSCON_PCONP_PCUSART0;
  putreg32(regval, LPC43_SYSCON_PCONP);

  /* Step 2: Enable clocking on UART */

  regval = getreg32(LPC43_SYSCON_PCLKSEL0);
  regval &= ~SYSCON_PCLKSEL0_USART0_MASK;
  regval |= (clkdiv << SYSCON_PCLKSEL0_USART0_SHIFT);
  putreg32(regval, LPC43_SYSCON_PCLKSEL0);

  /* Step 3: Configure I/O pins */

  lpc43_configgpio(GPIO_USART0_TXD);
  lpc43_configgpio(GPIO_USART0_RXD);
  irqrestore(flags);
};
#endif

#ifdef CONFIG_LPC43_UART1
static inline void lpc43_uart1config(uint32_t clkdiv)
{
  uint32_t   regval;
  irqstate_t flags;

  /* Step 1: Enable power on UART1 */

  flags   = irqsave();
  regval  = getreg32(LPC43_SYSCON_PCONP);
  regval |= SYSCON_PCONP_PCUART1;
  putreg32(regval, LPC43_SYSCON_PCONP);

  /* Step 2: Enable clocking on UART */

  regval = getreg32(LPC43_SYSCON_PCLKSEL0);
  regval &= ~SYSCON_PCLKSEL0_UART1_MASK;
  regval |= (clkdiv << SYSCON_PCLKSEL0_UART1_SHIFT);
  putreg32(regval, LPC43_SYSCON_PCLKSEL0);

  /* Step 3: Configure I/O pins */

  lpc43_configgpio(GPIO_UART1_TXD);
  lpc43_configgpio(GPIO_UART1_RXD);
#ifdef CONFIG_UART1_FLOWCONTROL
  lpc43_configgpio(GPIO_UART1_CTS);
  lpc43_configgpio(GPIO_UART1_RTS);
  lpc43_configgpio(GPIO_UART1_DCD);
  lpc43_configgpio(GPIO_UART1_DSR);
  lpc43_configgpio(GPIO_UART1_DTR);
#ifdef CONFIG_UART1_RINGINDICATOR
  lpc43_configgpio(GPIO_UART1_RI);
#endif
#endif
  irqrestore(flags);
};
#endif

#ifdef CONFIG_LPC43_USART2
static inline void lpc43_uart2config(uint32_t clkdiv)
{
  uint32_t   regval;
  irqstate_t flags;

  /* Step 1: Enable power on USART2 */

  flags   = irqsave();
  regval  = getreg32(LPC43_SYSCON_PCONP);
  regval |= SYSCON_PCONP_PCUSART2;
  putreg32(regval, LPC43_SYSCON_PCONP);

  /* Step 2: Enable clocking on UART */

  regval = getreg32(LPC43_SYSCON_PCLKSEL1);
  regval &= ~SYSCON_PCLKSEL1_USART2_MASK;
  regval |= (clkdiv << SYSCON_PCLKSEL1_USART2_SHIFT);
  putreg32(regval, LPC43_SYSCON_PCLKSEL1);

  /* Step 3: Configure I/O pins */

  lpc43_configgpio(GPIO_USART2_TXD);
  lpc43_configgpio(GPIO_USART2_RXD);
  irqrestore(flags);
};
#endif

#ifdef CONFIG_LPC43_USART3
static inline void lpc43_uart3config(uint32_t clkdiv)
{
  uint32_t   regval;
  irqstate_t flags;

  /* Step 1: Enable power on USART3 */

  flags   = irqsave();
  regval  = getreg32(LPC43_SYSCON_PCONP);
  regval |= SYSCON_PCONP_PCUSART3;
  putreg32(regval, LPC43_SYSCON_PCONP);

  /* Step 2: Enable clocking on UART */

  regval = getreg32(LPC43_SYSCON_PCLKSEL1);
  regval &= ~SYSCON_PCLKSEL1_USART3_MASK;
  regval |= (clkdiv << SYSCON_PCLKSEL1_USART3_SHIFT);
  putreg32(regval, LPC43_SYSCON_PCLKSEL1);

  /* Step 3: Configure I/O pins */

  lpc43_configgpio(GPIO_USART3_TXD);
  lpc43_configgpio(GPIO_USART3_RXD);
  irqrestore(flags);
};
#endif

/************************************************************************************
 * Name: lpc43_uartdl
 *
 * Descrption:
 *   Select a divider to produce the BAUD from the UART PCLK.
 *
 *     BAUD = PCLK / (16 * DL), or
 *     DL   = PCLK / BAUD / 16
 *
 *   Ignoring the fractional divider for now.
 *
 ************************************************************************************/

static inline uint32_t lpc43_uartdl(uint32_t baud, uint8_t divcode)
{
  uint32_t num;

  switch (divcode)
    {

    case SYSCON_PCLKSEL_CCLK4:   /* PCLK_peripheral = CCLK/4 */
      num = (LPC43_CCLK / 4);
      break;

    case SYSCON_PCLKSEL_CCLK:    /* PCLK_peripheral = CCLK */
      num = LPC43_CCLK;
      break;

    case SYSCON_PCLKSEL_CCLK2:   /* PCLK_peripheral = CCLK/2 */
      num = (LPC43_CCLK / 2);
      break;

    case SYSCON_PCLKSEL_CCLK8:   /* PCLK_peripheral = CCLK/8 (except CAN1, CAN2, and CAN) */
    default:
      num = (LPC43_CCLK / 8);
      break;
    }
  return num / (baud << 4);
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This method is
 *   called the first time that the serial port is opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_LPC43_UART_CONFIG
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  uint16_t dl;
  uint32_t lcr;

  /* Clear fifos */

  up_serialout(priv, LPC43_UART_FCR_OFFSET, (UART_FCR_RXRST|UART_FCR_TXRST));

  /* Set trigger */

  up_serialout(priv, LPC43_UART_FCR_OFFSET, (UART_FCR_FIFOEN|UART_FCR_RXTRIGGER_8));

  /* Set up the IER */

  priv->ier = up_serialin(priv, LPC43_UART_IER_OFFSET);

  /* Set up the LCR */

  lcr = 0;

  if (priv->bits == 7)
    {
      lcr |= UART_LCR_WLS_7BIT;
    }
  else
    {
      lcr |= UART_LCR_WLS_8BIT;
    }

  if (priv->stopbits2)
    {
      lcr |= UART_LCR_STOP;
    }

  if (priv->parity == 1)
    {
      lcr |= (UART_LCR_PE|UART_LCR_PS_ODD);
    }
  else if (priv->parity == 2)
    {
      lcr |= (UART_LCR_PE|UART_LCR_PS_EVEN);
    }

  /* Enter DLAB=1 */

  up_serialout(priv, LPC43_UART_LCR_OFFSET, (lcr | UART_LCR_DLAB));

  /* Set the BAUD divisor */

  dl = lpc43_uartdl(priv->baud, priv->cclkdiv);
  up_serialout(priv, LPC43_UART_DLM_OFFSET, dl >> 8);
  up_serialout(priv, LPC43_UART_DLL_OFFSET, dl & 0xff);

  /* Clear DLAB */

  up_serialout(priv, LPC43_UART_LCR_OFFSET, lcr);

  /* Configure the FIFOs */

  up_serialout(priv, LPC43_UART_FCR_OFFSET,
               (UART_FCR_RXTRIGGER_8|UART_FCR_TXRST|UART_FCR_RXRST|UART_FCR_FIFOEN));

  /* Enable Auto-RTS and Auto-CS Flow Control in the Modem Control Register */
  
#ifdef CONFIG_UART1_FLOWCONTROL
  if (priv->uartbase == LPC43_UART1_BASE)
    {
      up_serialout(priv, LPC43_UART_MCR_OFFSET, (UART_MCR_RTSEN|UART_MCR_CTSEN));
    }
#endif

#endif
  return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial port is closed
 *
 ****************************************************************************/

static void up_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  up_disableuartint(priv, NULL);
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method is
 *   called when the serial port is opened.  Normally, this is just after the
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() methods are called.
 *
 ****************************************************************************/

static int up_attach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, up_interrupt);
  if (ret == OK)
    {
       /* Enable the interrupt (RX and TX interrupts are still disabled
        * in the UART
        */

       up_enable_irq(priv->irq);
    }
  return ret;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception is
 *   the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: up_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int up_interrupt(int irq, void *context)
{
  struct uart_dev_s *dev = NULL;
  struct up_dev_s   *priv;
  uint32_t           status;
  int                passes;

#ifdef CONFIG_LPC43_USART0
  if (g_uart0priv.irq == irq)
    {
      dev = &g_uart0port;
    }
  else
#endif
#ifdef CONFIG_LPC43_UART1
  if (g_uart1priv.irq == irq)
    {
      dev = &g_uart1port;
    }
  else
#endif
#ifdef CONFIG_LPC43_USART2
  if (g_uart2priv.irq == irq)
    {
      dev = &g_uart2port;
    }
  else
#endif
#ifdef CONFIG_LPC43_USART3
  if (g_uart3priv.irq == irq)
    {
      dev = &g_uart3port;
    }
  else
#endif
    {
      PANIC(OSERR_INTERNAL);
    }
  priv = (struct up_dev_s*)dev->priv;

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  for (passes = 0; passes < 256; passes++)
    {
      /* Get the current UART status and check for loop
       * termination conditions
       */

       status = up_serialin(priv, LPC43_UART_IIR_OFFSET);

      /* The UART_IIR_INTSTATUS bit should be zero if there are pending
       * interrupts
       */

      if ((status & UART_IIR_INTSTATUS) != 0)
        {
          /* Break out of the loop when there is no longer a
           * pending interrupt
           */

          break;
        }

      /* Handle the interrupt by its interrupt ID field */

      switch (status & UART_IIR_INTID_MASK)
        {
          /* Handle incoming, receive bytes (with or without timeout) */

          case UART_IIR_INTID_RDA:
          case UART_IIR_INTID_CTI:
            {
              uart_recvchars(dev);
              break;
            }

          /* Handle outgoing, transmit bytes */

          case UART_IIR_INTID_THRE:
            {
              uart_xmitchars(dev);
              break;
            }

          /* Just clear modem status interrupts (UART1 only) */

          case UART_IIR_INTID_MSI:
            {
              /* Read the modem status register (MSR) to clear */

              status = up_serialin(priv, LPC43_UART_MSR_OFFSET);
              vdbg("MSR: %02x\n", status);
              break;
            }

          /* Just clear any line status interrupts */

          case UART_IIR_INTID_RLS:
            {
              /* Read the line status register (LSR) to clear */

              status = up_serialin(priv, LPC43_UART_LSR_OFFSET);
              vdbg("LSR: %02x\n", status);
              break;
            }

          /* There should be no other values */

          default:
            {
              dbg("Unexpected IIR: %02x\n", status);
              break;
            }
        }
    }
    return OK;
}

/****************************************************************************
 * Name: up_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
  struct up_dev_s   *priv  = (struct up_dev_s*)dev->priv;
  int                ret    = OK;

  switch (cmd)
    {
    case TIOCSERGSTRUCT:
      {
         struct up_dev_s *user = (struct up_dev_s*)arg;
         if (!user)
           {
             *get_errno_ptr() = EINVAL;
             ret = ERROR;
           }
         else
           {
             memcpy(user, dev, sizeof(struct up_dev_s));
           }
       }
       break;

    case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
      {
        irqstate_t flags = irqsave();
        up_enablebreaks(priv, true);
        irqrestore(flags);
      }
      break;

    case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
      {
        irqstate_t flags;
        flags = irqsave();
        up_enablebreaks(priv, false);
        irqrestore(flags);
      }
      break;

    default:
      *get_errno_ptr() = ENOTTY;
      ret = ERROR;
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: up_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, uint32_t *status)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  uint32_t rbr;

  *status = up_serialin(priv, LPC43_UART_LSR_OFFSET);
  rbr     = up_serialin(priv, LPC43_UART_RBR_OFFSET);
  return rbr;
}

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void up_rxint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ier |= UART_IER_RBRIE;
#endif
    }
  else
    {
      priv->ier &= ~UART_IER_RBRIE;
    }
  up_serialout(priv, LPC43_UART_IER_OFFSET, priv->ier);
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

static bool up_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  return ((up_serialin(priv, LPC43_UART_LSR_OFFSET) & UART_LSR_RDR) != 0);
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void up_send(struct uart_dev_s *dev, int ch)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  up_serialout(priv, LPC43_UART_THR_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void up_txint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  irqstate_t flags;

  flags = irqsave();
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ier |= UART_IER_THREIE;
      up_serialout(priv, LPC43_UART_IER_OFFSET, priv->ier);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

      uart_xmitchars(dev);
#endif
    }
  else
    {
      priv->ier &= ~UART_IER_THREIE;
      up_serialout(priv, LPC43_UART_IER_OFFSET, priv->ier);
    }
  irqrestore(flags);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the tranmsit fifo is not full
 *
 ****************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  return ((up_serialin(priv, LPC43_UART_LSR_OFFSET) & UART_LSR_THRE) != 0);
}

/****************************************************************************
 * Name: up_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ****************************************************************************/

static bool up_txempty(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  return ((up_serialin(priv, LPC43_UART_LSR_OFFSET) & UART_LSR_THRE) != 0);
}

/****************************************************************************
 * Public Funtions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before up_serialinit.
 *
 *   NOTE: Configuration of the CONSOLE UART was performed by up_lowsetup()
 *   very early in the boot sequence.
 *
 ****************************************************************************/

void up_earlyserialinit(void)
{
  /* Configure all UARTs (except the CONSOLE UART) and disable interrupts */

#ifdef CONFIG_LPC43_USART0
  g_uart0priv.cclkdiv = lpc43_uartcclkdiv(CONFIG_USART0_BAUD);
#ifndef CONFIG_USART0_SERIAL_CONSOLE
  lpc43_uart0config(g_uart0priv.cclkdiv);
#endif
  up_disableuartint(&g_uart0priv, NULL);
#endif

#ifdef CONFIG_LPC43_UART1
  g_uart1priv.cclkdiv = lpc43_uartcclkdiv(CONFIG_UART1_BAUD);
#ifndef CONFIG_UART1_SERIAL_CONSOLE
  lpc43_uart1config(g_uart1priv.cclkdiv);
#endif
  up_disableuartint(&g_uart1priv, NULL);
#endif

#ifdef CONFIG_LPC43_USART2
  g_uart2priv.cclkdiv = lpc43_uartcclkdiv(CONFIG_USART2_BAUD);
#ifndef CONFIG_USART2_SERIAL_CONSOLE
  lpc43_uart2config(g_uart2priv.cclkdiv);
#endif
  up_disableuartint(&g_uart2priv, NULL);
#endif

#ifdef CONFIG_LPC43_USART3
  g_uart3priv.cclkdiv = lpc43_uartcclkdiv(CONFIG_USART3_BAUD);
#ifndef CONFIG_USART3_SERIAL_CONSOLE
  lpc43_uart3config(g_uart3priv.cclkdiv);
#endif
  up_disableuartint(&g_uart3priv, NULL);
#endif

  /* Configuration whichever one is the console */

#ifdef CONSOLE_DEV
  CONSOLE_DEV.isconsole = true;
  up_setup(&CONSOLE_DEV);
#endif
}

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes that
 *   up_earlyserialinit was called previously.
 *
 ****************************************************************************/

void up_serialinit(void)
{
#ifdef CONSOLE_DEV
  (void)uart_register("/dev/console", &CONSOLE_DEV);
#endif
#ifdef TTYS0_DEV
  (void)uart_register("/dev/ttyS0", &TTYS0_DEV);
#endif
#ifdef TTYS1_DEV
  (void)uart_register("/dev/ttyS1", &TTYS1_DEV);
#endif
#ifdef TTYS2_DEV
  (void)uart_register("/dev/ttyS2", &TTYS2_DEV);
#endif
#ifdef TTYS3_DEV
  (void)uart_register("/dev/ttyS3", &TTYS3_DEV);
#endif
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef HAVE_CONSOLE
  struct up_dev_s *priv = (struct up_dev_s*)CONSOLE_DEV.priv;
  uint32_t ier;
  up_disableuartint(priv, &ier);
#endif

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  up_lowputc(ch);
#ifdef HAVE_CONSOLE
  up_restoreuartint(priv, ier);
#endif

  return ch;
}

#else /* USE_SERIALDRIVER */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef HAVE_UART
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  up_lowputc(ch);
#endif
  return ch;
}

#endif /* USE_SERIALDRIVER */
