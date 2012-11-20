/****************************************************************************
 * arch/sh/src/m16c/m16c_serial.c
 *
 *   Copyright (C) 2009, 2012 Gregory Nutt. All rights reserved.
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

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "os_internal.h"
#include "m16c_uart.h"

/* Is there any serial support?  This might be the case if the board does
 * not have serial ports but supports a console through, say, an LCD.
 */

#if defined(CONFIG_M16C_UART0) || defined(CONFIG_M16C_UART1) || defined(CONFIG_M16C_UART2)

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Configuration **********************************************************/

#ifndef M16C_XIN_PRESCALER
#  define M16C_XIN_PRESCALER 1
#endif

/* Make sure interrupt priorities are defined.  If not, use a default of 5 */

#ifndef M16C_S2T_PRIO
#  define  M16C_S2T_PRIO   5            /* UART2 transmit interrupt priority */
#endif
#ifndef M16C_S2R_PRIO
#  define  M16C_S2R_PRIO   5            /* UART2 receive interrupt priority */
#endif
#ifndef M16C_S0T_PRIO
#  define  M16C_S0T_PRIO   5            /* UART0 transmit interrupt priority */
#endif
#ifndef M16C_S0R_PRIO
#  define  M16C_S0R_PRIO   5            /* UART0 receive interrupt priority */
#endif
#ifndef M16C_S1T_PRIO
#  define  M16C_S1T_PRIO   5            /* UART1 transmit interrupt priority */
#endif
#ifndef M16C_S1R_PRIO
#  define  M16C_S1R_PRIO   5            /* UART1 receive interrupt priority */
#endif

/* Some sanity checks *******************************************************/

/* Are there any UARTs? */

#if !defined(CONFIG_M16C_UART0) && !defined(CONFIG_M16C_UART1) && !defined(CONFIG_M16C_UART2)
#  ifdef USE_SERIALDRIVER
#    error "Serial driver selected, but No UARTs is enabled"
#    undef USE_SERIALDRIVER
#  endif
#endif

/* Is there a serial console? */

#if defined(CONFIG_UART0_SERIAL_CONSOLE) && defined(CONFIG_M16C_UART0)
#  define HAVE_SERIALCONSOLE 1
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#elif defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_M16C_UART1)
#  define HAVE_SERIALCONSOLE 1
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#elif defined(CONFIG_UART2_SERIAL_CONSOLE) && defined(CONFIG_M16C_UART2)
#  define HAVE_SERIALCONSOLE 1
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#else
#  if defined(CONFIG_UART0_SERIAL_CONSOLE) || defined(CONFIG_UART1_SERIAL_CONSOLE)|| defined(CONFIG_UART2_SERIAL_CONSOLE)
#    error "Serial console selected, but corresponding UART not enabled"
#  endif
#  undef HAVE_SERIALCONSOLE
#  undef CONFIG_UART0_SERIAL_CONSOLE
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#endif

#if defined(HAVE_SERIALCONSOLE) && defined(CONFIG_LCD_CONSOLE)
#  error "Both serial and LCD consoles are defined"
#elif !defined(HAVE_SERIALCONSOLE) && !defined(CONFIG_LCD_CONSOLE)
#  warning "No console is defined"
#endif

#ifdef USE_SERIALDRIVER

/* Which UART with be tty0/console and which tty1 and tty2? */

/* CONFIG_UART0_SERIAL_CONSOLE (implies CONFIG_M16C_UART0 also defined) */

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_uart0port     /* UART0 is console */
#  define TTYS0_DEV       g_uart0port     /* UART0 is tty0 */
#  ifdef CONFIG_M16C_UART1
#    define TTYS1_DEV     g_uart1port     /* UART1 is tty1 */
#    ifdef CONFIG_M16C_UART2
#      define TTYS2_DEV   g_uart2port     /* UART2 is tty2 */
#    endif
#  else
#    ifdef CONFIG_M16C_UART2
#      define TTYS2_DEV   g_uart2port     /* UART2 is tty1 */
#    else
#      undef TTYS1_DEV                    /* No tty1 */
#    endif
#    undef TTYS2_DEV                      /* No tty2 */
#  endif

/* CONFIG_UART1_SERIAL_CONSOLE (implies CONFIG_M16C_UART1 also defined) */

#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_uart1port     /* UART1 is console */
#  define TTYS0_DEV       g_uart1port     /* UART1 is tty0 */
#  ifdef CONFIG_M16C_UART0
#    define TTYS1_DEV     g_uart0port     /* UART0 is tty1 */
#    ifdef CONFIG_M16C_UART2
#      define TTYS2_DEV   g_uart2port     /* UART2 is tty2 */
#    endif
#  else
#    ifdef CONFIG_M16C_UART2
#      define TTYS2_DEV   g_uart2port     /* UART2 is tty1 */
#    else
#      undef TTYS1_DEV                    /* No tty1 */
#    endif
#    undef TTYS2_DEV                      /* No tty2 */
#  endif

/* CONFIG_UART2_SERIAL_CONSOLE (implies CONFIG_M16C_UART2 also defined) */

#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_uart2port     /* UART2 is console */
#  define TTYS0_DEV       g_uart2port     /* UART2 is tty0 */
#  ifdef CONFIG_M16C_UART0
#    define TTYS1_DEV     g_uart0port     /* UART0 is tty1 */
#    ifdef CONFIG_M16C_UART1
#      define TTYS2_DEV   g_uart1port     /* UART1 is tty2 */
#    endif
#  else
#    ifdef CONFIG_M16C_UART1
#      define TTYS1_DEV   g_uart1port     /* UART1 is tty1 */
#    else
#      undef TTYS1_DEV                    /* No tty1 */
#    endif
#    undef TTYS2_DEV                      /* No tty2 */
#  endif

/* No console, at least one of CONFIG_M16C_UART0/1/2 defined */

#elif defined(CONFIG_M16C_UART0)
#  undef  CONSOLE_DEV                     /* No console */
#  define TTYS0_DEV       g_uart0port     /* UART0 is tty0 */
#  ifdef CONFIG_M16C_UART1
#    define TTYS1_DEV     g_uart1port     /* UART1 is tty1 */
#    ifdef CONFIG_M16C_UART2
#      define TTYS2_DEV   g_uart2port     /* UART2 is tty2 */
#    endif
#  else
#    ifdef CONFIG_M16C_UART2
#      define TTYS1_DEV   g_uart1port     /* UART2 is tty1 */
#    else
#      undef TTYS1_DEV                    /* No tty1 */
#    endif
#    undef TTYS2_DEV                      /* No tty2 */
#  endif

elif defined(CONFIG_M16C_UART1)
#  undef  CONSOLE_DEV                     /* No console */
#  undef  TTYS2_DEV                       /* No tty2 */
#  define TTYS0_DEV       g_uart1port     /* UART1 is tty0 */
#  ifdef CONFIG_M16C_UART2
#    define TTYS1_DEV     g_uart2port     /* UART2 is tty1 */
#  else
#    undef TTYS1_DEV                      /* No tty1 */
#  endif

/* Otherwise, there is no console and only CONFIG_M16C_UART2 is defined */

#else
#  undef  CONSOLE_DEV                    /* No console */
#  define TTYS0_DEV       g_uart2port    /* UART2 is tty0 */
#  undef TTYS1_DEV                       /* No tty1 */
#  undef TTYS1_DEV                       /* No tty2 */
#endif

/* Definitions for the enable field of the device structure */

#define M16C_RXENABLED     0x01
#define M16C_TXENABLED     0x02

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
          uint32_t baud;      /* Configured baud */
          uint16_t uartbase;  /* Base address of UART registers */
           uint8_t uartno;    /* UART number */
 volatile  uint8_t ucon;      /* Saved SCR value */
 volatile  uint8_t ssr;       /* Saved SR value (only used during interrupt processing) */
           uint8_t rcvirq;    /* UART receive data available IRQ */
           uint8_t xmtirq;    /* UART transmit complete IRQ */
           uint8_t enables;   /* Bit 0: 1=RX enabled, Bit 1: 1=TX enabled */
           uint8_t parity;    /* 0=none, 1=odd, 2=even */
           uint8_t bits;      /* Number of bits (7 or 8) */
              bool stopbits2; /* true: Configure with 2 stop bits instead of 1 */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_rcvinterrupt(int irq, void *context);
static int  up_receive(struct uart_dev_s *dev, unsigned int *status);
static void m16c_rxint(struct up_dev_s *dev, bool enable);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static int  up_xmtinterrupt(int irq, void *context);
static void up_send(struct uart_dev_s *dev, int ch);
static void m16c_txint(struct up_dev_s *dev, bool enable);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);

/****************************************************************************
 * Private Variables
 ****************************************************************************/

struct uart_ops_s g_uart_ops =
{
  .setup          = up_setup,
  .shutdown       = up_shutdown,
  .attach         = up_attach,
  .detach         = up_detach,
  .receive        = up_receive,
  .rxint          = up_rxint,
  .rxavailable    = up_rxavailable,
  .send           = up_send,
  .txint          = up_txint,
  .txready        = up_txready,
  .txempty        = up_txready,
};

/* I/O buffers */

#ifdef CONFIG_M16C_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif
#ifdef CONFIG_M16C_UART1
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];
#endif
#ifdef CONFIG_M16C_UART2
static char g_uart2rxbuffer[CONFIG_UART2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_UART2_TXBUFSIZE];
#endif

/* This describes the state of the M16C UART0 port. */

#ifdef CONFIG_M16C_UART0
static struct up_dev_s g_uart0priv =
{
  .baud           = CONFIG_UART0_BAUD,
  .uartbase       = M16C_UART0_BASE,
  .uartno         = 0,
  .rcvirq         = M16C_UART0RCV_IRQ,
  .xmtirq         = M16C_UART0XMT_IRQ,
  .parity         = CONFIG_UART0_PARITY,
  .bits           = CONFIG_UART0_BITS,
  .stopbits2      = CONFIG_UART0_2STOP,
};

static uart_dev_t g_uart0port =
{
  .recv     =
  {
    .size   = CONFIG_UART0_RXBUFSIZE,
    .buffer = g_uart0rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART0_TXBUFSIZE,
    .buffer = g_uart0txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uart0priv,
};
#endif

/* This describes the state of the M16C UART1 port. */

#ifdef CONFIG_M16C_UART1
static struct up_dev_s g_uart1priv =
{
  .baud           = CONFIG_UART1_BAUD,
  .uartbase       = M16C_UART1_BASE,
  .uartno         = 1,
  .rcvirq         = M16C_UART1RCV_IRQ,
  .xmtirq         = M16C_UART1XMT_IRQ,
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

/* This describes the state of the M16C UART2 port. */

#ifdef CONFIG_M16C_UART2
static struct up_dev_s g_uart2priv =
{
  .baud           = CONFIG_UART2_BAUD,
  .uartbase       = M16C_UART2_BASE,
  .uartno         = 2,
  .rcvirq         = M16C_UART2RCV_IRQ,
  .xmtirq         = M16C_UART2XMT_IRQ,
  .parity         = CONFIG_UART2_PARITY,
  .bits           = CONFIG_UART2_BITS,
  .stopbits2      = CONFIG_UART2_2STOP,
};

static uart_dev_t g_uart2port =
{
  .recv     =
  {
    .size   = CONFIG_UART2_RXBUFSIZE,
    .buffer = g_uart2rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART2_TXBUFSIZE,
    .buffer = g_uart2txbuffer,
   },
  .ops      = &g_uart_ops,
  .priv     = &g_uart2priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialin
 ****************************************************************************/

static inline uint8_t up_serialin(struct up_dev_s *priv, int offset)
{
  return getreg8(priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_serialin16
 ****************************************************************************/

static inline uint16_t up_serialin16(struct up_dev_s *priv, int offset)
{
  return getreg16(priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static inline void up_serialout(struct up_dev_s *priv, int offset, uint8_t value)
{
  putreg8(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_serialout16
 ****************************************************************************/

static inline void up_serialout16(struct up_dev_s *priv, int offset, uint16_t value)
{
  putreg16(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_disableuartint
 ****************************************************************************/

static inline void up_disableuartint(struct up_dev_s *priv, uint8_t *penables)
{
  uint8_t enables = priv->enables;
  m16c_txint(priv, false);
  m16c_rxint(priv, false);

  if (enables)
    {
      *penables = enables;
    }
}

/****************************************************************************
 * Name: up_restoreuartint
 ****************************************************************************/

static inline void up_restoreuartint(struct up_dev_s *priv, uint8_t enables)
{
  m16c_rxint(priv, (enables & M16C_RXENABLED) != 0);
  m16c_txint(priv, (enables & M16C_TXENABLED) != 0);
}

/****************************************************************************
 * Name: up_waittxready
 ****************************************************************************/

#ifdef HAVE_SERIALCONSOLE
static inline void up_waittxready(struct up_dev_s *priv)
{
  int tmp;

  /* Limit how long we will wait for the TDR empty condition */

  for (tmp = 1000 ; tmp > 0 ; tmp--)
    {
      /* Check the TI bit in the CI register.  1=Transmit buffer empty */

      if ((up_serialin(priv, M16C_UART_C1) & UART_C1_TI) != 0);
       {
          /* The transmit buffer is empty... return */
          break;
        }
    }
}
#endif

/****************************************************************************
 * Name: ub_setbrg
 *
 * Description:
 *   Calculate the correct value for the BRG given the configured frequency
 *   and the desired BAUD settings.
 *
 ****************************************************************************/

static inline void ub_setbrg(struct up_dev_s *priv, unsigned int baud)
{
  uint16_t brg;

  /* The Bit Rate Generator (BRG) value can be calculated by:
   *
   * BRG = source-frequency / prescaler / 16 / baud rate - 1
   *
   * Example:
   *  source-frequency = 20,000,000 (20MHz)
   *  prescaler = 1
   *  baud rate = 19200
   *  BRG = 20,000,000/1/16/19200 - 1 = 64
   */

  brg = (M16C_XIN_FREQ / (16L * M16C_XIN_PRESCALER * (uint32_t)baud)) - 1;
  up_serialout(priv, M16C_UART_BRG, brg);
}

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This
 *   method is called the first time that the serial port is
 *   opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  uint8_t regval;

  /* Set the baud rate generator */

  ub_setbrg(priv, priv->baud);

  /* Disable CTS/RTS */

  up_serialout(priv, M16C_UART_C0, UART_C0_CRD);

  /* Disable RX/TX interrupts */

  m16c_rxint(priv, false);
  m16c_txint(priv, false);

/* Set interrupt cause=TX complete and continuous receive mode */

#ifdef CONFIG_M16C_UART0
  if (priv->uartno == 0)
    {
      regval  = getreg8(M16C_UCON);
      regval |= (UART_CON_U0IRS|UART_CON_U0RRM);
      putreg8(regval, M16C_UCON);
    }
  else
#endif
#ifdef CONFIG_M16C_UART1
  if (priv->uartno == 1)
    {
      regval  = getreg8(M16C_UCON);
      regval |= (UART_CON_U1IRS|UART_CON_U1RRM);
      putreg8(regval, M16C_UCON);
    }
  else
#endif
#ifdef CONFIG_M16C_UART2
  if (priv->uartno == 2)
    {
      regval  = getreg8(M16C_U2C1);
      regval |= ((UART_C1_U2IRS|UART_C1_U2RRM);
      putreg8(regval, M16C_U2C1);
    }
  else
#endif
    {
      dbg("Invalid UART #\n");
    }

  /* Set UART transmit/receive control register 1 to enable transmit and receive */

  up_serialout(priv, M16C_UART_C1, UART_C1_TE|UART_C1_RE);
	
  /* Set UART transmit/receive mode register data bits, stop bits, parity */

  regval = 0;

  if (priv->bits == 7)
    {
      regval |= UART_MR_SMD7BITS;
    }
  else if (priv->bits == 8)
    {
      regval |= UART_MR_SMD8BITS;
    }
  else if (priv->bits == 9)
    {
      regval |= UART_MR_SMD9BITS;
    }
  else
    {
      dbg("Invalid bits=%d\n", priv->bits);
    }
 
  if (priv->parity != 0)
    {
      regval |= UART_MR_PRYE;
      if (priv->parity == 2)
        {
          regval |= UART_MR_PRY;
        }
    }

  if (priv->stopbits2)
    {
      regval |= UART_MR_STPS;
    }

  up_serialout(priv, M16C_UART_MR, regval);

  /* Set port direction registers for Rx/TX pins */

#ifdef CONFIG_M16C_UART0
  if (priv->uartno == 0)
    {
      regval  = getreg8(M16C_PD6);
      regval &= ~(1 << 2);                         /* PD6-2:RxD1 */
      regval |=  (1 << 3);                         /* PD6-3:TxD1 */
      putreg8(regval, M16C_PD6);
    }
  else
#endif
#ifdef CONFIG_M16C_UART1
  if (priv->uartno == 1)
    {
      regval  = getreg8(M16C_PD6);
      regval &= ~(1 << 6);                         /* PD6-6:RxD1 */
      regval |=  (1 << 7);                         /* PD6-7:TxD1 */
      putreg8(regval, M16C_PD6);
    }
  else
#endif
#ifdef CONFIG_M16C_UART2
  if (priv->uartno == 2)
    {
      regval  = getreg8(M16C_PD7);
      regval &= ~(1 << 1);                         /* PD7-1:RxD1 */
      regval &=  (1 << 0);                         /* PD7-0:TxD1 */
      putreg8(regval, M16C_PD7);
    }
  else
#endif
    {
      dbg("Invalid UART #\n");
    }

  /* Read any data left in the RX fifo */

  regval = (uint8_t)up_serialin16(priv, M16C_UART_RB);
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

  /* Attach the UART receive data available IRQ */

  ret = irq_attach(priv->rcvirq, up_rcvinterrupt);
  if (ret == OK)
    {
      /* Attach the UART transmit complete IRQ */

      ret = irq_attach(priv->xmtirq, up_xmtinterrupt);
      if (ret != OK)
        {
          /* Detach the ERI interrupt on failure */

          (void)irq_detach(priv->rcvirq);
        }
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

  /* Disable all UART interrupts */

  up_disableuartint(priv, NULL);

  /* Detach the UART interrupts */

  (void)irq_detach(priv->rcvirq);
  (void)irq_detach(priv->xmtirq);
}

/****************************************************************************
 * Name: up_recvinterrupt
 *
 * Description:
 *   This is the UART receive interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_receivechar to
 *   perform the appropriate data transfer.  The interrupt handling logic
 *   must be able to map the 'irq' number into the approprite up_dev_s
 *   structure in order to call these functions.
 *
 ****************************************************************************/

static int up_rcvinterrupt(int irq, void *context)
{
  struct uart_dev_s *dev = NULL;

#ifdef CONFIG_M16C_UART0
  if (irq == g_uart0priv.rcvirq)
    {
      dev = &g_uart0port;
    }
  else
#endif
#ifdef CONFIG_M16C_UART1
  if (irq == g_uart1priv.rcvirq)
    {
      dev = &g_uart1port;
    }
  else
#endif
#ifdef CONFIG_M16C_UART2
  if (irq = g_uart2priv.rcvirq)
    {
      dev = &g_uart2port;
    }
  else
#endif
    {
      PANIC(OSERR_INTERNAL);
    }

  /* Handle incoming, receive bytes (RDRF: Receive Data Register Full) */

  uart_recvchars(dev);
  return OK;
}

/****************************************************************************
 * Name: up_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one character from
 *   the UART.  Error bits associated with the receipt are provided in the
 *   return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  uint16_t rb;

  /* Read the character from the readbuffer */

  rb = up_serialin16(priv, M16C_UART_RB);

  /* Return the received byte with the receive status */

  *status = rb;

  /* Return the received character (up to 9 bits) */

  return (int)(rb & 0x01ff);
}

/****************************************************************************
 * Name: m16c_rxint, up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void m16c_rxint(struct up_dev_s *dev, bool enable)
{
  irqstate_t flags;
  uint16_t   regaddr;
  uint8_t    regvalue;

  /* Disable interrupts to prevent asynchronous accesses */

  flags = irqsave();

  /* Pick the SxTIC register and enable interrupt priority */

#ifdef CONFIG_M16C_UART0
  if (dev->uartno == 0)
    {
      regaddr  = M16C_S0RIC;
      regvalue = M16C_S0R_PRIO;
    }
  else
#endif
#ifdef CONFIG_M16C_UART1
  if (dev->uartno == 1)
    {
      regaddr  = M16C_S1RIC;
      regvalue = M16C_S1R_PRIO;
    }
  else
#endif
#ifdef CONFIG_M16C_UART2
  if (dev->uartno == 2)
    {
      regaddr  = M16C_S2RIC;
      regvalue = M16C_S2R_PRIO;
    }
  else
#endif
    {
      dbg("Invalid UART #\n");
      return;
    }

  /* Are we enabling or disabling? */

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      putreg8(regvalue, regaddr);
#endif
      dev->enables |= M16C_RXENABLED;
    }
  else
    {
      putreg8(0, regaddr);
      dev->enables = ~M16C_RXENABLED;
    }

  asm ("\tnop\n\tnop\n\tnop"); /* Three NOPs -- probably not necessary here */
  irqrestore(flags);
}

static void up_rxint(struct uart_dev_s *dev, bool enable)
{
  m16c_rxint((struct up_dev_s*)dev->priv, enable);
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the RDR is not empty
 *
 ****************************************************************************/

static bool up_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;

  /* Return true if there is data available in the read buffer */

  return ((up_serialin(priv, M16C_UART_C1) & UART_C1_RI) != 0);
}

/****************************************************************************
 * Name: up_xmtvinterrupt
 *
 * Description:
 *   This is the UART receive interrupt handler.  It will be invoked
 *   when an interrupt received on the 'irq'  It should call
 *   uart_transmitchars or uart_receivechar to perform the
 *   appropriate data transfers.  The interrupt handling logic\
 *   must be able to map the 'irq' number into the approprite
 *   up_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int up_xmtinterrupt(int irq, void *context)
{
  struct uart_dev_s *dev = NULL;

#ifdef CONFIG_M16C_UART0
  if (irq == g_uart0priv.xmtirq)
    {
      dev = &g_uart0port;
    }
  else
#endif
#ifdef CONFIG_M16C_UART1
  if (irq == g_uart1priv.xmtirq)
    {
      dev = &g_uart1port;
    }
  else
#endif
#ifdef CONFIG_M16C_UART2
  if (irq == g_uart2priv.xmtirq)
    {
      dev = &g_uart1port;
    }
  else
#endif
    {
      PANIC(OSERR_INTERNAL);
    }

  /* Handle outgoing, transmit bytes */

  uart_xmitchars(dev);
  return OK;
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

  /* Write the data to the transmit buffer */

  up_serialout16(priv, M16C_UART_TB, (uint16_t)ch);
}

/****************************************************************************
 * Name: m16c_txint, up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void m16c_txint(struct up_dev_s *dev, bool enable)
{
  irqstate_t flags;
  uint16_t   regaddr;
  uint8_t    regvalue;

  /* Disable interrupts to prevent asynchronous accesses */

  flags = irqsave();

  /* Pick the SxTIC register and enable interrupt priority */

#ifdef CONFIG_M16C_UART0
  if (dev->uartno == 0)
    {
      regaddr  = M16C_S0TIC;
      regvalue = M16C_S0T_PRIO;
    }
  else
#endif
#ifdef CONFIG_M16C_UART1
  if (dev->uartno == 1)
    {
      regaddr  = M16C_S1TIC;
      regvalue = M16C_S1T_PRIO;
    }
  else
#endif
#ifdef CONFIG_M16C_UART2
  if (dev->uartno == 2)
    {
      regaddr  = M16C_S2TIC;
      regvalue = M16C_S2T_PRIO;
    }
  else
#endif
    {
      dbg("Invalid UART #\n");
      return;
    }

  /* Are we enabling or disabling? */

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      putreg8(regvalue, regaddr);
#endif
      dev->enables |= M16C_TXENABLED;
    }
  else
    {
      putreg8(0, regaddr);
      dev->enables = ~M16C_TXENABLED;
    }

  asm ("\tnop\n\tnop\n\tnop"); /* Three NOPs -- probably not necessary here */
  irqrestore(flags);
}

static void up_txint(struct uart_dev_s *dev, bool enable)
{
  m16c_txint((struct up_dev_s*)dev->priv, enable);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the TDR is empty
 *
 ****************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
  return ((up_serialin(priv, M16C_UART_C1) & UART_C1_TI) != 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_earlyconsoleinit
 *
 * Description:
 *   Performs the low level UART initialization early in 
 *   debug so that the serial console will be available
 *   during bootup.  This must be called before up_consoleinit.
 *
 ****************************************************************************/

void up_earlyconsoleinit(void)
{
  /* NOTE:  All GPIO configuration for the UARTs was performed in
   * up_lowsetup
   */

  /* Disable all UARTs */

#ifdef TTYS0_DEV
  up_disableuartint(TTYS0_DEV.priv, NULL);
#ifdef TTYS1_DEV
  up_disableuartint(TTYS1_DEV.priv, NULL);
#ifdef TTYS2_DEV
  up_disableuartint(TTYS2_DEV.priv, NULL);
#endif
#endif
#endif

  /* Configuration whichever one is the console */

#ifdef HAVE_SERIALCONSOLE
  CONSOLE_DEV.isconsole = true;
  up_setup(&CONSOLE_DEV);
#endif
}

/****************************************************************************
 * Name: up_consoleinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that up_earlyconsoleinit was called previously.
 *
 ****************************************************************************/

void up_consoleinit(void)
{
  /* Register the console */

#ifdef HAVE_SERIALCONSOLE
  (void)uart_register("/dev/console", &CONSOLE_DEV);
#endif

  /* Register all UARTs */

#ifdef TTYS0_DEV
  (void)uart_register("/dev/ttyS0", &TTYS0_DEV);
#ifdef TTYS1_DEV
  (void)uart_register("/dev/ttyS1", &TTYS1_DEV);
#ifdef TTYS2_DEV
  (void)uart_register("/dev/ttyS2", &TTYS2_DEV);
#endif
#endif
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
#ifdef HAVE_SERIALCONSOLE
  struct up_dev_s *priv = (struct up_dev_s*)CONSOLE_DEV.priv;
  uint8_t  ucon;

  up_disableuartint(priv, &ucon);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_waittxready(priv);
      up_serialout16(priv, M16C_UART_TB, (uint16_t)'\r');
    }

  up_waittxready(priv);
  up_serialout16(priv, M16C_UART_TB, (uint16_t)ch);

  up_waittxready(priv);
  up_restoreuartint(priv, ucon);
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
#ifdef HAVE_SERIALCONSOLE
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
#elif defined(CONFIG_UART0_SERIAL_CONSOLE) || defined(CONFIG_UART1_SERIAL_CONSOLE)|| defined(CONFIG_UART2_SERIAL_CONSOLE)
#    error "A serial console selected, but corresponding UART not enabled"
#endif /* CONFIG_M16C_UART0 || CONFIG_M16C_UART1 || CONFIG_M16C_UART2 */

