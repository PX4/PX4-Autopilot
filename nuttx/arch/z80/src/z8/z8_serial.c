/****************************************************************************
 * arch/z80/src/z8/z8_serial.c
 *
 *   Copyright (C) 2008-2009, 2012 Gregory Nutt. All rights reserved.
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
#include <ez8.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>
#include <arch/serial.h>

#include "chip/chip.h"
#include "os_internal.h"
#include "up_internal.h"

#ifdef USE_SERIALDRIVER

extern uint32_t get_freq(void);

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* System clock frequency value from ZDS target settings */

#define STATE_DISABLED   0
#define STATE_RXENABLED  1
#define STATE_TXENABLED  2

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct z8_uart_s
{
  uint8_t volatile far* uartbase;		/* Base address of UART registers */
  uint32_t     baud;		/* Configured baud */
  bool         rxenabled;	/* RX interrupt enabled */
  bool         txenabled;	/* TX interrupt enabled */
  uint8_t      rxirq;		/* RX IRQ associated with this UART */
  uint8_t      txirq;		/* RX IRQ associated with this UART */
  uint8_t      parity;		/* 0=none, 1=odd, 2=even */
  bool         stopbits2;	/* true: Configure with 2 stop bits
							 * (instead of 1) */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  z8_setup(FAR struct uart_dev_s *dev);
static void z8_shutdown(FAR struct uart_dev_s *dev);
static int  z8_attach(FAR struct uart_dev_s *dev);
static void z8_detach(FAR struct uart_dev_s *dev);
static int  z8_rxinterrupt(int irq, FAR void *context);
static int  z8_txinterrupt(int irq, FAR void *context);
static int  z8_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int  z8_receive(FAR struct uart_dev_s *dev, FAR uint32_t *status);
static void z8_rxint(FAR struct uart_dev_s *dev, bool enable);
static bool z8_rxavailable(FAR struct uart_dev_s *dev);
static void z8_send(FAR struct uart_dev_s *dev, int ch);
static void z8_txint(FAR struct uart_dev_s *dev, bool enable);
static bool z8_txready(FAR struct uart_dev_s *dev);
static bool z8_txempty(FAR struct uart_dev_s *dev);

/****************************************************************************
 * Private Variables
 ****************************************************************************/

struct uart_ops_s g_uart_ops =
{
  z8_setup,          /* setup */
  z8_shutdown,       /* shutdown */
  z8_attach,         /* attach */
  z8_detach,         /* detach */
  z8_ioctl,          /* ioctl */
  z8_receive,        /* receive */
  z8_rxint,          /* rxint */
  z8_rxavailable,    /* rxavailable */
  z8_send,           /* send */
  z8_txint,          /* txint */
  z8_txready,        /* txready */
  z8_txempty         /* txempty */
};

/* I/O buffers */

static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
static char g_uart1rxbuffer[CONFIG_UART1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_UART1_TXBUFSIZE];

/* This describes the state of the DM320 uart0 port. */

static struct z8_uart_s g_uart0priv =
{
  Z8_UART0_BASE,            /* uartbase */
  CONFIG_UART0_BAUD,        /* baud */
  false,                    /* rxenabled */
  false,                    /* txenabled */
  Z8_UART0_RX_IRQ,          /* rxirq */
  Z8_UART0_TX_IRQ,          /* txirq */
  CONFIG_UART0_PARITY,      /* parity */
  CONFIG_UART0_2STOP        /* stopbits2 */
};

static uart_dev_t g_uart0port =
{
  0,                        /* open_count */
  false,                    /* xmitwaiting */
  false,                    /* recvwaiting */
#ifdef CONFIG_UART0_SERIAL_CONSOLE
  true,                     /* isconsole */
#else
  false,                    /* isconsole */
#endif
  { 0 },                    /* closesem */
  { 0 },                    /* xmitsem */
  { 0 },                    /* recvsem */
  {
    { 0 },                  /* xmit.sem */
    0,                      /* xmit.head */
    0,                      /* xmit.tail */
    CONFIG_UART0_TXBUFSIZE, /* xmit.size */
    g_uart0txbuffer,        /* xmit.buffer */
  },
  {
    { 0 },                  /* recv.sem */
    0,                      /* recv.head */
    0,                      /* recv.tail */
    CONFIG_UART0_RXBUFSIZE, /* recv.size */
    g_uart0rxbuffer,        /* recv.buffer */
  },
  &g_uart_ops,              /* ops */
  &g_uart0priv,             /* priv */
};

/* This describes the state of the DM320 uart1 port. */

static struct z8_uart_s g_uart1priv =
{
  Z8_UART1_BASE,            /* uartbase */
  CONFIG_UART1_BAUD,        /* baud */
  false,                    /* rxenabled */
  false,                    /* txenabled */
  Z8_UART1_RX_IRQ,          /* rxirq */
  Z8_UART1_TX_IRQ,          /* txirq */
  CONFIG_UART1_PARITY,      /* parity */
  CONFIG_UART1_2STOP        /* stopbits2 */
};

static uart_dev_t g_uart1port =
{
  0,                        /* open_count */
  false,                    /* xmitwaiting */
  false,                    /* recvwaiting */
#ifdef CONFIG_UART1_SERIAL_CONSOLE
  true,                     /* isconsole */
#else
  false,                    /* isconsole */
#endif
  { 0 },                    /* closesem */
  { 0 },                    /* xmitsem */
  { 0 },                    /* recvsem */
  {
    { 0 },                  /* xmit.sem */
    0,                      /* xmit.head */
    0,                      /* xmit.tail */
    CONFIG_UART1_TXBUFSIZE, /* xmit.size */
    g_uart1txbuffer,        /* xmit.buffer */
  },
  {
    { 0 },                  /* recv.sem */
    0,                      /* recv.head */
    0,                      /* recv.tail */
    CONFIG_UART1_RXBUFSIZE, /* recv.size */
    g_uart0rxbuffer,        /* recv.buffer */
  },
  &g_uart_ops,              /* ops */
  &g_uart1priv,             /* priv */
};

/* Now, which one with be tty0/console and which tty1? */

#ifdef CONFIG_UART1_SERIAL_CONSOLE
# define CONSOLE_DEV     g_uart1port
# define TTYS0_DEV       g_uart1port
# define TTYS1_DEV       g_uart0port
#else
# define CONSOLE_DEV     g_uart0port
# define TTYS0_DEV       g_uart0port
# define TTYS1_DEV       g_uart1port
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z8_putuart
 ****************************************************************************/
 
static inline void z8_putuart(FAR struct z8_uart_s *priv, uint8_t value,
                              uint8_t offset)
{
  putreg8(value, *(priv->uartbase + offset));
}

/****************************************************************************
 * Name: z8_getuart
 ****************************************************************************/
 
static inline uint8_t z8_getuart(FAR struct z8_uart_s *priv, uint8_t offset)
{
  return getreg8(*(priv->uartbase + offset));
}

/****************************************************************************
 * Name: z8_disableuartirq
 ****************************************************************************/

static uint8_t z8_disableuartirq(FAR struct uart_dev_s *dev)
{
  struct z8_uart_s *priv  = (struct z8_uart_s*)dev->priv;
  irqstate_t          flags = irqsave();
  uint8_t             state = priv->rxenabled ? STATE_RXENABLED : STATE_DISABLED | \
                              priv->txenabled ? STATE_TXENABLED : STATE_DISABLED;

  z8_txint(dev, false);
  z8_rxint(dev, false);

  irqrestore(flags);
  return state;
}

/****************************************************************************
 * Name: z8_restoreuartirq
 ****************************************************************************/

static void z8_restoreuartirq(FAR struct uart_dev_s *dev, uint8_t state)
{
  struct z8_uart_s *priv  = (struct z8_uart_s*)dev->priv;
  irqstate_t          flags = irqsave();

  z8_txint(dev, (state & STATE_TXENABLED) ? true : false);
  z8_rxint(dev, (state & STATE_RXENABLED) ? true : false);

  irqrestore(flags);
}

/****************************************************************************
 * Name: z8_consoleput
 ****************************************************************************/

static void z8_consoleput(uint8_t ch)
{
  struct z8_uart_s *priv = (struct z8_uart_s*)CONSOLE_DEV.priv;
  int tmp;

  for (tmp = 1000 ; tmp > 0 ; tmp--)
    {
      if (z8_txready(&CONSOLE_DEV))
        {
          break;
        }
    }
  z8_putuart(priv, ch,  Z8_UART_TXD);
}

/****************************************************************************
 * Name: z8_uartconfigure
 *
 * Description:
 *   Configure hardware for UART functionality
 *
 ****************************************************************************/

void z8_uartconfigure(void)
{
  uint16_t brg;
  uint8_t  val;

  /* Configure GPIO Port A pins 4 & 5 for alternate function */

  putreg8(0x02, PAADDR);
  val = getreg8(PACTL) | 0x30;    /* Set bits in alternate function register */
  putreg8(val, PACTL);
  putreg8(0x07, PAADDR);
  val = getreg8(PACTL) & 0xcf;    /* Reset bits in alternate function set-1 register */
  putreg8(val, PACTL);
  putreg8(0x08, PAADDR);
  val = getreg8(PACTL) & 0xcf;    /* Reset bits in alternate function set-2 register */
  putreg8(val, PACTL);
  putreg8(0x00, PAADDR);

#ifdef EZ8_UART1
  /* Configure GPIO Port D pins 4 & 5 for alternate function */
  
  putreg8(0x02, PAADDR);
  val = getreg8(PDCTL) | 0x30;    /* Set bits in alternate function register */
  putreg8(val, PDCTL);
  putreg8(0x07, PDADDR);
  val = getreg8(PDCTL) & 0xcf;    /* Reset bits in alternate function set-1 register */
  putreg8(val, PDCTL);
  putreg8(0x08, PDADDR);
  val = getreg8(PDCTL) & 0xcf;    /* Reset bits in alternate function set-2 register */
  putreg8(val, PDCTL);
  putreg8(0x00, PDADDR);
#endif
}

/****************************************************************************
 * Name: z8_setup
 *
 * Description:
 *   Configure the UART baud, parity, etc. This method is called the first
 *   time that the serial port is opened.
 *
 ****************************************************************************/

static int z8_setup(FAR struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
  struct z8_uart_s *priv = (struct z8_uart_s*)dev->priv;
  uint32_t freq = get_freq();
  uint16_t brg;
  uint8_t ctl0;
  uint8_t ctl1;

  /* Calculate and set the baud rate generation register.
   * BRG = (freq + baud * 8)/(baud * 16)
   */

  brg = (freq + (priv->baud << 3))/(priv->baud << 4);
  z8_putuart(priv, brg >> 8, Z8_UART_BRH);
  z8_putuart(priv, brg & 0xff, Z8_UART_BRL);

  /* Configure STOP bits */

  ctl0 = ctl1 = 0;
  if (priv->stopbits2)
    {
      ctl0 |= Z8_UARTCTL0_STOP;
    }

  /* Configure parity */

  if (priv->parity == 1)
    {
      ctl0 |= (Z8_UARTCTL0_PEN|Z8_UARTCTL0_PSEL);
    }
  else if (priv->parity == 2)
    {
      ctl0 |= Z8_UARTCTL0_PEN;
    }

  z8_putuart(priv, ctl0, Z8_UART_CTL0);
  z8_putuart(priv, ctl1, Z8_UART_CTL1);

  /* Enable UART receive (REN) and transmit (TEN) */

  ctl0 |= (Z8_UARTCTL0_TEN|Z8_UARTCTL0_REN);
  z8_putuart(priv, ctl0, Z8_UART_CTL0);
#endif
  return OK;
}

/****************************************************************************
 * Name: z8_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void z8_shutdown(FAR struct uart_dev_s *dev)
{
  struct z8_uart_s *priv = (struct z8_uart_s*)dev->priv;
  (void)z8_disableuartirq(dev);
}

/****************************************************************************
 * Name: z8_attach
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

static int z8_attach(FAR struct uart_dev_s *dev)
{
  struct z8_uart_s *priv = (struct z8_uart_s*)dev->priv;
  int ret;

  /* Attach the RX IRQ */

  ret = irq_attach(priv->rxirq, z8_rxinterrupt);
  if (ret == OK)
    {
      /* Attach the TX IRQ */

      ret = irq_attach(priv->txirq, z8_txinterrupt);
      if (ret != OK)
        {
          irq_detach(priv->rxirq);
        }
    }
  return ret;
}

/****************************************************************************
 * Name: z8_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception is
 *   the serial console which is never shutdown.
 *
 ****************************************************************************/

static void z8_detach(FAR struct uart_dev_s *dev)
{
  struct z8_uart_s *priv = (struct z8_uart_s*)dev->priv;
  up_disable_irq(priv->rxirq);
  up_disable_irq(priv->txirq);
  irq_detach(priv->rxirq);
  irq_detach(priv->txirq);
}

/****************************************************************************
 * Name: z8_rxinterrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an RX
 *   event occurs at the z8's LIN-UART.
 *
 ****************************************************************************/

static int z8_rxinterrupt(int irq, FAR void *context)
{
  struct uart_dev_s  *dev = NULL;
  struct z8_uart_s *priv;
  uint8_t            status;

  if (g_uart1priv.rxirq == irq)
    {
      dev = &g_uart1port;
    }
  else if (g_uart0priv.rxirq == irq)
    {
      dev = &g_uart0port;
    }
  else
    {
      PANIC(OSERR_INTERNAL);
    }

  priv = (struct z8_uart_s*)dev->priv;

  /* Check the LIN-UART status 0 register to determine whether the source of
   * the interrupt is error, break, or received data
   */

  status = z8_getuart(priv, Z8_UART_STAT0);

  /* REVISIT error and break handling */

  /* Check if received data is available  */

  if (status & Z8_UARTSTAT0_RDA)
    {
      /* Handline an incoming, receive byte */

      uart_recvchars(dev);
    }
  return OK;
}

/****************************************************************************
 * Name: z8_txinterrupt
 *
 * Description:
 *   This is the UART TX interrupt handler.  This interrupt handler will
 *   be invoked when the X16F LIN UART transmit data register is empty.
 *
 ****************************************************************************/

static int z8_txinterrupt(int irq, FAR void *context)
{
  struct uart_dev_s  *dev = NULL;
  struct z8_uart_s *priv;
  uint8_t            status;

  if (g_uart1priv.txirq == irq)
    {
      dev = &g_uart1port;
    }
  else if (g_uart0priv.txirq == irq)
    {
      dev = &g_uart0port;
    }
  else
    {
      PANIC(OSERR_INTERNAL);
    }

  priv = (struct z8_uart_s*)dev->priv;

  /* Verify that the transmit data register is empty */

  status = z8_getuart(priv, Z8_UART_STAT0);
  if (status & Z8_UARTSTAT0_TDRE)
    {
      /* Handle outgoing, transmit bytes */

      uart_xmitchars(dev);
    }
  return OK;
}

/****************************************************************************
 * Name: z8_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int z8_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  *get_errno_ptr() = ENOTTY;
  return ERROR;
}

/****************************************************************************
 * Name: z8_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one character from
 *   the UART.  Error bits associated with the receipt are provided in the
 *   return 'status'.
 *
 ****************************************************************************/

static int z8_receive(FAR struct uart_dev_s *dev, FAR uint32_t *status)
{
  struct z8_uart_s *priv = (struct z8_uart_s*)dev->priv;
  uint8_t  rxd;
  uint8_t  stat0;

  rxd     = z8_getuart(priv, Z8_UART_RXD);
  stat0   = z8_getuart(priv, Z8_UART_STAT0);
  *status = (uint32_t)rxd | (((uint32_t)stat0) << 8);
  return rxd;
}

/****************************************************************************
 * Name: z8_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void z8_rxint(FAR struct uart_dev_s *dev, bool enable)
{
  struct z8_uart_s *priv  = (struct z8_uart_s*)dev->priv;
  irqstate_t          flags = irqsave();

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      up_enable_irq(priv->rxirq);
#endif
    }
  else
    {
      up_disable_irq(priv->rxirq);
    }

  priv->rxenabled = enable;
  irqrestore(flags);
}

/****************************************************************************
 * Name: z8_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

static bool z8_rxavailable(FAR struct uart_dev_s *dev)
{
  struct z8_uart_s *priv = (struct z8_uart_s*)dev->priv;
  return ((z8_getuart(priv, Z8_UART_STAT0) & Z8_UARTSTAT0_RDA) != 0);
}

/****************************************************************************
 * Name: z8_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void z8_send(FAR struct uart_dev_s *dev, int ch)
{
  struct z8_uart_s *priv = (struct z8_uart_s*)dev->priv;
  z8_putuart(priv, ch, Z8_UART_TXD);
}

/****************************************************************************
 * Name: z8_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void z8_txint(FAR struct uart_dev_s *dev, bool enable)
{
  struct z8_uart_s *priv  = (struct z8_uart_s*)dev->priv;
  irqstate_t          flags = irqsave();

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      up_enable_irq(priv->txirq);
#endif
    }
  else
    {
      up_disable_irq(priv->txirq);
    }

  priv->txenabled = enable;
  irqrestore(flags);
}

/****************************************************************************
 * Name: z8_txready
 *
 * Description:
 *   Return true if the tranmsit fifo is not full
 *
 ****************************************************************************/

static bool z8_txready(FAR struct uart_dev_s *dev)
{
  struct z8_uart_s *priv = (struct z8_uart_s*)dev->priv;
  return ((z8_getuart(priv, Z8_UART_STAT0) & Z8_UARTSTAT0_TDRE) != 0);
}

/****************************************************************************
 * Name: z8_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ****************************************************************************/

static bool z8_txempty(FAR struct uart_dev_s *dev)
{
  struct z8_uart_s *priv = (struct z8_uart_s*)dev->priv;
  return ((z8_getuart(priv, Z8_UART_STAT0) & Z8_UARTSTAT0_TXE) != 0);
}

/****************************************************************************
 * Public Funtions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.
 *
 ****************************************************************************/

void up_serialinit(void)
{
   /* Disable all UART interrupts */

  (void)z8_disableuartirq(&TTYS0_DEV);
  (void)z8_disableuartirq(&TTYS1_DEV);

  /* Initialize the console for early use */
  CONSOLE_DEV.isconsole = true;
  z8_setup(&CONSOLE_DEV);

  /* Reigster console and tty devices */

  (void)uart_register("/dev/console", &CONSOLE_DEV);
  (void)uart_register("/dev/ttyS0", &TTYS0_DEV);
  (void)uart_register("/dev/ttyS1", &TTYS1_DEV);
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug
 *   writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
  uint8_t  state;

  /* Keep interrupts disabled so that we do not interfere with normal
   * driver operation 
   */

  state = z8_disableuartirq(&CONSOLE_DEV);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR before LF */

      z8_consoleput('\r');
    }

  /* Output the character */

  z8_consoleput((uint8_t)ch);

  /* It is important to restore the TX interrupt while the send is pending.
   * otherwise, TRDE interrupts can be lost since they do not pend after the
   * TRDE false->true transition.
   */

  z8_restoreuartirq(&CONSOLE_DEV, state);
  return ch;
}

#else /* USE_SERIALDRIVER */

/****************************************************************************
 * Definitions
 ****************************************************************************/

#ifdef CONFIG_UART1_SERIAL_CONSOLE
# define z8_contrde() \
  ((getreg8(*(Z8_UART1_BASE+Z8_UART_STAT0)) & Z8_UARTSTAT0_TDRE) != 0)
# define z8_contxd(ch) \
  putreg8((uint8_t)(ch), *(Z8_UART1_BASE+Z8_UART_TXD))
#else
# define z8_contrde() \
  ((getreg8(*(Z8_UART0_BASE+Z8_UART_STAT0)) & Z8_UARTSTAT0_TDRE) != 0)
# define z8_contxd(ch) \
  putreg8((uint8_t)(ch), *(Z8_UART0_BASE+Z8_UART_TXD))
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z8_putc
 ****************************************************************************/

static void z8_putc(int ch)
{
  int tmp;
  for (tmp = 1000 ; tmp > 0 && !z8_contrde(); tmp--);
  z8_contxd(ch);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_putc
 ****************************************************************************/

int up_putc(int ch)
{
  /* Check for LF */

  if (ch == '\n')
    {
      /* Output CR before LF */

      z8_putc('\r');
    }

  /* Output character */

  z8_putc(ch);
  return ch;
}

#endif /* USE_SERIALDRIVER */
