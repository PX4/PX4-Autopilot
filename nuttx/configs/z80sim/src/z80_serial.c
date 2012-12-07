/****************************************************************************
 * configs/z80sim/src/z80_serial.c
 *
 *   Copyright (C) 2007-2009, 2012 Gregory Nutt. All rights reserved.
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
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>
#include <arch/serial.h>

#include "up_arch.h"
#include "os_internal.h"
#include "up_internal.h"

#ifdef USE_SERIALDRIVER

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_setup(FAR struct uart_dev_s *dev);
static void up_shutdown(FAR struct uart_dev_s *dev);
static int  up_attach(FAR struct uart_dev_s *dev);
static void up_detach(FAR struct uart_dev_s *dev);
static int  up_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int  up_receive(FAR struct uart_dev_s *dev, unsigned int *status);
static void up_rxint(FAR struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(FAR struct uart_dev_s *dev);
static void up_send(FAR struct uart_dev_s *dev, int ch);
static void up_txint(FAR struct uart_dev_s *dev, bool enable);
static bool up_txready(FAR struct uart_dev_s *dev);
static bool up_txempty(FAR struct uart_dev_s *dev);

/****************************************************************************
 * Private Variables
 ****************************************************************************/

struct uart_ops_s g_uart_ops =
{
  up_setup,                 /* setup */
  up_shutdown,              /* shutdown */
  up_attach,                /* attach */
  up_detach,                /* detach */
  up_ioctl,                 /* ioctl */
  up_receive,               /* receive */
  up_rxint,                 /* rxint */
  up_rxavailable,           /* rxavailable */
  up_send,                  /* send */
  up_txint,                 /* txint */
  up_txready,               /* txready */
  up_txempty,               /* txempty */
};

/* I/O buffers */

static char g_uartrxbuffer[CONFIG_UART_RXBUFSIZE];
static char g_uarttxbuffer[CONFIG_UART_TXBUFSIZE];

/* This describes the state of the fake UART port. */

static uart_dev_t g_uartport =
{
  0,                        /* open_count */
  false,                    /* xmitwaiting */
  false,                    /* recvwaiting */
  true,                     /* isconsole */
  { 1 },                    /* closesem */
  { 0 },                    /* xmitsem */
  { 0 },                    /* recvsem */
  {                         /* xmit */
    { 1 },                  /*   sem */
    0,                      /*   head */
    0,                      /*   tail */
    CONFIG_UART_TXBUFSIZE,  /*   size */
    g_uarttxbuffer,         /*   buffer */
  },
  {                         /* recv */
    { 1 },                  /*   sem */
    0,                      /*   head */
    0,                      /*   tail */
    CONFIG_UART_RXBUFSIZE,  /*   size */
    g_uartrxbuffer,         /*   buffer */
  },
  &g_uart_ops,              /* ops */
  NULL,                     /* priv */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This
 *   method is called the first time that the serial port is
 *   opened.
 *
 ****************************************************************************/

static int up_setup(FAR struct uart_dev_s *dev)
{
  return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void up_shutdown(FAR struct uart_dev_s *dev)
{
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method is
 *   called when the serial port is opened.  Normally, this is just after the
 *   setup() method is called, however, the serial console may operate in a
 *   non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() methods are called.
 *
 ****************************************************************************/

static int up_attach(FAR struct uart_dev_s *dev)
{
  return OK;
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

static void up_detach(FAR struct uart_dev_s *dev)
{
}

/****************************************************************************
 * Name: up_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int up_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  *get_errno_ptr() = ENOTTY;
  return ERROR;
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

static int up_receive(FAR struct uart_dev_s *dev, unsigned int *status)
{
  uint8_t ch = z80_lowgetc();
  *status = 0;
  return ch;
}

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void up_rxint(FAR struct uart_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

static bool up_rxavailable(FAR struct uart_dev_s *dev)
{
  return true;
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void up_send(FAR struct uart_dev_s *dev, int ch)
{
  z80_lowputc(ch);
}

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void up_txint(FAR struct uart_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the tranmsit fifo is not full
 *
 ****************************************************************************/

static bool up_txready(FAR struct uart_dev_s *dev)
{
  return true;
}

/****************************************************************************
 * Name: up_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ****************************************************************************/

static bool up_txempty(FAR struct uart_dev_s *dev)
{
  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Performs the low level UART initialization early in 
 *   debug so that the serial console will be available
 *   during bootup.  This must be called before up_serialinit.
 *
 ****************************************************************************/

void up_earlyserialinit(void)
{
}

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that up_earlyserialinit was called previously.
 *
 ****************************************************************************/

void up_serialinit(void)
{
  (void)uart_register("/dev/console", &g_uartport);
  (void)uart_register("/dev/ttyS0", &g_uartport);
}
#endif /* USE_SERIALDRIVER */

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
  z80_lowputc(ch);
  return 0;
}

