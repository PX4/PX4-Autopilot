/****************************************************************************
 * arch/z80/src/ez08/z180_scc.c
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
#include <arch/io.h>

#include "chip/chip.h"
#include "os_internal.h"
#include "up_internal.h"

#ifdef USE_SERIALDRIVER

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct z180_dev_s
{
  uint32_t     baud;        /* Configured baud */
  uint8_t      cr;          /* [E]SCC control register */
  uint8_t      dr;          /* [E]SCC data register */
  uint8_t      irq;         /* IRQ associated with this [E]SCC */
  uint8_t      parity;      /* 0=none, 1=odd, 2=even */
  uint8_t      bits;        /* Number of bits (7 or 8) */
  bool         stopbits2;   /* true: Configure with 2 (vs 1) */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  z180_setup(struct uart_dev_s *dev);
static void z180_shutdown(struct uart_dev_s *dev);
static int  z180_attach(struct uart_dev_s *dev);
static void z180_detach(struct uart_dev_s *dev);
static int  z180_interrrupt(int irq, void *context);
static int  z180_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  z180_receive(struct uart_dev_s *dev, unsigned int *status);
static void z180_rxint(struct uart_dev_s *dev, bool enable);
static bool z180_rxavailable(struct uart_dev_s *dev);
static void z180_send(struct uart_dev_s *dev, int ch);
static void z180_txint(struct uart_dev_s *dev, bool enable);
static bool z180_txready(struct uart_dev_s *dev);
static bool z180_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Variables
 ****************************************************************************/

struct uart_ops_s g_uart_ops =
{
  z180_setup,          /* setup */
  z180_shutdown,       /* shutdown */
  z180_attach,         /* attach */
  z180_detach,         /* detach */
  z180_ioctl,          /* ioctl */
  z180_receive,        /* receive */
  z180_rxint,          /* rxint */
  z180_rxavailable,    /* rxavailable */
  z180_send,           /* send */
  z180_txint,          /* txint */
  z180_txready,        /* txready */
  z180_txempty         /* txempty */
};

/* I/O buffers */

#ifdef CONFIG_Z180_SCC
static char g_scc_rxbuffer[CONFIG_Z180_SCC_RXBUFSIZE];
static char g_scc_txbuffer[CONFIG_Z180_SCC_TXBUFSIZE];
#endif
#ifdef CONFIG_Z180_ESCCA
static char g_escca_rxbuffer[CONFIG_Z180_ESCCA_RXBUFSIZE];
static char g_escca_txbuffer[CONFIG_Z180_ESCCA_TXBUFSIZE];
#endif
#ifdef CONFIG_Z180_ESCCB
static char g_esccb_rxbuffer[CONFIG_Z180_ESCCB_RXBUFSIZE];
static char g_esccb_txbuffer[CONFIG_Z180_ESCCB_TXBUFSIZE];
#endif

/* This describes the state of the SCC port. */

#ifdef CONFIG_Z180_SCC
static const struct z180_dev_s g_scc_priv =
{
  CONFIG_Z180_SCC_BAUD,        /* baud */
  Z181_SCC_CR,                 /* cr */
  Z181_SCC_DR,                 /* dr */
  Z180_SCC_IRQ,                /* irq */
  CONFIG_Z180_SCC_PARITY,      /* parity */
  CONFIG_Z180_SCC_BITS,        /* bits */
  CONFIG_Z180_SCC_2STOP        /* stopbits2 */
};

static uart_dev_t g_scc_port =
{
  0,                           /* open_count */
  false,                       /* xmitwaiting */
  false,                       /* recvwaiting */
#ifdef CONFIG_Z180_SCC_SERIAL_CONSOLE
  true,                        /* isconsole */
#else
  false,                       /* isconsole */
#endif
  { 0 },                       /* closesem */
  { 0 },                       /* xmitsem */
  { 0 },                       /* recvsem */
  {
    { 0 },                     /* xmit.sem */
    0,                         /* xmit.head */
    0,                         /* xmit.tail */
    CONFIG_Z180_SCC_TXBUFSIZE, /* xmit.size */
    g_scc_txbuffer,            /* xmit.buffer */
  },
  {
    { 0 },                     /* recv.sem */
    0,                         /* recv.head */
    0,                         /* recv.tail */
    CONFIG_Z180_SCC_RXBUFSIZE, /* recv.size */
    g_scc_rxbuffer,            /* recv.buffer */
  },
  &g_uart_ops,                 /* ops */
  &g_scc_priv,                 /* priv */
};
#endif

/* This describes the state of the ESCC Channel A port. */

#ifdef CONFIG_Z180_ESCCA
static const struct z180_dev_s g_escca_priv =
{
  CONFIG_Z180_ESCCA_BAUD,      /* baud */
  Z182_ESCCA_CR,               /* cr */
  Z182_ESCCA_DR,               /* dr */
  Z180_UART1_IRQ,              /* irq */
  CONFIG_Z180_ESCCA_PARITY,    /* parity */
  CONFIG_Z180_ESCCA_BITS,      /* bits */
  CONFIG_Z180_ESCCA_2STOP      /* stopbits2 */
};

static uart_dev_t g_escca_port =
{
  0,                           /* open_count */
  false,                       /* xmitwaiting */
  false,                       /* recvwaiting */
#ifdef CONFIG_Z180_ESCCA_SERIAL_CONSOLE
  true,                        /* isconsole */
#else
  false,                       /* isconsole */
#endif
  { 0 },                       /* closesem */
  { 0 },                       /* xmitsem */
  { 0 },                       /* recvsem */
  {
    { 0 },                     /* xmit.sem */
    0,                         /* xmit.head */
    0,                         /* xmit.tail */
    CONFIG_Z180_ESCCA_TXBUFSIZE, /* xmit.size */
    g_escca_txbuffer,          /* xmit.buffer */
  },
  {
    { 0 },                     /* recv.sem */
    0,                         /* recv.head */
    0,                         /* recv.tail */
    CONFIG_Z180_ESCCA_RXBUFSIZE, /* recv.size */
    g_escca_rxbuffer,          /* recv.buffer */
  },
  &g_uart_ops,                 /* ops */
  &g_escca_priv,               /* priv */
};
#endif

/* This describes the state of the ESCC Channel B port. */

#ifdef CONFIG_Z180_ESCCB
static const struct z180_dev_s g_esccb_priv =
{
  CONFIG_Z180_ESCCB_BAUD,      /* baud */
  Z182_ESCCB_CR,               /* cr */
  Z182_ESCCB_DR,               /* dr */
  Z180_UART1_IRQ,              /* irq */
  CONFIG_Z180_ESCCB_PARITY,    /* parity */
  CONFIG_Z180_ESCCB_BITS,      /* bits */
  CONFIG_Z180_ESCCB_2STOP      /* stopbits2 */
};

static uart_dev_t g_escca_port =
{
  0,                           /* open_count */
  false,                       /* xmitwaiting */
  false,                       /* recvwaiting */
#ifdef CONFIG_Z180_ESCCA_SERIAL_CONSOLE
  true,                        /* isconsole */
#else
  false,                       /* isconsole */
#endif
  { 0 },                       /* closesem */
  { 0 },                       /* xmitsem */
  { 0 },                       /* recvsem */
  {
    { 0 },                     /* xmit.sem */
    0,                         /* xmit.head */
    0,                         /* xmit.tail */
    CONFIG_Z180_ESCCA_TXBUFSIZE, /* xmit.size */
    g_escca_txbuffer,          /* xmit.buffer */
  },
  {
    { 0 },                     /* recv.sem */
    0,                         /* recv.head */
    0,                         /* recv.tail */
    CONFIG_Z180_ESCCA_RXBUFSIZE, /* recv.size */
    g_escca_rxbuffer,          /* recv.buffer */
  },
  &g_uart_ops,                 /* ops */
  &g_escca_priv,               /* priv */
};
#endif

/* Now, which one with be tty0/console and which tty1? NOTE: SCC and ESCCA/B and
 * mutually exclusive.
 */

#undef CONSOLE_DEV
#undef TTYS0_DEV 
#undef TTYS1_DEV

#if defined(CONFIG_Z180_SCC_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_scc_port
#  define TTYS0_DEV       g_scc_port
#elif defined(CONFIG_Z180_SCC)
#  define TTYS0_DEV       g_scc_port

#elif defined(CONFIG_Z180_ESCCA_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_escca_port
#  define TTYS0_DEV       g_escca_port
#  if defined(CONFIG_Z180_ESCCB)
#    define TTYS1_DEV     g_esccb_port
#  endif

#elif defined(CONFIG_Z180_ESCCB_SERIAL_CONSOLE)
#  define CONSOLE_DEV     g_esccb_port
#  define TTYS0_DEV       g_esccb_port
#  if defined(CONFIG_Z180_ESCCA)
#    define TTYS1_DEV     g_escca_port
#  endif

#elif defined(CONFIG_Z180_ESCCA)
#  define TTYS0_DEV       g_escca_port
#  if defined(CONFIG_Z180_ESCCB)
#    define TTYS1_DEV     g_esccb_port
# endif

#elif defined(CONFIG_Z180_ESCCB)
# define TTYS0_DEV       g_esccb_port
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z180_serialin
 ****************************************************************************/

static inline uint8_t z180_serialin(struct z180_dev_s *priv, uint8_t regaddr)
{
#warning "Missing logic"
}

/****************************************************************************
 * Name: z180_serialout
 ****************************************************************************/

static inline void z180_serialout(struct z180_dev_s *priv, uint8_t regaddr,
                                  uint8_t value)
{
#warning "Missing logic"
}

/****************************************************************************
 * Name: z180_disableuartint
 ****************************************************************************/

static inline void z180_disableuartint(struct z180_dev_s *priv)
{
#warning "Missing logic"
}

/****************************************************************************
 * Name: z180_restoreuartint
 ****************************************************************************/

static inline void z180_restoreuartint(struct z180_dev_s *priv, uint8_t bits)
{
#warning "Missing logic"
}

/****************************************************************************
 * Name: z180_waittxready
 ****************************************************************************/

static inline void z180_waittxready(struct z180_dev_s *priv)
{
#warning "Missing logic"
}

/****************************************************************************
 * Name: z180_setbaud
 ****************************************************************************/

static inline void z180_setbaud(struct z180_dev_s *priv, uint24_t baud)
{
#warning "Missing logic"
}

/****************************************************************************
 * Name: z180_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This method is called
 *   the first time that the serial port is opened.
 *
 ****************************************************************************/

static int z180_setup(struct uart_dev_s *dev)
{
#ifndef CONFIG_SUPPRESS_UART_CONFIG
#  warning "Missing logic"
#endif
  return OK;
}

/****************************************************************************
 * Name: z180_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial port is closed
 *
 ****************************************************************************/

static void z180_shutdown(struct uart_dev_s *dev)
{
#warning "Missing logic"
}

/****************************************************************************
 * Name: z180_attach
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

static int z180_attach(struct uart_dev_s *dev)
{
#warning "Missing logic"
}

/****************************************************************************
 * Name: z180_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The exception
 *   is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void z180_detach(struct uart_dev_s *dev)
{
#warning "Missing logic"
}

/****************************************************************************
 * Name: z180_interrrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked
 *   when an interrupt received on the 'irq'  It should call
 *   uart_transmitchars or uart_receivechar to perform the
 *   appropriate data transfers.  The interrupt handling logic\
 *   must be able to map the 'irq' number into the approprite
 *   uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int z180_interrrupt(int irq, void *context)
{
#warning "Missing logic"
}

/****************************************************************************
 * Name: z180_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int z180_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#warning "Missing logic"
}

/****************************************************************************
 * Name: z180_receive
 *
 * Description:
 * Called (usually) from the interrupt level to receive one character from
 * the UART.  Error bits associated with the receipt are provided in the
 * the return 'status'.
 *
 ****************************************************************************/

static int z180_receive(struct uart_dev_s *dev, unsigned int *status)
{
#warning "Missing logic"
}

/****************************************************************************
 * Name: z180_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void z180_rxint(struct uart_dev_s *dev, bool enable)
{
#warning "Missing logic"
}

/****************************************************************************
 * Name: z180_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

static bool z180_rxavailable(struct uart_dev_s *dev)
{
#warning "Missing logic"
}

/****************************************************************************
 * Name: z180_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void z180_send(struct uart_dev_s *dev, int ch)
{
#warning "Missing logic"
}

/****************************************************************************
 * Name: z180_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void z180_txint(struct uart_dev_s *dev, bool enable)
{
#warning "Missing logic"
}

/****************************************************************************
 * Name: z180_txready
 *
 * Description:
 *   Return true if the tranmsit fifo is not full
 *
 ****************************************************************************/

static bool z180_txready(struct uart_dev_s *dev)
{
#warning "Missing logic"
}

/****************************************************************************
 * Name: z180_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ****************************************************************************/

static bool z180_txempty(struct uart_dev_s *dev)
{
#warning "Missing logic"
}

/****************************************************************************
 * Public Functions
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
  uint8_t regval;

  /* Make sure that all UART interrupts are disabled */
#warning "Missing logic"

  /* Configure for usage of [E]SCC channels */

#ifdef CONFIG_Z180_SCC
#  warning "Missing logic"
#endif

#ifdef CONFIG_Z180_ESCCA
#  warning "Missing logic"
#endif

#ifdef CONFIG_Z180_ESCCB
#  warning "Missing logic"
#endif

  /* If there is a console, then configure the console now */

#ifdef CONSOLE_DEV
  CONSOLE_DEV.isconsole = true;
  z180_setup(&CONSOLE_DEV);
#endif

  /* Register console and tty devices */

#ifdef CONSOLE_DEV
  (void)uart_register("/dev/console", &CONSOLE_DEV);
#endif
  (void)uart_register("/dev/ttyS0", &TTYS0_DEV);
#ifdef TTYS1_DEV
  (void)uart_register("/dev/ttyS1", &TTYS1_DEV);
#endif
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
#ifdef CONSOLE_DEV
#warning "Missing logic"
  z180_disableuartint(priv);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Output CR before LF*/
#warning "Missing logic"

    }

  /* Output the character */

#warning "Missing logic"

  /* Wait for the character to be sent before re-enabling interrupts */
 
#warning "Missing logic"
  return ch;
#endif
}

#else /* USE_SERIALDRIVER */

/****************************************************************************
 * Definitions
 ****************************************************************************/

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
 * Name: z180_putc
 ****************************************************************************/

static void z180_putc(int ch)
{
#warning "Missing logic"
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

      z180_putc('\r');
    }

  /* Output character */

  z180_putc(ch);
  return ch;
}

#endif /* USE_SERIALDRIVER */
