/****************************************************************************
 * arch/rgmp/src/x86/com.c
 *
 *   Copyright (C) 2011 Yu Qiang. All rights reserved.
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Authors: Yu Qiang <yuq825@gmail.com>
 *            Gregory Nutt <gnutt@nuttx.org>
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

#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>
#include <nuttx/kmalloc.h>

#include <arch/com.h>

#include <rgmp/trap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define COM1            0x3F8
#define COM2            0x2f8
#define COM3            0x3e8
#define COM4            0x2e8

#define COM_RX          0       // In:        Receive buffer (DLAB=0)
#define COM_DLL         0       // Out: Divisor Latch Low (DLAB=1)
#define COM_TX          0       // Out: Transmit buffer (DLAB=0)
#define COM_DLM         1       // Out: Divisor Latch High (DLAB=1)
#define COM_IER         1       // Out: Interrupt Enable Register
#define   COM_IER_TEI   0x02    //   Enable transmit buffer empty interrupt
#define   COM_IER_RDI   0x01    //   Enable receiver data interrupt
#define COM_IIR         2       // In:        Interrupt ID Register
#define COM_FCR         2       // Out: FIFO Control Register
#define COM_LCR         3       // Out: Line Control Register
#define   COM_LCR_DLAB  0x80    //   Divisor latch access bit
#define   COM_LCR_WLEN8 0x03    //   Wordlength: 8 bits
#define COM_MCR         4       // Out: Modem Control Register
#define   COM_MCR_RTS   0x02    // RTS complement
#define   COM_MCR_DTR   0x01    // DTR complement
#define   COM_MCR_OUT2  0x08    // Out2 complement
#define COM_LSR         5       // In:        Line Status Register
#define   COM_LSR_DATA  0x01    //   Data available
#define   COM_LSR_ETR   0x20    //   buffer has space
#define   COM_LSR_EDR   0x40    //   buffer empty

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifndef CONFIG_COM_RXBUFSIZE
#define CONFIG_COM_RXBUFSIZE    64
#endif

#ifndef CONFIG_COM_TXBUFSIZE
#define CONFIG_COM_TXBUFSIZE    64
#endif

struct up_dev_s
{
    unsigned int         base;          /* Base address of COM registers */
    unsigned int         baud;                /* Configured baud */
    int                  irq;                /* IRQ associated with this COM */
    struct irq_action    action;
    union {
        uint8_t val;
        struct {
            unsigned bits     : 2;      /* 3=8 bits, 2=7 bits, 1=6 bits, 0=5 bits */
            unsigned stopbits : 1;      /* 0=1 stop bit, 1=2 stop bits */
            unsigned parity   : 3;      /* xx0=none, 001=odd, 011=even */
            unsigned ebreak   : 1;
            unsigned dlab     : 1;
        } sep;
    } lcr;
    char rxbuff[CONFIG_COM_RXBUFSIZE];  /* receive buffer */
    char txbuff[CONFIG_COM_TXBUFSIZE];  /* transmit buffer */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static irqreturn_t up_com_int_handler(int irq, void *dev_id);
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  up_receive(struct uart_dev_s *dev, unsigned int *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);
static bool up_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Variables
 ****************************************************************************/

static struct uart_ops_s g_com_ops =
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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_alloc_com
 ****************************************************************************/

static uart_dev_t *up_alloc_com(unsigned int base, int irq)
{
    uart_dev_t *dev;
    struct up_dev_s *priv;

    priv = kzalloc(sizeof(struct up_dev_s));
    if (priv == NULL)
        goto err0;

    dev = kzalloc(sizeof(uart_dev_t));
    if (dev == NULL)
        goto err1;

    priv->base = base;
    priv->irq = irq;
    priv->baud = 115200;
    priv->lcr.val = 0;
    priv->lcr.sep.parity = 0;
    priv->lcr.sep.bits = 3;
    priv->lcr.sep.stopbits = 0;
    priv->action.handler = up_com_int_handler;
    priv->action.dev_id = dev;

    dev->recv.size = CONFIG_COM_RXBUFSIZE;
    dev->recv.buffer = priv->rxbuff;
    dev->xmit.size = CONFIG_COM_TXBUFSIZE;
    dev->xmit.buffer = priv->txbuff;
    dev->ops = &g_com_ops;
    dev->priv = priv;

    return dev;

 err1:
    kfree(priv);
 err0:
    return NULL;
}

/****************************************************************************
 * Name: up_alloc_com
 ****************************************************************************/

static inline void up_free_com(uart_dev_t *com)
{
    kfree(com->priv);
    kfree(com);
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
    struct up_dev_s *priv = dev->priv;
    uint16_t base = priv->base;
    union {
        uint16_t val;
        struct {
            uint8_t low;
            uint8_t high;
        } sep;
    } data;

    // clear and disable FIFO
    outb(base+COM_FCR, 1);
    outb(base+COM_FCR, 3);
    outb(base+COM_FCR, 0);

    // Clear any preexisting overrun indications and interrupts
    // Serial port doesn't exist if COM_LSR returns 0xFF
    inb(base+COM_LSR);
    inb(base+COM_IIR);
    inb(base+COM_RX);
    if (inb(base+COM_LSR) == 0xff) {
        dbg("COM %d does not exist\n", base);
        return -1;
    }

    // Set speed; requires DLAB latch
    outb(base+COM_LCR, COM_LCR_DLAB);
    data.val = 115200 / priv->baud;
    outb(base+COM_DLL, data.sep.low);
    outb(base+COM_DLM, data.sep.high);

    // set data bits, stop bit, parity; turn off DLAB latch
    outb(base+COM_LCR, priv->lcr.val);

    // OUT2 must be set to enable interrupt
    outb(base+COM_MCR, COM_MCR_OUT2);

    // setup FIFO
    outb(base+COM_FCR, 1);

    // disable COM interrupts
    outb(base+COM_IER, 0);

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
    struct up_dev_s *priv = dev->priv;
    uint16_t base = priv->base;

    // disable COM interrupts
    outb(base+COM_IER, 0);
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
    struct up_dev_s *priv = dev->priv;
    int err;

    err = rgmp_request_irq(priv->irq, &priv->action, 0);
  
    return err;
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
    struct up_dev_s *priv = dev->priv;

    rgmp_free_irq(priv->irq, &priv->action);
}

/****************************************************************************
 * Name: up_com_int_handler
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

static irqreturn_t up_com_int_handler(int irq, void *dev_id)
{
    struct uart_dev_s *dev = dev_id;
    struct up_dev_s   *priv = dev->priv;
    uint16_t base = priv->base;
    //uint8_t cause = inb(base+COM_IIR);
    uint8_t state = inb(base+COM_LSR);

    if (state & COM_LSR_DATA)
        uart_recvchars(dev);

    if (state & COM_LSR_ETR)
        uart_xmitchars(dev);

    return IRQ_HANDLED;
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
    
    switch (cmd) {
    case COM_SET_BAUD:
        priv->baud = arg;
        break;
    case COM_SET_PARITY:
        priv->lcr.sep.parity = arg;
        break;
    case COM_SET_STOPBITS:
        priv->lcr.sep.stopbits = arg;
        break;
    case COM_SET_BITS:
        priv->lcr.sep.bits = arg;
        break;
    default:
        return ERROR;
    }

    if (up_setup(dev) != OK)
        return ERROR;

    up_rxint(dev, 1);

    return OK;
}

/****************************************************************************
 * Name: up_receive
 *
 * Description:
 * Called (usually) from the interrupt level to receive one character from
 * the UART.  Error bits associated with the receipt are provided in the
 * the return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, unsigned int *status)
{
    struct up_dev_s *priv = (struct up_dev_s*)dev->priv;
    uint16_t base = priv->base;

    return inb(base+COM_RX);
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
    uint16_t base = priv->base;
    uint8_t ier;

    ier = inb(base+COM_IER);
    if (enable)
        ier |= COM_IER_RDI;
    else
        ier &= ~COM_IER_RDI;
    outb(base+COM_IER, ier);
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
    uint16_t base = priv->base;

    return inb(base+COM_LSR) & COM_LSR_DATA;
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
    uint16_t base = priv->base;

    outb(base+COM_TX, ch);
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
    uint16_t base = priv->base;
    irqstate_t flags;
    uint8_t ier;

    flags = irqsave();
    ier = inb(base+COM_IER);
    if (enable) {
        ier |= COM_IER_TEI;
        outb(base+COM_IER, ier);

        /* Fake a TX interrupt here by just calling uart_xmitchars() with
         * interrupts disabled (note this may recurse).
         */

        uart_xmitchars(dev);
    }
    else {
        ier &= ~COM_IER_TEI;
        outb(base+COM_IER, ier);
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
    uint16_t base = priv->base;

    return inb(base+COM_LSR) & COM_LSR_ETR;
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
    uint16_t base = priv->base;

    return inb(base+COM_LSR) & COM_LSR_EDR;
}

/****************************************************************************
 * Public Funtions
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
    uart_dev_t *dev;
    int err;

#ifdef CONFIG_COM1
    dev = up_alloc_com(COM1, 4);
    if (dev == NULL)
        dbg("alloc com1 fail\n");
    else {
        err = uart_register("/dev/ttyS0", dev);
        if (err)
            dbg("register com1 fail\n");
    }
#endif
#ifdef CONFIG_COM2
    dev = up_alloc_com(COM2, 3);
    if (dev == NULL)
        dbg("alloc com2 fail\n");
    else {
        err = uart_register("/dev/ttyS1", dev);
        if (err)
            dbg("register com2 fail\n");
    }
#endif
#ifdef CONFIG_COM3
    dev = up_alloc_com(COM3, 4);
    if (dev == NULL)
        dbg("alloc com3 fail\n");
    else {
        err = uart_register("/dev/ttyS2", dev);
        if (err)
            dbg("register com3 fail\n");
    }
#endif
#ifdef CONFIG_COM4
    dev = up_alloc_com(COM4, 3);
    if (dev == NULL)
        dbg("alloc com4 fail\n");
    else {
        err = uart_register("/dev/ttyS3", dev);
        if (err)
            dbg("register com4 fail\n");
    }
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
extern void cons_putc(int c);

int up_putc(int ch)
{
    cons_putc(ch);
    return ch;
}

