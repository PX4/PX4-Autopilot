/*******************************************************************************
 * arch/arm/src/stm32/stm32_otgfshost.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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
 *******************************************************************************/

/*******************************************************************************
 * Included Files
 *******************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbhost.h>

#include <arch/irq.h>

#include "chip.h"             /* Includes default GPIO settings */
#include <arch/board/board.h> /* May redefine GPIO settings */

#include "up_arch.h"
#include "up_internal.h"

#include "stm32_otgfs.h"

#if defined(CONFIG_USBHOST) && defined(CONFIG_STM32_OTGFS)

/*******************************************************************************
 * Definitions
 *******************************************************************************/

/* Configuration ***************************************************************/
/* Pre-requistites (partial) */

#ifndef CONFIG_STM32_SYSCFG
#  error "CONFIG_STM32_SYSCFG is required"
#endif

/* Default RX buffer size */

#ifndef CONFIG_STM32_OTGFS_RXBUFSIZE
#  define CONFIG_STM32_OTGFS_RXBUFSIZE 512
#endif

/* Default RxFIFO size */

#ifndef CONFIG_STM32_OTGFS_RXFIFO_SIZE
#  define CONFIG_STM32_OTGFS_RXFIFO_SIZE 128
#endif

/* Default host non-periodic transmit FIFO size */

#ifndef CONFIG_STM32_OTGFS_RXFIFO_SIZE
#  define CONFIG_STM32_OTGFS_RXFIFO_SIZE 128
#endif

/* Default host non-periodic transmit FIFO size */

#ifndef CONFIG_STM32_OTGFS_NPTXFIFO_SIZE
#  define CONFIG_STM32_OTGFS_NPTXFIFO_SIZE 96
#endif

/* Default the host periodic Tx fifo size register (HPTXFSIZ) */

#ifndef CONFIG_STM32_OTGFS_PTXFIFO_SIZE
#  define CONFIG_STM32_OTGFS_PTXFIFO_SIZE 96
#endif

/* HCD Setup *******************************************************************/
/* Hardware capabilities */

#define STM32_NHOST_CHANNELS      8  /* Number of host channels */
#define STM32_MAX_PACKET_SIZE     64 /* Full speed max packet size */
#define STM32_EP0_MAX_PACKET_SIZE 64 /* EP0 FS max packet size */
#define STM32_MAX_TX_FIFOS        15 /* Max number of TX FIFOs */

/* Delays **********************************************************************/

#define STM32_READY_DELAY            200000
#define STM32_FLUSH_DELAY            200000

/* Ever-present MIN/MAX macros */

#ifndef MIN
#  define  MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#  define  MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

/*******************************************************************************
 * Private Types
 *******************************************************************************/
/* USB Speeds */

enum stm32_usbspeed_e
{
  USBSPEED_LOW = 0, /* USB low speed */
  USBSPEED_FULL,    /* USB full speed (default) */
  USBSPEED_HIGH     /* USB high speed (not supported by OTG FS) */
};

/* The following enumeration represents the various states of the USB host
 * state machine
 */

enum stm32_smstate_e
{
  SMSTATE_IDLE = 0,      /* Not attached to a device (initial state) */
  SMSTATE_ATTACHED,      /* Attached to a device */
  SMSTATE_DETACHED,      /* Detached from a device */
  SMSTATE_ENUMERATION,   /* Attached, enumerating */
  SMSTATE_CLASS_REQUEST, /* Enumeration complete, class bound */
  SMSTATE_CLASS,         /* Process class standard control requests */
  SMSTATE_CTRLXFER,      /* Process control transfer */
  SMSTATE_ERROR          /* An irrecoverable error occurred */
};

/* This enumeration represents the state of one TX channel */

enum stm32_chstate_e
{
  CHSTATE_IDLE = 0,      /* Inactive (initial state) */
  CHSTATE_XFRC,          /* Transfer complete */
  CHSTATE_NAK,           /* NAK received */
  CHSTATE_NYET,          /* NotYet received */
  CHSTATE_STALL,         /* Endpoint stalled */
  CHSTATE_TXERR,         /* Transfer error received */
  CHSTATE_DTERR          /* Data error received */
};

/* This enumeration describes the state of the transfer on one TX channel */

enum stm32_rqstate_e
{
  RQSTATE_IDLE = 0,      /* No request in progress (initial state) */
  RQSTATE_DONE,          /* Request complete */
  RQSTATE_NOTREADY,      /* Channel not ready */
  RQSTATE_ERROR,         /* An error occurred */
  RQSTATE_STALL          /* Endpoint is stalled */
};

/* This structure retains the state of one host channel */

struct stn32_chan_s
{
  sem_t             chansem;   /* Channel wait semaphore */
  volatile uint8_t  chstate;   /* See enum stm32_chstate_e */
  volatile uint8_t  rqstate;   /* See enum stm32_rqstate_e */
  uint8_t           devaddr;   /* Device address */
  uint8_t           epno;      /* Device endpoint number */
  uint8_t           speed;     /* See enum stm32_usbspeed_e */
  uint8_t           eptype;    /* See OTGFS_EPTYPE_* definitions */
  uint8_t           pid;       /* Data PID */
  uint8_t           intoggle;  /* IN data toggle */
  uint8_t           outtoggle; /* OUT data toggle */
  bool              isin;      /* True: IN endpoint */
  volatile bool     chanwait;  /* True: Thread is waiting for a channel event */
  volatile uint16_t nerrors;   /* Number of errors detecgted */
  volatile uint16_t xfrd;      /* Number of bytes transferred */
  uint16_t          channel;   /* Encoded channel info */
  uint16_t          maxpacket; /* Max packet size */
  uint16_t          buflen;    /* Buffer length (remaining) */
  uint16_t          xfrlen;    /* Number of bytes transferrred */
  uint8_t          *buffer;    /* Transfer buffer pointer */
};

/* This structure retains the state of the USB host controller */

struct stm32_usbhost_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbhost_s
   * to structstm32_usbhost_s.
   */

  struct usbhost_driver_s drvr;

  /* The bound device class driver */

  struct usbhost_class_s *class;

  /* Overall driver status */

  uint8_t           smstate;   /* The state of the USB host state machine */
  uint8_t           smprev;    /* Preioius USB host state machine state */
  volatile bool     connected; /* Connected to device */
  volatile bool     eventwait; /* True: Thread is waiting for a port event */
  sem_t             exclsem;   /* Support mutually exclusive access */
  sem_t             eventsem;  /* Semaphore to wait for a port event */

  /* The state of each host channel */

  struct stn32_chan_s chan[STM32_MAX_TX_FIFOS];

  /* The RX buffer */

  uint8_t rxbuffer[CONFIG_STM32_OTGFS_RXBUFSIZE];
};

/*******************************************************************************
 * Private Function Prototypes
 *******************************************************************************/

/* Register operations ********************************************************/

#ifdef CONFIG_STM32_USBHOST_REGDEBUG
static void stm32_printreg(uint32_t addr, uint32_t val, bool iswrite);
static void stm32_checkreg(uint32_t addr, uint32_t val, bool iswrite);
static uint32_t stm32_getreg(uint32_t addr);
static void stm32_putreg(uint32_t addr, uint32_t value);
#else
# define stm32_getreg(addr)     getreg32(addr)
# define stm32_putreg(val,addr) putreg32(val,addr)
#endif

static inline void stm32_modifyreg(uint32_t addr, uint32_t clrbits,
                                   uint32_t setbits);

/* Semaphores ******************************************************************/

static void stm32_takesem(sem_t *sem);
#define stm32_givesem(s) sem_post(s);

/* Byte stream access helper functions *****************************************/

static inline uint16_t stm32_getle16(const uint8_t *val);
static void stm32_putle16(uint8_t *dest, uint16_t val);

/* ED list helper functions ****************************************************/

#if !defined(CONFIG_USBHOST_INT_DISABLE) || !defined(CONFIG_USBHOST_ISOC_DISABLE)
static unsigned int stm32_getinterval(uint8_t interval);
static void stm32_setinttab(uint32_t value, unsigned int interval, unsigned int offset);
#endif

/* Interrupt handling **********************************************************/
/* Third level interrupt handlers */

static inline void stm32_gint_hcinisr(FAR struct stm32_usbhost_s *priv,
                                      int chidx);
static inline void stm32_gint_hcoutisr(FAR struct stm32_usbhost_s *priv,
                                       int chidx);

/* Second level interrupt handlers */

#ifdef CONFIG_STM32_OTGFS_SOFINTR
static inline void stm32_gint_sofisr(FAR struct stm32_usbhost_s *priv);
#endif
static inline void stm32_gint_rxflvlisr(FAR struct stm32_usbhost_s *priv);
static inline void stm32_gint_nptxfeisr(FAR struct stm32_usbhost_s *priv);
static inline void stm32_gint_ptxfeisr(FAR struct stm32_usbhost_s *priv);
static inline void stm32_gint_hcisr(FAR struct stm32_usbhost_s *priv);
static inline void stm32_gint_hprtisr(FAR struct stm32_usbhost_s *priv);
static inline void stm32_gint_discisr(FAR struct stm32_usbhost_s *priv);
static inline void stm32_gint_iisooxfrisr(FAR struct stm32_usbhost_s *priv);

/* First level, global interrupt handler */

static int stm32_gint_isr(int irq, FAR void *context);

/* Interrupt controls */

static void stm32_gint_enable(void);
static void stm32_gint_disable(void);
static inline int stm32_hostinit_enable(void);

/* USB host controller operations **********************************************/

static int stm32_wait(FAR struct usbhost_driver_s *drvr, bool connected);
static int stm32_enumerate(FAR struct usbhost_driver_s *drvr);
static int stm32_ep0configure(FAR struct usbhost_driver_s *drvr, uint8_t funcaddr,
                              uint16_t maxpacketsize);
static int stm32_epalloc(FAR struct usbhost_driver_s *drvr,
                         const FAR struct usbhost_epdesc_s *epdesc, usbhost_ep_t *ep);
static int stm32_epfree(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep);
static int stm32_alloc(FAR struct usbhost_driver_s *drvr,
                       FAR uint8_t **buffer, FAR size_t *maxlen);
static int stm32_free(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer);
static int stm32_ioalloc(FAR struct usbhost_driver_s *drvr,
                         FAR uint8_t **buffer, size_t buflen);
static int stm32_iofree(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer);
static int stm32_ctrlin(FAR struct usbhost_driver_s *drvr,
                        FAR const struct usb_ctrlreq_s *req,
                        FAR uint8_t *buffer);
static int stm32_ctrlout(FAR struct usbhost_driver_s *drvr,
                         FAR const struct usb_ctrlreq_s *req,
                         FAR const uint8_t *buffer);
static int stm32_transfer(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                          FAR uint8_t *buffer, size_t buflen);
static void stm32_disconnect(FAR struct usbhost_driver_s *drvr);

/* Initialization **************************************************************/

static inline void stm32_ep0init(FAR struct stm32_usbhost_s *priv);
static void stm32_portreset(FAR struct stm32_usbhost_s *priv);
static inline void stm32_flush_txfifos(uint32_t txfnum);
static inline void stm32_flush_rxfifo(void);
static void stm32_host_initialize(FAR struct stm32_usbhost_s *priv);
static inline void stm32_sw_initialize(FAR struct stm32_usbhost_s *priv);
static inline int stm32_hw_initialize(FAR struct stm32_usbhost_s *priv);

/*******************************************************************************
 * Private Data
 *******************************************************************************/

/* In this driver implementation, support is provided for only a single a single
 * USB device.  All status information can be simply retained in a single global
 * instance.
 */

static struct stm32_usbhost_s g_usbhost =
{
  .drvr             =
    {
      .wait         = stm32_wait,
      .enumerate    = stm32_enumerate,
      .ep0configure = stm32_ep0configure,
      .epalloc      = stm32_epalloc,
      .epfree       = stm32_epfree,
      .alloc        = stm32_alloc,
      .free         = stm32_free,
      .ioalloc      = stm32_ioalloc,
      .iofree       = stm32_iofree,
      .ctrlin       = stm32_ctrlin,
      .ctrlout      = stm32_ctrlout,
      .transfer     = stm32_transfer,
      .disconnect   = stm32_disconnect,
    },
  .class            = NULL,
};

/* This is a free list of EDs and TD buffers */

static struct stm32_list_s *g_edfree; /* List of unused EDs */
static struct stm32_list_s *g_tdfree; /* List of unused TDs */
static struct stm32_list_s *g_tbfree; /* List of unused transfer buffers */
#if STM32_IOBUFFERS > 0
static struct stm32_list_s *g_iofree; /* List of unused I/O buffers */
#endif

/*******************************************************************************
 * Public Data
 *******************************************************************************/

/*******************************************************************************
 * Private Functions
 *******************************************************************************/

/*******************************************************************************
 * Name: stm32_printreg
 *
 * Description:
 *   Print the contents of an STM32xx register operation
 *
 *******************************************************************************/

#ifdef CONFIG_STM32_USBHOST_REGDEBUG
static void stm32_printreg(uint32_t addr, uint32_t val, bool iswrite)
{
  lldbg("%08x%s%08x\n", addr, iswrite ? "<-" : "->", val);
}
#endif

/*******************************************************************************
 * Name: stm32_checkreg
 *
 * Description:
 *   Get the contents of an STM32 register
 *
 *******************************************************************************/

#ifdef CONFIG_STM32_USBHOST_REGDEBUG
static void stm32_checkreg(uint32_t addr, uint32_t val, bool iswrite)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval = 0;
  static uint32_t count = 0;
  static bool     prevwrite = false;

  /* Is this the same value that we read from/wrote to the same register last time?
   * Are we polling the register?  If so, suppress the output.
   */

  if (addr == prevaddr && val == preval && prevwrite == iswrite)
    {
      /* Yes.. Just increment the count */

      count++;
    }
  else
    {
      /* No this is a new address or value or operation. Were there any
       * duplicate accesses before this one?
       */

      if (count > 0)
        {
          /* Yes.. Just one? */

          if (count == 1)
            {
              /* Yes.. Just one */

              stm32_printreg(prevaddr, preval, prevwrite);
            }
          else
            {
              /* No.. More than one. */

              lldbg("[repeats %d more times]\n", count);
            }
        }

      /* Save the new address, value, count, and operation for next time */

      prevaddr  = addr;
      preval    = val;
      count     = 0;
      prevwrite = iswrite;

      /* Show the new regisgter access */

      stm32_printreg(addr, val, iswrite);
    }
}
#endif

/*******************************************************************************
 * Name: stm32_getreg
 *
 * Description:
 *   Get the contents of an STM32 register
 *
 *******************************************************************************/

#ifdef CONFIG_STM32_USBHOST_REGDEBUG
static uint32_t stm32_getreg(uint32_t addr)
{
  /* Read the value from the register */

  uint32_t val = getreg32(addr);

  /* Check if we need to print this value */

  stm32_checkreg(addr, val, false);
  return val;
}
#endif

/*******************************************************************************
 * Name: stm32_putreg
 *
 * Description:
 *   Set the contents of an STM32 register to a value
 *
 *******************************************************************************/

#ifdef CONFIG_STM32_USBHOST_REGDEBUG
static void stm32_putreg(uint32_t val, uint32_t addr)
{
  /* Check if we need to print this value */

  stm32_checkreg(addr, val, true);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/*******************************************************************************
 * Name: stm32_modifyreg
 *
 * Description:
 *   Modify selected bits of an STM32 register.
 *
 *******************************************************************************/

static inline void stm32_modifyreg(uint32_t addr, uint32_t clrbits, uint32_t setbits)
{
  stm32_putreg(addr, (((stm32_getreg(addr)) & ~clrbits) | setbits));
}

/****************************************************************************
 * Name: stm32_takesem
 *
 * Description:
 *   This is just a wrapper to handle the annoying behavior of semaphore
 *   waits that return due to the receipt of a signal.
 *
 *******************************************************************************/

static void stm32_takesem(sem_t *sem)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(sem) != 0)
    {
      /* The only case that an error should occr here is if the wait was
       * awakened by a signal.
       */

      ASSERT(errno == EINTR);
    }
}

/****************************************************************************
 * Name: stm32_getle16
 *
 * Description:
 *   Get a (possibly unaligned) 16-bit little endian value.
 *
 *******************************************************************************/

static inline uint16_t stm32_getle16(const uint8_t *val)
{
  return (uint16_t)val[1] << 8 | (uint16_t)val[0];
}

/****************************************************************************
 * Name: stm32_putle16
 *
 * Description:
 *   Put a (possibly unaligned) 16-bit little endian value.
 *
 *******************************************************************************/

static void stm32_putle16(uint8_t *dest, uint16_t val)
{
  dest[0] = val & 0xff; /* Little endian means LS byte first in byte stream */
  dest[1] = val >> 8;
}

/*******************************************************************************
 * Name: stm32_getinterval
 *
 * Description:
 *   Convert the endpoint polling interval into a HCCA table increment
 *
 *******************************************************************************/

#if !defined(CONFIG_USBHOST_INT_DISABLE) || !defined(CONFIG_USBHOST_ISOC_DISABLE)
static unsigned int stm32_getinterval(uint8_t interval)
{
  /* The bInterval field of the endpoint descriptor contains the polling interval
   * for interrupt and isochronous endpoints. For other types of endpoint, this
   * value should be ignored. bInterval is provided in units of 1MS frames.
   */

  if (interval < 3)
    {
      return 2;
    }
  else if (interval < 7)
    {
      return 4;
    }
  else if (interval < 15)
    {
      return 8;
    }
  else if (interval < 31)
    {
      return 16;
    }
  else
    {
      return 32;
    }
}
#endif

/*******************************************************************************
 * Name: stm32_setinttab
 *
 * Description:
 *   Set the interrupt table to the selected value using the provided interval
 *   and offset.
 *
 *******************************************************************************/

#if !defined(CONFIG_USBHOST_INT_DISABLE) || !defined(CONFIG_USBHOST_ISOC_DISABLE)
static void stm32_setinttab(uint32_t value, unsigned int interval, unsigned int offset)
{
  unsigned int i;
  for (i = offset; i < HCCA_INTTBL_WSIZE; i += interval)
    {
      HCCA->inttbl[i] = value;
    }
}
#endif

/*******************************************************************************
 * Name: stm32_chanwait
 *
 * Description:
 *   Set the request for the Writeback Done Head event well BEFORE enabling the
 *   transfer (as soon as we are absolutely committed to the to avoid transfer).
 *   We do this to minimize race conditions.  This logic would have to be expanded
 *   if we want to have more than one packet in flight at a time!
 *
 *******************************************************************************/

static int stm32_chanwait(struct stm32_usbhost_s *priv, struct stn32_chan_s *chan)
{
  irqstate_t flags = irqsave();
  int        ret   = -ENODEV;

  /* Is the device still connected? */

  if (priv->connected)
    {
      /* Yes.. then set chanwait to indicate that we expect to be informed when
       * either (1) the device is disconnected, or (2) the transfer completed.
       */

      chan->chanwait = true;
      ret            = OK;
    }

  irqrestore(flags);
  return ret;
}

/*******************************************************************************
 * Name: stm32_ctrltd
 *
 * Description:
 *   Process a IN or OUT request on the control endpoint.  This function
 *   will enqueue the request and wait for it to complete.  Only one transfer
 *   may be queued; Neither these methods nor the transfer() method can be
 *   called again until the control transfer functions returns.
 *
 *   These are blocking methods; these functions will not return until the
 *   control transfer has completed.
 *
 *******************************************************************************/

static int stm32_ctrltd(struct stm32_usbhost_s *priv, uint32_t dirpid,
                        uint8_t *buffer, size_t buflen)
{
  uint32_t toggle;
  uint32_t regval;
  int ret;

  /* Set the request for the Writeback Done Head event well BEFORE enabling the
   * transfer.
   */

  ret = stm32_chanwait(priv, chan);
  if (ret != OK)
    {
      udbg("ERROR: Device disconnected\n");
      return ret;
    }

  /* Configure the toggle field in the TD */

  if (dirpid == GTD_STATUS_DP_SETUP)
    {
      toggle = GTD_STATUS_T_DATA0;
    }
  else
    {
      toggle = GTD_STATUS_T_DATA1;
    }

  /* Then enqueue the transfer */
#warning "Missing Logic"

  /* And wait for the transfer to complete */

  stm32_takesem(&chan->chansem);

  /* Check the transfer completion status bits */
#warning "Missing Logic"

  /* Make sure that there is no outstanding request on this endpoint */
#warning "Missing Logic"

  return ret;
}

/*******************************************************************************
 * Name: stm32_gint_hcinisr
 *
 * Description:
 *   USB OTG FS host IN channels interrupt handler
 *
 *******************************************************************************/

static inline void stm32_gint_hcinisr(FAR struct stm32_usbhost_s *priv,
                                      int chidx)
{
#warning "Missing logic"
}

/*******************************************************************************
 * Name: stm32_gint_hcoutisr
 *
 * Description:
 *   USB OTG FS host OUT channels interrupt handler
 *
 *******************************************************************************/

static inline void stm32_gint_hcoutisr(FAR struct stm32_usbhost_s *priv,
                                       int chidx)
{
#warning "Missing logic"
}

/*******************************************************************************
 * Name: stm32_gint_sofisr
 *
 * Description:
 *   USB OTG FS start-of-frame interrupt handler
 *
 *******************************************************************************/

#ifdef CONFIG_STM32_OTGFS_SOFINTR
static inline void stm32_gint_sofisr(FAR struct stm32_usbhost_s *priv)
{
  /* Handle SOF interrupt */
#warning "Do what?"

  /* Clear pending SOF interrupt */

  stm32_putreg(STM32_OTGFS_GINTSTS, OTGFS_GINT_SOF);
}
#endif

/*******************************************************************************
 * Name: stm32_gint_rxflvlisr
 *
 * Description:
 *   USB OTG FS RxFIFO non-empty interrupt handler
 *
 *******************************************************************************/

static inline void stm32_gint_rxflvlisr(FAR struct stm32_usbhost_s *priv)
{
  FAR uint32_t *dest;
  uint32_t grxsts;
  uint32_t intmsk;
  uint32_t hcchar;
  uint32_t hctsiz;
  uint32_t fifo;
  int bcnt;
  int bcnt32;
  int chidx;
  int i;

  /* Disable the RxFIFO non-empty interrupt */

  intmsk  = stm32_getreg(STM32_OTGFS_GINTMSK)
  intmsk &= ~OTGFS_GINT_RXFLVL;
  stm32_putreg(STM32_OTGFS_GINTMSK, intmsk);

  /* Read and pop the next status from the Rx FIFO */

  grxsts = stm32_getreg(STM32_OTGFS_GRXSTSP);

  /* Isolate the channel number/index in the status word */

  chidx = (grxsts & OTGFS_GRXSTSH_CHNUM_MASK) >> OTGFS_GRXSTSH_CHNUM_SHIFT;

  /* Get the host channel characteristics register (HCCHAR) for this channel */

  hcchar = stm32_getreg(STM32_OTGFS_HCCHAR(chidx));

  /* The process the interrupt according to the packet status */

  switch (grxsts & OTGFS_GRXSTSH_PKTSTS_MASK)
    {
    case OTGFS_GRXSTSH_PKTSTS_INRECVD: /* IN data packet received */
      {
        /* Read the data into the host buffer. */

        int bcnt = (grxsts & OTGFS_GRXSTSH_BCNT_MASK) >> OTGFS_GRXSTSH_BCNT_SHIFT;

        if (bcnt > 0 && priv->chan[chidx].buffer != NULL)
          {
            /* Transfer the packet from the Rx FIFO into the user buffer */

            FAR uint32_t *dest = (FAR uint32_t *)priv->chan[chidx].buffer;
            uint32_t fifo = STM32_OTGFS_DFIFO_HCH(0);
            uint32_t hctsiz;
            int bcnt32 = (bcnt + 3) >> 2;

            for (i = 0; i < count32b; i++, dest += 4)
              {
                *dest++ = stm32_getreg(fifo);
              }

            /* Manage multiple packet transfers */

            priv->chan[chidx].buffer += bcnt;
            priv->chan[chidx].xfrlen += bcnt;
            priv->chan[chidx].xfrd    = priv->chan[chidx].xfrlen;

            /* Check if more packets are expected */

            hctsiz = stm32_getreg(STM32_OTGFS_HCTSIZ(chidx));
            if (hctsiz.b.pktcnt > 0)
              {
                /* Re-activate the channel when more packets are expected */

                hcchar |= OTGFS_HCCHAR_CHENA;
                hcchar &= ~OTGFS_HCCHAR_CHDIS;
                stm32_putreg(STM32_OTGFS_HCCHAR(chidx), hcchar);
              }
          }
      }
      break;

    case OTGFS_GRXSTSH_PKTSTS_INDONE:  /* IN transfer completed */
    case OTGFS_GRXSTSH_PKTSTS_DTOGERR: /* Data toggle error */
    case OTGFS_GRXSTSH_PKTSTS_HALTED:  /* Channel halted */
    default:
      break;
    }

  /* Re-enable the RxFIFO non-empty interrupt */

  intmsk |= OTGFS_GINT_RXFLVL;
  stm32_putreg(STM32_OTGFS_GINTMSK, intmsk);
}

/*******************************************************************************
 * Name: stm32_gint_nptxfeisr
 *
 * Description:
 *   USB OTG FS non-periodic TxFIFO empty interrupt handler
 *
 *******************************************************************************/

static inline void stm32_gint_nptxfeisr(FAR struct stm32_usbhost_s *priv)
{
#warning "Missing logic"
}

/*******************************************************************************
 * Name: stm32_gint_ptxfeisr
 *
 * Description:
 *   USB OTG FS periodic TxFIFO empty interrupt handler
 *
 *******************************************************************************/

static inline void stm32_gint_ptxfeisr(FAR struct stm32_usbhost_s *priv)
{
#warning "Missing logic"
}

/*******************************************************************************
 * Name: stm32_gint_hcisr
 *
 * Description:
 *   USB OTG FS host channels interrupt handler
 *
 *******************************************************************************/

static inline void stm32_gint_hcisr(FAR struct stm32_usbhost_s *priv)
{
  uint32_t haint;
  uint32_t hcchar;
  int i = 0;

  /* Read the Host all channels interrupt register and test each bit in the
   * register. Each bit i, i=0...(STM32_NHOST_CHANNELS-1), corresponds to
   * a pending interrupt on channel i.
   */

  haint = stm32_getreg(STM32_OTGFS_HAINT);
  for (i = 0; i < STM32_NHOST_CHANNELS; i++)
    {
      /* Is an interrupt pending on this channel? */

      if ((haint & OTGFS_HAINT(i)) != 0)
        {
          /* Yes... read the HCCHAR register to get the direction bit */

          hcchar = stm32_getreg(STM32_OTGFS_HCCHAR(i));

          /* Was this an interrupt on an IN or an OUT channel? */

          if ((hcchar & OTGFS_HCCHAR_EPDIR) != 0)
            {
              /* Handle the HC IN channel interrupt */

              stm32_gint_hcinisr(priv, i);
            }
          else
            {
              /* Handle the HC OUT channel interrupt */

              stm32_gint_hcoutisr(priv, i);
            }
        }
    }
}

/*******************************************************************************
 * Name: stm32_gint_hprtisr
 *
 * Description:
 *   USB OTG FS host port interrupt handler
 *
 *******************************************************************************/

static inline void stm32_gint_hprtisr(FAR struct stm32_usbhost_s *priv)
{
#warning "Missing logic"
}

/*******************************************************************************
 * Name: stm32_gint_discisr
 *
 * Description:
 *   USB OTG FS disconnect detected interrupt handler
 *
 *******************************************************************************/

static inline void stm32_gint_discisr(FAR struct stm32_usbhost_s *priv)
{
  uint32_t regval;

  /* Were we previously connected? */

  if !priv->connected)
    {
      /* Yes.. then we no longer connected */
 
      ullvdbg("Disconnected\n");
      priv->connected = false;

      /* Are we bound to a class driver? */

      if (priv->class)
        {
          /* Yes.. Disconnect the class driver */

          CLASS_DISCONNECTED(priv->class);
          priv->class = NULL;
        }

      /* Notify any waiters that there is a change in the connection state */

     if (priv->eventwait)
        {
          stm32_givesem(&priv->eventsem);
          priv->eventwait = false;
        }
    }

  /* Clear the dicsonnect interrupt */

  stm32_putreg(STM32_OTGFS_GINTSTS, OTGFS_GINT_DISC);
}

/*******************************************************************************
 * Name: stm32_gint_iisooxfrisr
 *
 * Description:
 *   USB OTG FS incomplete isochronous interrupt handler
 *
 *******************************************************************************/

static inline void stm32_gint_iisooxfrisr(FAR struct stm32_usbhost_s *priv)
{
  uint32_t regval;

  /* CHENA : Set to enable the channel
   * CHDIS : Set to stop transmitting/receiving data on a channel
   */
 
  regval = stm32_getreg(STM32_OTGFS_HCCHAR(0));
  regval |= (OTGFS_HCCHAR_CHDIS | OTGFS_HCCHAR_CHENA);
  stm32_putreg(STM32_OTGFS_HCCHAR(0), regval);

  /* Clear the incomplete isochronous OUT interrupt */

  stm32_putreg(STM32_OTGFS_GINTSTS, OTGFS_GINT_IISOOXFR);
}

/*******************************************************************************
 * Name: stm32_gint_isr
 *
 * Description:
 *   USB OTG FS global interrupt handler
 *
 *******************************************************************************/

static int stm32_gint_isr(int irq, FAR void *context)
{
  /* At present, there is only support for a single OTG FS host. Hence it is
   * pre-allocated as g_usbhost.  However, in most code, the private data
   * structure will be referenced using the 'priv' pointer (rather than the
   * global data) in order to simplify any future support for multiple devices.
   */

  FAR struct stm32_usbhost_s *priv = &g_usbhost;
  uint32_t pending;

  /* If OTG were supported, we would need to check if we are in host or
   * device mode when the global interrupt occurs.  Here we support only
   * host mode
   */

  /* Get the unmasked bits in the GINT status */

  pending = stm32_getreg(STM32_OTGFS_GINTSTS);
  pending &= stm32_getreg(STM32_OTGFS_GINTMSK);

  /* The process each pending, unmasked GINT interrupts */

  if (pending != 0)
    {
      ullvdbg("GINTSTS: %08x\n", pending);

      /* Handle the start of frame interrupt */

#ifdef CONFIG_STM32_OTGFS_SOFINTR
      if ((pending & OTGFS_GINT_SOF) != 0)
        {
          stm32_gint_sofisr(priv);
        }
#endif

      /* Handle the RxFIFO non-empty interrupt */

      if ((pending & OTGFS_GINT_RXFLVL)) != 0)
        {
          stm32_gint_rxflvlisr(priv);
        }

      /* Handle the non-periodic TxFIFO empty interrupt */

      if ((pending & OTGFS_GINT_NPTXFE)) != 0)
        {
          stm32_gint_nptxfeisr(priv);
        }

      /* Handle the periodic TxFIFO empty interrupt */

      if ((pending & OTGFS_GINT_PTXFE)) != 0)
        {
          stm32_gint_ptxfeisr(priv);
        }

      /* Handle the host channels interrupt */

      if ((pending & OTGFS_GINT_HC)) != 0)
        {
          stm32_gint_hcisr(priv);
        }

      /* Handle the host port interrupt */

      if ((pending & OTGFS_GINT_HPRT)) != 0)
        {
          stm32_gint_hprtisr(priv);
        }

      /* Handle the disconnect detected interrupt */

      if ((pending & OTGFS_GINT_DISC)) != 0)
        {
          stm32_gint_discisr(priv);
        }

      /* Handle the incomplete isochronous OUT transfer */

      if ((pending & OTGFS_GINT_IISOOXFR)) != 0)
        {
          stm32_gint_iisooxfrisr(priv);
        }
    }

  return OK;
}

/*******************************************************************************
 * Name: stm32_gint_enable and stm32_gint_disable
 *
 * Description:
 *   Respectively enable or disable the global OTG FS interrupt.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 *******************************************************************************/

static void stm32_gint_enable(void)
{
  uint32_t regval;

  /* Set the GINTMSK bit to unmask the interrupt */

  regval  = stm32_getreg(STM32_OTGFS_GAHBCFG)
  regval |= OTGFS_GAHBCFG_GINTMSK;
  stm32_putreg(OTGFS_GAHBCFG_GINTMSK, regval);
}

static void stm32_gint_disable(void)
{
  uint32_t regval;

  /* Clear the GINTMSK bit to mask the interrupt */

  regval  = stm32_getreg(STM32_OTGFS_GAHBCFG)
  regval &= ~OTGFS_GAHBCFG_GINTMSK;
  stm32_putreg(OTGFS_GAHBCFG_GINTMSK, regval);
}

/*******************************************************************************
 * Name: stm32_hostinit_enable
 *
 * Description:
 *   Enable host interrupts.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 *******************************************************************************/

static inline void stm32_hostinit_enable(void)
{
  /* Disable all interrupts. */

  stm32_putreg(STM32_OTGFS_GINTMSK, 0);

  /* Clear any pending interrupts. */

  stm32_putreg(STM32_OTGFS_GINTSTS, 0xffffffff);

  /* Clear any pending USB OTG Interrupts (should be done elsewhere if OTG is supported) */

  stm32_putreg(STM32_OTGFS_GOTGINT, 0xffffffff);

  /* Clear any pending USB OTG interrupts */

  stm32_putreg(STM32_OTGFS_GINTSTS, 0xbfffffff);

  /* Enable the host interrupts */
  /* Common interrupts:
   *
   *   OTGFS_GINT_WKUP     : Resume/remote wakeup detected interrupt
   *   OTGFS_GINT_USBSUSP  : USB suspend
   */

  regval = (OTGFS_GINT_WKUP | OTGFS_GINT_USBSUSP);

  /* If OTG were supported, we would need to enable the following as well:
   *
   *   OTGFS_GINT_OTG      : OTG interrupt
   *   OTGFS_GINT_SRQ      : Session request/new session detected interrupt
   *   OTGFS_GINT_CIDSCHG  : Connector ID status change
   */

  /* Host-specific interrupts
   *
   *   OTGFS_GINT_SOF      : Start of frame
   *   OTGFS_GINT_RXFLVL   : RxFIFO non-empty
   *   OTGFS_GINT_IISOOXFR : Incomplete isochronous OUT transfer
   *   OTGFS_GINT_HPRT     : Host port interrupt
   *   OTGFS_GINT_HC       : Host channels interrupt
   *   OTGFS_GINT_DISC     : Disconnect detected interrupt
   */

#ifdef CONFIG_STM32_OTGFS_SOFINTR
  regval |= (OTGFS_GINT_SOF    | OTGFS_GINT_RXFLVL   | OTGFS_GINT_IISOOXFR |
             OTGFS_GINT_HPRT   | OTGFS_GINT_HC       | OTGFS_GINT_DISC);
#else
  regval |= (OTGFS_GINT_RXFLVL | OTGFS_GINT_IISOOXFR | OTGFS_GINT_HPRT     |
             OTGFS_GINT_HC     | OTGFS_GINT_DISC);
#endif
  stm32_putreg(STM32_OTGFS_GINTMSK, regval);
}

/*******************************************************************************
 * USB Host Controller Operations
 *******************************************************************************/

/*******************************************************************************
 * Name: stm32_wait
 *
 * Description:
 *   Wait for a device to be connected or disconneced.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   connected - TRUE: Wait for device to be connected; FALSE: wait for device
 *      to be disconnected
 *
 * Returned Values:
 *   Zero (OK) is returned when a device in connected. This function will not
 *   return until either (1) a device is connected or (2) some failure occurs.
 *   On a failure, a negated errno value is returned indicating the nature of
 *   the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static int stm32_wait(FAR struct usbhost_driver_s *drvr, bool connected)
{
  struct stm32_usbhost_s *priv = (struct stm32_usbhost_s *)drvr;
  irqstate_t flags;

  /* Are we already connected? */

  flags = irqsave();
  while (priv->connected == connected)
    {
      /* No... wait for the connection/disconnection */

      priv->eventwait = true;
      stm32_takesem(&priv->eventsem);
    }
  irqrestore(flags);

  udbg("Connected:%s\n", priv->connected ? "YES" : "NO");
  return OK;
}

/*******************************************************************************
 * Name: stm32_enumerate
 *
 * Description:
 *   Enumerate the connected device.  As part of this enumeration process,
 *   the driver will (1) get the device's configuration descriptor, (2)
 *   extract the class ID info from the configuration descriptor, (3) call
 *   usbhost_findclass() to find the class that supports this device, (4)
 *   call the create() method on the struct usbhost_registry_s interface
 *   to get a class instance, and finally (5) call the configdesc() method
 *   of the struct usbhost_class_s interface.  After that, the class is in
 *   charge of the sequence of operations.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Only a single class bound to a single device is supported.
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static int stm32_enumerate(FAR struct usbhost_driver_s *drvr)
{
  struct stm32_usbhost_s *priv = (struct stm32_usbhost_s *)drvr;

  /* Are we connected to a device?  The caller should have called the wait()
   * method first to be assured that a device is connected.
   */

  while (!priv->connected)
    {
      /* No, return an error */

      udbg("Not connected\n");
      return -ENODEV;
    }
 
  /* USB 2.0 spec says at least 50ms delay before port reset */

  up_mdelay(100);

  /* Put RH port 1 in reset (the STM32 supports only a single downstream port) */
#warning "Missing Logic"

  /* Wait for the port reset to complete */

  while ((stm32_getreg(???) & ???) != 0);

  /* Release RH port 1 from reset and wait a bit */
#warning "Missing Logic"
  up_mdelay(200);

  /* Let the common usbhost_enumerate do all of the real work.  Note that the
   * FunctionAddress (USB address) is hardcoded to one.
   */

  uvdbg("Enumerate the device\n");
  return usbhost_enumerate(drvr, 1, &priv->class);
}

/************************************************************************************
 * Name: stm32_ep0configure
 *
 * Description:
 *   Configure endpoint 0.  This method is normally used internally by the
 *   enumerate() method but is made available at the interface to support
 *   an external implementation of the enumeration logic.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   funcaddr - The USB address of the function containing the endpoint that EP0
 *     controls
 *   maxpacketsize - The maximum number of bytes that can be sent to or
 *    received from the endpoint in a single data packet
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int stm32_ep0configure(FAR struct usbhost_driver_s *drvr, uint8_t funcaddr,
                              uint16_t maxpacketsize)
{
  FAR struct stm32_usbhost_s *priv = (FAR struct stm32_usbhost_s *)drvr;

  DEBUGASSERT(drvr && funcaddr < 128 && maxpacketsize < 2048);

  /* We must have exclusive access to the USB host hardware and state structures */

  stm32_takesem(&priv->exclsem);
#warning "Missing logic"
  stm32_givesem(&priv->exclsem);

  return OK;
}

/************************************************************************************
 * Name: stm32_epalloc
 *
 * Description:
 *   Allocate and configure one endpoint.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   epdesc - Describes the endpoint to be allocated.
 *   ep - A memory location provided by the caller in which to receive the
 *      allocated endpoint desciptor.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int stm32_epalloc(FAR struct usbhost_driver_s *drvr,
                         const FAR struct usbhost_epdesc_s *epdesc, usbhost_ep_t *ep)
{
  struct stm32_usbhost_s *priv = (struct stm32_usbhost_s *)drvr;
  int                     ret  = -ENOMEM;

  /* Sanity check.  NOTE that this method should only be called if a device is
   * connected (because we need a valid low speed indication).
   */

  DEBUGASSERT(priv && epdesc && ep && priv->connected);

  /* We must have exclusive access to the USB host hardware and state structures */

  stm32_takesem(&priv->exclsem);

  /* Get the direction of the endpoint */

  if (epdesc->in)
    {
    }
  else
    {
    }

  /* Set the transfer type */

  /* Special Case isochronous transfer types */

  stm32_givesem(&priv->exclsem);
  return ret;
}

/************************************************************************************
 * Name: stm32_epfree
 *
 * Description:
 *   Free and endpoint previously allocated by DRVR_EPALLOC.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ep - The endpint to be freed.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int stm32_epfree(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep)
{
  struct stm32_usbhost_s *priv = (struct stm32_usbhost_s *)drvr;
  int                     ret;

  /* There should not be any pending, real TDs linked to this ED */

  DEBUGASSERT(drvr);

  /* We must have exclusive access to the USB host hardware and state structures */

  stm32_takesem(&priv->exclsem);
#warning "Missing logic"

  stm32_givesem(&priv->exclsem);
  return ret;
}

/*******************************************************************************
 * Name: stm32_alloc
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor data can
 *   be accessed more efficiently.  This method provides a mechanism to allocate
 *   the request/descriptor memory.  If the underlying hardware does not support
 *   such "special" memory, this functions may simply map to malloc.
 *
 *   This interface was optimized under a particular assumption.  It was assumed
 *   that the driver maintains a pool of small, pre-allocated buffers for descriptor
 *   traffic.  NOTE that size is not an input, but an output:  The size of the
 *   pre-allocated buffer is returned.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   buffer - The address of a memory location provided by the caller in which to
 *     return the allocated buffer memory address.
 *   maxlen - The address of a memory location provided by the caller in which to
 *     return the maximum size of the allocated buffer memory.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static int stm32_alloc(FAR struct usbhost_driver_s *drvr,
                       FAR uint8_t **buffer, FAR size_t *maxlen)
{
  struct stm32_usbhost_s *priv = (struct stm32_usbhost_s *)drvr;
  int ret = -ENOMEM;

  DEBUGASSERT(priv && buffer && maxlen);
  /* We must have exclusive access to the USB host hardware and state structures */

  stm32_takesem(&priv->exclsem);
#warning "Missing logic"
  stm32_givesem(&priv->exclsem);
  return ret;
}

/*******************************************************************************
 * Name: stm32_free
 *
 * Description:
 *   Some hardware supports special memory in which request and descriptor data can
 *   be accessed more efficiently.  This method provides a mechanism to free that
 *   request/descriptor memory.  If the underlying hardware does not support
 *   such "special" memory, this functions may simply map to free().
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   buffer - The address of the allocated buffer memory to be freed.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static int stm32_free(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer)
{
  struct stm32_usbhost_s *priv = (struct stm32_usbhost_s *)drvr;

  DEBUGASSERT(drvr && buffer);

  /* We must have exclusive access to the USB host hardware and state structures */

  stm32_takesem(&priv->exclsem);
#warning "Missing logic"
  stm32_givesem(&priv->exclsem);
  return OK;
}

/************************************************************************************
 * Name: stm32_ioalloc
 *
 * Description:
 *   Some hardware supports special memory in which larger IO buffers can
 *   be accessed more efficiently.  This method provides a mechanism to allocate
 *   the request/descriptor memory.  If the underlying hardware does not support
 *   such "special" memory, this functions may simply map to malloc.
 *
 *   This interface differs from DRVR_ALLOC in that the buffers are variable-sized.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   buffer - The address of a memory location provided by the caller in which to
 *     return the allocated buffer memory address.
 *   buflen - The size of the buffer required.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int stm32_ioalloc(FAR struct usbhost_driver_s *drvr,
                         FAR uint8_t **buffer, size_t buflen)
{
  DEBUGASSERT(drvr && buffer);
  return -ENOSYS;
}

/************************************************************************************
 * Name: stm32_iofree
 *
 * Description:
 *   Some hardware supports special memory in which IO data can  be accessed more
 *   efficiently.  This method provides a mechanism to free that IO buffer
 *   memory.  If the underlying hardware does not support such "special" memory,
 *   this functions may simply map to free().
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   buffer - The address of the allocated buffer memory to be freed.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   This function will *not* be called from an interrupt handler.
 *
 ************************************************************************************/

static int stm32_iofree(FAR struct usbhost_driver_s *drvr, FAR uint8_t *buffer)
{
  DEBUGASSERT(drvr && buffer);
  return -ENOSYS;
}

/*******************************************************************************
 * Name: stm32_ctrlin and stm32_ctrlout
 *
 * Description:
 *   Process a IN or OUT request on the control endpoint.  These methods
 *   will enqueue the request and wait for it to complete.  Only one transfer may be
 *   queued; Neither these methods nor the transfer() method can be called again
 *   until the control transfer functions returns.
 *
 *   These are blocking methods; these functions will not return until the
 *   control transfer has completed.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   req - Describes the request to be sent.  This request must lie in memory
 *      created by DRVR_ALLOC.
 *   buffer - A buffer used for sending the request and for returning any
 *     responses.  This buffer must be large enough to hold the length value
 *     in the request description. buffer must have been allocated using DRVR_ALLOC
 *
 *   NOTE: On an IN transaction, req and buffer may refer to the same allocated
 *   memory.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Only a single class bound to a single device is supported.
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static int stm32_ctrlin(FAR struct usbhost_driver_s *drvr,
                        FAR const struct usb_ctrlreq_s *req,
                        FAR uint8_t *buffer)
{
  struct stm32_usbhost_s *priv = (struct stm32_usbhost_s *)drvr;
  uint16_t len;
  int  ret;

  DEBUGASSERT(drvr && req);
  uvdbg("type:%02x req:%02x value:%02x%02x index:%02x%02x len:%02x%02x\n",
        req->type, req->req, req->value[1], req->value[0],
        req->index[1], req->index[0], req->len[1], req->len[0]);

  /* We must have exclusive access to the USB host hardware and state structures */

  stm32_takesem(&priv->exclsem);

  len = stm32_getle16(req->len);
  ret = stm32_ctrltd(priv, GTD_STATUS_DP_SETUP, (uint8_t*)req, USB_SIZEOF_CTRLREQ);
  if (ret == OK)
    {
      if (len)
        {
          ret = stm32_ctrltd(priv, GTD_STATUS_DP_IN, buffer, len);
        }

      if (ret == OK)
        {
          ret = stm32_ctrltd(priv, GTD_STATUS_DP_OUT, NULL, 0);
        }
    }

  stm32_givesem(&priv->exclsem);
  return ret;
}

static int stm32_ctrlout(FAR struct usbhost_driver_s *drvr,
                         FAR const struct usb_ctrlreq_s *req,
                         FAR const uint8_t *buffer)
{
  struct stm32_usbhost_s *priv = (struct stm32_usbhost_s *)drvr;
  uint16_t len;
  int  ret;

  DEBUGASSERT(drvr && req);
  uvdbg("type:%02x req:%02x value:%02x%02x index:%02x%02x len:%02x%02x\n",
        req->type, req->req, req->value[1], req->value[0],
        req->index[1], req->index[0], req->len[1], req->len[0]);

  /* We must have exclusive access to the USB host hardware and state structures */

  stm32_takesem(&priv->exclsem);

  len = stm32_getle16(req->len);
  ret = stm32_ctrltd(priv, GTD_STATUS_DP_SETUP, (uint8_t*)req, USB_SIZEOF_CTRLREQ);
  if (ret == OK)
    {
      if (len)
        {
          ret = stm32_ctrltd(priv, GTD_STATUS_DP_OUT, (uint8_t*)buffer, len);
        }

      if (ret == OK)
        {
          ret = stm32_ctrltd(priv, GTD_STATUS_DP_IN, NULL, 0);
        }
    }

  stm32_givesem(&priv->exclsem);
  return ret;
}

/*******************************************************************************
 * Name: stm32_transfer
 *
 * Description:
 *   Process a request to handle a transfer descriptor.  This method will
 *   enqueue the transfer request and return immediately.  Only one transfer may be
 *   queued; Neither this method nor the ctrlin or ctrlout methods can be called
 *   again until this function returns.
 *
 *   This is a blocking method; this functions will not return until the
 *   transfer has completed.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *   ep - The IN or OUT endpoint descriptor for the device endpoint on which to
 *      perform the transfer.
 *   buffer - A buffer containing the data to be sent (OUT endpoint) or received
 *     (IN endpoint).  buffer must have been allocated using DRVR_ALLOC
 *   buflen - The length of the data to be sent or received.
 *
 * Returned Values:
 *   On success, zero (OK) is returned. On a failure, a negated errno value is
 *   returned indicating the nature of the failure
 *
 * Assumptions:
 *   - Only a single class bound to a single device is supported.
 *   - Called from a single thread so no mutual exclusion is required.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/
 
static int stm32_transfer(FAR struct usbhost_driver_s *drvr, usbhost_ep_t ep,
                          FAR uint8_t *buffer, size_t buflen)
{
  struct stm32_usbhost_s *priv = (struct stm32_usbhost_s *)drvr;
  uint32_t dirpid;
  uint32_t regval;
#if STM32_IOBUFFERS > 0
  uint8_t *origbuf = NULL;
#endif
  bool in;
  int ret;

  DEBUGASSERT(priv && chan && buffer && buflen > 0);

  /* We must have exclusive access to the USB host hardware and state structures */

  stm32_takesem(&priv->exclsem);

  /* Set the request for the Writeback Done Head event well BEFORE enabling the
   * transfer.
   */

  ret = stm32_chanwait(priv, chan);
  if (ret != OK)
    {
      udbg("ERROR: Device disconnected\n");
      goto errout;
    }

  /* Get the direction of the endpoint */

  if (in)
    {
      dirpid    = GTD_STATUS_DP_IN;
    }
  else
    {
      dirpid    = GTD_STATUS_DP_OUT;
    }

  /* Then enqueue the transfer and wait for the transfer to complete */
#warning "Missing logic"

  /* Wait for the transfer to complete */
#warning "Missing Logic"

  stm32_takesem(&chan->chansem);

  /* Check the transfer completion status */
#warning "Missing Logic"

errout:
  /* Make sure that there is no outstanding request on this endpoint */
#warning "Missing Logic"

  stm32_givesem(&priv->exclsem);
  return ret;
}

/*******************************************************************************
 * Name: stm32_disconnect
 *
 * Description:
 *   Called by the class when an error occurs and driver has been disconnected.
 *   The USB host driver should discard the handle to the class instance (it is
 *   stale) and not attempt any further interaction with the class driver instance
 *   (until a new instance is received from the create() method).  The driver
 *   should not called the class' disconnected() method.
 *
 * Input Parameters:
 *   drvr - The USB host driver instance obtained as a parameter from the call to
 *      the class create() method.
 *
 * Returned Values:
 *   None
 *
 * Assumptions:
 *   - Only a single class bound to a single device is supported.
 *   - Never called from an interrupt handler.
 *
 *******************************************************************************/

static void stm32_disconnect(FAR struct usbhost_driver_s *drvr)
{
  struct stm32_usbhost_s *priv = (struct stm32_usbhost_s *)drvr;
  priv->class = NULL;
}
  
/*******************************************************************************
 * Initialization
 *******************************************************************************/
/*******************************************************************************
 * Name: stm32_ep0init
 *
 * Description:
 *   Initialize ED for EP0, add it to the control ED list, and enable control
 *   transfers.
 *
 * Input Parameters:
 *   priv - private driver state instance.
 *
 * Returned Values:
 *   None
 *
 *******************************************************************************/

static inline void stm32_ep0init(struct stm32_usbhost_s *priv)
{
  uint32_t regval;

  /* Set up some default values */

  (void)stm32_ep0configure(&priv->drvr, 1, 8);

#warning "Missing logic"
}

/*******************************************************************************
 * Name: stm32_portreset
 *
 * Description:
 *   Reset the USB host port.
 *
 *   NOTE: "Before starting to drive a USB reset, the application waits for the
 *   OTG interrupt triggered by the debounce done bit (DBCDNE bit in
 *   OTG_FS_GOTGINT), which indicates that the bus is stable again after the
 *   electrical debounce caused by the attachment of a pull-up resistor on DP
 *   (FS) or DM (LS).
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   None
 *
 *******************************************************************************/

static void stm32_portreset(FAR struct stm32_usbhost_s *priv)
{
  uint32_t regval;

  regval  = stm32_getreg(STM32_OTGFS_HPRT);
  retval &= ~(OTGFS_HPRT_PENA|OTGFS_HPRT_PCDET|OTGFS_HPRT_PENCHNG|OTGFS_HPRT_POCCHNG);
  regval |= OTGFS_HPRT_PRST;
  stm32_putreg(STM32_OTGFS_HPRT, regval);

  up_mdelay(10);

  regval &= ~OTGFS_HPRT_PRST;
  stm32_putreg(STM32_OTGFS_HPRT, regval);

  up_mdelay(20);
}

/*******************************************************************************
 * Name: stm32_flush_txfifos
 *
 * Description:
 *   Flush the selected Tx FIFO.
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   None.
 *
 *******************************************************************************/

static inline void stm32_flush_txfifos(uint32_t txfnum)
{
  uint32_t regval;
  uint32_t timeout;

  /* Initiate the TX FIFO flush operation */

  regval = OTGFS_GRSTCTL_TXFFLSH | txfnum;
  stm32_putreg(regval, STM32_OTGFS_GRSTCTL);

  /* Wait for the FLUSH to complete */

  for (timeout = 0; timeout < STM32_FLUSH_DELAY; timeout++)
    {
      regval = stm32_getreg(STM32_OTGFS_GRSTCTL);
      if ((regval & OTGFS_GRSTCTL_TXFFLSH) == 0)
        {
          break;
        }
    }

  /* Wait for 3 PHY Clocks */

  up_udelay(3);
}

/*******************************************************************************
 * Name: stm32_flush_rxfifo
 *
 * Description:
 *   Flush the Rx FIFO.
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   None.
 *
 *******************************************************************************/

static inline void stm32_flush_rxfifo(void)
{
  uint32_t regval;
  uint32_t timeout;

  /* Initiate the RX FIFO flush operation */

  stm32_putreg(OTGFS_GRSTCTL_RXFFLSH, STM32_OTGFS_GRSTCTL);

  /* Wait for the FLUSH to complete */

  for (timeout = 0; timeout < STM32_FLUSH_DELAY; timeout++)
    {
      regval = stm32_getreg(STM32_OTGFS_GRSTCTL);
      if ((regval & OTGFS_GRSTCTL_RXFFLSH) == 0)
        {
          break;
        }
    }

  /* Wait for 3 PHY Clocks */

  up_udelay(3);
}

/*******************************************************************************
 * Name: stm32_host_initialize
 *
 * Description:
 *   Initialize/re-initialize hardware for host mode operation.  At present,
 *   this function is called only from stm32_hw_initialize().  But if OTG mode
 *   were supported, this function would also be called to swtich between
 *   host and device modes on a connector ID change interrupt.
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   None.
 *
 *******************************************************************************/

static void stm32_host_initialize(FAR struct stm32_usbhost_s *priv)
{
  uint32_t regval;
  uint32_t offset;
  int ret;
  int i;

  /* Restart the PHY Clock */

  stm32_putreg(STM32_OTGFS_PCGCCTL, 0);

  /* Initialize Host Configuration (HCFG) register */

  regval  = stm32_getreg(STM32_OTGFS_HCFG);
  regval &= ~OTGFS_HCFG_FSLSPCS_MASK;
  regval |= OTGFS_HCFG_FSLSPCS_FS48MHz;
  stm32_putreg(STM32_OTGFS_HCFG, regval);

  /* Reset the host port */

  stm32_portreset(priv);

  /* Clear the FS-/LS-only support bit in the HCFG register */

  regval = stm32_getreg(STM32_OTGFS_HCFG);
  regval &= ~OTGFS_HCFG_FSLSS;
  stm32_putreg(STM32_OTGFS_HCFG, regval);

  /* Carve up FIFO memory for the Rx FIFO and the periodic and non-periodic Tx FIFOs */
  /* Configure Rx FIFO size (GRXFSIZ) */

  stm32_putreg(STM32_OTGFS_GRXFSIZ, CONFIG_STM32_OTGFS_RXFIFO_SIZE);
  offset = CONFIG_STM32_OTGFS_RXFIFO_SIZE;

  /* Setup the host non-periodic Tx FIFO size (HNPTXFSIZ) */

  regval = (offset | (CONFIG_STM32_OTGFS_NPTXFIFO_SIZE << OTGFS_HNPTXFSIZ_NPTXFD_MASK);
  stm32_putreg(STM32_OTGFS_DIEPTXF0_HNPTXFSIZ, regval);
  offset += CONFIG_STM32_OTGFS_NPTXFIFO_SIZE

  /* Set up the host periodic Tx fifo size register (HPTXFSIZ) */

  regval = (offset | (CONFIG_STM32_OTGFS_PTXFIFO_SIZE << OTGFS_HPTXFSIZ_PTXFD_SHIFT);
  stm32_putreg(STM32_OTGFS_HPTXFSIZ, regval);

  /* If OTG were supported, we sould need to clear HNP enable bit in the
   * USB_OTG control register about here.
   */

  /* Flush all FIFOs */

  stm32_flush_txfifos(OTGFS_GRSTCTL_TXFNUM_HALL);
  stm32_flush_rxfifo();

  /* Clear all pending HC Interrupts */

  for (i = 0; i < STM32_NHOST_CHANNELS; i++)
    {
      stm32_putreg(STM32_OTGFS_HCINT(i), 0xffffffff);
      stm32_putreg(STM32_OTGFS_HCINTMSK(i), 0);
    }

  /* Driver Vbus +5V (the smoke test).  Should be done elsewhere in OTG
   * mode.
   */

  stm32_usbhost_vbusdrive(0, true);

  /* Enable host interrupts */

  stm32_hostinit_enable(priv);
  return OK;
}

/*******************************************************************************
 * Name: stm32_sw_initialize
 *
 * Description:
 *   One-time setup of the host driver state structure.
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   None.
 *
 *******************************************************************************/

static inline void stm32_sw_initialize(FAR struct stm32_usbhost_s *priv)
{
  int i;

  /* Initialize the state data structure */

  sem_init(&priv->eventsem,  0, 0);
  sem_init(&priv->exclsem, 0, 1);

#warning "Missing logic"

  /* Indicate that we are not connected */

  priv->connected = false;

  /* Put all of the channels back in their initial state */

  memset(priv-chan, 0, STM32_MAX_TX_FIFOS * sizeof(struct stn32_chan_s));

  for (i = 0; i < STM32_MAX_TX_FIFOS; i++)
    {
      FAR struct stm32_chan_s *chan = &priv->chan[i];
      sem_init(&chan->chansem,  0, 0);
    }

  /* Initialize endpoint zero packet size */

  priv->chan[0].maxpacket = STM32_EP0_MAX_PACKET_SIZE;
}

/*******************************************************************************
 * Name: stm32_hw_initialize
 *
 * Description:
 *   One-time setup of the host controller harware for normal operations.
 *
 * Input Parameters:
 *   priv -- USB host driver private data structure.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 *******************************************************************************/

static inline int stm32_hw_initialize(FAR struct stm32_usbhost_s *priv)
{
  uint32_t regval;
  int ret;

  /* Set the PHYSEL bit in the GUSBCFG register to select the OTG FS serial
   * transceiver: "This bit is always 1 with write-only access"
   */
  
  regval = stm32_getreg(STM32_OTGFS_GUSBCFG);;
  regval |= OTGFS_GUSBCFG_PHYSEL;
  stm32_putreg(STM32_OTGFS_GUSBCFG, regval);

  /* Reset after a PHY select and set Host mode.  First, wait for AHB master
   * IDLE state.
   */

  for (timeout = 0; timeout < STM32_READY_DELAY; timeout++)
    {
      up_udelay(3);
      regval = stm32_getreg(STM32_OTGFS_GRSTCTL);
      if ((regval & OTGFS_GRSTCTL_AHBIDL) != 0)
        {
          break;
        }
    }

  /* Then perform the core soft reset. */

  stm32_putreg(STM32_OTGFS_GRSTCTL, OTGFS_GRSTCTL_CSRST);
  for (timeout = 0; timeout < STM32_READY_DELAY; timeout++)
    {
      regval = stm32_getreg(STM32_OTGFS_GRSTCTL);
      if ((regval & OTGFS_GRSTCTL_CSRST) == 0)
        {
          break;
        }
    }

  /* Wait for 3 PHY Clocks */

  up_udelay(3);

  /* Deactivate the power down */

  regval  = (OTGFS_GCCFG_PWRDWN | OTGFS_GCCFG_VBUSASEN | OTGFS_GCCFG_VBUSBSEN);
#ifndef CONFIG_USBDEV_VBUSSENSING
  regval |= OTGFS_GCCFG_NOVBUSSENS;
#endif
#ifdef CONFIG_STM32_OTGFS_SOFOUTPUT
  regval |= OTGFS_GCCFG_SOFOUTEN;
#endif
  stm32_putreg(STM32_OTGFS_GCCFG, regval);
  up_mdelay(20);

  /* Initialize OTG features:  In order to support OTP, the HNPCAP and SRPCAP
   * bits would need to be set in the GUSBCFG register about here.
   */

  /* Force Host Mode */

  regval  = stm32_getreg(STM32_OTGFS_GUSBCFG);
  regval &= ~OTGFS_GUSBCFG_FDMOD;
  regval |= OTGFS_GUSBCFG_FHMOD;
  stm32_putreg(STM32_OTGFS_GUSBCFG, regval);
  up_mdelay(50);

  /* Initialize host mode and return success */

  stm32_host_initialize(priv);
  return OK;
}

/*******************************************************************************
 * Public Functions
 *******************************************************************************/

/*******************************************************************************
 * Name: usbhost_initialize
 *
 * Description:
 *   Initialize USB host device controller hardware.
 *
 * Input Parameters:
 *   controller -- If the device supports more than USB host controller, then
 *     this identifies which controller is being intialized.  Normally, this
 *     is just zero.
 *
 * Returned Value:
 *   And instance of the USB host interface.  The controlling task should
 *   use this interface to (1) call the wait() method to wait for a device
 *   to be connected, and (2) call the enumerate() method to bind the device
 *   to a class driver.
 *
 * Assumptions:
 * - This function should called in the initialization sequence in order
 *   to initialize the USB device functionality.
 * - Class drivers should be initialized prior to calling this function.
 *   Otherwise, there is a race condition if the device is already connected.
 *
 *******************************************************************************/

FAR struct usbhost_driver_s *usbhost_initialize(int controller)
{
  /* At present, there is only support for a single OTG FS host. Hence it is
   * pre-allocated as g_usbhost.  However, in most code, the private data
   * structure will be referenced using the 'priv' pointer (rather than the
   * global data) in order to simplify any future support for multiple devices.
   */

  FAR struct stm32_usbhost_s *priv = &g_usbhost;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(controller == 0);

  /* Make sure that interrupts from the OTG FS core are disabled */

  stm32_gint_disable();

  /* Reset the state of the host driver */

  stm32_sw_initialize(priv);

  /* Alternate function pin configuration.  Here we assume that:
   *
   * 1. GPIOA, SYSCFG, and OTG FS peripheral clocking have already been\
   *    enabled as part of the boot sequence.
   * 2. Board-specific logic has already enabled other board specific GPIOs
   *    for things like soft pull-up, VBUS sensing, power controls, and over-
   *    current detection.
   */

  /* Configure OTG FS alternate function pins for DM, DP, ID, and SOF.
   *
   * PIN* SIGNAL      DIRECTION
   * ---- ----------- ----------
   * PA8  OTG_FS_SOF  SOF clock output
   * PA9  OTG_FS_VBUS VBUS input for device, Driven by external regulator by
   *                  host (not an alternate function)
   * PA10 OTG_FS_ID   OTG ID pin (only needed in Dual mode)
   * PA11 OTG_FS_DM   D- I/O
   * PA12 OTG_FS_DP   D+ I/O
   *
   * *Pins may vary from device-to-device.
   */

  stm32_configgpio(GPIO_OTGFS_DM);
  stm32_configgpio(GPIO_OTGFS_DP);
  stm32_configgpio(GPIO_OTGFS_ID);    /* Only needed for OTG */

  /* SOF output pin configuration is configurable */

#ifdef CONFIG_STM32_OTGFS_SOFOUTPUT
  stm32_configgpio(GPIO_OTGFS_SOF);
#endif

  /* Initialize the USB OTG FS core */     

  stm32_hw_initialize(priv);

  /* Attach USB host controller interrupt handler */

  if (irq_attach(STM32_IRQ_OTGFS, stm32_gint_isr) != 0)
    {
      udbg("Failed to attach IRQ\n");
      return NULL;
    }

  /* Enable USB OTG FS global interrupts */

  stm32_gint_enable();

  /* Enable interrupts at the interrupt controller */

  up_enable_irq(STM32_IRQ_OTGFS);
  return &priv->drvr;
}
