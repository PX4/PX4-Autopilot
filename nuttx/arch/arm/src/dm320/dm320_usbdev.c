/*******************************************************************************
 * arch/arm/src/dm320/dm320_usbdev.c
 *
 *   Copyright (C) 2008-2012 Gregory Nutt. All rights reserved.
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
 *******************************************************************************/

/*******************************************************************************
 * Included Files
 *******************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "dm320_usb.h"

/*******************************************************************************
 * Pre-Processor Definitions
 *******************************************************************************/

/* Configuration ***************************************************************/

#ifndef  CONFIG_USBDEV_MAXPOWER
#  define CONFIG_USBDEV_MAXPOWER 100  /* mA */
#endif

/* Verify the selected USB attach interrupt GIO line */

#if CONFIG_DM320_GIO_USBATTACH < 0 || CONFIG_DM320_GIO_USBATTACH > 15
#  error "CONFIG_DM320_GIO_USBATTACH invalid"
#endif

/* Calculate the IRQ number associated with this GIO */

#define IRQ_USBATTACH (DM320_IRQ_EXT0+CONFIG_DM320_GIO_USBATTACH)

/* Vendor ID & Product ID of the USB device */

#ifndef CONFIG_DM320_VENDORID
#  define CONFIG_DM320_VENDORID     0xd320
#endif

#ifndef CONFIG_DM320_PRODUCTID
#  define CONFIG_DM320_PRODUCTID    0x3211
#endif

/* Extremely detailed register debug that you would normally never want
 * enabled.
 */

#undef CONFIG_DM320_USBDEV_REGDEBUG

/* Debug ***********************************************************************/

/* Trace error codes */

#define DM320_TRACEERR_ALLOCFAIL         0x0001
#define DM320_TRACEERR_ATTACHIRQREG      0x0002
#define DM320_TRACEERR_BINDFAILED        0x0003
#define DM320_TRACEERR_COREIRQREG        0x0004
#define DM320_TRACEERR_DRIVER            0x0005
#define DM320_TRACEERR_DRIVERREGISTERED  0x0006
#define DM320_TRACEERR_EPREAD            0x0007
#define DM320_TRACEERR_EWRITE            0x0008
#define DM320_TRACEERR_INVALIDPARMS      0x0009
#define DM320_TRACEERR_NOEP              0x000a
#define DM320_TRACEERR_NOTCONFIGURED     0x000b
#define DM320_TRACEERR_NULLPACKET        0x000c
#define DM320_TRACEERR_NULLREQUEST       0x000d
#define DM320_TRACEERR_REQABORTED        0x000e
#define DM320_TRACEERR_STALLEDCLRFEATURE 0x000f
#define DM320_TRACEERR_STALLEDISPATCH    0x0010
#define DM320_TRACEERR_STALLEDGETST      0x0011
#define DM320_TRACEERR_STALLEDGETSTEP    0x0012
#define DM320_TRACEERR_STALLEDGETSTRECIP 0x0013
#define DM320_TRACEERR_STALLEDREQUEST    0x0014
#define DM320_TRACEERR_STALLEDSETFEATURE 0x0015

/* Trace interrupt codes */

#define DM320_TRACEINTID_ATTACHED        0x0001
#define DM320_TRACEINTID_ATTACH          0x0002
#define DM320_TRACEINTID_CLEARFEATURE    0x0003
#define DM320_TRACEINTID_CONNECTED       0x0004
#define DM320_TRACEINTID_CONTROL         0x0005
#define DM320_TRACEINTID_DETACHED        0x0006
#define DM320_TRACEINTID_DISCONNECTED    0x0007
#define DM320_TRACEINTID_DISPATCH        0x0008
#define DM320_TRACEINTID_GETENDPOINT     0x0009
#define DM320_TRACEINTID_GETIFDEV        0x000a
#define DM320_TRACEINTID_GETSETDESC      0x000b
#define DM320_TRACEINTID_GETSETIFCONFIG  0x000c
#define DM320_TRACEINTID_GETSTATUS       0x000d
#define DM320_TRACEINTID_RESET           0x000e
#define DM320_TRACEINTID_RESUME          0x000f
#define DM320_TRACEINTID_RXFIFO          0x0010
#define DM320_TRACEINTID_RXPKTRDY        0x0011
#define DM320_TRACEINTID_SESSRQ          0x0012
#define DM320_TRACEINTID_SETADDRESS      0x0013
#define DM320_TRACEINTID_SETFEATURE      0x0014
#define DM320_TRACEINTID_SOF             0x0015
#define DM320_TRACEINTID_SUSPEND         0x0016
#define DM320_TRACEINTID_SYNCHFRAME      0x0017
#define DM320_TRACEINTID_TESTMODE        0x0018
#define DM320_TRACEINTID_TXFIFO          0x0019
#define DM320_TRACEINTID_TXFIFOSETEND    0x001a
#define DM320_TRACEINTID_TXFIFOSTALL     0x001b
#define DM320_TRACEINTID_TXPKTRDY        0x001c
#define DM320_TRACEINTID_UNKNOWN         0x001d
#define DM320_TRACEINTID_USBCTLR         0x001d
#define DM320_TRACEINTID_VBUSERR         0x001f

/* Hardware interface **********************************************************/

/* The DM320 hardware supports 8 configurable endpoints EP1-4, IN and OUT
 * (in addition to EP0 IN and OUT).  This driver, however, does not exploit
 * the full configuratability of the hardware at this time but, instead,
 * supports the one interrupt IN, one bulk IN and one bulk OUT endpoint.
 */

/* Hardware dependent sizes and numbers */

#define DM320_EP0MAXPACKET      64          /* EP0 max packet size */
#define DM320_BULKMAXPACKET     64          /* Bulk endpoint max packet */
#define DM320_INTRMAXPACKET     64          /* Interrupt endpoint max packet */
#define DM320_NENDPOINTS         4          /* Includes EP0 */

/* Endpoint numbers */

#define DM320_EP0                0          /* Control endpoint */
#define DM320_EPBULKIN           1          /* Bulk EP for send to host */
#define DM320_EPBULKOUT          2          /* Bulk EP for recv to host */
#define DM320_EPINTRIN           3          /* Intr EP for host poll */

/* Request queue operations ****************************************************/

#define dm320_rqempty(ep)       ((ep)->head == NULL)
#define dm320_rqpeek(ep)        ((ep)->head)

/*******************************************************************************
 * Private Types
 *******************************************************************************/

/* A container for a request so that the request make be retained in a list */

struct dm320_req_s
{
  struct usbdev_req_s    req;           /* Standard USB request */
  struct dm320_req_s    *flink;         /* Supports a singly linked list */
};

/* This is the internal representation of an endpoint */

struct dm320_ep_s
{
  /* Common endpoint fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_ep_s
   * to struct dm320_ep_s.
   */

  struct usbdev_ep_s     ep;            /* Standard endpoint structure */

  /* DM320-specific fields */

  struct dm320_usbdev_s *dev;           /* Reference to private driver data */
  struct dm320_req_s    *head;          /* Request list for this endpoint */
  struct dm320_req_s    *tail;
  uint8_t                epphy;         /* Physical EP address/index */
  uint8_t                stalled:1;     /* Endpoint is halted */
  uint8_t                in:1;          /* Endpoint is IN only */
  uint8_t                halted:1;      /* Endpoint feature halted */
  uint8_t                txnullpkt:1;   /* Null packet needed at end of transfer */
};

/* This structure encapsulates the overall driver state */

struct dm320_usbdev_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_s
   * to struct dm320_usbdev_s.
   */

  struct usbdev_s        usbdev;

  /* The bound device class driver */

  struct usbdevclass_driver_s *driver;

  /* DM320-specific fields */

  uint8_t                stalled:1;     /* 1: Protocol stalled */
  uint8_t                selfpowered:1; /* 1: Device is self powered */
  uint8_t                paddrset:1;    /* 1: Peripheral addr has been set */
  uint8_t                attached:1;    /* 1: Host attached */
  uint8_t                rxpending:1;   /* 1: RX pending */
  uint8_t                paddr;         /* Peripheral address */

  /* The endpoint list */

  struct dm320_ep_s      eplist[DM320_NENDPOINTS];
};

/* For maintaining tables of endpoint info */

struct dm320_epinfo_s
{
  uint8_t                addr;          /* Logical endpoint address */
  uint8_t                attr;          /* Endpoint attributes */
  uint8_t                fifo;          /* FIFO mx pkt size + dual buffer bits */
#ifdef CONFIG_USBDEV_HIGHSPEED
  uint16_t               maxpacket;     /* Max packet size */
#else
  uint8_t                maxpacket;     /* Max packet size */
#endif
};

/*******************************************************************************
 * Private Function Prototypes
 *******************************************************************************/

/* Register operations */

#if defined(CONFIG_DM320_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
static uint32_t dm320_getreg8(uint32_t addr);
static uint32_t dm320_getreg16(uint32_t addr);
static uint32_t dm320_getreg32(uint32_t addr);
static void dm320_putreg8(uint8_t val, uint32_t addr);
static void dm320_putreg16(uint16_t val, uint32_t addr);
static void dm320_putreg32(uint32_t val, uint32_t addr);
#else
# define dm320_getreg8(addr)      getreg8(addr)
# define dm320_getreg16(addr)     getreg16(addr)
# define dm320_getreg32(addr)     getreg32(addr)
# define dm320_putreg8(val,addr)  putreg8(val,addr)
# define dm320_putreg16(val,addr) putreg16(val,addr)
# define dm320_putreg32(val,addr) putreg32(val,addr)
#endif

/* Request queue operations ****************************************************/

static FAR struct dm320_req_s *dm320_rqdequeue(FAR struct dm320_ep_s *privep);
static void dm320_rqenqueue(FAR struct dm320_ep_s *privep, FAR struct dm320_req_s *req);

/* Low level data transfers and request operations */

static int  dm320_ep0write(uint8_t *buf, uint16_t nbytes);
static int  dm320_epwrite(uint8_t epphy, uint8_t *buf, uint16_t nbytes);
static int  dm320_epread(uint8_t epphy, uint8_t *buf, uint16_t nbytes);
static inline void dm320_abortrequest(struct dm320_ep_s *privep,
              struct dm320_req_s *privreq, int16_t result);
static void dm320_reqcomplete(struct dm320_ep_s *privep, int16_t result);
static int  dm320_wrrequest(struct dm320_ep_s *privep);
static int  dm320_rdrequest(struct dm320_ep_s *privep);
static void dm320_cancelrequests(struct dm320_ep_s *privep);

/* Interrupt handling */

static struct dm320_ep_s *dm320_epfindbyaddr(struct dm320_usbdev_s *priv,
              uint16_t eplog);
static void dm320_dispatchrequest(struct dm320_usbdev_s *priv,
              const struct usb_ctrlreq_s *ctrl);
static inline void dm320_ep0setup(struct dm320_usbdev_s *priv);
static inline uint32_t dm320_highestpriinterrupt(int intstatus);
static int  dm320_ctlrinterrupt(int irq, FAR void *context);
static int  dm320_attachinterrupt(int irq, FAR void *context);

/* Initialization operations */

static void dm320_epreset(unsigned int index);
static inline void dm320_epinitialize(struct dm320_usbdev_s *priv);
static void dm320_ctrlinitialize(struct dm320_usbdev_s *priv);

/* Endpoint methods */

static int  dm320_epconfigure(FAR struct usbdev_ep_s *ep,
              const struct usb_epdesc_s *desc, bool last);
static int  dm320_epdisable(FAR struct usbdev_ep_s *ep);
static FAR struct usbdev_req_s *dm320_epallocreq(FAR struct usbdev_ep_s *ep);
static void dm320_epfreereq(FAR struct usbdev_ep_s *ep, FAR struct usbdev_req_s *req);
#ifdef CONFIG_USBDEV_DMA
static FAR void *dm320_epallocbuffer(FAR struct usbdev_ep_s *ep, uint16_t nbytes);
static void dm320_epfreebuffer(FAR struct usbdev_ep_s *ep, void *buf);
#endif
static int  dm320_epsubmit(FAR struct usbdev_ep_s *ep, struct usbdev_req_s *privreq);
static int  dm320_epcancel(FAR struct usbdev_ep_s *ep, struct usbdev_req_s *privreq);

/* USB device controller methods */

static FAR struct usbdev_ep_s *dm320_allocep(FAR struct usbdev_s *dev,
              uint8_t epno, bool in, uint8_t eptype);
static void dm320_freeep(FAR struct usbdev_s *dev, FAR struct usbdev_ep_s *ep);
static int dm320_getframe(struct usbdev_s *dev);
static int dm320_wakeup(struct usbdev_s *dev);
static int dm320_selfpowered(struct usbdev_s *dev, bool selfpowered);
static int dm320_pullup(struct usbdev_s *dev, bool enable);

/*******************************************************************************
 * Private Data
 *******************************************************************************/

/* Endpoint methods */

static const struct usbdev_epops_s g_epops =
{
  .configure   = dm320_epconfigure,
  .disable     = dm320_epdisable,
  .allocreq    = dm320_epallocreq,
  .freereq     = dm320_epfreereq,
#ifdef CONFIG_USBDEV_DMA
  .allocbuffer = dm320_epallocbuffer,
  .freebuffer  = dm320_epfreebuffer,
#endif
  .submit      = dm320_epsubmit,
  .cancel      = dm320_epcancel,
};

/* USB controller device methods */

static const struct usbdev_ops_s g_devops =
{
  .allocep     = dm320_allocep,
  .freeep      = dm320_freeep,
  .getframe    = dm320_getframe,
  .wakeup      = dm320_wakeup,
  .selfpowered = dm320_selfpowered,
#ifdef CONFIG_DM320_GIO_USBDPPULLUP
  .pullup      = dm320_pullup,
#endif
};

/* There is only one, single, pre-allocated instance of the driver structure */

static struct dm320_usbdev_s g_usbdev;

/* Summarizes information about all DM320 endpoints */

static const struct dm320_epinfo_s g_epinfo[DM320_NENDPOINTS] =
{
  {
    0,                                        /* EP0 */
    USB_EP_ATTR_XFER_CONTROL,                 /* Type: Control IN/OUT */
    USB_TXFIFO2_SZ_64|USB_TXFIFO2_SINGLE_BUF, /* Bits for TX/RXFIFO2 */
    DM320_EP0MAXPACKET                        /* Max packet size */
  },
  {
    DM320_EPBULKIN | USB_DIR_IN,              /* Logical endpoint number: 1 IN */
    USB_EP_ATTR_XFER_BULK,                    /* Type: Bulk */
    USB_TXFIFO2_SZ_64|USB_TXFIFO2_SINGLE_BUF, /* Bits for TX/RXFIFO2 */
    DM320_BULKMAXPACKET,                      /* Max packet size */
  },
  {
    DM320_EPBULKOUT | USB_DIR_OUT,            /* Logical endpoint number: 2 OUT */
    USB_EP_ATTR_XFER_BULK,                    /* Type: Bulk */
    USB_TXFIFO2_SZ_64|USB_TXFIFO2_SINGLE_BUF, /* Bits for TX/RXFIFO2 */
    DM320_BULKMAXPACKET                       /* Max packet size */
  },
  {
    DM320_EPINTRIN| USB_DIR_IN,               /* Logical endpoint number: 3 IN */
    USB_EP_ATTR_XFER_INT,                     /* Type: Interrupt */
    USB_TXFIFO2_SZ_64|USB_TXFIFO2_SINGLE_BUF, /* Bits for TX/RXFIFO2 */
    DM320_INTRMAXPACKET                       /* Max packet size */
  }
};

/*******************************************************************************
 * Private Functions
 *******************************************************************************/

/*******************************************************************************
 * Name: dm320_getreg8
 *
 * Description:
 *   Get the contents of an DM320 8-bit register
 *
 *******************************************************************************/

#if defined(CONFIG_DM320_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
static uint8_t dm320_getreg8(uint32_t addr)
{
  static uint32_t prevaddr = 0;
  static uint8_t preval = 0;
  static uint32_t count = 0;

  /* Read the value from the register */

  uint8_t val = getreg8(addr);

  /* Is this the same value that we read from the same registe last time?  Are
   * we polling the register?  If so, suppress some of the output.
   */

  if (addr == prevaddr || val == preval)
    {
      if (count == 0xffffffff || ++count > 3)
        {
           if (count == 4)
             {
               lldbg("...\n");
             }
          return val;
        }
    }

  /* No this is a new address or value */

  else
    {
       /* Did we print "..." for the previous value? */

       if (count > 3)
         {
           /* Yes.. then show how many times the value repeated */

           lldbg("[repeats %d more times]\n", count-3);
         }

       /* Save the new address, value, and count */

       prevaddr = addr;
       preval   = val;
       count    = 1;
    }

  /* Show the register value read */

  lldbg("%08x->%02x\n", addr, val);
  return val;
}
#endif

/*******************************************************************************
 * Name: dm320_getreg16
 *
 * Description:
 *   Get the contents of an DM320 16-bit register
 *
 *******************************************************************************/

#if defined(CONFIG_DM320_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
static uint32_t dm320_getreg16(uint32_t addr)
{
  static uint32_t prevaddr = 0;
  static uint16_t preval = 0;
  static uint32_t count = 0;

  /* Read the value from the register */

  uint16_t val = getreg16(addr);

  /* Is this the same value that we read from the same registe last time?  Are
   * we polling the register?  If so, suppress some of the output.
   */

  if (addr == prevaddr || val == preval)
    {
      if (count == 0xffffffff || ++count > 3)
        {
           if (count == 4)
             {
               lldbg("...\n");
             }
          return val;
        }
    }

  /* No this is a new address or value */

  else
    {
       /* Did we print "..." for the previous value? */

       if (count > 3)
         {
           /* Yes.. then show how many times the value repeated */

           lldbg("[repeats %d more times]\n", count-3);
         }

       /* Save the new address, value, and count */

       prevaddr = addr;
       preval   = val;
       count    = 1;
    }

  /* Show the register value read */

  lldbg("%08x->%04x\n", addr, val);
  return val;
}
#endif

/*******************************************************************************
 * Name: dm320_getreg32
 *
 * Description:
 *   Get the contents of an DM320 32-bit register
 *
 *******************************************************************************/

#if defined(CONFIG_DM320_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
static uint32_t dm320_getreg32(uint32_t addr)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval = 0;
  static uint32_t count = 0;

  /* Read the value from the register */

  uint32_t val = getreg32(addr);

  /* Is this the same value that we read from the same registe last time?  Are
   * we polling the register?  If so, suppress some of the output.
   */

  if (addr == prevaddr || val == preval)
    {
      if (count == 0xffffffff || ++count > 3)
        {
           if (count == 4)
             {
               lldbg("...\n");
             }
          return val;
        }
    }

  /* No this is a new address or value */

  else
    {
       /* Did we print "..." for the previous value? */

       if (count > 3)
         {
           /* Yes.. then show how many times the value repeated */

           lldbg("[repeats %d more times]\n", count-3);
         }

       /* Save the new address, value, and count */

       prevaddr = addr;
       preval   = val;
       count    = 1;
    }

  /* Show the register value read */

  lldbg("%08x->%08x\n", addr, val);
  return val;
}
#endif

/*******************************************************************************
 * Name: dm320_putreg8
 *
 * Description:
 *   Set the contents of an DM320 8-bit register to a value
 *
 *******************************************************************************/

#if defined(CONFIG_DM320_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
static void dm320_putreg8(uint8_t val, uint32_t addr)
{
  /* Show the register value being written */

  lldbg("%08x<-%02x\n", addr, val);

  /* Write the value */

  putreg8(val, addr);
}
#endif

/*******************************************************************************
 * Name: dm320_putreg16
 *
 * Description:
 *   Set the contents of an DM320 16-bit register to a value
 *
 *******************************************************************************/

#if defined(CONFIG_DM320_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
static void dm320_putreg16(uint16_t val, uint32_t addr)
{
  /* Show the register value being written */

  lldbg("%08x<-%04x\n", addr, val);

  /* Write the value */

  putreg16(val, addr);
}
#endif

/*******************************************************************************
 * Name: dm320_putreg32
 *
 * Description:
 *   Set the contents of an DM320 32-bit register to a value
 *
 *******************************************************************************/

#if defined(CONFIG_DM320_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
static void dm320_putreg32(uint32_t val, uint32_t addr)
{
  /* Show the register value being written */

  lldbg("%08x<-%08x\n", addr, val);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/*******************************************************************************
 * Name: dm320_rqdequeue
 *
 * Description:
 *   Remove a request from an endpoint request queue
 *
 *******************************************************************************/

static FAR struct dm320_req_s *dm320_rqdequeue(FAR struct dm320_ep_s *privep)
{
  FAR struct dm320_req_s *ret = privep->head;

  if (ret)
    {
      privep->head = ret->flink;
      if (!privep->head)
        {
          privep->tail = NULL;
        }

      ret->flink = NULL;
    }

  return ret;
}

/*******************************************************************************
 * Name: dm320_rqenqueue
 *
 * Description:
 *   Add a request from an endpoint request queue
 *
 *******************************************************************************/

static void dm320_rqenqueue(FAR struct dm320_ep_s *privep,
                            FAR struct dm320_req_s *req)
{
  req->flink = NULL;
  if (!privep->head)
    {
      privep->head = req;
      privep->tail = req;
    }
  else
    {
      privep->tail->flink = req;
      privep->tail        = req;
    }
}

/*******************************************************************************
 * Name: dm320_ep0write
 *
 * Description:
 *   Control endpoint write (IN)
 *
 *******************************************************************************/

static int dm320_ep0write(uint8_t *buf, uint16_t nbytes)
{
  uint8_t csr0 = USB_PERCSR0_TXPKTRDY; /* XMT packet ready bit */
  uint16_t bytesleft;
  uint16_t nwritten;

  if ( nbytes <=  DM320_EP0MAXPACKET)
    {
      bytesleft = nbytes;
      csr0     |= USB_PERCSR0_DATAEND; /* Transaction end bit */
    }
  else
    {
      bytesleft = DM320_EP0MAXPACKET;
    }

  nwritten = bytesleft;
  while (bytesleft > 0)
    {
      dm320_putreg8(*buf++, DM320_USB_FIFO0);
      bytesleft--;
    }

  dm320_putreg8(csr0, DM320_USB_PERCSR0);
  return nwritten;
}

/*******************************************************************************
 * Name: dm320_epwrite
 *
 * Description:
 *   Endpoint write (IN)
 *
 *******************************************************************************/

static int dm320_epwrite(uint8_t epphy, uint8_t *buf, uint16_t nbytes)
{
  volatile uint8_t *fifo;
  uint16_t bytesleft;
  int ret = ERROR;

  if (/*epphy < USB_EP0_SELECT || */ epphy >= DM320_NENDPOINTS)
    {
      return ret;
    }
  dm320_putreg8(epphy, DM320_USB_INDEX);

  if (epphy == USB_EP0_SELECT )
    {
      return dm320_ep0write(buf, nbytes);
    }

  bytesleft = DM320_BULKMAXPACKET;
  if (bytesleft > nbytes)
    {
      bytesleft = nbytes;
    }

  ret  = bytesleft;
  fifo = (volatile uint8_t *)DM320_USB_FIFO0;
  fifo = fifo + (epphy << 2);

  if (dm320_getreg8(DM320_USB_PERTXCSR1) & USB_TXCSR1_FIFOEMP)
    {
      dm320_putreg8(dm320_getreg8(DM320_USB_PERTXCSR1) | USB_TXCSR1_TXPKTRDY, DM320_USB_PERTXCSR1);
      while (dm320_getreg8(DM320_USB_PERTXCSR1) & USB_TXCSR1_TXPKTRDY);
      dm320_putreg8((dm320_getreg8(DM320_USB_PERTXCSR1) | USB_TXCSR1_FLFIFO), DM320_USB_PERTXCSR1);
    }

  while (bytesleft > 0)
    {
      *fifo = *buf++;
      bytesleft--;
    }
  dm320_putreg8(dm320_getreg8(DM320_USB_PERTXCSR1) | USB_TXCSR1_TXPKTRDY, DM320_USB_PERTXCSR1);
  return ret;
}

/*******************************************************************************
 * Name: dm320_epread
 *
 * Description:
 *   Endpoint read (OUT)
 *
 *******************************************************************************/

static int dm320_epread(uint8_t epphy, uint8_t *buf, uint16_t nbytes)
{
  volatile uint8_t *fifo;
  int bytesleft;
  int ret  = ERROR;

  if (/*epphy < USB_EP0_SELECT || */ epphy >= DM320_NENDPOINTS)
    {
      return ret;
    }
  dm320_putreg8(epphy, DM320_USB_INDEX);

  if (epphy == USB_EP0_SELECT)
    {
      bytesleft = dm320_getreg8(DM320_USB_COUNT0);
      if (bytesleft > nbytes)
        {
          bytesleft = nbytes;
        }
    }
  else 
   {
      bytesleft = dm320_getreg8(DM320_USB_RXCOUNT2);
      bytesleft = (bytesleft << 8) + dm320_getreg8(DM320_USB_RXCOUNT1);
      if (bytesleft > nbytes)
        {
          bytesleft = nbytes;
        }
    }

  ret    = bytesleft;
  fifo = (uint8_t*)DM320_USB_FIFO0;
  fifo = fifo + (epphy << 2);

  while (bytesleft > 0)
  {
    *buf++ = *fifo;
     bytesleft--;
  }

  /* Clear RXPKTRDY bit in PER_RXCSR1 */

  dm320_putreg8(dm320_getreg8(DM320_USB_PERRXCSR1) & ~(USB_PERRXCSR1_RXPKTRDY), DM320_USB_PERRXCSR1);
  return ret;
}

/*******************************************************************************
 * Name: dm320_abortrequest
 *
 * Description:
 *   Discard a request
 *
 *******************************************************************************/

static inline void dm320_abortrequest(struct dm320_ep_s *privep,
                                      struct dm320_req_s *privreq,
                                      int16_t result)
{
  usbtrace(TRACE_DEVERROR(DM320_TRACEERR_REQABORTED), (uint16_t)privep->epphy);

  /* Save the result in the request structure */

  privreq->req.result = result;

  /* Callback to the request completion handler */

  privreq->req.callback(&privep->ep, &privreq->req);
}

/*******************************************************************************
 * Name: dm320_reqcomplete
 *
 * Description:
 *   Handle termination of a request.
 *
 *******************************************************************************/

static void dm320_reqcomplete(struct dm320_ep_s *privep, int16_t result)
{
  struct dm320_req_s *privreq;
  int stalled = privep->stalled;
  irqstate_t flags;

  /* Remove the completed request at the head of the endpoint request list */

  flags = irqsave();
  privreq = dm320_rqdequeue(privep);
  irqrestore(flags);

  if (privreq)
    {
      /* If endpoint 0, temporarily reflect the state of protocol stalled
       * in the callback.
       */

      if (privep->epphy == 0)
        {
          if (privep->dev->stalled)
            {
              privep->stalled = 1;
            }
        }

      /* Save the result in the request structure */

      privreq->req.result = result;

      /* Callback to the request completion handler */

      privreq->flink = NULL;
      privreq->req.callback(&privep->ep, &privreq->req);

      /* Restore the stalled indication */

      privep->stalled = stalled;
    }
}

/*******************************************************************************
 * Name: dm320_wrrequest
 *
 * Description:
 *   Send from the next queued write request
 *
 * Returned Value:
 *  0:not finished; 1:completed; <0:error
 *
 *******************************************************************************/

static int dm320_wrrequest(struct dm320_ep_s *privep)
{
  struct dm320_req_s *privreq;
  uint8_t *buf;
  int nbytes;
  int bytesleft;
  int nbyteswritten;

  /* Check the request from the head of the endpoint request queue */

  privreq = dm320_rqpeek(privep);
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(DM320_TRACEERR_NULLREQUEST), 0);
      return OK;
    }

  /* Otherwise send the data in the packet (in the DMA on case, we
   * may be resuming transfer already in progress.
   */

  for (;;)
    {
      /* Get the number of bytes left to be sent in the packet */

      bytesleft = privreq->req.len - privreq->req.xfrd;

      /* Send the next packet if (1) there are more bytes to be sent, or
       * (2) the last packet sent was exactly maxpacketsize (bytesleft == 0)
       */

      usbtrace(TRACE_WRITE(privep->epphy), privreq->req.xfrd);
      if (bytesleft >  0 || privep->txnullpkt)
        {
          /* Try to send maxpacketsize -- unless we don't have that many
           * bytes to send.
           */

          privep->txnullpkt = 0;
          if (bytesleft > privep->ep.maxpacket)
            {
              nbytes = privep->ep.maxpacket;
            }
          else
            {
              nbytes = bytesleft;
              if ((privreq->req.flags & USBDEV_REQFLAGS_NULLPKT) != 0)
                {
                  privep->txnullpkt = (bytesleft == privep->ep.maxpacket);
                }
            }

          /* Send the largest number of bytes that we can in this packet */

          buf           = privreq->req.buf + privreq->req.xfrd;
          nbyteswritten = dm320_epwrite(privep->epphy, buf, nbytes);
          if (nbyteswritten < 0 || nbyteswritten != nbytes)
            {
              usbtrace(TRACE_DEVERROR(DM320_TRACEERR_EWRITE), nbyteswritten);
              return ERROR;
            }

          /* Update for the next time through the loop */

          privreq->req.xfrd += nbytes;
        }

      /* If all of the bytes were sent (including any final null packet)
       * then we are finished with the transfer
       */

      if (privreq->req.xfrd >= privreq->req.len && !privep->txnullpkt)
        {
          usbtrace(TRACE_COMPLETE(privep->epphy), privreq->req.xfrd);
          privep->txnullpkt = 0;
          dm320_reqcomplete(privep, OK);
          return OK;
        }
    }

  return OK; /* Won't get here */
}

/*******************************************************************************
 * Name: dm320_rdrequest
 *
 * Description:
 *   Receive to the next queued read request
 *
 *******************************************************************************/

static int dm320_rdrequest(struct dm320_ep_s *privep)
{
  struct dm320_req_s *privreq;
  uint8_t *buf;
  int nbytesread;

  /* Check the request from the head of the endpoint request queue */

  privreq = dm320_rqpeek(privep);
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(DM320_TRACEERR_NULLREQUEST), 0);
      return OK;
    }

  usbtrace(TRACE_READ(privep->epphy), privreq->req.xfrd);

  /* Receive the next packet */

  buf        = privreq->req.buf + privreq->req.xfrd;
  nbytesread = dm320_epread(privep->epphy, buf, privep->ep.maxpacket);
  if (nbytesread < 0)
    {
      usbtrace(TRACE_DEVERROR(DM320_TRACEERR_EPREAD), nbytesread);
      return ERROR;
    }

  /* If the receive buffer is full or if the last packet was not full
   * then we are finished with the transfer.
   */

  privreq->req.xfrd += nbytesread;
  if (privreq->req.len < privreq->req.xfrd || nbytesread < privep->ep.maxpacket)
    {
      usbtrace(TRACE_COMPLETE(privep->epphy), privreq->req.xfrd);
      dm320_reqcomplete(privep, OK);
    }

  return OK;
}

/*******************************************************************************
 * Name: dm320_cancelrequests
 *
 * Description:
 *   Cancel all pending requests for an endpoint
 *
 *******************************************************************************/

static void dm320_cancelrequests(struct dm320_ep_s *privep)
{
  while (!dm320_rqempty(privep))
    {
      usbtrace(TRACE_COMPLETE(privep->epphy),
               (dm320_rqpeek(privep))->req.xfrd);
      dm320_reqcomplete(privep, -ESHUTDOWN);
    }
}

/*******************************************************************************
 * Name: dm320_epfindbyaddr
 *
 * Description:
 *   Find the physical endpoint structure corresponding to a logic endpoint
 *   address
 *
 *******************************************************************************/

static struct dm320_ep_s *dm320_epfindbyaddr(struct dm320_usbdev_s *priv,
                                             uint16_t eplog)
{
  struct dm320_ep_s *privep;
  int i;

  /* Endpoint zero is a special case */

  if (USB_EPNO(eplog) == 0)
    {
      return &priv->eplist[0];
    }

  /* Handle the remaining */

  for (i = 1; i < DM320_NENDPOINTS; i++)
    {
      privep = &priv->eplist[i];

      /* Same logical endpoint number? (includes direction bit) */

      if (eplog == privep->ep.eplog)
        {
          /* Return endpoint found */

          return privep;
        }
    }

  /* Return endpoint not found */

  return NULL;
}

/*******************************************************************************
 * Name: dm320_dispatchrequest
 *
 * Description:
 *   Provide unhandled setup actions to the class driver
 *
 *******************************************************************************/

static void dm320_dispatchrequest(struct dm320_usbdev_s *priv,
                                  const struct usb_ctrlreq_s *ctrl)
{
  int ret;

  usbtrace(TRACE_INTDECODE(DM320_TRACEINTID_DISPATCH), 0);
  if (priv && priv->driver)
    {
      ret = CLASS_SETUP(priv->driver, &priv->usbdev, ctrl, NULL, 0);
      if (ret < 0)
        {
          /* Stall on failure */

          usbtrace(TRACE_DEVERROR(DM320_TRACEERR_STALLEDISPATCH), ctrl->req);
          priv->stalled = 1;
        }
    }
}

/*******************************************************************************
 * Name: dm320_ep0setup
 *
 * Description:
 *   USB Ctrl EP Setup Event
 *
 *******************************************************************************/

static inline void dm320_ep0setup(struct dm320_usbdev_s *priv)
{
  struct dm320_ep_s *ep0 = &priv->eplist[DM320_EP0];
  struct dm320_req_s *privreq = dm320_rqpeek(ep0);
  struct dm320_ep_s *privep;
  struct usb_ctrlreq_s ctrl;
  uint16_t index;
  uint16_t value;
  uint16_t len;
  int ret;

  /* Starting a control request? */

  if (priv->usbdev.speed == USB_SPEED_UNKNOWN)
    {
      priv->usbdev.speed = USB_SPEED_FULL;
    }

  /* Terminate any pending requests */

  while (!dm320_rqempty(ep0))
    {
      int16_t result = OK;
      if (privreq->req.xfrd != privreq->req.len)
        {
          result = -EPROTO;
        }

      usbtrace(TRACE_COMPLETE(ep0->epphy), privreq->req.xfrd);
      dm320_reqcomplete(ep0, result);
    }

  /* Assume NOT stalled */

  ep0->stalled  = 0;
  priv->stalled = 0;

  /* Read EP0 data */

  ret = dm320_epread(USB_EP0_SELECT, (uint8_t*)&ctrl, USB_SIZEOF_CTRLREQ);
  if (ret <= 0)
    {
      return;
    }

  index = GETUINT16(ctrl.index);
  value = GETUINT16(ctrl.value);
  len   = GETUINT16(ctrl.len);

  ullvdbg("type=%02x req=%02x value=%04x index=%04x len=%04x\n",
          ctrl.type, ctrl.req, value, index, len);

  /* Dispatch any non-standard requests */

  ep0->in = (ctrl.type & USB_DIR_IN) != 0;
  if ((ctrl.type & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_STANDARD)
    {
      dm320_putreg8(USB_PERCSR0_CLRRXRDY, DM320_USB_PERCSR0);
      dm320_dispatchrequest(priv, &ctrl);
      return;
    }

  /* Handle standard request.  Pick off the things of interest to the
   * USB device controller driver; pass what is left to the class driver
   */

  switch (ctrl.req)
    {
    case USB_REQ_GETSTATUS:
      {
        /* type:  device-to-host; recipient = device, interface, endpoint
         * value: 0
         * index: zero interface endpoint
         * len:   2; data = status
         */

        dm320_putreg8(USB_PERCSR0_CLRRXRDY | USB_PERCSR0_DATAEND, DM320_USB_PERCSR0);
        usbtrace(TRACE_INTDECODE(DM320_TRACEINTID_GETSTATUS), 0);

        if (len != 2 || (ctrl.type & USB_REQ_DIR_IN) == 0 || value != 0)
          {
            usbtrace(TRACE_DEVERROR(DM320_TRACEERR_STALLEDGETST), ctrl.req);
            priv->stalled = 1;
          }
        else
          {
            switch (ctrl.type & USB_REQ_RECIPIENT_MASK)
              {
              case USB_REQ_RECIPIENT_ENDPOINT:
                {
                  usbtrace(TRACE_INTDECODE(DM320_TRACEINTID_GETENDPOINT), 0);
                  privep = dm320_epfindbyaddr(priv, index);
                  if (!privep)
                    {
                      usbtrace(TRACE_DEVERROR(DM320_TRACEERR_STALLEDGETSTEP), ctrl.type);
                      priv->stalled = 1;
                    }
                }
                break;

              case USB_REQ_RECIPIENT_DEVICE:
              case USB_REQ_RECIPIENT_INTERFACE:
                usbtrace(TRACE_INTDECODE(DM320_TRACEINTID_GETIFDEV), 0);
                break;

              default:
                {
                  usbtrace(TRACE_DEVERROR(DM320_TRACEERR_STALLEDGETSTRECIP), ctrl.type);
                  priv->stalled = 1;
                }
                break;
              }
          }
      }
      break;

    case USB_REQ_CLEARFEATURE:
      {
        /* type:  host-to device; recipient = device, interface or endpoint
         * value: feature selector
         * index: zero interface endpoint;
         * len:   zero, data = none
         */

        dm320_putreg8(USB_PERCSR0_CLRRXRDY | USB_PERCSR0_DATAEND, DM320_USB_PERCSR0);
        usbtrace(TRACE_INTDECODE(DM320_TRACEINTID_CLEARFEATURE), (uint16_t)ctrl.req);
        if (ctrl.type != USB_REQ_RECIPIENT_ENDPOINT)
          {
              dm320_dispatchrequest(priv, &ctrl);
          }
        else if (value == USB_FEATURE_ENDPOINTHALT && len == 0 &&
                 (privep = dm320_epfindbyaddr(priv, index)) != NULL)
          {
            privep->halted = 0;

            /* Restart the write queue */

            (void)dm320_wrrequest(privep);
          }
        else
          {
            usbtrace(TRACE_DEVERROR(DM320_TRACEERR_STALLEDCLRFEATURE), ctrl.type);
            priv->stalled = 1;
          }
      }
      break;

    case USB_REQ_SETFEATURE:
      {
        /* type:  host-to-device; recipient = device, interface, endpoint
         * value: feature selector
         * index: zero interface endpoint;
         * len:   0; data = none
         */

        dm320_putreg8(USB_PERCSR0_CLRRXRDY | USB_PERCSR0_DATAEND, DM320_USB_PERCSR0);
        usbtrace(TRACE_INTDECODE(DM320_TRACEINTID_SETFEATURE), 0);
        if (ctrl.type == USB_REQ_RECIPIENT_DEVICE && value == USB_FEATURE_TESTMODE)
          {
            usbtrace(TRACE_INTDECODE(DM320_TRACEINTID_TESTMODE), index);
          }
        else if (ctrl.type != USB_REQ_RECIPIENT_ENDPOINT)
          {
            dm320_dispatchrequest(priv, &ctrl);
          }
        else if (value == USB_FEATURE_ENDPOINTHALT && len == 0 &&
                 (privep = dm320_epfindbyaddr(priv, index)) != NULL)
          {
            privep->halted = 1;
          }
        else
          {
            usbtrace(TRACE_DEVERROR(DM320_TRACEERR_STALLEDSETFEATURE), ctrl.type);
            priv->stalled = 1;
          }
      }
      break;

    case USB_REQ_SETADDRESS:
      {
        /* type:  host-to-device; recipient = device
         * value: device address
         * index: 0
         * len:   0; data = none
         */

        dm320_putreg8(USB_PERCSR0_CLRRXRDY|USB_PERCSR0_DATAEND, DM320_USB_PERCSR0);
        usbtrace(TRACE_INTDECODE(DM320_TRACEINTID_SETADDRESS), 0);
        priv->paddr = value & 0xff;
      }
      break;

    case USB_REQ_GETDESCRIPTOR:
      /* type:  device-to-host; recipient = device
       * value: descriptor type and index
       * index: 0 or language ID;
       * len:   descriptor len; data = descriptor
       */
    case USB_REQ_SETDESCRIPTOR:
      /* type:  host-to-device; recipient = device
       * value: descriptor type and index
       * index: 0 or language ID;
       * len:   descriptor len; data = descriptor
       */
      {
        dm320_putreg8(USB_PERCSR0_CLRRXRDY, DM320_USB_PERCSR0);
        usbtrace(TRACE_INTDECODE(DM320_TRACEINTID_GETSETDESC), 0);
        dm320_dispatchrequest(priv, &ctrl);
      }
      break;

    case USB_REQ_GETCONFIGURATION:
      /* type:  device-to-host; recipient = device
       * value: 0;
       * index: 0;
       * len:   1; data = configuration value
       */
    case USB_REQ_SETCONFIGURATION:
      /* type:  host-to-device; recipient = device
       * value: configuration value
       * index: 0;
       * len:   0; data = none
       */
    case USB_REQ_GETINTERFACE:
      /* type:  device-to-host; recipient = interface
       * value: 0
       * index: interface;
       * len:   1; data = alt interface
       */
    case USB_REQ_SETINTERFACE:
      /* type:  host-to-device; recipient = interface
       * value: alternate setting
       * index: interface;
       * len:   0; data = none
       */
      {
        dm320_putreg8(USB_PERCSR0_CLRRXRDY|USB_PERCSR0_DATAEND, DM320_USB_PERCSR0);
        usbtrace(TRACE_INTDECODE(DM320_TRACEINTID_GETSETIFCONFIG), 0);
        dm320_dispatchrequest(priv, &ctrl);
      }
      break;

    case USB_REQ_SYNCHFRAME:
      {
        /* type:  device-to-host; recipient = endpoint
         * value: 0
         * index: endpoint;
         * len:   2; data = frame number
         */

        dm320_putreg8(USB_PERCSR0_CLRRXRDY|USB_PERCSR0_SENDST, DM320_USB_PERCSR0);
        usbtrace(TRACE_INTDECODE(DM320_TRACEINTID_SYNCHFRAME), 0);
      }
      break;

    default:
      {
        dm320_putreg8(USB_PERCSR0_CLRRXRDY|USB_PERCSR0_SENDST, DM320_USB_PERCSR0);
        usbtrace(TRACE_DEVERROR(DM320_TRACEERR_STALLEDREQUEST), ctrl.req);
        priv->stalled = 1;
      }
      break;
    }
}

/*******************************************************************************
 * Name: dm320_highestpriinterrupt
 *
 * Description:
 *   Part of the USB core controller interrupt handling logic
 *
 *******************************************************************************/

static inline uint32_t dm320_highestpriinterrupt(int intstatus)
{
  if ((intstatus & USB_INT_CONNECTED) != 0)
    return USB_INT_CONNECTED;
  if ((intstatus & USB_INT_DISCONNECTED) != 0)
    return USB_INT_DISCONNECTED;
  if ((intstatus & USB_INT_RESET) != 0)
    return USB_INT_RESET;
  if ((intstatus & USB_INT_RESUME) != 0)
    return USB_INT_RESUME;
  if ((intstatus & USB_INT_SESSRQ) != 0)
    return USB_INT_SESSRQ;
  if ((intstatus & USB_INT_VBUSERR) != 0)
    return USB_INT_VBUSERR;
  if ((intstatus & USB_INT_SOF) != 0)
    return USB_INT_SOF;
  if ((intstatus & USB_INT_SUSPEND) != 0)
    return USB_INT_SUSPEND;
  if ((intstatus & USB_INT_CONTROL) != 0)
    return USB_INT_CONTROL;
  if ((intstatus & USB_INT_RXFIFO) != 0)
    return USB_INT_RXFIFO;
  if ((intstatus & USB_INT_TXFIFO) != 0)
    return USB_INT_TXFIFO;

  return USB_INT_NOINTERRUPT;
}

/*******************************************************************************
 * Name: dm320_ctlrinterrupt
 *
 * Description:
 *   Handle USB controller core interrupts
 *
 *******************************************************************************/

static int dm320_ctlrinterrupt(int irq, FAR void *context)
{
  struct dm320_usbdev_s *priv = &g_usbdev;
  struct dm320_ep_s *privep ;
  uint32_t intstatus;
  uint32_t priorityint;
  uint8_t csr0;

  usbtrace(TRACE_INTENTRY(DM320_TRACEINTID_USBCTLR), 0);

  /* Combine interretup registers into one interrupt status value */

  intstatus  = ((uint32_t)dm320_getreg8(DM320_USB_INTRTX1) << 12) |
               (((uint32_t)dm320_getreg8(DM320_USB_INTRRX1) >> 1) << 8) |
               (uint32_t)dm320_getreg8(DM320_USB_INTRUSB);
  /* Then process each interrupt source, highest priority first */

  do
    {
      priorityint = dm320_highestpriinterrupt(intstatus);
      switch (priorityint)
        {
        case USB_INT_RESET:
          usbtrace(TRACE_INTENTRY(DM320_TRACEINTID_RESET), 0);
          priv->paddrset = 0;
          break;

        case USB_INT_SESSRQ:
          usbtrace(TRACE_INTENTRY(DM320_TRACEINTID_SESSRQ), 0);
          break;

        case USB_INT_VBUSERR:
          usbtrace(TRACE_INTENTRY(DM320_TRACEINTID_VBUSERR), 0);
          break;

        case USB_INT_CONNECTED:
          usbtrace(TRACE_INTENTRY(DM320_TRACEINTID_CONNECTED), 0);
          break;

        case USB_INT_RESUME:
          usbtrace(TRACE_INTENTRY(DM320_TRACEINTID_RESUME), 0);
          break;

        case USB_INT_CONTROL:
          {
            /* Select EP0 */

            dm320_putreg8(USB_EP0_SELECT, DM320_USB_INDEX);

            /* Check for a response complete interrupt */

            csr0 = dm320_getreg8(DM320_USB_PERCSR0);
            usbtrace(TRACE_INTENTRY(DM320_TRACEINTID_CONTROL), csr0);
            if (csr0 == 0x00)
              {
                /* Check if we need to set the peripheral address */

                if (!priv->paddrset && priv->paddr != 0)
                  {
                    dm320_putreg8(priv->paddr, DM320_USB_FADDR);
                    priv->paddrset = 1;
                    break;
                  }
              }

            if ((csr0 & USB_PERCSR0_RXPKTRDY) != 0)
              {
                usbtrace(TRACE_INTENTRY(DM320_TRACEINTID_RXPKTRDY), csr0);
                (void)dm320_getreg8(DM320_USB_COUNT0);
                dm320_ep0setup(priv);
              }
            else if ((csr0 & USB_PERCSR0_SENTST) != 0)
              {
                usbtrace(TRACE_INTENTRY(DM320_TRACEINTID_TXFIFOSTALL), csr0);
                dm320_putreg8(0, DM320_USB_PERCSR0);
              }
            else if ((csr0 & USB_PERCSR0_SETEND) != 0)
              {
                usbtrace(TRACE_INTENTRY(DM320_TRACEINTID_TXFIFOSETEND), csr0);
                dm320_putreg8(USB_PERCSR0_CLRSETEND, DM320_USB_PERCSR0);
              }
            else if ((csr0 & USB_PERCSR0_TXPKTRDY) != 0)
              {
                usbtrace(TRACE_INTENTRY(DM320_TRACEINTID_TXPKTRDY), csr0);
              }
            else
              {
                /* Now ignore these unknown interrupts */

                dm320_putreg8(USB_PERCSR0_CLRRXRDY | USB_PERCSR0_DATAEND, DM320_USB_PERCSR0); 
                usbtrace(TRACE_INTENTRY(DM320_TRACEINTID_UNKNOWN), csr0);
              }
          }
          break;

        case USB_INT_RXFIFO:
          {
            usbtrace(TRACE_INTENTRY(DM320_TRACEINTID_RXFIFO), 0);
            privep =  &priv->eplist[DM320_EPBULKOUT];
            if (!dm320_rqempty(privep))
              {
                dm320_rdrequest(privep);
              }
            else
              {
                ullvdbg("Pending data on OUT endpoint\n");
                priv->rxpending = 1;
              }
          }
          break;

        case USB_INT_TXFIFO:
          {
            usbtrace(TRACE_INTENTRY(DM320_TRACEINTID_TXFIFO), 0);
#ifdef PIPE_STALL
            dm320_putreg8(DM320_EPBULKIN, DM320_USB_INDEX);
            if (dm320_getreg8(DM320_USB_PERTXCSR1) & USB_TXCSR1_SENTST)
              {
                dm320_putreg8(dm320_getreg8(DM320_USB_PERTXCSR1) & ~USB_TXCSR1_SENTST, DM320_USB_PERTXCSR1);
                dm320_putreg8(dm320_getreg8(DM320_USB_PERTXCSR1) & ~USB_TXCSR1_SENDST, DM320_USB_PERTXCSR1);
              }
#endif
            if (priv->usbdev.speed == USB_SPEED_UNKNOWN)
              {
                priv->usbdev.speed = USB_SPEED_FULL;
              }
            privep = &priv->eplist[DM320_EPBULKIN];

            if (!dm320_rqempty(privep))
              {
                (void)dm320_wrrequest(privep);
              }
          }
          break;

        case USB_INT_SOF:
          usbtrace(TRACE_INTENTRY(DM320_TRACEINTID_SOF), 0);
          break;

        case USB_INT_DISCONNECTED:
          usbtrace(TRACE_INTENTRY(DM320_TRACEINTID_DISCONNECTED), 0);
          break;

        case USB_INT_SUSPEND:
          usbtrace(TRACE_INTENTRY(DM320_TRACEINTID_SUSPEND), 0);
          break;

        default:
          usbtrace(TRACE_INTENTRY(DM320_TRACEINTID_UNKNOWN), 0);
          break;
        }

      intstatus = intstatus & ~priorityint;

    }
  while (intstatus != USB_INT_NOINTERRUPT);
  return OK;
}

/*******************************************************************************
 * Name: dm320_attachinterrupt
 *
 * Description:
 *   Attach GIO interrtup handler
 *
 *******************************************************************************/

static int dm320_attachinterrupt(int irq, FAR void *context)
{
  struct dm320_usbdev_s *priv = &g_usbdev;
  uint16_t gio;

  /* Check if the USB device was connected to or disconnected from a host */

  gio = dm320_getreg16(DM320_GIO_BITSET0);
  usbtrace(TRACE_INTENTRY(DM320_TRACEINTID_ATTACH), gio);
  if ((gio & (1 << CONFIG_DM320_GIO_USBATTACH)) == 0)
    {
      /* The host is disconnected */

      if (priv->attached)
        {
          /* We have detected a transition from attached to detached */

          usbtrace(TRACE_INTDECODE(DM320_TRACEINTID_DETACHED), 0);

          priv->usbdev.speed = USB_SPEED_UNKNOWN;
          priv->attached = 0;

          dm320_putreg16(dm320_getreg16(DM320_CLKC_LPCTL1) | 0x0010, DM320_CLKC_LPCTL1);
          if ((dm320_getreg8(DM320_USB_PERTXCSR1) & USB_TXCSR1_FIFOEMP))
            {
              dm320_putreg8(USB_TXCSR1_FLFIFO, DM320_USB_PERTXCSR1);
              up_mdelay(5);
            }
        }
    }
  else if (!priv->attached)
    {
      /* We have a transition from unattached to attached */

      usbtrace(TRACE_INTDECODE(DM320_TRACEINTID_ATTACHED), 0);

      priv->usbdev.speed = USB_SPEED_UNKNOWN;
      dm320_ctrlinitialize(priv);

      dm320_putreg16(dm320_getreg16(DM320_INTC_FISEL0) & 0x7f, DM320_INTC_FISEL0);
      dm320_putreg16(dm320_getreg16(DM320_INTC_EINT0) | 0x80, DM320_INTC_EINT0);

      priv->usbdev.speed  = USB_SPEED_UNKNOWN;
      priv->paddrset      = 0;
      priv->paddr         = 0;
      priv->attached      = 1;
    }
  return OK;
}

/*******************************************************************************
 * Name: dm320_epreset
 *******************************************************************************/

static void dm320_epreset(unsigned int index)
{
  dm320_putreg8(index, DM320_USB_INDEX);
  dm320_putreg8(USB_PERCSR0_CLRSETEND | USB_PERCSR0_CLRRXRDY, DM320_USB_PERCSR0);
  dm320_putreg8(USB_CSR2_FLFIFO, DM320_USB_CSR2);
  dm320_putreg8(USB_CSR2_FLFIFO, DM320_USB_CSR2);
}

/*******************************************************************************
 * Name: dm320_epinitialize
 *
 * Description:
 *   Initialize endpoints.  This is logically a part of dm320_ctrlinitialize
 *
 *******************************************************************************/

static inline void dm320_epinitialize(struct dm320_usbdev_s *priv)
{
  uint16_t  offset;     /* Full USB buffer offset */
  uint8_t addrhi;     /* MS bytes of ofset */
  uint8_t addrlo;     /* LS bytes of offset */
  int     i;

  /* Initialize endpoint 0 */

  dm320_putreg8(USB_EP0_SELECT, DM320_USB_INDEX); 
  dm320_putreg8(USB_PERCSR0_CLRSETEND|USB_PERCSR0_CLRRXRDY, DM320_USB_PERCSR0);
  dm320_putreg8(USB_CSR2_FLFIFO, DM320_USB_CSR2);
  dm320_putreg8(USB_CSR2_FLFIFO, DM320_USB_CSR2);

  /* EP0 Fifo size/address (ofset == 0) */

  dm320_putreg8(0x00, DM320_USB_TXFIFO1);
  dm320_putreg8(0x00, DM320_USB_RXFIFO1);
  dm320_putreg8(g_epinfo[0].fifo, DM320_USB_TXFIFO2);
  dm320_putreg8(USB_TXFIFO2_SZ_64, DM320_USB_RXFIFO2);

  /* EP0 max packet size */

  dm320_putreg8(g_epinfo[0].maxpacket >> 3, DM320_USB_TXMAXP);
  dm320_putreg8(g_epinfo[0].maxpacket >> 3, DM320_USB_RXMAXP);

  /* Setup bulk-in, bulk-out, iso-in, iso-out, and intr endpoints using the
   * g_epinfo[] array.
   */

  offset = DM320_EP0MAXPACKET;      /* move to next buffer position */
  for (i = 1; i < DM320_NENDPOINTS; i++)
    {
      dm320_putreg8(g_epinfo[i].addr & 0x0f, DM320_USB_INDEX);

      addrlo = (offset >> 8) & 0xff;
      addrhi = (offset >= 2048) ? 1 : 0;

      /* Configure IN endpoints (device-to-host) */

      if (USB_EPIN(g_epinfo[i].addr))
        {
          /* Initialize TX endpoint */

          dm320_putreg8(USB_TXCSR1_CLRDATTOG|USB_TXCSR1_FLFIFO|USB_TXCSR1_UNDERRUN,
                  DM320_USB_PERTXCSR1);
          dm320_putreg8(USB_TXCSR1_FLFIFO, DM320_USB_PERTXCSR1);
          dm320_putreg8(USB_TXCSR2_FRDATTOG|USB_TXCSR2_MODE_TX, DM320_USB_TXCSR2);

          /* FIFO address, max packet size, dual/single buffered */

          dm320_putreg8(addrhi, DM320_USB_TXFIFO1);
          dm320_putreg8(addrhi|g_epinfo[i].fifo, DM320_USB_TXFIFO2);

          /* TX endpoint max packet size */

          dm320_putreg8(g_epinfo[i].maxpacket >> 3, DM320_USB_TXMAXP);
        }

      /* Configure OUT endpoints (host-to-device) */

      else
        {
          /* Initialize RX endpoint */

          dm320_putreg8(USB_PERRXCSR1_CLRDATTOG|USB_PERRXCSR1_FLFIFO,
                  DM320_USB_PERRXCSR1);
          dm320_putreg8(USB_PERRXCSR1_FLFIFO, DM320_USB_PERRXCSR1);
          dm320_putreg8(0x00, DM320_USB_PERRXCSR2);

          /* FIFO address, max packet size, dual/single buffered */

          dm320_putreg8(addrhi, DM320_USB_RXFIFO1);
          dm320_putreg8(addrhi|g_epinfo[i].fifo | USB_RXFIF02_DPB, DM320_USB_RXFIFO2);

          /* RX endpoint max packet size */

          dm320_putreg8(g_epinfo[i].maxpacket >> 3, DM320_USB_RXMAXP);
        }
      offset += g_epinfo[i].maxpacket;
    }
}

/*******************************************************************************
 * Name: dm320_ctrlinitialize
 *
 * Description:
 *   Initialize the DM320 USB controller for peripheral mode operation .
 *
 *******************************************************************************/

static void dm320_ctrlinitialize(FAR struct dm320_usbdev_s *priv)
{
  /* Setup the USB controller for operation as a periperhal *******************/
  /* Enable USB clock */

  dm320_putreg16(dm320_getreg16(DM320_CLKC_MOD2) | 0x0060, DM320_CLKC_MOD2);

  /* Disable USB Power down mode */

  dm320_putreg16(dm320_getreg16(DM320_CLKC_LPCTL1) & 0xFFEF, DM320_CLKC_LPCTL1);

  /* Put USB controller in peripheral mode */

  dm320_putreg32(0x00000000, DM320_AHB_USBCTL);
  dm320_putreg8(USB_DEVCTL_SESSREQ, DM320_USB_DEVCTL);

  /* Reset USB controller registers */

  dm320_putreg8(0x00, DM320_USB_FADDR);      /* Reset peripheral address register */
  dm320_putreg8(0x00, DM320_USB_POWER);      /* Reset power control register */

  /* Initialize interrupts *****************************************************/

  up_maskack_irq(DM320_IRQ_USB0);      /* Clear USB controller interrupt */
  up_maskack_irq(DM320_IRQ_USB1);      /* Clear USB DMA interrupt flag */

  dm320_getreg8(DM320_USB_INTRTX1);          /* Clear TX interrupt */
  dm320_getreg8(DM320_USB_INTRRX1);          /* Clear RX interrupt */
  dm320_getreg8(DM320_USB_INTRUSB);          /* Clear control interrupt */
  dm320_getreg8(DM320_USB_DMAINTR);

  /* Enable USB interrupts */

  dm320_putreg8((DM320_EPBULKIN << 1), DM320_USB_INTRRX1E);
  dm320_putreg8((DM320_EPBULKOUT << 1) | USB_EP0, DM320_USB_INTRTX1E);
  dm320_putreg8(USB_INT_RESET|USB_INT_RESUME|USB_INT_SUSPEND|USB_INT_SESSRQ|USB_INT_SOF,
          DM320_USB_INTRUSBE); 

  /* Initialize endpoints ******************************************************/

  dm320_epinitialize(priv);

  /* Peripheral address has not yet been set */

  priv->paddr = 0;
  dm320_putreg8(0, DM320_USB_FADDR);

  /* Finished -- set default endpoint as EP0*/

  dm320_putreg8(USB_EP0_SELECT, DM320_USB_INDEX);
}

/*******************************************************************************
 * Endpoint Methods
 *******************************************************************************/

/*******************************************************************************
 * Name: dm320_epconfigure
 *
 * Description:
 *   Configure endpoint, making it usable
 *
 * Input Parameters:
 *   ep   - the struct usbdev_ep_s instance obtained from allocep()
 *   desc - A struct usb_epdesc_s instance describing the endpoint
 *   last - true if this this last endpoint to be configured.  Some hardware
 *          needs to take special action when all of the endpoints have been
 *          configured.
 *
 *******************************************************************************/

static int dm320_epconfigure(FAR struct usbdev_ep_s *ep,
                             FAR const struct usb_epdesc_s *desc,
                             bool last)
{
  FAR struct dm320_ep_s *privep = (FAR struct dm320_ep_s *)ep;

  /* Retain what we need from the descriptor */

  usbtrace(TRACE_EPCONFIGURE, privep->epphy);
  DEBUGASSERT(desc->addr == ep->eplog);
  return OK;
}

/*******************************************************************************
 * Name: dm320_epdisable
 *
 * Description:
 *   The endpoint will no longer be used
 *
 *******************************************************************************/

static int dm320_epdisable(FAR struct usbdev_ep_s *ep)
{
  FAR struct dm320_ep_s *privep = (FAR struct dm320_ep_s *)ep;
  irqstate_t flags;

#ifdef CONFIG_DEBUG
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(DM320_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif
  usbtrace(TRACE_EPDISABLE, privep->epphy);

  /* Cancel any ongoing activity and reset the endpoint */

  flags = irqsave();
  dm320_cancelrequests(privep);
  dm320_epreset(privep->epphy);

  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Name: dm320_epallocreq
 *
 * Description:
 *   Allocate an I/O request
 *
 *******************************************************************************/

static FAR struct usbdev_req_s *dm320_epallocreq(FAR struct usbdev_ep_s *ep)
{
  FAR struct dm320_req_s *privreq;

#ifdef CONFIG_DEBUG
  if (!ep)
    {
      return NULL;
    }
#endif
  usbtrace(TRACE_EPALLOCREQ, ((FAR struct dm320_ep_s *)ep)->epphy);

  privreq = (FAR struct dm320_req_s *)malloc(sizeof(struct dm320_req_s));
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(DM320_TRACEERR_ALLOCFAIL), 0);
      return NULL;
    }

  memset(privreq, 0, sizeof(struct dm320_req_s));
  return &privreq->req;
}

/*******************************************************************************
 * Name: dm320_epfreereq
 *
 * Description:
 *   Free an I/O request
 *
 *******************************************************************************/

static void dm320_epfreereq(FAR struct usbdev_ep_s *ep, FAR struct usbdev_req_s *req)
{
  FAR struct dm320_req_s *privreq = (FAR struct dm320_req_s *)req;

#ifdef CONFIG_DEBUG
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(DM320_TRACEERR_INVALIDPARMS), 0);
      return;
    }
#endif

  usbtrace(TRACE_EPFREEREQ, ((FAR struct dm320_ep_s *)ep)->epphy);
  free(privreq);
}

/*******************************************************************************
 * Name: dm320_epallocbuffer
 *
 * Description:
 *   Allocate an I/O buffer
 *
 *******************************************************************************/

#ifdef CONFIG_USBDEV_DMA
static void *dm320_epallocbuffer(FAR struct usbdev_ep_s *ep, unsigned bytes)
{
  usbtrace(TRACE_EPALLOCBUFFER, privep->epphy);

#ifdef CONFIG_USBDEV_DMAMEMORY
  return usbdev_dma_alloc(bytes);
#else
  return malloc(bytes);
#endif
}
#endif

/*******************************************************************************
 * Name: dm320_epfreebuffer
 *
 * Description:
 *   Free an I/O buffer
 *
 *******************************************************************************/

#ifdef CONFIG_USBDEV_DMA
static void dm320_epfreebuffer(FAR struct usbdev_ep_s *ep, FAR void *buf)
{
  usbtrace(TRACE_EPFREEBUFFER, privep->epphy);

#ifdef CONFIG_USBDEV_DMAMEMORY
  usbdev_dma_free(buf);
#else
  free(buf);
#endif
}
#endif

/*******************************************************************************
 * Name: dm320_epsubmit
 *
 * Description:
 *   Submit an I/O request to the endpoint
 *
 *******************************************************************************/

static int dm320_epsubmit(FAR struct usbdev_ep_s *ep, FAR struct usbdev_req_s *req)
{
  FAR struct dm320_req_s *privreq = (FAR struct dm320_req_s *)req;
  FAR struct dm320_ep_s *privep = (FAR struct dm320_ep_s *)ep;
  FAR struct dm320_usbdev_s *priv;
  irqstate_t flags;
  int ret = OK;

#ifdef CONFIG_DEBUG
  if (!req || !req->callback || !req->buf || !ep)
    {
      usbtrace(TRACE_DEVERROR(DM320_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif
  usbtrace(TRACE_EPSUBMIT, privep->epphy);
  priv = privep->dev;

  if (!priv->driver || priv->usbdev.speed == USB_SPEED_UNKNOWN)
    {
      usbtrace(TRACE_DEVERROR(DM320_TRACEERR_NOTCONFIGURED), 0);
      return -ESHUTDOWN;
    }

  req->result = -EINPROGRESS;
  req->xfrd   = 0;
  flags       = irqsave();

  /* Check for NULL packet */

  if (req->len == 0 && (privep->in || privep->epphy == 3))
    {
      usbtrace(TRACE_DEVERROR(DM320_TRACEERR_NULLPACKET), 0);
      dm320_putreg8(dm320_getreg8(DM320_USB_PERTXCSR1) | USB_TXCSR1_TXPKTRDY, DM320_USB_PERTXCSR1);
      dm320_abortrequest(privep, privreq, OK);
    }

  /* If we are stalled, then drop all requests on the floor */

  else if (privep->stalled)
    {
      dm320_abortrequest(privep, privreq, -EBUSY);
      ret = -EBUSY;
    }

  /* Handle zero-length transfers on EP0 */

  else if (privep->epphy == 0 && req->len == 0)
    {
      /* Nothing to transfer -- exit success, with zero bytes transferred */

      usbtrace(TRACE_COMPLETE(privep->epphy), privreq->req.xfrd);
      dm320_abortrequest(privep, privreq, OK);
    }

  /* Handle IN (device-to-host) requests */

  else if ((privep->in) || privep->epphy == 3)
    {
      /* Add the new request to the request queue for the IN endpoint */

      dm320_rqenqueue(privep, privreq);
      usbtrace(TRACE_INREQQUEUED(privep->epphy), privreq->req.len);
      ret = dm320_wrrequest(privep);
    }

  /* Handle OUT (host-to-device) requests */

  else
    {
      /* Add the new request to the request queue for the OUT endpoint */

      privep->txnullpkt = 0;
      dm320_rqenqueue(privep, privreq);
      usbtrace(TRACE_OUTREQQUEUED(privep->epphy), privreq->req.len);

      /* This there a incoming data pending the availability of a request? */

      if (priv->rxpending)
        {
          ret = dm320_rdrequest(privep);
          priv->rxpending = 0;
        }
    }

  irqrestore(flags);
  return ret;
}

/*******************************************************************************
 * Name: dm320_epcancel
 *
 * Description:
 *   Cancel an I/O request previously sent to an endpoint
 *
 *******************************************************************************/

static int dm320_epcancel(struct usbdev_ep_s *ep, FAR struct usbdev_req_s *req)
{
  FAR struct dm320_ep_s *privep = (FAR struct dm320_ep_s *)ep;
  FAR struct dm320_usbdev_s *priv;
  irqstate_t flags;

#ifdef CONFIG_DEBUG
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(DM320_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif
  usbtrace(TRACE_EPCANCEL, privep->epphy);
  priv = privep->dev;

  flags = irqsave();
  dm320_cancelrequests(privep);
  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Device Methods
 *******************************************************************************/

/*******************************************************************************
 * Name: dm320_allocep
 *
 * Description:
 *   Allocate an endpoint matching the parameters
 *
 * Input Parameters:
 *   eplog  - 7-bit logical endpoint number (direction bit ignored).  Zero means
 *            that any endpoint matching the other requirements will suffice.  The
 *            assigned endpoint can be found in the eplog field.
 *   in     - true: IN (device-to-host) endpoint requested
 *   eptype - Endpoint type.  One of {USB_EP_ATTR_XFER_ISOC, USB_EP_ATTR_XFER_BULK,
 *            USB_EP_ATTR_XFER_INT}
 *
 *******************************************************************************/

static FAR struct usbdev_ep_s *dm320_allocep(FAR struct usbdev_s *dev, uint8_t eplog,
                                             bool in, uint8_t eptype)
{
  FAR struct dm320_usbdev_s *priv = (FAR struct dm320_usbdev_s *)dev;
  int ndx;

  usbtrace(TRACE_DEVALLOCEP, 0);

  /* Ignore any direction bits in the logical address */

  eplog = USB_EPNO(eplog);

  /* Check all endpoints (except EP0) */

  for (ndx = 1; ndx < DM320_NENDPOINTS; ndx++)
    {
      /* Does this match the endpoint number (if one was provided?) */

      if (eplog != 0 && eplog != USB_EPNO(priv->eplist[ndx].ep.eplog))
        {
          continue;
        }

      /* Does the direction match */

      if (in)
        {
          if (!USB_EPIN(g_epinfo[ndx].addr))
            {
              continue;
            }
        }
      else
        {
          if (!USB_EPOUT(g_epinfo[ndx].addr))
            {
              continue;
            }
        }

      /* Does the type match? */

      if (g_epinfo[ndx].attr == eptype)
        {
           /* Success! */

          return &priv->eplist[ndx].ep;
        }
    }

  usbtrace(TRACE_DEVERROR(DM320_TRACEERR_NOEP), 0);
  return NULL;
}

/*******************************************************************************
 * Name: dm320_freeep
 *
 * Description:
 *   Free the previously allocated endpoint
 *
 *******************************************************************************/

static void dm320_freeep(FAR struct usbdev_s *dev, FAR struct usbdev_ep_s *ep)
{
  FAR struct dm320_ep_s *privep = (FAR struct dm320_ep_s *)ep;
  usbtrace(TRACE_DEVFREEEP, (uint16_t)privep->epphy);

  /* Nothing needs to be done */
}

/*******************************************************************************
 * Name: dm320_getframe
 *
 * Description:
 *   Returns the current frame number
 *
 *******************************************************************************/

static int dm320_getframe(struct usbdev_s *dev)
{
  irqstate_t flags;
  int ret;

  usbtrace(TRACE_DEVGETFRAME, 0);

#ifdef CONFIG_DEBUG
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(DM320_TRACEERR_INVALIDPARMS), 0);
      return -ENODEV;
    }
#endif

  /* Return the contents of the frame register.  Interrupts must be disabled
   * because the operation is not atomic.
   */

  flags = irqsave();
  ret = dm320_getreg8(DM320_USB_FRAME2) << 8;
  ret |= dm320_getreg8(DM320_USB_FRAME1);
  irqrestore(flags);
  return ret;
}

/*******************************************************************************
 * Name: dm320_wakeup
 *
 * Description:
 *   Tries to wake up the host connected to this device
 *
 *******************************************************************************/

static int dm320_wakeup(struct usbdev_s *dev)
{
  irqstate_t flags;
  usbtrace(TRACE_DEVWAKEUP, 0);

  flags = irqsave();
  dm320_putreg8(USB_POWER_RESUME, DM320_USB_POWER);
  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Name: dm320_selfpowered
 *
 * Description:
 *   Sets/clears the device selfpowered feature 
 *
 *******************************************************************************/

static int dm320_selfpowered(struct usbdev_s *dev, bool selfpowered)
{
  struct dm320_usbdev_s *priv = &g_usbdev;

  usbtrace(TRACE_DEVSELFPOWERED, (uint16_t)selfpowered);

#ifdef CONFIG_DEBUG
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(DM320_TRACEERR_INVALIDPARMS), 0);
      return -ENODEV;
    }
#endif

  priv->selfpowered = selfpowered;
  return OK;
}

/*******************************************************************************
 * Name: dm320_pullup
 *
 * Description:
 *    Software-controlled connect to/disconnect from USB host
 *
 *******************************************************************************/

#ifdef CONFIG_DM320_GIO_USBDPPULLUP
static int dm320_pullup(struct usbdev_s *dev, bool enable)
{
  irqstate_t flags;

  usbtrace(TRACE_DEVPULLUP, (uint16_t)enable);

  flags = irqsave();
  if (enable)
    {
      GIO_SET_OUTPUT(CONFIG_DM320_GIO_USBDPPULLUP); /* Set D+ pullup */
    }
  else
    {
      GIO_CLEAR_OUTPUT(CONFIG_DM320_GIO_USBDPPULLUP); /* Clear D+ pullup */
    }

  irqrestore(flags);
  return OK;
}
#endif

/*******************************************************************************
 * Public Functions
 *******************************************************************************/

/*******************************************************************************
 * Name: up_usbinitialize
 *
 * Description:
 *   Initialize USB hardware
 *
 *******************************************************************************/

void up_usbinitialize(void)
{
  struct dm320_usbdev_s *priv = &g_usbdev;
  struct dm320_ep_s *privep;
#ifdef CONFIG_DEBUG_USB
  uint16_t chiprev;
#endif
  int i;

  usbtrace(TRACE_DEVINIT, 0);

  /* Initialize the device state structure */

  memset(priv, 0, sizeof(struct dm320_usbdev_s));
  priv->usbdev.ops = &g_devops;

#ifdef CONFIG_DEBUG_USB
  chiprev = dm320_getreg16(DM320_BUSC_REVR);
  ulldbg("DM320 revision : %d.%d\n", chiprev >> 4, chiprev & 0x0f);
#endif

  /* Enable USB clock & GIO clock  */

  dm320_putreg16(dm320_getreg16(DM320_CLKC_MOD2) | 0x0060, DM320_CLKC_MOD2); 
  dm320_putreg16(dm320_getreg16(DM320_CLKC_DIV4) | (((4) - 1) << 8) | ((1) - 1), DM320_CLKC_DIV4);

  /* Initialize  D+ pullup control GIO */

  GIO_OUTPUT(CONFIG_DM320_GIO_USBDPPULLUP);
  GIO_SET_OUTPUT(CONFIG_DM320_GIO_USBDPPULLUP);

  /* Initilialize USB attach GIO */

  GIO_INTERRUPT(CONFIG_DM320_GIO_USBATTACH);
  GIO_BOTHEDGES(CONFIG_DM320_GIO_USBATTACH);
  dm320_putreg16(dm320_getreg16(DM320_GIO_CHAT0) | (1 << CONFIG_DM320_GIO_USBATTACH), DM320_GIO_CHAT0);

  /* Attach host attach GIO interrupt */

  if (irq_attach(IRQ_USBATTACH, dm320_attachinterrupt) != 0)
    {
      usbtrace(TRACE_DEVERROR(DM320_TRACEERR_ATTACHIRQREG), 0);
      goto errout;
    }

  /* Attach USB controller core interrupt handler -- interrupts will be
   * enabled when the driver is bound
   */

  if (irq_attach(DM320_IRQ_USB1, dm320_ctlrinterrupt) != 0)
    {
      usbtrace(TRACE_DEVERROR(DM320_TRACEERR_COREIRQREG), 0);
      goto errout;
    }

  /* Initialize the DM320 USB controller for peripheral mode operation. */

  dm320_ctrlinitialize(priv);

  /* Perform endpoint initialization */

  for (i = 0; i < DM320_NENDPOINTS; i++)
    {
      /* Set up the standard stuff */

      privep           = &priv->eplist[i];
      memset(privep, 0, sizeof(struct dm320_ep_s));
      privep->ep.ops   = &g_epops;
      privep->dev      = priv;

      /* The index, i, is the physical endpoint address;  Map this
       * to a logical endpoint address usable by the class driver.
       */

      privep->epphy    = i;
      privep->ep.eplog = g_epinfo[i].addr;

      /* Setup the endpoint-specific stuff */

      priv->eplist[i].ep.maxpacket = g_epinfo[i].maxpacket;
      if (USB_EPIN(g_epinfo[i].addr))
        {
          priv->eplist[i].in = 1;
        }

      /* Reset the endpoint */

      dm320_epreset(privep->epphy);
    }

  /* Expose only the standard EP0 */

  priv->usbdev.ep0 = &priv->eplist[0].ep;

  /* For 'B' device initiate session request protocol */

  dm320_putreg8(USB_DEVCTL_SESSREQ, DM320_USB_DEVCTL);
  return;

errout:
  up_usbuninitialize();
}

/*******************************************************************************
 * Name: up_usbuninitialize
 *******************************************************************************/

void up_usbuninitialize(void)
{
  struct dm320_usbdev_s *priv = &g_usbdev;

  usbtrace(TRACE_DEVUNINIT, 0);
  if (priv->driver)
    {
      usbtrace(TRACE_DEVERROR(DM320_TRACEERR_DRIVERREGISTERED), 0);
      usbdev_unregister(priv->driver);
    }

  priv->usbdev.speed = USB_SPEED_UNKNOWN;
  dm320_putreg16(dm320_getreg16(DM320_CLKC_LPCTL1) | 0x0010, DM320_CLKC_LPCTL1);

  /* Disable and detach IRQs */

  up_disable_irq(IRQ_USBATTACH);
  up_disable_irq(DM320_IRQ_USB1);

  irq_detach(IRQ_USBATTACH);
  irq_detach(DM320_IRQ_USB1);
}

/************************************************************************************
 * Name: usbdevclass_register
 *
 * Description:
 *   Register a USB device class driver. The class driver's bind() method will be
 *   called to bind it to a USB device driver.
 *
 ************************************************************************************/

int usbdev_register(FAR struct usbdevclass_driver_s *driver)
{
  int ret;

  usbtrace(TRACE_DEVREGISTER, 0);

#ifdef CONFIG_DEBUG
  if (!driver || (driver->speed != USB_SPEED_FULL) || !driver->ops->bind ||
      !driver->ops->unbind || !driver->ops->setup)
    {
      usbtrace(TRACE_DEVERROR(DM320_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }

  if (g_usbdev.driver)
    {
      usbtrace(TRACE_DEVERROR(DM320_TRACEERR_DRIVER), 0);
      return -EBUSY;
    }
#endif

  /* Hook up the driver */

 g_usbdev.driver = driver;

  /* Then bind the class driver */

  ret = CLASS_BIND(driver, &g_usbdev.usbdev);
  if (ret)
    {
      usbtrace(TRACE_DEVERROR(DM320_TRACEERR_BINDFAILED), (uint16_t)-ret);
      g_usbdev.driver = NULL;
      return ret;
    }

  /* Enable host detection and ep0 RX/TX */

  dm320_epreset(0);
  dm320_putreg8(USB_EP0, DM320_USB_INTRTX1E);
  dm320_putreg8(USB_INT_RESET|USB_INT_RESUME|USB_INT_SUSPEND|USB_INT_SESSRQ|USB_INT_SOF,
          DM320_USB_INTRUSBE);

  /* Enable interrupts */

  up_enable_irq(IRQ_USBATTACH);
  up_enable_irq(DM320_IRQ_USB1);
  return OK;
}

/************************************************************************************
 * Name: usbdev_unregister
 *
 * Description:
 *   Un-register usbdev class driver.If the USB device is connected to a USB host,
 *   it will first disconnect().  The driver is also requested to unbind() and clean
 *   up any device state, before this procedure finally returns.
 *
 ************************************************************************************/

int usbdev_unregister(FAR struct usbdevclass_driver_s *driver)
{
  usbtrace(TRACE_DEVUNREGISTER, 0);

#ifdef CONFIG_DEBUG
  if (driver != g_usbdev.driver)
    {
      usbtrace(TRACE_DEVERROR(DM320_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Unbind the class driver */

  CLASS_UNBIND(driver, &g_usbdev.usbdev);

  /* Disable IRQs */

  up_disable_irq(IRQ_USBATTACH);
  up_disable_irq(DM320_IRQ_USB1);

  /* Unhook the driver */

  g_usbdev.driver = NULL;
  return OK;
}


