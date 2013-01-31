/*******************************************************************************
 * arch/arm/src/lpc31xx/lpc31_usbdev.c
 *
 *   Authors: David Hewson
 *            Gregory Nutt <gnutt@nuttx.org>
 *
 * Part of the NuttX OS and based, in part, on the LPC2148 USB driver:
 *
 *   Copyright (C) 2010-2013 Gregory Nutt. All rights reserved.
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

#include "lpc31_usbotg.h"
#include "lpc31_evntrtr.h"
#include "lpc31_syscreg.h"

/*******************************************************************************
 * Definitions
 *******************************************************************************/

/* Configuration ***************************************************************/

#ifndef CONFIG_LPC31_USBDEV_EP0_MAXSIZE
#  define CONFIG_LPC31_USBDEV_EP0_MAXSIZE 64
#endif

#ifndef  CONFIG_USBDEV_MAXPOWER
#  define CONFIG_USBDEV_MAXPOWER 100  /* mA */
#endif

/* Extremely detailed register debug that you would normally never want
 * enabled.
 */

#undef CONFIG_LPC31_USBDEV_REGDEBUG

/* Enable reading SOF from interrupt handler vs. simply reading on demand.  Probably
 * a bad idea... Unless there is some issue with sampling the SOF from hardware
 * asynchronously.
 */

#ifdef CONFIG_LPC31_USBDEV_FRAME_INTERRUPT
#  define USB_FRAME_INT USBDEV_USBINTR_SRE
#else
#  define USB_FRAME_INT 0
#endif

#ifdef CONFIG_DEBUG
#  define USB_ERROR_INT USBDEV_USBINTR_UEE
#else
#  define USB_ERROR_INT 0
#endif

/* Debug ***********************************************************************/

/* Trace error codes */

#define LPC31_TRACEERR_ALLOCFAIL            0x0001
#define LPC31_TRACEERR_BADCLEARFEATURE      0x0002
#define LPC31_TRACEERR_BADDEVGETSTATUS      0x0003
#define LPC31_TRACEERR_BADEPNO              0x0004
#define LPC31_TRACEERR_BADEPGETSTATUS       0x0005
#define LPC31_TRACEERR_BADEPTYPE            0x0006
#define LPC31_TRACEERR_BADGETCONFIG         0x0007
#define LPC31_TRACEERR_BADGETSETDESC        0x0008
#define LPC31_TRACEERR_BADGETSTATUS         0x0009
#define LPC31_TRACEERR_BADSETADDRESS        0x000a
#define LPC31_TRACEERR_BADSETCONFIG         0x000b
#define LPC31_TRACEERR_BADSETFEATURE        0x000c
#define LPC31_TRACEERR_BINDFAILED           0x000d
#define LPC31_TRACEERR_DISPATCHSTALL        0x000e
#define LPC31_TRACEERR_DRIVER               0x000f
#define LPC31_TRACEERR_DRIVERREGISTERED     0x0010
#define LPC31_TRACEERR_EP0SETUPSTALLED      0x0011
#define LPC31_TRACEERR_EPINNULLPACKET       0x0012
#define LPC31_TRACEERR_EPOUTNULLPACKET      0x0013
#define LPC31_TRACEERR_INVALIDCTRLREQ       0x0014
#define LPC31_TRACEERR_INVALIDPARMS         0x0015
#define LPC31_TRACEERR_IRQREGISTRATION      0x0016
#define LPC31_TRACEERR_NOEP                 0x0017
#define LPC31_TRACEERR_NOTCONFIGURED        0x0018
#define LPC31_TRACEERR_REQABORTED           0x0019

/* Trace interrupt codes */

#define LPC31_TRACEINTID_USB                0x0001
#define LPC31_TRACEINTID_CLEARFEATURE       0x0002
#define LPC31_TRACEINTID_DEVGETSTATUS       0x0003
#define LPC31_TRACEINTID_DEVRESET           0x0004
#define LPC31_TRACEINTID_DISPATCH           0x0005
#define LPC31_TRACEINTID_EP0COMPLETE        0x0006
#define LPC31_TRACEINTID_EP0NAK             0x0007
#define LPC31_TRACEINTID_EP0SETUP           0x0008
#define LPC31_TRACEINTID_EPGETSTATUS        0x0009
#define LPC31_TRACEINTID_EPIN               0x000a
#define LPC31_TRACEINTID_EPINQEMPTY         0x000b
#define LPC31_TRACEINTID_EP0INSETADDRESS    0x000c
#define LPC31_TRACEINTID_EPOUT              0x000d
#define LPC31_TRACEINTID_EPOUTQEMPTY        0x000e
#define LPC31_TRACEINTID_EP0SETUPSETADDRESS 0x000f
#define LPC31_TRACEINTID_FRAME              0x0010
#define LPC31_TRACEINTID_GETCONFIG          0x0011
#define LPC31_TRACEINTID_GETSETDESC         0x0012
#define LPC31_TRACEINTID_GETSETIF           0x0013
#define LPC31_TRACEINTID_GETSTATUS          0x0014
#define LPC31_TRACEINTID_IFGETSTATUS        0x0015
#define LPC31_TRACEINTID_SETCONFIG          0x0016
#define LPC31_TRACEINTID_SETFEATURE         0x0017
#define LPC31_TRACEINTID_SUSPENDED          0x0018
#define LPC31_TRACEINTID_RESUMED            0x0019
#define LPC31_TRACEINTID_SYNCHFRAME         0x001a

/* Hardware interface **********************************************************/

/* This represents a Endpoint Transfer Descriptor - note these must be 32 byte aligned */
struct lpc31_dtd_s
{
  volatile uint32_t       nextdesc;      /* Address of the next DMA descripto in RAM */
  volatile uint32_t       config;        /* Misc. bit encoded configuration information */
  uint32_t                buffer0;       /* Buffer start address */
  uint32_t                buffer1;       /* Buffer start address */
  uint32_t                buffer2;       /* Buffer start address */
  uint32_t                buffer3;       /* Buffer start address */
  uint32_t                buffer4;       /* Buffer start address */
  uint32_t                xfer_len;      /* Software only - transfer len that was queued */
};

/* DTD nextdesc field*/
#define DTD_NEXTDESC_INVALID         (1 << 0)    /* Bit 0     : Next Descriptor Invalid */

/* DTD config field */
#define DTD_CONFIG_LENGTH(n)         ((n) << 16) /* Bits 16-31 : Total bytes to transfer */
#define DTD_CONFIG_IOC               (1 << 15)   /* Bit 15     : Interrupt on Completion */
#define DTD_CONFIG_MULT_VARIABLE     (0 << 10)   /* Bits 10-11 : Number of packets executed per transacation descriptor (override) */
#define DTD_CONFIG_MULT_NUM(n)       ((n) << 10)
#define DTD_CONFIG_ACTIVE            (1 << 7)    /* Bit 7      : Status Active */
#define DTD_CONFIG_HALTED            (1 << 6)    /* Bit 6      : Status Halted */
#define DTD_CONFIG_BUFFER_ERROR      (1 << 5)    /* Bit 6      : Status Buffer Error */
#define DTD_CONFIG_TRANSACTION_ERROR (1 << 3)    /* Bit 3      : Status Transaction Error */

/* This represents a queue head  - not these must be aligned to a 2048 byte boundary */
struct lpc31_dqh_s
{
  uint32_t                capability;  /* Endpoint capability */
  uint32_t                currdesc;    /* Current dTD pointer */
  struct lpc31_dtd_s      overlay;     /* DTD overlay */
  volatile uint32_t       setup[2];    /* Set-up buffer */
  uint32_t                gap[4];      /* align to 64 bytes */
};

/* DQH capability field */
#define DQH_CAPABILITY_MULT_VARIABLE (0 << 30)    /* Bits 30-31 : Number of packets executed per transaction descriptor */
#define DQH_CAPABILITY_MULT_NUM(n)   ((n) << 30)
#define DQH_CAPABILITY_ZLT           (1 << 29)    /* Bit 29     : Zero Length Termination Select */
#define DQH_CAPABILITY_MAX_PACKET(n) ((n) << 16)  /* Bits 16-29 : Maximum packet size of associated endpoint (<1024) */
#define DQH_CAPABILITY_IOS           (1 << 15)    /* Bit 15     : Interrupt on Setup */

/* Endpoints ******************************************************************/

/* Number of endpoints */
#define LPC31_NLOGENDPOINTS          (4)          /* ep0-3 */
#define LPC31_NPHYSENDPOINTS         (8)          /* x2 for IN and OUT */

/* Odd physical endpoint numbers are IN; even are OUT */
#define LPC31_EPPHYIN(epphy)         (((epphy)&1)!=0)
#define LPC31_EPPHYOUT(epphy)        (((epphy)&1)==0)

#define LPC31_EPPHYIN2LOG(epphy)     (((uint8_t)(epphy)>>1)|USB_DIR_IN)
#define LPC31_EPPHYOUT2LOG(epphy)    (((uint8_t)(epphy)>>1)|USB_DIR_OUT)

/* Endpoint 0 is special... */
#define LPC31_EP0_OUT                (0)
#define LPC31_EP0_IN                 (1)

/* Each endpoint has somewhat different characteristics */
#define LPC31_EPALLSET               (0xff)       /* All endpoints */
#define LPC31_EPOUTSET               (0x55)       /* Even phy endpoint numbers are OUT EPs */
#define LPC31_EPINSET                (0xaa)       /* Odd endpoint numbers are IN EPs */
#define LPC31_EPCTRLSET              (0x03)       /* EP0 IN/OUT are control endpoints */
#define LPC31_EPINTRSET              (0xa8)       /* Interrupt endpoints */
#define LPC31_EPBULKSET              (0xfc)       /* Bulk endpoints */
#define LPC31_EPISOCSET              (0xfc)       /* Isochronous endpoints */

/* Maximum packet sizes for endpoints */
#define LPC31_EP0MAXPACKET           (64)         /* EP0 max packet size (1-64) */
#define LPC31_BULKMAXPACKET          (512)        /* Bulk endpoint max packet (8/16/32/64/512) */
#define LPC31_INTRMAXPACKET          (1024)       /* Interrupt endpoint max packet (1 to 1024) */
#define LPC31_ISOCMAXPACKET          (512)        /* Acutally 1..1023 */

/* The address of the endpoint control register */
#define LPC31_USBDEV_ENDPTCTRL(epphy) (LPC31_USBDEV_ENDPTCTRL0 + ((epphy)>>1)*4)

/* Endpoint bit position in SETUPSTAT, PRIME, FLUSH, STAT, COMPLETE registers */
#define LPC31_ENDPTSHIFT(epphy)      (LPC31_EPPHYIN(epphy) ? (16 + ((epphy) >> 1)) : ((epphy) >> 1))
#define LPC31_ENDPTMASK(epphy)       (1 << LPC31_ENDPTSHIFT(epphy))
#define LPC31_ENDPTMASK_ALL          0x000f000f

/* Request queue operations ****************************************************/

#define lpc31_rqempty(ep)            ((ep)->head == NULL)
#define lpc31_rqpeek(ep)             ((ep)->head)

/*******************************************************************************
 * Private Types
 *******************************************************************************/

/* A container for a request so that the request may be retained in a list */

struct lpc31_req_s
{
  struct usbdev_req_s  req;           /* Standard USB request */
  struct lpc31_req_s  *flink;         /* Supports a singly linked list */
};

/* This is the internal representation of an endpoint */

struct lpc31_ep_s
{
  /* Common endpoint fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_ep_s
   * to struct lpc31_ep_s.
   */

  struct usbdev_ep_s      ep;          /* Standard endpoint structure */

  /* LPC31XX-specific fields */

  struct lpc31_usbdev_s *dev;          /* Reference to private driver data */
  struct lpc31_req_s    *head;         /* Request list for this endpoint */
  struct lpc31_req_s    *tail;
  uint8_t                epphy;        /* Physical EP address */
  uint8_t                stalled:1;    /* 1: Endpoint is stalled */
};

/* This structure retains the state of the USB device controller */

struct lpc31_usbdev_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_s
   * to struct lpc31_usbdev_s.
   */

  struct usbdev_s         usbdev;

  /* The bound device class driver */

  struct usbdevclass_driver_s *driver;

  /* LPC31XX-specific fields */

  uint8_t                 ep0state;      /* State of certain EP0 operations */
  uint8_t                 ep0buf[64];    /* buffer for EP0 short transfers */
  uint8_t                 paddr;         /* Address assigned by SETADDRESS */
  uint8_t                 stalled:1;     /* 1: Protocol stalled */
  uint8_t                 selfpowered:1; /* 1: Device is self powered */
  uint8_t                 paddrset:1;    /* 1: Peripheral addr has been set */
  uint8_t                 attached:1;    /* 1: Host attached */
  uint8_t                 suspended:1;   /* 1: Suspended */
  uint32_t                softprio;      /* Bitset of high priority interrupts */
  uint32_t                epavail;       /* Bitset of available endpoints */
#ifdef CONFIG_LPC31_USBDEV_FRAME_INTERRUPT
  uint32_t                sof;           /* Last start-of-frame */
#endif

  /* The endpoint list */
  struct lpc31_ep_s     eplist[LPC31_NPHYSENDPOINTS];
};

#define EP0STATE_IDLE             0        /* Idle State, leave on receiving a setup packet or epsubmit */
#define EP0STATE_SETUP_OUT        1        /* Setup Packet received - SET/CLEAR */
#define EP0STATE_SETUP_IN         2        /* Setup Packet received - GET */
#define EP0STATE_SHORTWRITE       3        /* Short write without a usb_request */
#define EP0STATE_WAIT_NAK_OUT     4        /* Waiting for Host to illicit status phase (GET) */
#define EP0STATE_WAIT_NAK_IN      5        /* Waiting for Host to illicit status phase (SET/CLEAR) */
#define EP0STATE_WAIT_STATUS_OUT  6        /* Wait for status phase to complete */
#define EP0STATE_WAIT_STATUS_IN   7        /* Wait for status phase to complete */
#define EP0STATE_DATA_IN          8
#define EP0STATE_DATA_OUT         9

/*******************************************************************************
 * Private Function Prototypes
 *******************************************************************************/

/* Register operations ********************************************************/

#if defined(CONFIG_LPC31_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
static uint32_t lpc31_getreg(uint32_t addr);
static void lpc31_putreg(uint32_t val, uint32_t addr);
#else
# define lpc31_getreg(addr)     getreg32(addr)
# define lpc31_putreg(val,addr) putreg32(val,addr)
#endif

static inline void lpc31_clrbits(uint32_t mask, uint32_t addr);
static inline void lpc31_setbits(uint32_t mask, uint32_t addr);
static inline void lpc31_chgbits(uint32_t mask, uint32_t val, uint32_t addr);

/* Request queue operations ****************************************************/

static FAR struct lpc31_req_s *lpc31_rqdequeue(FAR struct lpc31_ep_s *privep);
static bool       lpc31_rqenqueue(FAR struct lpc31_ep_s *privep,
                    FAR struct lpc31_req_s *req);

/* Low level data transfers and request operations *****************************/

static inline void lpc31_writedtd(struct lpc31_dtd_s *dtd, const uint8_t *data,
                     uint32_t nbytes);
static inline void lpc31_queuedtd(uint8_t epphy, struct lpc31_dtd_s *dtd);
static inline void lpc31_ep0xfer(uint8_t epphy, uint8_t *data, uint32_t nbytes);
static void        lpc31_readsetup(uint8_t epphy, struct usb_ctrlreq_s *ctrl);

static inline void lpc31_set_address(struct lpc31_usbdev_s *priv, uint16_t address);

static void        lpc31_flushep(struct lpc31_ep_s *privep);

static int         lpc31_progressep(struct lpc31_ep_s *privep);
static inline void lpc31_abortrequest(struct lpc31_ep_s *privep,
                     struct lpc31_req_s *privreq, int16_t result);
static void        lpc31_reqcomplete(struct lpc31_ep_s *privep,
                     struct lpc31_req_s *privreq, int16_t result);

static void        lpc31_cancelrequests(struct lpc31_ep_s *privep, int16_t status);

/* Interrupt handling **********************************************************/
static struct lpc31_ep_s *lpc31_epfindbyaddr(struct lpc31_usbdev_s *priv,
                     uint16_t eplog);
static void        lpc31_dispatchrequest(struct lpc31_usbdev_s *priv,
                     const struct usb_ctrlreq_s *ctrl);
static void        lpc31_ep0configure(struct lpc31_usbdev_s *priv);
static void        lpc31_usbreset(struct lpc31_usbdev_s *priv);

static inline void lpc31_ep0state(struct lpc31_usbdev_s *priv, uint16_t state);
static void        lpc31_ep0setup(struct lpc31_usbdev_s *priv);

static void        lpc31_ep0complete(struct lpc31_usbdev_s *priv, uint8_t epphy);
static void        lpc31_ep0nak(struct lpc31_usbdev_s *priv, uint8_t epphy);
static bool        lpc31_epcomplete(struct lpc31_usbdev_s *priv, uint8_t epphy);

static int         lpc31_usbinterrupt(int irq, FAR void *context);

/* Endpoint operations *********************************************************/

/* USB device controller operations ********************************************/

static int  lpc31_epconfigure(FAR struct usbdev_ep_s *ep,
              const struct usb_epdesc_s *desc, bool last);
static int  lpc31_epdisable(FAR struct usbdev_ep_s *ep);
static FAR struct usbdev_req_s *lpc31_epallocreq(FAR struct usbdev_ep_s *ep);
static void lpc31_epfreereq(FAR struct usbdev_ep_s *ep,
              FAR struct usbdev_req_s *);
#ifdef CONFIG_USBDEV_DMA
static void *lpc31_epallocbuffer(FAR struct usbdev_ep_s *ep, unsigned bytes);
static void lpc31_epfreebuffer(FAR struct usbdev_ep_s *ep, FAR void *buf);
#endif
static int  lpc31_epsubmit(FAR struct usbdev_ep_s *ep,
              struct usbdev_req_s *req);
static int  lpc31_epcancel(FAR struct usbdev_ep_s *ep,
              struct usbdev_req_s *req);
static int  lpc31_epstall(FAR struct usbdev_ep_s *ep, bool resume);

static FAR struct usbdev_ep_s *lpc31_allocep(FAR struct usbdev_s *dev,
              uint8_t epno, bool in, uint8_t eptype);
static void lpc31_freeep(FAR struct usbdev_s *dev, FAR struct usbdev_ep_s *ep);
static int  lpc31_getframe(struct usbdev_s *dev);
static int  lpc31_wakeup(struct usbdev_s *dev);
static int  lpc31_selfpowered(struct usbdev_s *dev, bool selfpowered);
static int  lpc31_pullup(struct usbdev_s *dev, bool enable);

/*******************************************************************************
 * Private Data
 *******************************************************************************/

/* Since there is only a single USB interface, all status information can be
 * be simply retained in a single global instance.
 */

static struct lpc31_usbdev_s g_usbdev;

static struct lpc31_dqh_s __attribute__((aligned(2048))) g_qh[LPC31_NPHYSENDPOINTS];
static struct lpc31_dtd_s __attribute__((aligned(32)))   g_td[LPC31_NPHYSENDPOINTS];

static const struct usbdev_epops_s g_epops =
{
  .configure   = lpc31_epconfigure,
  .disable     = lpc31_epdisable,
  .allocreq    = lpc31_epallocreq,
  .freereq     = lpc31_epfreereq,
#ifdef CONFIG_USBDEV_DMA
  .allocbuffer = lpc31_epallocbuffer,
  .freebuffer  = lpc31_epfreebuffer,
#endif
  .submit      = lpc31_epsubmit,
  .cancel      = lpc31_epcancel,
  .stall       = lpc31_epstall,
};

static const struct usbdev_ops_s g_devops =
{
  .allocep     = lpc31_allocep,
  .freeep      = lpc31_freeep,
  .getframe    = lpc31_getframe,
  .wakeup      = lpc31_wakeup,
  .selfpowered = lpc31_selfpowered,
  .pullup      = lpc31_pullup,
};

/*******************************************************************************
 * Public Data
 *******************************************************************************/

/*******************************************************************************
 * Private Functions
 *******************************************************************************/

/*******************************************************************************
 * Name: lpc31_getreg
 *
 * Description:
 *   Get the contents of an LPC313x register
 *
 *******************************************************************************/

#if defined(CONFIG_LPC31_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
static uint32_t lpc31_getreg(uint32_t addr)
{
  static uint32_t prevaddr = 0;
  static uint32_t preval = 0;
  static uint32_t count = 0;

  /* Read the value from the register */

  uint32_t val = getreg32(addr);

  /* Is this the same value that we read from the same registe last time?  Are
   * we polling the register?  If so, suppress some of the output.
   */

  if (addr == prevaddr && val == preval)
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
 * Name: lpc31_putreg
 *
 * Description:
 *   Set the contents of an LPC313x register to a value
 *
 *******************************************************************************/

#if defined(CONFIG_LPC31_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
static void lpc31_putreg(uint32_t val, uint32_t addr)
{
  /* Show the register value being written */

  lldbg("%08x<-%08x\n", addr, val);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/*******************************************************************************
 * Name: lpc31_clrbits
 *
 * Description:
 *   Clear bits in a register
 *
 *******************************************************************************/

static inline void lpc31_clrbits(uint32_t mask, uint32_t addr)
{
  uint32_t reg = lpc31_getreg(addr);
  reg &= ~mask;
  lpc31_putreg(reg, addr);
}

/*******************************************************************************
 * Name: lpc31_setbits
 *
 * Description:
 *   Set bits in a register
 *
 *******************************************************************************/

static inline void lpc31_setbits(uint32_t mask, uint32_t addr)
{
  uint32_t reg = lpc31_getreg(addr);
  reg |= mask;
  lpc31_putreg(reg, addr);
}

/*******************************************************************************
 * Name: lpc31_chgbits
 *
 * Description:
 *   Change bits in a register
 *
 *******************************************************************************/

static inline void lpc31_chgbits(uint32_t mask, uint32_t val, uint32_t addr)
{
  uint32_t reg = lpc31_getreg(addr);
  reg &= ~mask;
  reg |= val;
  lpc31_putreg(reg, addr);
}

/*******************************************************************************
 * Name: lpc31_rqdequeue
 *
 * Description:
 *   Remove a request from an endpoint request queue
 *
 *******************************************************************************/

static FAR struct lpc31_req_s *lpc31_rqdequeue(FAR struct lpc31_ep_s *privep)
{
  FAR struct lpc31_req_s *ret = privep->head;

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
 * Name: lpc31_rqenqueue
 *
 * Description:
 *   Add a request from an endpoint request queue
 *
 *******************************************************************************/

static bool lpc31_rqenqueue(FAR struct lpc31_ep_s *privep,
                              FAR struct lpc31_req_s *req)
{
  bool is_empty = !privep->head;
  
  req->flink = NULL;
  if (is_empty)
    {
      privep->head = req;
      privep->tail = req;
    }
  else
    {
      privep->tail->flink = req;
      privep->tail        = req;
    }
  return is_empty;
}

/*******************************************************************************
 * Name: lpc31_writedtd
 *
 * Description:
 *   Initialise a DTD to transfer the data
 *
 *******************************************************************************/

static inline void lpc31_writedtd(struct lpc31_dtd_s *dtd, const uint8_t *data, uint32_t nbytes)
{
  dtd->nextdesc  = DTD_NEXTDESC_INVALID;
  dtd->config    = DTD_CONFIG_LENGTH(nbytes) | DTD_CONFIG_IOC | DTD_CONFIG_ACTIVE;
  dtd->buffer0   = ((uint32_t) data);
  dtd->buffer1   = (((uint32_t) data) + 0x1000) & 0xfffff000;
  dtd->buffer2   = (((uint32_t) data) + 0x2000) & 0xfffff000;
  dtd->buffer3   = (((uint32_t) data) + 0x3000) & 0xfffff000;
  dtd->buffer4   = (((uint32_t) data) + 0x4000) & 0xfffff000;
  dtd->xfer_len  = nbytes;
}

/*******************************************************************************
 * Name: lpc31_queuedtd
 *
 * Description:
 *   Add the DTD to the device list
 *
 *******************************************************************************/

static void lpc31_queuedtd(uint8_t epphy, struct lpc31_dtd_s *dtd)
{
  /* Queue the DTD onto the Endpoint */
  /* NOTE - this only works when no DTD is currently queued */

  g_qh[epphy].overlay.nextdesc = (uint32_t) dtd;
  g_qh[epphy].overlay.config  &= ~(DTD_CONFIG_ACTIVE | DTD_CONFIG_HALTED);

  uint32_t bit = LPC31_ENDPTMASK(epphy);

  lpc31_setbits (bit, LPC31_USBDEV_ENDPTPRIME);
    
  while (lpc31_getreg (LPC31_USBDEV_ENDPTPRIME) & bit)
    ;
}

/*******************************************************************************
 * Name: lpc31_ep0xfer
 *
 * Description:
 *   Schedule a short transfer for Endpoint 0 (IN or OUT)
 *
 *******************************************************************************/

static inline void lpc31_ep0xfer(uint8_t epphy, uint8_t *buf, uint32_t nbytes)
{
  struct lpc31_dtd_s *dtd = &g_td[epphy];

  lpc31_writedtd(dtd, buf, nbytes);
    
  lpc31_queuedtd(epphy, dtd);
}

/*******************************************************************************
 * Name: lpc31_readsetup
 *
 * Description:
 *   Read a Setup packet from the DTD.
 *
 *******************************************************************************/
static void lpc31_readsetup(uint8_t epphy, struct usb_ctrlreq_s *ctrl)
{
    struct lpc31_dqh_s *dqh = &g_qh[epphy];
    int i;

    do {
    /* Set the trip wire */
    lpc31_setbits(USBDEV_USBCMD_SUTW, LPC31_USBDEV_USBCMD);

    /* copy the request... */
    for (i = 0; i < 8; i++)
        ((uint8_t *) ctrl)[i] = ((uint8_t *) dqh->setup)[i];
    
    } while (!(lpc31_getreg(LPC31_USBDEV_USBCMD) & USBDEV_USBCMD_SUTW));
    
    /* Clear the trip wire */
    lpc31_clrbits(USBDEV_USBCMD_SUTW, LPC31_USBDEV_USBCMD);

    /* Clear the Setup Interrupt */
    lpc31_putreg (LPC31_ENDPTMASK(LPC31_EP0_OUT), LPC31_USBDEV_ENDPTSETUPSTAT);
}

/*******************************************************************************
 * Name: lpc31_set_address
 *
 * Description:
 *   Set the devices USB address
 *
 *******************************************************************************/

static inline void lpc31_set_address(struct lpc31_usbdev_s *priv, uint16_t address)
{
  priv->paddr    = address;
  priv->paddrset = address != 0;

  lpc31_chgbits(USBDEV_DEVICEADDR_MASK, priv->paddr << USBDEV_DEVICEADDR_SHIFT, 
                LPC31_USBDEV_DEVICEADDR);
}

/*******************************************************************************
 * Name: lpc31_flushep
 *
 * Description:
 *   Flush any primed descriptors from this ep
 *
 *******************************************************************************/

static void lpc31_flushep(struct lpc31_ep_s *privep)
{
  uint32_t mask = LPC31_ENDPTMASK(privep->epphy);
  do
    {
      lpc31_putreg (mask, LPC31_USBDEV_ENDPTFLUSH);
      while ((lpc31_getreg(LPC31_USBDEV_ENDPTFLUSH) & mask) != 0)
      ;
    }
  while ((lpc31_getreg(LPC31_USBDEV_ENDPTSTATUS) & mask) != 0);
}


/*******************************************************************************
 * Name: lpc31_progressep
 *
 * Description:
 *   Progress the Endpoint by priming the first request into the queue head
 *
 *******************************************************************************/

static int lpc31_progressep(struct lpc31_ep_s *privep)
{
  struct lpc31_dtd_s *dtd = &g_td[privep->epphy];
  struct lpc31_req_s *privreq;

  /* Check the request from the head of the endpoint request queue */

  privreq = lpc31_rqpeek(privep);
  if (!privreq)
    {
      usbtrace(TRACE_INTDECODE(LPC31_TRACEINTID_EPINQEMPTY), 0);
      return OK;
    }

  /* Ignore any attempt to send a zero length packet */

  if (privreq->req.len == 0)
    {
    /* If the class driver is responding to a setup packet, then wait for the 
     * host to illicit thr response */

    if (privep->epphy == LPC31_EP0_IN && privep->dev->ep0state == EP0STATE_SETUP_OUT)
      lpc31_ep0state (privep->dev, EP0STATE_WAIT_NAK_IN);
    else
      {
        if (LPC31_EPPHYIN(privep->epphy))
        usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_EPINNULLPACKET), 0);
        else
        usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_EPOUTNULLPACKET), 0);
      }
      
      lpc31_reqcomplete(privep, lpc31_rqdequeue(privep), OK);
      return OK;
    }

  if (privep->epphy == LPC31_EP0_IN)
    lpc31_ep0state (privep->dev,  EP0STATE_DATA_IN);
  else if (privep->epphy == LPC31_EP0_OUT)
    lpc31_ep0state (privep->dev, EP0STATE_DATA_OUT);

  int bytesleft = privreq->req.len - privreq->req.xfrd;

  if (LPC31_EPPHYIN(privep->epphy))
    usbtrace(TRACE_WRITE(privep->epphy), privreq->req.xfrd);
  else
    usbtrace(TRACE_READ(privep->epphy), privreq->req.xfrd);

  /* Initialise the DTD to transfer the next chunk */

  lpc31_writedtd (dtd, privreq->req.buf + privreq->req.xfrd, bytesleft);

  /* then queue onto the DQH */
  lpc31_queuedtd(privep->epphy, dtd);

  return OK;
}

/*******************************************************************************
 * Name: lpc31_abortrequest
 *
 * Description:
 *   Discard a request
 *
 *******************************************************************************/

static inline void lpc31_abortrequest(struct lpc31_ep_s *privep,
                                      struct lpc31_req_s *privreq,
                                      int16_t result)
{
  usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_REQABORTED), (uint16_t)privep->epphy);

  /* Save the result in the request structure */

  privreq->req.result = result;

  /* Callback to the request completion handler */

  privreq->req.callback(&privep->ep, &privreq->req);
}

/*******************************************************************************
 * Name: lpc31_reqcomplete
 *
 * Description:
 *   Handle termination of the request at the head of the endpoint request queue.
 *
 *******************************************************************************/

static void lpc31_reqcomplete(struct lpc31_ep_s *privep,
                              struct lpc31_req_s *privreq, int16_t result)
{
  /* If endpoint 0, temporarily reflect the state of protocol stalled
   * in the callback.
   */

  bool stalled = privep->stalled;
  if (privep->epphy == LPC31_EP0_IN)
    privep->stalled = privep->dev->stalled;

  /* Save the result in the request structure */

  privreq->req.result = result;

  /* Callback to the request completion handler */

  privreq->req.callback(&privep->ep, &privreq->req);

  /* Restore the stalled indication */

  privep->stalled = stalled;
}

/*******************************************************************************
 * Name: lpc31_cancelrequests
 *
 * Description:
 *   Cancel all pending requests for an endpoint
 *
 *******************************************************************************/

static void lpc31_cancelrequests(struct lpc31_ep_s *privep, int16_t status)
{
  if (!lpc31_rqempty(privep))
      lpc31_flushep(privep);

  while (!lpc31_rqempty(privep))
    {
      // FIXME: the entry at the head should be sync'd with the DTD
      // FIXME: only report the error status if the transfer hasn't completed
      usbtrace(TRACE_COMPLETE(privep->epphy),
               (lpc31_rqpeek(privep))->req.xfrd);
      lpc31_reqcomplete(privep, lpc31_rqdequeue(privep), status);
    }
}

/*******************************************************************************
 * Name: lpc31_epfindbyaddr
 *
 * Description:
 *   Find the physical endpoint structure corresponding to a logic endpoint
 *   address
 *
 *******************************************************************************/

static struct lpc31_ep_s *lpc31_epfindbyaddr(struct lpc31_usbdev_s *priv,
                         uint16_t eplog)
{
  struct lpc31_ep_s *privep;
  int i;

  /* Endpoint zero is a special case */

  if (USB_EPNO(eplog) == 0)
    {
      return &priv->eplist[0];
    }

  /* Handle the remaining */

  for (i = 1; i < LPC31_NPHYSENDPOINTS; i++)
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
 * Name: lpc31_dispatchrequest
 *
 * Description:
 *   Provide unhandled setup actions to the class driver. This is logically part
 *   of the USB interrupt handler.
 *
 *******************************************************************************/

static void lpc31_dispatchrequest(struct lpc31_usbdev_s *priv,
                                    const struct usb_ctrlreq_s *ctrl)
{
  int ret = -EIO;

  usbtrace(TRACE_INTDECODE(LPC31_TRACEINTID_DISPATCH), 0);
  if (priv->driver)
    {
      /* Forward to the control request to the class driver implementation */

      ret = CLASS_SETUP(priv->driver, &priv->usbdev, ctrl, NULL, 0);
    }

  if (ret < 0)
    {
      /* Stall on failure */

      usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_DISPATCHSTALL), 0);
      priv->stalled = true;
    }
}

/*******************************************************************************
 * Name: lpc31_ep0configure
 *
 * Description:
 *   Reset Usb engine
 *
 *******************************************************************************/

static void lpc31_ep0configure(struct lpc31_usbdev_s *priv)
{
  /* Enable ep0 IN and ep0 OUT */
  g_qh[LPC31_EP0_OUT].capability = (DQH_CAPABILITY_MAX_PACKET(CONFIG_LPC31_USBDEV_EP0_MAXSIZE) |
                      DQH_CAPABILITY_IOS |
                      DQH_CAPABILITY_ZLT);

  g_qh[LPC31_EP0_IN ].capability = (DQH_CAPABILITY_MAX_PACKET(CONFIG_LPC31_USBDEV_EP0_MAXSIZE) |
                      DQH_CAPABILITY_IOS |
                      DQH_CAPABILITY_ZLT);
  
  g_qh[LPC31_EP0_OUT].currdesc = DTD_NEXTDESC_INVALID;
  g_qh[LPC31_EP0_IN ].currdesc = DTD_NEXTDESC_INVALID;
  
  /* Enable EP0 */
  lpc31_setbits (USBDEV_ENDPTCTRL0_RXE | USBDEV_ENDPTCTRL0_TXE, LPC31_USBDEV_ENDPTCTRL0);
}

/*******************************************************************************
 * Name: lpc31_usbreset
 *
 * Description:
 *   Reset Usb engine
 *
 *******************************************************************************/

static void lpc31_usbreset(struct lpc31_usbdev_s *priv)
{
  int epphy;

  /* Disable all endpoints */

  lpc31_clrbits (USBDEV_ENDPTCTRL_RXE | USBDEV_ENDPTCTRL_TXE, LPC31_USBDEV_ENDPTCTRL0);
  lpc31_clrbits (USBDEV_ENDPTCTRL_RXE | USBDEV_ENDPTCTRL_TXE, LPC31_USBDEV_ENDPTCTRL1);
  lpc31_clrbits (USBDEV_ENDPTCTRL_RXE | USBDEV_ENDPTCTRL_TXE, LPC31_USBDEV_ENDPTCTRL2);
  lpc31_clrbits (USBDEV_ENDPTCTRL_RXE | USBDEV_ENDPTCTRL_TXE, LPC31_USBDEV_ENDPTCTRL3);

  /* Clear all pending interrupts */

  lpc31_putreg (lpc31_getreg(LPC31_USBDEV_ENDPTNAK),       LPC31_USBDEV_ENDPTNAK);
  lpc31_putreg (lpc31_getreg(LPC31_USBDEV_ENDPTSETUPSTAT), LPC31_USBDEV_ENDPTSETUPSTAT);
  lpc31_putreg (lpc31_getreg(LPC31_USBDEV_ENDPTCOMPLETE),  LPC31_USBDEV_ENDPTCOMPLETE);

  /* Wait for all prime operations to have completed and then flush all DTDs */
  while (lpc31_getreg (LPC31_USBDEV_ENDPTPRIME) != 0)
    ;
  lpc31_putreg (LPC31_ENDPTMASK_ALL, LPC31_USBDEV_ENDPTFLUSH);
  while (lpc31_getreg (LPC31_USBDEV_ENDPTFLUSH))
    ;

  /* Reset endpoints */
  for (epphy = 0; epphy < LPC31_NPHYSENDPOINTS; epphy++)
    {
      struct lpc31_ep_s *privep = &priv->eplist[epphy];

      lpc31_cancelrequests (privep, -ESHUTDOWN);

      /* Reset endpoint status */
      privep->stalled = false;
    }

  /* Tell the class driver that we are disconnected. The class 
   * driver should then accept any new configurations. */

  if (priv->driver)
    {
      CLASS_DISCONNECT(priv->driver, &priv->usbdev);
    }

  /* Set the interrupt Threshold control interval to 0 */
  lpc31_chgbits(USBDEV_USBCMD_ITC_MASK, USBDEV_USBCMD_ITCIMME, LPC31_USBDEV_USBCMD);

  /* Zero out the Endpoint queue heads */
  memset ((void *) g_qh, 0, sizeof (g_qh));
  memset ((void *) g_td, 0, sizeof (g_td));

  /* Set USB address to 0 */
  lpc31_set_address (priv, 0);

  /* Initialise the Enpoint List Address */
  lpc31_putreg ((uint32_t)g_qh, LPC31_USBDEV_ENDPOINTLIST);

  /* EndPoint 0 initialization */
  lpc31_ep0configure(priv);

  /* Enable Device interrupts */
  lpc31_putreg(USB_FRAME_INT | USB_ERROR_INT | 
         USBDEV_USBINTR_NAKE | USBDEV_USBINTR_SLE | USBDEV_USBINTR_URE | USBDEV_USBINTR_PCE | USBDEV_USBINTR_UE,
         LPC31_USBDEV_USBINTR);
}

/*******************************************************************************
 * Name: lpc31_setstate
 *
 * Description:
 *   Sets the EP0 state and manages the NAK interrupts
 *
 *******************************************************************************/

static inline void lpc31_ep0state(struct lpc31_usbdev_s *priv, uint16_t state)
{
  priv->ep0state = state;
  
  switch (state)
    {
    case EP0STATE_WAIT_NAK_IN:
      lpc31_putreg (LPC31_ENDPTMASK(LPC31_EP0_IN), LPC31_USBDEV_ENDPTNAKEN);
      break;
    case EP0STATE_WAIT_NAK_OUT:
      lpc31_putreg (LPC31_ENDPTMASK(LPC31_EP0_OUT), LPC31_USBDEV_ENDPTNAKEN);
      break;
    default:
      lpc31_putreg(0, LPC31_USBDEV_ENDPTNAKEN);
      break;
    }
}

/*******************************************************************************
 * Name: lpc31_ep0setup
 *
 * Description:
 *   USB Ctrl EP Setup Event. This is logically part of the USB interrupt
 *   handler.  This event occurs when a setup packet is receive on EP0 OUT.
 *
 *******************************************************************************/

static inline void lpc31_ep0setup(struct lpc31_usbdev_s *priv)
{
  struct lpc31_ep_s *privep;
  struct usb_ctrlreq_s ctrl;
  uint16_t value;
  uint16_t index;
  uint16_t len;

  /* Terminate any pending requests - since all DTDs will have been retired 
   * because of the setup packet */

  lpc31_cancelrequests(&priv->eplist[LPC31_EP0_OUT], -EPROTO);
  lpc31_cancelrequests(&priv->eplist[LPC31_EP0_IN],  -EPROTO);

  /* Assume NOT stalled */

  priv->eplist[LPC31_EP0_OUT].stalled = false;
  priv->eplist[LPC31_EP0_IN].stalled = false;
  priv->stalled = false;

  /* Read EP0 setup data */
  lpc31_readsetup(LPC31_EP0_OUT, &ctrl);

  /* Starting a control request - update state */
  lpc31_ep0state (priv, (ctrl.type & USB_REQ_DIR_IN) ? EP0STATE_SETUP_IN : EP0STATE_SETUP_OUT);

  /* And extract the little-endian 16-bit values to host order */
  value = GETUINT16(ctrl.value);
  index = GETUINT16(ctrl.index);
  len   = GETUINT16(ctrl.len);

  ullvdbg("type=%02x req=%02x value=%04x index=%04x len=%04x\n",
          ctrl.type, ctrl.req, value, index, len);

  /* Dispatch any non-standard requests */
  if ((ctrl.type & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_STANDARD)
    lpc31_dispatchrequest(priv, &ctrl);
  else
    {
    /* Handle standard request.  Pick off the things of interest to the USB
     * device controller driver; pass what is left to the class driver */
    switch (ctrl.req)
      {
      case USB_REQ_GETSTATUS:
        {
          /* type:  device-to-host; recipient = device, interface, endpoint
           * value: 0
           * index: zero interface endpoint
           * len:   2; data = status
           */
  
          usbtrace(TRACE_INTDECODE(LPC31_TRACEINTID_GETSTATUS), 0);
          if (!priv->paddrset || len != 2 ||
              (ctrl.type & USB_REQ_DIR_IN) == 0 || value != 0)
            {
              priv->stalled = true;
            }
          else
            {
              switch (ctrl.type & USB_REQ_RECIPIENT_MASK)
                {
                case USB_REQ_RECIPIENT_ENDPOINT:
                  {
                    usbtrace(TRACE_INTDECODE(LPC31_TRACEINTID_EPGETSTATUS), 0);
                    privep = lpc31_epfindbyaddr(priv, index);
                    if (!privep)
                      {
                        usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_BADEPGETSTATUS), 0);
                        priv->stalled = true;
                      }
                    else
                      {
                        if (privep->stalled)
                          priv->ep0buf[0] = 1; /* Stalled */
                        else
                          priv->ep0buf[0] = 0; /* Not stalled */
                        priv->ep0buf[1] = 0;
  
                        lpc31_ep0xfer (LPC31_EP0_IN, priv->ep0buf, 2);
                        lpc31_ep0state (priv, EP0STATE_SHORTWRITE);
                      }
                    }
                  break;
  
                case USB_REQ_RECIPIENT_DEVICE:
                  {
                    if (index == 0)
                      {
                        usbtrace(TRACE_INTDECODE(LPC31_TRACEINTID_DEVGETSTATUS), 0);
  
                        /* Features:  Remote Wakeup=YES; selfpowered=? */
  
                        priv->ep0buf[0] = (priv->selfpowered << USB_FEATURE_SELFPOWERED) |
                              (1 << USB_FEATURE_REMOTEWAKEUP);
                        priv->ep0buf[1] = 0;

                        lpc31_ep0xfer(LPC31_EP0_IN, priv->ep0buf, 2);
                        lpc31_ep0state (priv, EP0STATE_SHORTWRITE);
                      }
                    else
                      {
                        usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_BADDEVGETSTATUS), 0);
                        priv->stalled = true;
                      }
                  }
                  break;
  
                case USB_REQ_RECIPIENT_INTERFACE:
                  {
                    usbtrace(TRACE_INTDECODE(LPC31_TRACEINTID_IFGETSTATUS), 0);
                    priv->ep0buf[0] = 0;
                    priv->ep0buf[1] = 0;

                    lpc31_ep0xfer(LPC31_EP0_IN, priv->ep0buf, 2);
                    lpc31_ep0state (priv, EP0STATE_SHORTWRITE);
                  }
                  break;
  
                default:
                  {
                    usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_BADGETSTATUS), 0);
                    priv->stalled = true;
                  }
                  break;
                }
            }
        }
        break;
  
      case USB_REQ_CLEARFEATURE:
        {
          /* type:  host-to-device; recipient = device, interface or endpoint
           * value: feature selector
           * index: zero interface endpoint;
           * len:   zero, data = none
           */
  
          usbtrace(TRACE_INTDECODE(LPC31_TRACEINTID_CLEARFEATURE), 0);
          if ((ctrl.type & USB_REQ_RECIPIENT_MASK) != USB_REQ_RECIPIENT_ENDPOINT)
            {
              lpc31_dispatchrequest(priv, &ctrl);
            }
          else if (priv->paddrset != 0 && value == USB_FEATURE_ENDPOINTHALT && len == 0 &&
                   (privep = lpc31_epfindbyaddr(priv, index)) != NULL)
            {
              lpc31_epstall(&privep->ep, true);
              lpc31_ep0state (priv, EP0STATE_WAIT_NAK_IN);
            }
          else
            {
              usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_BADCLEARFEATURE), 0);
              priv->stalled = true;
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
  
          usbtrace(TRACE_INTDECODE(LPC31_TRACEINTID_SETFEATURE), 0);
          if (((ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE) &&
              value == USB_FEATURE_TESTMODE)
            {
              ullvdbg("test mode: %d\n", index);
            }
          else if ((ctrl.type & USB_REQ_RECIPIENT_MASK) != USB_REQ_RECIPIENT_ENDPOINT)
            {
              lpc31_dispatchrequest(priv, &ctrl);
            }
          else if (priv->paddrset != 0 && value == USB_FEATURE_ENDPOINTHALT && len == 0 &&
                   (privep = lpc31_epfindbyaddr(priv, index)) != NULL)
            {
              lpc31_epstall(&privep->ep, false);
              lpc31_ep0state (priv, EP0STATE_WAIT_NAK_IN);
            }
          else
            {
              usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_BADSETFEATURE), 0);
              priv->stalled = true;
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
          usbtrace(TRACE_INTDECODE(LPC31_TRACEINTID_EP0SETUPSETADDRESS), value);
          if ((ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE &&
              index  == 0 && len == 0 && value < 128)
            {
              /* Save the address.  We cannot actually change to the next address until
               * the completion of the status phase. */
  
              priv->paddr = ctrl.value[0];
              priv->paddrset = false;
              lpc31_ep0state (priv, EP0STATE_WAIT_NAK_IN);
            }
          else
            {
              usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_BADSETADDRESS), 0);
              priv->stalled = true;
            }
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
          usbtrace(TRACE_INTDECODE(LPC31_TRACEINTID_GETSETDESC), 0);
          if ((ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE)
            {
              lpc31_dispatchrequest(priv, &ctrl);
            }
          else
            {
              usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_BADGETSETDESC), 0);
              priv->stalled = true;
            }
        }
        break;
  
      case USB_REQ_GETCONFIGURATION:
        /* type:  device-to-host; recipient = device
         * value: 0;
         * index: 0;
         * len:   1; data = configuration value
         */
        {
          usbtrace(TRACE_INTDECODE(LPC31_TRACEINTID_GETCONFIG), 0);
          if (priv->paddrset && (ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE &&
              value == 0 && index == 0 && len == 1)
            {
              lpc31_dispatchrequest(priv, &ctrl);
            }
          else
            {
              usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_BADGETCONFIG), 0);
              priv->stalled = true;
            }
        }
        break;
  
      case USB_REQ_SETCONFIGURATION:
        /* type:  host-to-device; recipient = device
         * value: configuration value
         * index: 0;
         * len:   0; data = none
         */
        {
          usbtrace(TRACE_INTDECODE(LPC31_TRACEINTID_SETCONFIG), 0);
          if ((ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE &&
              index == 0 && len == 0)
            {
              lpc31_dispatchrequest(priv, &ctrl);
            }
          else
            {
              usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_BADSETCONFIG), 0);
              priv->stalled = true;
            }
        }
        break;
  
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
          usbtrace(TRACE_INTDECODE(LPC31_TRACEINTID_GETSETIF), 0);
          lpc31_dispatchrequest(priv, &ctrl);
        }
        break;
  
      case USB_REQ_SYNCHFRAME:
        /* type:  device-to-host; recipient = endpoint
         * value: 0
         * index: endpoint;
         * len:   2; data = frame number
         */
        {
          usbtrace(TRACE_INTDECODE(LPC31_TRACEINTID_SYNCHFRAME), 0);
        }
        break;
  
      default:
        {
          usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_INVALIDCTRLREQ), 0);
          priv->stalled = true;
        }
        break;
      }
  }

  if (priv->stalled)
    {
      usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_EP0SETUPSTALLED), priv->ep0state);
      lpc31_epstall(&priv->eplist[LPC31_EP0_IN].ep, false);
      lpc31_epstall(&priv->eplist[LPC31_EP0_OUT].ep, false);
    }
}

/*******************************************************************************
 * Name: lpc31_ep0complete
 *
 * Description:
 *   Transfer complete handler for Endpoint 0
 *
 *******************************************************************************/

static void lpc31_ep0complete(struct lpc31_usbdev_s *priv, uint8_t epphy)
{
  struct lpc31_ep_s *privep = &priv->eplist[epphy];

  usbtrace(TRACE_INTDECODE(LPC31_TRACEINTID_EP0COMPLETE), (uint16_t)priv->ep0state);
  
  switch (priv->ep0state)
    {
    case EP0STATE_DATA_IN:
      if (lpc31_rqempty(privep))
        return;

      if (lpc31_epcomplete (priv, epphy))
        lpc31_ep0state (priv, EP0STATE_WAIT_NAK_OUT);
      break;

    case EP0STATE_DATA_OUT:
      if (lpc31_rqempty(privep))
    return;
    
      if (lpc31_epcomplete (priv, epphy))
        lpc31_ep0state (priv, EP0STATE_WAIT_NAK_IN);
      break;
    
    case EP0STATE_SHORTWRITE:
      lpc31_ep0state (priv, EP0STATE_WAIT_NAK_OUT);
      break;
    
    case EP0STATE_WAIT_STATUS_IN:
      lpc31_ep0state (priv, EP0STATE_IDLE);

      /* If we've received a SETADDRESS packet, then we set the address
       * now that the status phase has completed */
      if (! priv->paddrset && priv->paddr != 0)
        {
          usbtrace(TRACE_INTDECODE(LPC31_TRACEINTID_EP0INSETADDRESS), (uint16_t)priv->paddr);
          lpc31_set_address (priv, priv->paddr);
        }
      break;

    case EP0STATE_WAIT_STATUS_OUT:
      lpc31_ep0state (priv, EP0STATE_IDLE);
      break;

    default:
#ifdef CONFIG_DEBUG
      DEBUGASSERT(priv->ep0state != EP0STATE_DATA_IN &&
          priv->ep0state != EP0STATE_DATA_OUT        &&
          priv->ep0state != EP0STATE_SHORTWRITE      &&
          priv->ep0state != EP0STATE_WAIT_STATUS_IN  &&
          priv->ep0state != EP0STATE_WAIT_STATUS_OUT);
#endif
      priv->stalled = true;
      break;
    }

  if (priv->stalled)
    {
      usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_EP0SETUPSTALLED), priv->ep0state);
      lpc31_epstall(&priv->eplist[LPC31_EP0_IN].ep, false);
      lpc31_epstall(&priv->eplist[LPC31_EP0_OUT].ep, false);
    }
}

/*******************************************************************************
 * Name: lpc31_ep0nak
 *
 * Description:
 *   Handle a NAK interrupt on EP0
 *
 *******************************************************************************/

static void lpc31_ep0nak(struct lpc31_usbdev_s *priv, uint8_t epphy)
{
  usbtrace(TRACE_INTDECODE(LPC31_TRACEINTID_EP0NAK), (uint16_t)priv->ep0state);

  switch (priv->ep0state)
    {
    case EP0STATE_WAIT_NAK_IN:
      lpc31_ep0xfer (LPC31_EP0_IN, NULL, 0);
      lpc31_ep0state (priv, EP0STATE_WAIT_STATUS_IN);
      break;
    case EP0STATE_WAIT_NAK_OUT:
      lpc31_ep0xfer (LPC31_EP0_OUT, NULL, 0);
      lpc31_ep0state (priv, EP0STATE_WAIT_STATUS_OUT);
      break;
    default:
#ifdef CONFIG_DEBUG
      DEBUGASSERT(priv->ep0state != EP0STATE_WAIT_NAK_IN &&
          priv->ep0state != EP0STATE_WAIT_NAK_OUT);
#endif
      priv->stalled = true;
      break;
    }

  if (priv->stalled)
    {
      usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_EP0SETUPSTALLED), priv->ep0state);
      lpc31_epstall(&priv->eplist[LPC31_EP0_IN].ep, false);
      lpc31_epstall(&priv->eplist[LPC31_EP0_OUT].ep, false);
    }
}

/*******************************************************************************
 * Name: lpc31_epcomplete
 *
 * Description:
 *   Transfer complete handler for Endpoints other than 0
 *   returns whether the request at the head has completed
 *
 *******************************************************************************/

bool lpc31_epcomplete(struct lpc31_usbdev_s *priv, uint8_t epphy)
{
  struct lpc31_ep_s  *privep  = &priv->eplist[epphy];
  struct lpc31_req_s *privreq = privep->head;
  struct lpc31_dtd_s *dtd     = &g_td[epphy];

  if (privreq == NULL)        /* This shouldn't really happen */
  {
    if (LPC31_EPPHYOUT(privep->epphy))
      usbtrace(TRACE_INTDECODE(LPC31_TRACEINTID_EPINQEMPTY), 0);
    else
      usbtrace(TRACE_INTDECODE(LPC31_TRACEINTID_EPOUTQEMPTY), 0);
    return true;
  }
    
  int xfrd = dtd->xfer_len - (dtd->config >> 16);
    
  privreq->req.xfrd += xfrd;

  bool complete = true;
  if (LPC31_EPPHYOUT(privep->epphy))
    {
      /* read(OUT) completes when request filled, or a short transfer is received */

      usbtrace(TRACE_INTDECODE(LPC31_TRACEINTID_EPIN), complete);
    }
  else
    {
      /* write(IN) completes when request finished, unless we need to terminate with a ZLP */

      bool need_zlp = (xfrd == privep->ep.maxpacket) && ((privreq->req.flags & USBDEV_REQFLAGS_NULLPKT) != 0);

      complete = (privreq->req.xfrd >= privreq->req.len && !need_zlp);

      usbtrace(TRACE_INTDECODE(LPC31_TRACEINTID_EPOUT), complete);
    }

  /* If the transfer is complete, then dequeue and progress any further queued requests */

  if (complete)
    {
      privreq = lpc31_rqdequeue (privep);
    }
    
  if (!lpc31_rqempty(privep))
    {
      lpc31_progressep(privep);
    }

  /* Now it's safe to call the completion callback as it may well submit a new request */

  if (complete)
    {
      usbtrace(TRACE_COMPLETE(privep->epphy), privreq->req.xfrd);
      lpc31_reqcomplete(privep, privreq, OK);
    }

  return complete;
}


/*******************************************************************************
 * Name: lpc31_usbinterrupt
 *
 * Description:
 *   USB interrupt handler
 *
 *******************************************************************************/

static int lpc31_usbinterrupt(int irq, FAR void *context)
{
  struct lpc31_usbdev_s *priv = &g_usbdev;
  uint32_t disr, portsc1, n;

  usbtrace(TRACE_INTENTRY(LPC31_TRACEINTID_USB), 0);

  /* Read the interrupts and then clear them */

  disr = lpc31_getreg(LPC31_USBDEV_USBSTS);
  lpc31_putreg(disr, LPC31_USBDEV_USBSTS);

  if (disr & USBDEV_USBSTS_URI)
    {
      usbtrace(TRACE_INTDECODE(LPC31_TRACEINTID_DEVRESET),0);

      lpc31_usbreset(priv);

      usbtrace(TRACE_INTEXIT(LPC31_TRACEINTID_USB), 0);
      return OK;
    }

  /* When the device controller enters a suspend state from an active state,
   * the SLI bit will be set to a one.
   */
 
  if (!priv->suspended && (disr & USBDEV_USBSTS_SLI) != 0)
    {
      usbtrace(TRACE_INTDECODE(LPC31_TRACEINTID_SUSPENDED),0);

      /* Inform the Class driver of the suspend event */

      priv->suspended = 1;
      if (priv->driver)
        {
          CLASS_SUSPEND(priv->driver, &priv->usbdev);
        }

      /* TODO: Perform power management operations here. */
    }

  /* The device controller clears the SLI bit upon exiting from a suspend
   * state. This bit can also be cleared by software writing a one to it.
   */

  else if (priv->suspended && (disr & USBDEV_USBSTS_SLI) == 0)
    {
      usbtrace(TRACE_INTDECODE(LPC31_TRACEINTID_RESUMED),0);

      /* Inform the Class driver of the resume event */

      priv->suspended = 0;
      if (priv->driver)
        {
          CLASS_RESUME(priv->driver, &priv->usbdev);
        }

      /* TODO: Perform power management operations here. */
    }

  if (disr & USBDEV_USBSTS_PCI)
    {
      portsc1 = lpc31_getreg(LPC31_USBDEV_PORTSC1);

      if (portsc1 & USBDEV_PRTSC1_HSP)
        priv->usbdev.speed = USB_SPEED_HIGH;
      else
        priv->usbdev.speed = USB_SPEED_FULL;

      if (portsc1 & USBDEV_PRTSC1_FPR)
        {
          /* FIXME: this occurs because of a J-to-K transition detected
           *         while the port is in SUSPEND state - presumambly this 
           *         is where the host is resuming the device?
           *
           *  - but do we need to "ack" the interrupt
           */
        }
    }
  
#ifdef CONFIG_LPC31_USBDEV_FRAME_INTERRUPT
  if (disr & USBDEV_USBSTT_SRI)
    {
      usbtrace(TRACE_INTDECODE(LPC31_TRACEINTID_FRAME), 0);
      
      priv->sof = (int)lpc31_getreg(LPC31_USBDEV_FRINDEX_OFFSET); 
    }
#endif

  if (disr & USBDEV_USBSTS_UEI)
    {
      /* FIXME: these occur when a transfer results in an error condition
       *        it is set alongside USBINT if the DTD also had its IOC 
       *        bit set. */
    }
  
  if (disr & USBDEV_USBSTS_UI)
    {
      /* Handle completion interrupts */
      uint32_t mask = lpc31_getreg (LPC31_USBDEV_ENDPTCOMPLETE);

      if (mask)
        {
          /* Clear any NAK interrupt and completion interrupts */
          lpc31_putreg (mask, LPC31_USBDEV_ENDPTNAK);
          lpc31_putreg (mask, LPC31_USBDEV_ENDPTCOMPLETE);
      
          if (mask & LPC31_ENDPTMASK(0))
            lpc31_ep0complete(priv, 0);
          if (mask & LPC31_ENDPTMASK(1))
            lpc31_ep0complete(priv, 1);
      
          for (n = 1; n < LPC31_NLOGENDPOINTS; n++)
            {
              if (mask & LPC31_ENDPTMASK((n<<1)))
                lpc31_epcomplete (priv, (n<<1));
              if (mask & LPC31_ENDPTMASK((n<<1)+1))
                lpc31_epcomplete(priv, (n<<1)+1);
            }
        }

      /* Handle setup interrupts */
      uint32_t setupstat = lpc31_getreg(LPC31_USBDEV_ENDPTSETUPSTAT);
      if (setupstat)
        {
          /* Clear the endpoint complete CTRL OUT and IN when a Setup is received */
          lpc31_putreg(LPC31_ENDPTMASK(LPC31_EP0_IN) | LPC31_ENDPTMASK(LPC31_EP0_OUT), 
                       LPC31_USBDEV_ENDPTCOMPLETE);
      
          if (setupstat & LPC31_ENDPTMASK(LPC31_EP0_OUT))
            {
              usbtrace(TRACE_INTDECODE(LPC31_TRACEINTID_EP0SETUP), setupstat);
              lpc31_ep0setup(priv);
            }
        }
    }

  if (disr & USBDEV_USBSTS_NAKI)
    {
      uint32_t pending = lpc31_getreg(LPC31_USBDEV_ENDPTNAK) & lpc31_getreg(LPC31_USBDEV_ENDPTNAKEN);
      if (pending)
        {
          /* We shouldn't see NAK interrupts except on Endpoint 0 */
          if (pending & LPC31_ENDPTMASK(0))
              lpc31_ep0nak(priv, 0);
          if (pending & LPC31_ENDPTMASK(1))
              lpc31_ep0nak(priv, 1);
        }

      /* Clear the interrupts */
      lpc31_putreg(pending, LPC31_USBDEV_ENDPTNAK);
    }

  usbtrace(TRACE_INTEXIT(LPC31_TRACEINTID_USB), 0);
  return OK;
}

/*******************************************************************************
 * Endpoint operations
 *******************************************************************************/

/*******************************************************************************
 * Name: lpc31_epconfigure
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

static int lpc31_epconfigure(FAR struct usbdev_ep_s *ep,
                               FAR const struct usb_epdesc_s *desc,
                               bool last)
{
  FAR struct lpc31_ep_s *privep = (FAR struct lpc31_ep_s *)ep;

  usbtrace(TRACE_EPCONFIGURE, privep->epphy);
  DEBUGASSERT(desc->addr == ep->eplog);

  /* Initialise EP capabilities */
  
  uint16_t maxsize = GETUINT16(desc->mxpacketsize);
  if ((desc->attr & USB_EP_ATTR_XFERTYPE_MASK) == USB_EP_ATTR_XFER_ISOC)
    {
      g_qh[privep->epphy].capability = (DQH_CAPABILITY_MAX_PACKET(maxsize) |
                    DQH_CAPABILITY_IOS |
                    DQH_CAPABILITY_ZLT);
    }
  else
    {
      g_qh[privep->epphy].capability = (DQH_CAPABILITY_MAX_PACKET(maxsize) |
                    DQH_CAPABILITY_ZLT);
    }

  /* Setup Endpoint Control Register */

  if (LPC31_EPPHYIN(privep->epphy))
    {
      /* Reset the data toggles */
      uint32_t cfg = USBDEV_ENDPTCTRL_TXR;

      /* Set the endpoint type */
      switch (desc->attr & USB_EP_ATTR_XFERTYPE_MASK)
        {
          case USB_EP_ATTR_XFER_CONTROL: cfg |= USBDEV_ENDPTCTRL_TXT_CTRL; break;
          case USB_EP_ATTR_XFER_ISOC:    cfg |= USBDEV_ENDPTCTRL_TXT_ISOC; break;
          case USB_EP_ATTR_XFER_BULK:    cfg |= USBDEV_ENDPTCTRL_TXT_BULK; break;
          case USB_EP_ATTR_XFER_INT:     cfg |= USBDEV_ENDPTCTRL_TXT_INTR; break;
        }
      lpc31_chgbits (0xFFFF0000, cfg, LPC31_USBDEV_ENDPTCTRL(privep->epphy));
    }
  else
    {
      /* Reset the data toggles */
      uint32_t cfg = USBDEV_ENDPTCTRL_RXR;

      /* Set the endpoint type */
      switch (desc->attr & USB_EP_ATTR_XFERTYPE_MASK)
        {
          case USB_EP_ATTR_XFER_CONTROL: cfg |= USBDEV_ENDPTCTRL_RXT_CTRL; break;
          case USB_EP_ATTR_XFER_ISOC:    cfg |= USBDEV_ENDPTCTRL_RXT_ISOC; break;
          case USB_EP_ATTR_XFER_BULK:    cfg |= USBDEV_ENDPTCTRL_RXT_BULK; break;
        }
      lpc31_chgbits (0x0000FFFF, cfg, LPC31_USBDEV_ENDPTCTRL(privep->epphy));
    }

  /* Reset endpoint status */
  privep->stalled = false;

  /* Enable the endpoint */
  if (LPC31_EPPHYIN(privep->epphy))
    lpc31_setbits (USBDEV_ENDPTCTRL_TXE, LPC31_USBDEV_ENDPTCTRL(privep->epphy));
  else
    lpc31_setbits (USBDEV_ENDPTCTRL_RXE, LPC31_USBDEV_ENDPTCTRL(privep->epphy));
  
   return OK;
}

/*******************************************************************************
 * Name: lpc31_epdisable
 *
 * Description:
 *   The endpoint will no longer be used
 *
 *******************************************************************************/

static int lpc31_epdisable(FAR struct usbdev_ep_s *ep)
{
  FAR struct lpc31_ep_s *privep = (FAR struct lpc31_ep_s *)ep;
  irqstate_t flags;

#ifdef CONFIG_DEBUG
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif
  usbtrace(TRACE_EPDISABLE, privep->epphy);

  flags = irqsave();

  /* Disable Endpoint */
  if (LPC31_EPPHYIN(privep->epphy))
    lpc31_clrbits (USBDEV_ENDPTCTRL_TXE, LPC31_USBDEV_ENDPTCTRL(privep->epphy));
  else
    lpc31_clrbits (USBDEV_ENDPTCTRL_RXE, LPC31_USBDEV_ENDPTCTRL(privep->epphy));

  privep->stalled = true;

  /* Cancel any ongoing activity */
  lpc31_cancelrequests(privep, -ESHUTDOWN);

  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Name: lpc31_epallocreq
 *
 * Description:
 *   Allocate an I/O request
 *
 *******************************************************************************/

static FAR struct usbdev_req_s *lpc31_epallocreq(FAR struct usbdev_ep_s *ep)
{
  FAR struct lpc31_req_s *privreq;

#ifdef CONFIG_DEBUG
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_INVALIDPARMS), 0);
      return NULL;
    }
#endif
  usbtrace(TRACE_EPALLOCREQ, ((FAR struct lpc31_ep_s *)ep)->epphy);

  privreq = (FAR struct lpc31_req_s *)malloc(sizeof(struct lpc31_req_s));
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_ALLOCFAIL), 0);
      return NULL;
    }

  memset(privreq, 0, sizeof(struct lpc31_req_s));
  return &privreq->req;
}

/*******************************************************************************
 * Name: lpc31_epfreereq
 *
 * Description:
 *   Free an I/O request
 *
 *******************************************************************************/

static void lpc31_epfreereq(FAR struct usbdev_ep_s *ep, FAR struct usbdev_req_s *req)
{
  FAR struct lpc31_req_s *privreq = (FAR struct lpc31_req_s *)req;

#ifdef CONFIG_DEBUG
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_INVALIDPARMS), 0);
      return;
    }
#endif

  usbtrace(TRACE_EPFREEREQ, ((FAR struct lpc31_ep_s *)ep)->epphy);
  free(privreq);
}

/*******************************************************************************
 * Name: lpc31_epallocbuffer
 *
 * Description:
 *   Allocate an I/O buffer
 *
 *******************************************************************************/

#ifdef CONFIG_USBDEV_DMA
static void *lpc31_epallocbuffer(FAR struct usbdev_ep_s *ep, unsigned bytes)
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
 * Name: lpc31_epfreebuffer
 *
 * Description:
 *   Free an I/O buffer
 *
 *******************************************************************************/

#ifdef CONFIG_USBDEV_DMA
static void lpc31_epfreebuffer(FAR struct usbdev_ep_s *ep, FAR void *buf)
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
 * Name: lpc31_epsubmit
 *
 * Description:
 *   Submit an I/O request to the endpoint
 *
 *******************************************************************************/

static int lpc31_epsubmit(FAR struct usbdev_ep_s *ep, FAR struct usbdev_req_s *req)
{
  FAR struct lpc31_req_s *privreq = (FAR struct lpc31_req_s *)req;
  FAR struct lpc31_ep_s *privep = (FAR struct lpc31_ep_s *)ep;
  FAR struct lpc31_usbdev_s *priv;
  irqstate_t flags;
  int ret = OK;

#ifdef CONFIG_DEBUG
  if (!req || !req->callback || !req->buf || !ep)
    {
      usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_INVALIDPARMS), 0);
      ullvdbg("req=%p callback=%p buf=%p ep=%p\n", req, req->callback, req->buf, ep);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPSUBMIT, privep->epphy);
  priv = privep->dev;

  if (!priv->driver || priv->usbdev.speed == USB_SPEED_UNKNOWN)
    {
      usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_NOTCONFIGURED), priv->usbdev.speed);
      return -ESHUTDOWN;
    }

  /* Handle the request from the class driver */

  req->result = -EINPROGRESS;
  req->xfrd   = 0;

  /* Disable Interrupts */

  flags = irqsave();

  /* If we are stalled, then drop all requests on the floor */

  if (privep->stalled)
    {
      ret = -EBUSY;
    }
  else 
    {
      /* Add the new request to the request queue for the endpoint */

      if (LPC31_EPPHYIN(privep->epphy))
        usbtrace(TRACE_INREQQUEUED(privep->epphy), privreq->req.len);
      else
        usbtrace(TRACE_OUTREQQUEUED(privep->epphy), privreq->req.len);

      if (lpc31_rqenqueue(privep, privreq))
        {
          lpc31_progressep(privep);
        }
    }

  irqrestore(flags);
  return ret;
}

/*******************************************************************************
 * Name: lpc31_epcancel
 *
 * Description:
 *   Cancel an I/O request previously sent to an endpoint
 *
 *******************************************************************************/

static int lpc31_epcancel(FAR struct usbdev_ep_s *ep, FAR struct usbdev_req_s *req)
{
  FAR struct lpc31_ep_s *privep = (FAR struct lpc31_ep_s *)ep;
  FAR struct lpc31_usbdev_s *priv;
  irqstate_t flags;

#ifdef CONFIG_DEBUG
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPCANCEL, privep->epphy);
  priv = privep->dev;

  flags = irqsave();
  
  /* FIXME: if the request is the first, then we need to flush the EP
   *         otherwise just remove it from the list
   *
   *  but ... all other implementations cancel all requests ...
   */

  lpc31_cancelrequests(privep, -ESHUTDOWN);
  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Name: lpc31_epstall
 *
 * Description:
 *   Stall or resume and endpoint
 *
 *******************************************************************************/

static int lpc31_epstall(FAR struct usbdev_ep_s *ep, bool resume)
{
  FAR struct lpc31_ep_s *privep = (FAR struct lpc31_ep_s *)ep;
  irqstate_t flags;

  /* STALL or RESUME the endpoint */

  flags = irqsave();
  usbtrace(resume ? TRACE_EPRESUME : TRACE_EPSTALL, privep->epphy);

  uint32_t addr    = LPC31_USBDEV_ENDPTCTRL(privep->epphy);
  uint32_t ctrl_xs = LPC31_EPPHYIN(privep->epphy) ? USBDEV_ENDPTCTRL_TXS : USBDEV_ENDPTCTRL_RXS;
  uint32_t ctrl_xr = LPC31_EPPHYIN(privep->epphy) ? USBDEV_ENDPTCTRL_TXR : USBDEV_ENDPTCTRL_RXR;

  if (resume)
    {
      privep->stalled = false;

      /* Clear stall and reset the data toggle */

      lpc31_chgbits (ctrl_xs | ctrl_xr, ctrl_xr, addr);
    }
  else
    {
      privep->stalled = true;

      lpc31_setbits (ctrl_xs, addr);
    }

  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Device operations
 *******************************************************************************/

/*******************************************************************************
 * Name: lpc31_allocep
 *
 * Description:
 *   Allocate an endpoint matching the parameters.
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

static FAR struct usbdev_ep_s *lpc31_allocep(FAR struct usbdev_s *dev, uint8_t eplog,
                                               bool in, uint8_t eptype)
{
  FAR struct lpc31_usbdev_s *priv = (FAR struct lpc31_usbdev_s *)dev;
  uint32_t epset = LPC31_EPALLSET & ~LPC31_EPCTRLSET;
  irqstate_t flags;
  int epndx = 0;

  usbtrace(TRACE_DEVALLOCEP, (uint16_t)eplog);

  /* Ignore any direction bits in the logical address */

  eplog = USB_EPNO(eplog);

  /* A logical address of 0 means that any endpoint will do */

  if (eplog > 0)
    {
      /* Otherwise, we will return the endpoint structure only for the requested
       * 'logical' endpoint.  All of the other checks will still be performed.
       *
       * First, verify that the logical endpoint is in the range supported by
       * by the hardware.
       */

      if (eplog >= LPC31_NLOGENDPOINTS)
        {
          usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_BADEPNO), (uint16_t)eplog);
          return NULL;
        }

      /* Convert the logical address to a physical OUT endpoint address and
       * remove all of the candidate endpoints from the bitset except for the
       * the IN/OUT pair for this logical address.
       */

      epset &= 3 << (eplog << 1);
    }

  /* Get the subset matching the requested direction */

  if (in)
    {
      epset &= LPC31_EPINSET;
    }
  else
    {
      epset &= LPC31_EPOUTSET;
    }

  /* Get the subset matching the requested type */

  switch (eptype)
    {
    case USB_EP_ATTR_XFER_INT: /* Interrupt endpoint */
      epset &= LPC31_EPINTRSET;
      break;

    case USB_EP_ATTR_XFER_BULK: /* Bulk endpoint */
      epset &= LPC31_EPBULKSET;
      break;

    case USB_EP_ATTR_XFER_ISOC: /* Isochronous endpoint */
      epset &= LPC31_EPISOCSET;
      break;

    case USB_EP_ATTR_XFER_CONTROL: /* Control endpoint -- not a valid choice */
    default:
      usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_BADEPTYPE), (uint16_t)eptype);
      return NULL;
    }

  /* Is the resulting endpoint supported by the LPC313x? */

  if (epset)
    {
      /* Yes.. now see if any of the request endpoints are available */

      flags = irqsave();
      epset &= priv->epavail;
      if (epset)
        {
          /* Select the lowest bit in the set of matching, available endpoints */

          for (epndx = 2; epndx < LPC31_NPHYSENDPOINTS; epndx++)
            {
              uint32_t bit = 1 << epndx;
              if ((epset & bit) != 0)
                {
                  /* Mark the IN/OUT endpoint no longer available */

                  priv->epavail &= ~(3 << (bit & ~1));
                  irqrestore(flags);

                  /* And return the pointer to the standard endpoint structure */

                  return &priv->eplist[epndx].ep;
                }
            }
          /* Shouldn't get here */
        }
      irqrestore(flags);
    }

  usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_NOEP), (uint16_t)eplog);
  return NULL;
}

/*******************************************************************************
 * Name: lpc31_freeep
 *
 * Description:
 *   Free the previously allocated endpoint
 *
 *******************************************************************************/

static void lpc31_freeep(FAR struct usbdev_s *dev, FAR struct usbdev_ep_s *ep)
{
  FAR struct lpc31_usbdev_s *priv = (FAR struct lpc31_usbdev_s *)dev;
  FAR struct lpc31_ep_s *privep = (FAR struct lpc31_ep_s *)ep;
  irqstate_t flags;

  usbtrace(TRACE_DEVFREEEP, (uint16_t)privep->epphy);

  if (priv && privep)
    {
      /* Mark the endpoint as available */

      flags = irqsave();
      priv->epavail |= (1 << privep->epphy);
      irqrestore(flags);
    }
}

/*******************************************************************************
 * Name: lpc31_getframe
 *
 * Description:
 *   Returns the current frame number
 *
 *******************************************************************************/

static int lpc31_getframe(struct usbdev_s *dev)
{
#ifdef CONFIG_LPC31_USBDEV_FRAME_INTERRUPT
  FAR struct lpc31_usbdev_s *priv = (FAR struct lpc31_usbdev_s *)dev;

  /* Return last valid value of SOF read by the interrupt handler */

  usbtrace(TRACE_DEVGETFRAME, (uint16_t)priv->sof);
  return priv->sof;
#else
  /* Return the last frame number detected by the hardware */

  usbtrace(TRACE_DEVGETFRAME, 0);

  /* FIXME: this actually returns the micro frame number! */
  return (int)lpc31_getreg(LPC31_USBDEV_FRINDEX_OFFSET);
#endif
}

/*******************************************************************************
 * Name: lpc31_wakeup
 *
 * Description:
 *   Tries to wake up the host connected to this device
 *
 *******************************************************************************/

static int lpc31_wakeup(struct usbdev_s *dev)
{
  irqstate_t flags;

  usbtrace(TRACE_DEVWAKEUP, 0);

  flags = irqsave();
  lpc31_setbits(USBDEV_PRTSC1_FPR, LPC31_USBDEV_PORTSC1);
  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Name: lpc31_selfpowered
 *
 * Description:
 *   Sets/clears the device selfpowered feature 
 *
 *******************************************************************************/

static int lpc31_selfpowered(struct usbdev_s *dev, bool selfpowered)
{
  FAR struct lpc31_usbdev_s *priv = (FAR struct lpc31_usbdev_s *)dev;

  usbtrace(TRACE_DEVSELFPOWERED, (uint16_t)selfpowered);

#ifdef CONFIG_DEBUG
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_INVALIDPARMS), 0);
      return -ENODEV;
    }
#endif

  priv->selfpowered = selfpowered;
  return OK;
}

/*******************************************************************************
 * Name: lpc31_pullup
 *
 * Description:
 *   Software-controlled connect to/disconnect from USB host
 *
 *******************************************************************************/

static int lpc31_pullup(struct usbdev_s *dev, bool enable)
{
  usbtrace(TRACE_DEVPULLUP, (uint16_t)enable);

  irqstate_t flags = irqsave();
  if (enable)
    lpc31_setbits (USBDEV_USBCMD_RS, LPC31_USBDEV_USBCMD);
  else
    lpc31_clrbits (USBDEV_USBCMD_RS, LPC31_USBDEV_USBCMD);
  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Public Functions
 *******************************************************************************/

/*******************************************************************************
 * Name: up_usbinitialize
 *
 * Description:
 *   Initialize USB hardware.
 *
 * Assumptions:
 * - This function is called very early in the initialization sequence
 * - PLL and GIO pin initialization is not performed here but should been in
 *   the low-level  boot logic:  PLL1 must be configured for operation at 48MHz
 *   and P0.23 and PO.31 in PINSEL1 must be configured for Vbus and USB connect
 *   LED.
 *
 *******************************************************************************/

void up_usbinitialize(void)
{
  struct lpc31_usbdev_s *priv = &g_usbdev;
  int i;

  usbtrace(TRACE_DEVINIT, 0);

  /* Disable USB interrupts */
  
  lpc31_putreg(0, LPC31_USBDEV_USBINTR);

  /* Initialize the device state structure */

  memset(priv, 0, sizeof(struct lpc31_usbdev_s));
  priv->usbdev.ops = &g_devops;
  priv->usbdev.ep0 = &priv->eplist[LPC31_EP0_IN].ep;
  priv->epavail    = LPC31_EPALLSET;

  /* Initialize the endpoint list */

  for (i = 0; i < LPC31_NPHYSENDPOINTS; i++)
    {
      uint32_t bit = 1 << i;

      /* Set endpoint operations, reference to driver structure (not
       * really necessary because there is only one controller), and
       * the physical endpoint number (which is just the index to the
       * endpoint).
       */
      priv->eplist[i].ep.ops       = &g_epops;
      priv->eplist[i].dev          = priv;

      /* The index, i, is the physical endpoint address;  Map this
       * to a logical endpoint address usable by the class driver.
       */

      priv->eplist[i].epphy        = i;
      if (LPC31_EPPHYIN(i))
        {
          priv->eplist[i].ep.eplog = LPC31_EPPHYIN2LOG(i);
        }
      else
        {
          priv->eplist[i].ep.eplog = LPC31_EPPHYOUT2LOG(i);
        }

      /* The maximum packet size may depend on the type of endpoint */

      if ((LPC31_EPCTRLSET & bit) != 0)
        {
          priv->eplist[i].ep.maxpacket = LPC31_EP0MAXPACKET;
        }
      else if ((LPC31_EPINTRSET & bit) != 0)
        {
          priv->eplist[i].ep.maxpacket = LPC31_INTRMAXPACKET;
        }
      else if ((LPC31_EPBULKSET & bit) != 0)
        {
          priv->eplist[i].ep.maxpacket = LPC31_BULKMAXPACKET;
        }
      else /* if ((LPC31_EPISOCSET & bit) != 0) */
        {
          priv->eplist[i].ep.maxpacket = LPC31_ISOCMAXPACKET;
        }
    }

  /* Enable USB to AHB clock and to Event router*/

  lpc31_enableclock (CLKID_USBOTGAHBCLK);
  lpc31_enableclock (CLKID_EVENTROUTERPCLK);

  /* Reset USB block */

  lpc31_softreset (RESETID_USBOTGAHBRST);

  /* Enable USB OTG PLL and wait for lock */

  lpc31_putreg (0, LPC31_SYSCREG_USB_ATXPLLPDREG);
  
  uint32_t bank = EVNTRTR_BANK(EVENTRTR_USBATXPLLLOCK);
  uint32_t bit  = EVNTRTR_BIT(EVENTRTR_USBATXPLLLOCK);

  while (! (lpc31_getreg(LPC31_EVNTRTR_RSR(bank)) & (1 << bit)))
    ;

  /* Enable USB AHB clock */

  lpc31_enableclock (CLKID_USBOTGAHBCLK);

  /* Reset the controller */

  lpc31_putreg (USBDEV_USBCMD_RST, LPC31_USBDEV_USBCMD);
  while (lpc31_getreg (LPC31_USBDEV_USBCMD) & USBDEV_USBCMD_RST)
      ;

  /* Attach USB controller interrupt handler */

  if (irq_attach(LPC31_IRQ_USBOTG, lpc31_usbinterrupt) != 0)
    {
      usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_IRQREGISTRATION),
               (uint16_t)LPC31_IRQ_USBOTG);
      goto errout;
    }


  /* Program the controller to be the USB device controller */

  lpc31_putreg (USBDEV_USBMODE_SDIS | USBDEV_USBMODE_SLOM | USBDEV_USBMODE_CMDEVICE,
          LPC31_USBDEV_USBMODE);

  /* Disconnect device */

  lpc31_pullup(&priv->usbdev, false);

  /* Reset/Re-initialize the USB hardware */

  lpc31_usbreset(priv);

  return;

errout:
  up_usbuninitialize();
}

/*******************************************************************************
 * Name: up_usbuninitialize
 *******************************************************************************/

void up_usbuninitialize(void)
{
  struct lpc31_usbdev_s *priv = &g_usbdev;
  irqstate_t flags;

  usbtrace(TRACE_DEVUNINIT, 0);

  if (priv->driver)
    {
      usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_DRIVERREGISTERED), 0);
      usbdev_unregister(priv->driver);
    }

  /* Disconnect device */

  flags = irqsave();
  lpc31_pullup(&priv->usbdev, false);
  priv->usbdev.speed = USB_SPEED_UNKNOWN;

  /* Disable and detach IRQs */

  up_disable_irq(LPC31_IRQ_USBOTG);
  irq_detach(LPC31_IRQ_USBOTG);

  /* Reset the controller */

  lpc31_putreg (USBDEV_USBCMD_RST, LPC31_USBDEV_USBCMD);
  while (lpc31_getreg (LPC31_USBDEV_USBCMD) & USBDEV_USBCMD_RST)
      ;

  /* Turn off USB power and clocking */

  lpc31_disableclock (CLKID_USBOTGAHBCLK);
  lpc31_disableclock (CLKID_EVENTROUTERPCLK);


  irqrestore(flags);
}

/*******************************************************************************
 * Name: usbdev_register
 *
 * Description:
 *   Register a USB device class driver. The class driver's bind() method will be
 *   called to bind it to a USB device driver.
 *
 *******************************************************************************/

int usbdev_register(struct usbdevclass_driver_s *driver)
{
  int ret;

  usbtrace(TRACE_DEVREGISTER, 0);

#ifdef CONFIG_DEBUG
  if (!driver || !driver->ops->bind || !driver->ops->unbind ||
      !driver->ops->disconnect || !driver->ops->setup)
    {
      usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }

  if (g_usbdev.driver)
    {
      usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_DRIVER), 0);
      return -EBUSY;
    }
#endif

  /* First hook up the driver */

  g_usbdev.driver = driver;

  /* Then bind the class driver */

  ret = CLASS_BIND(driver, &g_usbdev.usbdev);
  if (ret)
    {
      usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_BINDFAILED), (uint16_t)-ret);
      g_usbdev.driver = NULL;
    }
  else
    {
      /* Enable USB controller interrupts */

      up_enable_irq(LPC31_IRQ_USBOTG);

      /* FIXME: nothing seems to call DEV_CONNECT(), but we need to set
       *        the RS bit to enable the controller.  It kind of makes sense 
       *        to do this after the class has bound to us...
       * GEN:   This bug is really in the class driver.  It should make the
       *        soft connect when it is ready to be enumerated.  I have added
       *        that logic to the class drivers but left this logic here.
       */

      lpc31_pullup(&g_usbdev.usbdev, true);
    }
  return ret;
}

/*******************************************************************************
 * Name: usbdev_unregister
 *
 * Description:
 *   Un-register usbdev class driver.If the USB device is connected to a USB host,
 *   it will first disconnect().  The driver is also requested to unbind() and clean
 *   up any device state, before this procedure finally returns.
 *
 *******************************************************************************/

int usbdev_unregister(struct usbdevclass_driver_s *driver)
{
  usbtrace(TRACE_DEVUNREGISTER, 0);

#ifdef CONFIG_DEBUG
  if (driver != g_usbdev.driver)
    {
      usbtrace(TRACE_DEVERROR(LPC31_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Unbind the class driver */

  CLASS_UNBIND(driver, &g_usbdev.usbdev);

  /* Disable USB controller interrupts */

  up_disable_irq(LPC31_IRQ_USBOTG);

  /* Unhook the driver */

  g_usbdev.driver = NULL;
  return OK;
}
