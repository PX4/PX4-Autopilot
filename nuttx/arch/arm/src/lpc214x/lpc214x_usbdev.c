/*******************************************************************************
 * arch/arm/src/lpc214x/lpc214x_usbdev.c
 *
 *   Copyright (C) 2008-2010, 2012-2013 Gregory Nutt. All rights reserved.
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

#include "lpc214x_usbdev.h"
#include "lpc214x_pll.h"
#include "lpc214x_power.h"

/*******************************************************************************
 * Definitions
 *******************************************************************************/

/* Configuration ***************************************************************/

#ifndef CONFIG_USBDEV_EP0_MAXSIZE
#  define CONFIG_USBDEV_EP0_MAXSIZE 64
#endif

#ifndef  CONFIG_USBDEV_MAXPOWER
#  define CONFIG_USBDEV_MAXPOWER 100  /* mA */
#endif

#define USB_SLOW_INT USBDEV_DEVINT_EPSLOW
#define USB_DEVSTATUS_INT USBDEV_DEVINT_DEVSTAT

#ifdef CONFIG_LPC214X_USBDEV_EPFAST_INTERRUPT
#  define USB_FAST_INT USBDEV_DEVINT_EPFAST
#else
#  define USB_FAST_INT 0
#endif

/* Extremely detailed register debug that you would normally never want
 * enabled.
 */

#undef CONFIG_LPC214X_USBDEV_REGDEBUG

/* Enable reading SOF from interrupt handler vs. simply reading on demand.  Probably
 * a bad idea... Unless there is some issue with sampling the SOF from hardware
 * asynchronously.
 */

#ifdef CONFIG_LPC214X_USBDEV_FRAME_INTERRUPT
#  define USB_FRAME_INT USBDEV_DEVINT_FRAME
#else
#  define USB_FRAME_INT 0
#endif

#ifdef CONFIG_DEBUG
#  define USB_ERROR_INT USBDEV_DEVINT_EPRINT
#else
#  define USB_ERROR_INT 0
#endif

/* Number of DMA descriptors */

#ifdef CONFIG_LPC214X_USBDEV_DMA
# error DMA SUPPORT NOT YET FULLY IMPLEMENTED
#  ifndef CONFIG_LPC214X_USBDEV_NDMADESCRIPTORS
#    define CONFIG_LPC214X_USBDEV_NDMADESCRIPTORS 8
#  elif CONFIG_LPC214X_USBDEV_NDMADESCRIPTORS > 30
#    define CONFIG_LPC214X_USBDEV_NDMADESCRIPTORS 30
#  endif
#endif

/* Debug ***********************************************************************/

/* Trace error codes */

#define LPC214X_TRACEERR_ALLOCFAIL            0x0001
#define LPC214X_TRACEERR_BADCLEARFEATURE      0x0002
#define LPC214X_TRACEERR_BADDEVGETSTATUS      0x0003
#define LPC214X_TRACEERR_BADEPNO              0x0004
#define LPC214X_TRACEERR_BADEPGETSTATUS       0x0005
#define LPC214X_TRACEERR_BADEPTYPE            0x0006
#define LPC214X_TRACEERR_BADGETCONFIG         0x0007
#define LPC214X_TRACEERR_BADGETSETDESC        0x0008
#define LPC214X_TRACEERR_BADGETSTATUS         0x0009
#define LPC214X_TRACEERR_BADSETADDRESS        0x000a
#define LPC214X_TRACEERR_BADSETCONFIG         0x000b
#define LPC214X_TRACEERR_BADSETFEATURE        0x000c
#define LPC214X_TRACEERR_BINDFAILED           0x000d
#define LPC214X_TRACEERR_DISPATCHSTALL        0x000e
#define LPC214X_TRACEERR_DMABUSY              0x000f
#define LPC214X_TRACEERR_DRIVER               0x0010
#define LPC214X_TRACEERR_DRIVERREGISTERED     0x0011
#define LPC214X_TRACEERR_EP0INSTALLED         0x0012
#define LPC214X_TRACEERR_EP0OUTSTALLED        0x0013
#define LPC214X_TRACEERR_EP0SETUPSTALLED      0x0014
#define LPC214X_TRACEERR_EPINNULLPACKET       0x0015
#define LPC214X_TRACEERR_EPOUTNULLPACKET      0x0016
#define LPC214X_TRACEERR_EPREAD               0x0017
#define LPC214X_TRACEERR_INVALIDCMD           0x0018
#define LPC214X_TRACEERR_INVALIDCTRLREQ       0x0019
#define LPC214X_TRACEERR_INVALIDPARMS         0x001a
#define LPC214X_TRACEERR_IRQREGISTRATION      0x001b
#define LPC214X_TRACEERR_NODMADESC            0x001c
#define LPC214X_TRACEERR_NOEP                 0x001d
#define LPC214X_TRACEERR_NOTCONFIGURED        0x001e
#define LPC214X_TRACEERR_REQABORTED           0x001f

/* Trace interrupt codes */

#define LPC214X_TRACEINTID_USB                0x0001
#define LPC214X_TRACEINTID_CLEARFEATURE       0x0002
#define LPC214X_TRACEINTID_CONNECTCHG         0x0003
#define LPC214X_TRACEINTID_CONNECTED          0x0004
#define LPC214X_TRACEINTID_DEVGETSTATUS       0x0005
#define LPC214X_TRACEINTID_DEVRESET           0x0006
#define LPC214X_TRACEINTID_DEVSTAT            0x0007
#define LPC214X_TRACEINTID_DISCONNECTED       0x0008
#define LPC214X_TRACEINTID_DISPATCH           0x0009
#define LPC214X_TRACEINTID_EP0IN              0x000a
#define LPC214X_TRACEINTID_EP0OUT             0x000b
#define LPC214X_TRACEINTID_EP0SETUP           0x000c
#define LPC214X_TRACEINTID_EPDMA              0x000d
#define LPC214X_TRACEINTID_EPFAST             0x000e
#define LPC214X_TRACEINTID_EPGETSTATUS        0x000f
#define LPC214X_TRACEINTID_EPIN               0x0010
#define LPC214X_TRACEINTID_EPINQEMPTY         0x0011
#define LPC214X_TRACEINTID_EP0INSETADDRESS    0x0012
#define LPC214X_TRACEINTID_EPOUT              0x0013
#define LPC214X_TRACEINTID_EPOUTQEMPTY        0x0014
#define LPC214X_TRACEINTID_EP0SETUPSETADDRESS 0x0015
#define LPC214X_TRACEINTID_EPRINT             0x0016
#define LPC214X_TRACEINTID_EPSLOW             0x0017
#define LPC214X_TRACEINTID_FRAME              0x0018
#define LPC214X_TRACEINTID_GETCONFIG          0x0019
#define LPC214X_TRACEINTID_GETSETDESC         0x001a
#define LPC214X_TRACEINTID_GETSETIF           0x001b
#define LPC214X_TRACEINTID_GETSTATUS          0x001c
#define LPC214X_TRACEINTID_IFGETSTATUS        0x001d
#define LPC214X_TRACEINTID_SETCONFIG          0x001e
#define LPC214X_TRACEINTID_SETFEATURE         0x001f
#define LPC214X_TRACEINTID_SUSPENDCHG         0x0020
#define LPC214X_TRACEINTID_SYNCHFRAME         0x0021

/* Hardware interface **********************************************************/

/* Macros for testing the device status response */

#define DEVSTATUS_CONNECT(s)    (((s)&USBDEV_DEVSTATUS_CONNECT)!=0)
#define DEVSTATUS_CONNCHG(s)    (((s)&USBDEV_DEVSTATUS_CONNCHG)!=0)
#define DEVSTATUS_SUSPEND(s)    (((s)&USBDEV_DEVSTATUS_SUSPEND)!=0)
#define DEVSTATUS_SUSPCHG(s)    (((s)&USBDEV_DEVSTATUS_SUSPCHG)!=0)
#define DEVSTATUS_RESET(s)      (((s)&USBDEV_DEVSTATUS_RESET)!=0)

/* If this bit is set in the lpc214x_epread response, it means that the
 * recevied packet was overwritten by a later setup packet (ep0 only).
 */

#define LPC214X_READOVERRUN_BIT (0x80000000)
#define LPC214X_READOVERRUN(s)  (((s) & LPC214X_READOVERRUN_BIT) != 0)

/* USB RAM  ********************************************************************
 *
 * UBS_UDCA is is list of 32 pointers to DMA desciptors located at the
 * beginning of USB RAM.  Each pointer points to a DMA descriptor with
 * assocated DMA buffer.
 */

#define USB_UDCA           (uint32_t*)LPC214X_USBDEV_RAMBASE)
#define USB_USCASIZE       (LPC214X_NPHYSENDPOINTS*sizeof(uint32_t))

/* Each descriptor must be aligned to a 128 address boundary */

#define USB_DDALIGNDOWN(a) ((a)&~0x7f)
#define USB_DDALIGNUP(a)   USB_DDALIGNDOWN((a)+0x7f)

#define USB_DDSIZE USB_DDALIGNDOWN((LPC214X_USBDEV_RAMSIZE-USB_USCASIZE)/CONFIG_LPC214X_USBDEV_NDMADESCRIPTORS)
#define USB_DDESC  ((struct lpc214x_dmadesc_s*)(LPC214X_USBDEV_RAMBASE+USB_USCASIZE))

#ifdef CONFIG_USBDEV_ISOCHRONOUS
#  define USB_DDESCSIZE (5*sizeof(uint32_t))
#else
#  define USB_DDESCSIZE (4*sizeof(uint32_t))
#endif

/* Endpoints ******************************************************************/

/* Number of endpoints */

#define LPC214X_NLOGENDPOINTS        (16)          /* ep0-15 */
#define LPC214X_NPHYSENDPOINTS       (32)          /* x2 for IN and OUT */

/* Odd physical endpoint numbers are IN; even are out */

#define LPC214X_EPPHYIN(epphy)       (((epphy)&1)!=0)
#define LPC214X_EPPHYOUT(epphy)      (((epphy)&1)==0)

#define LPC214X_EPPHYIN2LOG(epphy)   (((uint8_t)(epphy)>>1)|USB_DIR_IN)
#define LPC214X_EPPHYOUT2LOG(epphy)  (((uint8_t)(epphy)>>1)|USB_DIR_OUT)

/* Each endpoint has somewhat different characteristics */

#define LPC214X_EPALLSET             (0xffffffff)  /* All endpoints */
#define LPC214X_EPOUTSET             (0x55555555)  /* Even phy endpoint numbers are OUT EPs */
#define LPC214X_EPINSET              (0xaaaaaaaa)  /* Odd endpoint numbers are IN EPs */
#define LPC214X_EPCTRLSET            (0x00000003)  /* EP0 IN/OUT are control endpoints */
#define LPC214X_EPINTRSET            (0x0c30c30c)  /* Interrupt endpoints */
#define LPC214X_EPBULKSET            (0xf0c30c30)  /* Bulk endpoints */
#define LPC214X_EPISOCSET            (0x030c30c0)  /* Isochronous endpoints */
#define LPC214X_EPDBLBUFFER          (0xf3cf3cf0)  /* Double buffered endpoints */

#define LPC214X_EP0MAXPACKET         (64)          /* EP0 max packet size (1-64) */
#define LPC214X_BULKMAXPACKET        (64)          /* Bulk endpoint max packet (8/16/32/64) */
#define LPC214X_INTRMAXPACKET        (64)          /* Interrupt endpoint max packet (1 to 64) */
#define LPC214X_ISOCMAXPACKET        (512)         /* Acutally 1..1023 */

/* EP0 status.  EP0 transfers occur in a number of different contexts.  A
 * simple state machine is required to handle the various transfer complete
 * interrupt responses.  The following values are the various states:
 */
                                                   /*** INTERRUPT CAUSE ***/
#define LPC214X_EP0REQUEST           (0)           /* Normal request handling */
#define LPC214X_EP0STATUSIN          (1)           /* Status sent */
#define LPC214X_EP0STATUSOUT         (2)           /* Status received */
#define LPC214X_EP0SHORTWRITE        (3)           /* Short data sent with no request */
#define LPC214X_EP0SHORTWRSENT       (4)           /* Short data write complete */
#define LPC214X_EP0SETADDRESS        (5)           /* Set address received */
#define LPC214X_EP0WRITEREQUEST      (6)           /* EP0 write request sent */

/* Request queue operations ****************************************************/

#define lpc214x_rqempty(ep)          ((ep)->head == NULL)
#define lpc214x_rqpeek(ep)           ((ep)->head)

/*******************************************************************************
 * Private Types
 *******************************************************************************/

/* A container for a request so that the request make be retained in a list */

struct lpc214x_req_s
{
  struct usbdev_req_s    req;           /* Standard USB request */
  struct lpc214x_req_s  *flink;         /* Supports a singly linked list */
};

/* This is the internal representation of an endpoint */

struct lpc214x_ep_s
{
  /* Common endpoint fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_ep_s
   * to struct lpc214x_ep_s.
   */

  struct usbdev_ep_s      ep;            /* Standard endpoint structure */

  /* LPC214X-specific fields */

  struct lpc214x_usbdev_s *dev;          /* Reference to private driver data */
  struct lpc214x_req_s    *head;         /* Request list for this endpoint */
  struct lpc214x_req_s    *tail;
  uint8_t                 epphy;         /* Physical EP address */
  uint8_t                 stalled:1;     /* 1: Endpoint is stalled */
  uint8_t                 halted:1;      /* 1: Endpoint feature halted */
  uint8_t                 txbusy:1;      /* 1: TX endpoint FIFO full */
  uint8_t                 txnullpkt:1;   /* Null packet needed at end of transfer */
};

/* This represents a DMA descriptor */

#ifdef CONFIG_LPC214X_USBDEV_DMA
struct lpc214x_dmadesc_s
{
  uint32_t                nextdesc;      /* Address of the next DMA descripto in RAM */
  uint32_t                config;        /* Misc. bit encoded configuration information */
  uint32_t                start;         /* DMA start address */
  uint32_t                status;        /* Misc. bit encoded status inforamation */
#ifdef CONFIG_USBDEV_ISOCHRONOUS
  uint32_t                size;          /* Isochronous packet size address */
#endif
  uint8_t                 buffer[USB_DDSIZE-USB_DDESCSIZE];
};
#endif

/* This structure retains the state of the USB device controller */

struct lpc214x_usbdev_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_s
   * to structlpc214x_usbdev_s.
   */

  struct usbdev_s         usbdev;

  /* The bound device class driver */

  struct usbdevclass_driver_s *driver;

  /* LPC214X-specific fields */

  uint8_t                 devstatus;     /* Last response to device status command */
  uint8_t                 ep0state;      /* State of certain EP0 operations */
  uint8_t                 paddr;         /* Address assigned by SETADDRESS */
  uint8_t                 stalled:1;     /* 1: Protocol stalled */
  uint8_t                 selfpowered:1; /* 1: Device is self powered */
  uint8_t                 paddrset:1;    /* 1: Peripheral addr has been set */
  uint8_t                 attached:1;    /* 1: Host attached */
  uint8_t                 rxpending:1;   /* 1: RX pending */
  uint32_t                softprio;      /* Bitset of high priority interrupts */
  uint32_t                epavail;       /* Bitset of available endpoints */
#ifdef CONFIG_LPC214X_USBDEV_FRAME_INTERRUPT
  uint32_t                sof;           /* Last start-of-frame */
#endif

  /* Allocated DMA descriptor */

#ifdef CONFIG_LPC214X_USBDEV_DMA
  struct lpc214x_dmadesc_s *dmadesc;
#endif

  /* The endpoint list */

  struct lpc214x_ep_s     eplist[LPC214X_NPHYSENDPOINTS];
};

/*******************************************************************************
 * Private Function Prototypes
 *******************************************************************************/

/* Register operations ********************************************************/

#if defined(CONFIG_LPC214X_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
static uint32_t lpc214x_getreg(uint32_t addr);
static void lpc214x_putreg(uint32_t val, uint32_t addr);
#else
# define lpc214x_getreg(addr)     getreg32(addr)
# define lpc214x_putreg(val,addr) putreg32(val,addr)
#endif

/* Command operations **********************************************************/

static uint32_t lpc214x_usbcmd(uint16_t cmd, uint8_t data);

/* Request queue operations ****************************************************/

static FAR struct lpc214x_req_s *lpc214x_rqdequeue(FAR struct lpc214x_ep_s *privep);
static void lpc214x_rqenqueue(FAR struct lpc214x_ep_s *privep,
              FAR struct lpc214x_req_s *req);

/* Low level data transfers and request operations *****************************/

static void lpc214x_epwrite(uint8_t epphy, const uint8_t *data, uint32_t nbytes);
static int  lpc214x_epread(uint8_t epphy, uint8_t *data, uint32_t nbytes);
static inline void lpc214x_abortrequest(struct lpc214x_ep_s *privep,
              struct lpc214x_req_s *privreq, int16_t result);
static void lpc214x_reqcomplete(struct lpc214x_ep_s *privep, int16_t result);
static int  lpc214x_wrrequest(struct lpc214x_ep_s *privep);
static int  lpc214x_rdrequest(struct lpc214x_ep_s *privep);
static void lpc214x_cancelrequests(struct lpc214x_ep_s *privep);

/* Interrupt handling **********************************************************/

static struct lpc214x_ep_s *lpc214x_epfindbyaddr(struct lpc214x_usbdev_s *priv,
              uint16_t eplog);
static void lpc214x_eprealize(struct lpc214x_ep_s *privep, bool prio,
              uint32_t packetsize);
static uint8_t lpc214x_epclrinterrupt(uint8_t epphy);
static inline void lpc214x_ep0configure(struct lpc214x_usbdev_s *priv);
#ifdef CONFIG_LPC214X_USBDEV_DMA
static inline void lpc214x_dmareset(uint32_t enable);
#endif
static void lpc214x_usbreset(struct lpc214x_usbdev_s *priv);
static void lpc214x_dispatchrequest(struct lpc214x_usbdev_s *priv,
              const struct usb_ctrlreq_s *ctrl);
static inline void lpc214x_ep0setup(struct lpc214x_usbdev_s *priv);
static inline void lpc214x_ep0dataoutinterrupt(struct lpc214x_usbdev_s *priv);
static inline void lpc214x_ep0dataininterrupt(struct lpc214x_usbdev_s *priv);
static int lpc214x_usbinterrupt(int irq, FAR void *context);

#ifdef CONFIG_LPC214X_USBDEV_DMA
static int  lpc214x_dmasetup(struct lpc214x_usbdev_s *priv, uint8_t epphy,
              uint32_t epmaxsize, uint32_t nbytes, uint32_t *isocpacket,
              bool isochronous);
static void lpc214x_dmarestart(uint8_t epphy, uint32_t descndx);
static void lpc214x_dmadisable(uint8_t epphy);
#endif /* CONFIG_LPC214X_USBDEV_DMA */

/* Endpoint operations *********************************************************/

static int  lpc214x_epconfigure(FAR struct usbdev_ep_s *ep,
              const struct usb_epdesc_s *desc, bool last);
static int  lpc214x_epdisable(FAR struct usbdev_ep_s *ep);
static FAR struct usbdev_req_s *lpc214x_epallocreq(FAR struct usbdev_ep_s *ep);
static void lpc214x_epfreereq(FAR struct usbdev_ep_s *ep,
              FAR struct usbdev_req_s *);
#ifdef CONFIG_USBDEV_DMA
static FAR void *lpc214x_epallocbuffer(FAR struct usbdev_ep_s *ep,
              uint16_t nbytes);
static void lpc214x_epfreebuffer(FAR struct usbdev_ep_s *ep, void *buf);
#endif
static int  lpc214x_epsubmit(FAR struct usbdev_ep_s *ep,
              struct usbdev_req_s *req);
static int  lpc214x_epcancel(FAR struct usbdev_ep_s *ep,
              struct usbdev_req_s *req);
static int  lpc214x_epstall(FAR struct usbdev_ep_s *ep, bool resume);

/* USB device controller operations ********************************************/

static FAR struct usbdev_ep_s *lcp214x_allocep(FAR struct usbdev_s *dev,
              uint8_t epno, bool in, uint8_t eptype);
static void lpc214x_freeep(FAR struct usbdev_s *dev, FAR struct usbdev_ep_s *ep);
static int  lpc214x_getframe(struct usbdev_s *dev);
static int  lpc214x_wakeup(struct usbdev_s *dev);
static int  lpc214x_selfpowered(struct usbdev_s *dev, bool selfpowered);
static int  lpc214x_pullup(struct usbdev_s *dev, bool enable);

/*******************************************************************************
 * Private Data
 *******************************************************************************/

/* Since there is only a single USB interface, all status information can be
 * be simply retained in a single global instance.
 */

static struct lpc214x_usbdev_s g_usbdev;

static const struct usbdev_epops_s g_epops =
{
  .configure   = lpc214x_epconfigure,
  .disable     = lpc214x_epdisable,
  .allocreq    = lpc214x_epallocreq,
  .freereq     = lpc214x_epfreereq,
#ifdef CONFIG_USBDEV_DMA
  .allocbuffer = lpc214x_epallocbuffer,
  .freebuffer  = lpc214x_epfreebuffer,
#endif
  .submit      = lpc214x_epsubmit,
  .cancel      = lpc214x_epcancel,
  .stall       = lpc214x_epstall,
};

static const struct usbdev_ops_s g_devops =
{
  .allocep     = lcp214x_allocep,
  .freeep      = lpc214x_freeep,
  .getframe    = lpc214x_getframe,
  .wakeup      = lpc214x_wakeup,
  .selfpowered = lpc214x_selfpowered,
  .pullup      = lpc214x_pullup,
};

/*******************************************************************************
 * Public Data
 *******************************************************************************/

/*******************************************************************************
 * Private Functions
 *******************************************************************************/

/*******************************************************************************
 * Name: lpc214x_getreg
 *
 * Description:
 *   Get the contents of an LPC214x register
 *
 *******************************************************************************/

#if defined(CONFIG_LPC214X_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
static uint32_t lpc214x_getreg(uint32_t addr)
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
 * Name: lpc214x_putreg
 *
 * Description:
 *   Set the contents of an LPC214x register to a value
 *
 *******************************************************************************/

#if defined(CONFIG_LPC214X_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
static void lpc214x_putreg(uint32_t val, uint32_t addr)
{
  /* Show the register value being written */

  lldbg("%08x<-%08x\n", addr, val);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/*******************************************************************************
 * Name: lpc214x_usbcmd
 *
 * Description:
 *   Transmit commands to the USB engine
 *
 *******************************************************************************/

static uint32_t lpc214x_usbcmd(uint16_t cmd, uint8_t data)
{
  irqstate_t flags;
  uint32_t tmp = 0;

  /* Disable interrupt and clear CDFULL and CCEMPTY interrupt status */

  flags = irqsave();
  lpc214x_putreg(USBDEV_DEVINT_CDFULL|USBDEV_DEVINT_CCEMTY, LPC214X_USBDEV_DEVINTCLR);

  /* Load command + WR in command code register */

  lpc214x_putreg(((cmd & 0xff) << 16) + CMD_USB_CMDWR, LPC214X_USBDEV_CMDCODE);

  /* Wait until the command register is empty (CCEMPTY != 0, command is accepted) */

  while ((lpc214x_getreg(LPC214X_USBDEV_DEVINTST) & USBDEV_DEVINT_CCEMTY) == 0);

  /* Clear command register empty (CCEMPTY) interrupt */

  lpc214x_putreg(USBDEV_DEVINT_CCEMTY, LPC214X_USBDEV_DEVINTCLR);

  /* Determine next phase of the command */

  switch (cmd)
    {
      /* Write operations (1 byte of data) */

    case CMD_USB_DEV_SETADDRESS:
    case CMD_USB_DEV_CONFIG:
    case CMD_USB_DEV_SETMODE:
    case CMD_USB_DEV_SETSTATUS:
      {
        /* Send data + WR and wait for CCEMPTY */

        lpc214x_putreg((data << 16) + CMD_USB_DATAWR, LPC214X_USBDEV_CMDCODE);
        while ((lpc214x_getreg(LPC214X_USBDEV_DEVINTST) & USBDEV_DEVINT_CCEMTY) == 0);
      }
      break;

      /* 16 bit read operations */

    case CMD_USB_DEV_READFRAMENO:
    case CMD_USB_DEV_READTESTREG:
      {
        /* Send command code + RD and wait for CDFULL */

        lpc214x_putreg((cmd << 16) + CMD_USB_DATARD, LPC214X_USBDEV_CMDCODE);
        while ((lpc214x_getreg(LPC214X_USBDEV_DEVINTST) & USBDEV_DEVINT_CDFULL) == 0);

        /* Clear CDFULL and read LS data */

        lpc214x_putreg(USBDEV_DEVINT_CDFULL, LPC214X_USBDEV_DEVINTCLR);
        tmp = lpc214x_getreg(LPC214X_USBDEV_CMDDATA);

        /* Send command code + RD and wait for CDFULL */

        lpc214x_putreg((cmd << 16) + CMD_USB_DATARD, LPC214X_USBDEV_CMDCODE);
        while ((lpc214x_getreg(LPC214X_USBDEV_DEVINTST) & USBDEV_DEVINT_CDFULL) == 0);

        /* Read MS data */

        tmp |= lpc214x_getreg(LPC214X_USBDEV_CMDDATA) << 8;
      }
      break;

     /* 8-bit read operations */

    case CMD_USB_DEV_GETSTATUS:
    case CMD_USB_DEV_GETERRORCODE:
    case CMD_USB_DEV_READERRORSTATUS:
    case CMD_USB_EP_CLRBUFFER:
      {
        /* Send command code + RD and wait for CDFULL */

        lpc214x_putreg((cmd << 16) + CMD_USB_DATARD, LPC214X_USBDEV_CMDCODE);
        while ((lpc214x_getreg(LPC214X_USBDEV_DEVINTST) & USBDEV_DEVINT_CDFULL) == 0);

        /* Read data */

        tmp = lpc214x_getreg(LPC214X_USBDEV_CMDDATA);
      }
      break;

      /* No data transfer */

    case CMD_USB_EP_VALIDATEBUFFER:
      break;

    default:
      switch (cmd & 0x1e0)
        {
        case CMD_USB_EP_SELECT:
        case CMD_USB_EP_SELECTCLEAR:
          {
            /* Send command code + RD and wait for CDFULL */

            lpc214x_putreg((cmd << 16) + CMD_USB_DATARD, LPC214X_USBDEV_CMDCODE);
            while ((lpc214x_getreg(LPC214X_USBDEV_DEVINTST) & USBDEV_DEVINT_CDFULL) == 0);

            /* Read data */

            tmp = lpc214x_getreg(LPC214X_USBDEV_CMDDATA);
          }
          break;

        case CMD_USB_EP_SETSTATUS:
          {
            /* Send data + RD and wait for CCEMPTY */

            lpc214x_putreg((data << 16) + CMD_USB_DATAWR, LPC214X_USBDEV_CMDCODE);
            while ((lpc214x_getreg(LPC214X_USBDEV_DEVINTST) & USBDEV_DEVINT_CCEMTY) == 0);
          }
          break;

        default:
          usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_INVALIDCMD), 0);
          break;
        }
      break;
    }

  /* Restore the interrupt flags */

  irqrestore(flags);
  return tmp;
}

/*******************************************************************************
 * Name: lpc214x_rqdequeue
 *
 * Description:
 *   Remove a request from an endpoint request queue
 *
 *******************************************************************************/

static FAR struct lpc214x_req_s *lpc214x_rqdequeue(FAR struct lpc214x_ep_s *privep)
{
  FAR struct lpc214x_req_s *ret = privep->head;

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
 * Name: lpc214x_rqenqueue
 *
 * Description:
 *   Add a request from an endpoint request queue
 *
 *******************************************************************************/

static void lpc214x_rqenqueue(FAR struct lpc214x_ep_s *privep,
                              FAR struct lpc214x_req_s *req)
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
 * Name: lpc214x_epwrite
 *
 * Description:
 *   Endpoint write (IN)
 *
 *******************************************************************************/

static void lpc214x_epwrite(uint8_t epphy, const uint8_t *data, uint32_t nbytes)
{
  uint32_t value;
  bool aligned = (((uint32_t)data & 3) == 0);

  /* Set the write enable bit for this physical EP address. Bits 2-5 are
   * the logical endpoint number (0-15)
   */

  lpc214x_putreg(((epphy << 1) & LPC214X_USBCTRL_EPMASK) | LPC214X_USBCTRL_WREN,
                 LPC214X_USBDEV_CTRL);

  /* Set the transmit packet length (nbytes must be less than 2048) */

  lpc214x_putreg(nbytes, LPC214X_USBDEV_TXPLEN);

  /* Transfer the packet data */

  do
    {
      /* Zero length packets are a special case */

      if (nbytes)
        {
          if (aligned)
            {
              value = *(uint32_t*)data;
            }
          else
            {
              value =  (uint32_t)data[0]        | ((uint32_t)data[1] << 8) |
                      ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
            }

          lpc214x_putreg(value, LPC214X_USBDEV_TXDATA);
          data += 4;
        }
      else
        {
          /* Zero length packet */

          lpc214x_putreg(0, LPC214X_USBDEV_TXDATA);
        }
    }
  while ((lpc214x_getreg(LPC214X_USBDEV_CTRL) & LPC214X_USBCTRL_WREN) != 0);

  /* Done */

  lpc214x_putreg(0, LPC214X_USBDEV_CTRL);
  (void)lpc214x_usbcmd(CMD_USB_EP_SELECT | epphy, 0);
  (void)lpc214x_usbcmd(CMD_USB_EP_VALIDATEBUFFER, 0);
}

/*******************************************************************************
 * Name: lpc214x_epread
 *
 * Description:
 *   Endpoint read (OUT)
 *
 *******************************************************************************/

static int lpc214x_epread(uint8_t epphy, uint8_t *data, uint32_t nbytes)
{
  uint32_t pktlen;
  uint32_t result;
  uint32_t value;
  uint8_t  aligned = 0;

  /* If data is NULL, then we are being asked to read but discard the data.
   * For most cases, the resulting buffer will be aligned and we will be
   * able to do faster 32-bit transfers.
   */

  if (data)
    {
       if (((uint32_t)data & 3) == 0)
        {
          aligned = 1;
        }
       else
        {
          aligned = 2;
        }
    }

  /* Set the read enable bit for this physical EP address.  Bits 2-5 are
   * the logical endpoint number (0-15).
   */

  lpc214x_putreg(((epphy << 1) & LPC214X_USBCTRL_EPMASK) | LPC214X_USBCTRL_RDEN,
                 LPC214X_USBDEV_CTRL);

  /* Wait for packet buffer ready for reading */

  while ((lpc214x_getreg(LPC214X_USBDEV_RXPLEN) & USBDEV_RXPLEN_PKTRDY) == 0);

  /* Get the number of bytes of data to be read */

  pktlen = lpc214x_getreg(LPC214X_USBDEV_RXPLEN) & USBDEV_RXPLEN_PKTLENGTH;

  /* Read data from input buffer while read data is valid (DV) */

  while ((lpc214x_getreg(LPC214X_USBDEV_RXPLEN) & USBDEV_RXPLEN_DV) != 0)
    {
      value = lpc214x_getreg(LPC214X_USBDEV_RXDATA);
      if (aligned == 1)
        {
          *(uint32_t*)data = value;
          data += 4;
        }
      else if (aligned == 2)
        {
          *data++ = (uint8_t)value;
          *data++ = (uint8_t)(value >> 8);
          *data++ = (uint8_t)(value >> 16);
          *data++ = (uint8_t)(value >> 24);
        }
    }

  /* Done */

  lpc214x_putreg(0, LPC214X_USBDEV_CTRL);
  (void)lpc214x_usbcmd(CMD_USB_EP_SELECT | epphy, 0);
  result = lpc214x_usbcmd(CMD_USB_EP_CLRBUFFER, 0);

  /* The packet overrun bit in the clear buffer response is applicable only
   * on EP0 transfers.  If set it means that the recevied packet was overwritten
   * by a later setup packet.
   */

  if (epphy == LPC214X_EP0_OUT && (result & CMD_USB_CLRBUFFER_PO) != 0)
    {
      /* Pass this information in bit 31 */

      pktlen |= LPC214X_READOVERRUN_BIT;
    }
  return pktlen;
}

/*******************************************************************************
 * Name: lpc214x_abortrequest
 *
 * Description:
 *   Discard a request
 *
 *******************************************************************************/

static inline void lpc214x_abortrequest(struct lpc214x_ep_s *privep,
                                        struct lpc214x_req_s *privreq,
                                        int16_t result)
{
  usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_REQABORTED), (uint16_t)privep->epphy);

  /* Save the result in the request structure */

  privreq->req.result = result;

  /* Callback to the request completion handler */

  privreq->req.callback(&privep->ep, &privreq->req);
}

/*******************************************************************************
 * Name: lpc214x_reqcomplete
 *
 * Description:
 *   Handle termination of the request at the head of the endpoint request queue.
 *
 *******************************************************************************/

static void lpc214x_reqcomplete(struct lpc214x_ep_s *privep, int16_t result)
{
  struct lpc214x_req_s *privreq;
  int stalled = privep->stalled;
  irqstate_t flags;

  /* Remove the completed request at the head of the endpoint request list */

  flags = irqsave();
  privreq = lpc214x_rqdequeue(privep);
  irqrestore(flags);

  if (privreq)
    {
      /* If endpoint 0, temporarily reflect the state of protocol stalled
       * in the callback.
       */

      if (privep->epphy == LPC214X_EP0_IN)
        {
          privep->stalled = privep->dev->stalled;
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
 * Name: lpc214x_wrrequest
 *
 * Description:
 *   Send from the next queued write request
 *
 *******************************************************************************/

static int lpc214x_wrrequest(struct lpc214x_ep_s *privep)
{
  struct lpc214x_req_s *privreq;
  uint8_t *buf;
  int nbytes;
  int bytesleft;

  /* Check the request from the head of the endpoint request queue */

  privreq = lpc214x_rqpeek(privep);
  if (!privreq)
    {
      usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_EPINQEMPTY), 0);
      return OK;
    }

  ullvdbg("epphy=%d req=%p: len=%d xfrd=%d nullpkt=%d\n",
          privep->epphy, privreq, privreq->req.len, privreq->req.xfrd, privep->txnullpkt);

  /* Ignore any attempt to send a zero length packet on anything but EP0IN */

  if (privreq->req.len == 0)
    {
      if (privep->epphy == LPC214X_EP0_IN)
        {
          lpc214x_epwrite(LPC214X_EP0_IN, NULL, 0);
        }
      else
        {
          usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_EPINNULLPACKET), 0);
        }

      /* In any event, the request is complete */

      lpc214x_reqcomplete(privep, OK);
      return OK;
    }

  /* Otherwise send the data in the packet (in the DMA on case, we
   * may be resuming transfer already in progress.
   */
#warning REVISIT... If the EP supports double buffering, then we can do better

  /* Get the number of bytes left to be sent in the packet */

  bytesleft = privreq->req.len - privreq->req.xfrd;

  /* Send the next packet if (1) there are more bytes to be sent, or
   * (2) the last packet sent was exactly maxpacketsize (bytesleft == 0)
   */

  usbtrace(TRACE_WRITE(privep->epphy), privreq->req.xfrd);
  if (bytesleft >  0 || privep->txnullpkt)
    {
      /* Indicate that there is data in the TX FIFO.  This will be cleared
       * when the EPIN interrupt is received
       */

      privep->txbusy = 1;

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

      buf = privreq->req.buf + privreq->req.xfrd;
      lpc214x_epwrite(privep->epphy, buf, nbytes);

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
      lpc214x_reqcomplete(privep, OK);
    }

  return OK;
}

/*******************************************************************************
 * Name: lpc214x_rdrequest
 *
 * Description:
 *   Receive to the next queued read request
 *
 *******************************************************************************/

static int lpc214x_rdrequest(struct lpc214x_ep_s *privep)
{
  struct lpc214x_req_s *privreq;
  uint8_t *buf;
  int nbytesread;

  /* Check the request from the head of the endpoint request queue */

  privreq = lpc214x_rqpeek(privep);
  if (!privreq)
    {
      usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_EPOUTQEMPTY), 0);
      return OK;
    }

  ullvdbg("len=%d xfrd=%d nullpkt=%d\n",
          privreq->req.len, privreq->req.xfrd, privep->txnullpkt);

  /* Ignore any attempt to receive a zero length packet */

  if (privreq->req.len == 0)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_EPOUTNULLPACKET), 0);
      lpc214x_reqcomplete(privep, OK);
      return OK;
    }

  usbtrace(TRACE_READ(privep->epphy), privreq->req.xfrd);

  /* Receive the next packet */

  buf        = privreq->req.buf + privreq->req.xfrd;
  nbytesread = lpc214x_epread(privep->epphy, buf, privep->ep.maxpacket);
  if (nbytesread < 0)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_EPREAD), nbytesread);
      return ERROR;
    }

  /* If the receive buffer is full or if the last packet was not full
   * then we are finished with the transfer.
   */

  privreq->req.xfrd += nbytesread;
  if (privreq->req.xfrd >= privreq->req.len || nbytesread < privep->ep.maxpacket)
    {
      usbtrace(TRACE_COMPLETE(privep->epphy), privreq->req.xfrd);
      lpc214x_reqcomplete(privep, OK);
    }

  return OK;
}

/*******************************************************************************
 * Name: lpc214x_cancelrequests
 *
 * Description:
 *   Cancel all pending requests for an endpoint
 *
 *******************************************************************************/

static void lpc214x_cancelrequests(struct lpc214x_ep_s *privep)
{
  while (!lpc214x_rqempty(privep))
    {
      usbtrace(TRACE_COMPLETE(privep->epphy),
               (lpc214x_rqpeek(privep))->req.xfrd);
      lpc214x_reqcomplete(privep, -ESHUTDOWN);
    }
}

/*******************************************************************************
 * Name: lpc214x_epfindbyaddr
 *
 * Description:
 *   Find the physical endpoint structure corresponding to a logic endpoint
 *   address
 *
 *******************************************************************************/

static struct lpc214x_ep_s *lpc214x_epfindbyaddr(struct lpc214x_usbdev_s *priv,
              uint16_t eplog)
{
  struct lpc214x_ep_s *privep;
  int i;

  /* Endpoint zero is a special case */

  if (USB_EPNO(eplog) == 0)
    {
      return &priv->eplist[0];
    }

  /* Handle the remaining */

  for (i = 1; i < LPC214X_NPHYSENDPOINTS; i++)
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
 * Name: lpc214x_eprealize
 *
 * Description:
 *   Enable or disable an endpoint
 *
 *******************************************************************************/

static void lpc214x_eprealize(struct lpc214x_ep_s *privep, bool prio, uint32_t packetsize)
{
  struct lpc214x_usbdev_s *priv = privep->dev;
  uint32_t mask;
  uint32_t reg;

  /* Initialize endpoint software priority */

  mask = 1 << privep->epphy;
  if (prio)
    {
      priv->softprio = priv->softprio | mask;
    }
  else
    {
      priv->softprio = priv->softprio & ~mask;
    }

  /* Clear realize interrupt bit */

  lpc214x_putreg(USBDEV_DEVINT_EPRLZED, LPC214X_USBDEV_DEVINTCLR);

  /* Realize the endpoint */

  reg  = lpc214x_getreg(LPC214X_USBDEV_REEP);
  reg |= (1 << privep->epphy);
  lpc214x_putreg(reg, LPC214X_USBDEV_REEP);

  /* Set endpoint maximum packet size */

  lpc214x_putreg(privep->epphy, LPC214X_USBDEV_EPIND);
  lpc214x_putreg(packetsize, LPC214X_USBDEV_MAXPSIZE);

  /* Wait for Realize complete */

  while ((lpc214x_getreg(LPC214X_USBDEV_DEVINTST) & USBDEV_DEVINT_EPRLZED) == 0);

  /* Clear realize interrupt bit */

  lpc214x_putreg(USBDEV_DEVINT_EPRLZED,LPC214X_USBDEV_DEVINTCLR);
}

/*******************************************************************************
 * Name: lpc214x_epclrinterrupt
 *
 * Description:
 *   Clear the EP interrupt flag and return the current EP status
 *
 *******************************************************************************/

static uint8_t lpc214x_epclrinterrupt(uint8_t epphy)
{
  /* Clear the endpoint interrupt */

  lpc214x_putreg(1 << epphy, LPC214X_USBDEV_EPINTCLR);

  /* Wait for data in the command data register */

  while ((lpc214x_getreg(LPC214X_USBDEV_DEVINTST) & USBDEV_DEVINT_CDFULL) == 0);

  /* Return the value of the command data register */

  return lpc214x_getreg(LPC214X_USBDEV_CMDDATA);
}

/*******************************************************************************
 * Name: lpc214x_ep0configure
 *
 * Description:
 *   Configure endpoint 0
 *
 *******************************************************************************/

static inline void lpc214x_ep0configure(struct lpc214x_usbdev_s *priv)
{
  uint32_t inten;

  /* EndPoint 0 initialization */

  lpc214x_eprealize(&priv->eplist[LPC214X_CTRLEP_OUT], 0, CONFIG_USBDEV_EP0_MAXSIZE);
  lpc214x_eprealize(&priv->eplist[LPC214X_CTRLEP_IN], 1, CONFIG_USBDEV_EP0_MAXSIZE);

  /* Enable EP0 interrupts (not DMA) */

  inten = lpc214x_getreg(LPC214X_USBDEV_EPINTEN);
  inten |= 3; /* EP0 Rx and Tx */
  lpc214x_putreg(inten, LPC214X_USBDEV_EPINTEN);
}

/*******************************************************************************
 * Name: lpc214x_dmareset
 *
 * Description: Reset USB DMA
 *
 *******************************************************************************/

#ifdef CONFIG_LPC214X_USBDEV_DMA
static inline void lpc214x_dmareset(uint32_t enable)
{
  int i;

  /* Disable All DMA interrupts */

  lpc214x_putreg(0, LPC214X_USBDEV_DMAINTEN);

  /* DMA Disable */

  lpc214x_putreg(0xffffffff, LPC214X_USBDEV_EPDMADIS);

  /* DMA Request clear */

  putreq32(0xffffffff, LPC214X_USBDEV_DMARCLR);

  /* End of Transfer Interrupt Clear */

  putreq32(0xffffffff, LPC214X_USBDEV_EOTINTCLR);

  /* New DD Request Interrupt Clear */

  putreq32(0xffffffff, LPC214X_USBDEV_NDDRINTCLR);

  /* System Error Interrupt Clear */

  putreq32(0xffffffff, LPC214X_USBDEV_SYSERRINTCLR);

  /* Nullify all pointers in the UDCA */

  for (i = 0; i < LPC214X_NPHYSENDPOINTS; ++i)
    {
      USB_UDCA[i] = NULL;
    }

  /* Set USB UDCA Head register */

  lpc214x_putreg((uint32_t)USB_UDCA, LPC214X_USBDEV_UDCAH);

  /* Invalidate all DMA descriptors */

  for (i = 0; i < CONFIG_LPC214X_USBDEV_NDMADESCRIPTORS; ++i)
    {
      memset(&USB_DDESC[i], 0, USB_DDESCSIZE);
    }

  /* Enable DMA interrupts */

  lpc214x_putreg(enable, LPC214X_USBDEV_DMAINTEN);
}
#endif

/*******************************************************************************
 * Name: lpc214x_usbreset
 *
 * Description:
 *   Reset Usb engine
 *
 *******************************************************************************/

static void lpc214x_usbreset(struct lpc214x_usbdev_s *priv)
{
  int epphy;

  /* Disable all endpoint interrupts */

  lpc214x_putreg(0, LPC214X_USBDEV_EPINTEN);

  /* Frame is Hp interrupt */

  lpc214x_putreg(1, LPC214X_USBDEV_DEVINTPRI);

  /* Clear all pending interrupts */

  lpc214x_putreg(0xffffffff, LPC214X_USBDEV_EPINTCLR);
  lpc214x_putreg(0xffffffff, LPC214X_USBDEV_DEVINTCLR);

  /* Periperhal address is needed */

  priv->paddrset = 0;

  /* Reset endpoints */

  for (epphy = 0; epphy < LPC214X_NPHYSENDPOINTS; epphy++)
    {
      struct lpc214x_ep_s *privep = &priv->eplist[epphy];

      lpc214x_cancelrequests(privep);

      /* Reset endpoint status */

      privep->stalled = false;
    }

  /* Tell the class driver that we are disconnected. The class
   * driver should then accept any new configurations.
   */

  if (priv->driver)
    {
      CLASS_DISCONNECT(priv->driver, &priv->usbdev);
    }

  /* Endpoints not yet configured */

  lpc214x_usbcmd(CMD_USB_DEV_CONFIG, 0);

  /* EndPoint 0 initialization */

  lpc214x_ep0configure(priv);

  /* Enable End_of_Transfer_Interrupt and System_Error_Interrupt USB DMA
   * interrupts
   */

#ifdef CONFIG_LPC214X_USBDEV_DMA
  lpc214x_dmareset(CONFIG_LPC214X_USBDEV_DMAINTMASK);
#endif

  /* Enable Device interrupts */

  lpc214x_putreg(USB_SLOW_INT|USB_DEVSTATUS_INT|USB_FAST_INT|USB_FRAME_INT|USB_ERROR_INT,
                 LPC214X_USBDEV_DEVINTEN);
}

/*******************************************************************************
 * Name: lpc214x_dispatchrequest
 *
 * Description:
 *   Provide unhandled setup actions to the class driver. This is logically part
 *   of the USB interrupt handler.
 *
 *******************************************************************************/

static void lpc214x_dispatchrequest(struct lpc214x_usbdev_s *priv,
                                    const struct usb_ctrlreq_s *ctrl)
{
  int ret;

  usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_DISPATCH), 0);
  if (priv && priv->driver)
    {
      /* Forward to the control request to the class driver implementation */

      ret = CLASS_SETUP(priv->driver, &priv->usbdev, ctrl, NULL, 0);
      if (ret < 0)
        {
          /* Stall on failure */

          usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_DISPATCHSTALL), 0);
          priv->stalled = 1;
        }
    }
}

/*******************************************************************************
 * Name: lpc214x_ep0setup
 *
 * Description:
 *   USB Ctrl EP Setup Event. This is logically part of the USB interrupt
 *   handler.  This event occurs when a setup packet is receive on EP0 OUT.
 *
 *******************************************************************************/

static inline void lpc214x_ep0setup(struct lpc214x_usbdev_s *priv)
{
  struct lpc214x_ep_s *ep0 = &priv->eplist[LPC214X_EP0_OUT];
  struct lpc214x_ep_s *privep;
  struct lpc214x_req_s *privreq = lpc214x_rqpeek(ep0);
  struct usb_ctrlreq_s ctrl;
  uint16_t value;
  uint16_t index;
  uint16_t len;
  uint8_t  response[2];
  int    ret;

  /* Starting a control request? */

  if (priv->usbdev.speed == USB_SPEED_UNKNOWN)
    {
      priv->usbdev.speed = USB_SPEED_FULL;
      lpc214x_usbcmd(CMD_USB_DEV_CONFIG, 1);
    }

  /* Terminate any pending requests */

  while (!lpc214x_rqempty(ep0))
    {
      int16_t result = OK;
      if (privreq->req.xfrd != privreq->req.len)
        {
          result = -EPROTO;
        }

      usbtrace(TRACE_COMPLETE(ep0->epphy), privreq->req.xfrd);
      lpc214x_reqcomplete(ep0, result);
    }

  /* Assume NOT stalled */

  ep0->stalled  = 0;
  priv->stalled = 0;

  /* Read EP0 data */

  ret = lpc214x_epread(LPC214X_EP0_OUT, (uint8_t*)&ctrl, USB_SIZEOF_CTRLREQ);
  if (ret <= 0)
    {
      return;
    }

  /* And extract the little-endian 16-bit values to host order */

  value = GETUINT16(ctrl.value);
  index = GETUINT16(ctrl.index);
  len   = GETUINT16(ctrl.len);

  ullvdbg("type=%02x req=%02x value=%04x index=%04x len=%04x\n",
          ctrl.type, ctrl.req, value, index, len);

  /* Dispatch any non-standard requests */

  if ((ctrl.type & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_STANDARD)
    {
      lpc214x_dispatchrequest(priv, &ctrl);
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

        usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_GETSTATUS), 0);
        if (!priv->paddrset || len != 2 ||
            (ctrl.type & USB_REQ_DIR_IN) == 0 || value != 0)
          {
            priv->stalled = 1;
          }
        else
          {
            switch (ctrl.type & USB_REQ_RECIPIENT_MASK)
              {
              case USB_REQ_RECIPIENT_ENDPOINT:
                {
                  usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_EPGETSTATUS), 0);
                  privep = lpc214x_epfindbyaddr(priv, index);
                  if (!privep)
                    {
                      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_BADEPGETSTATUS), 0);
                      priv->stalled = 1;
                    }
                  else
                    {
                       if ((lpc214x_usbcmd(CMD_USB_EP_SELECT|privep->epphy, 0) & CMD_USB_EPSELECT_ST) != 0)
                         {
                           response[0] = 1; /* Stalled */
                         }
                       else
                         {
                           response[0] = 0; /* Not stalled */
                         }
                      response[1] = 0;
                      lpc214x_epwrite(LPC214X_EP0_IN, response, 2);
                      priv->ep0state = LPC214X_EP0SHORTWRITE;
                    }
                }
                break;

              case USB_REQ_RECIPIENT_DEVICE:
                {
                  if (index == 0)
                    {
                      usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_DEVGETSTATUS), 0);

                      /* Features:  Remote Wakeup=YES; selfpowered=? */

                      response[0] = (priv->selfpowered << USB_FEATURE_SELFPOWERED) |
                                    (1 << USB_FEATURE_REMOTEWAKEUP);
                      response[1] = 0;
                      lpc214x_epwrite(LPC214X_EP0_IN, response, 2);
                      priv->ep0state = LPC214X_EP0SHORTWRITE;
                    }
                  else
                    {
                      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_BADDEVGETSTATUS), 0);
                      priv->stalled = 1;
                    }
                }
                break;

              case USB_REQ_RECIPIENT_INTERFACE:
                {
                  usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_IFGETSTATUS), 0);
                  response[0] = 0;
                  response[1] = 0;
                  lpc214x_epwrite(LPC214X_EP0_IN, response, 2);
                  priv->ep0state = LPC214X_EP0SHORTWRITE;
                }
                break;

              default:
                {
                  usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_BADGETSTATUS), 0);
                  priv->stalled = 1;
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

        usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_CLEARFEATURE), 0);
        if ((ctrl.type & USB_REQ_RECIPIENT_MASK) != USB_REQ_RECIPIENT_ENDPOINT)
          {
            lpc214x_dispatchrequest(priv, &ctrl);
          }
        else if (priv->paddrset != 0 && value == USB_FEATURE_ENDPOINTHALT && len == 0 &&
                 (privep = lpc214x_epfindbyaddr(priv, index)) != NULL)
          {
            privep->halted = 0;
            ret = lpc214x_epstall(&privep->ep, true);
            lpc214x_epwrite(LPC214X_EP0_IN, NULL, 0);
            priv->ep0state = LPC214X_EP0STATUSIN;
          }
        else
          {
            usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_BADCLEARFEATURE), 0);
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

        usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_SETFEATURE), 0);
        if (((ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE) &&
            value == USB_FEATURE_TESTMODE)
          {
            ullvdbg("test mode: %d\n", index);
          }
        else if ((ctrl.type & USB_REQ_RECIPIENT_MASK) != USB_REQ_RECIPIENT_ENDPOINT)
          {
           lpc214x_dispatchrequest(priv, &ctrl);
          }
        else if (priv->paddrset != 0 && value == USB_FEATURE_ENDPOINTHALT && len == 0 &&
                 (privep = lpc214x_epfindbyaddr(priv, index)) != NULL)
          {
            privep->halted = 1;
            lpc214x_epwrite(LPC214X_EP0_IN, NULL, 0);
            priv->ep0state = LPC214X_EP0STATUSIN;
          }
        else
          {
            usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_BADSETFEATURE), 0);
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

        usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_EP0SETUPSETADDRESS), value);
        if ((ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE &&
            index  == 0 && len == 0 && value < 128)
          {
            /* Save the address.  We cannot actually change to the next address until
             * the completion of the status phase.
             */

            priv->paddr = ctrl.value[0];

            /* Note that if we send the SETADDRESS command twice, that will force the
             * address change.  Otherwise, the hardware will automatically set the
             * address at the end of the status phase.
             */

            lpc214x_usbcmd(CMD_USB_DEV_SETADDRESS, CMD_USB_SETADDRESS_DEVEN | priv->paddr);

            /* Send a NULL packet. The status phase completes when the null packet has
             * been sent successfully.
             */

            lpc214x_epwrite(LPC214X_EP0_IN, NULL, 0);
            priv->ep0state = LPC214X_EP0SETADDRESS;
          }
        else
          {
            usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_BADSETADDRESS), 0);
            priv->stalled = 1;
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
        usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_GETSETDESC), 0);
        if ((ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE)
          {
            lpc214x_dispatchrequest(priv, &ctrl);
          }
        else
          {
            usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_BADGETSETDESC), 0);
            priv->stalled = 1;
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
        usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_GETCONFIG), 0);
        if (priv->paddrset && (ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE &&
            value == 0 && index == 0 && len == 1)
          {
            lpc214x_dispatchrequest(priv, &ctrl);
          }
        else
          {
            usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_BADGETCONFIG), 0);
            priv->stalled = 1;
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
        usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_SETCONFIG), 0);
        if ((ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE &&
            index == 0 && len == 0)
          {
            lpc214x_dispatchrequest(priv, &ctrl);
          }
        else
          {
            usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_BADSETCONFIG), 0);
            priv->stalled = 1;
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
        usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_GETSETIF), 0);
        lpc214x_dispatchrequest(priv, &ctrl);
      }
      break;

    case USB_REQ_SYNCHFRAME:
      /* type:  device-to-host; recipient = endpoint
       * value: 0
       * index: endpoint;
       * len:   2; data = frame number
       */
      {
        usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_SYNCHFRAME), 0);
      }
      break;

    default:
      {
        usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_INVALIDCTRLREQ), 0);
        priv->stalled = 1;
      }
      break;
    }

  if (priv->stalled)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_EP0SETUPSTALLED), priv->ep0state);
      ep0 = &priv->eplist[LPC214X_EP0_OUT];
      lpc214x_epstall(&ep0->ep, false);
      ep0 = &priv->eplist[LPC214X_EP0_IN];
      lpc214x_epstall(&ep0->ep, false);
    }
}

/*******************************************************************************
 * Name: lpc214x_ep0dataoutinterrupt
 *
 * Description:
 *   USB Ctrl EP Data OUT Event. This is logically part of the USB interrupt
 *   handler.  Each non-isochronous OUT endpoint gives an interrupt when they
 *   receive a packet without error.
 *
 *******************************************************************************/

static inline void lpc214x_ep0dataoutinterrupt(struct lpc214x_usbdev_s *priv)
{
  struct lpc214x_ep_s *ep0;
  uint32_t pktlen;

  /* Copy new setup packet into setup buffer */

  switch (priv->ep0state)
    {
    case LPC214X_EP0SHORTWRITE:
      {
        priv->ep0state = LPC214X_EP0STATUSOUT;
        pktlen = lpc214x_epread(LPC214X_EP0_OUT, NULL, CONFIG_USBDEV_EP0_MAXSIZE);
        if (LPC214X_READOVERRUN(pktlen))
          {
            lpc214x_ep0setup(priv);
          }
      }
      break;

    case LPC214X_EP0SHORTWRSENT:
      {
        priv->ep0state = LPC214X_EP0REQUEST;
        pktlen = lpc214x_epread(LPC214X_EP0_OUT, NULL, CONFIG_USBDEV_EP0_MAXSIZE);
        if (LPC214X_READOVERRUN(pktlen))
          {
            lpc214x_ep0setup(priv);
          }
      }
      break;

    case LPC214X_EP0REQUEST:
      {
        /* Process the next request action (if any) */

        lpc214x_rdrequest(&priv->eplist[LPC214X_EP0_OUT]);
      }
      break;

    default:
      priv->stalled = 1;
      break;
    }

  if (priv->stalled)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_EP0OUTSTALLED), priv->ep0state);
      ep0 = &priv->eplist[LPC214X_EP0_OUT];
      lpc214x_epstall(&ep0->ep, false);
      ep0 = &priv->eplist[LPC214X_EP0_IN];
      lpc214x_epstall(&ep0->ep, false);
    }
  return;
}

/*******************************************************************************
 * Name: lpc214x_ep0dataininterrupt
 *
 * Description:
 *   USB Ctrl EP Data IN Event. This is logically part of the USB interrupt
 *   handler.  All non-isochronous IN endpoints give this interrupt when a
 *   packet is successfully transmitted (OR a NAK handshake is sent on the bus
 *   provided that the interrupt on NAK feature is enabled).
 *
 *******************************************************************************/

static inline void lpc214x_ep0dataininterrupt(struct lpc214x_usbdev_s *priv)
{
  struct lpc214x_ep_s *ep0;

  switch (priv->ep0state)
    {
    case LPC214X_EP0STATUSOUT:
    case LPC214X_EP0STATUSIN:
      priv->ep0state = LPC214X_EP0REQUEST;
      break;

    case LPC214X_EP0SHORTWRITE:
      priv->ep0state = LPC214X_EP0SHORTWRSENT;
      break;

    case LPC214X_EP0SETADDRESS:
      {
        /* If the address was set to a non-zero value, then thiscompletes the
         * default phase, and begins the address phase (still not fully configured)
         */

        usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_EP0INSETADDRESS), (uint16_t)priv->paddr);
        lpc214x_usbcmd(CMD_USB_DEV_CONFIG, 0);
        if (priv->paddr)
          {
            priv->paddrset = 1;
            priv->ep0state = LPC214X_EP0REQUEST;
          }
      }
      break;

    case LPC214X_EP0REQUEST:
      {
        /* Process the next request action (if any) */

        ep0 = &priv->eplist[LPC214X_EP0_IN];
        ep0->txbusy = 0;
        lpc214x_wrrequest(ep0);
      }
      break;

    default:
      priv->stalled = 1;
      break;
    }

  if (priv->stalled)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_EP0INSTALLED), priv->ep0state);
      ep0 = &priv->eplist[LPC214X_EP0_OUT];
      lpc214x_epstall(&ep0->ep, false);
      ep0 = &priv->eplist[LPC214X_EP0_IN];
      lpc214x_epstall(&ep0->ep, false);
    }
}

/*******************************************************************************
 * Name: lpc214x_usbinterrupt
 *
 * Description:
 *   USB interrupt handler
 *
 *******************************************************************************/

static int lpc214x_usbinterrupt(int irq, FAR void *context)
{
  struct lpc214x_usbdev_s *priv = &g_usbdev;
  struct lpc214x_ep_s *privep ;

  uint32_t devintstatus;  /* Sampled state of the device interrupt status register */
  uint32_t epintstatus;   /* Sampled state of the endpoint interrupt status register */
#ifdef CONFIG_LPC214X_USBDEV_DMA
  uint32_t dmaintstatus;  /* Sampled state of dma interrupt status register */
#endif
  uint32_t softprio;      /* Current priority interrupt bitset */
  uint32_t pending;       /* Pending subset of priority interrupt bitset */
  uint8_t  epphy;         /* Physical endpoint number being processed */
  int      i;

  usbtrace(TRACE_INTENTRY(LPC214X_TRACEINTID_USB), 0);

  /* Read the device interrupt status register */

  devintstatus = lpc214x_getreg(LPC214X_USBDEV_DEVINTST);

#ifdef CONFIG_LPC214X_USBDEV_DMA
  /* Check for low priority and high priority (non-DMA) interrupts */

  if ((lpc214x_getreg(LPC214X_USBDEV_INTST) & (USBDEV_INTST_REQLP|USBDEV_INTST_REQHP)) != 0)
    {
#endif
#ifdef CONFIG_LPC214X_USBDEV_EPFAST_INTERRUPT
      /* Fast EP interrupt */

      if ((devintstatus & USBDEV_DEVINT_EPFAST) != 0)
        {
          /* Clear Fast EP interrupt */

         lpc214x_putreg(USBDEV_DEVINT_EPFAST, LPC214X_USBDEV_DEVINTCLR);
         usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_EPFAST), 0);

         /* Do what? */
        }

#endif

#if CONFIG_DEBUG
      /* USB engine error interrupt */

      if ((devintstatus & USBDEV_DEVINT_EPRINT))
        {
          uint8_t errcode;

          /* Clear the error interrupt */

          lpc214x_putreg(USBDEV_DEVINT_EPRINT, LPC214X_USBDEV_DEVINTCLR);

          /* And show what error occurred */

          errcode  = (uint8_t)lpc214x_usbcmd(CMD_USB_DEV_READERRORSTATUS, 0) & 0x0f;
          usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_EPRINT), (uint16_t)errcode);
        }
#endif

#ifdef CONFIG_LPC214X_USBDEV_FRAME_INTERRUPT
      /* Frame interrupt */

      if ((devintstatus & USBDEV_DEVINT_FRAME) != 0)
        {
          /* Clear the frame interrupt */

          lpc214x_putreg(USBDEV_DEVINT_FRAME, LPC214X_USBDEV_DEVINTCLR);
          usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_FRAME), 0);

          /* Then read the start of frame value */

          priv->sof = (uint16_t)lpc214x_usbcmd(CMD_USB_DEV_READFRAMENO, 0);
        }
#endif

      /* Device Status interrupt */

      if ((devintstatus & USBDEV_DEVINT_DEVSTAT) != 0)
        {
          /* Clear Device status interrupt */

          lpc214x_putreg(USBDEV_DEVINT_DEVSTAT, LPC214X_USBDEV_DEVINTCLR);

          /* Get device status */

          g_usbdev.devstatus = (uint8_t)lpc214x_usbcmd(CMD_USB_DEV_GETSTATUS, 0);
          usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_DEVSTAT), (uint16_t)g_usbdev.devstatus);

          /* Device connection status */

          if (DEVSTATUS_CONNCHG(g_usbdev.devstatus))
            {
              usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_CONNECTCHG),
                       (uint16_t)g_usbdev.devstatus);
              if (DEVSTATUS_CONNECT(g_usbdev.devstatus))
                {
                   /* Host is connected */

                   if (!priv->attached)
                     {
                       /* We have a transition from unattached to attached */

                       usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_CONNECTED),
                                (uint16_t)g_usbdev.devstatus);
                       priv->usbdev.speed = USB_SPEED_UNKNOWN;
                       lpc214x_usbcmd(CMD_USB_DEV_CONFIG, 0);
                       priv->attached     = 1;
                    }
                 }

               /* Otherwise the host is not attached */

               else if (priv->attached)
                 {
                   usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_DISCONNECTED),
                            (uint16_t)g_usbdev.devstatus);
                   priv->usbdev.speed = USB_SPEED_UNKNOWN;
                   lpc214x_usbcmd(CMD_USB_DEV_CONFIG, 0);
                   priv->attached = 0;
                   priv->paddrset = 0;
                 }
            }

          /* Device suspend status */

          if (DEVSTATUS_SUSPCHG(g_usbdev.devstatus))
            {
              usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_SUSPENDCHG),
                       (uint16_t)g_usbdev.devstatus);

              /* Inform the Class driver of the change */

              if (priv->driver)
                {
                  if (DEVSTATUS_SUSPEND(g_usbdev.devstatus))
                    {
                      CLASS_SUSPEND(priv->driver, &priv->usbdev);
                    }
                  else
                    {
                      CLASS_RESUME(priv->driver, &priv->usbdev);
                    }
                }

              /* TODO: Perform power management operations here. */
            }

          /* Device reset */

          if (DEVSTATUS_RESET(g_usbdev.devstatus))
            {
              usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_DEVRESET),
                       (uint16_t)g_usbdev.devstatus);
              lpc214x_usbreset(priv);
            }
        }

      /* Slow EP interrupt */

      if ((devintstatus & USBDEV_DEVINT_EPSLOW) != 0)
        {
          /* Clear Slow EP interrupt */

          lpc214x_putreg(USBDEV_DEVINT_EPSLOW, LPC214X_USBDEV_DEVINTCLR);
          usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_EPSLOW), 0);

          do
            {
              /* Read the endpoint interrupt status register */

              epintstatus = lpc214x_getreg(LPC214X_USBDEV_EPINTST);

              /* Loop twice:  Process software high priority interrupts
               * on the first pass and low priority interrupts on the
               * second.
               */

              softprio = priv->softprio;
              for (i = 0; i < 2; i++, softprio = ~softprio)
                {
                  /* On the first time through the loop, pending will be
                   * the bitset of high priority pending interrupts; on the
                   * second time throught it will be the bitset of low
                   * priority interrupts.
                   */

                  pending = epintstatus & softprio;

                  /* EP0 OUT interrupt indicated by bit0 == 1 */

                  if ((pending & 1) != 0)
                    {
                      /* Clear the endpoint interrupt */

                      uint32_t result = lpc214x_epclrinterrupt(LPC214X_CTRLEP_OUT);
                      if (result & USBDEV_EPSETUPPACKET)
                        {
                          usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_EP0SETUP), (uint16_t)result);
                          lpc214x_ep0setup(priv);
                        }
                      else
                        {
                          usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_EP0OUT), priv->ep0state);
                          lpc214x_ep0dataoutinterrupt(priv);
                        }
                      break;
                    }

                  /* EP0 IN interrupt indicated by bit1 == 1 */

                  if ((pending & 2) != 0)
                    {
                      /* Clear the endpoint interrupt */

                      usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_EP0IN), priv->ep0state);
                      (void)lpc214x_epclrinterrupt(LPC214X_CTRLEP_IN);
                      lpc214x_ep0dataininterrupt(priv);
                    }
                  pending >>= 2;

                  /* All other endpoints EP 1-31 */

                  for (epphy = 2; pending; epphy++, pending >>= 1)
                    {
                      /* Is the endpoint interrupt pending? */

                      if ((pending & 1) != 0)
                        {
                           /* Yes.. clear the endpoint interrupt */

                          (void)lpc214x_epclrinterrupt(epphy);

                          /* Get the endpoint sructure corresponding to the physical
                           * endpoint number.
                           */

                          privep =  &priv->eplist[epphy];

                          /* Check for complete on IN or OUT endpoint.  Odd physical
                           * endpoint addresses are IN endpoints.
                           */

                          if ((epphy & 1) != 0)
                            {
                              /* IN: device-to-host */

                              usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_EPOUT), (uint16_t)epphy);
                              if (priv->usbdev.speed == USB_SPEED_UNKNOWN)
                                {
                                  priv->usbdev.speed = USB_SPEED_FULL;
                                  lpc214x_usbcmd(CMD_USB_DEV_CONFIG, 1);
                                }

                              /* Write host data from the current write request (if any) */

                              privep->txbusy = 0;
                              lpc214x_wrrequest(privep);
                           }
                          else
                            {
                              /* OUT: host-to-device */

                              usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_EPIN), (uint16_t)epphy);

                              /* Read host data into the current read request */

                             if (!lpc214x_rqempty(privep))
                                 {
                                  lpc214x_rdrequest(privep);
                                }
                              else
                                {
                                  ullvdbg("Pending data on OUT endpoint\n");
                                  priv->rxpending = 1;
                                }
                            }
                        }
                    }
                }
            }
          while (epintstatus);
        }
#ifdef CONFIG_LPC214X_USBDEV_DMA
    }

  /* Check for DMA interrupts */

  if ((lpc214x_getreg(LPC214X_USBDEV_INTST) & USBDEV_INTST_REQDMA) != 0)
    {
      /* First Software High priority and then low priority */

      uint32_t tmp;

      /* Collect the DMA interrupt sources */

      dmaintstatus = 0;
      tmp = lpc214x_getreg(LPC214X_USBDEV_EOTINTST);
      if (lpc214x_getreg(LPC214X_USBDEV_DMAINTEN) & 1)
        {
          dmaintstatus |= tmp;
        }
      lpc214x_putreg(tmp, LPC214X_USBDEV_EOTINTCLR);

      tmp = lpc214x_getreg(LPC214X_USBDEV_NDDRINTST);
      if (lpc214x_getreg(LPC214X_USBDEV_DMAINTEN) & 2)
        {
          dmaintstatus |= tmp;
        }
      lpc214x_putreg(tmp, LPC214X_USBDEV_NDDRINTCLR);

      tmp = lpc214x_getreg(LPC214X_USBDEV_SYSERRINTST);
      if (lpc214x_getreg(LPC214X_USBDEV_DMAINTEN) & 4)
        {
          dmaintstatus |= tmp;
        }
      lpc214x_putreg(tmp, LPC214X_USBDEV_SYSERRINTCLR);

      /* Loop twice:  Process software high priority interrupts on the
       * first pass and low priority interrupts on the second.
       */

      softprio = priv->softprio;
      for (i = 0; i < 2; i++, softprio = ~softprio)
        {
          /* On the first time through the loop, pending will be
           * the bitset of high priority pending interrupts; on the
           * second time throught it will be the bitset of low
           * priority interrupts. Note that EP0 IN and OUT are
           * omitted.
           */

          pending = (dmaintstatus & softprio) >> 2;
          for (epphy = 2; pending; epphy++, pending >>= 1)
            {
              if ((pending & 1) != 0)
                {
                  usbtrace(TRACE_INTDECODE(LPC214X_TRACEINTID_EPDMA), (uint16_t)epphy);
#warning DO WHAT?
                }
            }
        }
    }
#endif
  usbtrace(TRACE_INTEXIT(LPC214X_TRACEINTID_USB), 0);
  return OK;
}

/*******************************************************************************
 * Name: lpc214x_dmasetup
 *
 * Description:
 *   Setup for DMA Transfer
 *
 *******************************************************************************/

#ifdef CONFIG_LPC214X_USBDEV_DMA
static int lpc214x_dmasetup(struct lpc214x_usbdev_s *priv, uint8_t epphy,
                            uint32_t epmaxsize, uint32_t nbytes, uint32_t *isocpacket,
                            bool isochronous);
{
  struct lpc214x_dmadesc_s *dmadesc = priv;
  uint32_t reg;

#ifdef CONFIG_DEBUG
  if (!priv || epphy < 2)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Check if a DMA descriptor has been assigned.  If not, than that indicates
   * that we will have to do parallel I/O
   */

  if (!dmadesc)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_NODMADESC), 0);
      return -EBUSY;
    }

  /* Verify that the DMA descriptor is available */

  if ((dmadesc->status & USB_DMADESC_STATUSMASK) == USB_DMADESC_BEINGSERVICED)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_DMABUSY), 0);
      return -EBUSY; /* Shouldn't happen */
    }

  /* Init DMA Descriptor */

  dmadesc->nexdesc = 0;
  dmadesc->config  = USB_DMADESC_MODENORMAL |
                     ((epmaxsize << USB_DMADESC_PKTSIZESHIFT) & USB_DMADESC_PKTSIZEMASK) |
                     ((nbytes << USB_DMADESC_BULENSHIFT) & USB_DMADESC_BUFLENMASK);

#ifdef CONFIG_USBDEV_ISOCHRONOUS
  if (isochronous)
    {
      dmadesc->config |= USB_DMADESC_ISCOEP;
    }
#endif

  dmadesc->start = (uint32_t)&dmadesc->buffer;
  dmadesc->status = 0;

#ifdef CONFIG_USBDEV_ISOCHRONOUS
  dmadesc->size = (uint32_t)packet;
#endif

  /* Enable DMA tranfer for this endpoint */

  putreq32(1 << epphy, LPC214X_USBDEV_EPDMAEN);

  /* Check state of IN/OUT Ep buffer */

  reg = lpc214x_usbcmd(CMD_USB_EP_SELECT | epphy, 0);

  if ((LPC214X_EPPHYIN(epphy) &&  (reg & 0x60) == 0) ||
      (LPC214X_EPPHYOUT(epphy) &&  (reg & 0x60) == 0x60))
    {
      /* DMA should be "being serviced" */

      if ((dmadesc->status & USB_DMADESC_STATUSMASK) != USB_DMADESC_BEINGSERVICED))
        {
          /* Re-trigger the DMA Transfer */

          putreq21(1 << epphy, LPC214X_USBDEV_DMARCLR);
          putreq32(1 << epphy, LPC214X_USBDEV_EPDMAEN);
        }
    }
  return OK;
}
#endif /* CONFIG_LPC214X_USBDEV_DMA */

/*******************************************************************************
 * Name: lpc214x_dmarestart
 *
 * Description:
 *   Restart DMA Transfer
 *
 *******************************************************************************/

#ifdef CONFIG_LPC214X_USBDEV_DMA
static void lpc214x_dmarestart(uint8_t epphy, uint32_t descndx)
{
  uint32_t reg;

  /* Clear DMA descriptor status */

  USB_DmaDesc[descndx].status = 0;

  /* Enable DMA transfer on the endpoint */

  lpc214x_putreg(1 << epph, LPC214X_USBDEV_EPDMAEN);

  /* Check the state of IN/OUT EP buffer */

  uint32_t reg = lpc214x_usbcmd(CMD_USB_EP_SELECT | epphy, 0);
  if ((LPC214X_EPPHYIN(epphy) &&  (reg & 0x60) == 0) ||
      (LPC214X_EPPHYIN(epphy) &&  (reg & 0x60) == 0x60))
    {
      /* Re-trigger the DMA Transfer */

      putreq21(1 << epphy, LPC214X_USBDEV_DMARCLR);
      putreq32(1 << epphy, LPC214X_USBDEV_EPDMAEN);
    }
}
#endif /* CONFIG_LPC214X_USBDEV_DMA */

/*******************************************************************************
 * Name: lpc214x_dmadisable
 *
 * Description:
 *   Disable DMA transfer for the EP
 *
 *******************************************************************************/

#ifdef CONFIG_LPC214X_USBDEV_DMA
static void lpc214x_dmadisable(uint8_t epphy)
{
  EPDMADIS = 1 << epphy;
}
#endif /* CONFIG_LPC214X_USBDEV_DMA */

/*******************************************************************************
 * Endpoint operations
 *******************************************************************************/

/*******************************************************************************
 * Name: lpc214x_epconfigure
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

static int lpc214x_epconfigure(FAR struct usbdev_ep_s *ep,
                               FAR const struct usb_epdesc_s *desc,
                               bool last)
{
  FAR struct lpc214x_ep_s *privep = (FAR struct lpc214x_ep_s *)ep;
  uint32_t inten;

  usbtrace(TRACE_EPCONFIGURE, privep->epphy);
  DEBUGASSERT(desc->addr == ep->eplog);

  /* Realize the endpoint */

  lpc214x_eprealize(privep, 1, GETUINT16(desc->mxpacketsize));

  /* Enable and reset EP -- twice */

  lpc214x_usbcmd(CMD_USB_EP_SETSTATUS | privep->epphy, 0);
  lpc214x_usbcmd(CMD_USB_EP_SETSTATUS | privep->epphy, 0);

#ifdef CONFIG_LPC214X_USBDEV_DMA
  /* Enable DMA Ep interrupt (WO) */

   lpc214x_putreg(1 << privep->epphy, LPC214X_USBDEV_EPDMAEN);
#else
  /* Enable Ep interrupt (R/W) */

  inten = lpc214x_getreg(LPC214X_USBDEV_EPINTEN);
  inten |= (1 << privep->epphy);
  lpc214x_putreg(inten, LPC214X_USBDEV_EPINTEN);
#endif

  /* If all of the endpoints have been configured, then tell the USB controller
   * to enabled normal activity on all realized endpoints.
   */

  if (last)
    {
      lpc214x_usbcmd(CMD_USB_DEV_CONFIG, 1);
    }
   return OK;
}

/*******************************************************************************
 * Name: lpc214x_epdisable
 *
 * Description:
 *   The endpoint will no longer be used
 *
 *******************************************************************************/

static int lpc214x_epdisable(FAR struct usbdev_ep_s *ep)
{
  FAR struct lpc214x_ep_s *privep = (FAR struct lpc214x_ep_s *)ep;
  irqstate_t flags;
  uint32_t mask = (1 << privep->epphy);
  uint32_t reg;

#ifdef CONFIG_DEBUG
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif
  usbtrace(TRACE_EPDISABLE, privep->epphy);

  /* Cancel any ongoing activity */

  flags = irqsave();
  lpc214x_cancelrequests(privep);

  /* Disable endpoint and interrupt */

  reg  = lpc214x_getreg(LPC214X_USBDEV_REEP);
  reg &= ~mask;
  lpc214x_putreg(reg, LPC214X_USBDEV_REEP);

  lpc214x_putreg(mask, LPC214X_USBDEV_EPDMADIS);

  reg  = lpc214x_getreg(LPC214X_USBDEV_EPINTEN);
  reg &= ~mask;
  lpc214x_putreg(reg, LPC214X_USBDEV_EPINTEN);

  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Name: lpc214x_epallocreq
 *
 * Description:
 *   Allocate an I/O request
 *
 *******************************************************************************/

static FAR struct usbdev_req_s *lpc214x_epallocreq(FAR struct usbdev_ep_s *ep)
{
  FAR struct lpc214x_req_s *privreq;

#ifdef CONFIG_DEBUG
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_INVALIDPARMS), 0);
      return NULL;
    }
#endif
  usbtrace(TRACE_EPALLOCREQ, ((FAR struct lpc214x_ep_s *)ep)->epphy);

  privreq = (FAR struct lpc214x_req_s *)malloc(sizeof(struct lpc214x_req_s));
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_ALLOCFAIL), 0);
      return NULL;
    }

  memset(privreq, 0, sizeof(struct lpc214x_req_s));
  return &privreq->req;
}

/*******************************************************************************
 * Name: lpc214x_epfreereq
 *
 * Description:
 *   Free an I/O request
 *
 *******************************************************************************/

static void lpc214x_epfreereq(FAR struct usbdev_ep_s *ep, FAR struct usbdev_req_s *req)
{
  FAR struct lpc214x_req_s *privreq = (FAR struct lpc214x_req_s *)req;

#ifdef CONFIG_DEBUG
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_INVALIDPARMS), 0);
      return;
    }
#endif
  usbtrace(TRACE_EPFREEREQ, ((FAR struct lpc214x_ep_s *)ep)->epphy);

  free(privreq);
}

/*******************************************************************************
 * Name: lpc214x_epallocbuffer
 *
 * Description:
 *   Allocate an I/O buffer
 *
 *******************************************************************************/

#ifdef CONFIG_LPC214X_USBDEV_DMA
static FAR void *lpc214x_epallocbuffer(FAR struct usbdev_ep_s *ep, uint16_t nbytes)
{
#ifdef CONFIG_USBDEV_DMA

  FAR struct lpc214x_ep_s *privep = (FAR struct lpc214x_ep_s *)ep;
  int descndx;

  usbtrace(TRACE_EPALLOCBUFFER, privep->epphy);

  /* Find a free  DMA description */

#error "LOGIC INCOMPLETE"

  /* Set UDCA to the allocated DMA descriptor for this endpoint */

  USB_UDCA[privep->epphy] = &USB_DDESC[descndx];
  return &USB_DDESC[descndx]

#elif defined(CONFIG_USBDEV_DMAMEMORY)

  usbtrace(TRACE_EPALLOCBUFFER, privep->epphy);
  return usbdev_dma_alloc(bytes);

#else

  usbtrace(TRACE_EPALLOCBUFFER, privep->epphy);
  return malloc(bytes);

#endif
}
#endif

/*******************************************************************************
 * Name: lpc214x_epfreebuffer
 *
 * Description:
 *   Free an I/O buffer
 *
 *******************************************************************************/

#ifdef CONFIG_USBDEV_DMA

static void lpc214x_epfreebuffer(FAR struct usbdev_ep_s *ep, FAR void *buf)
{
#ifdef CONFIG_LPC214X_USBDEV_DMA
  FAR struct lpc214x_ep_s *privep = (FAR struct lpc214x_ep_s *)ep;

  usbtrace(TRACE_EPFREEBUFFER, privep->epphy);

  /* Indicate that there is no DMA descriptor associated with this endpoint  */

  USB_UDCA[privep->epphy] = NULL;

  /* Mark the DMA descriptor as free for re-allocation */

#  error "LOGIC INCOMPLETE"

#elif defined(CONFIG_USBDEV_DMAMEMORY)

  usbtrace(TRACE_EPFREEBUFFER, privep->epphy);
  usbdev_dma_free(buf);

#else

  usbtrace(TRACE_EPFREEBUFFER, privep->epphy);
  free(buf);

#endif
}
#endif

/*******************************************************************************
 * Name: lpc214x_epsubmit
 *
 * Description:
 *   Submit an I/O request to the endpoint
 *
 *******************************************************************************/

static int lpc214x_epsubmit(FAR struct usbdev_ep_s *ep, FAR struct usbdev_req_s *req)
{
  FAR struct lpc214x_req_s *privreq = (FAR struct lpc214x_req_s *)req;
  FAR struct lpc214x_ep_s *privep = (FAR struct lpc214x_ep_s *)ep;
  FAR struct lpc214x_usbdev_s *priv;
  irqstate_t flags;
  int ret = OK;

#ifdef CONFIG_DEBUG
  if (!req || !req->callback || !req->buf || !ep)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_INVALIDPARMS), 0);
      ullvdbg("req=%p callback=%p buf=%p ep=%p\n", req, req->callback, req->buf, ep);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPSUBMIT, privep->epphy);
  priv = privep->dev;

  if (!priv->driver || priv->usbdev.speed == USB_SPEED_UNKNOWN)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_NOTCONFIGURED), priv->usbdev.speed);
      return -ESHUTDOWN;
    }

  /* Handle the request from the class driver */

  req->result = -EINPROGRESS;
  req->xfrd   = 0;
  flags       = irqsave();

  /* If we are stalled, then drop all requests on the floor */

  if (privep->stalled)
    {
      lpc214x_abortrequest(privep, privreq, -EBUSY);
      ret = -EBUSY;
    }

  /* Handle IN (device-to-host) requests */

  else if (LPC214X_EPPHYIN(privep->epphy))
    {
      /* Add the new request to the request queue for the IN endpoint */

      lpc214x_rqenqueue(privep, privreq);
      usbtrace(TRACE_INREQQUEUED(privep->epphy), privreq->req.len);

      /* If the IN endpoint FIFO is available, then transfer the data now */

      if (privep->txbusy == 0)
        {
          ret = lpc214x_wrrequest(privep);
        }
    }

  /* Handle OUT (host-to-device) requests */

  else
    {
      /* Add the new request to the request queue for the OUT endpoint */

      privep->txnullpkt = 0;
      lpc214x_rqenqueue(privep, privreq);
      usbtrace(TRACE_OUTREQQUEUED(privep->epphy), privreq->req.len);

      /* This there a incoming data pending the availability of a request? */

      if (priv->rxpending)
        {
          ret = lpc214x_rdrequest(privep);
          priv->rxpending = 0;
        }
    }

  irqrestore(flags);
  return ret;
}

/*******************************************************************************
 * Name: lpc214x_epcancel
 *
 * Description:
 *   Cancel an I/O request previously sent to an endpoint
 *
 *******************************************************************************/

static int lpc214x_epcancel(FAR struct usbdev_ep_s *ep, FAR struct usbdev_req_s *req)
{
  FAR struct lpc214x_ep_s *privep = (FAR struct lpc214x_ep_s *)ep;
  FAR struct lpc214x_usbdev_s *priv;
  irqstate_t flags;

#ifdef CONFIG_DEBUG
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif
  usbtrace(TRACE_EPCANCEL, privep->epphy);
  priv = privep->dev;

  flags = irqsave();
  lpc214x_cancelrequests(privep);
  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Name: lpc214x_epstall
 *
 * Description:
 *   Stall or resume and endpoint
 *
 *******************************************************************************/

static int lpc214x_epstall(FAR struct usbdev_ep_s *ep, bool resume)
{
  FAR struct lpc214x_ep_s *privep = (FAR struct lpc214x_ep_s *)ep;
  irqstate_t flags;

  /* STALL or RESUME the endpoint */

  flags = irqsave();
  usbtrace(resume ? TRACE_EPRESUME : TRACE_EPSTALL, privep->epphy);
  lpc214x_usbcmd(CMD_USB_EP_SETSTATUS | privep->epphy, (resume ? 0 : USBDEV_EPSTALL));

  /* If the endpoint of was resumed, then restart any queue write requests */

  if (resume)
    {
      (void)lpc214x_wrrequest(privep);
    }
  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Device operations
 *******************************************************************************/

/*******************************************************************************
 * Name: lcp214x_allocep
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

static FAR struct usbdev_ep_s *lcp214x_allocep(FAR struct usbdev_s *dev, uint8_t eplog,
                                               bool in, uint8_t eptype)
{
  FAR struct lpc214x_usbdev_s *priv = (FAR struct lpc214x_usbdev_s *)dev;
  uint32_t epset = LPC214X_EPALLSET & ~LPC214X_EPCTRLSET;
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

      if (eplog >= LPC214X_NLOGENDPOINTS)
        {
          usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_BADEPNO), (uint16_t)eplog);
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
      epset &= LPC214X_EPINSET;
    }
  else
    {
      epset &= LPC214X_EPOUTSET;
    }

  /* Get the subset matching the requested type */

  switch (eptype)
    {
    case USB_EP_ATTR_XFER_INT: /* Interrupt endpoint */
      epset &= LPC214X_EPINTRSET;
      break;

    case USB_EP_ATTR_XFER_BULK: /* Bulk endpoint */
      epset &= LPC214X_EPBULKSET;
      break;

    case USB_EP_ATTR_XFER_ISOC: /* Isochronous endpoint */
      epset &= LPC214X_EPISOCSET;
      break;

    case USB_EP_ATTR_XFER_CONTROL: /* Control endpoint -- not a valid choice */
    default:
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_BADEPTYPE), (uint16_t)eptype);
      return NULL;
    }

  /* Is the resulting endpoint supported by the LPC214x? */

  if (epset)
    {
      /* Yes.. now see if any of the request endpoints are available */

      flags = irqsave();
      epset &= priv->epavail;
      if (epset)
        {
          /* Select the lowest bit in the set of matching, available endpoints */

          for (epndx = 2; epndx < LPC214X_NPHYSENDPOINTS; epndx++)
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

  usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_NOEP), (uint16_t)eplog);
  return NULL;
}

/*******************************************************************************
 * Name: lpc214x_freeep
 *
 * Description:
 *   Free the previously allocated endpoint
 *
 *******************************************************************************/

static void lpc214x_freeep(FAR struct usbdev_s *dev, FAR struct usbdev_ep_s *ep)
{
  FAR struct lpc214x_usbdev_s *priv = (FAR struct lpc214x_usbdev_s *)dev;
  FAR struct lpc214x_ep_s *privep = (FAR struct lpc214x_ep_s *)ep;
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
 * Name: lpc214x_getframe
 *
 * Description:
 *   Returns the current frame number
 *
 *******************************************************************************/

static int lpc214x_getframe(struct usbdev_s *dev)
{
#ifdef CONFIG_LPC214X_USBDEV_FRAME_INTERRUPT
  FAR struct lpc214x_usbdev_s *priv = (FAR struct lpc214x_usbdev_s *)dev;

  /* Return last valid value of SOF read by the interrupt handler */

  usbtrace(TRACE_DEVGETFRAME, (uint16_t)priv->sof);
  return priv->sof;
#else
  /* Return the last frame number detected by the hardware */

  usbtrace(TRACE_DEVGETFRAME, 0);
  return (int)lpc214x_usbcmd(CMD_USB_DEV_READFRAMENO, 0);
#endif
}

/*******************************************************************************
 * Name: lpc214x_wakeup
 *
 * Description:
 *   Tries to wake up the host connected to this device
 *
 *******************************************************************************/

static int lpc214x_wakeup(struct usbdev_s *dev)
{
  uint8_t arg = USBDEV_DEVSTATUS_SUSPEND;
  irqstate_t flags;

  usbtrace(TRACE_DEVWAKEUP, (uint16_t)g_usbdev.devstatus);

  flags = irqsave();
  if (DEVSTATUS_CONNECT(g_usbdev.devstatus))
    {
      arg |= USBDEV_DEVSTATUS_CONNECT;
    }

  lpc214x_usbcmd(CMD_USB_DEV_SETSTATUS, arg);
  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Name: lpc214x_selfpowered
 *
 * Description:
 *   Sets/clears the device selfpowered feature 
 *
 *******************************************************************************/

static int lpc214x_selfpowered(struct usbdev_s *dev, bool selfpowered)
{
  FAR struct lpc214x_usbdev_s *priv = (FAR struct lpc214x_usbdev_s *)dev;

  usbtrace(TRACE_DEVSELFPOWERED, (uint16_t)selfpowered);

#ifdef CONFIG_DEBUG
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_INVALIDPARMS), 0);
      return -ENODEV;
    }
#endif

  priv->selfpowered = selfpowered;
  return OK;
}

/*******************************************************************************
 * Name: lpc214x_pullup
 *
 * Description:
 *   Software-controlled connect to/disconnect from USB host
 *
 *******************************************************************************/

static int lpc214x_pullup(struct usbdev_s *dev, bool enable)
{
  usbtrace(TRACE_DEVPULLUP, (uint16_t)enable);

  /* The USBDEV_DEVSTATUS_CONNECT bit in the CMD_USB_DEV_SETSTATUS command
   * controls the LPC214x SoftConnect_N output pin that is used for SoftConnect.
   */

  lpc214x_usbcmd(CMD_USB_DEV_SETSTATUS, (enable ? USBDEV_DEVSTATUS_CONNECT : 0));
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
  struct lpc214x_usbdev_s *priv = &g_usbdev;
  uint32_t reg;
  int i;

  usbtrace(TRACE_DEVINIT, 0);

  /* Disable USB interrupts */

  lpc214x_putreg(0, LPC214X_USBDEV_INTST);

  /* Initialize the device state structure */

  memset(priv, 0, sizeof(struct lpc214x_usbdev_s));
  priv->usbdev.ops = &g_devops;
  priv->usbdev.ep0 = &priv->eplist[LPC214X_EP0_IN].ep;
  priv->epavail    = LPC214X_EPALLSET;

  /* Initialize the endpoint list */

  for (i = 0; i < LPC214X_NPHYSENDPOINTS; i++)
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
      if (LPC214X_EPPHYIN(i))
        {
          priv->eplist[i].ep.eplog = LPC214X_EPPHYIN2LOG(i);
        }
      else
        {
          priv->eplist[i].ep.eplog = LPC214X_EPPHYOUT2LOG(i);
        }

      /* The maximum packet size may depend on the type of endpoint */

      if ((LPC214X_EPCTRLSET & bit) != 0)
        {
          priv->eplist[i].ep.maxpacket = LPC214X_EP0MAXPACKET;
        }
      else if ((LPC214X_EPINTRSET & bit) != 0)
        {
          priv->eplist[i].ep.maxpacket = LPC214X_INTRMAXPACKET;
        }
      else if ((LPC214X_EPBULKSET & bit) != 0)
        {
          priv->eplist[i].ep.maxpacket = LPC214X_BULKMAXPACKET;
        }
      else /* if ((LPC214X_EPISOCSET & bit) != 0) */
        {
          priv->eplist[i].ep.maxpacket = LPC214X_ISOCMAXPACKET;
        }
    }

  /* Turn on USB power and clocking */

  reg = lpc214x_getreg(LPC214X_PCON_PCONP);
  reg |= LPC214X_PCONP_PCUSB;
  lpc214x_putreg(reg, LPC214X_PCON_PCONP);

  /* Attach USB controller interrupt handler */

  if (irq_attach(LPC214X_USB_IRQ, lpc214x_usbinterrupt) != 0)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_IRQREGISTRATION),
               (uint16_t)LPC214X_USB_IRQ);
      goto errout;
    }

  /* Enable USB inerrupts at the controller -- but do not disable
   * the ARM interrupt until the device is bound to the class
   * driver
   */

  lpc214x_putreg(USBDEV_INTST_ENUSBINTS, LPC214X_USBDEV_INTST);

  /* Disconnect device */

  lpc214x_pullup(&priv->usbdev, false);

  /* Enable EP0 for OUT (host-to-device) */

  lpc214x_usbcmd(CMD_USB_DEV_SETADDRESS, CMD_USB_SETADDRESS_DEVEN|0);
  lpc214x_usbcmd(CMD_USB_DEV_SETADDRESS, CMD_USB_SETADDRESS_DEVEN|0);

  /* Reset/Re-initialize the USB hardware */

  lpc214x_usbreset(priv);

  /* Init Device state structure */

  priv->devstatus = lpc214x_usbcmd(CMD_USB_DEV_GETSTATUS, 0);
  return;

errout:
  up_usbuninitialize();
}

/*******************************************************************************
 * Name: up_usbuninitialize
 *******************************************************************************/

void up_usbuninitialize(void)
{
  struct lpc214x_usbdev_s *priv = &g_usbdev;
  uint32_t reg;
  irqstate_t flags;

  usbtrace(TRACE_DEVUNINIT, 0);

  if (priv->driver)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_DRIVERREGISTERED), 0);
      usbdev_unregister(priv->driver);
    }

  /* Disconnect device */

  flags = irqsave();
  lpc214x_pullup(&priv->usbdev, false);
  priv->usbdev.speed = USB_SPEED_UNKNOWN;
  lpc214x_usbcmd(CMD_USB_DEV_CONFIG, 0);

  /* Disable and detach IRQs */

  up_disable_irq(LPC214X_USB_IRQ);
  irq_detach(LPC214X_USB_IRQ);

  /* Turn off USB power and clocking */

  reg = lpc214x_getreg(LPC214X_PCON_PCONP);
  reg &= ~LPC214X_PCONP_PCUSB;
  lpc214x_putreg(reg, LPC214X_PCON_PCONP);
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
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }

  if (g_usbdev.driver)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_DRIVER), 0);
      return -EBUSY;
    }
#endif

  /* First hook up the driver */

  g_usbdev.driver = driver;

  /* Then bind the class driver */

  ret = CLASS_BIND(driver, &g_usbdev.usbdev);
  if (ret)
    {
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_BINDFAILED), (uint16_t)-ret);
      g_usbdev.driver = NULL;
    }
  else
    {
      /* Enable USB controller interrupts */

      up_enable_irq(LPC214X_USB_IRQ);
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
      usbtrace(TRACE_DEVERROR(LPC214X_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Unbind the class driver */

  CLASS_UNBIND(driver, &g_usbdev.usbdev);

  /* Disable USB controller interrupts */

  up_disable_irq(LPC214X_USB_IRQ);

  /* Unhook the driver */

  g_usbdev.driver = NULL;
  return OK;
}
