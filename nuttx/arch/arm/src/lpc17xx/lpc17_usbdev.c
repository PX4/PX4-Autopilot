/*******************************************************************************
 * arch/arm/src/lpc17xx/lpc17_usbdev.c
 *
 *   Copyright (C) 2010, 2012 Gregory Nutt. All rights reserved.
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

#include "up_arch.h"
#include "up_internal.h"

#include "chip.h"
#include "chip/lpc17_usb.h"
#include "chip/lpc17_syscon.h"
#include "lpc17_gpio.h"
#include "lpc17_gpdma.h"

/*******************************************************************************
 * Definitions
 *******************************************************************************/

/* Configuration ***************************************************************/

#ifndef CONFIG_LPC17_USBDEV_EP0_MAXSIZE
#  define CONFIG_LPC17_USBDEV_EP0_MAXSIZE 64
#endif

#ifndef  CONFIG_USBDEV_MAXPOWER
#  define CONFIG_USBDEV_MAXPOWER 100  /* mA */
#endif

#define USB_SLOW_INT USBDEV_INT_EPSLOW
#define USB_DEVSTATUS_INT USBDEV_INT_DEVSTAT

#ifdef CONFIG_LPC17_USBDEV_EPFAST_INTERRUPT
#  define USB_FAST_INT USBDEV_INT_EPFAST
#else
#  define USB_FAST_INT 0
#endif

/* Enable reading SOF from interrupt handler vs. simply reading on demand.  Probably
 * a bad idea... Unless there is some issue with sampling the SOF from hardware
 * asynchronously.
 */

#ifdef CONFIG_LPC17_USBDEV_FRAME_INTERRUPT
#  define USB_FRAME_INT USBDEV_INT_FRAME
#else
#  define USB_FRAME_INT 0
#endif

#ifdef CONFIG_DEBUG
#  define USB_ERROR_INT USBDEV_INT_ERRINT
#else
#  undef  CONFIG_LPC17_USBDEV_REGDEBUG
#  define USB_ERROR_INT 0
#endif

/* CLKCTRL enable bits */

#define LPC17_CLKCTRL_ENABLES (USBDEV_CLK_DEVCLK|USBDEV_CLK_AHBCLK)

/* Dump GPIO registers */

#if defined(CONFIG_LPC17_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG_GPIO)
#  define usbdev_dumpgpio() \
   do { \
     lpc17_dumpgpio(GPIO_USB_DP, "D+ P0.29; D- P0.30"); \
     lpc17_dumpgpio(GPIO_USB_VBUS, "LED P1:18; VBUS P1:30"); \
     lpc17_dumpgpio(GPIO_USB_CONNECT, "CONNECT P2:9"); \
   } while (0);
#else
#  define usbdev_dumpgpio()
#endif

/* Number of DMA descriptors */

#ifdef CONFIG_LPC17_USBDEV_DMA
# error DMA SUPPORT NOT YET FULLY IMPLEMENTED
#  ifndef CONFIG_LPC17_USBDEV_NDMADESCRIPTORS
#    define CONFIG_LPC17_USBDEV_NDMADESCRIPTORS 8
#  elif CONFIG_LPC17_USBDEV_NDMADESCRIPTORS > 30
#    define CONFIG_LPC17_USBDEV_NDMADESCRIPTORS 30
#  endif
#endif

/* Debug ***********************************************************************/

/* Trace error codes */

#define LPC17_TRACEERR_ALLOCFAIL            0x0001
#define LPC17_TRACEERR_BADCLEARFEATURE      0x0002
#define LPC17_TRACEERR_BADDEVGETSTATUS      0x0003
#define LPC17_TRACEERR_BADEPNO              0x0004
#define LPC17_TRACEERR_BADEPGETSTATUS       0x0005
#define LPC17_TRACEERR_BADEPTYPE            0x0006
#define LPC17_TRACEERR_BADGETCONFIG         0x0007
#define LPC17_TRACEERR_BADGETSETDESC        0x0008
#define LPC17_TRACEERR_BADGETSTATUS         0x0009
#define LPC17_TRACEERR_BADSETADDRESS        0x000a
#define LPC17_TRACEERR_BADSETCONFIG         0x000b
#define LPC17_TRACEERR_BADSETFEATURE        0x000c
#define LPC17_TRACEERR_BINDFAILED           0x000d
#define LPC17_TRACEERR_DISPATCHSTALL        0x000e
#define LPC17_TRACEERR_DMABUSY              0x000f
#define LPC17_TRACEERR_DRIVER               0x0010
#define LPC17_TRACEERR_DRIVERREGISTERED     0x0011
#define LPC17_TRACEERR_EP0INSTALLED         0x0012
#define LPC17_TRACEERR_EP0OUTSTALLED        0x0013
#define LPC17_TRACEERR_EP0SETUPSTALLED      0x0014
#define LPC17_TRACEERR_EPINNULLPACKET       0x0015
#define LPC17_TRACEERR_EPOUTNULLPACKET      0x0016
#define LPC17_TRACEERR_EPREAD               0x0017
#define LPC17_TRACEERR_INVALIDCMD           0x0018
#define LPC17_TRACEERR_INVALIDCTRLREQ       0x0019
#define LPC17_TRACEERR_INVALIDPARMS         0x001a
#define LPC17_TRACEERR_IRQREGISTRATION      0x001b
#define LPC17_TRACEERR_NODMADESC            0x001c
#define LPC17_TRACEERR_NOEP                 0x001d
#define LPC17_TRACEERR_NOTCONFIGURED        0x001e
#define LPC17_TRACEERR_REQABORTED           0x001f

/* Trace interrupt codes */

#define LPC17_TRACEINTID_USB                0x0001
#define LPC17_TRACEINTID_CLEARFEATURE       0x0002
#define LPC17_TRACEINTID_CONNECTCHG         0x0003
#define LPC17_TRACEINTID_CONNECTED          0x0004
#define LPC17_TRACEINTID_DEVGETSTATUS       0x0005
#define LPC17_TRACEINTID_DEVRESET           0x0006
#define LPC17_TRACEINTID_DEVSTAT            0x0007
#define LPC17_TRACEINTID_DISCONNECTED       0x0008
#define LPC17_TRACEINTID_DISPATCH           0x0009
#define LPC17_TRACEINTID_EP0IN              0x000a
#define LPC17_TRACEINTID_EP0OUT             0x000b
#define LPC17_TRACEINTID_EP0SETUP           0x000c
#define LPC17_TRACEINTID_EPDMA              0x000d
#define LPC17_TRACEINTID_EPFAST             0x000e
#define LPC17_TRACEINTID_EPGETSTATUS        0x000f
#define LPC17_TRACEINTID_EPIN               0x0010
#define LPC17_TRACEINTID_EPINQEMPTY         0x0011
#define LPC17_TRACEINTID_EP0INSETADDRESS    0x0012
#define LPC17_TRACEINTID_EPOUT              0x0013
#define LPC17_TRACEINTID_EPOUTQEMPTY        0x0014
#define LPC17_TRACEINTID_EP0SETUPSETADDRESS 0x0015
#define LPC17_TRACEINTID_ERRINT             0x0016
#define LPC17_TRACEINTID_EPSLOW             0x0017
#define LPC17_TRACEINTID_FRAME              0x0018
#define LPC17_TRACEINTID_GETCONFIG          0x0019
#define LPC17_TRACEINTID_GETSETDESC         0x001a
#define LPC17_TRACEINTID_GETSETIF           0x001b
#define LPC17_TRACEINTID_GETSTATUS          0x001c
#define LPC17_TRACEINTID_IFGETSTATUS        0x001d
#define LPC17_TRACEINTID_SETCONFIG          0x001e
#define LPC17_TRACEINTID_SETFEATURE         0x001f
#define LPC17_TRACEINTID_SUSPENDCHG         0x0020
#define LPC17_TRACEINTID_SYNCHFRAME         0x0021

/* Hardware interface **********************************************************/

/* Macros for testing the device status response */

#define DEVSTATUS_CONNECT(s)    (((s)&CMD_STATUS_CONNECT)!=0)
#define DEVSTATUS_CONNCHG(s)    (((s)&CMD_STATUS_CONNCHG)!=0)
#define DEVSTATUS_SUSPEND(s)    (((s)&CMD_STATUS_SUSPEND)!=0)
#define DEVSTATUS_SUSPCHG(s)    (((s)&CMD_STATUS_SUSPCHG)!=0)
#define DEVSTATUS_RESET(s)      (((s)&CMD_STATUS_RESET)!=0)

/* If this bit is set in the lpc17_epread response, it means that the
 * recevied packet was overwritten by a later setup packet (ep0 only).
 */

#define LPC17_READOVERRUN_BIT (0x80000000)
#define LPC17_READOVERRUN(s)  (((s) & LPC17_READOVERRUN_BIT) != 0)

/* Endpoints ******************************************************************/

/* Number of endpoints */

#define LPC17_NLOGENDPOINTS        (16)          /* ep0-15 */
#define LPC17_NPHYSENDPOINTS       (32)          /* x2 for IN and OUT */

/* Odd physical endpoint numbers are IN; even are out */

#define LPC17_EPPHYIN(epphy)       (((epphy)&1)!=0)
#define LPC17_EPPHYOUT(epphy)      (((epphy)&1)==0)

#define LPC17_EPPHYIN2LOG(epphy)   (((uint8_t)(epphy)>>1)|USB_DIR_IN)
#define LPC17_EPPHYOUT2LOG(epphy)  (((uint8_t)(epphy)>>1)|USB_DIR_OUT)

/* Each endpoint has somewhat different characteristics */

#define LPC17_EPALLSET             (0xffffffff)  /* All endpoints */
#define LPC17_EPOUTSET             (0x55555555)  /* Even phy endpoint numbers are OUT EPs */
#define LPC17_EPINSET              (0xaaaaaaaa)  /* Odd endpoint numbers are IN EPs */
#define LPC17_EPCTRLSET            (0x00000003)  /* EP0 IN/OUT are control endpoints */
#define LPC17_EPINTRSET            (0x0c30c30c)  /* Interrupt endpoints */
#define LPC17_EPBULKSET            (0xf0c30c30)  /* Bulk endpoints */
#define LPC17_EPISOCSET            (0x030c30c0)  /* Isochronous endpoints */
#define LPC17_EPDBLBUFFER          (0xf3cf3cf0)  /* Double buffered endpoints */

#define LPC17_EP0MAXPACKET         (64)          /* EP0 max packet size (1-64) */
#define LPC17_BULKMAXPACKET        (64)          /* Bulk endpoint max packet (8/16/32/64) */
#define LPC17_INTRMAXPACKET        (64)          /* Interrupt endpoint max packet (1 to 64) */
#define LPC17_ISOCMAXPACKET        (512)         /* Acutally 1..1023 */

/* EP0 status.  EP0 transfers occur in a number of different contexts.  A
 * simple state machine is required to handle the various transfer complete
 * interrupt responses.  The following values are the various states:
 */
                                                   /*** INTERRUPT CAUSE ***/
#define LPC17_EP0REQUEST           (0)           /* Normal request handling */
#define LPC17_EP0STATUSIN          (1)           /* Status sent */
#define LPC17_EP0STATUSOUT         (2)           /* Status received */
#define LPC17_EP0SHORTWRITE        (3)           /* Short data sent with no request */
#define LPC17_EP0SHORTWRSENT       (4)           /* Short data write complete */
#define LPC17_EP0SETADDRESS        (5)           /* Set address received */
#define LPC17_EP0WRITEREQUEST      (6)           /* EP0 write request sent */

/* Request queue operations ****************************************************/

#define lpc17_rqempty(ep)          ((ep)->head == NULL)
#define lpc17_rqpeek(ep)           ((ep)->head)

/*******************************************************************************
 * Private Types
 *******************************************************************************/

/* A container for a request so that the request make be retained in a list */

struct lpc17_req_s
{
  struct usbdev_req_s    req;           /* Standard USB request */
  struct lpc17_req_s  *flink;         /* Supports a singly linked list */
};

/* This is the internal representation of an endpoint */

struct lpc17_ep_s
{
  /* Common endpoint fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_ep_s
   * to struct lpc17_ep_s.
   */

  struct usbdev_ep_s      ep;            /* Standard endpoint structure */

  /* LPC17xx-specific fields */

  struct lpc17_usbdev_s *dev;          /* Reference to private driver data */
  struct lpc17_req_s    *head;         /* Request list for this endpoint */
  struct lpc17_req_s    *tail;
  uint8_t                 epphy;         /* Physical EP address */
  uint8_t                 stalled:1;     /* 1: Endpoint is stalled */
  uint8_t                 halted:1;      /* 1: Endpoint feature halted */
  uint8_t                 txbusy:1;      /* 1: TX endpoint FIFO full */
  uint8_t                 txnullpkt:1;   /* Null packet needed at end of transfer */
};

/* This represents a DMA descriptor */

#ifdef CONFIG_LPC17_USBDEV_DMA
struct lpc17_dmadesc_s
{
  uint32_t                nextdesc;      /* Address of the next DMA descriptor in RAM */
  uint32_t                config;        /* Misc. bit encoded configuration information */
  uint32_t                start;         /* DMA start address */
  uint32_t                status;        /* Misc. bit encoded status inforamation */
#ifdef CONFIG_USBDEV_ISOCHRONOUS
  uint32_t                size;          /* Isochronous packet size address */
#endif
};
#endif

/* This structure retains the state of the USB device controller */

struct lpc17_usbdev_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_s
   * to structlpc17_usbdev_s.
   */

  struct usbdev_s         usbdev;

  /* The bound device class driver */

  struct usbdevclass_driver_s *driver;

  /* LPC17xx-specific fields */

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
#ifdef CONFIG_LPC17_USBDEV_FRAME_INTERRUPT
  uint32_t                sof;           /* Last start-of-frame */
#endif

  /* Allocated DMA descriptor */

#ifdef CONFIG_LPC17_USBDEV_DMA
  struct lpc17_dmadesc_s *dmadesc;
#endif

  /* The endpoint list */

  struct lpc17_ep_s     eplist[LPC17_NPHYSENDPOINTS];
};

/*******************************************************************************
 * Private Function Prototypes
 *******************************************************************************/

/* Register operations ********************************************************/

#ifdef CONFIG_LPC17_USBDEV_REGDEBUG
static void lpc17_printreg(uint32_t addr, uint32_t val, bool iswrite);
static void lpc17_checkreg(uint32_t addr, uint32_t val, bool iswrite);
static uint32_t lpc17_getreg(uint32_t addr);
static void lpc17_putreg(uint32_t val, uint32_t addr);
#else
# define lpc17_getreg(addr)     getreg32(addr)
# define lpc17_putreg(val,addr) putreg32(val,addr)
#endif

/* Command operations **********************************************************/

static uint32_t lpc17_usbcmd(uint16_t cmd, uint8_t data);

/* Request queue operations ****************************************************/

static FAR struct lpc17_req_s *lpc17_rqdequeue(FAR struct lpc17_ep_s *privep);
static void lpc17_rqenqueue(FAR struct lpc17_ep_s *privep,
              FAR struct lpc17_req_s *req);

/* Low level data transfers and request operations *****************************/

static void lpc17_epwrite(uint8_t epphy, const uint8_t *data, uint32_t nbytes);
static int  lpc17_epread(uint8_t epphy, uint8_t *data, uint32_t nbytes);
static inline void lpc17_abortrequest(struct lpc17_ep_s *privep,
              struct lpc17_req_s *privreq, int16_t result);
static void lpc17_reqcomplete(struct lpc17_ep_s *privep, int16_t result);
static int  lpc17_wrrequest(struct lpc17_ep_s *privep);
static int  lpc17_rdrequest(struct lpc17_ep_s *privep);
static void lpc17_cancelrequests(struct lpc17_ep_s *privep);

/* Interrupt handling **********************************************************/

static struct lpc17_ep_s *lpc17_epfindbyaddr(struct lpc17_usbdev_s *priv,
              uint16_t eplog);
static void lpc17_eprealize(struct lpc17_ep_s *privep, bool prio,
              uint32_t packetsize);
static uint8_t lpc17_epclrinterrupt(uint8_t epphy);
static inline void lpc17_ep0configure(struct lpc17_usbdev_s *priv);
#ifdef CONFIG_LPC17_USBDEV_DMA
static inline void lpc17_dmareset(uint32_t enable);
#endif
static void lpc17_usbreset(struct lpc17_usbdev_s *priv);
static void lpc17_dispatchrequest(struct lpc17_usbdev_s *priv,
              const struct usb_ctrlreq_s *ctrl);
static inline void lpc17_ep0setup(struct lpc17_usbdev_s *priv);
static inline void lpc17_ep0dataoutinterrupt(struct lpc17_usbdev_s *priv);
static inline void lpc17_ep0dataininterrupt(struct lpc17_usbdev_s *priv);
static int lpc17_usbinterrupt(int irq, FAR void *context);

#ifdef CONFIG_LPC17_USBDEV_DMA
static int  lpc17_dmasetup(struct lpc17_usbdev_s *priv, uint8_t epphy,
              uint32_t epmaxsize, uint32_t nbytes, uint32_t *isocpacket,
              bool isochronous);
static void lpc17_dmarestart(uint8_t epphy, uint32_t descndx);
static void lpc17_dmadisable(uint8_t epphy);
#endif /* CONFIG_LPC17_USBDEV_DMA */

/* Endpoint operations *********************************************************/

static int  lpc17_epconfigure(FAR struct usbdev_ep_s *ep,
              const struct usb_epdesc_s *desc, bool last);
static int  lpc17_epdisable(FAR struct usbdev_ep_s *ep);
static FAR struct usbdev_req_s *lpc17_epallocreq(FAR struct usbdev_ep_s *ep);
static void lpc17_epfreereq(FAR struct usbdev_ep_s *ep,
              FAR struct usbdev_req_s *);
#ifdef CONFIG_USBDEV_DMA
static FAR void *lpc17_epallocbuffer(FAR struct usbdev_ep_s *ep,
              uint16_t nbytes);
static void lpc17_epfreebuffer(FAR struct usbdev_ep_s *ep, void *buf);
#endif
static int  lpc17_epsubmit(FAR struct usbdev_ep_s *ep,
              struct usbdev_req_s *req);
static int  lpc17_epcancel(FAR struct usbdev_ep_s *ep,
              struct usbdev_req_s *req);
static int  lpc17_epstall(FAR struct usbdev_ep_s *ep, bool resume);

/* USB device controller operations ********************************************/

static FAR struct usbdev_ep_s *lpc17_allocep(FAR struct usbdev_s *dev,
              uint8_t epno, bool in, uint8_t eptype);
static void lpc17_freeep(FAR struct usbdev_s *dev, FAR struct usbdev_ep_s *ep);
static int  lpc17_getframe(struct usbdev_s *dev);
static int  lpc17_wakeup(struct usbdev_s *dev);
static int  lpc17_selfpowered(struct usbdev_s *dev, bool selfpowered);
static int  lpc17_pullup(struct usbdev_s *dev, bool enable);

/*******************************************************************************
 * Private Data
 *******************************************************************************/

/* Since there is only a single USB interface, all status information can be
 * be simply retained in a single global instance.
 */

static struct lpc17_usbdev_s g_usbdev;

static const struct usbdev_epops_s g_epops =
{
  .configure   = lpc17_epconfigure,
  .disable     = lpc17_epdisable,
  .allocreq    = lpc17_epallocreq,
  .freereq     = lpc17_epfreereq,
#ifdef CONFIG_USBDEV_DMA
  .allocbuffer = lpc17_epallocbuffer,
  .freebuffer  = lpc17_epfreebuffer,
#endif
  .submit      = lpc17_epsubmit,
  .cancel      = lpc17_epcancel,
  .stall       = lpc17_epstall,
};

static const struct usbdev_ops_s g_devops =
{
  .allocep     = lpc17_allocep,
  .freeep      = lpc17_freeep,
  .getframe    = lpc17_getframe,
  .wakeup      = lpc17_wakeup,
  .selfpowered = lpc17_selfpowered,
  .pullup      = lpc17_pullup,
};

/* USB Device Communication Area ***********************************************
 *
 * The CPU and DMA controller communicate through a common area of memory, called
 * the USB Device Communication Area, or UDCA. The UDCA is a 32-word array of DMA
 * Descriptor Pointers (DDPs), each of which corresponds to a physical endpoint.
 * Each DDP points to the start address of a DMA Descriptor, if one is defined for
 * the endpoint. DDPs for unrealized endpoints and endpoints disabled for DMA
 * operation are ignored and can be set to a NULL (0x0) value.
 *
 * The start address of the UDCA is stored in the USBUDCAH register. The UDCA can
 * reside at any 128-byte boundary of RAM that is accessible to both the CPU and DMA
 * controller (on other MCU's like the LPC2148, the UDCA lies in a specialized
 * 8Kb memory region).
 */

#ifdef CONFIG_LPC17_USBDEV_DMA
static uint32_t                g_udca[LPC17_NPHYSENDPOINTS] __attribute__ ((aligned (128)));
static struct lpc17_dmadesc_s  g_usbddesc[CONFIG_LPC17_USBDEV_NDMADESCRIPTORS];
#endif

/*******************************************************************************
 * Public Data
 *******************************************************************************/

/*******************************************************************************
 * Private Functions
 *******************************************************************************/

/*******************************************************************************
 * Name: lpc17_printreg
 *
 * Description:
 *   Print the contents of an LPC17xx register operation
 *
 *******************************************************************************/

#ifdef CONFIG_LPC17_USBDEV_REGDEBUG
static void lpc17_printreg(uint32_t addr, uint32_t val, bool iswrite)
{
  lldbg("%08x%s%08x\n", addr, iswrite ? "<-" : "->", val);
}
#endif

/*******************************************************************************
 * Name: lpc17_checkreg
 *
 * Description:
 *   Get the contents of an LPC17xx register
 *
 *******************************************************************************/

#ifdef CONFIG_LPC17_USBDEV_REGDEBUG
static void lpc17_checkreg(uint32_t addr, uint32_t val, bool iswrite)
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

              lpc17_printreg(prevaddr, preval, prevwrite);
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

      lpc17_printreg(addr, val, iswrite);
    }
}
#endif

/*******************************************************************************
 * Name: lpc17_getreg
 *
 * Description:
 *   Get the contents of an LPC17xx register
 *
 *******************************************************************************/

#ifdef CONFIG_LPC17_USBDEV_REGDEBUG
static uint32_t lpc17_getreg(uint32_t addr)
{
  /* Read the value from the register */

  uint32_t val = getreg32(addr);

  /* Check if we need to print this value */

  lpc17_checkreg(addr, val, false);
  return val;
}
#endif

/*******************************************************************************
 * Name: lpc17_putreg
 *
 * Description:
 *   Set the contents of an LPC17xx register to a value
 *
 *******************************************************************************/

#ifdef CONFIG_LPC17_USBDEV_REGDEBUG
static void lpc17_putreg(uint32_t val, uint32_t addr)
{
  /* Check if we need to print this value */

  lpc17_checkreg(addr, val, true);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/*******************************************************************************
 * Name: lpc17_usbcmd
 *
 * Description:
 *   Transmit commands to the USB engine
 *
 *******************************************************************************/

static uint32_t lpc17_usbcmd(uint16_t cmd, uint8_t data)
{
  irqstate_t flags;
  uint32_t cmd32;
  uint32_t data32;
  uint32_t tmp = 0;

  /* Disable interrupt and clear CDFULL and CCEMPTY interrupt status */

  flags = irqsave();
  lpc17_putreg(USBDEV_INT_CDFULL|USBDEV_INT_CCEMPTY, LPC17_USBDEV_INTCLR);

  /* Shift the command in position and mask out extra bits */

  cmd32 = ((uint32_t)cmd << CMD_USBDEV_CMDSHIFT) & CMD_USBDEV_CMDMASK;

  /* Load command + WR in command code register */

  lpc17_putreg(cmd32 | CMD_USBDEV_CMDWR, LPC17_USBDEV_CMDCODE);

  /* Wait until the command register is empty (CCEMPTY != 0, command is accepted) */

  while ((lpc17_getreg(LPC17_USBDEV_INTST) & USBDEV_INT_CCEMPTY) == 0);

  /* Clear command register empty (CCEMPTY) interrupt */

  lpc17_putreg(USBDEV_INT_CCEMPTY, LPC17_USBDEV_INTCLR);

  /* Determine next phase of the command */

  switch (cmd)
    {
      /* Write operations (1 byte of data) */

    case CMD_USBDEV_SETADDRESS:
    case CMD_USBDEV_CONFIG:
    case CMD_USBDEV_SETMODE:
    case CMD_USBDEV_SETSTATUS:
      {
        /* Send data + WR and wait for CCEMPTY */

        data32 = (uint32_t)data << CMD_USBDEV_WDATASHIFT;
        lpc17_putreg(data32 | CMD_USBDEV_DATAWR, LPC17_USBDEV_CMDCODE);
        while ((lpc17_getreg(LPC17_USBDEV_INTST) & USBDEV_INT_CCEMPTY) == 0);
      }
      break;

      /* 16 bit read operations */

    case CMD_USBDEV_READFRAMENO:
    case CMD_USBDEV_READTESTREG:
      {
        /* Send command code + RD and wait for CDFULL */

        lpc17_putreg(cmd32 | CMD_USBDEV_DATARD, LPC17_USBDEV_CMDCODE);
        while ((lpc17_getreg(LPC17_USBDEV_INTST) & USBDEV_INT_CDFULL) == 0);

        /* Clear CDFULL and read LS data */

        lpc17_putreg(USBDEV_INT_CDFULL, LPC17_USBDEV_INTCLR);
        tmp = lpc17_getreg(LPC17_USBDEV_CMDDATA);

        /* Send command code + RD and wait for CDFULL */

        lpc17_putreg(cmd32 | CMD_USBDEV_DATARD, LPC17_USBDEV_CMDCODE);
        while ((lpc17_getreg(LPC17_USBDEV_INTST) & USBDEV_INT_CDFULL) == 0);

        /* Read MS data */

        tmp |= lpc17_getreg(LPC17_USBDEV_CMDDATA) << 8;
      }
      break;

     /* 8-bit read operations */

    case CMD_USBDEV_GETSTATUS:
    case CMD_USBDEV_GETERRORCODE:
    case CMD_USBDEV_READERRORSTATUS:
    case CMD_USBDEV_EPCLRBUFFER:
      {
        /* Send command code + RD and wait for CDFULL */

        lpc17_putreg(cmd32 | CMD_USBDEV_DATARD, LPC17_USBDEV_CMDCODE);
        while ((lpc17_getreg(LPC17_USBDEV_INTST) & USBDEV_INT_CDFULL) == 0);

        /* Read data */

        tmp = lpc17_getreg(LPC17_USBDEV_CMDDATA);
      }
      break;

      /* No data transfer */

    case CMD_USBDEV_EPVALIDATEBUFFER:
      break;

    default:
      switch (cmd & 0x1e0)
        {
        case CMD_USBDEV_EPSELECT:
        case CMD_USBDEV_EPSELECTCLEAR:
          {
            /* Send command code + RD and wait for CDFULL */

            lpc17_putreg(cmd32 | CMD_USBDEV_DATARD, LPC17_USBDEV_CMDCODE);
            while ((lpc17_getreg(LPC17_USBDEV_INTST) & USBDEV_INT_CDFULL) == 0);

            /* Read data */

            tmp = lpc17_getreg(LPC17_USBDEV_CMDDATA);
          }
          break;

        case CMD_USBDEV_EPSETSTATUS:
          {
            /* Send data + RD and wait for CCEMPTY */

            data32 = (uint32_t)data << CMD_USBDEV_WDATASHIFT;
            lpc17_putreg(data32 | CMD_USBDEV_DATAWR, LPC17_USBDEV_CMDCODE);
            while ((lpc17_getreg(LPC17_USBDEV_INTST) & USBDEV_INT_CCEMPTY) == 0);
          }
          break;

        default:
          usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_INVALIDCMD), 0);
          break;
        }
      break;
    }

  /* Restore the interrupt flags */

  irqrestore(flags);
  return tmp;
}

/*******************************************************************************
 * Name: lpc17_rqdequeue
 *
 * Description:
 *   Remove a request from an endpoint request queue
 *
 *******************************************************************************/

static FAR struct lpc17_req_s *lpc17_rqdequeue(FAR struct lpc17_ep_s *privep)
{
  FAR struct lpc17_req_s *ret = privep->head;

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
 * Name: lpc17_rqenqueue
 *
 * Description:
 *   Add a request from an endpoint request queue
 *
 *******************************************************************************/

static void lpc17_rqenqueue(FAR struct lpc17_ep_s *privep,
                              FAR struct lpc17_req_s *req)
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
 * Name: lpc17_epwrite
 *
 * Description:
 *   Endpoint write (IN)
 *
 *******************************************************************************/

static void lpc17_epwrite(uint8_t epphy, const uint8_t *data, uint32_t nbytes)
{
  uint32_t value;
  bool aligned = (((uint32_t)data & 3) == 0);

  /* Set the write enable bit for this physical EP address. Bits 2-5 are
   * the logical endpoint number (0-15)
   */

  lpc17_putreg(((epphy << 1) & USBDEV_CTRL_LOGEP_MASK) | USBDEV_CTRL_WREN,
                 LPC17_USBDEV_CTRL);

  /* Set the transmit packet length (nbytes must be less than 2048) */

  lpc17_putreg(nbytes, LPC17_USBDEV_TXPLEN);

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

          lpc17_putreg(value, LPC17_USBDEV_TXDATA);
          data += 4;
        }
      else
        {
          /* Zero length packet */

          lpc17_putreg(0, LPC17_USBDEV_TXDATA);
        }
    }
  while ((lpc17_getreg(LPC17_USBDEV_CTRL) & USBDEV_CTRL_WREN) != 0);

  /* Done */

  lpc17_putreg(0, LPC17_USBDEV_CTRL);
  (void)lpc17_usbcmd(CMD_USBDEV_EPSELECT | epphy, 0);
  (void)lpc17_usbcmd(CMD_USBDEV_EPVALIDATEBUFFER, 0);
}

/*******************************************************************************
 * Name: lpc17_epread
 *
 * Description:
 *   Endpoint read (OUT)
 *
 *******************************************************************************/

static int lpc17_epread(uint8_t epphy, uint8_t *data, uint32_t nbytes)
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

  lpc17_putreg(((epphy << 1) & USBDEV_CTRL_LOGEP_MASK) | USBDEV_CTRL_RDEN,
                 LPC17_USBDEV_CTRL);

  /* Wait for packet buffer ready for reading */

  while ((lpc17_getreg(LPC17_USBDEV_RXPLEN) & USBDEV_RXPLEN_PKTRDY) == 0);

  /* Get the number of bytes of data to be read */

  pktlen = lpc17_getreg(LPC17_USBDEV_RXPLEN) & USBDEV_RXPLEN_MASK;

  /* Read data from input buffer while read data is valid (DV) */

  while ((lpc17_getreg(LPC17_USBDEV_RXPLEN) & USBDEV_RXPLEN_DV) != 0)
    {
      value = lpc17_getreg(LPC17_USBDEV_RXDATA);
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

  lpc17_putreg(0, LPC17_USBDEV_CTRL);
  (void)lpc17_usbcmd(CMD_USBDEV_EPSELECT | epphy, 0);
  result = lpc17_usbcmd(CMD_USBDEV_EPCLRBUFFER, 0);

  /* The packet overrun bit in the clear buffer response is applicable only
   * on EP0 transfers.  If set it means that the recevied packet was overwritten
   * by a later setup packet.
   */

  if (epphy == LPC17_EP0_OUT && (result & CMD_USBDEV_CLRBUFFER_PO) != 0)
    {
      /* Pass this information in bit 31 */

      pktlen |= LPC17_READOVERRUN_BIT;
    }
  return pktlen;
}

/*******************************************************************************
 * Name: lpc17_abortrequest
 *
 * Description:
 *   Discard a request
 *
 *******************************************************************************/

static inline void lpc17_abortrequest(struct lpc17_ep_s *privep,
                                        struct lpc17_req_s *privreq,
                                        int16_t result)
{
  usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_REQABORTED), (uint16_t)privep->epphy);

  /* Save the result in the request structure */

  privreq->req.result = result;

  /* Callback to the request completion handler */

  privreq->req.callback(&privep->ep, &privreq->req);
}

/*******************************************************************************
 * Name: lpc17_reqcomplete
 *
 * Description:
 *   Handle termination of the request at the head of the endpoint request queue.
 *
 *******************************************************************************/

static void lpc17_reqcomplete(struct lpc17_ep_s *privep, int16_t result)
{
  struct lpc17_req_s *privreq;
  int stalled = privep->stalled;
  irqstate_t flags;

  /* Remove the completed request at the head of the endpoint request list */

  flags = irqsave();
  privreq = lpc17_rqdequeue(privep);
  irqrestore(flags);

  if (privreq)
    {
      /* If endpoint 0, temporarily reflect the state of protocol stalled
       * in the callback.
       */

      if (privep->epphy == LPC17_EP0_IN)
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
 * Name: lpc17_wrrequest
 *
 * Description:
 *   Send from the next queued write request
 *
 *******************************************************************************/

static int lpc17_wrrequest(struct lpc17_ep_s *privep)
{
  struct lpc17_req_s *privreq;
  uint8_t *buf;
  int nbytes;
  int bytesleft;

  /* Check the request from the head of the endpoint request queue */

  privreq = lpc17_rqpeek(privep);
  if (!privreq)
    {
      usbtrace(TRACE_INTDECODE(LPC17_TRACEINTID_EPINQEMPTY), 0);
      return OK;
    }

  ullvdbg("epphy=%d req=%p: len=%d xfrd=%d nullpkt=%d\n",
          privep->epphy, privreq, privreq->req.len, privreq->req.xfrd, privep->txnullpkt);

  /* Ignore any attempt to send a zero length packet on anything but EP0IN */

  if (privreq->req.len == 0)
    {
      if (privep->epphy == LPC17_EP0_IN)
        {
          lpc17_epwrite(LPC17_EP0_IN, NULL, 0);
        }
      else
        {
          usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_EPINNULLPACKET), 0);
        }

      /* In any event, the request is complete */

      lpc17_reqcomplete(privep, OK);
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
      lpc17_epwrite(privep->epphy, buf, nbytes);

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
      lpc17_reqcomplete(privep, OK);
    }

  return OK;
}

/*******************************************************************************
 * Name: lpc17_rdrequest
 *
 * Description:
 *   Receive to the next queued read request
 *
 *******************************************************************************/

static int lpc17_rdrequest(struct lpc17_ep_s *privep)
{
  struct lpc17_req_s *privreq;
  uint8_t *buf;
  int nbytesread;

  /* Check the request from the head of the endpoint request queue */

  privreq = lpc17_rqpeek(privep);
  if (!privreq)
    {
      usbtrace(TRACE_INTDECODE(LPC17_TRACEINTID_EPOUTQEMPTY), 0);
      return OK;
    }

  ullvdbg("len=%d xfrd=%d nullpkt=%d\n",
          privreq->req.len, privreq->req.xfrd, privep->txnullpkt);

  /* Ignore any attempt to receive a zero length packet */

  if (privreq->req.len == 0)
    {
      usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_EPOUTNULLPACKET), 0);
      lpc17_reqcomplete(privep, OK);
      return OK;
    }

  usbtrace(TRACE_READ(privep->epphy), privreq->req.xfrd);

  /* Receive the next packet */

  buf        = privreq->req.buf + privreq->req.xfrd;
  nbytesread = lpc17_epread(privep->epphy, buf, privep->ep.maxpacket);
  if (nbytesread < 0)
    {
      usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_EPREAD), nbytesread);
      return ERROR;
    }

  /* If the receive buffer is full or if the last packet was not full
   * then we are finished with the transfer.
   */

  privreq->req.xfrd += nbytesread;
  if (privreq->req.xfrd >= privreq->req.len || nbytesread < privep->ep.maxpacket)
    {
      usbtrace(TRACE_COMPLETE(privep->epphy), privreq->req.xfrd);
      lpc17_reqcomplete(privep, OK);
    }

  return OK;
}

/*******************************************************************************
 * Name: lpc17_cancelrequests
 *
 * Description:
 *   Cancel all pending requests for an endpoint
 *
 *******************************************************************************/

static void lpc17_cancelrequests(struct lpc17_ep_s *privep)
{
  while (!lpc17_rqempty(privep))
    {
      usbtrace(TRACE_COMPLETE(privep->epphy),
               (lpc17_rqpeek(privep))->req.xfrd);
      lpc17_reqcomplete(privep, -ESHUTDOWN);
    }
}

/*******************************************************************************
 * Name: lpc17_epfindbyaddr
 *
 * Description:
 *   Find the physical endpoint structure corresponding to a logic endpoint
 *   address
 *
 *******************************************************************************/

static struct lpc17_ep_s *lpc17_epfindbyaddr(struct lpc17_usbdev_s *priv,
              uint16_t eplog)
{
  struct lpc17_ep_s *privep;
  int i;

  /* Endpoint zero is a special case */

  if (USB_EPNO(eplog) == 0)
    {
      return &priv->eplist[0];
    }

  /* Handle the remaining */

  for (i = 1; i < LPC17_NPHYSENDPOINTS; i++)
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
 * Name: lpc17_eprealize
 *
 * Description:
 *   Enable or disable an endpoint
 *
 *******************************************************************************/

static void lpc17_eprealize(struct lpc17_ep_s *privep, bool prio, uint32_t packetsize)
{
  struct lpc17_usbdev_s *priv = privep->dev;
  uint32_t mask;
  uint32_t regval;

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

  lpc17_putreg(USBDEV_INT_EPRLZED, LPC17_USBDEV_INTCLR);

  /* Realize the endpoint */

  regval  = lpc17_getreg(LPC17_USBDEV_REEP);
  regval |= (1 << privep->epphy);
  lpc17_putreg(regval, LPC17_USBDEV_REEP);

  /* Set endpoint maximum packet size */

  lpc17_putreg(privep->epphy, LPC17_USBDEV_EPIND);
  lpc17_putreg(packetsize, LPC17_USBDEV_MAXPSIZE);

  /* Wait for Realize complete */

  while ((lpc17_getreg(LPC17_USBDEV_INTST) & USBDEV_INT_EPRLZED) == 0);

  /* Clear realize interrupt bit */

  lpc17_putreg(USBDEV_INT_EPRLZED,LPC17_USBDEV_INTCLR);
}

/*******************************************************************************
 * Name: lpc17_epclrinterrupt
 *
 * Description:
 *   Clear the EP interrupt flag and return the current EP status
 *
 *******************************************************************************/

static uint8_t lpc17_epclrinterrupt(uint8_t epphy)
{
  /* Clear the endpoint interrupt */

  lpc17_putreg(1 << epphy, LPC17_USBDEV_EPINTCLR);

  /* Wait for data in the command data register */

  while ((lpc17_getreg(LPC17_USBDEV_INTST) & USBDEV_INT_CDFULL) == 0);

  /* Return the value of the command data register */

  return lpc17_getreg(LPC17_USBDEV_CMDDATA);
}

/*******************************************************************************
 * Name: lpc17_ep0configure
 *
 * Description:
 *   Configure endpoint 0
 *
 *******************************************************************************/

static inline void lpc17_ep0configure(struct lpc17_usbdev_s *priv)
{
  uint32_t inten;

  /* EndPoint 0 initialization */

  lpc17_eprealize(&priv->eplist[LPC17_CTRLEP_OUT], 0, CONFIG_LPC17_USBDEV_EP0_MAXSIZE);
  lpc17_eprealize(&priv->eplist[LPC17_CTRLEP_IN], 1, CONFIG_LPC17_USBDEV_EP0_MAXSIZE);

  /* Enable EP0 interrupts (not DMA) */

  inten = lpc17_getreg(LPC17_USBDEV_EPINTEN);
  inten |= 3; /* EP0 Rx and Tx */
  lpc17_putreg(inten, LPC17_USBDEV_EPINTEN);
}

/*******************************************************************************
 * Name: lpc17_dmareset
 *
 * Description: Reset USB DMA
 *
 *******************************************************************************/

#ifdef CONFIG_LPC17_USBDEV_DMA
static inline void lpc17_dmareset(uint32_t enable)
{
  int i;

  /* Disable All DMA interrupts */

  lpc17_putreg(0, LPC17_USBDEV_DMAINTEN);

  /* DMA Disable */

  lpc17_putreg(0xffffffff, LPC17_USBDEV_EPDMADIS);

  /* DMA Request clear */

  putreq32(0xffffffff, LPC17_USBDEV_DMARCLR);

  /* End of Transfer Interrupt Clear */

  putreq32(0xffffffff, LPC17_USBDEV_EOTINTCLR);

  /* New DD Request Interrupt Clear */

  putreq32(0xffffffff, LPC17_USBDEV_NDDRINTCLR);

  /* System Error Interrupt Clear */

  putreq32(0xffffffff, LPC17_USBDEV_SYSERRINTCLR);

  /* Nullify all pointers in the UDCA */

  for (i = 0; i < LPC17_NPHYSENDPOINTS; ++i)
    {
      g_udca[i] = NULL;
    }

  /* Set USB UDCA Head register */

  lpc17_putreg((uint32_t)g_udca, LPC17_USBDEV_UDCAH);

  /* Invalidate all DMA descriptors */

  for (i = 0; i < CONFIG_LPC17_USBDEV_NDMADESCRIPTORS; ++i)
    {
      memset(&g_usbddesc[i], 0, sizeof(struct lpc17_dmadesc_s));
    }

  /* Enable DMA interrupts */

  lpc17_putreg(enable, LPC17_USBDEV_DMAINTEN);
}
#endif

/*******************************************************************************
 * Name: lpc17_usbreset
 *
 * Description:
 *   Reset Usb engine
 *
 *******************************************************************************/

static void lpc17_usbreset(struct lpc17_usbdev_s *priv)
{
  /* Disable all endpoint interrupts */

  lpc17_putreg(0, LPC17_USBDEV_EPINTEN);

  /* Frame is Hp interrupt */

  lpc17_putreg(USBDEV_INT_FRAME, LPC17_USBDEV_INTPRI);

  /* Clear all pending interrupts */

  lpc17_putreg(0xffffffff, LPC17_USBDEV_EPINTCLR);
  lpc17_putreg(0xffffffff, LPC17_USBDEV_INTCLR);

  /* Periperhal address is needed */

  priv->paddrset = 0;

  /* Endpoints not yet configured */

  lpc17_usbcmd(CMD_USBDEV_CONFIG, 0);

  /* EndPoint 0 initialization */

  lpc17_ep0configure(priv);

#ifdef CONFIG_LPC17_USBDEV_DMA
  /* Enable End_of_Transfer_Interrupt and System_Error_Interrupt USB DMA
   * interrupts
   */

  lpc17_dmareset(CONFIG_LPC17_USBDEV_DMAINT_MASK);

#endif

  /* Enable Device interrupts */

  lpc17_putreg(USB_SLOW_INT|USB_DEVSTATUS_INT|USB_FAST_INT|USB_FRAME_INT|USB_ERROR_INT,
               LPC17_USBDEV_INTEN);

  /* Tell the class driver that we are disconnected. The class 
   * driver should then accept any new configurations.
   */

  if (priv->driver)
    {
      CLASS_DISCONNECT(priv->driver, &priv->usbdev);
    }
}

/*******************************************************************************
 * Name: lpc17_dispatchrequest
 *
 * Description:
 *   Provide unhandled setup actions to the class driver. This is logically part
 *   of the USB interrupt handler.
 *
 *******************************************************************************/

static void lpc17_dispatchrequest(struct lpc17_usbdev_s *priv,
                                    const struct usb_ctrlreq_s *ctrl)
{
  int ret;

  usbtrace(TRACE_INTDECODE(LPC17_TRACEINTID_DISPATCH), 0);
  if (priv && priv->driver)
    {
      /* Forward to the control request to the class driver implementation */

      ret = CLASS_SETUP(priv->driver, &priv->usbdev, ctrl, NULL, 0);
      if (ret < 0)
        {
          /* Stall on failure */

          usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_DISPATCHSTALL), 0);
          priv->stalled = 1;
        }
    }
}

/*******************************************************************************
 * Name: lpc17_ep0setup
 *
 * Description:
 *   USB Ctrl EP Setup Event. This is logically part of the USB interrupt
 *   handler.  This event occurs when a setup packet is receive on EP0 OUT.
 *
 *******************************************************************************/

static inline void lpc17_ep0setup(struct lpc17_usbdev_s *priv)
{
  struct lpc17_ep_s *ep0 = &priv->eplist[LPC17_EP0_OUT];
  struct lpc17_ep_s *privep;
  struct lpc17_req_s *privreq = lpc17_rqpeek(ep0);
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
      lpc17_usbcmd(CMD_USBDEV_CONFIG, 1);
    }

  /* Terminate any pending requests */

  while (!lpc17_rqempty(ep0))
    {
      int16_t result = OK;
      if (privreq->req.xfrd != privreq->req.len)
        {
          result = -EPROTO;
        }

      usbtrace(TRACE_COMPLETE(ep0->epphy), privreq->req.xfrd);
      lpc17_reqcomplete(ep0, result);
    }

  /* Assume NOT stalled */

  ep0->stalled  = 0;
  priv->stalled = 0;

  /* Read EP0 data */

  ret = lpc17_epread(LPC17_EP0_OUT, (uint8_t*)&ctrl, USB_SIZEOF_CTRLREQ);
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
      lpc17_dispatchrequest(priv, &ctrl);
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

        usbtrace(TRACE_INTDECODE(LPC17_TRACEINTID_GETSTATUS), 0);
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
                  usbtrace(TRACE_INTDECODE(LPC17_TRACEINTID_EPGETSTATUS), 0);
                  privep = lpc17_epfindbyaddr(priv, index);
                  if (!privep)
                    {
                      usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_BADEPGETSTATUS), 0);
                      priv->stalled = 1;
                    }
                  else
                    {
                       if ((lpc17_usbcmd(CMD_USBDEV_EPSELECT|privep->epphy, 0) & CMD_EPSELECT_ST) != 0)
                         {
                           response[0] = 1; /* Stalled */
                         }
                       else
                         {
                           response[0] = 0; /* Not stalled */
                         }
                      response[1] = 0;
                      lpc17_epwrite(LPC17_EP0_IN, response, 2);
                      priv->ep0state = LPC17_EP0SHORTWRITE;
                    }
                }
                break;

              case USB_REQ_RECIPIENT_DEVICE:
                {
                  if (index == 0)
                    {
                      usbtrace(TRACE_INTDECODE(LPC17_TRACEINTID_DEVGETSTATUS), 0);

                      /* Features:  Remote Wakeup=YES; selfpowered=? */

                      response[0] = (priv->selfpowered << USB_FEATURE_SELFPOWERED) |
                                    (1 << USB_FEATURE_REMOTEWAKEUP);
                      response[1] = 0;
                      lpc17_epwrite(LPC17_EP0_IN, response, 2);
                      priv->ep0state = LPC17_EP0SHORTWRITE;
                    }
                  else
                    {
                      usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_BADDEVGETSTATUS), 0);
                      priv->stalled = 1;
                    }
                }
                break;

              case USB_REQ_RECIPIENT_INTERFACE:
                {
                  usbtrace(TRACE_INTDECODE(LPC17_TRACEINTID_IFGETSTATUS), 0);
                  response[0] = 0;
                  response[1] = 0;
                  lpc17_epwrite(LPC17_EP0_IN, response, 2);
                  priv->ep0state = LPC17_EP0SHORTWRITE;
                }
                break;

              default:
                {
                  usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_BADGETSTATUS), 0);
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

        usbtrace(TRACE_INTDECODE(LPC17_TRACEINTID_CLEARFEATURE), 0);
        if ((ctrl.type & USB_REQ_RECIPIENT_MASK) != USB_REQ_RECIPIENT_ENDPOINT)
          {
            lpc17_dispatchrequest(priv, &ctrl);
          }
        else if (priv->paddrset != 0 && value == USB_FEATURE_ENDPOINTHALT && len == 0 &&
                 (privep = lpc17_epfindbyaddr(priv, index)) != NULL)
          {
            privep->halted = 0;
            ret = lpc17_epstall(&privep->ep, true);
            lpc17_epwrite(LPC17_EP0_IN, NULL, 0);
            priv->ep0state = LPC17_EP0STATUSIN;
          }
        else
          {
            usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_BADCLEARFEATURE), 0);
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

        usbtrace(TRACE_INTDECODE(LPC17_TRACEINTID_SETFEATURE), 0);
        if (((ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE) &&
            value == USB_FEATURE_TESTMODE)
          {
            ullvdbg("test mode: %d\n", index);
          }
        else if ((ctrl.type & USB_REQ_RECIPIENT_MASK) != USB_REQ_RECIPIENT_ENDPOINT)
          {
           lpc17_dispatchrequest(priv, &ctrl);
          }
        else if (priv->paddrset != 0 && value == USB_FEATURE_ENDPOINTHALT && len == 0 &&
                 (privep = lpc17_epfindbyaddr(priv, index)) != NULL)
          {
            privep->halted = 1;
            lpc17_epwrite(LPC17_EP0_IN, NULL, 0);
            priv->ep0state = LPC17_EP0STATUSIN;
          }
        else
          {
            usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_BADSETFEATURE), 0);
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

        usbtrace(TRACE_INTDECODE(LPC17_TRACEINTID_EP0SETUPSETADDRESS), value);
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

            lpc17_usbcmd(CMD_USBDEV_SETADDRESS, CMD_USBDEV_SETADDRESS_DEVEN | priv->paddr);

            /* Send a NULL packet. The status phase completes when the null packet has
             * been sent successfully.
             */

            lpc17_epwrite(LPC17_EP0_IN, NULL, 0);
            priv->ep0state = LPC17_EP0SETADDRESS;
          }
        else
          {
            usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_BADSETADDRESS), 0);
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
        usbtrace(TRACE_INTDECODE(LPC17_TRACEINTID_GETSETDESC), 0);
        if ((ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE)
          {
            lpc17_dispatchrequest(priv, &ctrl);
          }
        else
          {
            usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_BADGETSETDESC), 0);
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
        usbtrace(TRACE_INTDECODE(LPC17_TRACEINTID_GETCONFIG), 0);
        if (priv->paddrset && (ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE &&
            value == 0 && index == 0 && len == 1)
          {
            lpc17_dispatchrequest(priv, &ctrl);
          }
        else
          {
            usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_BADGETCONFIG), 0);
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
        usbtrace(TRACE_INTDECODE(LPC17_TRACEINTID_SETCONFIG), 0);
        if ((ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE &&
            index == 0 && len == 0)
          {
            lpc17_dispatchrequest(priv, &ctrl);
          }
        else
          {
            usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_BADSETCONFIG), 0);
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
        usbtrace(TRACE_INTDECODE(LPC17_TRACEINTID_GETSETIF), 0);
        lpc17_dispatchrequest(priv, &ctrl);
      }
      break;

    case USB_REQ_SYNCHFRAME:
      /* type:  device-to-host; recipient = endpoint
       * value: 0
       * index: endpoint;
       * len:   2; data = frame number
       */
      {
        usbtrace(TRACE_INTDECODE(LPC17_TRACEINTID_SYNCHFRAME), 0);
      }
      break;

    default:
      {
        usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_INVALIDCTRLREQ), 0);
        priv->stalled = 1;
      }
      break;
    }

  if (priv->stalled)
    {
      usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_EP0SETUPSTALLED), priv->ep0state);
      lpc17_epstall(&priv->eplist[LPC17_EP0_IN].ep, false);
      lpc17_epstall(&priv->eplist[LPC17_EP0_OUT].ep, false);
    }
}

/*******************************************************************************
 * Name: lpc17_ep0dataoutinterrupt
 *
 * Description:
 *   USB Ctrl EP Data OUT Event. This is logically part of the USB interrupt
 *   handler.  Each non-isochronous OUT endpoint gives an interrupt when they
 *   receive a packet without error.
 *
 *******************************************************************************/

static inline void lpc17_ep0dataoutinterrupt(struct lpc17_usbdev_s *priv)
{
  uint32_t pktlen;

  /* Copy new setup packet into setup buffer */

  switch (priv->ep0state)
    {
    case LPC17_EP0SHORTWRITE:
      {
        priv->ep0state = LPC17_EP0STATUSOUT;
        pktlen = lpc17_epread(LPC17_EP0_OUT, NULL, CONFIG_LPC17_USBDEV_EP0_MAXSIZE);
        if (LPC17_READOVERRUN(pktlen))
          {
            lpc17_ep0setup(priv);
          }
      }
      break;

    case LPC17_EP0SHORTWRSENT:
      {
        priv->ep0state = LPC17_EP0REQUEST;
        pktlen = lpc17_epread(LPC17_EP0_OUT, NULL, CONFIG_LPC17_USBDEV_EP0_MAXSIZE);
        if (LPC17_READOVERRUN(pktlen))
          {
            lpc17_ep0setup(priv);
          }
      }
      break;

    case LPC17_EP0REQUEST:
      {
        /* Process the next request action (if any) */

        lpc17_rdrequest(&priv->eplist[LPC17_EP0_OUT]);
      }
      break;

    default:
      priv->stalled = 1;
      break;
    }

  if (priv->stalled)
    {
      usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_EP0OUTSTALLED), priv->ep0state);
      lpc17_epstall(&priv->eplist[LPC17_EP0_IN].ep, false);
      lpc17_epstall(&priv->eplist[LPC17_EP0_OUT].ep, false);
    }
}

/*******************************************************************************
 * Name: lpc17_ep0dataininterrupt
 *
 * Description:
 *   USB Ctrl EP Data IN Event. This is logically part of the USB interrupt
 *   handler.  All non-isochronous IN endpoints give this interrupt when a
 *   packet is successfully transmitted (OR a NAK handshake is sent on the bus
 *   provided that the interrupt on NAK feature is enabled).
 *
 *******************************************************************************/

static inline void lpc17_ep0dataininterrupt(struct lpc17_usbdev_s *priv)
{
  struct lpc17_ep_s *ep0;

  switch (priv->ep0state)
    {
    case LPC17_EP0STATUSOUT:
    case LPC17_EP0STATUSIN:
      priv->ep0state = LPC17_EP0REQUEST;
      break;

    case LPC17_EP0SHORTWRITE:
      priv->ep0state = LPC17_EP0SHORTWRSENT;
      break;

    case LPC17_EP0SETADDRESS:
      {
        /* If the address was set to a non-zero value, then thiscompletes the
         * default phase, and begins the address phase (still not fully configured)
         */

        usbtrace(TRACE_INTDECODE(LPC17_TRACEINTID_EP0INSETADDRESS), (uint16_t)priv->paddr);
        lpc17_usbcmd(CMD_USBDEV_CONFIG, 0);
        if (priv->paddr)
          {
            priv->paddrset = 1;
            priv->ep0state = LPC17_EP0REQUEST;
          }
      }
      break;

    case LPC17_EP0REQUEST:
      {
        /* Process the next request action (if any) */

        ep0 = &priv->eplist[LPC17_EP0_IN];
        ep0->txbusy = 0;
        lpc17_wrrequest(ep0);
      }
      break;

    default:
      priv->stalled = 1;
      break;
    }

  if (priv->stalled)
    {
      usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_EP0INSTALLED), priv->ep0state);
      lpc17_epstall(&priv->eplist[LPC17_EP0_IN].ep, false);
      lpc17_epstall(&priv->eplist[LPC17_EP0_OUT].ep, false);
    }
}

/*******************************************************************************
 * Name: lpc17_usbinterrupt
 *
 * Description:
 *   USB interrupt handler
 *
 *******************************************************************************/

static int lpc17_usbinterrupt(int irq, FAR void *context)
{
  struct lpc17_usbdev_s *priv = &g_usbdev;
  struct lpc17_ep_s *privep ;

  uint32_t devintstatus;  /* Sampled state of the device interrupt status register */
  uint32_t epintstatus;   /* Sampled state of the endpoint interrupt status register */
#ifdef CONFIG_LPC17_USBDEV_DMA
  uint32_t usbintstatus;  /* Sampled state is SYSCON USB interrupt status */
  uint32_t dmaintstatus;  /* Sampled state of dma interrupt status register */
#endif
  uint32_t softprio;      /* Current priority interrupt bitset */
  uint32_t pending;       /* Pending subset of priority interrupt bitset */
  uint8_t  epphy;         /* Physical endpoint number being processed */
  int      i;

  /* Read the device interrupt status register */

  devintstatus = lpc17_getreg(LPC17_USBDEV_INTST);
  usbtrace(TRACE_INTENTRY(LPC17_TRACEINTID_USB), (uint16_t)devintstatus);

#ifdef CONFIG_LPC17_USBDEV_DMA
  /* Check for low priority and high priority (non-DMA) interrupts */

  usbintstatus = lpc17_getreg(LPC17_SYSCON_USBINTST);
  if ((usbintstatus & (SYSCON_USBINTST_REQLP|SYSCON_USBINTST_REQHP)) != 0)
    {
#endif
#ifdef CONFIG_LPC17_USBDEV_EPFAST_INTERRUPT
      /* Fast EP interrupt */

      if ((devintstatus & USBDEV_INT_EPFAST) != 0)
        {
          /* Clear Fast EP interrupt */

         lpc17_putreg(USBDEV_INT_EPFAST, LPC17_USBDEV_INTCLR);
         usbtrace(TRACE_INTDECODE(LPC17_TRACEINTID_EPFAST), 0);

         /* Do what? */
        }

#endif

#if CONFIG_DEBUG
      /* USB engine error interrupt */

      if ((devintstatus & USBDEV_INT_ERRINT) != 0)
        {
          uint8_t errcode;

          /* Clear the error interrupt */

          lpc17_putreg(USBDEV_INT_ERRINT, LPC17_USBDEV_INTCLR);

          /* And show what error occurred */

          errcode  = (uint8_t)lpc17_usbcmd(CMD_USBDEV_READERRORSTATUS, 0) & CMD_READERRORSTATUS_ALLERRS;
          usbtrace(TRACE_INTDECODE(LPC17_TRACEINTID_ERRINT), (uint16_t)errcode);
        }
#endif

#ifdef CONFIG_LPC17_USBDEV_FRAME_INTERRUPT
      /* Frame interrupt */

      if ((devintstatus & USBDEV_INT_FRAME) != 0)
        {
          /* Clear the frame interrupt */

          lpc17_putreg(USBDEV_INT_FRAME, LPC17_USBDEV_INTCLR);
          usbtrace(TRACE_INTDECODE(LPC17_TRACEINTID_FRAME), 0);

          /* Then read the start of frame value */

          priv->sof = (uint16_t)lpc17_usbcmd(CMD_USBDEV_READFRAMENO, 0);
        }
#endif

      /* Device Status interrupt */

      if ((devintstatus & USBDEV_INT_DEVSTAT) != 0)
        {
          /* Clear Device status interrupt */

          lpc17_putreg(USBDEV_INT_DEVSTAT, LPC17_USBDEV_INTCLR);

          /* Get device status */

          g_usbdev.devstatus = (uint8_t)lpc17_usbcmd(CMD_USBDEV_GETSTATUS, 0);
          usbtrace(TRACE_INTDECODE(LPC17_TRACEINTID_DEVSTAT), (uint16_t)g_usbdev.devstatus);

          /* Device connection status */

          if (DEVSTATUS_CONNCHG(g_usbdev.devstatus))
            {
              usbtrace(TRACE_INTDECODE(LPC17_TRACEINTID_CONNECTCHG),
                       (uint16_t)g_usbdev.devstatus);
              if (DEVSTATUS_CONNECT(g_usbdev.devstatus))
                {
                   /* Host is connected */

                   if (!priv->attached)
                     {
                       /* We have a transition from unattached to attached */

                       usbtrace(TRACE_INTDECODE(LPC17_TRACEINTID_CONNECTED),
                                (uint16_t)g_usbdev.devstatus);
                       priv->usbdev.speed = USB_SPEED_UNKNOWN;
                       lpc17_usbcmd(CMD_USBDEV_CONFIG, 0);
                       priv->attached     = 1;
                    }
                 }

               /* Otherwise the host is not attached */

               else if (priv->attached)
                 {
                   usbtrace(TRACE_INTDECODE(LPC17_TRACEINTID_DISCONNECTED),
                            (uint16_t)g_usbdev.devstatus);
                   priv->usbdev.speed = USB_SPEED_UNKNOWN;
                   lpc17_usbcmd(CMD_USBDEV_CONFIG, 0);
                   priv->attached = 0;
                   priv->paddrset = 0;
                 }
            }

          /* Device suspend status */

          if (DEVSTATUS_SUSPCHG(g_usbdev.devstatus))
            {
              usbtrace(TRACE_INTDECODE(LPC17_TRACEINTID_SUSPENDCHG),
                       (uint16_t)g_usbdev.devstatus);
            }

          /* Device reset */

          if (DEVSTATUS_RESET(g_usbdev.devstatus))
            {
              usbtrace(TRACE_INTDECODE(LPC17_TRACEINTID_DEVRESET),
                       (uint16_t)g_usbdev.devstatus);
              lpc17_usbreset(priv);
            }
        }

      /* Slow EP interrupt */

      if ((devintstatus & USBDEV_INT_EPSLOW) != 0)
        {
          /* Clear Slow EP interrupt */

          lpc17_putreg(USBDEV_INT_EPSLOW, LPC17_USBDEV_INTCLR);
          usbtrace(TRACE_INTDECODE(LPC17_TRACEINTID_EPSLOW), 0);

          do
            {
              /* Read the endpoint interrupt status register */

              epintstatus = lpc17_getreg(LPC17_USBDEV_EPINTST);

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

                      uint32_t result = lpc17_epclrinterrupt(LPC17_CTRLEP_OUT);
                      if (result & CMD_EPSELECT_STP)
                        {
                          usbtrace(TRACE_INTDECODE(LPC17_TRACEINTID_EP0SETUP), (uint16_t)result);
                          lpc17_ep0setup(priv);
                        }
                      else
                        {
                          usbtrace(TRACE_INTDECODE(LPC17_TRACEINTID_EP0OUT), priv->ep0state);
                          lpc17_ep0dataoutinterrupt(priv);
                        }
                      break;
                    }

                  /* EP0 IN interrupt indicated by bit1 == 1 */

                  if ((pending & 2) != 0)
                    {
                      /* Clear the endpoint interrupt */

                      usbtrace(TRACE_INTDECODE(LPC17_TRACEINTID_EP0IN), priv->ep0state);
                      (void)lpc17_epclrinterrupt(LPC17_CTRLEP_IN);
                      lpc17_ep0dataininterrupt(priv);
                    }
                  pending >>= 2;

                  /* All other endpoints EP 1-31 */

                  for (epphy = 2; pending; epphy++, pending >>= 1)
                    {
                      /* Is the endpoint interrupt pending? */

                      if ((pending & 1) != 0)
                        {
                           /* Yes.. clear the endpoint interrupt */

                          (void)lpc17_epclrinterrupt(epphy);

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

                              usbtrace(TRACE_INTDECODE(LPC17_TRACEINTID_EPOUT), (uint16_t)epphy);
                              if (priv->usbdev.speed == USB_SPEED_UNKNOWN)
                                {
                                  priv->usbdev.speed = USB_SPEED_FULL;
                                  lpc17_usbcmd(CMD_USBDEV_CONFIG, 1);
                                }

                              /* Write host data from the current write request (if any) */

                              privep->txbusy = 0;
                              lpc17_wrrequest(privep);
                           }
                          else
                            {
                              /* OUT: host-to-device */

                              usbtrace(TRACE_INTDECODE(LPC17_TRACEINTID_EPIN), (uint16_t)epphy);

                              /* Read host data into the current read request */

                             if (!lpc17_rqempty(privep))
                                 {
                                  lpc17_rdrequest(privep);
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
#ifdef CONFIG_LPC17_USBDEV_DMA
    }

  /* Check for DMA interrupts */

  if (usbintstatus & SYSCON_USBINTST_REQDMA) != 0)
    {
      /* First Software High priority and then low priority */

      uint32_t tmp;

      /* Collect the DMA interrupt sources */

      dmaintstatus = 0;
      tmp = lpc17_getreg(LPC17_USBDEV_EOTINTST);
      if (lpc17_getreg(LPC17_USBDEV_DMAINTEN) & 1)
        {
          dmaintstatus |= tmp;
        }
      lpc17_putreg(tmp, LPC17_USBDEV_EOTINTCLR);

      tmp = lpc17_getreg(LPC17_USBDEV_NDDRINTST);
      if (lpc17_getreg(LPC17_USBDEV_DMAINTEN) & 2)
        {
          dmaintstatus |= tmp;
        }
      lpc17_putreg(tmp, LPC17_USBDEV_NDDRINTCLR);

      tmp = lpc17_getreg(LPC17_USBDEV_SYSERRINTST);
      if (lpc17_getreg(LPC17_USBDEV_DMAINTEN) & 4)
        {
          dmaintstatus |= tmp;
        }
      lpc17_putreg(tmp, LPC17_USBDEV_SYSERRINTCLR);

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
                  usbtrace(TRACE_INTDECODE(LPC17_TRACEINTID_EPDMA), (uint16_t)epphy);
#warning DO WHAT?
                }
            }
        }
    }
#endif
  usbtrace(TRACE_INTEXIT(LPC17_TRACEINTID_USB), 0);
  return OK;
}

/*******************************************************************************
 * Name: lpc17_dmasetup
 *
 * Description:
 *   Setup for DMA Transfer
 *
 *******************************************************************************/

#ifdef CONFIG_LPC17_USBDEV_DMA
static int lpc17_dmasetup(struct lpc17_usbdev_s *priv, uint8_t epphy,
                            uint32_t epmaxsize, uint32_t nbytes, uint32_t *isocpacket,
                            bool isochronous);
{
  struct lpc17_dmadesc_s *dmadesc = priv;
  uint32_t regval;

#ifdef CONFIG_DEBUG
  if (!priv || epphy < 2)
    {
      usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Check if a DMA descriptor has been assigned.  If not, than that indicates
   * that we will have to do parallel I/O
   */

  if (!dmadesc)
    {
      usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_NODMADESC), 0);
      return -EBUSY;
    }

  /* Verify that the DMA descriptor is available */

  if ((dmadesc->status & USB_DMADESC_STATUS_MASK) == USB_DMADESC_BEINGSERVICED)
    {
      usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_DMABUSY), 0);
      return -EBUSY; /* Shouldn't happen */
    }

  /* Init DMA Descriptor */

  dmadesc->nexdesc = 0;
  dmadesc->config  = USB_DMADESC_MODENORMAL |
                     ((epmaxsize << USB_DMADESC_PKTSIZE_SHIFT) & USB_DMADESC_PKTSIZE_MASK) |
                     ((nbytes << USB_DMADESC_BUFLEN_SHIFT) & USB_DMADESC_BUFLEN_MASK);

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

  putreq32(1 << epphy, LPC17_USBDEV_EPDMAEN);

  /* Check state of IN/OUT Ep buffer */

  regval = lpc17_usbcmd(CMD_USBDEV_EPSELECT | epphy, 0);

  if ((LPC17_EPPHYIN(epphy) &&  (regval & 0x60) == 0) ||
      (LPC17_EPPHYOUT(epphy) &&  (regval & 0x60) == 0x60))
    {
      /* DMA should be "being serviced" */

      if ((dmadesc->status & USB_DMADESC_STATUS_MASK) != USB_DMADESC_BEINGSERVICED))
        {
          /* Re-trigger the DMA Transfer */

          putreq32(1 << epphy, LPC17_USBDEV_DMARCLR);
          putreq32(1 << epphy, LPC17_USBDEV_EPDMAEN);
        }
    }
  return OK;
}
#endif /* CONFIG_LPC17_USBDEV_DMA */

/*******************************************************************************
 * Name: lpc17_dmarestart
 *
 * Description:
 *   Restart DMA Transfer
 *
 *******************************************************************************/

#ifdef CONFIG_LPC17_USBDEV_DMA
static void lpc17_dmarestart(uint8_t epphy, uint32_t descndx)
{
  uint32_t regval;

  /* Clear DMA descriptor status */

  USB_DmaDesc[descndx].status = 0;

  /* Enable DMA transfer on the endpoint */

  lpc17_putreg(1 << epph, LPC17_USBDEV_EPDMAEN);

  /* Check the state of IN/OUT EP buffer */

  uint32_t regval = lpc17_usbcmd(CMD_USBDEV_EPSELECT | epphy, 0);
  if ((LPC17_EPPHYIN(epphy) &&  (regval & 0x60) == 0) ||
      (LPC17_EPPHYIN(epphy) &&  (regval & 0x60) == 0x60))
    {
      /* Re-trigger the DMA Transfer */

      putreq32(1 << epphy, LPC17_USBDEV_DMARCLR);
      putreq32(1 << epphy, LPC17_USBDEV_EPDMAEN);
    }
}
#endif /* CONFIG_LPC17_USBDEV_DMA */

/*******************************************************************************
 * Name: lpc17_dmadisable
 *
 * Description:
 *   Disable DMA transfer for the EP
 *
 *******************************************************************************/

#ifdef CONFIG_LPC17_USBDEV_DMA
static void lpc17_dmadisable(uint8_t epphy)
{
  EPDMADIS = 1 << epphy;
}
#endif /* CONFIG_LPC17_USBDEV_DMA */

/*******************************************************************************
 * Endpoint operations
 *******************************************************************************/

/*******************************************************************************
 * Name: lpc17_epconfigure
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

static int lpc17_epconfigure(FAR struct usbdev_ep_s *ep,
                               FAR const struct usb_epdesc_s *desc,
                               bool last)
{
  FAR struct lpc17_ep_s *privep = (FAR struct lpc17_ep_s *)ep;
  uint32_t inten;

  usbtrace(TRACE_EPCONFIGURE, privep->epphy);
  DEBUGASSERT(desc->addr == ep->eplog);

  /* Realize the endpoint */

  lpc17_eprealize(privep, 1, GETUINT16(desc->mxpacketsize));

  /* Enable and reset EP -- twice */

  lpc17_usbcmd(CMD_USBDEV_EPSETSTATUS | privep->epphy, 0);
  lpc17_usbcmd(CMD_USBDEV_EPSETSTATUS | privep->epphy, 0);

#ifdef CONFIG_LPC17_USBDEV_DMA
  /* Enable DMA Ep interrupt (WO) */

   lpc17_putreg(1 << privep->epphy, LPC17_USBDEV_EPDMAEN);
#else
  /* Enable Ep interrupt (R/W) */

  inten = lpc17_getreg(LPC17_USBDEV_EPINTEN);
  inten |= (1 << privep->epphy);
  lpc17_putreg(inten, LPC17_USBDEV_EPINTEN);
#endif

  /* If all of the endpoints have been configured, then tell the USB controller
   * to enable normal activity on all realized endpoints.
   */

  if (last)
    {
      lpc17_usbcmd(CMD_USBDEV_CONFIG, 1);
    }
   return OK;
}

/*******************************************************************************
 * Name: lpc17_epdisable
 *
 * Description:
 *   The endpoint will no longer be used
 *
 *******************************************************************************/

static int lpc17_epdisable(FAR struct usbdev_ep_s *ep)
{
  FAR struct lpc17_ep_s *privep = (FAR struct lpc17_ep_s *)ep;
  irqstate_t flags;
  uint32_t mask = (1 << privep->epphy);
  uint32_t regval;

#ifdef CONFIG_DEBUG
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif
  usbtrace(TRACE_EPDISABLE, privep->epphy);

  /* Cancel any ongoing activity */

  flags = irqsave();
  lpc17_cancelrequests(privep);

  /* Disable endpoint and interrupt */

  regval  = lpc17_getreg(LPC17_USBDEV_REEP);
  regval &= ~mask;
  lpc17_putreg(regval, LPC17_USBDEV_REEP);

  lpc17_putreg(mask, LPC17_USBDEV_EPDMADIS);

  regval  = lpc17_getreg(LPC17_USBDEV_EPINTEN);
  regval &= ~mask;
  lpc17_putreg(regval, LPC17_USBDEV_EPINTEN);

  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Name: lpc17_epallocreq
 *
 * Description:
 *   Allocate an I/O request
 *
 *******************************************************************************/

static FAR struct usbdev_req_s *lpc17_epallocreq(FAR struct usbdev_ep_s *ep)
{
  FAR struct lpc17_req_s *privreq;

#ifdef CONFIG_DEBUG
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_INVALIDPARMS), 0);
      return NULL;
    }
#endif
  usbtrace(TRACE_EPALLOCREQ, ((FAR struct lpc17_ep_s *)ep)->epphy);

  privreq = (FAR struct lpc17_req_s *)malloc(sizeof(struct lpc17_req_s));
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_ALLOCFAIL), 0);
      return NULL;
    }

  memset(privreq, 0, sizeof(struct lpc17_req_s));
  return &privreq->req;
}

/*******************************************************************************
 * Name: lpc17_epfreereq
 *
 * Description:
 *   Free an I/O request
 *
 *******************************************************************************/

static void lpc17_epfreereq(FAR struct usbdev_ep_s *ep, FAR struct usbdev_req_s *req)
{
  FAR struct lpc17_req_s *privreq = (FAR struct lpc17_req_s *)req;

#ifdef CONFIG_DEBUG
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_INVALIDPARMS), 0);
      return;
    }
#endif
  usbtrace(TRACE_EPFREEREQ, ((FAR struct lpc17_ep_s *)ep)->epphy);

  free(privreq);
}

/*******************************************************************************
 * Name: lpc17_epallocbuffer
 *
 * Description:
 *   Allocate an I/O buffer
 *
 *******************************************************************************/

#ifdef CONFIG_USBDEV_DMA
static FAR void *lpc17_epallocbuffer(FAR struct usbdev_ep_s *ep, uint16_t nbytes)
{
#if defined(CONFIG_LPC17_USBDEV_DMA)

  FAR struct lpc17_ep_s *privep = (FAR struct lpc17_ep_s *)ep;
  int descndx;

  usbtrace(TRACE_EPALLOCBUFFER, privep->epphy);

  /* Find a free  DMA description */

#error "LOGIC INCOMPLETE"

  /* Set UDCA to the allocated DMA descriptor for this endpoint */

  g_udca[privep->epphy] = &g_usbddesc[descndx];
  return &g_usbddesc[descndx]

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
 * Name: lpc17_epfreebuffer
 *
 * Description:
 *   Free an I/O buffer
 *
 *******************************************************************************/

#ifdef CONFIG_USBDEV_DMA
static void lpc17_epfreebuffer(FAR struct usbdev_ep_s *ep, FAR void *buf)
{
#if defined(CONFIG_LPC17_USBDEV_DMA)

  FAR struct lpc17_ep_s *privep = (FAR struct lpc17_ep_s *)ep;

  usbtrace(TRACE_EPFREEBUFFER, privep->epphy);

  /* Indicate that there is no DMA descriptor associated with this endpoint  */

  g_udca[privep->epphy] = NULL;

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
 * Name: lpc17_epsubmit
 *
 * Description:
 *   Submit an I/O request to the endpoint
 *
 *******************************************************************************/

static int lpc17_epsubmit(FAR struct usbdev_ep_s *ep, FAR struct usbdev_req_s *req)
{
  FAR struct lpc17_req_s *privreq = (FAR struct lpc17_req_s *)req;
  FAR struct lpc17_ep_s *privep = (FAR struct lpc17_ep_s *)ep;
  FAR struct lpc17_usbdev_s *priv;
  irqstate_t flags;
  int ret = OK;

#ifdef CONFIG_DEBUG
  if (!req || !req->callback || !req->buf || !ep)
    {
      usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_INVALIDPARMS), 0);
      ullvdbg("req=%p callback=%p buf=%p ep=%p\n", req, req->callback, req->buf, ep);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPSUBMIT, privep->epphy);
  priv = privep->dev;

  if (!priv->driver || priv->usbdev.speed == USB_SPEED_UNKNOWN)
    {
      usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_NOTCONFIGURED), priv->usbdev.speed);
      return -ESHUTDOWN;
    }

  /* Handle the request from the class driver */

  req->result = -EINPROGRESS;
  req->xfrd   = 0;
  flags       = irqsave();

  /* If we are stalled, then drop all requests on the floor */

  if (privep->stalled)
    {
      lpc17_abortrequest(privep, privreq, -EBUSY);
      ret = -EBUSY;
    }

  /* Handle IN (device-to-host) requests */

  else if (LPC17_EPPHYIN(privep->epphy))
    {
      /* Add the new request to the request queue for the IN endpoint */

      lpc17_rqenqueue(privep, privreq);
      usbtrace(TRACE_INREQQUEUED(privep->epphy), privreq->req.len);

      /* If the IN endpoint FIFO is available, then transfer the data now */

      if (privep->txbusy == 0)
        {
          ret = lpc17_wrrequest(privep);
        }
    }

  /* Handle OUT (host-to-device) requests */

  else
    {
      /* Add the new request to the request queue for the OUT endpoint */

      privep->txnullpkt = 0;
      lpc17_rqenqueue(privep, privreq);
      usbtrace(TRACE_OUTREQQUEUED(privep->epphy), privreq->req.len);

      /* This there a incoming data pending the availability of a request? */

      if (priv->rxpending)
        {
          ret = lpc17_rdrequest(privep);
          priv->rxpending = 0;
        }
    }

  irqrestore(flags);
  return ret;
}

/*******************************************************************************
 * Name: lpc17_epcancel
 *
 * Description:
 *   Cancel an I/O request previously sent to an endpoint
 *
 *******************************************************************************/

static int lpc17_epcancel(FAR struct usbdev_ep_s *ep, FAR struct usbdev_req_s *req)
{
  FAR struct lpc17_ep_s *privep = (FAR struct lpc17_ep_s *)ep;
  FAR struct lpc17_usbdev_s *priv;
  irqstate_t flags;

#ifdef CONFIG_DEBUG
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif
  usbtrace(TRACE_EPCANCEL, privep->epphy);
  priv = privep->dev;

  flags = irqsave();
  lpc17_cancelrequests(privep);
  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Name: lpc17_epstall
 *
 * Description:
 *   Stall or resume and endpoint
 *
 *******************************************************************************/

static int lpc17_epstall(FAR struct usbdev_ep_s *ep, bool resume)
{
  FAR struct lpc17_ep_s *privep = (FAR struct lpc17_ep_s *)ep;
  irqstate_t flags;

  /* STALL or RESUME the endpoint */

  flags = irqsave();
  usbtrace(resume ? TRACE_EPRESUME : TRACE_EPSTALL, privep->epphy);
  lpc17_usbcmd(CMD_USBDEV_EPSETSTATUS | privep->epphy, (resume ? 0 : CMD_SETSTAUS_ST));

  /* If the endpoint of was resumed, then restart any queue write requests */

  if (resume)
    {
      (void)lpc17_wrrequest(privep);
    }
  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Device operations
 *******************************************************************************/

/*******************************************************************************
 * Name: lpc17_allocep
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

static FAR struct usbdev_ep_s *lpc17_allocep(FAR struct usbdev_s *dev, uint8_t eplog,
                                               bool in, uint8_t eptype)
{
  FAR struct lpc17_usbdev_s *priv = (FAR struct lpc17_usbdev_s *)dev;
  uint32_t epset = LPC17_EPALLSET & ~LPC17_EPCTRLSET;
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

      if (eplog >= LPC17_NLOGENDPOINTS)
        {
          usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_BADEPNO), (uint16_t)eplog);
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
      epset &= LPC17_EPINSET;
    }
  else
    {
      epset &= LPC17_EPOUTSET;
    }

  /* Get the subset matching the requested type */

  switch (eptype)
    {
    case USB_EP_ATTR_XFER_INT: /* Interrupt endpoint */
      epset &= LPC17_EPINTRSET;
      break;

    case USB_EP_ATTR_XFER_BULK: /* Bulk endpoint */
      epset &= LPC17_EPBULKSET;
      break;

    case USB_EP_ATTR_XFER_ISOC: /* Isochronous endpoint */
      epset &= LPC17_EPISOCSET;
      break;

    case USB_EP_ATTR_XFER_CONTROL: /* Control endpoint -- not a valid choice */
    default:
      usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_BADEPTYPE), (uint16_t)eptype);
      return NULL;
    }

  /* Is the resulting endpoint supported by the LPC17xx? */

  if (epset)
    {
      /* Yes.. now see if any of the request endpoints are available */

      flags = irqsave();
      epset &= priv->epavail;
      if (epset)
        {
          /* Select the lowest bit in the set of matching, available endpoints */

          for (epndx = 2; epndx < LPC17_NPHYSENDPOINTS; epndx++)
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

  usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_NOEP), (uint16_t)eplog);
  return NULL;
}

/*******************************************************************************
 * Name: lpc17_freeep
 *
 * Description:
 *   Free the previously allocated endpoint
 *
 *******************************************************************************/

static void lpc17_freeep(FAR struct usbdev_s *dev, FAR struct usbdev_ep_s *ep)
{
  FAR struct lpc17_usbdev_s *priv = (FAR struct lpc17_usbdev_s *)dev;
  FAR struct lpc17_ep_s *privep = (FAR struct lpc17_ep_s *)ep;
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
 * Name: lpc17_getframe
 *
 * Description:
 *   Returns the current frame number
 *
 *******************************************************************************/

static int lpc17_getframe(struct usbdev_s *dev)
{
#ifdef CONFIG_LPC17_USBDEV_FRAME_INTERRUPT
  FAR struct lpc17_usbdev_s *priv = (FAR struct lpc17_usbdev_s *)dev;

  /* Return last valid value of SOF read by the interrupt handler */

  usbtrace(TRACE_DEVGETFRAME, (uint16_t)priv->sof);
  return priv->sof;
#else
  /* Return the last frame number detected by the hardware */

  usbtrace(TRACE_DEVGETFRAME, 0);
  return (int)lpc17_usbcmd(CMD_USBDEV_READFRAMENO, 0);
#endif
}

/*******************************************************************************
 * Name: lpc17_wakeup
 *
 * Description:
 *   Tries to wake up the host connected to this device
 *
 *******************************************************************************/

static int lpc17_wakeup(struct usbdev_s *dev)
{
  uint8_t arg = CMD_STATUS_SUSPEND;
  irqstate_t flags;

  usbtrace(TRACE_DEVWAKEUP, (uint16_t)g_usbdev.devstatus);

  flags = irqsave();
  if (DEVSTATUS_CONNECT(g_usbdev.devstatus))
    {
      arg |= CMD_STATUS_CONNECT;
    }

  lpc17_usbcmd(CMD_USBDEV_SETSTATUS, arg);
  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Name: lpc17_selfpowered
 *
 * Description:
 *   Sets/clears the device selfpowered feature 
 *
 *******************************************************************************/

static int lpc17_selfpowered(struct usbdev_s *dev, bool selfpowered)
{
  FAR struct lpc17_usbdev_s *priv = (FAR struct lpc17_usbdev_s *)dev;

  usbtrace(TRACE_DEVSELFPOWERED, (uint16_t)selfpowered);

#ifdef CONFIG_DEBUG
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_INVALIDPARMS), 0);
      return -ENODEV;
    }
#endif

  priv->selfpowered = selfpowered;
  return OK;
}

/*******************************************************************************
 * Name: lpc17_pullup
 *
 * Description:
 *   Software-controlled connect to/disconnect from USB host
 *
 *******************************************************************************/

static int lpc17_pullup(struct usbdev_s *dev, bool enable)
{
  usbtrace(TRACE_DEVPULLUP, (uint16_t)enable);

  /* The CMD_STATUS_CONNECT bit in the CMD_USBDEV_SETSTATUS command
   * controls the LPC17xx SoftConnect_N output pin that is used for SoftConnect.
   */

  lpc17_usbcmd(CMD_USBDEV_SETSTATUS, (enable ? CMD_STATUS_CONNECT : 0));
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
 *   This function is called very early in the initialization sequence in order
 *   to initialize the USB device functionality.
 *
 *******************************************************************************/

void up_usbinitialize(void)
{
  struct lpc17_usbdev_s *priv = &g_usbdev;
  uint32_t regval;
  irqstate_t flags;
  int i;

  usbtrace(TRACE_DEVINIT, 0);

  /* Step 1: Enable power by setting PCUSB in the PCONP register */

  flags   = irqsave();
  regval  = lpc17_getreg(LPC17_SYSCON_PCONP);
  regval |= SYSCON_PCONP_PCUSB;
  lpc17_putreg(regval, LPC17_SYSCON_PCONP);

  /* Step 2: Enable clocking on USB (USB PLL clocking was initialized in
   * in very low-level clock setup logic (see lpc17_clockconfig.c)).  We
   * do still need to set up USBCLKCTRL to enable device and AHB clocking.
   */

  lpc17_putreg(LPC17_CLKCTRL_ENABLES, LPC17_USBDEV_CLKCTRL);

  /* Then wait for the clocks to be reported as "ON" */

  do
    {
      regval = lpc17_getreg(LPC17_USBDEV_CLKST);
    }
  while ((regval & LPC17_CLKCTRL_ENABLES) != LPC17_CLKCTRL_ENABLES);

  /* Step 3: Configure I/O pins */

  usbdev_dumpgpio();
#ifndef CONFIG_LPC17_USBDEV_NOVBUS
  lpc17_configgpio(GPIO_USB_VBUS);    /* VBUS status input */
#endif
  lpc17_configgpio(GPIO_USB_CONNECT); /* SoftConnect control signal */
#ifndef CONFIG_LPC17_USBDEV_NOLED
  lpc17_configgpio(GPIO_USB_UPLED);   /* GoodLink LED control signal */
#endif
  lpc17_configgpio(GPIO_USB_DP);      /* Positive differential data */
  lpc17_configgpio(GPIO_USB_DM);      /* Negative differential data */
  usbdev_dumpgpio();

  /* Disable USB interrupts */

  regval = lpc17_getreg(LPC17_SYSCON_USBINTST);
  regval &= ~SYSCON_USBINTST_ENINTS;
  lpc17_putreg(regval, LPC17_SYSCON_USBINTST);
  irqrestore(flags);

  /* Initialize the device state structure */

  memset(priv, 0, sizeof(struct lpc17_usbdev_s));
  priv->usbdev.ops = &g_devops;
  priv->usbdev.ep0 = &priv->eplist[LPC17_EP0_IN].ep;
  priv->epavail    = LPC17_EPALLSET;

  /* Initialize the endpoint list */

  for (i = 0; i < LPC17_NPHYSENDPOINTS; i++)
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
      if (LPC17_EPPHYIN(i))
        {
          priv->eplist[i].ep.eplog = LPC17_EPPHYIN2LOG(i);
        }
      else
        {
          priv->eplist[i].ep.eplog = LPC17_EPPHYOUT2LOG(i);
        }

      /* The maximum packet size may depend on the type of endpoint */

      if ((LPC17_EPCTRLSET & bit) != 0)
        {
          priv->eplist[i].ep.maxpacket = LPC17_EP0MAXPACKET;
        }
      else if ((LPC17_EPINTRSET & bit) != 0)
        {
          priv->eplist[i].ep.maxpacket = LPC17_INTRMAXPACKET;
        }
      else if ((LPC17_EPBULKSET & bit) != 0)
        {
          priv->eplist[i].ep.maxpacket = LPC17_BULKMAXPACKET;
        }
      else /* if ((LPC17_EPISOCSET & bit) != 0) */
        {
          priv->eplist[i].ep.maxpacket = LPC17_ISOCMAXPACKET;
        }
    }

  /* Make sure all USB interrupts are disabled and cleared */

  lpc17_putreg(0, LPC17_USBDEV_INTEN);
  lpc17_putreg(0xffffffff, LPC17_USBDEV_INTCLR);
  lpc17_putreg(0, LPC17_USBDEV_INTPRI);

  lpc17_putreg(0, LPC17_USBDEV_EPINTEN);
  lpc17_putreg(0xffffffff, LPC17_USBDEV_EPINTCLR);
  lpc17_putreg(0, LPC17_USBDEV_EPINTPRI);

  /* Interrupt only on ACKs */

  lpc17_usbcmd(CMD_USBDEV_SETMODE, 0);

  /* Attach USB controller interrupt handler */

  if (irq_attach(LPC17_IRQ_USB, lpc17_usbinterrupt) != 0)
    {
      usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_IRQREGISTRATION),
               (uint16_t)LPC17_IRQ_USB);
      goto errout;
    }

  /* Enable USB interrupts at the controller -- but do not enable
   * the ARM interrupt until the device is bound to the class
   * driver
   */

  flags = irqsave();
  regval = lpc17_getreg(LPC17_SYSCON_USBINTST);
  regval |= SYSCON_USBINTST_ENINTS;
  lpc17_putreg(regval, LPC17_SYSCON_USBINTST);
  irqrestore(flags);

  /* Disconnect device */

  lpc17_pullup(&priv->usbdev, false);

  /* Enable EP0 for OUT (host-to-device) */

  lpc17_usbcmd(CMD_USBDEV_SETADDRESS, CMD_USBDEV_SETADDRESS_DEVEN|0);
  lpc17_usbcmd(CMD_USBDEV_SETADDRESS, CMD_USBDEV_SETADDRESS_DEVEN|0);

  /* Reset/Re-initialize the USB hardware */

  lpc17_usbreset(priv);

  /* Init Device state structure */

  priv->devstatus = lpc17_usbcmd(CMD_USBDEV_GETSTATUS, 0);
  return;

errout:
  up_usbuninitialize();
}

/*******************************************************************************
 * Name: up_usbuninitialize
 *******************************************************************************/

void up_usbuninitialize(void)
{
  struct lpc17_usbdev_s *priv = &g_usbdev;
  uint32_t regval;
  irqstate_t flags;

  usbtrace(TRACE_DEVUNINIT, 0);

  if (priv->driver)
    {
      usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_DRIVERREGISTERED), 0);
      usbdev_unregister(priv->driver);
    }

  /* Disconnect device */

  flags = irqsave();
  lpc17_pullup(&priv->usbdev, false);
  priv->usbdev.speed = USB_SPEED_UNKNOWN;
  lpc17_usbcmd(CMD_USBDEV_CONFIG, 0);

  /* Disable and detach IRQs */

  up_disable_irq(LPC17_IRQ_USB);
  irq_detach(LPC17_IRQ_USB);

  /* Turn off USB power and clocking */

  regval = lpc17_getreg(LPC17_SYSCON_PCONP);
  regval &= ~SYSCON_PCONP_PCUSB;
  lpc17_putreg(regval, LPC17_SYSCON_PCONP);
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
      usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }

  if (g_usbdev.driver)
    {
      usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_DRIVER), 0);
      return -EBUSY;
    }
#endif

  /* First hook up the driver */

  g_usbdev.driver = driver;

  /* Then bind the class driver */

  ret = CLASS_BIND(driver, &g_usbdev.usbdev);
  if (ret)
    {
      usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_BINDFAILED), (uint16_t)-ret);
      g_usbdev.driver = NULL;
    }
  else
    {
      /* Enable USB controller interrupts */

      up_enable_irq(LPC17_IRQ_USB);
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
      usbtrace(TRACE_DEVERROR(LPC17_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Unbind the class driver */

  CLASS_UNBIND(driver, &g_usbdev.usbdev);

  /* Disable USB controller interrupts */

  up_disable_irq(LPC17_IRQ_USB);

  /* Unhook the driver */

  g_usbdev.driver = NULL;
  return OK;
}
