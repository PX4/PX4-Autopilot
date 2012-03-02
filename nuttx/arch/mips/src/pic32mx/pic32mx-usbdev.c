/****************************************************************************
 * arch/mips/src/pic32mx/pic32mx_usbdev.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   This file derives from the STM32 USB device driver with modifications
 *   based on additional information from:
 *
 *   - "USB On-The-Go (OTG)", DS61126E, Microchip Technology Inc., 2009
 *   - Sample code provided with the Sure Electronics PIC32 board
 *     (which seems to have derived from Microchip PICDEM PIC18 code).
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
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include <arch/irq.h>

#include "up_arch.h"
#include "pic32mx-internal.h"
#include "pic32mx-usbotg.h"

#if defined(CONFIG_USBDEV) && defined(CONFIG_PIC32MX_USBDEV)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_USBDEV_EP0_MAXSIZE
#  define CONFIG_USBDEV_EP0_MAXSIZE 64
#endif

/* Extremely detailed register/BDT debug that you would normally never want
 * enabled.
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_PIC32MX_USBDEV_REGDEBUG
#  undef CONFIG_PIC32MX_USBDEV_BDTDEBUG
#endif

/* Interrupts ***************************************************************/
/* Initial interrupt sets */

#ifdef CONFIG_USB_SOFINTS
#  define USB_SOF_INTERRUPT USB_INT_SOF
#else
#  define USB_SOF_INTERRUPT 0
#endif

#define ERROR_INTERRUPTS  (USB_EINT_PID|USB_EINT_CRC5|USB_EINT_EOF|\
                           USB_EINT_CRC16|USB_EINT_DFN8|USB_EINT_BTO|\
                           USB_EINT_BTS)
#define NORMAL_INTERRUPTS (USB_INT_URST|USB_INT_UERR|USB_SOF_INTERRUPT|\
                           USB_INT_TRN|USB_INT_IDLE|USB_INT_STALL)

/* Endpoints ****************************************************************/
/* Endpoint identifiers. The PIC32MX supports up to 16 mono-directional or 8
 * bidirectional endpoints.  However, when you take into account PMA buffer
 * usage (see below) and the fact that EP0 is bidirectional, then there is
 * a functional limitation of EP0 + 5 mono-directional endpoints = 6.  We'll
 * define PIC32MX_NENDPOINTS to be 8, however, because that is how many
 * endpoint register sets there are.
 */

#define PIC32MX_NENDPOINTS    (16)
#define EP0                   (0)

#define PIC32MX_ENDP_BIT(ep)  (1 << (ep))
#define PIC32MX_ENDP_ALLSET   0xffff

/* BDT Table Indexing.  The BDT is addressed in the hardware as follows:
 *
 *   Bits 9-31:  These come the BDT address bits written into the BDTP3,
 *      BDTP2, and BDTP1 registers
 *   Bits 5-8:   The endpoint number
 *   Bit 4:      Direction:
 *               1 = Transmit: SETUP/OUT for host, IN for function
 *               0 = Receive: IN for host, SETUP/OUT for function
 *   Bit 3:      PPBI, the ping point buffer index bit (0=EVEN, 1=ODD)
 *   Bits 0-2:   Supports 8-byte BDT entries
 */

#define EP0_OUT_EVEN          (0)
#define EP0_OUT_ODD           (1)
#define EP0_IN_EVEN           (2)
#define EP0_IN_ODD            (3)
#define EP_OUT_EVEN(ep)       ((ep) << 2)
#define EP_OUT_ODD(ep)        (((ep) << 2) + 1)
#define EP_IN_EVEN(ep)        (((ep) << 2) + 2)
#define EP_IN_ODD(ep)         (((ep) << 2) + 3)

#define EP(ep,dir,pp)         (((ep) << 2) + ((dir) << 1) + (pp))
#define EP_DIR_OUT            0
#define EP_DIR_IN             1
#define EP_PP_EVEN            0
#define EP_PP_ODD             1

/* Packet sizes.  We use a fixed 64 max packet size for all endpoint types */

#define PIC32MX_MAXPACKET_SHIFT (6)
#define PIC32MX_MAXPACKET_SIZE  (1 << (PIC32MX_MAXPACKET_SHIFT))

#define PIC32MX_EP0MAXPACKET  PIC32MX_MAXPACKET_SIZE

/* Endpoint register initialization parameters */

#define PIC32MX_EP_CONTROL    (USB_EP_EPHSHK|USB_EP_EPTXEN|USB_EP_EPRXEN)
#define PIC32MX_EP_BULKIN     (USB_EP_EPTXEN|USB_EP_EPCONDIS|USB_EP_EPHSHK)
#define PIC32MX_EP_BULKOUT    (USB_EP_EPRXEN|USB_EP_EPCONDIS|USB_EP_EPHSHK)
#define PIC32MX_EP_INTIN      (USB_EP_EPTXEN|USB_EP_EPCONDIS|USB_EP_EPHSHK)
#define PIC32MX_EP_INTOUT     (USB_EP_EPRXEN|USB_EP_EPCONDIS|USB_EP_EPHSHK)
#define PIC32MX_EP_ISOCIN     (USB_EP_EPTXEN|USB_EP_EPCONDIS)
#define PIC32MX_EP_ISOCOUT    (USB_EP_EPRXEN|USB_EP_EPCONDIS)

/* USB-related masks */

#define REQRECIPIENT_MASK     (USB_REQ_TYPE_MASK | USB_REQ_RECIPIENT_MASK)

/* Request queue operations *************************************************/

#define pic32mx_rqempty(ep)   ((ep)->head == NULL)
#define pic32mx_rqpeek(ep)    ((ep)->head)

/* USB trace ****************************************************************/
/* Trace error codes */

#define PIC32MX_TRACEERR_ALLOCFAIL            0x0001
#define PIC32MX_TRACEERR_BADCLEARFEATURE      0x0002
#define PIC32MX_TRACEERR_BADDEVGETSTATUS      0x0003
#define PIC32MX_TRACEERR_BADEPGETSTATUS       0x0004
#define PIC32MX_TRACEERR_BADEPNO              0x0005
#define PIC32MX_TRACEERR_BADEPTYPE            0x0006
#define PIC32MX_TRACEERR_BADGETCONFIG         0x0007
#define PIC32MX_TRACEERR_BADGETSETDESC        0x0008
#define PIC32MX_TRACEERR_BADGETSTATUS         0x0009
#define PIC32MX_TRACEERR_BADSETADDRESS        0x000a
#define PIC32MX_TRACEERR_BADSETCONFIG         0x000b
#define PIC32MX_TRACEERR_BADSETFEATURE        0x000c
#define PIC32MX_TRACEERR_BINDFAILED           0x000d
#define PIC32MX_TRACEERR_DISPATCHSTALL        0x000e
#define PIC32MX_TRACEERR_DRIVER               0x000f
#define PIC32MX_TRACEERR_DRIVERREGISTERED     0x0010
#define PIC32MX_TRACEERR_EP0SETUPSTALLED      0x0011
#define PIC32MX_TRACEERR_EPDISABLED           0x0012
#define PIC32MX_TRACEERR_EPOUTNULLPACKET      0x0013
#define PIC32MX_TRACEERR_EPRESERVE            0x0014
#define PIC32MX_TRACEERR_INVALIDCTRLREQ       0x0015
#define PIC32MX_TRACEERR_INVALIDPARMS         0x0016
#define PIC32MX_TRACEERR_IRQREGISTRATION      0x0017
#define PIC32MX_TRACEERR_NOTCONFIGURED        0x0018
#define PIC32MX_TRACEERR_REQABORTED           0x0019
#define PIC32MX_TRACEERR_INVALIDSTATE         0x001a

/* Trace interrupt codes */

#define PIC32MX_TRACEINTID_CLEARFEATURE       0x0001
#define PIC32MX_TRACEINTID_DEVGETSTATUS       0x0002
#define PIC32MX_TRACEINTID_DISPATCH           0x0003
#define PIC32MX_TRACEINTID_EP0IN              0x0004
#define PIC32MX_TRACEINTID_EP0INDONE          0x0005
#define PIC32MX_TRACEINTID_EP0OUTDONE         0x0006
#define PIC32MX_TRACEINTID_EP0SETUPDONE       0x0007
#define PIC32MX_TRACEINTID_EP0SETUPSETADDRESS 0x0008
#define PIC32MX_TRACEINTID_EP0ADDRESSSET      0x0009
#define PIC32MX_TRACEINTID_EPGETSTATUS        0x000a
#define PIC32MX_TRACEINTID_EPINDONE           0x000b
#define PIC32MX_TRACEINTID_EPINQEMPTY         0x000c
#define PIC32MX_TRACEINTID_EPOUTDONE          0x000d
#define PIC32MX_TRACEINTID_EPOUTPENDING       0x000e
#define PIC32MX_TRACEINTID_EPOUTQEMPTY        0x000f
#define PIC32MX_TRACEINTID_SOF                0x0010
#define PIC32MX_TRACEINTID_GETCONFIG          0x0011
#define PIC32MX_TRACEINTID_GETSETDESC         0x0012
#define PIC32MX_TRACEINTID_GETSETIF           0x0013
#define PIC32MX_TRACEINTID_GETSTATUS          0x0014
#define PIC32MX_TRACEINTID_IFGETSTATUS        0x0015
#define PIC32MX_TRACEINTID_TRNC               0x0016
#define PIC32MX_TRACEINTID_TRNCS              0x0017
#define PIC32MX_TRACEINTID_INTERRUPT          0x0018
#define PIC32MX_TRACEINTID_NOSTDREQ           0x0019
#define PIC32MX_TRACEINTID_RESET              0x001a
#define PIC32MX_TRACEINTID_SETCONFIG          0x001b
#define PIC32MX_TRACEINTID_SETFEATURE         0x001c
#define PIC32MX_TRACEINTID_IDLE               0x001d
#define PIC32MX_TRACEINTID_SYNCHFRAME         0x001e
#define PIC32MX_TRACEINTID_WKUP               0x001f
#define PIC32MX_TRACEINTID_T1MSEC             0x0020
#define PIC32MX_TRACEINTID_OTGID              0x0021
#define PIC32MX_TRACEINTID_STALL              0x0022
#define PIC32MX_TRACEINTID_UERR               0x0023
#define PIC32MX_TRACEINTID_SUSPENDED          0x0024
#define PIC32MX_TRACEINTID_WAITRESET          0x0025

/* Misc Helper Macros *******************************************************/

#define PHYS_ADDR(va) ((uint32_t)(va) & 0x1fffffff)
#define VIRT_ADDR(pa) (KSEG1_BASE | (uint32_t)(pa))

/* Ever-present MIN and MAX macros */

#ifndef MIN
#  define MIN(a,b) (a < b ? a : b)
#endif

#ifndef MAX
#  define MAX(a,b) (a > b ? a : b)
#endif

/* Byte ordering in host-based values */

#ifdef CONFIG_ENDIAN_BIG
#  define LSB 1
#  define MSB 0
#else
#  define LSB 0
#  define MSB 1
#endif

/* Debug ********************************************************************/

#ifdef CONFIG_PIC32MX_USBDEV_REGDEBUG

#  undef CONFIG_PIC32MX_USBDEV_BDTDEBUG
#  define CONFIG_PIC32MX_USBDEV_BDTDEBUG 1

#  define regdbg lldbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define regvdbg lldbg
#  else
#    define regvdbg(x...)
#  endif

#else

#  define pic32mx_getreg(addr)      getreg16(addr)
#  define pic32mx_putreg(val,addr)  putreg16(val,addr)
#  define regdbg(x...)
#  define regvdbg(x...)

#endif

#ifdef CONFIG_PIC32MX_USBDEV_BDTDEBUG

#  define bdtdbg lldbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define bdtvdbg lldbg
#  else
#    define bdtvdbg(x...)
#  endif

#else

#  define bdtdbg(x...)
#  define bdtvdbg(x...)
#  define pic32mx_ep0bdtdump(msg)

#endif

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* Overvall device state */

enum pic32mx_devstate_e
{
  DEVSTATE_DETACHED = 0,  /* Not connected to a host */
  DEVSTATE_ATTACHED,      /* Connected to a host */
  DEVSTATE_POWERED,       /* Powered */
  DEVSTATE_DEFAULT,       /* Default state */
  DEVSTATE_ADDRPENDING,   /* Waiting for an address */
  DEVSTATE_ADDRESS,       /* Address received */
  DEVSTATE_CONFIGURED,    /* Configuration received */
};

/* The various states of the control pipe */

enum pic32mx_ctrlstate_e
{
  CTRLSTATE_WAITSETUP = 0,  /* No request in progress, waiting for setup */
  CTRLSTATE_RDREQUEST,      /* Read request (OUT) in progress */
  CTRLSTATE_WRREQUEST,      /* Write request (IN) in progress */
  CTRLSTATE_STALLED         /* We are stalled */
};

union wb_u
{
  uint16_t w;
  uint8_t  b[2];
};

/* A container for a request so that the request make be retained in a list */

struct pic32mx_req_s
{
  struct usbdev_req_s   req;           /* Standard USB request */
  uint16_t              inflight;      /* The number of bytes "in-flight" */
  struct pic32mx_req_s *flink;         /* Supports a singly linked list */
};

/* This is the internal representation of an endpoint */

struct pic32mx_ep_s
{
  /* Common endpoint fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_ep_s
   * to struct pic32mx_ep_s.
   */

  struct usbdev_ep_s       ep;            /* Standard endpoint structure */

  /* PIC32MX-specific fields */

  struct pic32mx_usbdev_s *dev;           /* Reference to private driver data */
  struct pic32mx_req_s    *head;          /* Request list for this endpoint */
  struct pic32mx_req_s    *tail;
  uint8_t                  stalled:1;     /* true: Endpoint is stalled */
  uint8_t                  halted:1;      /* true: Endpoint feature halted */
  uint8_t                  txbusy:1;      /* true: TX endpoint FIFO full */
  uint8_t                  txnullpkt:1;   /* Null packet needed at end of transfer */
  volatile struct usbotg_bdtentry_s *bdtin;  /* BDT entry for the IN transaction*/
  volatile struct usbotg_bdtentry_s *bdtout; /* BDT entry for the OUT transaction */
};

struct pic32mx_usbdev_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_s
   * to structpic32mx_usbdev_s.
   */

  struct usbdev_s usbdev;

  /* The bound device class driver */

  struct usbdevclass_driver_s *driver;

  /* PIC32MX-specific fields */

  struct usb_ctrlreq_s     ctrl;          /* Last EP0 request */
  uint8_t                  devstate;      /* Driver state (see enum pic32mx_devstate_e) */
  uint8_t                  ctrlstate;     /* Control EP state (see enum pic32mx_ctrlstate_e) */
  uint8_t                  selfpowered:1; /* 1: Device is self powered */
  uint8_t                  rwakeup:1;     /* 1: Device supports remote wakeup */
  uint8_t                  attached:1;    /* Device is attached to the host */
  uint8_t                  ep0done:1;    /* EP0 OUT already prepared */
  uint16_t                 epavail;       /* Bitset of available endpoints */

  /* The endpoint list */

  struct pic32mx_ep_s      eplist[PIC32MX_NENDPOINTS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

#ifdef CONFIG_PIC32MX_USBDEV_REGDEBUG
static uint16_t pic32mx_getreg(uint32_t addr);
static void pic32mx_putreg(uint16_t val, uint32_t addr);
#endif

/* Suspend/Resume Helpers ***************************************************/

static void   pic32mx_suspend(struct pic32mx_usbdev_s *priv);
static void   pic32mx_resume(struct pic32mx_usbdev_s *priv);

/* Request Helpers **********************************************************/

static struct pic32mx_req_s *
              pic32mx_rqdequeue(struct pic32mx_ep_s *privep);
static void   pic32mx_rqenqueue(struct pic32mx_ep_s *privep,
                struct pic32mx_req_s *req);
static inline void
              pic32mx_abortrequest(struct pic32mx_ep_s *privep,
                struct pic32mx_req_s *privreq, int16_t result);
static void   pic32mx_reqcomplete(struct pic32mx_ep_s *privep, int16_t result);
static void   pic32mx_epwrite(struct pic32mx_ep_s *privep,
                const uint8_t *src, uint32_t nbytes);
static void   pic32mx_wrcomplete(struct pic32mx_usbdev_s *priv,
                struct pic32mx_ep_s *privep);
static void   pic32mx_wrrestart(struct pic32mx_usbdev_s *priv,
                struct pic32mx_ep_s *privep);
static int    pic32mx_wrrequest(struct pic32mx_usbdev_s *priv,
                struct pic32mx_ep_s *privep);
static int    pic32mx_rdcomplete(struct pic32mx_usbdev_s *priv,
                struct pic32mx_ep_s *privep);
static int    pic32mx_ep0rdsetup(struct pic32mx_usbdev_s *priv,
                uint8_t *dest, int readlen);
static int    pic32mx_rdsetup(struct pic32mx_usbdev_s *priv,
                struct pic32mx_ep_s *privep, uint8_t *dest, int readlen);
static int    pic32mx_rdrequest(struct pic32mx_usbdev_s *priv,
                struct pic32mx_ep_s *privep);
static void   pic32mx_cancelrequests(struct pic32mx_ep_s *privep);

/* Interrupt level processing ***********************************************/

static void   pic32mx_dispatchrequest(struct pic32mx_usbdev_s *priv);
static void   pic32mx_ep0stall(struct pic32mx_usbdev_s *priv);
static void   pic32mx_eptransfer(struct pic32mx_usbdev_s *priv, uint8_t epno,
                uint16_t status);
static void   pic32mx_ep0nextsetup(struct pic32mx_usbdev_s *priv);
static void   pic32mx_ep0setup(struct pic32mx_usbdev_s *priv);
static void   pic32mx_ep0outcomplete(struct pic32mx_usbdev_s *priv);
static void   pic32mx_ep0incomplete(struct pic32mx_usbdev_s *priv);
static void   pic32mx_ep0transfer(struct pic32mx_usbdev_s *priv,
                uint16_t status);
static int    pic32mx_interrupt(int irq, void *context);

/* Endpoint helpers *********************************************************/

static inline struct pic32mx_ep_s *
              pic32mx_epreserve(struct pic32mx_usbdev_s *priv, uint8_t epset);
static inline void
              pic32mx_epunreserve(struct pic32mx_usbdev_s *priv,
                struct pic32mx_ep_s *privep);
static inline bool
              pic32mx_epreserved(struct pic32mx_usbdev_s *priv, int epno);
static void  pic32mx_ep0configure(struct pic32mx_usbdev_s *priv);
#ifdef CONFIG_PIC32MX_USBDEV_BDTDEBUG
static void  pic32mx_ep0bdtdump(const char *msg);
#endif

/* Endpoint operations ******************************************************/

static int    pic32mx_epconfigure(struct usbdev_ep_s *ep,
                const struct usb_epdesc_s *desc, bool last);
static int    pic32mx_epdisable(struct usbdev_ep_s *ep);
static struct usbdev_req_s *
              pic32mx_epallocreq(struct usbdev_ep_s *ep);
static void   pic32mx_epfreereq(struct usbdev_ep_s *ep,
                struct usbdev_req_s *);
static int    pic32mx_epsubmit(struct usbdev_ep_s *ep,
                struct usbdev_req_s *req);
static int    pic32mx_epcancel(struct usbdev_ep_s *ep,
                struct usbdev_req_s *req);
static int    pic32mx_epbdtstall(struct usbdev_ep_s *ep,
                volatile struct usbotg_bdtentry_s *bdt, bool resume,
                bool epin);
static int    pic32mx_epstall(struct usbdev_ep_s *ep, bool resume);

/* USB device controller operations *****************************************/

static struct usbdev_ep_s *
              pic32mx_allocep(struct usbdev_s *dev, uint8_t epno, bool in,
                uint8_t eptype);
static void   pic32mx_freeep(struct usbdev_s *dev, struct usbdev_ep_s *ep);
static int    pic32mx_getframe(struct usbdev_s *dev);
static int    pic32mx_wakeup(struct usbdev_s *dev);
static int    pic32mx_selfpowered(struct usbdev_s *dev, bool selfpowered);

/* Initialization/Reset *****************************************************/

static void   pic32mx_reset(struct pic32mx_usbdev_s *priv);
static void   pic32mx_attach(struct pic32mx_usbdev_s *priv);
static void   pic32mx_detach(struct pic32mx_usbdev_s *priv);
static void   pic32mx_hwreset(struct pic32mx_usbdev_s *priv);
static void   pic32mx_stateinit(struct pic32mx_usbdev_s *priv);
static void   pic32mx_hwshutdown(struct pic32mx_usbdev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Since there is only a single USB interface, all status information can be
 * be simply retained in a single global instance.
 */

static struct pic32mx_usbdev_s g_usbdev;

static const struct usbdev_epops_s g_epops =
{
  .configure   = pic32mx_epconfigure,
  .disable     = pic32mx_epdisable,
  .allocreq    = pic32mx_epallocreq,
  .freereq     = pic32mx_epfreereq,
  .submit      = pic32mx_epsubmit,
  .cancel      = pic32mx_epcancel,
  .stall       = pic32mx_epstall,
};

static const struct usbdev_ops_s g_devops =
{
  .allocep     = pic32mx_allocep,
  .freeep      = pic32mx_freeep,
  .getframe    = pic32mx_getframe,
  .wakeup      = pic32mx_wakeup,
  .selfpowered = pic32mx_selfpowered,
  .pullup      = pic32mx_usbpullup,
};

/* Buffer Descriptor Table.  Four BDT entries per
 *
 * The BDT is addressed in the hardware as follows:
 *
 *   Bits 9-31:  These come the BDT address bits written into the BDTP3, BDTP2
 *      and BDTP1 registers
 *   Bits 5-8:   The endpoint number
 *   Bit 4:      Direction (0=IN/Tx, 1 = OUT/Rx)
 *   Bit 3:      PPBI, the ping point buffer index bit.
 *   Bits 0-2:   Supports 8-byte BDT entries
 */

static volatile struct usbotg_bdtentry_s g_bdt[4*PIC32MX_NENDPOINTS]
  __attribute__ ((aligned(512)));

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Private Functions
 ****************************************************************************/

/****************************************************************************
 * Register Operations
 ****************************************************************************/
/****************************************************************************
 * Name: pic32mx_getreg
 ****************************************************************************/

#ifdef CONFIG_PIC32MX_USBDEV_REGDEBUG
static uint16_t pic32mx_getreg(uint32_t addr)
{
  static uint32_t prevaddr = 0;
  static uint16_t preval = 0;
  static uint32_t count = 0;

  /* Read the value from the register */

  uint16_t val = getreg16(addr);

  /* Is this the same value that we read from the same register last time?
   * Are we polling the register?  If so, suppress some of the output.
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

  lldbg("%08x->%04x\n", addr, val);
  return val;
}
#endif

/****************************************************************************
 * Name: pic32mx_putreg
 ****************************************************************************/

#ifdef CONFIG_PIC32MX_USBDEV_REGDEBUG
static void pic32mx_putreg(uint16_t val, uint32_t addr)
{
  /* Show the register value being written */

  lldbg("%08x<-%04x\n", addr, val);

  /* Write the value */

  putreg16(val, addr);
}
#endif

/****************************************************************************
 * Request Helpers
 ****************************************************************************/
/****************************************************************************
 * Name: pic32mx_rqdequeue
 ****************************************************************************/

static struct pic32mx_req_s *pic32mx_rqdequeue(struct pic32mx_ep_s *privep)
{
  struct pic32mx_req_s *ret = privep->head;

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

/****************************************************************************
 * Name: pic32mx_rqenqueue
 ****************************************************************************/

static void pic32mx_rqenqueue(struct pic32mx_ep_s *privep, struct pic32mx_req_s *req)
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

/****************************************************************************
 * Name: pic32mx_abortrequest
 ****************************************************************************/

static inline void
pic32mx_abortrequest(struct pic32mx_ep_s *privep, struct pic32mx_req_s *privreq, int16_t result)
{
  usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_REQABORTED), (uint16_t)USB_EPNO(privep->ep.eplog));

  /* Save the result in the request structure */

  privreq->req.result = result;

  /* Callback to the request completion handler */

  privreq->req.callback(&privep->ep, &privreq->req);
}

/****************************************************************************
 * Name: pic32mx_reqcomplete
 ****************************************************************************/

static void pic32mx_reqcomplete(struct pic32mx_ep_s *privep, int16_t result)
{
  struct pic32mx_req_s *privreq;
  irqstate_t flags;

  /* Remove the completed request at the head of the endpoint request list */

  flags = irqsave();
  privreq = pic32mx_rqdequeue(privep);
  irqrestore(flags);

  if (privreq)
    {
      /* If endpoint 0, temporarily reflect the state of protocol stalled
       * in the callback.
       */

      bool stalled = privep->stalled;
      if (USB_EPNO(privep->ep.eplog) == EP0)
        {
          privep->stalled = (privep->dev->ctrlstate == CTRLSTATE_STALLED);
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

/****************************************************************************
 * Name: pic32mx_epwrite
 ****************************************************************************/

static void pic32mx_epwrite(struct pic32mx_ep_s *privep, const uint8_t *src,
                            uint32_t nbytes)
{
  volatile struct usbotg_bdtentry_s *bdt = privep->bdtin;
  uint32_t status;

  usbtrace(TRACE_WRITE(USB_EPNO(privep->ep.eplog)), nbytes);

  /* Clear all bits in the status preserving only the data toggle bit */

  status      = bdt->status;
  status     &= USB_BDT_DATA01;

  /* Toggle the data toggle */

  status     ^= USB_BDT_DATA01;
  bdt->status = status;

  /* Set the data pointer, data length, and enable the endpoint */

  bdt->addr = (uint8_t *)PHYS_ADDR(src);

  /* Set the data length and give the BDT to USB.  Preserving
   * only the data toggle.
   */

  status |= (nbytes << USB_BDT_BYTECOUNT_SHIFT) | USB_BDT_DTS;

  /* And, finally, give the BDT to the USB */

  status |= USB_BDT_UOWN;

  bdtdbg("EP%d BDT IN [%p] {%08x, %08x}\n",
         USB_EPNO(privep->ep.eplog), bdt, status, bdt->addr);

  bdt->status = status;

  /* Indicate that there is TX data inflight.  This will be cleared
   * when the next data out interrupt is received.
   */

  privep->txbusy = true;
}

/****************************************************************************
 * Name: pic32mx_wrcomplete
 ****************************************************************************/

static void pic32mx_wrcomplete(struct pic32mx_usbdev_s *priv,
                               struct pic32mx_ep_s *privep)
{
  struct pic32mx_req_s *privreq;
  int bytesleft;

  /* Check the request from the head of the endpoint request queue.  Since
   * we got here from a write completion event, the request queue should
   * be non-NULL.
   */

  privreq = pic32mx_rqpeek(privep);
  if (privreq != NULL)
    {
      /* An outgoing IN packet has completed. Update the number of bytes
       * transferred.
       */
   
      privreq->req.xfrd += privreq->inflight;
      privreq->inflight  = 0;
      bytesleft          = privreq->req.len - privreq->req.xfrd;

      /* If all of the bytes were sent (bytesleft == 0) and no NULL packet is
       * needed (!txnullpkt), then we are finished with the transfer
       */

      if (bytesleft == 0 && !privep->txnullpkt)
        {

          /* The transfer is complete.  Give the completed request back to
           * the class driver.
          */

          usbtrace(TRACE_COMPLETE(USB_EPNO(privep->ep.eplog)), privreq->req.xfrd);
          pic32mx_reqcomplete(privep, OK);

          /* Special case writes to endpoint zero.  If there is no transfer in
           * progress, then we need to configure to received the next SETUP packet.
          */

          if (USB_EPNO(privep->ep.eplog) == 0)
            {
              priv->ctrlstate = CTRLSTATE_WAITSETUP;
            }
        }
    }
}

/****************************************************************************
 * Name: pic32mx_wrrestart
 ****************************************************************************/

static void pic32mx_wrrestart(struct pic32mx_usbdev_s *priv,
                              struct pic32mx_ep_s *privep)
{
  struct pic32mx_req_s *privreq;

  /* Reset some endpoint state variables */

  privep->txnullpkt = false;
  privep->txbusy    = false;

  /* Check the request from the head of the endpoint request queue */

  privreq = pic32mx_rqpeek(privep);
  if (!privreq)
    {
      /* Restart transmission after we have recovered from a stall */

      privreq->req.xfrd = 0;
      privreq->inflight = 0;
      (void)pic32mx_wrrequest(priv, privep);
    }
}

/****************************************************************************
 * Name: pic32mx_wrrequest
 ****************************************************************************/

static int pic32mx_wrrequest(struct pic32mx_usbdev_s *priv, struct pic32mx_ep_s *privep)
{
  struct pic32mx_req_s *privreq;
  uint8_t *buf;
  uint8_t epno;
  int nbytes;
  int bytesleft;

  /* We get here when an IN endpoint interrupt occurs.  So now we know that
   * there is no TX transfer in progress.
   */

  privep->txbusy = false;

  /* Get the endpoint number that we are servicing */

  epno = USB_EPNO(privep->ep.eplog);

  /* Check the request from the head of the endpoint request queue */

  privreq = pic32mx_rqpeek(privep);
  if (!privreq)
    {
      /* There is no TX transfer in progress and no new pending TX
       * requests to send.
       */

      usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_EPINQEMPTY), epno);

      /* Special case endpoint zero.  If there is nothing to be sent, then
       * we need to configure to received the next SETUP packet.
       */

      if (epno == 0)
        {
          priv->ctrlstate = CTRLSTATE_WAITSETUP;
        }
      return OK;
    }

  ullvdbg("epno=%d req=%p: len=%d xfrd=%d nullpkt=%d\n",
          epno, privreq, privreq->req.len, privreq->req.xfrd, privep->txnullpkt);

  /* Get the number of bytes left to be sent in the packet */

  bytesleft = privreq->req.len - privreq->req.xfrd;
  nbytes    = bytesleft;

  /* Send the next packet */

  if (nbytes > 0)
    {
      /* Either send the maxpacketsize or all of the remaining data in
       * the request.
       */

      privep->txnullpkt = 0;
      if (nbytes >= privep->ep.maxpacket)
        {
          nbytes =  privep->ep.maxpacket;

          /* Handle the case where this packet is exactly the
           * maxpacketsize.  Do we need to send a zero-length packet
           * in this case?
           */

          if (bytesleft ==  privep->ep.maxpacket &&
             (privreq->req.flags & USBDEV_REQFLAGS_NULLPKT) != 0)
            {
              privep->txnullpkt = 1;
            }
        }
    }

  /* Send the packet (might be a null packet nbytes == 0) */

  buf = privreq->req.buf + privreq->req.xfrd;

  /* Setup the writes to the endpoints */

  pic32mx_epwrite(privep, buf, nbytes);

  /* Special case endpoint 0 state information.  The write request is in
   * progress.
   */

  if (epno == 0)
    {
      priv->ctrlstate = CTRLSTATE_WRREQUEST;
    }

  /* Update for the next data IN interrupt */

  privreq->inflight = nbytes;
  return OK;
}

/****************************************************************************
 * Name: pic32mx_rdcomplete
 ****************************************************************************/

static int pic32mx_rdcomplete(struct pic32mx_usbdev_s *priv,
                              struct pic32mx_ep_s *privep)
{
  volatile struct usbotg_bdtentry_s *bdt = privep->bdtout;
  struct pic32mx_req_s *privreq;
  int readlen;

  /* Check the request from the head of the endpoint request queue */

  privreq = pic32mx_rqpeek(privep);
  if (!privreq)
    {
      /* There is no packet to receive any data. Then why are we here? */

      usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_EPOUTQEMPTY),
               USB_EPNO(privep->ep.eplog));
      return -EINVAL;
    }

  ullvdbg("EP%d: len=%d xfrd=%d BDT={%08x, %08x}\n",
          USB_EPNO(privep->ep.eplog), privreq->req.len, privreq->req.xfrd,
          bdt->status, bdt->addr);

  /* Get the length of the data received from the BDT */

  readlen = (bdt->status & USB_BDT_BYTECOUNT_MASK) >> USB_BDT_BYTECOUNT_SHIFT;

  /* If the receive buffer is full or this is a partial packet,
   * then we are finished with the transfer
   */

  privreq->req.xfrd += readlen;
  if (readlen < privep->ep.maxpacket || privreq->req.xfrd >= privreq->req.len)
    {
      /* Complete the transfer and mark the state IDLE.  The endpoint
       * RX will be marked valid when the data phase completes.
       */

      usbtrace(TRACE_COMPLETE(USB_EPNO(privep->ep.eplog)), privreq->req.xfrd);
      pic32mx_reqcomplete(privep, OK);
    }

  /* Set up the next read operation */

  return pic32mx_rdrequest(priv, privep);
}

/****************************************************************************
 * Name: pic32mx_ep0rdsetup
 ****************************************************************************/

static int pic32mx_ep0rdsetup(struct pic32mx_usbdev_s *priv, uint8_t *dest,
                              int readlen)
{
  volatile struct usbotg_bdtentry_s *bdt = priv->eplist[EP0].bdtout;
  uint16_t status;

  /* Clear status bits (making sure that UOWN is cleared before doing anything
   * else.
   */

  status      = bdt->status;
  status     &= ~(USB_BDT_BSTALL | USB_BDT_NINC | USB_BDT_KEEP| USB_BDT_UOWN |
                  USB_BDT_BYTECOUNT_MASK | USB_BDT_DATA01);
  bdt->status = status;

  /* Set the data pointer, data length, and enable the endpoint */

  bdt->addr   = (uint8_t *)PHYS_ADDR(&priv->ctrl);
  status     |= (USB_SIZEOF_CTRLREQ << USB_BDT_BYTECOUNT_SHIFT);

  /* Select data0 or data 1 when enabling */

  if ((status & USB_BDT_DATA01) == USB_BDT_DATA0)
    {
      status |= (USB_BDT_UOWN | USB_BDT_DATA1 | USB_BDT_DTS);
    }
  else
    {
      status |= (USB_BDT_UOWN | USB_BDT_DATA0 | USB_BDT_DTS);
    }

  /* Then give the BDT to the USB */

  bdtdbg("EP0 BDT OUT [%p] {%08x, %08x}\n", bdt, status, bdt->addr);

  bdt->status     = status;
  priv->ctrlstate = CTRLSTATE_RDREQUEST;
  return OK;
}

/****************************************************************************
 * Name: pic32mx_rdsetup
 ****************************************************************************/

static int pic32mx_rdsetup(struct pic32mx_usbdev_s *priv,
                           struct pic32mx_ep_s *privep, uint8_t *dest, int readlen)
{
  volatile struct usbotg_bdtentry_s *bdt = privep->bdtout;
  uint16_t status;

  /* Clear status bits (making sure that UOWN is cleared before doing anything
   * else).  The DATA01 is (only) is preserved.
   */

  status      = bdt->status & USB_BDT_DATA01;
  bdt->status = status;

  /* Set the data pointer, data length, and enable the endpoint */

  bdt->addr = (uint8_t *)PHYS_ADDR(dest);

  /* Set the data length (preserving the data toggle). */

  status |= (readlen << USB_BDT_BYTECOUNT_SHIFT) | USB_BDT_DTS;

  /* Then give the BDT to the USB */

  status |= USB_BDT_UOWN;

  bdtdbg("EP%d BDT OUT [%p] {%08x, %08x}\n",
         USB_EPNO(privep->ep.eplog), bdt, status, bdt->addr);

  bdt->status = status;
  return OK;
}

/****************************************************************************
 * Name: pic32mx_rdrequest
 ****************************************************************************/

static int pic32mx_rdrequest(struct pic32mx_usbdev_s *priv,
                             struct pic32mx_ep_s *privep)
{
  struct pic32mx_req_s *privreq;
  uint8_t *dest;
  int readlen;
  int ret;

  /* Check the request from the head of the endpoint request queue */

  privreq = pic32mx_rqpeek(privep);
  if (!privreq)
    {
      /* There is no packet to receive any data. */

      usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_EPOUTQEMPTY),
               USB_EPNO(privep->ep.eplog));
      return OK;
    }

  ullvdbg("EP%d: len=%d xfrd=%d\n",
          USB_EPNO(privep->ep.eplog), privreq->req.len, privreq->req.xfrd);

  /* Ignore any attempt to receive a zero length packet */

  if (privreq->req.len == 0)
    {
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_EPOUTNULLPACKET), 0);
      pic32mx_reqcomplete(privep, OK);
      return OK;
    }

  usbtrace(TRACE_READ(USB_EPNO(privep->ep.eplog)), privreq->req.xfrd);

  /* Get the destination transfer address and size */

  dest = privreq->req.buf + privreq->req.xfrd;
  readlen = MIN(privreq->req.len, privep->ep.maxpacket);

  /* Handle EP0 in a few special ways */

  if (USB_EPNO(privep->ep.eplog) == EP0)
    {
      ret = pic32mx_ep0rdsetup(priv, dest, readlen);
    }
  else
    {
      ret = pic32mx_rdsetup(priv, privep, dest, readlen);
    }
  return ret;
}

/****************************************************************************
 * Name: pic32mx_cancelrequests
 ****************************************************************************/

static void pic32mx_cancelrequests(struct pic32mx_ep_s *privep)
{
  while (!pic32mx_rqempty(privep))
    {
      usbtrace(TRACE_COMPLETE(USB_EPNO(privep->ep.eplog)),
               (pic32mx_rqpeek(privep))->req.xfrd);
      pic32mx_reqcomplete(privep, -ESHUTDOWN);
    }
}

/****************************************************************************
 * Interrupt Level Processing
 ****************************************************************************/
/****************************************************************************
 * Name: pic32mx_dispatchrequest
 ****************************************************************************/

static void pic32mx_dispatchrequest(struct pic32mx_usbdev_s *priv)
{
  int ret;

  usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_DISPATCH), 0);
  if (priv && priv->driver)
    {
      /* Forward to the control request to the class driver implementation */

      ret = CLASS_SETUP(priv->driver, &priv->usbdev, &priv->ctrl);
      if (ret < 0)
        {
          /* Stall on failure */

          usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_DISPATCHSTALL), 0);
          priv->ctrlstate = CTRLSTATE_STALLED;
        }
    }
}

/****************************************************************************
 * Name: pic32mx_ep0stall
 ****************************************************************************/

static void pic32mx_ep0stall(struct pic32mx_usbdev_s *priv)
{
  uint16_t regval;

  /* Check if EP0 is stalled */

  regval = pic32mx_getreg(PIC32MX_USB_EP0);
  if ((regval & USB_EP_EPSTALL) != 0)
    {
      /* Check if we own the BDTs.  All BDs of endpoint zero should be owned by
       * the USB.  Check anyway.
       */

      struct pic32mx_ep_s *ep0 = &priv->eplist[EP0];

      if ((ep0->bdtout->status & USB_BDT_UOWN) != 0 &&
          (ep0->bdtin->status & (USB_BDT_UOWN | USB_BDT_BSTALL)) == (USB_BDT_UOWN | USB_BDT_BSTALL))
        {
          /* Set ep0 BDT status to stall also */

          uint16_t status = (USB_BDT_UOWN| USB_BDT_DATA0 | USB_BDT_DTS | USB_BDT_BSTALL);

          bdtdbg("EP0 BDT OUT [%p] {%08x, %08x}\n",
                 ep0->bdtout, status, ep0->bdtout->addr);

          ep0->bdtout->status = status;
        }

      /* Then clear the EP0 stall status */

      regval &= ~USB_EP_EPSTALL;
      pic32mx_putreg(regval, PIC32MX_USB_EP0);
    }
}

/****************************************************************************
 * Name: pic32mx_eptransfer
 ****************************************************************************/

static void pic32mx_eptransfer(struct pic32mx_usbdev_s *priv, uint8_t epno,
                               uint16_t status)
{
  struct pic32mx_ep_s *privep;

  /* Decode and service non control endpoints interrupt */

  privep = &priv->eplist[epno];

  /* Check if the last transaction was an EP0 OUT transaction */

  if ((status & USB_STAT_DIR) == USB_STAT_DIR_OUT)
    {
      /* OUT: host-to-device */

      usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_EPOUTDONE), status);

      /* Handle read requests.  First check if a read request is available to
       * accept the host data.
       */

      if (!pic32mx_rqempty(privep))
        {
          /* Read host data into the current read request */

          pic32mx_rdcomplete(priv, privep);
        }

      /* NAK further OUT packets if there there no more read requests */

      if (pic32mx_rqempty(privep))
        {
          usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_EPOUTPENDING), (uint16_t)epno);
#warning "Missing logic"
        }
    }
  else
    {
      /* IN: device-to-host */

      usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_EPINDONE), status);

      /* An outgoing IN packet has completed.  Update the number of bytes transferred
       * and check for completion of the transfer.
       */

      pic32mx_wrcomplete(priv, privep);

      /* Handle additional queued write requests */

      pic32mx_wrrequest(priv, privep);
    }
}

/****************************************************************************
 * Name: pic32mx_ep0nextsetup
 ****************************************************************************/

static void pic32mx_ep0nextsetup(struct pic32mx_usbdev_s *priv)
{
  struct pic32mx_ep_s *ep0 = &priv->eplist[EP0];
  volatile struct usbotg_bdtentry_s *bdtlast;
  volatile struct usbotg_bdtentry_s *bdtnext;
  uint32_t status;
  uint32_t bytecount;

  /* This operation should be performed no more than once per OUT transaction.
   * priv->ep0done is set to zero at the beginning of processing of each EP0
   * transfer.  It is set the first time that this function runs after the EP0
   * transfer.
   */

  if (!priv->ep0done)
    {
      /* Which BDT did we just finish working with?  Which one will be next */

      bdtlast = ep0->bdtout;
      if (bdtlast == &g_bdt[EP0_OUT_EVEN])
        {
          bdtnext = &g_bdt[EP0_OUT_ODD];
        }
      else
        {
          DEBUGASSERT(bdtlast == &g_bdt[EP0_OUT_ODD]);
          bdtnext = &g_bdt[EP0_OUT_EVEN];
        }

      /* Setup to receive the next SETUP packet. Data toggle synchronization
       * is not needed for SETUP packets.
       */

      bytecount                = (USB_SIZEOF_CTRLREQ << USB_BDT_BYTECOUNT_SHIFT);

      status                   =  bdtnext->status;
      status                   = (USB_BDT_UOWN | bytecount);

      bdtnext->addr            = (uint8_t *)PHYS_ADDR(&priv->ctrl);
      bdtnext->status          = status;
      priv->eplist[EP0].bdtout = bdtnext;

      /* Force a STALL if there is any access to the other buffer. */

      status                   = bdtlast->status;
      status                   = (USB_BDT_BSTALL | bytecount);

      bdtlast->status          = status;
      bdtlast->addr            = (uint8_t *)PHYS_ADDR(&priv->ctrl);

      priv->ep0done            = 1;
    }
}

/****************************************************************************
 * Name: pic32mx_ep0setup
 ****************************************************************************/

static void pic32mx_ep0setup(struct pic32mx_usbdev_s *priv)
{
  volatile struct usbotg_bdtentry_s *bdt;
  struct pic32mx_ep_s  *ep0     = &priv->eplist[EP0];
  struct pic32mx_req_s *privreq = pic32mx_rqpeek(ep0);
  struct pic32mx_ep_s  *privep;
  union wb_u            value;
  union wb_u            index;
  union wb_u            len;
  union wb_u            response;
  uint16_t              regval;
  bool                  dispatched = false;
  uint8_t               epno;
  int                   nbytes = 0; /* Assume zero-length packet */
  int                   ret;

  /* Terminate any pending requests (doesn't work if the pending request
   * was a zero-length transfer!)
   */

  while (!pic32mx_rqempty(ep0))
    {
      int16_t result = OK;
      if (privreq->req.xfrd != privreq->req.len)
        {
          result = -EPROTO;
        }

      usbtrace(TRACE_COMPLETE(ep0->ep.eplog), privreq->req.xfrd);
      pic32mx_reqcomplete(ep0, result);
    }

  /* Check if the USB currently owns the buffer */

  if ((ep0->bdtin->status & USB_BDT_UOWN) != 0)
    {
      /* Yes.. give control back to the CPU.  This handles for the
       * ownership after a stall.
       */

      bdtdbg("EP0 BDT IN [%p] {%08x, %08x}\n",
             ep0->bdtin, ep0->bdtin->status, ep0->bdtin->addr);

      ep0->bdtin->status &= ~USB_BDT_UOWN;
    }

  /* Assume NOT stalled; no TX in progress */

  ep0->stalled = false;
  ep0->txbusy  = false;

  /* Initialize for the SETUP */

  priv->ctrlstate = CTRLSTATE_WAITSETUP;

  /* And extract the little-endian 16-bit values to host order */

  value.w = GETUINT16(priv->ctrl.value);
  index.w = GETUINT16(priv->ctrl.index);
  len.w   = GETUINT16(priv->ctrl.len);

  ullvdbg("SETUP: type=%02x req=%02x value=%04x index=%04x len=%04x\n",
          priv->ctrl.type, priv->ctrl.req, value.w, index.w, len.w);

  /* Dispatch any non-standard requests */

  if ((priv->ctrl.type & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_STANDARD)
    {
      usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_NOSTDREQ), priv->ctrl.type);

      /* Let the class implementation handle all non-standar requests */

      pic32mx_dispatchrequest(priv);
      dispatched = true;
      goto resume_packet_processing;  /* Sorry about the goto */
    }

  /* Handle standard request.  Pick off the things of interest to the
   * USB device controller driver; pass what is left to the class driver
   */

  switch (priv->ctrl.req)
    {
    case USB_REQ_GETSTATUS:
      {
        /* type:  device-to-host; recipient = device, interface, endpoint
         * value: 0
         * index: zero interface endpoint
         * len:   2; data = status
         */

        usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_GETSTATUS), priv->ctrl.type);
        if (len.w != 2 || (priv->ctrl.type & USB_REQ_DIR_IN) == 0 ||
            index.b[MSB] != 0 || value.w != 0)
          {
            usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_BADEPGETSTATUS), 0);
            priv->ctrlstate = CTRLSTATE_STALLED;
          }
        else
          {
            switch (priv->ctrl.type & USB_REQ_RECIPIENT_MASK)
              {
               case USB_REQ_RECIPIENT_ENDPOINT:
                {
                  epno = USB_EPNO(index.b[LSB]);
                  usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_EPGETSTATUS), epno);
                  if (epno >= PIC32MX_NENDPOINTS)
                    {
                      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_BADEPGETSTATUS), epno);
                      priv->ctrlstate = CTRLSTATE_STALLED;
                    }
                  else
                    {
                      privep            = &priv->eplist[epno];
                      response.w        = 0; /* Not stalled */
                      nbytes            = 2; /* Response size: 2 bytes */

                      if (USB_ISEPIN(index.b[LSB]))
                        {
                          /* IN endpoint */

                          bdt = privep->bdtin;
                        }
                      else
                        {
                          /* OUT endpoint */

                          bdt = privep->bdtout;
                        }

                      /* BSTALL set if stalled */

                      if ((bdt->status & USB_BDT_BSTALL) != 0)
                        {
                          response.b[LSB] = 1; /* Stalled, set bit 0 */
                        }
                    }
                }
                break;

              case USB_REQ_RECIPIENT_DEVICE:
                {
                 if (index.w == 0)
                    {
                      usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_DEVGETSTATUS), 0);

                      /* Features:  Remote Wakeup=YES; selfpowered=? */

                      response.w      = 0;
                      response.b[LSB] = (priv->selfpowered << USB_FEATURE_SELFPOWERED) |
                                        (priv->rwakeup << USB_FEATURE_REMOTEWAKEUP);
                      nbytes          = 2; /* Response size: 2 bytes */
                    }
                  else
                    {
                      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_BADDEVGETSTATUS), 0);
                      priv->ctrlstate = CTRLSTATE_STALLED;
                    }
                }
                break;

              case USB_REQ_RECIPIENT_INTERFACE:
                {
                  usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_IFGETSTATUS), 0);
                  response.w        = 0;
                  nbytes            = 2; /* Response size: 2 bytes */
                }
                break;

              default:
                {
                  usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_BADGETSTATUS), 0);
                  priv->ctrlstate = CTRLSTATE_STALLED;
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

        usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_CLEARFEATURE), priv->ctrl.type);
        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) != USB_REQ_RECIPIENT_DEVICE)
          {
            /* Disable B device from performing HNP */

#ifdef CONFIG_USBOTG
            if (value.w == USBOTG_FEATURE_B_HNP_ENABLE)
              {
                /* Disable HNP */
#warning Missing Logic
              }

            /* Disable A device HNP support */

            else if (value.w == USBOTG_FEATURE_A_HNP_SUPPORT)
              {
                /* Disable HNP support*/
#warning Missing Logic
              }

            /* Disable alternate HNP support */

            else if (value.w == USBOTG_FEATURE_A_ALT_HNP_SUPPORT)
              {
                /* Disable alternate HNP */
#warning Missing Logic
              }
            else
#endif
            /* Disable remote wakeup */

            if (value.w == USB_FEATURE_REMOTEWAKEUP)
              {
                priv->rwakeup     = 0;
              }
            else
              {
                /* Let the class implementation handle all other device features */

                pic32mx_dispatchrequest(priv);
                dispatched = true;
              }
          }
        else if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_ENDPOINT)
          {
            epno = USB_EPNO(index.b[LSB]);
            if (epno > 0 && epno < PIC32MX_NENDPOINTS && index.b[MSB] == 0 &&
                value.w == USB_FEATURE_ENDPOINTHALT && len.w == 0)
              {
                privep            = &priv->eplist[epno];
                privep->halted    = 0;
                ret               = pic32mx_epstall(&privep->ep, true);
              }
            else
              {
                usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_BADCLEARFEATURE), 0);
                priv->ctrlstate = CTRLSTATE_STALLED;
              }
          }
        else
          {
            /* Let the class implementation handle all other recipients (except for the
             * endpoint recipient)
             */

            pic32mx_dispatchrequest(priv);
            dispatched = true;
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

        usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_SETFEATURE), priv->ctrl.type);

        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE)
          {
            /* Enable B device to perform HNP */

#ifdef CONFIG_USBOTG
            if (value.w == USBOTG_FEATURE_B_HNP_ENABLE)
              {
                /* Enable HNP */
#warning "Missing logic"
              }

            /* Enable A device HNP supports */

            else if (value.w == USBOTG_FEATURE_A_HNP_SUPPORT)
              {
                /* Enable HNP support */
#warning "Missing logic"
              }

            /* Another port on the A device supports HNP */

            else if (value.w == USBOTG_FEATURE_A_ALT_HNP_SUPPORT)
              {
                /* Enable alternate HNP */
#warning "Missing logic"
              }
            else
#endif

            if (value.w == USB_FEATURE_REMOTEWAKEUP)
              {
                priv->rwakeup     = 0;
              }
            else if (value.w == USB_FEATURE_TESTMODE)
              {
                /* Special case recipient=device test mode */

                ullvdbg("test mode: %d\n", index.w);
              }
            else
              {
                /* Let the class implementation handle all other device features */

                pic32mx_dispatchrequest(priv);
                dispatched = true;
              }
          }
        else if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_ENDPOINT)
          {
            /* Handler recipient=endpoint */

            epno = USB_EPNO(index.b[LSB]);
            if (epno < PIC32MX_NENDPOINTS && index.b[MSB] == 0 &&
                value.w == USB_FEATURE_ENDPOINTHALT && len.w == 0)
              {
                privep            = &priv->eplist[epno];
                privep->halted    = 1;
                ret               = pic32mx_epstall(&privep->ep, false);
              }
            else
              {
                usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_BADSETFEATURE), 0);
                priv->ctrlstate = CTRLSTATE_STALLED;
              }
          }
        else
          {
            /* The class driver handles all recipients except recipient=endpoint */

            pic32mx_dispatchrequest(priv);
            dispatched = true;
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

        usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_EP0SETUPSETADDRESS), value.w);
        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) != USB_REQ_RECIPIENT_DEVICE ||
            index.w != 0 || len.w != 0 || value.w > 127)
          {
            usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_BADSETADDRESS), 0);
            priv->ctrlstate = CTRLSTATE_STALLED;
          }
        else
          {
            /* Note that setting of the device address will be deferred.  A zero-length
             * packet will be sent and the device address will be set when the zero-
             * length packet transfer completes.
             */

            priv->devstate = DEVSTATE_ADDRPENDING;
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
        usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_GETSETDESC), priv->ctrl.type);
        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE)
          {
            /* The request seems valid... let the class implementation handle it */

            pic32mx_dispatchrequest(priv);
            dispatched = true;
          }
        else
          {
            usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_BADGETSETDESC), 0);
            priv->ctrlstate = CTRLSTATE_STALLED;
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
        usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_GETCONFIG), priv->ctrl.type);
        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE &&
            value.w == 0 && index.w == 0 && len.w == 1)
          {
            /* The request seems valid... let the class implementation handle it */

            pic32mx_dispatchrequest(priv);
            dispatched = true;
          }
        else
          {
            usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_BADGETCONFIG), 0);
            priv->ctrlstate = CTRLSTATE_STALLED;
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
        usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_SETCONFIG), priv->ctrl.type);
        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE &&
            index.w == 0 && len.w == 0)
          {
             /* The request seems valid... let the class implementation handle it */

             pic32mx_dispatchrequest(priv);
             dispatched = true;
          }
        else
          {
            usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_BADSETCONFIG), 0);
            priv->ctrlstate = CTRLSTATE_STALLED;
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
        /* Let the class implementation handle the request */

        usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_GETSETIF), priv->ctrl.type);
        pic32mx_dispatchrequest(priv);
        dispatched = true;
      }
      break;

    case USB_REQ_SYNCHFRAME:
      /* type:  device-to-host; recipient = endpoint
       * value: 0
       * index: endpoint;
       * len:   2; data = frame number
       */

      {
        usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_SYNCHFRAME), 0);
      }
      break;

    default:
      {
        usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_INVALIDCTRLREQ), priv->ctrl.req);
        priv->ctrlstate = CTRLSTATE_STALLED;
      }
      break;
    }

  /* PKTDIS bit is set when a Setup Transaction is received. Clear to resume
   * packet processing.
   */

resume_packet_processing:
  regval = pic32mx_getreg(PIC32MX_USB_CON);
  regval &= ~USB_CON_PKTDIS;
  pic32mx_putreg(regval, PIC32MX_USB_CON);

  /* At this point, the request has been handled and there are three possible
   * outcomes:
   *
   * 1. The setup request was successfully handled above and a response packet
   *    must be sent (may be a zero length packet).
   * 2. The request was successfully handled by the class implementation.  In
   *    case, the EP0 IN response has already been queued and the local variable
   *    'dispatched' will be set to true and ctrlstate != CTRLSTATE_STALLED;
   * 3. An error was detected in either the above logic or by the class implementation
   *    logic.  In either case, priv->state will be set CTRLSTATE_STALLED
   *    to indicate this case.
   *
   * NOTE: Non-standard requests are a special case.  They are handled by the
   * class implementation and this function returned early above, skipping this
   * logic altogether.
   */

  if (!dispatched && (priv->ctrlstate != CTRLSTATE_STALLED))
    {
      /* The SETUP command was not dispatched to the class driver and the SETUP
       * command did not cause a stall. We will respond.  First, restrict the
       * data length to the length requested in the setup packet
       */

      if (nbytes > len.w)
        {
          nbytes = len.w;
        }

      /* Send the EP0 SETUP response (might be a zero-length packet) */

      pic32mx_epwrite(&priv->eplist[EP0], response.b, nbytes);
      priv->ctrlstate = CTRLSTATE_WAITSETUP;
    }

  /* Did we stall?  This might have occurred from the above logic OR the stall
   * condition may have been set less obviously in pic32mx_dispatchrequest().
   * In either case, we handle the stall condition the same.
   */

  if (priv->ctrlstate != CTRLSTATE_STALLED)
    {
      /* No.. Set up the BDTs to accept the next setup commend. */

      pic32mx_ep0nextsetup(priv);
    }
  else
    {
      /* Stall EP0 */

      (void)pic32mx_epstall(&ep0->ep, false);
    }
}

/****************************************************************************
 * Name: pic32mx_ep0incomplete
 ****************************************************************************/

static void pic32mx_ep0incomplete(struct pic32mx_usbdev_s *priv)
{
  struct pic32mx_ep_s *ep0 = &priv->eplist[EP0];
  volatile struct usbotg_bdtentry_s *bdtlast;
  volatile struct usbotg_bdtentry_s *bdtnext;
  uint32_t data01;
  int ret;

  /* An EP0 OUT transfer has just completed */

  ep0->txbusy = false;

  /* Get the last IN BDT and its data toggle */

  bdtlast     = ep0->bdtin;
  data01      = (bdtlast->status & USB_BDT_DATA01);

  /* Get the next BDT */

  if (bdtlast == &g_bdt[EP0_IN_EVEN])
    {
      bdtnext = &g_bdt[EP0_IN_ODD];
    }
  else
    {
      DEBUGASSERT(bdtlast == &g_bdt[EP0_IN_ODD]);
      bdtnext = &g_bdt[EP0_IN_EVEN];
    }

  /* Make sure that we own the last BDT. */

  bdtlast->status  = (USB_BDT_DATA0 | USB_BDT_DTS | USB_BDT_BSTALL);
  bdtlast->addr    = 0;

  /* Setup the next BDT with the same data toggle (We own this one too) */

  bdtnext->status = (data01 | USB_BDT_DTS | USB_BDT_BSTALL);
  bdtnext->addr   = 0;

  /* Save the next BDT */

  ep0->bdtin = bdtnext;

  /* Are we processing the completion of one packet of an outgoing request
   * from the class driver?
   */

  if (priv->ctrlstate == CTRLSTATE_WRREQUEST)
    {
      /* An outgoing EP0 transfer has completed.  Update the byte count and
       * check for the completion of the transfer.
       */

      pic32mx_wrcomplete(priv, &priv->eplist[EP0]);

      /* Handle the next queue IN transfer.  If there are no further IN
       * transfers, pic32mx_wrrequest will set ctrlstate = CTRLSTATE_WAITSETUP
       */

      ret = pic32mx_wrrequest(priv, &priv->eplist[EP0]);
      if (ret == OK)
        {
          /* Is there another IN transfer in-flight? */

          if (priv->ctrlstate == CTRLSTATE_WAITSETUP)
            {
              /* Set DATA1 to one in the next BDT because the first thing
               * we will do when transmitting is toggle the bit 
               */

              bdtnext->status &= ~USB_BDT_DATA01;
            }
        }
    }

  /* No.. Are we processing the completion of a status response? */

  else if (priv->ctrlstate == CTRLSTATE_WAITSETUP)
    {
      /* Look at the saved SETUP command.  Was it a SET ADDRESS request?
       * If so, then now is the time to set the address.
       */

      if (priv->devstate == DEVSTATE_ADDRPENDING)
        {
          uint16_t addr = GETUINT16(priv->ctrl.value);
          usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_EP0ADDRESSSET), addr);

          /* This should be the equivalent state */

          DEBUGASSERT(priv->ctrl.req == USB_REQ_SETADDRESS &&
                     (priv->ctrl.type & REQRECIPIENT_MASK) ==
                     (USB_REQ_TYPE_STANDARD | USB_REQ_RECIPIENT_DEVICE));

          /* Set (or clear) the address */

          pic32mx_putreg(addr, PIC32MX_USB_ADDR);
          if (addr > 0)
            {
              priv->devstate = DEVSTATE_ADDRESS;
            }
          else
            {
              priv->devstate = DEVSTATE_DEFAULT;
            }
        }

      /* Clear DATA1 to one in the next BDT because the first thing we will
       * do when transmitting is toggle the bit 
       */

      bdtnext->status &= ~USB_BDT_DATA01;
    }

  /* No other state is expected in this context */

  else
    {
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_INVALIDSTATE), priv->ctrlstate);
      priv->ctrlstate = CTRLSTATE_STALLED;
    }
}

/****************************************************************************
 * Name: pic32mx_ep0outcomplete
 ****************************************************************************/

static void pic32mx_ep0outcomplete(struct pic32mx_usbdev_s *priv)
{
  struct pic32mx_ep_s *ep0 = &priv->eplist[EP0];

  switch (priv->ctrlstate)
    {
      case CTRLSTATE_RDREQUEST:  /* Read request in progress */

        /* Process the next read request for EP0 */

        pic32mx_rdcomplete(priv, ep0);

        /* Was this the end of the OUT transfer? */

        if (priv->ctrlstate == CTRLSTATE_WAITSETUP)
          {
            /* Prepare EP0 OUT for the next SETUP transaction. */

            pic32mx_ep0nextsetup(priv);
          }
        break;

      case CTRLSTATE_WAITSETUP:      /* No transfer in progress, waiting for SETUP */
        {
          /* In this case the last OUT transaction must have been a status
           * stage of a CTRLSTATE_WRREQUEST: Prepare EP0 OUT for the next SETUP
           * transaction.
           */

           pic32mx_ep0nextsetup(priv);
        }
        break;

      default:
        {
          /* Unexpected state OR host aborted the OUT transfer before it
           * completed, STALL the endpoint in either case
           */

          usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_INVALIDSTATE), priv->ctrlstate);
          priv->ctrlstate = CTRLSTATE_STALLED;
        }
        break;
    }
}

/****************************************************************************
 * Name: pic32mx_ep0transfer
 ****************************************************************************/

static void pic32mx_ep0transfer(struct pic32mx_usbdev_s *priv, uint16_t status)
{
  volatile struct usbotg_bdtentry_s *bdt;

  /* The following information is available in the status register :
   *
   * ENDPT - The 4 bit endpoint number that cause the interrupt.
   * DIR   - The direction of the endpoint.
   * PPBI  - The ping-pong buffer used in the transaction.
   */

  priv->ep0done = 0;

  /* Check if the last transaction was an EP0 OUT transaction */

  if ((status & USB_STAT_DIR) == USB_STAT_DIR_OUT)
    {
      int index;

      /* It was an EP0 OUT transaction.  Get the index to the BDT. */

      index = ((status & USB_STAT_PPBI) == 0 ? EP0_OUT_EVEN : EP0_OUT_ODD);
      bdt = &g_bdt[index];
      priv->eplist[0].bdtout = bdt;

      bdtdbg("EP0 BDT OUT [%p] {%08x, %08x}\n", bdt, bdt->status, bdt->addr);

      /* Check the current EP0 OUT buffer contains a SETUP packet */

      if (((bdt->status & USB_BDT_PID_MASK) >> USB_BDT_PID_SHIFT) == USB_PID_SETUP_TOKEN)
        {
          /* Check if the SETUP transaction data went into the priv->ctrl
           * buffer. If not, then we will need to copy it.
           */

          if (bdt->addr != (uint8_t *)PHYS_ADDR(&priv->ctrl))
            {
              void *src = (void *)VIRT_ADDR(bdt->addr);
              void *dest = &priv->ctrl;

              memcpy(dest, src, USB_SIZEOF_CTRLREQ);
              bdt->addr = (uint8_t *)PHYS_ADDR(&priv->ctrl);
            }

          /* Handle the control OUT transfer */

          usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_EP0SETUPDONE), bdt->status);
          pic32mx_ep0setup(priv);
        }
      else
        {
          /* Handle the data OUT transfer */

          usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_EP0OUTDONE), status);
          pic32mx_ep0outcomplete(priv);
        }
    }

  /* No.. it was an EP0 IN transfer */

  else /* if ((status & USB_STAT_DIR) == USB_STAT_DIR_IN) */
    {
      usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_EP0INDONE), status);

      /* Handle the IN transfer complete */

      pic32mx_ep0incomplete(priv);
    }

  /* Check for a stall */

  if (priv->ctrlstate == CTRLSTATE_STALLED)
    {
      /* Stall EP0 */

      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_EP0SETUPSTALLED), priv->ctrlstate);
      (void)pic32mx_epstall(&priv->eplist[EP0].ep, false);
    }
}

/****************************************************************************
 * Name: pic32mx_interrupt
 ****************************************************************************/

static int pic32mx_interrupt(int irq, void *context)
{
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple USB controllers
   * easier.
   */

  struct pic32mx_usbdev_s *priv = &g_usbdev;
  uint16_t usbir;
  uint16_t otgir;
  uint16_t regval;
  int i;

  /* Get the set of pending USB and OTG interrupts interrupts */

  usbir = pic32mx_getreg(PIC32MX_USB_IR) & pic32mx_getreg(PIC32MX_USB_IE);
  otgir = pic32mx_getreg(PIC32MX_USBOTG_IR) & pic32mx_getreg(PIC32MX_USBOTG_IE);
  usbtrace(TRACE_INTENTRY(PIC32MX_TRACEINTID_INTERRUPT), usbir | otgir);

#ifdef CONFIG_USBOTG
  /* Session Request Protocol (SRP) Time Out Check */

  /* if USB OTG SRP is ready */
#  warning "Missing logic"
    {
      /* Check if the 1 millisecond timer has expired */

      if (otgir & OTG_INT_T1MSEC) != 0)
        {
          usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_T1MSEC), otgir);

          /* Check for the USB OTG SRP timeout */
#  warning "Missing logic"
            {
              /* Handle OTG events of the SRP timeout has expired */
#  warning "Missing logic"
            }

            /* Clear Interrupt 1 msec timer Flag */

            pic32mx_putreg(USBOTG_INT_T1MSEC, PIC32MX_USBOTG_IR);
        }
    }
#endif

  /* Handle events while we are in the attached state */

  if (priv->devstate == DEVSTATE_ATTACHED)
    {
      /* Clear all USB interrupts */

      pic32mx_putreg(USB_INT_ALL, PIC32MX_USB_IR);

      /* Make sure that the USE reset and IDLE detect interrupts are enabled */

      regval = pic32mx_getreg(PIC32MX_USB_IE);
      regval |= (USB_INT_URST|USB_INT_IDLE);
      pic32mx_putreg(regval, PIC32MX_USB_IE);

      /* Now were are in the powered state */

      priv->devstate = DEVSTATE_POWERED;
    }

#ifdef  CONFIG_USBOTG
  /* Check if the ID Pin Changed State */

  if (otgir & USBOTG_INT_ID) != 0)
    {
      usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_OTGID), otgir);

      /* Re-detect and re-initialize */
#warning "Missing logic"

      pic32mx_putreg(USBOTG_INT_ID, PIC32MX_USBOTG_IR);
    }
#endif

  /* Service the USB Activity Interrupt */

  if ((otgir & USBOTG_INT_ACTV) != 0)
    {
      usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_WKUP), otgir);

      /* Wake-up from susepnd mode */

      pic32mx_putreg(USBOTG_INT_ACTV, PIC32MX_USBOTG_IR);
      pic32mx_resume(priv);
    }

  /*  It is pointless to continue servicing if the device is in suspend mode. */

  if ((pic32mx_getreg(PIC32MX_USB_PWRC) & USB_PWRC_USUSPEND) != 0)
    {
      /* Just clear the interrupt and return */

      usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_SUSPENDED), pic32mx_getreg(PIC32MX_USB_PWRC));
      goto interrupt_exit;
    }

  /* Service USB Bus Reset Interrupt. When bus reset is received during
   * suspend, ACTVIF will be set first, once the UCONbits.SUSPND is clear,
   * then the URSTIF bit will be asserted. This is why URSTIF is checked
   * after ACTVIF.  The USB reset flag is masked when the USB state is in
   * DEVSTATE_DETACHED or DEVSTATE_ATTACHED, and therefore cannot cause a
   * USB reset event during these two states.
   */

  if ((usbir & USB_INT_URST) != 0)
    {
      usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_RESET), usbir);

      /* Reset interrupt received. Restore our power-up state */

#warning "Broken Logic"
#if 0 /* If there is no pull-up, then this will just cause another reset */
      pic32mx_reset(priv);
      pic32mx_attach(priv);
#endif
      priv->devstate = DEVSTATE_DEFAULT;

#ifdef CONFIG_USBOTG
        /* Disable and deactivate HNP */
#warning Missing Logic
#endif
        pic32mx_putreg(USB_INT_URST, PIC32MX_USB_IR);
        goto interrupt_exit;
    }

  /* Service IDLE interrupts */

  if ((usbir & USB_INT_IDLE) != 0)
    {
      usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_IDLE), usbir);

#ifdef  CONFIG_USBOTG
      /* If Suspended, Try to switch to Host */
#warning "Missing logic"
#else
      pic32mx_suspend(priv);

#endif
      pic32mx_putreg(USB_INT_IDLE, PIC32MX_USB_IR);
    }

  /* Service SOF interrupts */

#ifdef CONFIG_USB_SOFINTS
  if ((usbir & USB_INT_SOF) != 0)
    {
      usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_SOF), 0);

      /* I am not sure why you would ever enable SOF interrupts */

      pic32mx_putreg(USB_INT_SOF, PIC32MX_USB_IR);
    }
#endif

   /* Service stall interrupts */

   if ((usbir & USB_INT_STALL) != 0)
    {
      usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_STALL), usbir);

      pic32mx_ep0stall(priv);

      /* Clear the pending STALL interrupt */

      pic32mx_putreg(USB_INT_STALL, PIC32MX_USB_IR);
    }

  /* Service error interrupts */

  if ((usbir & USB_INT_UERR) != 0)
    {
      usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_UERR), usbir);
      ulldbg("Error: EIR=%04x\n", pic32mx_getreg(PIC32MX_USB_EIR));

      /* Clear all pending USB error interrupts */

      pic32mx_putreg(USB_EINT_ALL, PIC32MX_USB_EIR);
    }

  /* There is no point in continuing if the host has not sent a bus reset.
   * Once bus reset is received, the device transitions into the DEFAULT
   * state and is ready for communication.
   */

  if (priv->devstate < DEVSTATE_DEFAULT)
    {
      /* Just clear the interrupt and return */

      usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_WAITRESET), priv->devstate);
      goto interrupt_exit;
    }

  /*  Service USB Transaction Complete Interrupt */

  if ((usbir & USB_INT_TRN) != 0)
    {
      usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_TRNC), usbir);

      /* Drain the USAT FIFO entries.  If the USB FIFO ever gets full, USB
       * bandwidth utilization can be compromised, and the device won't be
       * able to receive SETUP packets.
       */

      for (i = 0; i < 4; i++)
        {
          uint8_t epno;

          /* Check the pending interrupt register.  Is token processing complete. */

          if ((pic32mx_getreg(PIC32MX_USB_IR) & USB_INT_TRN) != 0)
            {
              regval = pic32mx_getreg(PIC32MX_USB_STAT);
              pic32mx_putreg(USB_INT_TRN, PIC32MX_USB_IR);

              usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_TRNCS), regval);

              /* Handle the endpoint tranfer complete event. */

              epno = (regval & USB_STAT_ENDPT_MASK) >> USB_STAT_ENDPT_SHIFT;
              if (epno == 0)
                {
                   pic32mx_ep0transfer(priv, regval);
                }
              else
                {
                  pic32mx_eptransfer(priv, epno, regval);
                }
            }
          else
            {
               /* USTAT FIFO must be empty. */

               break;
            }
        }
    }

  /* Clear the pending USB interrupt.  Goto is used in the above to assure
   * that all interrupt exists pass through this logic.
   */

interrupt_exit:
  up_clrpend_irq(PIC32MX_IRQSRC_USB);
  usbtrace(TRACE_INTEXIT(PIC32MX_TRACEINTID_INTERRUPT), usbir | otgir);
  return OK;
}

/****************************************************************************
 * Suspend/Resume Helpers
 ****************************************************************************/
/****************************************************************************
 * Name: pic32mx_suspend
 ****************************************************************************/

static void pic32mx_suspend(struct pic32mx_usbdev_s *priv)
{
  uint16_t regval;

  /* Enable the ACTV interrupt.
   *
   * NOTE: Do not clear UIRbits.ACTVIF here! Reason: ACTVIF is only
   * generated once an IDLEIF has been generated. This is a 1:1 ratio
   * interrupt generation. For every IDLEIF, there will be only one ACTVIF
   * regardless of the number of subsequent bus transitions.  If the ACTIF
   * is cleared here, a problem could occur. The driver services IDLEIF
   * first because ACTIVIE=0. If this routine clears the only ACTIVIF,
   * then it can never get out of the suspend mode.
   */

  regval  = pic32mx_getreg(PIC32MX_USBOTG_IE);
  regval |= USBOTG_INT_ACTV;
  pic32mx_putreg(regval, PIC32MX_USBOTG_IE);

  /* Disable further IDLE interrupts.  Once is enough. */

  regval  = pic32mx_getreg(PIC32MX_USB_IE);
  regval &= ~USB_INT_IDLE;
  pic32mx_putreg(regval, PIC32MX_USB_IE);

  /* Invoke a callback into board-specific logic.  The board-specific logic
   * may enter into sleep or idle modes or switch to a slower clock, etc.
   */

  pic32mx_usbsuspend((struct usbdev_s *)priv, false);
}

/****************************************************************************
 * Name: pic32mx_resume
 ****************************************************************************/

static void pic32mx_resume(struct pic32mx_usbdev_s *priv)
{
  irqstate_t flags;
  uint16_t regval;

  flags = irqsave();

  /* Start RESUME signaling */

  regval = pic32mx_getreg(PIC32MX_USB_CON);
  regval |= USB_CON_RESUME;
  pic32mx_putreg(regval, PIC32MX_USB_CON);

  /* Keep the RESUME line set for 1-13 ms */

  up_mdelay(10);

  regval &= ~USB_CON_RESUME;
  pic32mx_putreg(regval, PIC32MX_USB_CON);

  /* This function is called when the USB activity interrupt occurs.
   * If using clock switching, this is the place to call out to
   * logic to restore the original MCU core clock frequency.
   */

  pic32mx_usbsuspend((struct usbdev_s *)priv, true);

  /* Disable further activity interrupts */

  regval  = pic32mx_getreg(PIC32MX_USBOTG_IE);
  regval &= ~USBOTG_INT_ACTV;
  pic32mx_putreg(regval, PIC32MX_USBOTG_IE);

  /* The ACTVIF bit cannot be cleared immediately after the USB module wakes
   * up from Suspend or while the USB module is suspended. A few clock cycles
   * are required to synchronize the internal hardware state machine before
   * the ACTIVIF bit can be cleared by firmware. Clearing the ACTVIF bit
   * before the internal hardware is synchronized may not have an effect on
   * the value of ACTVIF. Additionally, if the USB module uses the clock from
   * the 96 MHz PLL source, then after clearing the SUSPND bit, the USB
   * module may not be immediately operational while waiting for the 96 MHz
   * PLL to lock.
   */

   pic32mx_putreg(USB_INT_IDLE, PIC32MX_USBOTG_IR);
   irqrestore(flags);
}

/****************************************************************************
 * Endpoint Helpers
 ****************************************************************************/
/****************************************************************************
 * Name: pic32mx_epreserve
 ****************************************************************************/

static inline struct pic32mx_ep_s *
pic32mx_epreserve(struct pic32mx_usbdev_s *priv, uint8_t epset)
{
  struct pic32mx_ep_s *privep = NULL;
  irqstate_t flags;
  int epndx = 0;

  flags = irqsave();
  epset &= priv->epavail;
  if (epset)
    {
      /* Select the lowest bit in the set of matching, available endpoints
       * (skipping EP0)
       */

      for (epndx = 1; epndx < PIC32MX_NENDPOINTS; epndx++)
        {
          uint8_t bit = PIC32MX_ENDP_BIT(epndx);
          if ((epset & bit) != 0)
            {
              /* Mark the endpoint no longer available */

              priv->epavail &= ~bit;

              /* And return the pointer to the standard endpoint structure */

              privep = &priv->eplist[epndx];
              break;
            }
        }
    }

  irqrestore(flags);
  return privep;
}

/****************************************************************************
 * Name: pic32mx_epunreserve
 ****************************************************************************/

static inline void
pic32mx_epunreserve(struct pic32mx_usbdev_s *priv, struct pic32mx_ep_s *privep)
{
  irqstate_t flags = irqsave();
  priv->epavail   |= PIC32MX_ENDP_BIT(USB_EPNO(privep->ep.eplog));
  irqrestore(flags);
}

/****************************************************************************
 * Name: pic32mx_epreserved
 ****************************************************************************/

static inline bool
pic32mx_epreserved(struct pic32mx_usbdev_s *priv, int epno)
{
  return ((priv->epavail & PIC32MX_ENDP_BIT(epno)) == 0);
}

/****************************************************************************
 * Name: pic32mx_ep0configure
 ****************************************************************************/

static void pic32mx_ep0configure(struct pic32mx_usbdev_s *priv)
{
  volatile struct usbotg_bdtentry_s *bdt;
  uint32_t bytecount;

  /* Enable the EP0 endpoint */

  pic32mx_putreg(PIC32MX_EP_CONTROL, PIC32MX_USB_EP0);

  /* Configure the OUT BDTs.  We assume that the ping-poing buffer index has
   * just been reset and we expect to receive on the EVEN BDT first.  Data
   * toggle synchronization is not needed for SETUP packets.
   */

  bytecount                = (USB_SIZEOF_CTRLREQ << USB_BDT_BYTECOUNT_SHIFT);

  bdt                      = &g_bdt[EP0_OUT_EVEN];
  bdt->addr                = (uint8_t *)PHYS_ADDR(&priv->ctrl);
  bdt->status              = (USB_BDT_UOWN | bytecount);
  priv->eplist[EP0].bdtout = bdt;

  bdt                      = &g_bdt[EP0_OUT_ODD];
  bdt->status              = (USB_BDT_BSTALL | bytecount);
  bdt->addr                = (uint8_t *)PHYS_ADDR(&priv->ctrl);

  /* Configure the IN BDTs.  Set DATA0 in the EVEN BDT because the first
   * thing we will do when transmitting is toggle the bit
   */

  bdt                      = &g_bdt[EP0_IN_EVEN];
  bdt->status              = (USB_BDT_DATA0 | USB_BDT_DTS | USB_BDT_BSTALL);
  bdt->addr                = 0;
  priv->eplist[EP0].bdtin  = bdt;

  bdt                      = &g_bdt[EP0_IN_ODD];
  bdt->status              = (USB_BDT_DATA0 | USB_BDT_DTS | USB_BDT_BSTALL);
  bdt->addr                = 0;
}

/****************************************************************************
 * Name: pic32mx_ep0bdtdump
 ****************************************************************************/

#ifdef CONFIG_PIC32MX_USBDEV_BDTDEBUG
static void pic32mx_ep0bdtdump(const char *msg)
{
  volatile struct usbotg_bdtentry_s *bdt;

  bdtdbg("EP0 BDT: %s\n", msg);

  bdt = &g_bdt[EP0_OUT_EVEN] ;
  bdtdbg("  OUT EVEN [%p] {%08x, %08x}\n", bdt, bdt->status, bdt->addr);
  bdt = &g_bdt[EP0_OUT_ODD];
  bdtdbg("  OUT ODD  [%p] {%08x, %08x}\n", bdt, bdt->status, bdt->addr);
  bdt = &g_bdt[EP0_IN_EVEN];
  bdtdbg("  IN EVEN  [%p] {%08x, %08x}\n", bdt, bdt->status, bdt->addr);
  bdt = &g_bdt[EP0_IN_ODD];
  bdtdbg("  IN ODD   [%p] {%08x, %08x}\n", bdt, bdt->status, bdt->addr);
}
#endif

/****************************************************************************
 * Endpoint operations
 ****************************************************************************/
/****************************************************************************
 * Name: pic32mx_epconfigure
 ****************************************************************************/

static int pic32mx_epconfigure(struct usbdev_ep_s *ep,
                               const struct usb_epdesc_s *desc,
                               bool last)
{
  struct pic32mx_ep_s *privep = (struct pic32mx_ep_s *)ep;
  volatile struct usbotg_bdtentry_s *bdt;
  uint16_t maxpacket;
  uint16_t regval;
  uint16_t status;
  uint8_t  epno;
  bool     epin;
  bool     bidi;
  int      index;

#ifdef CONFIG_DEBUG
  if (!ep || !desc)
    {
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_INVALIDPARMS), 0);
      ulldbg("ERROR: ep=%p desc=%p\n");
      return -EINVAL;
    }
#endif

  /* Get the unadorned endpoint address */

  epno = USB_EPNO(desc->addr);
  epin = USB_ISEPIN(desc->addr);

  usbtrace(TRACE_EPCONFIGURE, (uint16_t)epno);
  DEBUGASSERT(epno == USB_EPNO(ep->eplog));

  /* Set the requested type */

  switch (desc->attr & USB_EP_ATTR_XFERTYPE_MASK)
   {
    case USB_EP_ATTR_XFER_INT: /* Interrupt endpoint */
      regval = epin ? PIC32MX_EP_INTIN : PIC32MX_EP_INTOUT;
      break;

    case USB_EP_ATTR_XFER_BULK: /* Bulk endpoint */
      regval = epin ? PIC32MX_EP_BULKIN : PIC32MX_EP_BULKOUT;
      break;

    case USB_EP_ATTR_XFER_ISOC: /* Isochronous endpoint */
      regval = epin ? PIC32MX_EP_ISOCIN : PIC32MX_EP_ISOCOUT;
      break;

    case USB_EP_ATTR_XFER_CONTROL: /* Control endpoint */
      regval = PIC32MX_EP_CONTROL;
      bidi   = true;
      break;

    default:
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_BADEPTYPE), (uint16_t)desc->type);
      return -EINVAL;
    }

  /* Enable the endpoint */

  pic32mx_putreg(regval, PIC32MX_USB_EP(epno));

  /* Setup up buffer descriptor table (BDT) entry/ies for this endpoint */

  if (epin || bidi)
    {
      /* Get the pointer to BDT entry */

      index         =  EP(epno, 1, 0);
      bdt           = &g_bdt[index];
      privep->bdtin = bdt;

      /* Mark that we own the entry */

      status        =  bdt->status;
      status       &= ~USB_BDT_UOWN;

      /* Set DATA1 to one because the first thing we will do when transmitting is
       * toggle the bit
       */

      status       |= USB_BDT_DATA1;
      bdt->status   = status;

      bdtdbg("EP%d BDT IN [%p] {%08x, %08x}\n", epno, bdt, status, bdt->addr);

      /* Now do the same for the other buffer.  The only difference is the
       * we clear DATA1 (making it DATA0)
       */

      bdt           = &g_bdt[index+1];
      status        =  bdt->status;
      status       &= ~(USB_BDT_UOWN | USB_BDT_DATA01);
      bdt->status   = status;

      bdtdbg("EP%d BDT IN [%p] {%08x, %08x}\n", epno, bdt, status, bdt->addr);
    }

  if (!epin || bidi)
    {
      index          =  EP(epno, 0, 0);
      bdt            = &g_bdt[index];
      privep->bdtout = bdt;

      /* Mark that we own the entry */

      status        =  bdt->status;
      status       &= ~USB_BDT_UOWN;

      /* Set DATA1 to one because the first thing we will do when transmitting is
       * toggle the bit
       */

      status       |= USB_BDT_DATA1;
      bdt->status   = status;

      bdtdbg("EP%d BDT OUT [%p] {%08x, %08x}\n", epno, bdt, status, bdt->addr);

      /* Now do the same for the other buffer. The only difference is the
       * we clear DATA1 (making it DATA0)
       */

      bdt           = &g_bdt[index+1];
      status        =  bdt->status & ~USB_BDT_UOWN;

      status       &= ~USB_BDT_DATA01;
      bdt->status   = status;

      bdtdbg("EP%d BDT OUT [%p] {%08x, %08x}\n", epno, bdt, status, bdt->addr);
    }

  /* Get the maxpacket size of the endpoint. */

  maxpacket = GETUINT16(desc->mxpacketsize);
  DEBUGASSERT(maxpacket <= PIC32MX_MAXPACKET_SIZE);
  ep->maxpacket = maxpacket;

  /* Set the full, logic EP number (that includes direction encoded in bit 7) */

  if (epin)
    {
      ep->eplog = USB_EPIN(epno);
    }
  else
    {
      ep->eplog = USB_EPOUT(epno);
    }

  return OK;
}

/****************************************************************************
 * Name: pic32mx_epdisable
 ****************************************************************************/

static int pic32mx_epdisable(struct usbdev_ep_s *ep)
{
  struct pic32mx_usbdev_s *priv;
  struct pic32mx_ep_s *privep;
  volatile uint32_t *ptr;
  int epno;
  int i;
  irqstate_t flags;

#ifdef CONFIG_DEBUG
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_INVALIDPARMS), 0);
      ulldbg("ERROR: ep=%p\n", ep);
      return -EINVAL;
    }
#endif

  privep = (struct pic32mx_ep_s *)ep;
  priv   = privep->dev;
  epno   = USB_EPNO(ep->eplog);
  usbtrace(TRACE_EPDISABLE, epno);

  /* Cancel any ongoing activity */

  flags = irqsave();
  pic32mx_cancelrequests(privep);

  /* Disable the endpoint */

  pic32mx_putreg(0, PIC32MX_USB_EP(epno));

  /* Reset the BDTs for the endpoint.  Four BDT entries per endpoint; Two
   * 32-bit words per BDT.
   */

  ptr = (uint32_t*)&g_bdt[EP(epno, 0, 0)];
  for (i = 0; i < USB_BDT_WORD_SIZE * USB_NBDTS_PER_EP; i++)
    {
      *ptr++ = 0;
    }

  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Name: pic32mx_epallocreq
 ****************************************************************************/

static struct usbdev_req_s *pic32mx_epallocreq(struct usbdev_ep_s *ep)
{
  struct pic32mx_req_s *privreq;

#ifdef CONFIG_DEBUG
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_INVALIDPARMS), 0);
      return NULL;
    }
#endif
  usbtrace(TRACE_EPALLOCREQ, USB_EPNO(ep->eplog));

  privreq = (struct pic32mx_req_s *)malloc(sizeof(struct pic32mx_req_s));
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_ALLOCFAIL), 0);
      return NULL;
    }

  memset(privreq, 0, sizeof(struct pic32mx_req_s));
  return &privreq->req;
}

/****************************************************************************
 * Name: pic32mx_epfreereq
 ****************************************************************************/

static void pic32mx_epfreereq(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  struct pic32mx_req_s *privreq = (struct pic32mx_req_s*)req;

#ifdef CONFIG_DEBUG
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_INVALIDPARMS), 0);
      return;
    }
#endif
  usbtrace(TRACE_EPFREEREQ, USB_EPNO(ep->eplog));

  free(privreq);
}

/****************************************************************************
 * Name: pic32mx_epsubmit
 ****************************************************************************/

static int pic32mx_epsubmit(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  struct pic32mx_req_s *privreq = (struct pic32mx_req_s *)req;
  struct pic32mx_ep_s *privep = (struct pic32mx_ep_s *)ep;
  struct pic32mx_usbdev_s *priv;
  irqstate_t flags;
  uint8_t epno;
  int ret = OK;

#ifdef CONFIG_DEBUG
  if (!req || !req->callback || !req->buf || !ep)
    {
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_INVALIDPARMS), 0);
      ulldbg("ERROR: req=%p callback=%p buf=%p ep=%p\n", req, req->callback, req->buf, ep);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPSUBMIT, USB_EPNO(ep->eplog));
  priv = privep->dev;

#ifdef CONFIG_DEBUG
  if (!priv->driver)
    {
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_NOTCONFIGURED), priv->usbdev.speed);
      ulldbg("ERROR: driver=%p\n", priv->driver);
      return -ESHUTDOWN;
    }
#endif

  /* Handle the request from the class driver */

  epno              = USB_EPNO(ep->eplog);
  req->result       = -EINPROGRESS;
  req->xfrd         = 0;
  privreq->inflight = 0;
  flags             = irqsave();

  /* If we are stalled, then drop all requests on the floor */

  if (privep->stalled)
    {
      pic32mx_abortrequest(privep, privreq, -EBUSY);
      ulldbg("ERROR: stalled\n");
      ret = -EBUSY;
    }

  /* Handle IN (device-to-host) requests.  NOTE:  If the class device is
   * using the bi-directional EP0, then we assume that they intend the EP0
   * IN functionality.
   */

  else if (USB_ISEPIN(ep->eplog) || epno == EP0)
    {
      /* Add the new request to the request queue for the IN endpoint */

      pic32mx_rqenqueue(privep, privreq);
      usbtrace(TRACE_INREQQUEUED(epno), req->len);

      /* If the IN endpoint FIFO is available, then transfer the data now */

      if (!privep->txbusy)
        {
          ret = pic32mx_wrrequest(priv, privep);
        }
    }

  /* Handle OUT (host-to-device) requests */

  else
    {
      /* Add the new request to the request queue for the OUT endpoint */

      privep->txnullpkt = 0;
      pic32mx_rqenqueue(privep, privreq);
      usbtrace(TRACE_OUTREQQUEUED(epno), req->len);
    }

  irqrestore(flags);
  return ret;
}

/****************************************************************************
 * Name: pic32mx_epcancel
 ****************************************************************************/

static int pic32mx_epcancel(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  struct pic32mx_ep_s *privep = (struct pic32mx_ep_s *)ep;
  struct pic32mx_usbdev_s *priv;
  irqstate_t flags;

#ifdef CONFIG_DEBUG
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif
  usbtrace(TRACE_EPCANCEL, USB_EPNO(ep->eplog));
  priv = privep->dev;

  flags = irqsave();
  pic32mx_cancelrequests(privep);
  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Name: pic32mx_epbdtstall
 ****************************************************************************/

static int pic32mx_epbdtstall(struct usbdev_ep_s *ep,
                              volatile struct usbotg_bdtentry_s *bdt,
                              bool resume, bool epin)
{
  struct pic32mx_ep_s *privep;
  struct pic32mx_usbdev_s *priv;
  uint32_t regaddr;
  uint16_t regval;
  uint16_t status;
  uint8_t epno;

  /* Recover pointers */

  privep = (struct pic32mx_ep_s *)ep;
  priv   = (struct pic32mx_usbdev_s *)privep->dev;
  epno   = USB_EPNO(ep->eplog);

  /* Handle the resume condition */

  if (resume)
    {
      /* Resuming a stalled endpoint */

      usbtrace(TRACE_EPRESUME, epno);
      privep->stalled = false;

      /* Point to the appropriate EP register */

      regaddr = PIC32MX_USB_EP(epno);

      /* Clear the STALL bit in the UEP register */

      regval = pic32mx_getreg(regaddr);
      regval &= ~USB_EP_EPSTALL;
      pic32mx_putreg(regval, regaddr);

      /* Check for an IN endpoint */

      if (USB_ISEPIN(ep->eplog))
        {
          /* If the endpoint is an IN endpoint then we need to return it
           * to the CPU and reset the DTS bit so that the next transfer
           * is correct
           */

          status  = bdt->status;
          status |= (USB_BDT_UOWN | USB_BDT_DATA1);

          /* Then give the BDT to the USB */

          bdtdbg("EP%d BDT OUT [%p] {%08x, %08x}\n",  epno, bdt, status, bdt->addr);
          bdt->status = status;

          /* Restart any queued write requests */

          pic32mx_wrrestart(priv, privep);
        }
      else
        {
          /* If the endpoint was an OUT endpoint then we need to give
           * control of the endpoint back to the USB so that the
           * function driver can receive the data as they expected.
           * Also need to set the DTS bit so the next packet will be
           * correct.
           */

          status  = bdt->status;
          status |= (USB_BDT_UOWN | USB_BDT_DATA1 | USB_BDT_DTS);

          /* Then give the BDT to the USB */

          bdtdbg("EP%d BDT OUT [%p] {%08x, %08x}\n", epno, bdt, status, bdt->addr);

          bdt->status = status;
        }
    }

  /* Handle the stall condition */

  else
    {
      usbtrace(TRACE_EPSTALL, epno);
      privep->stalled = true;

      /* Then STALL the endpoint */

      status  = bdt->status & ~(USB_BDT_BYTECOUNT_MASK | USB_BDT_DATA01);
      status |= (USB_BDT_UOWN | USB_BDT_DATA0 | USB_BDT_DTS | USB_BDT_BSTALL);

      /* And give the BDT to the USB */

      bdtdbg("EP%d BDT OUT [%p] {%08x, %08x}\n", epno, bdt, status, bdt->addr);

      bdt->status = status;
    }

  return OK;
}

/****************************************************************************
 * Name: pic32mx_epstall
 ****************************************************************************/

static int pic32mx_epstall(struct usbdev_ep_s *ep, bool resume)
{
  struct pic32mx_ep_s *privep;
  irqstate_t flags;
  int ret;

#ifdef CONFIG_DEBUG
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Recover pointers */

  privep = (struct pic32mx_ep_s *)ep;

  /* STALL or RESUME the endpoint */

  flags = irqsave();
  usbtrace(resume ? TRACE_EPRESUME : TRACE_EPSTALL, USB_EPNO(ep->eplog));

  /* Special case EP0.  When we stall EP0 we have to stall both the IN and
   * OUT BDTs.
   */

  if (USB_EPNO(ep->eplog) == 0)
    {
      ret = pic32mx_epbdtstall(ep, privep->bdtin, resume, true);
      if (ret == OK)
        {
          ret = pic32mx_epbdtstall(ep, privep->bdtout, resume, false);
        }
    }

  /* Otherwise, select the BDT for the endpoint direction */

  else if (USB_ISEPIN(ep->eplog))
    {
      /* It is an IN endpoint */

      ret = pic32mx_epbdtstall(ep, privep->bdtin, resume, true);
    }
  else
    {
      /* It is an OUT endpoint */

      ret = pic32mx_epbdtstall(ep, privep->bdtout, resume, false);
    }

  irqrestore(flags);
  return ret;
}

/****************************************************************************
 * Device Controller Operations
 ****************************************************************************/
/****************************************************************************
 * Name: pic32mx_allocep
 ****************************************************************************/

static struct usbdev_ep_s *pic32mx_allocep(struct usbdev_s *dev, uint8_t epno,
                                           bool epin, uint8_t eptype)
{
  struct pic32mx_usbdev_s *priv = (struct pic32mx_usbdev_s *)dev;
  struct pic32mx_ep_s *privep = NULL;
  uint16_t epset = PIC32MX_ENDP_ALLSET;

  usbtrace(TRACE_DEVALLOCEP, (uint16_t)epno);
#ifdef CONFIG_DEBUG
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_INVALIDPARMS), 0);
      return NULL;
    }
#endif

  /* Ignore any direction bits in the logical address */

  epno = USB_EPNO(epno);

  /* A logical address of 0 means that any endpoint will do */

  if (epno > 0)
    {
      /* Otherwise, we will return the endpoint structure only for the requested
       * 'logical' endpoint.  All of the other checks will still be performed.
       *
       * First, verify that the logical endpoint is in the range supported by
       * by the hardware.
       */

      if (epno >= PIC32MX_NENDPOINTS)
        {
          usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_BADEPNO), (uint16_t)epno);
          return NULL;
        }

      /* Convert the logical address to a physical OUT endpoint address and
       * remove all of the candidate endpoints from the bitset except for the
       * the IN/OUT pair for this logical address.
       */

      epset = PIC32MX_ENDP_BIT(epno);
    }

  /* Check if the selected endpoint number is available */

  privep = pic32mx_epreserve(priv, epset);
  if (!privep)
    {
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_EPRESERVE), (uint16_t)epset);
      return NULL;
    }

  return &privep->ep;
}

/****************************************************************************
 * Name: pic32mx_freeep
 ****************************************************************************/

static void pic32mx_freeep(struct usbdev_s *dev, struct usbdev_ep_s *ep)
{
  struct pic32mx_usbdev_s *priv;
  struct pic32mx_ep_s *privep;

#ifdef CONFIG_DEBUG
  if (!dev || !ep)
    {
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_INVALIDPARMS), 0);
      return;
    }
#endif

  priv   = (struct pic32mx_usbdev_s *)dev;
  privep = (struct pic32mx_ep_s *)ep;
  usbtrace(TRACE_DEVFREEEP, (uint16_t)USB_EPNO(ep->eplog));
  DEBUGASSERT(priv && privep);

  /* Disable the endpoint */

  (void)pic32mx_epdisable(ep);

  /* Mark the endpoint as available */

  pic32mx_epunreserve(priv, privep);
}

/****************************************************************************
 * Name: pic32mx_getframe
 ****************************************************************************/

static int pic32mx_getframe(struct usbdev_s *dev)
{
  uint16_t frml;
  uint16_t frmh;
  uint16_t tmp;

#ifdef CONFIG_DEBUG
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Return the last frame number detected by the hardware.  Thr FRMH/L
   * registers are updated with the current frame number whenever a SOF
   * TOKEN is received.
   */

  do
    {
       /* Loop until we can be sure that there was no wrap from the FRML
        * to the FRMH register.
        */

       frmh = pic32mx_getreg(PIC32MX_USB_FRMH) & USB_FRMH_MASK;
       frml = pic32mx_getreg(PIC32MX_USB_FRML) & USB_FRML_MASK;
       tmp  = pic32mx_getreg(PIC32MX_USB_FRMH) & USB_FRMH_MASK;
    }
  while (frmh != tmp);

  /* Combine to for the full 11-bit value */

  tmp = (frmh) << 8 | frml;
  usbtrace(TRACE_DEVGETFRAME, tmp);
  return tmp;
}

/****************************************************************************
 * Name: pic32mx_wakeup
 ****************************************************************************/

static int pic32mx_wakeup(struct usbdev_s *dev)
{
  struct pic32mx_usbdev_s *priv = (struct pic32mx_usbdev_s *)dev;

  usbtrace(TRACE_DEVWAKEUP, 0);
#ifdef CONFIG_DEBUG
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Resume normal operation. */

  pic32mx_resume(priv);
  return OK;
}

/****************************************************************************
 * Name: pic32mx_selfpowered
 ****************************************************************************/

static int pic32mx_selfpowered(struct usbdev_s *dev, bool selfpowered)
{
  struct pic32mx_usbdev_s *priv = (struct pic32mx_usbdev_s *)dev;

  usbtrace(TRACE_DEVSELFPOWERED, (uint16_t)selfpowered);

#ifdef CONFIG_DEBUG
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_INVALIDPARMS), 0);
      return -ENODEV;
    }
#endif

  priv->selfpowered = selfpowered;
  return OK;
}

/****************************************************************************
 * Initialization/Reset
 ****************************************************************************/
/****************************************************************************
 * Name: pic32mx_reset
 ****************************************************************************/

static void pic32mx_reset(struct pic32mx_usbdev_s *priv)
{
  int epno;

  /* Tell the class driver that we are disconnected.  The class driver
   * should then accept any new configurations.
   */

  if (priv->driver)
    {
      CLASS_DISCONNECT(priv->driver, &priv->usbdev);
    }

  /* Reset the device state structure */

  priv->ctrlstate  = CTRLSTATE_WAITSETUP;

  /* Reset endpoints */

  for (epno = 0; epno < PIC32MX_NENDPOINTS; epno++)
    {
      struct pic32mx_ep_s *privep = &priv->eplist[epno];

      /* Cancel any queued requests.  Since they are canceled
       * with status -ESHUTDOWN, then will not be requeued
       * until the configuration is reset.  NOTE:  This should
       * not be necessary... the CLASS_DISCONNECT above should
       * result in the class implementation calling pic32mx_epdisable
       * for each of its configured endpoints.
       */

      pic32mx_cancelrequests(privep);

      /* Reset endpoint status */

      privep->stalled   = false;
      privep->halted    = false;
      privep->txbusy    = false;
      privep->txnullpkt = false;
    }

  /* Re-configure the USB controller in its initial, unconnected state */

  pic32mx_hwreset(priv);
}

/****************************************************************************
 * Name: pic32mx_attach
 ****************************************************************************/

static void pic32mx_attach(struct pic32mx_usbdev_s *priv)
{
  uint16_t regval;

  /* Check if we are in the detached state */

  if (priv->devstate == DEVSTATE_DETACHED)
    {
      /* Disable USB interrupts at the interrupt controller */

      up_disable_irq(PIC32MX_IRQSRC_USB);

      /* Initialize registers to known states. */

      pic32mx_putreg(0, PIC32MX_USB_CON);

      /* Configure things like: pull ups, full/low-speed mode,
       * set the ping pong mode, and set internal transceiver
       */

      pic32mx_putreg(0, PIC32MX_USB_CNFG1);

      /* Enable interrupts at the USB controller */

      pic32mx_putreg(ERROR_INTERRUPTS, PIC32MX_USB_EIE);
      pic32mx_putreg(NORMAL_INTERRUPTS, PIC32MX_USB_IE);

      /* Configure EP0 */

      pic32mx_ep0configure(priv);
      pic32mx_ep0bdtdump("Attached");

      /* Flush any pending transactions */

      while ((pic32mx_getreg(PIC32MX_USB_IR) & USB_INT_TRN) != 0)
        {
          pic32mx_putreg(USB_INT_TRN, PIC32MX_USB_IR);
        }

      /* Make sure packet processing is enabled */

      regval = pic32mx_getreg(PIC32MX_USB_CON);
      regval &= ~USB_CON_PKTDIS;
      pic32mx_putreg(regval, PIC32MX_USB_CON);

      /* Enable the USB module and attach to bus */

      do
        {
          regval = pic32mx_getreg(PIC32MX_USB_CON);
          if ((regval & USB_CON_USBEN) == 0)
            {
              pic32mx_putreg(regval | USB_CON_USBEN, PIC32MX_USB_CON);
            }
        }
      while ((regval & USB_CON_USBEN) == 0);

      /* Enable OTG */

#ifdef CONFIG_USBOTG
      regval  = pic32mx_getreg(PIC32MX_USBOTG_CON);
      regval |= (USBOTG_CON_DPPULUP | USBOTG_CON_OTGEN);
      pic32mx_putreg(regval, PIC32MX_USBOTG_CON);
#endif

      /* Transition to the attached state */

      priv->devstate     = DEVSTATE_ATTACHED;
      priv->usbdev.speed = USB_SPEED_FULL;

      /* Clear all pending USB interrupts */

      pic32mx_putreg(USB_EINT_ALL, PIC32MX_USB_EIR);
      pic32mx_putreg(USB_INT_ALL, PIC32MX_USB_IR);

      /* Enable USB interrupts at the interrupt controller */

      up_enable_irq(PIC32MX_IRQSRC_USB);

      /* Enable pull-up to connect the device.  The host should enumerate us
       * some time after this
       */

      pic32mx_usbpullup(&priv->usbdev, true);
    }
}

/****************************************************************************
 * Name: pic32mx_detach
 ****************************************************************************/

static void pic32mx_detach(struct pic32mx_usbdev_s *priv)
{
  /* Disable USB interrupts at the interrupt controller */

  up_disable_irq(PIC32MX_IRQSRC_USB);

  /* Disable the USB controller and detach from the bus. */

  pic32mx_putreg(0, PIC32MX_USB_CON);

  /* Mask all USB interrupts */

  pic32mx_putreg(0, PIC32MX_USB_IE);

  /* We are now in the detached state  */

  priv->attached = 0;
  priv->devstate = DEVSTATE_DETACHED;

#ifdef CONFIG_USBOTG
  /* Disable the D+ Pullup */

  regval  = pic32mx_getreg(PIC32MX_USBOTG_CON);
  regval &= ~USBOTG_CON_DPPULUP;
  pic32mx_putreg(regval, PIC32MX_USBOTG_CON);

  /* Disable and deactivate HNP */
#warning Missing Logic

  /* Check if the ID Pin Changed State */

  if ((pic32mx_getreg(PIC32MX_USBOTG_IR) & pic32mx_getreg(PIC32MX_USBOTG_IE) & USBOTG_INT_ID) != 0)
    {
      /* Re-detect & Initialize */
#warning "Missing logic"

      /* Clear ID Interrupt Flag */

      pic32mx_putreg(USBOTG_INT_ID, PIC32MX_USBOTG_IR);
    }
#endif
}

/****************************************************************************
 * Name: pic32mx_hwreset
 ****************************************************************************/

static void pic32mx_hwreset(struct pic32mx_usbdev_s *priv)
{
  uint32_t physaddr;
  uint16_t regval;
  int i;

  /* Power down the USB module.  This will reset all USB registers. */

  regval = pic32mx_getreg(PIC32MX_USB_PWRC);
  regval &= ~USB_PWRC_USBPWR;

  /* Clear all of the buffer descriptor table (BDT) entries */

  memset((void*)g_bdt, 0, sizeof(g_bdt));

  /* Power up the USB module */

  regval |= USB_PWRC_USBPWR;
  pic32mx_putreg(regval, PIC32MX_USB_PWRC);

  /* Reset configuration and disable interrrupts at the USB controller */

  pic32mx_putreg(0, PIC32MX_USB_CNFG1);
  pic32mx_putreg(0, PIC32MX_USB_EIE);
  pic32mx_putreg(0, PIC32MX_USB_IE);

  /* Set the address of the buffer descriptor table (BDT)
   *
   * BDTP1: Bit 1-7: Bits 9-15 of the BDT base address
   * BDTP2: Bit 0-7: Bits 16-23 of the BDT base address
   * BDTP3: Bit 0-7: Bits 24-31 of the BDT base address
   */

  physaddr = PHYS_ADDR(g_bdt);
  pic32mx_putreg((uint16_t)((physaddr >> 24) & USB_BDTP3_MASK), PIC32MX_USB_BDTP3);
  pic32mx_putreg((uint16_t)((physaddr >> 16) & USB_BDTP2_MASK), PIC32MX_USB_BDTP2);
  pic32mx_putreg((uint16_t)((physaddr >>  8) & USB_BDTP1_MASK), PIC32MX_USB_BDTP1);

  /* Reset to then default address */

  pic32mx_putreg(0, PIC32MX_USB_ADDR);

  /* Clear all of the endpoint control registers */

  for (i = 0; i < PIC32MX_NENDPOINTS; i++)
    {
      pic32mx_putreg(0, PIC32MX_USB_EP(i));
    }

  /* Assert reset request to all of the Ping Pong buffer pointers.  This
   * will reset all Even/Odd buffer pointers to the EVEN BD banks.
   */

  regval  = pic32mx_getreg(PIC32MX_USB_CON);
  regval |= USB_CON_PPBRST;
  pic32mx_putreg(regval, PIC32MX_USB_CON);

  /* Bring the ping pong buffer pointers out of reset */

  up_mdelay(5);
  regval &= ~USB_CON_PPBRST;
  pic32mx_putreg(regval, PIC32MX_USB_CON);

  /* Indicate that we are now in the detached state  */

  priv->devstate = DEVSTATE_DETACHED;
}

/****************************************************************************
 * Name: pic32mx_stateinit
 ****************************************************************************/

static void pic32mx_stateinit(struct pic32mx_usbdev_s *priv)
{
  int epno;

  /* Disconnect the device / disable the pull-up.  We don't want the
   * host to enumerate us until the class driver is registered.
   */

  pic32mx_usbpullup(&priv->usbdev, false);

  /* Initialize the device state structure.  NOTE: many fields
   * have the initial value of zero and, hence, are not explicitly
   * initialized here.
   */

  memset(priv, 0, sizeof(struct pic32mx_usbdev_s));
  priv->usbdev.ops   = &g_devops;
  priv->usbdev.ep0   = &priv->eplist[EP0].ep;
  priv->epavail      = PIC32MX_ENDP_ALLSET & ~PIC32MX_ENDP_BIT(EP0);
  priv->rwakeup      = 1;

  /* Initialize the endpoint list */

  for (epno = 0; epno < PIC32MX_NENDPOINTS; epno++)
    {
      struct pic32mx_ep_s *privep = &priv->eplist[epno];

      /* Set endpoint operations, reference to driver structure (not
       * really necessary because there is only one controller), and
       * the (physical) endpoint number which is just the index to the
       * endpoint.
       */

      privep->ep.ops           = &g_epops;
      privep->dev              = priv;
      privep->ep.eplog         = epno;

      /* We will use a fixed maxpacket size for all endpoints (perhaps
       * ISOC endpoints could have larger maxpacket???).  A smaller
       * packet size can be selected when the endpoint is configured.
       */

      privep->ep.maxpacket     = PIC32MX_MAXPACKET_SIZE;
    }

  /* Select a smaller endpoint size for EP0 */

#if PIC32MX_EP0MAXPACKET < PIC32MX_MAXPACKET_SIZE
  priv->eplist[EP0].ep.maxpacket = PIC32MX_EP0MAXPACKET;
#endif
}

/****************************************************************************
 * Name: pic32mx_hwshutdown
 ****************************************************************************/

static void pic32mx_hwshutdown(struct pic32mx_usbdev_s *priv)
{
  uint16_t regval;

  /* Put the hardware in its normal, unconnected state */

  pic32mx_reset(priv);
  priv->usbdev.speed = USB_SPEED_UNKNOWN;

  /* Disable all interrupts and force the USB controller into reset */

  pic32mx_putreg(0, PIC32MX_USB_EIE);
  pic32mx_putreg(0, PIC32MX_USB_IE);

  /* Clear any pending interrupts */

  pic32mx_putreg(USB_EINT_ALL, PIC32MX_USB_EIR);
  pic32mx_putreg(USB_INT_ALL, PIC32MX_USB_IR);

  /* Disconnect the device / disable the pull-up */

  pic32mx_usbpullup(&priv->usbdev, false);

  /* Power down the USB controller */

  regval  = pic32mx_getreg(PIC32MX_USB_PWRC);
  regval &= ~USB_PWRC_USBPWR;
  pic32mx_putreg(regval, PIC32MX_USB_PWRC);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: up_usbinitialize
 *
 * Description:
 *   Initialize the USB driver
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_usbinitialize(void)
{
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple USB controllers
   * easier.
   */

  struct pic32mx_usbdev_s *priv = &g_usbdev;

  usbtrace(TRACE_DEVINIT, 0);

  /* Initialize the driver state structure */

  pic32mx_stateinit(priv);

  /* Attach USB controller interrupt handler.  The hardware will not be
   * initialized and interrupts will not be enabled until the class device
   * driver is bound.  Getting the IRQs here only makes sure that we have
   * them when we need them later.
   */

  if (irq_attach(PIC32MX_IRQ_USB, pic32mx_interrupt) != 0)
    {
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_IRQREGISTRATION),
               (uint16_t)PIC32MX_IRQ_USB);
      up_usbuninitialize();
    }
}

/****************************************************************************
 * Name: up_usbuninitialize
 * Description:
 *   Initialize the USB driver
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_usbuninitialize(void)
{
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple USB controllers
   * easier.
   */

  struct pic32mx_usbdev_s *priv = &g_usbdev;
  irqstate_t flags;

  flags = irqsave();
  usbtrace(TRACE_DEVUNINIT, 0);

  /* Disable and detach the USB IRQs */

  up_disable_irq(PIC32MX_IRQSRC_USB);
  irq_detach(PIC32MX_IRQ_USB);

  if (priv->driver)
    {
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_DRIVERREGISTERED), 0);
      usbdev_unregister(priv->driver);
    }

  /* Put the hardware in an inactive state */

  pic32mx_hwshutdown(priv);
  irqrestore(flags);
}

/****************************************************************************
 * Name: usbdev_register
 *
 * Description:
 *   Register a USB device class driver. The class driver's bind() method
 *   will be called to bind it to a USB device driver.
 *
 ****************************************************************************/

int usbdev_register(struct usbdevclass_driver_s *driver)
{
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple USB controllers
   * easier.
   */

  struct pic32mx_usbdev_s *priv = &g_usbdev;
  int ret;

  usbtrace(TRACE_DEVREGISTER, 0);

#ifdef CONFIG_DEBUG
  if (!driver || !driver->ops->bind || !driver->ops->unbind ||
      !driver->ops->disconnect || !driver->ops->setup)
    {
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }

  if (priv->driver)
    {
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_DRIVER), 0);
      return -EBUSY;
    }
#endif

  /* First hook up the driver */

  priv->driver = driver;

  /* Then bind the class driver */

  ret = CLASS_BIND(driver, &priv->usbdev);
  if (ret)
    {
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_BINDFAILED), (uint16_t)-ret);
      priv->driver = NULL;
    }

  /* The class driver has been successfully bound. */

  else
    {
      /* Setup the USB controller in it initial unconnected state */

      DEBUGASSERT(priv->devstate == DEVSTATE_DETACHED);
      pic32mx_hwreset(priv);

      /* We do not know the order in which the user will call APIs.  If
       * pic32mx_attach() were called before we got here, the the attach
       * flag will be set and we should also attach to the USB bus.
       */

      if (priv->attached)
        {
          /* usbdev_attach() has already been called.. attach to the bus
           * now
           */

          pic32mx_attach(priv);
        }
   }
  return ret;
}

/****************************************************************************
 * Name: usbdev_unregister
 *
 * Description:
 *   Un-register usbdev class driver. If the USB device is connected to a
 *   USB host, it will first disconnect().  The driver is also requested to
 *   unbind() and clean up any device state, before this procedure finally
 *   returns.
 *
 ****************************************************************************/

int usbdev_unregister(struct usbdevclass_driver_s *driver)
{
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple USB controllers
   * easier.
   */

  struct pic32mx_usbdev_s *priv = &g_usbdev;
  irqstate_t flags;

  usbtrace(TRACE_DEVUNREGISTER, 0);

#ifdef CONFIG_DEBUG
  if (driver != priv->driver)
    {
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Reset the hardware and cancel all requests.  All requests must be
   * canceled while the class driver is still bound.
   */

  flags = irqsave();
  pic32mx_reset(priv);

  /* Unbind the class driver */

  CLASS_UNBIND(driver, &priv->usbdev);

  /* Disable USB controller interrupts (but keep them attached) */

  up_disable_irq(PIC32MX_IRQSRC_USB);

  /* Put the hardware in an inactive state.  Then bring the hardware back up
   * in the reset state (this is probably not necessary, the pic32mx_reset()
   * call above was probably sufficient).
   */

  pic32mx_hwshutdown(priv);
  pic32mx_stateinit(priv);

  /* Unhook the driver */

  priv->driver = NULL;
  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Name: pic32mx_usbattach and pic32mx_usbdetach
 *
 * Description:
 *   The USB stack must be notified when the device is attached or detached
 *   by calling one of these functions.
 *
 ****************************************************************************/

void pic32mx_usbattach(void)
{
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple USB controllers
   * easier.
   */

  struct pic32mx_usbdev_s *priv = &g_usbdev;

  /* Mark that we are attached */

  priv->attached = 1;

  /* This API may be called asynchronously from other initialization
   * interfaces.  In particular, we may not want to attach the bus yet...
   * that should only be done when the class driver is attached.  Has
   * the class driver been attached?
   */

  if (priv->driver)
    {
      /* Yes.. then attach to the bus */

      pic32mx_attach(priv);
    }
}

void pic32mx_usbdetach(void)
{
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple USB controllers
   * easier.
   */

  struct pic32mx_usbdev_s *priv = &g_usbdev;

  /* Detach from the bus */

  pic32mx_detach(priv);
}

#endif /* CONFIG_USBDEV && CONFIG_PIC32MX_USB */
