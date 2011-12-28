/****************************************************************************
 * arch/mips/src/pic32mx/pic32mx_usbdev.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.orgr>
 *
 * References:
 *   - This file derives from the STM32 USB device driver
 *   - "USB On-The-Go (OTG)", DS61126E, Microchip Technology Inc., 2009
 *   - Sample code provided with the Sure Electronics PIC32 board.
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

#ifndef CONFIG_USB_PRI
#  define CONFIG_USB_PRI NVIC_SYSH_PRIORITY_DEFAULT
#endif

/* Extremely detailed register debug that you would normally never want
 * enabled.
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_PIC32MX_USBDEV_REGDEBUG
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

#define NEP_REGISTERS         (16) /* 16 endpoint control registers */
#define PIC32MX_NENDPOINTS    (16)
#define EP0                   (0)
#define EP1                   (1)
#define EP2                   (2)
#define EP3                   (3)
#define EP4                   (4)
#define EP5                   (5)
#define EP6                   (6)
#define EP7                   (7)
#define EP8                   (8)
#define EP9                   (9)
#define EP10                  (10)
#define EP11                  (11)
#define EP12                  (12)
#define EP13                  (13)
#define EP14                  (14)
#define EP15                  (15)

#define PIC32MX_ENDP_BIT(ep)  (1 << (ep))
#define PIC32MX_ENDP_ALLSET   0xff

/* Packet sizes.  We us a fixed 64 max packet size for all endpoint types */

#define PIC32MX_MAXPACKET_SHIFT (6)
#define PIC32MX_MAXPACKET_SIZE  (1 << (PIC32MX_MAXPACKET_SHIFT))
#define PIC32MX_MAXPACKET_MASK  (PIC32MX_MAXPACKET_SIZE-1)

#define PIC32MX_EP0MAXPACKET    PIC32MX_MAXPACKET_SIZE 

/* UEPn Initialization Parameters */

#define EP_CTRL               (USB_EP_EPTXEN|USB_EP_EPRXEN)
#define EP_OUT                (USB_EP_EPRXEN|USB_EP_EPCONDIS)
#define EP_IN                 (USB_EP_EPTXEN|USB_EP_EPCONDIS)
#define EP_OUT_IN             (USB_EP_EPTXEN|USB_EP_EPRXEN|USB_EP_EPCONDIS)

/* USB-related masks */

#define REQRECIPIENT_MASK     (USB_REQ_TYPE_MASK | USB_REQ_RECIPIENT_MASK)

/* Endpoint register masks (handling toggle fields) */

#define EPR_NOTOG_MASK        (USB_EPR_CTR_RX  | USB_EPR_SETUP  | USB_EPR_EPTYPE_MASK |\
                               USB_EPR_EP_KIND | USB_EPR_CTR_TX | USB_EPR_EA_MASK)
#define EPR_TXDTOG_MASK       (USB_EPR_STATTX_MASK | EPR_NOTOG_MASK)
#define EPR_RXDTOG_MASK       (USB_EPR_STATRX_MASK | EPR_NOTOG_MASK)

/* Request queue operations *************************************************/

#define pic32mx_rqempty(ep)     ((ep)->head == NULL)
#define pic32mx_rqpeek(ep)      ((ep)->head)

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
#define PIC32MX_TRACEERR_EP0BADCTR            0x0011
#define PIC32MX_TRACEERR_EP0SETUPSTALLED      0x0012
#define PIC32MX_TRACEERR_EPBUFFER             0x0013
#define PIC32MX_TRACEERR_EPDISABLED           0x0014
#define PIC32MX_TRACEERR_EPOUTNULLPACKET      0x0015
#define PIC32MX_TRACEERR_EPRESERVE            0x0016
#define PIC32MX_TRACEERR_INVALIDCTRLREQ       0x0017
#define PIC32MX_TRACEERR_INVALIDPARMS         0x0018
#define PIC32MX_TRACEERR_IRQREGISTRATION      0x0019
#define PIC32MX_TRACEERR_NOTCONFIGURED        0x001a
#define PIC32MX_TRACEERR_REQABORTED           0x001b

/* Trace interrupt codes */

#define PIC32MX_TRACEINTID_CLEARFEATURE       0x0001
#define PIC32MX_TRACEINTID_DEVGETSTATUS       0x0002
#define PIC32MX_TRACEINTID_DISPATCH           0x0003
#define PIC32MX_TRACEINTID_EP0IN              0x0004
#define PIC32MX_TRACEINTID_EP0INDONE          0x0005
#define PIC32MX_TRACEINTID_EP0OUTDONE         0x0006
#define PIC32MX_TRACEINTID_EP0SETUPDONE       0x0007
#define PIC32MX_TRACEINTID_EP0SETUPSETADDRESS 0x0008
#define PIC32MX_TRACEINTID_EPGETSTATUS        0x0009
#define PIC32MX_TRACEINTID_EPINDONE           0x000a
#define PIC32MX_TRACEINTID_EPINQEMPTY         0x000b
#define PIC32MX_TRACEINTID_EPOUTDONE          0x000c
#define PIC32MX_TRACEINTID_EPOUTPENDING       0x000d
#define PIC32MX_TRACEINTID_EPOUTQEMPTY        0x000e
#define PIC32MX_TRACEINTID_SOF               0x000f
#define PIC32MX_TRACEINTID_GETCONFIG          0x0010
#define PIC32MX_TRACEINTID_GETSETDESC         0x0011
#define PIC32MX_TRACEINTID_GETSETIF           0x0012
#define PIC32MX_TRACEINTID_GETSTATUS          0x0013
#define PIC32MX_TRACEINTID_IFGETSTATUS        0x0015
#define PIC32MX_TRACEINTID_TRNC              0x0016
#define PIC32MX_TRACEINTID_INTERRUPT        0x0017
#define PIC32MX_TRACEINTID_NOSTDREQ           0x0018
#define PIC32MX_TRACEINTID_RESET              0x0019
#define PIC32MX_TRACEINTID_SETCONFIG          0x001a
#define PIC32MX_TRACEINTID_SETFEATURE         0x001b
#define PIC32MX_TRACEINTID_IDLE               0x001c
#define PIC32MX_TRACEINTID_SYNCHFRAME         0x001d
#define PIC32MX_TRACEINTID_WKUP               0x001e
#define PIC32MX_TRACEINTID_T1MSEC             0x001f
#define PIC32MX_TRACEINTID_OTGID              0x0020
#define PIC32MX_TRACEINTID_STALL              0x0021
#define PIC32MX_TRACEINTID_UERR               0x0022

/* Misc Helper Macros *******************************************************/

#define PHYS_ADDR(a) ((uint32_t)(a) & 0x1fffffff)

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
  CTRLSTATE_IDLE = 0,       /* No request in progress */
  CTRLSTATE_RDREQUEST,      /* Read request in progress */
  CTRLSTATE_WRREQUEST,      /* Write request in progress */
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
  struct usbdev_req_s  req;             /* Standard USB request */
  struct pic32mx_req_s  *flink;         /* Supports a singly linked list */
};

/* This is the internal representation of an endpoint */

/* This structure is used to keep track of data that is sent out via an IN EP */

struct usb_inpipe_s
{
  uint8_t  *src;
  bool     inuse;
  uint16_t  wcount;
};

/* This structure is used to keep track of data that is coming in via an OUT EP */

struct usb_outpipe_s
{
  uint8_t *dest;
  bool     inuse;
  uint16_t wcount;
  void (*callback)(void);
};

struct pic32mx_ep_s
{
  /* Common endpoint fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_ep_s
   * to struct pic32mx_ep_s.
   */

  struct usbdev_ep_s        ep;         /* Standard endpoint structure */

  /* PIC32MX-specific fields */

  struct usb_ctrlreq_s     ctrl;        /* Last EP0 request */
  struct pic32mx_usbdev_s *dev;         /* Reference to private driver data */
  struct pic32mx_req_s    *head;        /* Request list for this endpoint */
  struct pic32mx_req_s    *tail;
  uint8_t                  bufno;       /* Allocated buffer number */
  uint8_t                  stalled:1;   /* true: Endpoint is stalled */
  uint8_t                  halted:1;    /* true: Endpoint feature halted */
  uint8_t                  txbusy:1;    /* true: TX endpoint FIFO full */
  uint8_t                  txnullpkt:1; /* Null packet needed at end of transfer */

  struct usbotg_bdtentry_s *bdt;        /* BDT entry */
  union
  {
    struct usb_inpipe_s    inpipe;      /* State of the outgoing data transfer */
    struct usb_outpipe_s   outpipe;     /* State of the incoming data transfer */
  } u;
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
  uint8_t                  nesofs;        /* ESOF counter (for resume support) */
  uint8_t                  config;        /* Active configuration */
  uint8_t                  rxpending:1;   /* 1: OUT data in PMA, but no read requests */
  uint8_t                  selfpowered:1; /* 1: Device is self powered */
  uint8_t                  epavail;       /* Bitset of available endpoints */
  uint8_t                  bufavail;      /* Bitset of available buffers */
  uint16_t                 rxstatus;      /* Saved during interrupt processing */
  uint16_t                 txstatus;      /* "   " "    " "       " "        " */
  uint16_t                 imask;         /* Current interrupt mask */

  /* The endpoint list */

  struct pic32mx_ep_s      eplist[PIC32MX_NENDPOINTS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

#if defined(CONFIG_PIC32MX_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
static uint16_t pic32mx_getreg(uint32_t addr);
static void pic32mx_putreg(uint16_t val, uint32_t addr);
static void pic32mx_dumpep(int epno);
#else
# define pic32mx_getreg(addr)      getreg16(addr)
# define pic32mx_putreg(val,addr)  putreg16(val,addr)
# define pic32mx_dumpep(epno)
#endif

/* Low-Level Helpers ********************************************************/

static inline void
              pic32mx_seteptxcount(uint8_t epno, uint16_t count);
static inline void
              pic32mx_seteptxaddr(uint8_t epno, uint16_t addr);
static inline uint16_t
              pic32mx_geteptxaddr(uint8_t epno);
static void   pic32mx_seteprxcount(uint8_t epno, uint16_t count);
static inline uint16_t
              pic32mx_geteprxcount(uint8_t epno);
static inline void
              pic32mx_seteprxaddr(uint8_t epno, uint16_t addr);
static inline uint16_t
              pic32mx_geteprxaddr(uint8_t epno);
static inline void
              pic32mx_setepaddress(uint8_t epno, uint16_t addr);
static inline void
              pic32mx_seteptype(uint8_t epno, uint16_t type);
static inline void
              pic32mx_seteptxaddr(uint8_t epno, uint16_t addr);
static inline void
              pic32mx_setstatusout(uint8_t epno);
static inline void
              pic32mx_clrstatusout(uint8_t epno);
static void   pic32mx_clrrxdtog(uint8_t epno);
static void   pic32mx_clrtxdtog(uint8_t epno);
static void   pic32mx_clrepctrrx(uint8_t epno);
static void   pic32mx_clrepctrtx(uint8_t epno);
static void   pic32mx_seteptxstatus(uint8_t epno, uint16_t state);
static void   pic32mx_seteprxstatus(uint8_t epno, uint16_t state);
static inline uint16_t
              pic32mx_geteptxstatus(uint8_t epno);
static inline uint16_t
              pic32mx_geteprxstatus(uint8_t epno);
static uint16_t pic32mx_eptxstalled(uint8_t epno);
static uint16_t pic32mx_eprxstalled(uint8_t epno);

/* Suspend/Resume Helpers ***************************************************/

static void   pic32mx_suspend(struct pic32mx_usbdev_s *priv);
static void   pic32mx_resume(struct pic32mx_usbdev_s *priv);

/* Request Helpers **********************************************************/

static void   pic32mx_copytopma(const uint8_t *buffer, uint16_t pma,
                uint16_t nbytes);
static inline void
              pic32mx_copyfrompma(uint8_t *buffer, uint16_t pma, uint16_t nbytes);
static struct pic32mx_req_s *
              pic32mx_rqdequeue(struct pic32mx_ep_s *privep);
static void   pic32mx_rqenqueue(struct pic32mx_ep_s *privep,
                struct pic32mx_req_s *req);
static inline void
              pic32mx_abortrequest(struct pic32mx_ep_s *privep,
                struct pic32mx_req_s *privreq, int16_t result);
static void   pic32mx_reqcomplete(struct pic32mx_ep_s *privep, int16_t result);
static void   pic32mx_epwrite(struct pic32mx_usbdev_s *buf,
                struct pic32mx_ep_s *privep, const uint8_t *data, uint32_t nbytes);
static int    pic32mx_wrrequest(struct pic32mx_usbdev_s *priv,
                struct pic32mx_ep_s *privep);
static int    pic32mx_rdrequest(struct pic32mx_usbdev_s *priv,
                struct pic32mx_ep_s *privep);
static void   pic32mx_cancelrequests(struct pic32mx_ep_s *privep);

/* Interrupt level processing ***********************************************/

static void   pic32mx_dispatchrequest(struct pic32mx_usbdev_s *priv);
static void   pic32mx_eptransfer(struct pic32mx_usbdev_s *priv, uint8_t epno);
static void   pic32mx_setdevaddr(struct pic32mx_usbdev_s *priv, uint8_t value);
static void   pic32mx_ep0setup(struct pic32mx_usbdev_s *priv);
static void   pic32mx_ep0out(struct pic32mx_usbdev_s *priv);
static void   pic32mx_ep0in(struct pic32mx_usbdev_s *priv);
static void   pic32mx_ep0transfer(struct pic32mx_usbdev_s *priv, uint16_t pending);
static int    pic32mx_interrupt(int irq, void *context);

/* Endpoint helpers *********************************************************/

static inline struct pic32mx_ep_s *
              pic32mx_epreserve(struct pic32mx_usbdev_s *priv, uint8_t epset);
static inline void
              pic32mx_epunreserve(struct pic32mx_usbdev_s *priv,
                struct pic32mx_ep_s *privep);
static inline bool
              pic32mx_epreserved(struct pic32mx_usbdev_s *priv, int epno);
static int    pic32mx_epallocpma(struct pic32mx_usbdev_s *priv);
static inline void
              pic32mx_epfreepma(struct pic32mx_usbdev_s *priv,
                struct pic32mx_ep_s *privep);

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
static void   pic32mx_hwsetup(struct pic32mx_usbdev_s *priv);
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

/* Buffer Descriptor Table */

#ifndef CONFIG_USB_PINGPONG
volatile struct usbotg_bdtentry_s
  g_bdt[(PIC32MX_NENDPOINTS + 1) * 2] __attribute__ ((aligned(512)));
#else
volatile struct usbotg_bdtentry_s
  g_bdt[(PIC32MX_NENDPOINTS + 1) * 4] __attribute__ ((aligned(512)));
#endif

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

#if defined(CONFIG_PIC32MX_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
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

#if defined(CONFIG_PIC32MX_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
static void pic32mx_putreg(uint16_t val, uint32_t addr)
{
  /* Show the register value being written */

  lldbg("%08x<-%04x\n", addr, val);

  /* Write the value */

  putreg16(val, addr);
}
#endif

/****************************************************************************
 * Name: pic32mx_dumpep
 ****************************************************************************/

#if defined(CONFIG_PIC32MX_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
static void pic32mx_dumpep(int epno)
{
  uint32_t addr;

  /* Common registers */

  lldbg("CNTR:   %04x\n", getreg16(PIC32MX_USB_CNTR));
  lldbg("ISTR:   %04x\n", getreg16(PIC32MX_USB_ISTR));
  lldbg("FNR:    %04x\n", getreg16(PIC32MX_USB_FNR));
  lldbg("DADDR:  %04x\n", getreg16(PIC32MX_USB_DADDR));
  lldbg("BTABLE: %04x\n", getreg16(PIC32MX_USB_BTABLE));

  /* Endpoint register */

  addr = PIC32MX_USB_EPR(epno);
  lldbg("EPR%d:   [%08x] %04x\n", epno, addr, getreg16(addr));

  /* Endpoint descriptor */

  addr = PIC32MX_USB_BTABLE_ADDR(epno, 0);
  lldbg("DESC:   %08x\n", addr);

  /* Endpoint buffer descriptor */

  addr = PIC32MX_USB_ADDR_TX(epno);
  lldbg("  TX ADDR:  [%08x] %04x\n",  addr, getreg16(addr));

  addr = PIC32MX_USB_COUNT_TX(epno);
  lldbg("     COUNT: [%08x] %04x\n",  addr, getreg16(addr));

  addr = PIC32MX_USB_ADDR_RX(epno);
  lldbg("  RX ADDR:  [%08x] %04x\n",  addr, getreg16(addr));

  addr = PIC32MX_USB_COUNT_RX(epno);
  lldbg("     COUNT: [%08x] %04x\n",  addr, getreg16(addr));
}
#endif

/****************************************************************************
 * Low-Level Helpers
 ****************************************************************************/
/****************************************************************************
 * Name: pic32mx_seteptxcount
 ****************************************************************************/

static inline void pic32mx_seteptxcount(uint8_t epno, uint16_t count) 
{
  volatile uint32_t *epaddr = (uint32_t*)PIC32MX_USB_COUNT_TX(epno);
  *epaddr = count;
} 

/****************************************************************************
 * Name: pic32mx_seteptxaddr
 ****************************************************************************/

static inline void pic32mx_seteptxaddr(uint8_t epno, uint16_t addr)
{
  volatile uint32_t *txaddr = (uint32_t*)PIC32MX_USB_ADDR_TX(epno);
  *txaddr = addr;
}

/****************************************************************************
 * Name: pic32mx_geteptxaddr
 ****************************************************************************/

static inline uint16_t pic32mx_geteptxaddr(uint8_t epno)
{
  volatile uint32_t *txaddr = (uint32_t*)PIC32MX_USB_ADDR_TX(epno);
  return (uint16_t)*txaddr;
}

/****************************************************************************
 * Name: pic32mx_seteprxcount
 ****************************************************************************/

static void pic32mx_seteprxcount(uint8_t epno, uint16_t count) 
{
  volatile uint32_t *epaddr = (uint32_t*)PIC32MX_USB_COUNT_RX(epno);
  uint32_t rxcount = 0;
  uint16_t nblocks;

  /* The upper bits of the RX COUNT value contain the size of allocated
   * RX buffer.  This is based on a block size of 2 or 32:
   *
   * USB_COUNT_RX_BL_SIZE not set:
   *   nblocks is in units of 2 bytes.
   *     00000 - not allowed
   *     00001 - 2 bytes
   *     ....
   *     11111 - 62 bytes
   *
   * USB_COUNT_RX_BL_SIZE set:
   *     00000 - 32 bytes
   *     00001 - 64 bytes
   *     ...
   *     01111 - 512 bytes
   *     1xxxx - Not allowed
   */

  if (count > 62)
    {
      /* Blocks of 32 (with 0 meaning one block of 32) */

      nblocks = (count >> 5) - 1 ;
      DEBUGASSERT(nblocks <= 0x0f);
      rxcount = (uint32_t)((nblocks << USB_COUNT_RX_NUM_BLOCK_SHIFT) | USB_COUNT_RX_BL_SIZE);
    }
  else if (count > 0)
    {
      /* Blocks of 2 (with 1 meaning one block of 2) */

      nblocks = (count + 1) >> 1;
      DEBUGASSERT(nblocks > 0 && nblocks < 0x1f);
      rxcount = (uint32_t)(nblocks << USB_COUNT_RX_NUM_BLOCK_SHIFT);
    }
  *epaddr = rxcount;
} 

/****************************************************************************
 * Name: pic32mx_geteprxcount
 ****************************************************************************/

static inline uint16_t pic32mx_geteprxcount(uint8_t epno)
{
  volatile uint32_t *epaddr = (uint32_t*)PIC32MX_USB_COUNT_RX(epno);
  return (*epaddr) & USB_COUNT_RX_MASK;
}

/****************************************************************************
 * Name: pic32mx_seteprxaddr
 ****************************************************************************/

static inline void pic32mx_seteprxaddr(uint8_t epno, uint16_t addr)
{
  volatile uint32_t *rxaddr = (uint32_t*)PIC32MX_USB_ADDR_RX(epno);
  *rxaddr = addr;
}

/****************************************************************************
 * Name: pic32mx_seteprxaddr
 ****************************************************************************/

static inline uint16_t pic32mx_geteprxaddr(uint8_t epno)
{
  volatile uint32_t *rxaddr = (uint32_t*)PIC32MX_USB_ADDR_RX(epno);
  return (uint16_t)*rxaddr;
}

/****************************************************************************
 * Name: pic32mx_setepaddress
 ****************************************************************************/

static inline void pic32mx_setepaddress(uint8_t epno, uint16_t addr) 
{
  uint32_t epaddr = PIC32MX_USB_EPR(epno);
  uint16_t regval;

  regval  = pic32mx_getreg(epaddr);
  regval &= EPR_NOTOG_MASK;
  regval &= ~USB_EPR_EA_MASK;
  regval |= (addr << USB_EPR_EA_SHIFT);
  pic32mx_putreg(regval, epaddr);
} 

/****************************************************************************
 * Name: pic32mx_seteptype
 ****************************************************************************/

static inline void pic32mx_seteptype(uint8_t epno, uint16_t type)
{
  uint32_t epaddr = PIC32MX_USB_EPR(epno);
  uint16_t regval;

  regval  = pic32mx_getreg(epaddr);
  regval &= EPR_NOTOG_MASK;
  regval &= ~USB_EPR_EPTYPE_MASK;
  regval |= type;
  pic32mx_putreg(regval, epaddr);
}

/****************************************************************************
 * Name: pic32mx_setstatusout
 ****************************************************************************/

static inline void pic32mx_setstatusout(uint8_t epno)
{
  uint32_t epaddr = PIC32MX_USB_EPR(epno);
  uint16_t regval;

  /* For a BULK endpoint the EP_KIND bit is used to enabled double buffering;
   * for a CONTROL endpoint, it is set to indicate that a status OUT
   * transaction is expected.  The bit is not used with out endpoint types.
   */

  regval  = pic32mx_getreg(epaddr);
  regval &= EPR_NOTOG_MASK;
  regval |= USB_EPR_EP_KIND;
  pic32mx_putreg(regval, epaddr);
}

/****************************************************************************
 * Name: pic32mx_clrstatusout
 ****************************************************************************/

static inline void pic32mx_clrstatusout(uint8_t epno)
{
  uint32_t epaddr = PIC32MX_USB_EPR(epno);
  uint16_t regval;

  /* For a BULK endpoint the EP_KIND bit is used to enabled double buffering;
   * for a CONTROL endpoint, it is set to indicate that a status OUT
   * transaction is expected.  The bit is not used with out endpoint types.
   */

  regval  = pic32mx_getreg(epaddr);
  regval &= EPR_NOTOG_MASK;
  regval &= ~USB_EPR_EP_KIND;
  pic32mx_putreg(regval, epaddr);
}

/****************************************************************************
 * Name: pic32mx_clrrxdtog
 ****************************************************************************/

static void pic32mx_clrrxdtog(uint8_t epno) 
{
  uint32_t epaddr = PIC32MX_USB_EPR(epno);
  uint16_t regval;

  regval = pic32mx_getreg(epaddr);
  if ((regval & USB_EPR_DTOG_RX) != 0)
    {
      regval &= EPR_NOTOG_MASK;
      regval |= USB_EPR_DTOG_RX;
      pic32mx_putreg(regval, epaddr);
    } 
} 

/****************************************************************************
 * Name: pic32mx_clrtxdtog
 ****************************************************************************/

static void pic32mx_clrtxdtog(uint8_t epno) 
{
  uint32_t epaddr = PIC32MX_USB_EPR(epno);
  uint16_t regval;

  regval = pic32mx_getreg(epaddr);
  if ((regval & USB_EPR_DTOG_TX) != 0)
    {
      regval &= EPR_NOTOG_MASK;
      regval |= USB_EPR_DTOG_TX;
      pic32mx_putreg(regval, epaddr);
    }
} 

/****************************************************************************
 * Name: pic32mx_clrepctrrx
 ****************************************************************************/

static void pic32mx_clrepctrrx(uint8_t epno) 
{
  uint32_t epaddr = PIC32MX_USB_EPR(epno);
  uint16_t regval;

  regval  = pic32mx_getreg(epaddr);
  regval &= EPR_NOTOG_MASK;
  regval &= ~USB_EPR_CTR_RX;
  pic32mx_putreg(regval, epaddr);
} 

/****************************************************************************
 * Name: pic32mx_clrepctrtx
 ****************************************************************************/

static void pic32mx_clrepctrtx(uint8_t epno) 
{
  uint32_t epaddr = PIC32MX_USB_EPR(epno);
  uint16_t regval;

  regval  = pic32mx_getreg(epaddr);
  regval &= EPR_NOTOG_MASK;
  regval &= ~USB_EPR_CTR_TX;
  pic32mx_putreg(regval, epaddr);
} 

/****************************************************************************
 * Name: pic32mx_geteptxstatus
 ****************************************************************************/

static inline uint16_t pic32mx_geteptxstatus(uint8_t epno) 
{
  return (uint16_t)(pic32mx_getreg(PIC32MX_USB_EPR(epno)) & USB_EPR_STATTX_MASK);
}

/****************************************************************************
 * Name: pic32mx_geteprxstatus
 ****************************************************************************/

static inline uint16_t pic32mx_geteprxstatus(uint8_t epno) 
{
  return (pic32mx_getreg(PIC32MX_USB_EPR(epno)) & USB_EPR_STATRX_MASK);
}

/****************************************************************************
 * Name: pic32mx_seteptxstatus
 ****************************************************************************/

static void pic32mx_seteptxstatus(uint8_t epno, uint16_t state) 
{
  uint32_t epaddr = PIC32MX_USB_EPR(epno);
  uint16_t regval;

  /* The bits in the STAT_TX field can be toggled by software to set their
   * value. When set to 0, the value remains unchanged; when set to one,
   * value toggles.
   */

  regval = pic32mx_getreg(epaddr);

  /* The exclusive OR will set STAT_TX bits to 1 if there value is different
   * from the bits requested in 'state'
   */

  regval ^= state;
  regval &= EPR_TXDTOG_MASK;
  pic32mx_putreg(regval, epaddr);
} 

/****************************************************************************
 * Name: pic32mx_seteprxstatus
 ****************************************************************************/

static void pic32mx_seteprxstatus(uint8_t epno, uint16_t state) 
{
  uint32_t epaddr = PIC32MX_USB_EPR(epno);
  uint16_t regval;

  /* The bits in the STAT_RX field can be toggled by software to set their
   * value. When set to 0, the value remains unchanged; when set to one,
   * value toggles.
   */

  regval = pic32mx_getreg(epaddr);

  /* The exclusive OR will set STAT_RX bits to 1 if there value is different
   * from the bits requested in 'state'
   */

  regval ^= state;
  regval &= EPR_RXDTOG_MASK;
  pic32mx_putreg(regval, epaddr);
} 

/****************************************************************************
 * Name: pic32mx_eptxstalled
 ****************************************************************************/

static inline uint16_t pic32mx_eptxstalled(uint8_t epno) 
{
  return (pic32mx_geteptxstatus(epno) == USB_EPR_STATTX_STALL);
}

/****************************************************************************
 * Name: pic32mx_eprxstalled
 ****************************************************************************/

static inline uint16_t pic32mx_eprxstalled(uint8_t epno) 
{
  return (pic32mx_geteprxstatus(epno) == USB_EPR_STATRX_STALL);
}

/****************************************************************************
 * Request Helpers
 ****************************************************************************/
/****************************************************************************
 * Name: pic32mx_copytopma
 ****************************************************************************/

static void pic32mx_copytopma(const uint8_t *buffer, uint16_t pma, uint16_t nbytes) 
{
  uint16_t *dest;
  uint16_t  ms;
  uint16_t  ls;
  int     nwords = (nbytes + 1) >> 1;
  int     i;

  /* Copy loop.  Source=user buffer, Dest=packet memory */

  dest = (uint16_t*)(PIC32MX_USBCANRAM_BASE + ((uint32_t)pma << 1));
  for (i = nwords; i != 0; i--)
    {
      /* Read two bytes and pack into on 16-bit word */

      ls = (uint16_t)(*buffer++);
      ms = (uint16_t)(*buffer++);
      *dest = ms << 8 | ls;

      /* Source address increments by 2*sizeof(uint8_t) = 2; Dest address
       * increments by 2*sizeof(uint16_t) = 4.
       */

      dest += 2;
    }
}

/****************************************************************************
 * Name: pic32mx_copyfrompma
 ****************************************************************************/

static inline void
pic32mx_copyfrompma(uint8_t *buffer, uint16_t pma, uint16_t nbytes) 
{
  uint32_t *src;
  int     nwords = (nbytes + 1) >> 1;
  int     i;

  /* Copy loop.  Source=packet memory, Dest=user buffer */

  src = (uint32_t*)(PIC32MX_USBCANRAM_BASE + ((uint32_t)pma << 1));
  for (i = nwords; i != 0; i--)
    {
      /* Copy 16-bits from packet memory to user buffer. */

      *(uint16_t*)buffer = *src++;

      /* Source address increments by 1*sizeof(uint32_t) = 4; Dest address
       * increments by 2*sizeof(uint8_t) = 2.
       */

      buffer += 2;
    }
}

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
 * Name: tm32_epwrite
 ****************************************************************************/

static void pic32mx_epwrite(struct pic32mx_usbdev_s *priv,
                          struct pic32mx_ep_s *privep,
                          const uint8_t *buf, uint32_t nbytes)
{
  uint8_t epno = USB_EPNO(privep->ep.eplog);
  usbtrace(TRACE_WRITE(epno), nbytes);

  /* Check for a zero-length packet */

  if (nbytes > 0)
    {
      /* Copy the data from the user buffer into packet memory for this
       * endpoint
       */

      pic32mx_copytopma(buf, pic32mx_geteptxaddr(epno), nbytes);
    }

  /* Send the packet (might be a null packet nbytes == 0) */

  pic32mx_seteptxcount(epno, nbytes);
  priv->txstatus = USB_EPR_STATTX_VALID;

  /* Indicate that there is data in the TX packet memory.  This will be cleared
   * when the next data out interrupt is received.
   */

  privep->txbusy = true;
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

  /* Check the request from the head of the endpoint request queue */

  privreq = pic32mx_rqpeek(privep);
  if (!privreq)
    {
      /* There is no TX transfer in progress and no new pending TX
       * requests to send.
       */

      usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_EPINQEMPTY), 0);
      priv->ctrlstate = CTRLSTATE_IDLE;
      return OK;
    }

  epno = USB_EPNO(privep->ep.eplog);
  ullvdbg("epno=%d req=%p: len=%d xfrd=%d nullpkt=%d\n",
          epno, privreq, privreq->req.len, privreq->req.xfrd, privep->txnullpkt);

  /* Get the number of bytes left to be sent in the packet */

  bytesleft         = privreq->req.len - privreq->req.xfrd;
  nbytes            = bytesleft;

#warning "REVISIT: If the EP supports double buffering, then we can do better"

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
  pic32mx_epwrite(priv, privep, buf, nbytes);
  priv->ctrlstate = CTRLSTATE_WRREQUEST;

  /* Update for the next data IN interrupt */

  privreq->req.xfrd += nbytes;
  bytesleft          = privreq->req.len - privreq->req.xfrd;

  /* If all of the bytes were sent (including any final null packet)
   * then we are finished with the transfer
   */

  if (bytesleft == 0 && !privep->txnullpkt)
    {
      usbtrace(TRACE_COMPLETE(USB_EPNO(privep->ep.eplog)), privreq->req.xfrd);
      privep->txnullpkt = 0;
      pic32mx_reqcomplete(privep, OK);
      priv->ctrlstate = CTRLSTATE_IDLE;
    }

  return OK;
}

/****************************************************************************
 * Name: pic32mx_rdrequest
 ****************************************************************************/

static int pic32mx_rdrequest(struct pic32mx_usbdev_s *priv, struct pic32mx_ep_s *privep)
{
  struct pic32mx_req_s *privreq;
  uint32_t src;
  uint8_t *dest;
  uint8_t epno;
  int pmalen;
  int readlen;

  /* Check the request from the head of the endpoint request queue */

  epno    = USB_EPNO(privep->ep.eplog);
  privreq = pic32mx_rqpeek(privep);
  if (!privreq)
    {
      /* Incoming data available in PMA, but no packet to receive the data.
       * Mark that the RX data is pending and hope that a packet is returned
       * soon.
       */

      usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_EPOUTQEMPTY), epno);
      return OK;
    }

  ullvdbg("EP%d: len=%d xfrd=%d\n", epno, privreq->req.len, privreq->req.xfrd);

  /* Ignore any attempt to receive a zero length packet */

  if (privreq->req.len == 0)
    {
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_EPOUTNULLPACKET), 0);
      pic32mx_reqcomplete(privep, OK);
      return OK;
    }

  usbtrace(TRACE_READ(USB_EPNO(privep->ep.eplog)), privreq->req.xfrd);

  /* Get the source and destination transfer addresses */

  dest    = privreq->req.buf + privreq->req.xfrd;
  src     = pic32mx_geteprxaddr(epno);

  /* Get the number of bytes to read from packet memory */

  pmalen  = pic32mx_geteprxcount(epno);
  readlen = MIN(privreq->req.len, pmalen);

  /* Receive the next packet */

  pic32mx_copyfrompma(dest, src, readlen);
  priv->ctrlstate = CTRLSTATE_RDREQUEST;

  /* If the receive buffer is full or this is a partial packet,
   * then we are finished with the transfer
   */

  privreq->req.xfrd += readlen;
  if (pmalen < privep->ep.maxpacket || privreq->req.xfrd >= privreq->req.len)
    {
      /* Complete the transfer and mark the state IDLE.  The endpoint
       * RX will be marked valid when the data phase completes.
       */

      usbtrace(TRACE_COMPLETE(epno), privreq->req.xfrd);
      pic32mx_reqcomplete(privep, OK);
      priv->ctrlstate = CTRLSTATE_IDLE;
    }

  return OK;
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
 * Name: pic32mx_eptransfer
 ****************************************************************************/

static void pic32mx_eptransfer(struct pic32mx_usbdev_s *priv, uint8_t epno)
{
  struct pic32mx_ep_s *privep;
  uint16_t epr;

  /* Decode and service non control endpoints interrupt */ 

  epr    = pic32mx_getreg(PIC32MX_USB_EPR(epno));
  privep = &priv->eplist[epno];

  /* OUT: host-to-device
   * CTR_RX is set by the hardware when an OUT/SETUP transaction
   * successfully completed on this endpoint.
   */

  if ((epr & USB_EPR_CTR_RX) != 0)
    {
      usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_EPOUTDONE), epr);

      /* Handle read requests.  First check if a read request is available to
       * accept the host data.
       */

      if (!pic32mx_rqempty(privep))
        {
          /* Read host data into the current read request */

          pic32mx_rdrequest(priv, privep);

          /* "After the received data is processed, the application software
           *  should set the STAT_RX bits to '11' (Valid) in the USB_EPnR,
           *  enabling further transactions. "
           */

          priv->rxstatus  = USB_EPR_STATRX_VALID;
        }

      /* NAK further OUT packets if there there no more read requests */

      if (pic32mx_rqempty(privep))
        {
          usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_EPOUTPENDING), (uint16_t)epno);

          /* Mark the RX processing as pending and NAK any OUT actions
           * on this endpoint.  "While the STAT_RX bits are equal to '10'
           * (NAK), any OUT request addressed to that endpoint is NAKed,
           * indicating a flow control condition: the USB host will retry
           * the transaction until it succeeds."
           */

          priv->rxstatus  = USB_EPR_STATRX_NAK;
          priv->rxpending = true;
        }

      /* Clear the interrupt status and set the new RX status */

      pic32mx_clrepctrrx(epno);
      pic32mx_seteprxstatus(epno, priv->rxstatus);
    }

  /* IN: device-to-host
   * CTR_TX is set when an IN transaction successfully completes on
   * an endpoint
   */

  else if ((epr & USB_EPR_CTR_TX) != 0)
    {
      /* Clear interrupt status */

      pic32mx_clrepctrtx(epno);
      usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_EPINDONE), epr);
          
      /* Handle write requests */ 

      priv->txstatus = USB_EPR_STATTX_NAK;
      pic32mx_wrrequest(priv, privep);

      /* Set the new TX status */

      pic32mx_seteptxstatus(epno, priv->txstatus);
    }  
}

/****************************************************************************
 * Name: pic32mx_setdevaddr
 ****************************************************************************/

static void pic32mx_setdevaddr(struct pic32mx_usbdev_s *priv, uint8_t value) 
{
  int epno;
  
  /* Set address in every allocated endpoint */

  for (epno = 0; epno < PIC32MX_NENDPOINTS; epno++)
    {
      if (pic32mx_epreserved(priv, epno))
        {
          pic32mx_setepaddress((uint8_t)epno, (uint8_t)epno);
        }
    }

  /* Set the device address and enable function */

  pic32mx_putreg(value|USB_DADDR_EF, PIC32MX_USB_DADDR);
}

/****************************************************************************
 * Name: pic32mx_ep0setup
 ****************************************************************************/

static void pic32mx_ep0setup(struct pic32mx_usbdev_s *priv)
{
  struct pic32mx_ep_s   *ep0     = &priv->eplist[EP0];
  struct pic32mx_req_s  *privreq = pic32mx_rqpeek(ep0);
  struct pic32mx_ep_s   *privep;
  union wb_u           value;
  union wb_u           index;
  union wb_u           len;
  union wb_u           response;
  bool                 handled = false;
  uint8_t              epno;
  int                  nbytes = 0; /* Assume zero-length packet */
  int                  ret;

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

  /* Assume NOT stalled; no TX in progress */

  ep0->stalled  = 0;
  ep0->txbusy   = 0;

  /* Get a 32-bit PMA address and use that to get the 8-byte setup request */

  pic32mx_copyfrompma((uint8_t*)&priv->ctrl, pic32mx_geteprxaddr(EP0), USB_SIZEOF_CTRLREQ);

  /* And extract the little-endian 16-bit values to host order */

  value.w = GETUINT16(priv->ctrl.value);
  index.w = GETUINT16(priv->ctrl.index);
  len.w   = GETUINT16(priv->ctrl.len);

  ullvdbg("SETUP: type=%02x req=%02x value=%04x index=%04x len=%04x\n",
          priv->ctrl.type, priv->ctrl.req, value.w, index.w, len.w);

  priv->ctrlstate = CTRLSTATE_IDLE;

  /* Dispatch any non-standard requests */

  if ((priv->ctrl.type & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_STANDARD)
    {
      usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_NOSTDREQ), priv->ctrl.type);

      /* Let the class implementation handle all non-standar requests */

      pic32mx_dispatchrequest(priv);
      return;
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
                      privep     = &priv->eplist[epno];
                      response.w = 0; /* Not stalled */
                      nbytes     = 2; /* Response size: 2 bytes */

                      if (USB_ISEPIN(index.b[LSB]))
                        {
                          /* IN endpoint */ 

                          if (pic32mx_eptxstalled(epno))
                            {
                              /* IN Endpoint stalled */

                              response.b[LSB] = 1; /* Stalled */
                            }
                          }
                      else
                        {
                          /* OUT endpoint */ 

                          if (pic32mx_eprxstalled(epno))
                            {
                              /* OUT Endpoint stalled */

                              response.b[LSB] = 1; /* Stalled */
                            }
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
                                        (1 << USB_FEATURE_REMOTEWAKEUP);
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
                  response.w = 0;
                  nbytes     = 2; /* Response size: 2 bytes */
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
        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) != USB_REQ_RECIPIENT_ENDPOINT)
          {
            /* Let the class implementation handle all recipients (except for the
             * endpoint recipient)
             */

            pic32mx_dispatchrequest(priv);
            handled = true;
          }
        else
          {
            /* Endpoint recipient */

            epno = USB_EPNO(index.b[LSB]);
            if (epno < PIC32MX_NENDPOINTS && index.b[MSB] == 0 &&
                value.w == USB_FEATURE_ENDPOINTHALT && len.w == 0)
              {
                privep         = &priv->eplist[epno];
                privep->halted = 0;
                ret            = pic32mx_epstall(&privep->ep, true);
              }
            else
              {
                usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_BADCLEARFEATURE), 0);
                priv->ctrlstate = CTRLSTATE_STALLED;
              }
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
        if (((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE) &&
            value.w == USB_FEATURE_TESTMODE)
          {
            /* Special case recipient=device test mode */

            ullvdbg("test mode: %d\n", index.w);
          }
        else if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) != USB_REQ_RECIPIENT_ENDPOINT)
          {
            /* The class driver handles all recipients except recipient=endpoint */

            pic32mx_dispatchrequest(priv);
            handled = true;
          }
        else
          {
            /* Handler recipient=endpoint */

            epno = USB_EPNO(index.b[LSB]);
            if (epno < PIC32MX_NENDPOINTS && index.b[MSB] == 0 &&
                value.w == USB_FEATURE_ENDPOINTHALT && len.w == 0)
              {
                privep         = &priv->eplist[epno];
                privep->halted = 1;
                ret            = pic32mx_epstall(&privep->ep, false);
              }
            else
              {
                usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_BADSETFEATURE), 0);
                priv->ctrlstate = CTRLSTATE_STALLED;
              }
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

        /* Note that setting of the device address will be deferred.  A zero-length
         * packet will be sent and the device address will be set when the zero-
         * length packet transfer completes.
         */
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
            handled = true;
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
            handled = true;
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
             handled = true;
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
        handled = true;
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

  /* At this point, the request has been handled and there are three possible
   * outcomes:
   *
   * 1. The setup request was successfully handled above and a response packet
   *    must be sent (may be a zero length packet).
   * 2. The request was successfully handled by the class implementation.  In
   *    case, the EP0 IN response has already been queued and the local variable
   *    'handled' will be set to true and ctrlstate != CTRLSTATE_STALLED;
   * 3. An error was detected in either the above logic or by the class implementation
   *    logic.  In either case, priv->state will be set CTRLSTATE_STALLED
   *    to indicate this case.
   *
   * NOTE: Non-standard requests are a special case.  They are handled by the
   * class implementation and this function returned early above, skipping this
   * logic altogether.
   */

  if (priv->ctrlstate != CTRLSTATE_STALLED && !handled)
    {
      /* We will response.  First, restrict the data length to the length
       * requested in the setup packet
       */

      if (nbytes > len.w)
        {
          nbytes = len.w;
        }

      /* Send the response (might be a zero-length packet) */

      pic32mx_epwrite(priv, ep0, response.b, nbytes);
      priv->ctrlstate = CTRLSTATE_IDLE;
    }
}

/****************************************************************************
 * Name: pic32mx_ep0in
 ****************************************************************************/

static void pic32mx_ep0in(struct pic32mx_usbdev_s *priv)
{
  /* There is no longer anything in the EP0 TX packet memory */

  priv->eplist[EP0].txbusy = false;

  /* Are we processing the completion of one packet of an outgoing request
   * from the class driver?
   */

  if (priv->ctrlstate == CTRLSTATE_WRREQUEST)
    {
      pic32mx_wrrequest(priv, &priv->eplist[EP0]);
    }

  /* No.. Are we processing the completion of a status response? */

  else if (priv->ctrlstate == CTRLSTATE_IDLE)
    {
      /* Look at the saved SETUP command.  Was it a SET ADDRESS request?
       * If so, then now is the time to set the address.
       */

      if (priv->ctrl.req == USB_REQ_SETADDRESS && 
          (priv->ctrl.type & REQRECIPIENT_MASK) == (USB_REQ_TYPE_STANDARD | USB_REQ_RECIPIENT_DEVICE))
        {
          union wb_u value;
          value.w = GETUINT16(priv->ctrl.value);
          pic32mx_setdevaddr(priv, value.b[LSB]);
        }
    }
  else
    {
      priv->ctrlstate = CTRLSTATE_STALLED;
    }
}

/****************************************************************************
 * Name: pic32mx_ep0out
 ****************************************************************************/

static void pic32mx_ep0out(struct pic32mx_usbdev_s *priv)
{
  struct pic32mx_ep_s *privep = &priv->eplist[EP0];
  switch (priv->ctrlstate)
    {
      case CTRLSTATE_RDREQUEST:  /* Write request in progress */
      case CTRLSTATE_IDLE:       /* No transfer in progress */
        pic32mx_rdrequest(priv, privep);
        break;

      default:
        /* Unexpected state OR host aborted the OUT transfer before it
         * completed, STALL the endpoint in either case
         */

        priv->ctrlstate = CTRLSTATE_STALLED;
        break;
    }
}

/****************************************************************************
 * Name: pic32mx_ep0transfer
 ****************************************************************************/

static void pic32mx_ep0transfer(struct pic32mx_usbdev_s *priv, uint16_t pending)
{
  uint16_t epr;

  /* Initialize RX and TX status.  We shouldn't have to actually look at the
   * status because the hardware is supposed to set the both RX and TX status
   * to NAK when an EP0 SETUP occurs (of course, this might not be a setup)
   */ 

  priv->rxstatus = USB_EPR_STATRX_NAK;
  priv->txstatus = USB_EPR_STATTX_NAK;

  /* Set both RX and TX status to NAK  */ 

  pic32mx_seteprxstatus(EP0, USB_EPR_STATRX_NAK);
  pic32mx_seteptxstatus(EP0, USB_EPR_STATTX_NAK);
          
  /* Check the direction bit to determine if this the completion of an EP0
   * packet sent to or received from the host PC.
   */

  if ((pending & USB_ISTR_DIR) == 0)
    {
      /* EP0 IN: device-to-host (DIR=0) */

      usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_EP0IN), pending);
      pic32mx_clrepctrtx(EP0);
      pic32mx_ep0in(priv);
    }
  else
    {
      /* EP0 OUT: host-to-device (DIR=1) */

      epr = pic32mx_getreg(PIC32MX_USB_EPR(EP0));

      /* CTR_TX is set when an IN transaction successfully
       * completes on an endpoint
       */

      if ((epr & USB_EPR_CTR_TX) != 0)
        {
          usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_EP0INDONE), epr);
          pic32mx_clrepctrtx(EP0);
          pic32mx_ep0in(priv);
        }

      /* SETUP is set by the hardware when the last completed
       * transaction was a control endpoint SETUP
       */
 
      else if ((epr & USB_EPR_SETUP) != 0)
        {
          usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_EP0SETUPDONE), epr);
          pic32mx_clrepctrrx(EP0);
          pic32mx_ep0setup(priv);
        }

      /* Set by the hardware when an OUT/SETUP transaction successfully
       * completed on this endpoint.
       */

      else if ((epr & USB_EPR_CTR_RX) != 0)
        {
          usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_EP0OUTDONE), epr);
          pic32mx_clrepctrrx(EP0);
          pic32mx_ep0out(priv);
        }

      /* None of the above */

      else
        {
          usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_EP0BADCTR), epr);
          return; /* Does this ever happen? */
        }
    }

  /* Make sure that the EP0 packet size is still OK (superstitious?) */

  pic32mx_seteprxcount(EP0, PIC32MX_EP0MAXPACKET);

  /* Now figure out the new RX/TX status.  Here are all possible
   * consequences of the above EP0 operations:
   *
   * rxstatus txstatus ctrlstate  MEANING
   * -------- -------- --------- ---------------------------------
   * NAK      NAK      IDLE      Nothing happened
   * NAK      VALID    IDLE      EP0 response sent from USBDEV driver
   * NAK      VALID    WRREQUEST EP0 response sent from class driver
   * NAK      ---      STALL     Some protocol error occurred
   *
   * First handle the STALL condition:
   */

  if (priv->ctrlstate == CTRLSTATE_STALLED)
    {
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_EP0SETUPSTALLED), priv->ctrlstate);
      priv->rxstatus = USB_EPR_STATRX_STALL;
      priv->txstatus = USB_EPR_STATTX_STALL;
    }

  /* Was a transmission started?  If so, txstatus will be VALID.  The
   * only special case to handle is when both are set to NAK.  In that
   * case, we need to set RX status to VALID in order to accept the next
   * SETUP request.
   */

  else if (priv->rxstatus == USB_EPR_STATRX_NAK &&
           priv->txstatus == USB_EPR_STATTX_NAK)
    {
      priv->rxstatus = USB_EPR_STATRX_VALID;
    }

  /* Now set the new TX and RX status */ 

  pic32mx_seteprxstatus(EP0, priv->rxstatus);
  pic32mx_seteptxstatus(EP0, priv->txstatus);
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
  uint16_t pending;
  uint16_t regval;
   
  /* Get the set of pending USB interrupts */

  pending = pic32mx_getreg(PIC32MX_USBOTG_IR) & pic32mx_getreg(PIC32MX_USBOTG_IE);
  usbtrace(TRACE_INTENTRY(PIC32MX_TRACEINTID_INTERRUPT), pending);

#ifdef CONFIG_USBOTG
  /* Session Request Protocol (SRP) Time Out Check */
  
  /* if USB OTG SRP is ready */
#  warning "Missing logic"
    {
      /* Check if the 1 millisecond timer has expired */
 
      if ((pic32mx_getreg(PIC32MX_USBOTG_IR) & pic32mx_getreg(PIC32MX_USBOTG_IE) & USB  OTG_INT_T1MSEC) != 0)
        {
          usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_T1MSEC), pending);

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

      /* Make sure the the USE reset and idle detect interrupts are enabled */

      regval = pic32mx_getreg(PIC32MX_USB_IE);
      regval |= (USB_INT_URST|USB_INT_IDLE);
      pic32mx_putreg(regval, PIC32MX_USB_IE);

      /* Now were are in the powered state */

      priv->devstate = DEVSTATE_POWERED;
    }
   
#ifdef  CONFIG_USBOTG
  /* Check if the ID Pin Changed State */

  if (pending & USBOTG_INT_ID) != 0)
    {
      usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_OTGID), pending);

      /* Re-detect and re-initialize */
#warning "Missing logic"

      pic32mx_putreg(USBOTG_INT_ID, PIC32MX_USBOTG_IR);
    }
#endif
    
  /* Service the USB Activity Interrupt */

  if ((pending & USBOTG_INT_ACTV) != 0)
    {
      usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_WKUP), pending);
      pic32mx_putreg(USB_INT_IDLE, PIC32MX_USB_IR);

#if defined(CONFIG_USBOTG)
      pic32mx_putreg(USBOTG_INT_ACTV, PIC32MX_USBOTG_IR);
#else
      pic32mx_resume(priv);
#endif
    }

  /*  It is pointless to continue servicing if the device is in suspend mode. */

  if ((pic32mx_getreg(PIC32MX_USB_PWRC) & USB_PWRC_USUSPEND) != 0)
    {
      /* Just clear the interrupt and return */

      up_clrpend_irq(PIC32MX_IRQSRC_USB);
      return OK;
    }
   
  /* Service USB Bus Reset Interrupt. When bus reset is received during
   * suspend, ACTVIF will be set first, once the UCONbits.SUSPND is clear,
   * then the URSTIF bit will be asserted. This is why URSTIF is checked
   * after ACTVIF.  The USB reset flag is masked when the USB state is in
   * DEVSTATE_DETACHED or DEVSTATE_ATTACHED, and therefore cannot cause a
   * USB reset event during these two states.
   */

  if ((pending & USB_INT_URST) != 0)
    {
      usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_RESET), pending);

      /* Reset interrupt received. Restore our power-up state */

      pic32mx_reset(priv);
      pic32mx_attach(priv);
      priv->devstate = DEVSTATE_DEFAULT;

      /* Re-initialize EP0 */

      g_bdt[EP0_OUT_EVEN].addr    =  (uint8_t *)PHYS_ADDR(&priv->ctrl);
      g_bdt[EP0_OUT_EVEN].status &= ~USB_BDT_BYTECOUNT_MASK;
      g_bdt[EP0_OUT_EVEN].status |= (USB_EP0_BUFF_SIZE << USB_BDT_BYTECOUNT_SHIFT);
      g_bdt[EP0_OUT_EVEN].status &= ~USB_BDT_STATUS_MASK;
      g_bdt[EP0_OUT_EVEN].status |= USB_BDT_UOWN | USB_BDT_DATA0 | USB_BDT_DTS | USB_BDT_BSTALL;
       
#ifdef CONFIG_USBOTG
        /* Disable and deactivate HNP */
#warning Missing Logic
#endif
        pic32mx_putreg(USB_INT_URST, PIC32MX_USB_IR);
        return OK;
    }
   
  /* Service IDLE interrupts */

  if ((pending & USB_INT_IDLE) != 0)
    {
      usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_IDLE), pending);

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
  if ((pending & USB_INT_SOF) != 0)
    {
      usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_SOF), 0);

      /* I am not sure why you would ever enable SOF interrupts */

      pic32mx_putreg(USB_INT_SOF, PIC32MX_USB_IR);
    }
#endif

   /* Service stall interrupts */
 
   if ((pending & USB_INT_STALL) != 0)
    {
      usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_STALL), pending);

      pic32mx_stall(priv);
    }

  /* Service error interrupts */

  if ((pending & USB_INT_UERR) != 0)
    {
      usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_UERR), pending);
      ulldbg("Error: EIR=%04x\n", pic32mx_getreg(PIC32MX_USB_EIR));

      /* Clear all pending USB error interrupts */

      pic32mx_putreg(USB_EINT_ALL, PIC32MX_USB_EIR);
    }
   
  /* It is pointless to continue servicing if the host has not sent a bus
   * reset. Once bus reset is received, the device transitions into the DEFAULT
   * state and is ready for communication.
   */

  if (priv->devstate < DEVSTATE_DEFAULT)
    {
      up_clrpend_irq(PIC32MX_IRQSRC_USB);
      return OK;
    }
   
  /*  Service USB Transaction Complete Interrupt */

  if ((pending & USB_INT_TRN) != 0)
    {
      usbtrace(TRACE_INTDECODE(PIC32MX_TRACEINTID_TRNC), pending);

      /* Drain the USAT FIFO entries.  If the USB FIFO ever gets full, USB
       * bandwidth utilization can be compromised, and the device won't be
       * able to receive SETUP packets.
       */

      for (i = 0; i < 4; i++)
        {
          uint8_t epno;

          /* Is token processing complete */

          if ((pic32mx_getreg(PIC32MX_USB_IR) & USB_INT_TRN) != 0)
            
            {
              regval = pic32mx_getreg(PIC32MX_USB_STAT);
              pic32mx_putreg(USB_INT_TRN, PIC32MX_USB_IR);
               
              /* Handle the endpoint tranfer complete event. */

              epno = (regval & USB_STAT_ENDPT_MASK) >> USB_STAT_ENDPT_SHIFT;
              if (epno == 0)
                {
                   pic32mx_ep0transfer(priv, regval);
                }
              else
                {
                  pic32mx_eptransfer(priv, epno);
                }
            } 
          else
            {
               /* USTAT FIFO must be empty. */

               break;
            }
        }
    }

  /* Clear the pending USB interrupt */

  up_clrpend_irq(PIC32MX_IRQSRC_USB);
  usbtrace(TRACE_INTEXIT(PIC32MX_TRACEINTID_INTERRUPT), pending);
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

  /* NOTE: Do not clear UIRbits.ACTVIF here! Reason: ACTVIF is only
   * generated once an IDLEIF has been generated. This is a 1:1 ratio
   * interrupt generation. For every IDLEIF, there will be only one ACTVIF
   * regardless of the number of subsequent bus transitions.  If the ACTIF 
   * is cleared here, a problem could occur. The driver services IDLEIF
   * first because ACTIVIE=0. If this routine clears the only ACTIVIF,
   * then it can never get out of the suspend mode.
   */

  regval = pic32mx_getreg(PIC32MX_USBOTG_IE);
  regval |= USBOTG_INT_ACTV;
  pic32mx_putreg(regval, PIC32MX_USBOTG_IE);

  pic32mx_putreg(USB_INT_IDLE, PIC32MX_USB_IR);
   
  /* At this point it would be appropriate to invoke a callback to enter
   * into sleep or idle modes or switch to a slower clock, etc.
   *
   * Let the board-specific logic know that we have entered the suspend
   * state
   */ 

  pic32mx_usbsuspend((struct usbdev_s *)priv, false);
} 

/****************************************************************************
 * Name: pic32mx_resume
 ****************************************************************************/

static void pic32mx_resume(struct pic32mx_usbdev_s *priv) 
{
  uint16_t regval;

  /* Start RESUME signaling */

  regval = pic32mx_getreg(PIC32MX_USB_CON);
  regval |= USB_CON_RESUME;
  pic32mx_putreg(regval, PIC32MX_USBOTG_IE);

  /* Keep the RESUME line for 1-13 ms */

  up_mdelay(10);

  regval &~= USB_CON_RESUME;
  pic32mx_putreg(regval, PIC32MX_USBOTG_IE);

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
 * Name: pic32mx_epallocpma
 ****************************************************************************/

static int pic32mx_epallocpma(struct pic32mx_usbdev_s *priv)
{
  irqstate_t flags;
  int bufno = ERROR;
  int bufndx;

  flags = irqsave();
  for (bufndx = 2; bufndx < PIC32MX_NBUFFERS; bufndx++)
    {
      /* Check if this buffer is available */

      uint8_t bit = PIC32MX_BUFFER_BIT(bufndx);
      if ((priv->bufavail & bit) != 0)
        {
          /* Yes.. Mark the endpoint no longer available */

          priv->bufavail &= ~bit;

          /* And return the index of the allocated buffer */

          bufno = bufndx;
          break;
        }
    }

  irqrestore(flags);
  return bufno;
}

/****************************************************************************
 * Name: pic32mx_epfreepma
 ****************************************************************************/

static inline void
pic32mx_epfreepma(struct pic32mx_usbdev_s *priv, struct pic32mx_ep_s *privep)
{
  irqstate_t flags = irqsave();
  priv->epavail   |= PIC32MX_ENDP_BIT(privep->bufno);
  irqrestore(flags);
}

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
  uint16_t pma;
  uint16_t setting;
  uint16_t maxpacket;
  uint8_t  epno;

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
  usbtrace(TRACE_EPCONFIGURE, (uint16_t)epno);
  DEBUGASSERT(epno == USB_EPNO(ep->eplog));

  /* Set the requested type */

  switch (desc->attr & USB_EP_ATTR_XFERTYPE_MASK)
   {
    case USB_EP_ATTR_XFER_INT: /* Interrupt endpoint */
      setting = USB_EPR_EPTYPE_INTERRUPT;
      break;

    case USB_EP_ATTR_XFER_BULK: /* Bulk endpoint */
      setting = USB_EPR_EPTYPE_BULK;
      break;

    case USB_EP_ATTR_XFER_ISOC: /* Isochronous endpoint */
#warning "REVISIT: Need to review isochronous EP setup"
      setting = USB_EPR_EPTYPE_ISOC;
      break;

    case USB_EP_ATTR_XFER_CONTROL: /* Control endpoint */
      setting = USB_EPR_EPTYPE_CONTROL;
      break;

    default:
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_BADEPTYPE), (uint16_t)desc->type);
      return -EINVAL;
    }

  pic32mx_seteptype(epno, setting);

  /* Get the address of the PMA buffer allocated for this endpoint */

#warning "REVISIT: Should configure BULK EPs using double buffer feature"
  pma = PIC32MX_BUFNO2BUF(privep->bufno);

  /* Get the maxpacket size of the endpoint. */

  maxpacket = GETUINT16(desc->mxpacketsize);
  DEBUGASSERT(maxpacket <= PIC32MX_MAXPACKET_SIZE);
  ep->maxpacket = maxpacket;

  /* Get the subset matching the requested direction */

  if (USB_ISEPIN(desc->addr))
    {
      /* The full, logical EP number includes direction */
 
      ep->eplog = USB_EPIN(epno);

      /* Set up TX; disable RX */

      pic32mx_seteptxaddr(epno, pma);
      pic32mx_seteptxstatus(epno, USB_EPR_STATTX_NAK);
      pic32mx_seteprxstatus(epno, USB_EPR_STATRX_DIS);
    }
  else
    {
      /* The full, logical EP number includes direction */

      ep->eplog = USB_EPOUT(epno);

      /* Set up RX; disable TX */

      pic32mx_seteprxaddr(epno, pma);
      pic32mx_seteprxcount(epno, maxpacket);
      pic32mx_seteprxstatus(epno, USB_EPR_STATRX_VALID);
      pic32mx_seteptxstatus(epno, USB_EPR_STATTX_DIS);
    }

   pic32mx_dumpep(epno);
   return OK;
}

/****************************************************************************
 * Name: pic32mx_epdisable
 ****************************************************************************/

static int pic32mx_epdisable(struct usbdev_ep_s *ep)
{
  struct pic32mx_ep_s *privep = (struct pic32mx_ep_s *)ep;
  irqstate_t flags;
  uint8_t epno;

#ifdef CONFIG_DEBUG
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_INVALIDPARMS), 0);
      ulldbg("ERROR: ep=%p\n", ep);
      return -EINVAL;
    }
#endif

  epno = USB_EPNO(ep->eplog);
  usbtrace(TRACE_EPDISABLE, epno);

  /* Cancel any ongoing activity */

  flags = irqsave();
  pic32mx_cancelrequests(privep);

  /* Disable TX; disable RX */

  pic32mx_seteprxcount(epno, 0);
  pic32mx_seteprxstatus(epno, USB_EPR_STATRX_DIS);
  pic32mx_seteptxstatus(epno, USB_EPR_STATTX_DIS);

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

  epno        = USB_EPNO(ep->eplog);
  req->result = -EINPROGRESS;
  req->xfrd   = 0;
  flags       = irqsave();

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
          priv->txstatus = USB_EPR_STATTX_NAK;
          ret            = pic32mx_wrrequest(priv, privep);

          /* Set the new TX status */

          pic32mx_seteptxstatus(epno, priv->txstatus);
        }
    }

  /* Handle OUT (host-to-device) requests */

  else
    {
      /* Add the new request to the request queue for the OUT endpoint */

      privep->txnullpkt = 0;
      pic32mx_rqenqueue(privep, privreq);
      usbtrace(TRACE_OUTREQQUEUED(epno), req->len);

      /* This there a incoming data pending the availability of a request? */

      if (priv->rxpending)
        {
          /* Set STAT_RX bits to '11' in the USB_EPnR, enabling further
           * transactions. "While the STAT_RX bits are equal to '10'
           * (NAK), any OUT request addressed to that endpoint is NAKed,
           * indicating a flow control condition: the USB host will retry
           * the transaction until it succeeds."
           */

          priv->rxstatus  = USB_EPR_STATRX_VALID;
          pic32mx_seteprxstatus(epno, priv->rxstatus);

          /* Data is no longer pending */

          priv->rxpending = false;
        }
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
 * Name: pic32mx_epstall
 ****************************************************************************/

static int pic32mx_epstall(struct usbdev_ep_s *ep, bool resume)
{
  struct pic32mx_ep_s *privep;
  struct pic32mx_usbdev_s *priv;
  uint8_t epno = USB_EPNO(ep->eplog);
  uint16_t status;
  irqstate_t flags;

#ifdef CONFIG_DEBUG
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif
  privep = (struct pic32mx_ep_s *)ep;
  priv   = (struct pic32mx_usbdev_s *)privep->dev;
  epno   = USB_EPNO(ep->eplog);

  /* STALL or RESUME the endpoint */

  flags = irqsave();
  usbtrace(resume ? TRACE_EPRESUME : TRACE_EPSTALL, USB_EPNO(ep->eplog));

  /* Get status of the endpoint; stall the request if the endpoint is
   * disabled
   */
 
  if (USB_ISEPIN(ep->eplog))
    {
      status = pic32mx_geteptxstatus(epno);
    }
  else
    {
      status = pic32mx_geteprxstatus(epno);
    }

  if (status == 0)
    {
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_EPDISABLED), 0);
      priv->ctrlstate = CTRLSTATE_STALLED;
      return -ENODEV;
    }

  /* Handle the resume condition */

  if (resume)
    {
      /* Resuming a stalled endpoint */

      usbtrace(TRACE_EPRESUME, epno);
      privep->stalled = false;

      if (USB_ISEPIN(ep->eplog))
        {
          /* IN endpoint */ 

          if (pic32mx_eptxstalled(epno))
            {
              pic32mx_clrtxdtog(epno);

              /* Restart any queued write requests */

              priv->txstatus = USB_EPR_STATTX_NAK;
              (void)pic32mx_wrrequest(priv, privep);

              /* Set the new TX status */

              pic32mx_seteptxstatus(epno, priv->txstatus);
            }
        }
      else
        {
          /* OUT endpoint */ 

          if (pic32mx_eprxstalled(epno))
            {
              if (epno == EP0)
                {
                  /* After clear the STALL, enable the default endpoint receiver */

                  pic32mx_seteprxcount(epno, ep->maxpacket);
                }
              else
                {
                  pic32mx_clrrxdtog(epno);
                }

              priv->rxstatus = USB_EPR_STATRX_VALID;
              pic32mx_seteprxstatus(epno, USB_EPR_STATRX_VALID);
            }
        }  
    }

  /* Handle the stall condition */

  else
    {
      usbtrace(TRACE_EPSTALL, epno);
      privep->stalled = true;

      if (USB_ISEPIN(ep->eplog))
        {
          /* IN endpoint */ 

          priv->txstatus = USB_EPR_STATTX_STALL;
          pic32mx_seteptxstatus(epno, USB_EPR_STATTX_STALL);
        }
      else
        {
          /* OUT endpoint */ 

          priv->rxstatus = USB_EPR_STATRX_STALL;
          pic32mx_seteprxstatus(epno, USB_EPR_STATRX_STALL);
        }
    }

  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Device Controller Operations
 ****************************************************************************/
/****************************************************************************
 * Name: pic32mx_allocep
 ****************************************************************************/

static struct usbdev_ep_s *pic32mx_allocep(struct usbdev_s *dev, uint8_t epno,
                                         bool in, uint8_t eptype)
{
  struct pic32mx_usbdev_s *priv = (struct pic32mx_usbdev_s *)dev;
  struct pic32mx_ep_s *privep = NULL;
  uint8_t epset = PIC32MX_ENDP_ALLSET;
  int bufno;

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
      goto errout;
    }
  epno = USB_EPNO(privep->ep.eplog);

  /* Allocate a PMA buffer for this endpoint */

#warning "REVISIT: Should configure BULK EPs using double buffer feature"
  bufno = pic32mx_epallocpma(priv);
  if (bufno < 0)
    {
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_EPBUFFER), 0);
      goto errout_with_ep;
    }
  privep->bufno = (uint8_t)bufno;
  return &privep->ep;

errout_with_ep:
  pic32mx_epunreserve(priv, privep);
errout:
  return NULL;
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

  if (priv && privep)
    {
      /* Free the PMA buffer assigned to this endpoint */

      pic32mx_epfreepma(priv, privep);

      /* Mark the endpoint as available */

      pic32mx_epunreserve(priv, privep);
    }
}

/****************************************************************************
 * Name: pic32mx_getframe
 ****************************************************************************/

static int pic32mx_getframe(struct usbdev_s *dev)
{
  uint16_t fnr;

#ifdef CONFIG_DEBUG
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Return the last frame number detected by the hardware */

  fnr = pic32mx_getreg(PIC32MX_USB_FNR);
  usbtrace(TRACE_DEVGETFRAME, fnr);
  return (fnr & USB_FNR_FN_MASK);
}

/****************************************************************************
 * Name: pic32mx_wakeup
 ****************************************************************************/

static int pic32mx_wakeup(struct usbdev_s *dev)
{
  struct pic32mx_usbdev_s *priv = (struct pic32mx_usbdev_s *)dev;
  irqstate_t flags;

  usbtrace(TRACE_DEVWAKEUP, 0);
#ifdef CONFIG_DEBUG
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Start the resume sequence.  The actual resume steps will be driven
   * by the ESOF interrupt.
   */

  flags = irqsave();
  pic32mx_resume(priv);

  /* Disable the SUSP interrupt (until we are fully resumed), disable
   * the WKUP interrupt (we are already waking up), and enable the
   * ESOF interrupt that will drive the resume operations.  Clear any
   * pending ESOF interrupt.
   */

  pic32mx_setimask(priv, USB_CNTR_ESOFM, USB_CNTR_WKUPM|USB_CNTR_SUSPM);
  pic32mx_putreg(~USB_ISTR_ESOF, PIC32MX_USB_ISTR);
  irqrestore(flags);
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

  /* Put the USB controller in reset, disable all interrupts */

  pic32mx_putreg(USB_CNTR_FRES, PIC32MX_USB_CNTR);

  /* Tell the class driver that we are disconnected.  The class driver
   * should then accept any new configurations.
   */

  CLASS_DISCONNECT(priv->driver, &priv->usbdev);

  /* Reset the device state structure */

  priv->ctrlstate  = CTRLSTATE_IDLE;
  priv->rxpending = false;

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
      /* Initialize registers to known states. */

      pic32mx_putreg(0, PIC32MX_USB_CON);
       
      /* Mask all USB interrupts */

      pic32mx_putreg(0, PIC32MX_USB_IE);

      /* Configure things like: pull ups, full/low-speed mode, 
       * set the ping pong mode, and set internal transceiver
       */

      pic32mx_putreg(0, PIC32MX_USB_CNFG1);
      pic32mx_putreg(ERROR_INTERRUPTS, PIC32MX_USB_EIE);
      pic32mx_putreg(NORMAL_INTERRUPTS, PIC32MX_USB_IE);

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

      /* Enable USB interrupts */

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

  priv->devstate = DEVSTATE_DETACHED;
       
#ifdef CONFIG_USBOTG    
  /* Disable the D+ Pullup */

  U1OTGCONbits.DPPULUP = 0;
       
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
  uint16_t regval;
  int i;

  /* Clear all pending USB interrupts */

  pic32mx_putreg(USB_EINT_ALL, PIC32MX_USB_EIR);
  pic32mx_putreg(USB_INT_ALL, PIC32MX_USB_IR);

  /* Reset configuration and interrrupt enabled */

  pic32mx_putreg(0, PIC32MX_USB_CNFG1);
  pic32mx_putreg(ERROR_INTERRUPTS, PIC32MX_USB_EIE);
  pic32mx_putreg(NORMAL_INTERRUPTS, PIC32MX_USB_EIE);
  
  /* Power up the USB module */

  regval = pic32mx_getreg(PIC32MX_USB_PWRC);
  regval |= USB_PWRC_USBPWR;
  pic32mx_putreg(regval, PIC32MX_USB_PWRC);

  /* Set the address of the buffer descriptor table (BDT) */

  regval = (uint16_t)((PHYS_ADDR(g_bdt)) >> 8);
  pic32mx_putreg(regval, PIC32MX_USB_BDTP1);

  /* Assert reset request to all of the Ping Pong buffer pointers */

  regval  = pic32mx_getreg(PIC32MX_USB_CON);
  regval |= USB_CON_PPBRST;
  pic32mx_putreg(regval, PIC32MX_USB_CON);

  /* Reset to default address */

  pic32mx_putreg(0, PIC32MX_USB_ADDR);
   
  /* Clear all of the endpoint control registers */

  for (i = 0; i < NEP_REGISTERS; i++)
    {
      pic32mx_putreg(0, PIC32MX_USB_EP(i));
    }
   
  /* Bring the ping pong buffer pointers out of reset */

  regval &= ~USB_CON_PPBRST;
  pic32mx_putreg(regval, IC32MX_USB_CON);

  /* Clear all of the buffer descriptor table (BDT) entries */

  memset((void*)g_bdt, 0, sizeof(g_bdt));
   
  /* Initialize EP0 as a Ctrl EP */

  pic32mx_putreg(EP_CTRL | USB_EP_EPHSHK, PIC32MX_USB_EP0);

  /* Flush any pending transactions */

  while ((pic32mx_getreg(PIC32MX_USB_IR) & USB_INT_TRN) != 0)
    {
      pic32mx_putreg(USB_INT_TRN, PIC32MX_USB_IR);
    }
 
  /* Make sure packet processing is enabled */

  regval = pic32mx_getreg(PIC32MX_USB_CON);
  regval &= ~USB_CON_PKTDIS;
  pic32mx_putreg(regval, PIC32MX_USB_CON);

  /* Get ready for the first packet */

  privep->bdt = &g_bdt[EP0_IN_EVEN];
   
  /* Clear active configuration */

  priv->config = 0;
   
  /* Indicate that we are now in the detached state  */

  priv->devstate = DEVSTATE_DETACHED;
  pic32mx_dumpep(EP0);
}

/****************************************************************************
 * Name: pic32mx_hwsetup
 ****************************************************************************/

static void pic32mx_hwsetup(struct pic32mx_usbdev_s *priv)
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
  priv->bufavail     = PIC32MX_BUFFER_ALLSET & ~PIC32MX_BUFFER_EP0;

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

      /* Clear all of the internal pipe information */

      privep->u.inpipe.inuse   = 0;
      privep->u.outpipe.inuse  = 0;
      privep->u.outpipe.wcount = 0;
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
  priv->usbdev.speed = USB_SPEED_UNKNOWN;

  /* Disable all interrupts and force the USB controller into reset */ 

  pic32mx_putreg(USB_CNTR_FRES, PIC32MX_USB_CNTR);

  /* Clear any pending interrupts */ 

  pic32mx_putreg(0, PIC32MX_USB_ISTR);

  /* Disconnect the device / disable the pull-up */ 

  pic32mx_usbpullup(&priv->usbdev, false);
  
  /* Power down the USB controller */

  pic32mx_putreg(USB_CNTR_FRES|USB_CNTR_PDWN, PIC32MX_USB_CNTR);
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

  /* Power up the USB controller, but leave it in the reset state */

  pic32mx_hwsetup(priv);

  /* Attach USB controller interrupt handlers.  The hardware will not be
   * initialized and interrupts will not be enabled until the class device
   * driver is bound.  Getting the IRQs here only makes sure that we have
   * them when we need them later.
   */

  if (irq_attach(PIC32MX_IRQ_USB, pic32mx_interrupt) != 0)
    {
      usbtrace(TRACE_DEVERROR(PIC32MX_TRACEERR_IRQREGISTRATION),
               (uint16_t)PIC32MX_IRQ_USB);
      goto errout;
    }

errout:
  up_usbuninitialize();
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
 *   Register a USB device class driver. The class driver's bind() method will be
 *   called to bind it to a USB device driver.
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
  else
    {
      /* Setup the USB controller in it initial unconnected state */

      pic32mx_hwreset(priv);
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
  pic32mx_hwsetup(priv);

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
  pic32mx_attach(priv);
}

void pic32mx_usbdetach(void)
{
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple USB controllers
   * easier.
   */

  struct pic32mx_usbdev_s *priv = &g_usbdev;
  pic32mx_detach(priv);
}

#endif /* CONFIG_USBDEV && CONFIG_PIC32MX_USB */
