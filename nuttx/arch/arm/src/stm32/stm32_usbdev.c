/****************************************************************************
 * arch/arm/src/stm32/stm32_usbdev.c
 *
 *   Copyright (C) 2009-2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.orgr>
 *
 * References:
 *   - RM0008 Reference manual, STMicro document ID 13902
 *   - STM32F10xxx USB development kit, UM0424, STMicro
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
#include "stm32.h"
#include "stm32_syscfg.h"
#include "stm32_gpio.h"
#include "stm32_usbdev.h"

#if defined(CONFIG_USBDEV) && defined(CONFIG_STM32_USB)

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

/* USB Interrupts.  Should be re-mapped if CAN is used. */

#ifdef CONFIG_STM32_STM32F30XX
#  ifdef CONFIG_STM32_USB_ITRMP
#    define STM32_IRQ_USBHP   STM32_IRQ_USBHP_2
#    define STM32_IRQ_USBLP   STM32_IRQ_USBLP_2
#    define STM32_IRQ_USBWKUP STM32_IRQ_USBWKUP_2
#  else
#    define STM32_IRQ_USBHP   STM32_IRQ_USBHP_1
#    define STM32_IRQ_USBLP   STM32_IRQ_USBLP_1
#    define STM32_IRQ_USBWKUP STM32_IRQ_USBWKUP_1
#  endif
#endif

/* Extremely detailed register debug that you would normally never want
 * enabled.
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_STM32_USBDEV_REGDEBUG
#endif

/* Initial interrupt mask: Reset + Suspend + Correct Transfer */

#define STM32_CNTR_SETUP     (USB_CNTR_RESETM|USB_CNTR_SUSPM|USB_CNTR_CTRM)

/* Endpoint identifiers. The STM32 supports up to 16 mono-directional or 8
 * bidirectional endpoints.  However, when you take into account PMA buffer
 * usage (see below) and the fact that EP0 is bidirectional, then there is
 * a functional limitation of EP0 + 5 mono-directional endpoints = 6.  We'll
 * define STM32_NENDPOINTS to be 8, however, because that is how many
 * endpoint register sets there are.
 */

#define STM32_NENDPOINTS      (8)
#define EP0                   (0)
#define EP1                   (1)
#define EP2                   (2)
#define EP3                   (3)
#define EP4                   (4)
#define EP5                   (5)
#define EP6                   (6)
#define EP7                   (7)

#define STM32_ENDP_BIT(ep)    (1 << (ep))
#define STM32_ENDP_ALLSET     0xff

/* Packet sizes.  We us a fixed 64 max packet size for all endpoint types */

#define STM32_MAXPACKET_SHIFT (6)
#define STM32_MAXPACKET_SIZE  (1 << (STM32_MAXPACKET_SHIFT))
#define STM32_MAXPACKET_MASK  (STM32_MAXPACKET_SIZE-1)

#define STM32_EP0MAXPACKET    STM32_MAXPACKET_SIZE 

/* Buffer descriptor table.  We assume that USB has exclusive use of CAN/USB
 * memory.  The buffer table is positioned at the beginning of the 512-byte
 * CAN/USB memory.  We will use the first STM32_NENDPOINTS*4 words for the buffer
 * table.  That is exactly 64 bytes, leaving 7*64 bytes for endpoint buffers.
 */

#define STM32_BTABLE_ADDRESS  (0x00)   /* Start at the beginning of USB/CAN RAM */
#define STM32_DESC_SIZE       (8)      /* Each descriptor is 4*2=8 bytes in size */
#define STM32_BTABLE_SIZE     (STM32_NENDPOINTS*STM32_DESC_SIZE)

/* Buffer layout.  Assume that all buffers are 64-bytes (maxpacketsize), then
 * we have space for only 7 buffers; endpoint 0 will require two buffers, leaving
 * 5 for other endpoints.
 */

#define STM32_BUFFER_START    STM32_BTABLE_SIZE
#define STM32_EP0_RXADDR      STM32_BUFFER_START
#define STM32_EP0_TXADDR      (STM32_EP0_RXADDR+STM32_EP0MAXPACKET)

#define STM32_BUFFER_EP0      0x03
#define STM32_NBUFFERS        7
#define STM32_BUFFER_BIT(bn)  (1 << (bn))
#define STM32_BUFFER_ALLSET   0x7f
#define STM32_BUFNO2BUF(bn)   (STM32_BUFFER_START+((bn)<<STM32_MAXPACKET_SHIFT))

/* USB-related masks */

#define REQRECIPIENT_MASK     (USB_REQ_TYPE_MASK | USB_REQ_RECIPIENT_MASK)

/* Endpoint register masks (handling toggle fields) */

#define EPR_NOTOG_MASK        (USB_EPR_CTR_RX  | USB_EPR_SETUP  | USB_EPR_EPTYPE_MASK |\
                               USB_EPR_EP_KIND | USB_EPR_CTR_TX | USB_EPR_EA_MASK)
#define EPR_TXDTOG_MASK       (USB_EPR_STATTX_MASK | EPR_NOTOG_MASK)
#define EPR_RXDTOG_MASK       (USB_EPR_STATRX_MASK | EPR_NOTOG_MASK)

/* Request queue operations *************************************************/

#define stm32_rqempty(ep)     ((ep)->head == NULL)
#define stm32_rqpeek(ep)      ((ep)->head)

/* USB trace ****************************************************************/
/* Trace error codes */

#define STM32_TRACEERR_ALLOCFAIL            0x0001
#define STM32_TRACEERR_BADCLEARFEATURE      0x0002
#define STM32_TRACEERR_BADDEVGETSTATUS      0x0003
#define STM32_TRACEERR_BADEPGETSTATUS       0x0004
#define STM32_TRACEERR_BADEPNO              0x0005
#define STM32_TRACEERR_BADEPTYPE            0x0006
#define STM32_TRACEERR_BADGETCONFIG         0x0007
#define STM32_TRACEERR_BADGETSETDESC        0x0008
#define STM32_TRACEERR_BADGETSTATUS         0x0009
#define STM32_TRACEERR_BADSETADDRESS        0x000a
#define STM32_TRACEERR_BADSETCONFIG         0x000b
#define STM32_TRACEERR_BADSETFEATURE        0x000c
#define STM32_TRACEERR_BINDFAILED           0x000d
#define STM32_TRACEERR_DISPATCHSTALL        0x000e
#define STM32_TRACEERR_DRIVER               0x000f
#define STM32_TRACEERR_DRIVERREGISTERED     0x0010
#define STM32_TRACEERR_EP0BADCTR            0x0011
#define STM32_TRACEERR_EP0SETUPSTALLED      0x0012
#define STM32_TRACEERR_EPBUFFER             0x0013
#define STM32_TRACEERR_EPDISABLED           0x0014
#define STM32_TRACEERR_EPOUTNULLPACKET      0x0015
#define STM32_TRACEERR_EPRESERVE            0x0016
#define STM32_TRACEERR_INVALIDCTRLREQ       0x0017
#define STM32_TRACEERR_INVALIDPARMS         0x0018
#define STM32_TRACEERR_IRQREGISTRATION      0x0019
#define STM32_TRACEERR_NOTCONFIGURED        0x001a
#define STM32_TRACEERR_REQABORTED           0x001b

/* Trace interrupt codes */

#define STM32_TRACEINTID_CLEARFEATURE       0x0001
#define STM32_TRACEINTID_DEVGETSTATUS       0x0002
#define STM32_TRACEINTID_DISPATCH           0x0003
#define STM32_TRACEINTID_EP0IN              0x0004
#define STM32_TRACEINTID_EP0INDONE          0x0005
#define STM32_TRACEINTID_EP0OUTDONE         0x0006
#define STM32_TRACEINTID_EP0SETUPDONE       0x0007
#define STM32_TRACEINTID_EP0SETUPSETADDRESS 0x0008
#define STM32_TRACEINTID_EPGETSTATUS        0x0009
#define STM32_TRACEINTID_EPINDONE           0x000a
#define STM32_TRACEINTID_EPINQEMPTY         0x000b
#define STM32_TRACEINTID_EPOUTDONE          0x000c
#define STM32_TRACEINTID_EPOUTPENDING       0x000d
#define STM32_TRACEINTID_EPOUTQEMPTY        0x000e
#define STM32_TRACEINTID_ESOF               0x000f
#define STM32_TRACEINTID_GETCONFIG          0x0010
#define STM32_TRACEINTID_GETSETDESC         0x0011
#define STM32_TRACEINTID_GETSETIF           0x0012
#define STM32_TRACEINTID_GETSTATUS          0x0013
#define STM32_TRACEINTID_HPINTERRUPT        0x0014
#define STM32_TRACEINTID_IFGETSTATUS        0x0015
#define STM32_TRACEINTID_LPCTR              0x0016
#define STM32_TRACEINTID_LPINTERRUPT        0x0017
#define STM32_TRACEINTID_NOSTDREQ           0x0018
#define STM32_TRACEINTID_RESET              0x0019
#define STM32_TRACEINTID_SETCONFIG          0x001a
#define STM32_TRACEINTID_SETFEATURE         0x001b
#define STM32_TRACEINTID_SUSP               0x001c
#define STM32_TRACEINTID_SYNCHFRAME         0x001d
#define STM32_TRACEINTID_WKUP               0x001e

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

/* The various states of a control pipe */

enum stm32_ep0state_e 
{
  EP0STATE_IDLE = 0,        /* No request in progress */
  EP0STATE_RDREQUEST,       /* Read request in progress */
  EP0STATE_WRREQUEST,       /* Write request in progress */
  EP0STATE_STALLED          /* We are stalled */
};

/* Resume states */

enum stm32_rsmstate_e 
{
  RSMSTATE_IDLE = 0,        /* Device is either fully suspended or running */
  RSMSTATE_STARTED,         /* Resume sequence has been started */
  RSMSTATE_WAITING          /* Waiting (on ESOFs) for end of sequence */
};

union wb_u
{
  uint16_t w;
  uint8_t  b[2];
};

/* A container for a request so that the request make be retained in a list */

struct stm32_req_s
{
  struct usbdev_req_s  req;             /* Standard USB request */
  struct stm32_req_s  *flink;           /* Supports a singly linked list */
};

/* This is the internal representation of an endpoint */

struct stm32_ep_s
{
  /* Common endpoint fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_ep_s
   * to struct stm32_ep_s.
   */

  struct usbdev_ep_s      ep;           /* Standard endpoint structure */

  /* STR71X-specific fields */

  struct stm32_usbdev_s *dev;           /* Reference to private driver data */
  struct stm32_req_s    *head;          /* Request list for this endpoint */
  struct stm32_req_s    *tail;
  uint8_t                bufno;         /* Allocated buffer number */
  uint8_t                stalled:1;     /* true: Endpoint is stalled */
  uint8_t                halted:1;      /* true: Endpoint feature halted */
  uint8_t                txbusy:1;      /* true: TX endpoint FIFO full */
  uint8_t                txnullpkt:1;   /* Null packet needed at end of transfer */
};

struct stm32_usbdev_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_s
   * to structstm32_usbdev_s.
   */

  struct usbdev_s usbdev;

  /* The bound device class driver */

  struct usbdevclass_driver_s *driver;

  /* STM32-specific fields */

  struct usb_ctrlreq_s     ctrl;          /* Last EP0 request */
  uint8_t                  ep0state;      /* State of EP0 (see enum stm32_ep0state_e) */
  uint8_t                  rsmstate;      /* Resume state (see enum stm32_rsmstate_e) */
  uint8_t                  nesofs;        /* ESOF counter (for resume support) */
  uint8_t                  rxpending:1;   /* 1: OUT data in PMA, but no read requests */
  uint8_t                  selfpowered:1; /* 1: Device is self powered */
  uint8_t                  epavail;       /* Bitset of available endpoints */
  uint8_t                  bufavail;      /* Bitset of available buffers */
  uint16_t                 rxstatus;      /* Saved during interrupt processing */
  uint16_t                 txstatus;      /* "   " "    " "       " "        " */
  uint16_t                 imask;         /* Current interrupt mask */

  /* The endpoint list */

  struct stm32_ep_s        eplist[STM32_NENDPOINTS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

#if defined(CONFIG_STM32_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
static uint16_t stm32_getreg(uint32_t addr);
static void stm32_putreg(uint16_t val, uint32_t addr);
static void stm32_checksetup(void);
static void stm32_dumpep(int epno);
#else
# define stm32_getreg(addr)      getreg16(addr)
# define stm32_putreg(val,addr)  putreg16(val,addr)
# define stm32_checksetup()
# define stm32_dumpep(epno)
#endif

/* Low-Level Helpers ********************************************************/

static inline void
              stm32_seteptxcount(uint8_t epno, uint16_t count);
static inline void
              stm32_seteptxaddr(uint8_t epno, uint16_t addr);
static inline uint16_t
              stm32_geteptxaddr(uint8_t epno);
static void   stm32_seteprxcount(uint8_t epno, uint16_t count);
static inline uint16_t
              stm32_geteprxcount(uint8_t epno);
static inline void
              stm32_seteprxaddr(uint8_t epno, uint16_t addr);
static inline uint16_t
              stm32_geteprxaddr(uint8_t epno);
static inline void
              stm32_setepaddress(uint8_t epno, uint16_t addr);
static inline void
              stm32_seteptype(uint8_t epno, uint16_t type);
static inline void
              stm32_seteptxaddr(uint8_t epno, uint16_t addr);
static inline void
              stm32_setstatusout(uint8_t epno);
static inline void
              stm32_clrstatusout(uint8_t epno);
static void   stm32_clrrxdtog(uint8_t epno);
static void   stm32_clrtxdtog(uint8_t epno);
static void   stm32_clrepctrrx(uint8_t epno);
static void   stm32_clrepctrtx(uint8_t epno);
static void   stm32_seteptxstatus(uint8_t epno, uint16_t state);
static void   stm32_seteprxstatus(uint8_t epno, uint16_t state);
static inline uint16_t
              stm32_geteptxstatus(uint8_t epno);
static inline uint16_t
              stm32_geteprxstatus(uint8_t epno);
static bool   stm32_eptxstalled(uint8_t epno);
static bool   stm32_eprxstalled(uint8_t epno);
static void   stm32_setimask(struct stm32_usbdev_s *priv, uint16_t setbits,
                uint16_t clrbits);

/* Suspend/Resume Helpers ***************************************************/

static void   stm32_suspend(struct stm32_usbdev_s *priv);
static void   stm32_initresume(struct stm32_usbdev_s *priv);
static void   stm32_esofpoll(struct stm32_usbdev_s *priv) ;

/* Request Helpers **********************************************************/

static void   stm32_copytopma(const uint8_t *buffer, uint16_t pma,
                uint16_t nbytes);
static inline void
              stm32_copyfrompma(uint8_t *buffer, uint16_t pma, uint16_t nbytes);
static struct stm32_req_s *
              stm32_rqdequeue(struct stm32_ep_s *privep);
static void   stm32_rqenqueue(struct stm32_ep_s *privep,
                struct stm32_req_s *req);
static inline void
              stm32_abortrequest(struct stm32_ep_s *privep,
                struct stm32_req_s *privreq, int16_t result);
static void   stm32_reqcomplete(struct stm32_ep_s *privep, int16_t result);
static void   stm32_epwrite(struct stm32_usbdev_s *buf,
                struct stm32_ep_s *privep, const uint8_t *data, uint32_t nbytes);
static int    stm32_wrrequest(struct stm32_usbdev_s *priv,
                struct stm32_ep_s *privep);
static int    stm32_rdrequest(struct stm32_usbdev_s *priv,
                struct stm32_ep_s *privep);
static void   stm32_cancelrequests(struct stm32_ep_s *privep);

/* Interrupt level processing ***********************************************/

static void   stm32_dispatchrequest(struct stm32_usbdev_s *priv);
static void   stm32_epdone(struct stm32_usbdev_s *priv, uint8_t epno);
static void   stm32_setdevaddr(struct stm32_usbdev_s *priv, uint8_t value);
static void   stm32_ep0setup(struct stm32_usbdev_s *priv);
static void   stm32_ep0out(struct stm32_usbdev_s *priv);
static void   stm32_ep0in(struct stm32_usbdev_s *priv);
static inline void
              stm32_ep0done(struct stm32_usbdev_s *priv, uint16_t istr);
static void   stm32_lptransfer(struct stm32_usbdev_s *priv);
static int    stm32_hpinterrupt(int irq, void *context);
static int    stm32_lpinterrupt(int irq, void *context);

/* Endpoint helpers *********************************************************/

static inline struct stm32_ep_s *
              stm32_epreserve(struct stm32_usbdev_s *priv, uint8_t epset);
static inline void
              stm32_epunreserve(struct stm32_usbdev_s *priv,
                struct stm32_ep_s *privep);
static inline bool
              stm32_epreserved(struct stm32_usbdev_s *priv, int epno);
static int    stm32_epallocpma(struct stm32_usbdev_s *priv);
static inline void
              stm32_epfreepma(struct stm32_usbdev_s *priv,
                struct stm32_ep_s *privep);

/* Endpoint operations ******************************************************/

static int    stm32_epconfigure(struct usbdev_ep_s *ep,
                const struct usb_epdesc_s *desc, bool last);
static int    stm32_epdisable(struct usbdev_ep_s *ep);
static struct usbdev_req_s *
              stm32_epallocreq(struct usbdev_ep_s *ep);
static void   stm32_epfreereq(struct usbdev_ep_s *ep,
                struct usbdev_req_s *);
static int    stm32_epsubmit(struct usbdev_ep_s *ep,
                struct usbdev_req_s *req);
static int    stm32_epcancel(struct usbdev_ep_s *ep,
                struct usbdev_req_s *req);
static int    stm32_epstall(struct usbdev_ep_s *ep, bool resume);

/* USB device controller operations *****************************************/

static struct usbdev_ep_s *
              stm32_allocep(struct usbdev_s *dev, uint8_t epno, bool in,
                uint8_t eptype);
static void   stm32_freeep(struct usbdev_s *dev, struct usbdev_ep_s *ep);
static int    stm32_getframe(struct usbdev_s *dev);
static int    stm32_wakeup(struct usbdev_s *dev);
static int    stm32_selfpowered(struct usbdev_s *dev, bool selfpowered);

/* Initialization/Reset *****************************************************/

static void   stm32_reset(struct stm32_usbdev_s *priv);
static void   stm32_hwreset(struct stm32_usbdev_s *priv);
static void   stm32_hwsetup(struct stm32_usbdev_s *priv);
static void   stm32_hwshutdown(struct stm32_usbdev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Since there is only a single USB interface, all status information can be
 * be simply retained in a single global instance.
 */

static struct stm32_usbdev_s g_usbdev;

static const struct usbdev_epops_s g_epops =
{
  .configure   = stm32_epconfigure,
  .disable     = stm32_epdisable,
  .allocreq    = stm32_epallocreq,
  .freereq     = stm32_epfreereq,
  .submit      = stm32_epsubmit,
  .cancel      = stm32_epcancel,
  .stall       = stm32_epstall,
};

static const struct usbdev_ops_s g_devops =
{
  .allocep     = stm32_allocep,
  .freeep      = stm32_freeep,
  .getframe    = stm32_getframe,
  .wakeup      = stm32_wakeup,
  .selfpowered = stm32_selfpowered,
  .pullup      = stm32_usbpullup,
};

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
 * Name: stm32_getreg
 ****************************************************************************/

#if defined(CONFIG_STM32_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
static uint16_t stm32_getreg(uint32_t addr)
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
 * Name: stm32_putreg
 ****************************************************************************/

#if defined(CONFIG_STM32_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
static void stm32_putreg(uint16_t val, uint32_t addr)
{
  /* Show the register value being written */

  lldbg("%08x<-%04x\n", addr, val);

  /* Write the value */

  putreg16(val, addr);
}
#endif

/****************************************************************************
 * Name: stm32_dumpep
 ****************************************************************************/

#if defined(CONFIG_STM32_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
static void stm32_dumpep(int epno)
{
  uint32_t addr;

  /* Common registers */

  lldbg("CNTR:   %04x\n", getreg16(STM32_USB_CNTR));
  lldbg("ISTR:   %04x\n", getreg16(STM32_USB_ISTR));
  lldbg("FNR:    %04x\n", getreg16(STM32_USB_FNR));
  lldbg("DADDR:  %04x\n", getreg16(STM32_USB_DADDR));
  lldbg("BTABLE: %04x\n", getreg16(STM32_USB_BTABLE));

  /* Endpoint register */

  addr = STM32_USB_EPR(epno);
  lldbg("EPR%d:   [%08x] %04x\n", epno, addr, getreg16(addr));

  /* Endpoint descriptor */

  addr = STM32_USB_BTABLE_ADDR(epno, 0);
  lldbg("DESC:   %08x\n", addr);

  /* Endpoint buffer descriptor */

  addr = STM32_USB_ADDR_TX(epno);
  lldbg("  TX ADDR:  [%08x] %04x\n",  addr, getreg16(addr));

  addr = STM32_USB_COUNT_TX(epno);
  lldbg("     COUNT: [%08x] %04x\n",  addr, getreg16(addr));

  addr = STM32_USB_ADDR_RX(epno);
  lldbg("  RX ADDR:  [%08x] %04x\n",  addr, getreg16(addr));

  addr = STM32_USB_COUNT_RX(epno);
  lldbg("     COUNT: [%08x] %04x\n",  addr, getreg16(addr));
}
#endif

/****************************************************************************
 * Name: stm32_checksetup
 ****************************************************************************/

#if defined(CONFIG_STM32_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
static void stm32_checksetup(void)
{
  uint32_t cfgr     = getreg32(STM32_RCC_CFGR);
  uint32_t apb1rstr = getreg32(STM32_RCC_APB1RSTR);
  uint32_t apb1enr  = getreg32(STM32_RCC_APB1ENR);

  lldbg("CFGR: %08x APB1RSTR: %08x APB1ENR: %08x\n", cfgr, apb1rstr, apb1enr);

  if ((apb1rstr & RCC_APB1RSTR_USBRST) != 0 ||
      (apb1enr & RCC_APB1ENR_USBEN) == 0)
    {
      lldbg("ERROR: USB is NOT setup correctly\n");
    }
}
#endif

/****************************************************************************
 * Low-Level Helpers
 ****************************************************************************/
/****************************************************************************
 * Name: stm32_seteptxcount
 ****************************************************************************/

static inline void stm32_seteptxcount(uint8_t epno, uint16_t count) 
{
  volatile uint32_t *epaddr = (uint32_t*)STM32_USB_COUNT_TX(epno);
  *epaddr = count;
} 

/****************************************************************************
 * Name: stm32_seteptxaddr
 ****************************************************************************/

static inline void stm32_seteptxaddr(uint8_t epno, uint16_t addr)
{
  volatile uint32_t *txaddr = (uint32_t*)STM32_USB_ADDR_TX(epno);
  *txaddr = addr;
}

/****************************************************************************
 * Name: stm32_geteptxaddr
 ****************************************************************************/

static inline uint16_t stm32_geteptxaddr(uint8_t epno)
{
  volatile uint32_t *txaddr = (uint32_t*)STM32_USB_ADDR_TX(epno);
  return (uint16_t)*txaddr;
}

/****************************************************************************
 * Name: stm32_seteprxcount
 ****************************************************************************/

static void stm32_seteprxcount(uint8_t epno, uint16_t count) 
{
  volatile uint32_t *epaddr = (uint32_t*)STM32_USB_COUNT_RX(epno);
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
 * Name: stm32_geteprxcount
 ****************************************************************************/

static inline uint16_t stm32_geteprxcount(uint8_t epno)
{
  volatile uint32_t *epaddr = (uint32_t*)STM32_USB_COUNT_RX(epno);
  return (*epaddr) & USB_COUNT_RX_MASK;
}

/****************************************************************************
 * Name: stm32_seteprxaddr
 ****************************************************************************/

static inline void stm32_seteprxaddr(uint8_t epno, uint16_t addr)
{
  volatile uint32_t *rxaddr = (uint32_t*)STM32_USB_ADDR_RX(epno);
  *rxaddr = addr;
}

/****************************************************************************
 * Name: stm32_seteprxaddr
 ****************************************************************************/

static inline uint16_t stm32_geteprxaddr(uint8_t epno)
{
  volatile uint32_t *rxaddr = (uint32_t*)STM32_USB_ADDR_RX(epno);
  return (uint16_t)*rxaddr;
}

/****************************************************************************
 * Name: stm32_setepaddress
 ****************************************************************************/

static inline void stm32_setepaddress(uint8_t epno, uint16_t addr) 
{
  uint32_t epaddr = STM32_USB_EPR(epno);
  uint16_t regval;

  regval  = stm32_getreg(epaddr);
  regval &= EPR_NOTOG_MASK;
  regval &= ~USB_EPR_EA_MASK;
  regval |= (addr << USB_EPR_EA_SHIFT);
  stm32_putreg(regval, epaddr);
} 

/****************************************************************************
 * Name: stm32_seteptype
 ****************************************************************************/

static inline void stm32_seteptype(uint8_t epno, uint16_t type)
{
  uint32_t epaddr = STM32_USB_EPR(epno);
  uint16_t regval;

  regval  = stm32_getreg(epaddr);
  regval &= EPR_NOTOG_MASK;
  regval &= ~USB_EPR_EPTYPE_MASK;
  regval |= type;
  stm32_putreg(regval, epaddr);
}

/****************************************************************************
 * Name: stm32_setstatusout
 ****************************************************************************/

static inline void stm32_setstatusout(uint8_t epno)
{
  uint32_t epaddr = STM32_USB_EPR(epno);
  uint16_t regval;

  /* For a BULK endpoint the EP_KIND bit is used to enabled double buffering;
   * for a CONTROL endpoint, it is set to indicate that a status OUT
   * transaction is expected.  The bit is not used with out endpoint types.
   */

  regval  = stm32_getreg(epaddr);
  regval &= EPR_NOTOG_MASK;
  regval |= USB_EPR_EP_KIND;
  stm32_putreg(regval, epaddr);
}

/****************************************************************************
 * Name: stm32_clrstatusout
 ****************************************************************************/

static inline void stm32_clrstatusout(uint8_t epno)
{
  uint32_t epaddr = STM32_USB_EPR(epno);
  uint16_t regval;

  /* For a BULK endpoint the EP_KIND bit is used to enabled double buffering;
   * for a CONTROL endpoint, it is set to indicate that a status OUT
   * transaction is expected.  The bit is not used with out endpoint types.
   */

  regval  = stm32_getreg(epaddr);
  regval &= EPR_NOTOG_MASK;
  regval &= ~USB_EPR_EP_KIND;
  stm32_putreg(regval, epaddr);
}

/****************************************************************************
 * Name: stm32_clrrxdtog
 ****************************************************************************/

static void stm32_clrrxdtog(uint8_t epno) 
{
  uint32_t epaddr = STM32_USB_EPR(epno);
  uint16_t regval;

  regval = stm32_getreg(epaddr);
  if ((regval & USB_EPR_DTOG_RX) != 0)
    {
      regval &= EPR_NOTOG_MASK;
      regval |= USB_EPR_DTOG_RX;
      stm32_putreg(regval, epaddr);
    } 
} 

/****************************************************************************
 * Name: stm32_clrtxdtog
 ****************************************************************************/

static void stm32_clrtxdtog(uint8_t epno) 
{
  uint32_t epaddr = STM32_USB_EPR(epno);
  uint16_t regval;

  regval = stm32_getreg(epaddr);
  if ((regval & USB_EPR_DTOG_TX) != 0)
    {
      regval &= EPR_NOTOG_MASK;
      regval |= USB_EPR_DTOG_TX;
      stm32_putreg(regval, epaddr);
    }
} 

/****************************************************************************
 * Name: stm32_clrepctrrx
 ****************************************************************************/

static void stm32_clrepctrrx(uint8_t epno) 
{
  uint32_t epaddr = STM32_USB_EPR(epno);
  uint16_t regval;

  regval  = stm32_getreg(epaddr);
  regval &= EPR_NOTOG_MASK;
  regval &= ~USB_EPR_CTR_RX;
  stm32_putreg(regval, epaddr);
} 

/****************************************************************************
 * Name: stm32_clrepctrtx
 ****************************************************************************/

static void stm32_clrepctrtx(uint8_t epno) 
{
  uint32_t epaddr = STM32_USB_EPR(epno);
  uint16_t regval;

  regval  = stm32_getreg(epaddr);
  regval &= EPR_NOTOG_MASK;
  regval &= ~USB_EPR_CTR_TX;
  stm32_putreg(regval, epaddr);
} 

/****************************************************************************
 * Name: stm32_geteptxstatus
 ****************************************************************************/

static inline uint16_t stm32_geteptxstatus(uint8_t epno) 
{
  return (uint16_t)(stm32_getreg(STM32_USB_EPR(epno)) & USB_EPR_STATTX_MASK);
}

/****************************************************************************
 * Name: stm32_geteprxstatus
 ****************************************************************************/

static inline uint16_t stm32_geteprxstatus(uint8_t epno) 
{
  return (stm32_getreg(STM32_USB_EPR(epno)) & USB_EPR_STATRX_MASK);
}

/****************************************************************************
 * Name: stm32_seteptxstatus
 ****************************************************************************/

static void stm32_seteptxstatus(uint8_t epno, uint16_t state) 
{
  uint32_t epaddr = STM32_USB_EPR(epno);
  uint16_t regval;

  /* The bits in the STAT_TX field can be toggled by software to set their
   * value. When set to 0, the value remains unchanged; when set to one,
   * value toggles.
   */

  regval = stm32_getreg(epaddr);

  /* The exclusive OR will set STAT_TX bits to 1 if there value is different
   * from the bits requested in 'state'
   */

  regval ^= state;
  regval &= EPR_TXDTOG_MASK;
  stm32_putreg(regval, epaddr);
} 

/****************************************************************************
 * Name: stm32_seteprxstatus
 ****************************************************************************/

static void stm32_seteprxstatus(uint8_t epno, uint16_t state) 
{
  uint32_t epaddr = STM32_USB_EPR(epno);
  uint16_t regval;

  /* The bits in the STAT_RX field can be toggled by software to set their
   * value. When set to 0, the value remains unchanged; when set to one,
   * value toggles.
   */

  regval = stm32_getreg(epaddr);

  /* The exclusive OR will set STAT_RX bits to 1 if there value is different
   * from the bits requested in 'state'
   */

  regval ^= state;
  regval &= EPR_RXDTOG_MASK;
  stm32_putreg(regval, epaddr);
} 

/****************************************************************************
 * Name: stm32_eptxstalled
 ****************************************************************************/

static inline bool stm32_eptxstalled(uint8_t epno) 
{
  return (stm32_geteptxstatus(epno) == USB_EPR_STATTX_STALL);
}

/****************************************************************************
 * Name: stm32_eprxstalled
 ****************************************************************************/

static inline bool stm32_eprxstalled(uint8_t epno) 
{
  return (stm32_geteprxstatus(epno) == USB_EPR_STATRX_STALL);
}

/****************************************************************************
 * Request Helpers
 ****************************************************************************/
/****************************************************************************
 * Name: stm32_copytopma
 ****************************************************************************/

static void stm32_copytopma(const uint8_t *buffer, uint16_t pma, uint16_t nbytes) 
{
  uint16_t *dest;
  uint16_t  ms;
  uint16_t  ls;
  int     nwords = (nbytes + 1) >> 1;
  int     i;

  /* Copy loop.  Source=user buffer, Dest=packet memory */

  dest = (uint16_t*)(STM32_USBRAM_BASE + ((uint32_t)pma << 1));
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
 * Name: stm32_copyfrompma
 ****************************************************************************/

static inline void
stm32_copyfrompma(uint8_t *buffer, uint16_t pma, uint16_t nbytes) 
{
  uint32_t *src;
  int     nwords = (nbytes + 1) >> 1;
  int     i;

  /* Copy loop.  Source=packet memory, Dest=user buffer */

  src = (uint32_t*)(STM32_USBRAM_BASE + ((uint32_t)pma << 1));
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
 * Name: stm32_rqdequeue
 ****************************************************************************/

static struct stm32_req_s *stm32_rqdequeue(struct stm32_ep_s *privep)
{
  struct stm32_req_s *ret = privep->head;

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
 * Name: stm32_rqenqueue
 ****************************************************************************/

static void stm32_rqenqueue(struct stm32_ep_s *privep, struct stm32_req_s *req)
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
 * Name: stm32_abortrequest
 ****************************************************************************/

static inline void
stm32_abortrequest(struct stm32_ep_s *privep, struct stm32_req_s *privreq, int16_t result)
{
  usbtrace(TRACE_DEVERROR(STM32_TRACEERR_REQABORTED), (uint16_t)USB_EPNO(privep->ep.eplog));

  /* Save the result in the request structure */

  privreq->req.result = result;

  /* Callback to the request completion handler */

  privreq->req.callback(&privep->ep, &privreq->req);
}

/****************************************************************************
 * Name: stm32_reqcomplete
 ****************************************************************************/

static void stm32_reqcomplete(struct stm32_ep_s *privep, int16_t result)
{
  struct stm32_req_s *privreq;
  irqstate_t flags;

  /* Remove the completed request at the head of the endpoint request list */

  flags = irqsave();
  privreq = stm32_rqdequeue(privep);
  irqrestore(flags);

  if (privreq)
    {
      /* If endpoint 0, temporarily reflect the state of protocol stalled
       * in the callback.
       */

      bool stalled = privep->stalled;
      if (USB_EPNO(privep->ep.eplog) == EP0)
        {
          privep->stalled = (privep->dev->ep0state == EP0STATE_STALLED);
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

static void stm32_epwrite(struct stm32_usbdev_s *priv,
                          struct stm32_ep_s *privep,
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

      stm32_copytopma(buf, stm32_geteptxaddr(epno), nbytes);
    }

  /* Send the packet (might be a null packet nbytes == 0) */

  stm32_seteptxcount(epno, nbytes);
  priv->txstatus = USB_EPR_STATTX_VALID;

  /* Indicate that there is data in the TX packet memory.  This will be cleared
   * when the next data out interrupt is received.
   */

  privep->txbusy = true;
}

/****************************************************************************
 * Name: stm32_wrrequest
 ****************************************************************************/

static int stm32_wrrequest(struct stm32_usbdev_s *priv, struct stm32_ep_s *privep)
{
  struct stm32_req_s *privreq;
  uint8_t *buf;
  uint8_t epno;
  int nbytes;
  int bytesleft;

  /* We get here when an IN endpoint interrupt occurs.  So now we know that
   * there is no TX transfer in progress.
   */
  
  privep->txbusy = false;

  /* Check the request from the head of the endpoint request queue */

  privreq = stm32_rqpeek(privep);
  if (!privreq)
    {
      /* There is no TX transfer in progress and no new pending TX
       * requests to send.
       */

      usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPINQEMPTY), 0);
      return -ENOENT;
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
  stm32_epwrite(priv, privep, buf, nbytes);

  /* Update for the next data IN interrupt */

  privreq->req.xfrd += nbytes;
  bytesleft          = privreq->req.len - privreq->req.xfrd;

  /* If all of the bytes were sent (including any final null packet)
   * then we are finished with the request buffer).
   */

  if (bytesleft == 0 && !privep->txnullpkt)
    {
      /* Return the write request to the class driver */

      usbtrace(TRACE_COMPLETE(USB_EPNO(privep->ep.eplog)), privreq->req.xfrd);
      privep->txnullpkt = 0;
      stm32_reqcomplete(privep, OK);
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_rdrequest
 ****************************************************************************/

static int stm32_rdrequest(struct stm32_usbdev_s *priv, struct stm32_ep_s *privep)
{
  struct stm32_req_s *privreq;
  uint32_t src;
  uint8_t *dest;
  uint8_t epno;
  int pmalen;
  int readlen;

  /* Check the request from the head of the endpoint request queue */

  epno    = USB_EPNO(privep->ep.eplog);
  privreq = stm32_rqpeek(privep);
  if (!privreq)
    {
      /* Incoming data available in PMA, but no packet to receive the data.
       * Mark that the RX data is pending and hope that a packet is returned
       * soon.
       */

      usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPOUTQEMPTY), epno);
      return -ENOENT;
    }

  ullvdbg("EP%d: len=%d xfrd=%d\n", epno, privreq->req.len, privreq->req.xfrd);

  /* Ignore any attempt to receive a zero length packet */

  if (privreq->req.len == 0)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_EPOUTNULLPACKET), 0);
      stm32_reqcomplete(privep, OK);
      return OK;
    }

  usbtrace(TRACE_READ(USB_EPNO(privep->ep.eplog)), privreq->req.xfrd);

  /* Get the source and destination transfer addresses */

  dest    = privreq->req.buf + privreq->req.xfrd;
  src     = stm32_geteprxaddr(epno);

  /* Get the number of bytes to read from packet memory */

  pmalen  = stm32_geteprxcount(epno);
  readlen = MIN(privreq->req.len, pmalen);

  /* Receive the next packet */

  stm32_copyfrompma(dest, src, readlen);

  /* If the receive buffer is full or this is a partial packet,
   * then we are finished with the request buffer).
   */

  privreq->req.xfrd += readlen;
  if (pmalen < privep->ep.maxpacket || privreq->req.xfrd >= privreq->req.len)
    {
      /* Return the read request to the class driver. */

      usbtrace(TRACE_COMPLETE(epno), privreq->req.xfrd);
      stm32_reqcomplete(privep, OK);
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_cancelrequests
 ****************************************************************************/

static void stm32_cancelrequests(struct stm32_ep_s *privep)
{
  while (!stm32_rqempty(privep))
    {
      usbtrace(TRACE_COMPLETE(USB_EPNO(privep->ep.eplog)),
               (stm32_rqpeek(privep))->req.xfrd);
      stm32_reqcomplete(privep, -ESHUTDOWN);
    }
}

/****************************************************************************
 * Interrupt Level Processing
 ****************************************************************************/
/****************************************************************************
 * Name: stm32_dispatchrequest
 ****************************************************************************/

static void stm32_dispatchrequest(struct stm32_usbdev_s *priv)
{
  int ret;

  usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_DISPATCH), 0);
  if (priv && priv->driver)
    {
      /* Forward to the control request to the class driver implementation */

      ret = CLASS_SETUP(priv->driver, &priv->usbdev, &priv->ctrl, NULL, 0);
      if (ret < 0)
        {
          /* Stall on failure */

          usbtrace(TRACE_DEVERROR(STM32_TRACEERR_DISPATCHSTALL), 0);
          priv->ep0state = EP0STATE_STALLED;
        }
    }
}

/****************************************************************************
 * Name: stm32_epdone
 ****************************************************************************/

static void stm32_epdone(struct stm32_usbdev_s *priv, uint8_t epno)
{
  struct stm32_ep_s *privep;
  uint16_t epr;

  /* Decode and service non control endpoints interrupt */ 

  epr    = stm32_getreg(STM32_USB_EPR(epno));
  privep = &priv->eplist[epno];

  /* OUT: host-to-device
   * CTR_RX is set by the hardware when an OUT/SETUP transaction
   * successfully completed on this endpoint.
   */

  if ((epr & USB_EPR_CTR_RX) != 0)
    {
      usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPOUTDONE), epr);

      /* Handle read requests.  First check if a read request is available to
       * accept the host data.
       */

      if (!stm32_rqempty(privep))
        {
          /* Read host data into the current read request */

          (void)stm32_rdrequest(priv, privep);

          /* "After the received data is processed, the application software
           *  should set the STAT_RX bits to '11' (Valid) in the USB_EPnR,
           *  enabling further transactions. "
           */

          priv->rxstatus  = USB_EPR_STATRX_VALID;
        }

      /* NAK further OUT packets if there there no more read requests */

      if (stm32_rqempty(privep))
        {
          usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPOUTPENDING), (uint16_t)epno);

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

      stm32_clrepctrrx(epno);
      stm32_seteprxstatus(epno, priv->rxstatus);
    }

  /* IN: device-to-host
   * CTR_TX is set when an IN transaction successfully completes on
   * an endpoint
   */

  else if ((epr & USB_EPR_CTR_TX) != 0)
    {
      /* Clear interrupt status */

      stm32_clrepctrtx(epno);
      usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPINDONE), epr);
          
      /* Handle write requests */ 

      priv->txstatus = USB_EPR_STATTX_NAK;
      stm32_wrrequest(priv, privep);

      /* Set the new TX status */

      stm32_seteptxstatus(epno, priv->txstatus);
    }  
}

/****************************************************************************
 * Name: stm32_setdevaddr
 ****************************************************************************/

static void stm32_setdevaddr(struct stm32_usbdev_s *priv, uint8_t value) 
{
  int epno;
  
  /* Set address in every allocated endpoint */

  for (epno = 0; epno < STM32_NENDPOINTS; epno++)
    {
      if (stm32_epreserved(priv, epno))
        {
          stm32_setepaddress((uint8_t)epno, (uint8_t)epno);
        }
    }

  /* Set the device address and enable function */

  stm32_putreg(value|USB_DADDR_EF, STM32_USB_DADDR);
}

/****************************************************************************
 * Name: stm32_ep0setup
 ****************************************************************************/

static void stm32_ep0setup(struct stm32_usbdev_s *priv)
{
  struct stm32_ep_s   *ep0     = &priv->eplist[EP0];
  struct stm32_req_s  *privreq = stm32_rqpeek(ep0);
  struct stm32_ep_s   *privep;
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

  while (!stm32_rqempty(ep0))
    {
      int16_t result = OK;
      if (privreq->req.xfrd != privreq->req.len)
        {
          result = -EPROTO;
        }

      usbtrace(TRACE_COMPLETE(ep0->ep.eplog), privreq->req.xfrd);
      stm32_reqcomplete(ep0, result);
    }

  /* Assume NOT stalled; no TX in progress */

  ep0->stalled  = 0;
  ep0->txbusy   = 0;

  /* Get a 32-bit PMA address and use that to get the 8-byte setup request */

  stm32_copyfrompma((uint8_t*)&priv->ctrl, stm32_geteprxaddr(EP0), USB_SIZEOF_CTRLREQ);

  /* And extract the little-endian 16-bit values to host order */

  value.w = GETUINT16(priv->ctrl.value);
  index.w = GETUINT16(priv->ctrl.index);
  len.w   = GETUINT16(priv->ctrl.len);

  ullvdbg("SETUP: type=%02x req=%02x value=%04x index=%04x len=%04x\n",
          priv->ctrl.type, priv->ctrl.req, value.w, index.w, len.w);

  priv->ep0state = EP0STATE_IDLE;

  /* Dispatch any non-standard requests */

  if ((priv->ctrl.type & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_STANDARD)
    {
      usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_NOSTDREQ), priv->ctrl.type);

      /* Let the class implementation handle all non-standar requests */

      stm32_dispatchrequest(priv);
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

        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_GETSTATUS), priv->ctrl.type);
        if (len.w != 2 || (priv->ctrl.type & USB_REQ_DIR_IN) == 0 ||
            index.b[MSB] != 0 || value.w != 0)
          {
            usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADEPGETSTATUS), 0);
            priv->ep0state = EP0STATE_STALLED;
          }
        else
          {
            switch (priv->ctrl.type & USB_REQ_RECIPIENT_MASK)
              {
               case USB_REQ_RECIPIENT_ENDPOINT:
                {
                  epno = USB_EPNO(index.b[LSB]);
                  usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPGETSTATUS), epno);
                  if (epno >= STM32_NENDPOINTS)
                    {
                      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADEPGETSTATUS), epno);
                      priv->ep0state = EP0STATE_STALLED;
                    }
                  else
                    {
                      privep     = &priv->eplist[epno];
                      response.w = 0; /* Not stalled */
                      nbytes     = 2; /* Response size: 2 bytes */

                      if (USB_ISEPIN(index.b[LSB]))
                        {
                          /* IN endpoint */ 

                          if (stm32_eptxstalled(epno))
                            {
                              /* IN Endpoint stalled */

                              response.b[LSB] = 1; /* Stalled */
                            }
                          }
                      else
                        {
                          /* OUT endpoint */ 

                          if (stm32_eprxstalled(epno))
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
                      usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_DEVGETSTATUS), 0);

                      /* Features:  Remote Wakeup=YES; selfpowered=? */

                      response.w      = 0;
                      response.b[LSB] = (priv->selfpowered << USB_FEATURE_SELFPOWERED) |
                                        (1 << USB_FEATURE_REMOTEWAKEUP);
                      nbytes          = 2; /* Response size: 2 bytes */
                    }
                  else
                    {
                      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADDEVGETSTATUS), 0);
                      priv->ep0state = EP0STATE_STALLED;
                    }
                }
                break;

              case USB_REQ_RECIPIENT_INTERFACE:
                {
                  usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_IFGETSTATUS), 0);
                  response.w = 0;
                  nbytes     = 2; /* Response size: 2 bytes */
                }
                break;

              default:
                {
                  usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADGETSTATUS), 0);
                  priv->ep0state = EP0STATE_STALLED;
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

        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_CLEARFEATURE), priv->ctrl.type);
        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) != USB_REQ_RECIPIENT_ENDPOINT)
          {
            /* Let the class implementation handle all recipients (except for the
             * endpoint recipient)
             */

            stm32_dispatchrequest(priv);
            handled = true;
          }
        else
          {
            /* Endpoint recipient */

            epno = USB_EPNO(index.b[LSB]);
            if (epno < STM32_NENDPOINTS && index.b[MSB] == 0 &&
                value.w == USB_FEATURE_ENDPOINTHALT && len.w == 0)
              {
                privep         = &priv->eplist[epno];
                privep->halted = 0;
                ret            = stm32_epstall(&privep->ep, true);
              }
            else
              {
                usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADCLEARFEATURE), 0);
                priv->ep0state = EP0STATE_STALLED;
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

        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_SETFEATURE), priv->ctrl.type);
        if (((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE) &&
            value.w == USB_FEATURE_TESTMODE)
          {
            /* Special case recipient=device test mode */

            ullvdbg("test mode: %d\n", index.w);
          }
        else if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) != USB_REQ_RECIPIENT_ENDPOINT)
          {
            /* The class driver handles all recipients except recipient=endpoint */

            stm32_dispatchrequest(priv);
            handled = true;
          }
        else
          {
            /* Handler recipient=endpoint */

            epno = USB_EPNO(index.b[LSB]);
            if (epno < STM32_NENDPOINTS && index.b[MSB] == 0 &&
                value.w == USB_FEATURE_ENDPOINTHALT && len.w == 0)
              {
                privep         = &priv->eplist[epno];
                privep->halted = 1;
                ret            = stm32_epstall(&privep->ep, false);
              }
            else
              {
                usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADSETFEATURE), 0);
                priv->ep0state = EP0STATE_STALLED;
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

        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EP0SETUPSETADDRESS), value.w);
        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) != USB_REQ_RECIPIENT_DEVICE ||
            index.w != 0 || len.w != 0 || value.w > 127)
          {
            usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADSETADDRESS), 0);
            priv->ep0state = EP0STATE_STALLED;
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
        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_GETSETDESC), priv->ctrl.type);
        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE)
          {
            /* The request seems valid... let the class implementation handle it */

            stm32_dispatchrequest(priv);
            handled = true;
          }
        else
          {
            usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADGETSETDESC), 0);
            priv->ep0state = EP0STATE_STALLED;
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
        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_GETCONFIG), priv->ctrl.type);
        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE &&
            value.w == 0 && index.w == 0 && len.w == 1)
          {
            /* The request seems valid... let the class implementation handle it */

            stm32_dispatchrequest(priv);
            handled = true;
          }
        else
          {
            usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADGETCONFIG), 0);
            priv->ep0state = EP0STATE_STALLED;
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
        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_SETCONFIG), priv->ctrl.type);
        if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE &&
            index.w == 0 && len.w == 0)
          {
             /* The request seems valid... let the class implementation handle it */

             stm32_dispatchrequest(priv);
             handled = true;
          }
        else
          {
            usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADSETCONFIG), 0);
            priv->ep0state = EP0STATE_STALLED;
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

        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_GETSETIF), priv->ctrl.type);
        stm32_dispatchrequest(priv);
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
        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_SYNCHFRAME), 0);
      }
      break;

    default:
      {
        usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDCTRLREQ), priv->ctrl.req);
        priv->ep0state = EP0STATE_STALLED;
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
   *    'handled' will be set to true and ep0state != EP0STATE_STALLED;
   * 3. An error was detected in either the above logic or by the class implementation
   *    logic.  In either case, priv->state will be set EP0STATE_STALLED
   *    to indicate this case.
   *
   * NOTE: Non-standard requests are a special case.  They are handled by the
   * class implementation and this function returned early above, skipping this
   * logic altogether.
   */

  if (priv->ep0state != EP0STATE_STALLED && !handled)
    {
      /* We will response.  First, restrict the data length to the length
       * requested in the setup packet
       */

      if (nbytes > len.w)
        {
          nbytes = len.w;
        }

      /* Send the response (might be a zero-length packet) */

      stm32_epwrite(priv, ep0, response.b, nbytes);
      priv->ep0state = EP0STATE_IDLE;
    }
}

/****************************************************************************
 * Name: stm32_ep0in
 ****************************************************************************/

static void stm32_ep0in(struct stm32_usbdev_s *priv)
{
  int ret;

  /* There is no longer anything in the EP0 TX packet memory */

  priv->eplist[EP0].txbusy = false;

  /* Are we processing the completion of one packet of an outgoing request
   * from the class driver?
   */

  if (priv->ep0state == EP0STATE_WRREQUEST)
    {
      ret = stm32_wrrequest(priv, &priv->eplist[EP0]);
      priv->ep0state = ((ret == OK) ? EP0STATE_WRREQUEST : EP0STATE_IDLE);
    }

  /* No.. Are we processing the completion of a status response? */

  else if (priv->ep0state == EP0STATE_IDLE)
    {
      /* Look at the saved SETUP command.  Was it a SET ADDRESS request?
       * If so, then now is the time to set the address.
       */

      if (priv->ctrl.req == USB_REQ_SETADDRESS && 
          (priv->ctrl.type & REQRECIPIENT_MASK) == (USB_REQ_TYPE_STANDARD | USB_REQ_RECIPIENT_DEVICE))
        {
          union wb_u value;
          value.w = GETUINT16(priv->ctrl.value);
          stm32_setdevaddr(priv, value.b[LSB]);
        }
    }
  else
    {
      priv->ep0state = EP0STATE_STALLED;
    }
}

/****************************************************************************
 * Name: stm32_ep0out
 ****************************************************************************/

static void stm32_ep0out(struct stm32_usbdev_s *priv)
{
  int ret;

  struct stm32_ep_s *privep = &priv->eplist[EP0];
  switch (priv->ep0state)
    {
      case EP0STATE_RDREQUEST:  /* Write request in progress */
      case EP0STATE_IDLE:       /* No transfer in progress */
        ret = stm32_rdrequest(priv, privep);
        priv->ep0state = ((ret == OK) ? EP0STATE_RDREQUEST : EP0STATE_IDLE);
        break;

      default:
        /* Unexpected state OR host aborted the OUT transfer before it
         * completed, STALL the endpoint in either case
         */

        priv->ep0state = EP0STATE_STALLED;
        break;
    }
}

/****************************************************************************
 * Name: stm32_ep0done
 ****************************************************************************/

static inline void stm32_ep0done(struct stm32_usbdev_s *priv, uint16_t istr)
{
  uint16_t epr;

  /* Initialize RX and TX status.  We shouldn't have to actually look at the
   * status because the hardware is supposed to set the both RX and TX status
   * to NAK when an EP0 SETUP occurs (of course, this might not be a setup)
   */ 

  priv->rxstatus = USB_EPR_STATRX_NAK;
  priv->txstatus = USB_EPR_STATTX_NAK;

  /* Set both RX and TX status to NAK  */ 

  stm32_seteprxstatus(EP0, USB_EPR_STATRX_NAK);
  stm32_seteptxstatus(EP0, USB_EPR_STATTX_NAK);
          
  /* Check the direction bit to determine if this the completion of an EP0
   * packet sent to or received from the host PC.
   */

  if ((istr & USB_ISTR_DIR) == 0)
    {
      /* EP0 IN: device-to-host (DIR=0) */

      usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EP0IN), istr);
      stm32_clrepctrtx(EP0);
      stm32_ep0in(priv);
    }
  else
    {
      /* EP0 OUT: host-to-device (DIR=1) */

      epr = stm32_getreg(STM32_USB_EPR(EP0));

      /* CTR_TX is set when an IN transaction successfully
       * completes on an endpoint
       */

      if ((epr & USB_EPR_CTR_TX) != 0)
        {
          usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EP0INDONE), epr);
          stm32_clrepctrtx(EP0);
          stm32_ep0in(priv);
        }

      /* SETUP is set by the hardware when the last completed
       * transaction was a control endpoint SETUP
       */
 
      else if ((epr & USB_EPR_SETUP) != 0)
        {
          usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EP0SETUPDONE), epr);
          stm32_clrepctrrx(EP0);
          stm32_ep0setup(priv);
        }

      /* Set by the hardware when an OUT/SETUP transaction successfully
       * completed on this endpoint.
       */

      else if ((epr & USB_EPR_CTR_RX) != 0)
        {
          usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EP0OUTDONE), epr);
          stm32_clrepctrrx(EP0);
          stm32_ep0out(priv);
        }

      /* None of the above */

      else
        {
          usbtrace(TRACE_DEVERROR(STM32_TRACEERR_EP0BADCTR), epr);
          return; /* Does this ever happen? */
        }
    }

  /* Make sure that the EP0 packet size is still OK (superstitious?) */

  stm32_seteprxcount(EP0, STM32_EP0MAXPACKET);

  /* Now figure out the new RX/TX status.  Here are all possible
   * consequences of the above EP0 operations:
   *
   * rxstatus txstatus ep0state  MEANING
   * -------- -------- --------- ---------------------------------
   * NAK      NAK      IDLE      Nothing happened
   * NAK      VALID    IDLE      EP0 response sent from USBDEV driver
   * NAK      VALID    WRREQUEST EP0 response sent from class driver
   * NAK      ---      STALL     Some protocol error occurred
   *
   * First handle the STALL condition:
   */

  if (priv->ep0state == EP0STATE_STALLED)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_EP0SETUPSTALLED), priv->ep0state);
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

  stm32_seteprxstatus(EP0, priv->rxstatus);
  stm32_seteptxstatus(EP0, priv->txstatus);
}

/****************************************************************************
 * Name: stm32_lptransfer
 ****************************************************************************/

static void stm32_lptransfer(struct stm32_usbdev_s *priv) 
{
  uint8_t  epno;
  uint16_t istr;

  /* Stay in loop while LP interrupts are pending */

  while (((istr = stm32_getreg(STM32_USB_ISTR)) & USB_ISTR_CTR) != 0)
    {
      stm32_putreg((uint16_t)~USB_ISTR_CTR, STM32_USB_ISTR);

      /* Extract highest priority endpoint number */ 

      epno = (uint8_t)(istr & USB_ISTR_EPID_MASK);

      /* Handle EP0 completion events */

      if (epno == 0)
        {
          stm32_ep0done(priv, istr);
        }

      /* Handle other endpoint completion events */

      else
        {
          stm32_epdone(priv, epno);
        }
    }
}

/****************************************************************************
 * Name: stm32_hpinterrupt
 ****************************************************************************/

static int stm32_hpinterrupt(int irq, void *context)
{
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple USB controllers
   * easier.
   */

  struct stm32_usbdev_s *priv = &g_usbdev;
  uint16_t istr;
  uint8_t  epno;

  /* High priority interrupts are only triggered by a correct transfer event
   * for isochronous and double-buffer bulk transfers.
   */

  istr = stm32_getreg(STM32_USB_ISTR);
  usbtrace(TRACE_INTENTRY(STM32_TRACEINTID_HPINTERRUPT), istr);
  while ((istr & USB_ISTR_CTR) != 0)
    {
      stm32_putreg((uint16_t)~USB_ISTR_CTR, STM32_USB_ISTR);
      
      /* Extract highest priority endpoint number */ 

      epno = (uint8_t)(istr & USB_ISTR_EPID_MASK);

      /* And handle the completion event */

      stm32_epdone(priv, epno);

      /* Fetch the status again for the next time through the loop */

      istr = stm32_getreg(STM32_USB_ISTR);
    }

  usbtrace(TRACE_INTEXIT(STM32_TRACEINTID_HPINTERRUPT), 0);
  return OK;
}

/****************************************************************************
 * Name: stm32_lpinterrupt
 ****************************************************************************/

static int stm32_lpinterrupt(int irq, void *context)
{
  /* For now there is only one USB controller, but we will always refer to
   * it using a pointer to make any future ports to multiple USB controllers
   * easier.
   */

  struct stm32_usbdev_s *priv = &g_usbdev;
  uint16_t istr = stm32_getreg(STM32_USB_ISTR);

  usbtrace(TRACE_INTENTRY(STM32_TRACEINTID_LPINTERRUPT), istr);

  /* Handle Reset interrupts.  When this event occurs, the peripheral is left
   * in the same conditions it is left by the system reset (but with the
   * USB controller enabled).
   */

  if ((istr & USB_ISTR_RESET) != 0)
    {
      /* Reset interrupt received. Clear the RESET interrupt status. */

      stm32_putreg(~USB_ISTR_RESET, STM32_USB_ISTR);
      usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_RESET), istr);

      /* Restore our power-up state and exit now because istr is no longer
       * valid.
       */

      stm32_reset(priv);
      goto exit_lpinterrupt;
    }

  /* Handle Wakeup interrupts.  This interrupt is only enable while the USB is
   * suspended.
   */

  if ((istr & USB_ISTR_WKUP & priv->imask) != 0)
    {
      /* Wakeup interrupt received. Clear the WKUP interrupt status.  The
       * cause of the resume is indicated in the FNR register
       */

      stm32_putreg(~USB_ISTR_WKUP, STM32_USB_ISTR);
      usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_WKUP), stm32_getreg(STM32_USB_FNR));

      /* Perform the wakeup action */

      stm32_initresume(priv);
      priv->rsmstate = RSMSTATE_IDLE;

      /* Disable ESOF polling, disable the wakeup interrupt, and
       * re-enable the suspend interrupt.  Clear any pending SUSP
       * interrupts.
       */

      stm32_setimask(priv, USB_CNTR_SUSPM, USB_CNTR_ESOFM|USB_CNTR_WKUPM);
      stm32_putreg(~USB_CNTR_SUSPM, STM32_USB_ISTR);
    }

  if ((istr & USB_ISTR_SUSP & priv->imask) != 0)
    {
        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_SUSP), 0);
        stm32_suspend(priv);

        /* Clear of the ISTR bit must be done after setting of USB_CNTR_FSUSP */ 

        stm32_putreg(~USB_ISTR_SUSP, STM32_USB_ISTR);
    }

  if ((istr & USB_ISTR_ESOF & priv->imask) != 0)
    {
      stm32_putreg(~USB_ISTR_ESOF, STM32_USB_ISTR);
      
      /* Resume handling timing is made with ESOFs */ 

      usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_ESOF), 0);
      stm32_esofpoll(priv);
    }

  if ((istr & USB_ISTR_CTR & priv->imask) != 0)
    {
      /* Low priority endpoint correct transfer interrupt */ 

      usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_LPCTR), istr);
      stm32_lptransfer(priv);
    }

exit_lpinterrupt:
  usbtrace(TRACE_INTEXIT(STM32_TRACEINTID_LPINTERRUPT), stm32_getreg(STM32_USB_EP0R));
  return OK;
}

/****************************************************************************
 * Name: stm32_setimask
 ****************************************************************************/

static void
stm32_setimask(struct stm32_usbdev_s *priv, uint16_t setbits, uint16_t clrbits)
{
  uint16_t regval;

  /* Adjust the interrupt mask bits in the shadow copy first */

  priv->imask &= ~clrbits;
  priv->imask |= setbits;

  /* Then make the interrupt mask bits in the CNTR register match the shadow
   * register (Hmmm... who is shadowing whom?)
   */

  regval  = stm32_getreg(STM32_USB_CNTR);
  regval &= ~USB_CNTR_ALLINTS;
  regval |= priv->imask;
  stm32_putreg(regval, STM32_USB_CNTR);
}

/****************************************************************************
 * Suspend/Resume Helpers
 ****************************************************************************/
/****************************************************************************
 * Name: stm32_suspend
 ****************************************************************************/

static void stm32_suspend(struct stm32_usbdev_s *priv) 
{
  uint16_t regval;
  
  /* Notify the class driver of the suspend event */

  if (priv->driver)
    {
      CLASS_SUSPEND(priv->driver, &priv->usbdev);
    }

  /* Disable ESOF polling, disable the SUSP interrupt, and enable the WKUP
   * interrupt.  Clear any pending WKUP interrupt.
   */

  stm32_setimask(priv, USB_CNTR_WKUPM, USB_CNTR_ESOFM|USB_CNTR_SUSPM);
  stm32_putreg(~USB_ISTR_WKUP, STM32_USB_ISTR);

  /* Set the FSUSP bit in the CNTR register.  This activates suspend mode
   * within the USB peripheral and disables further SUSP interrupts.
   */

  regval  = stm32_getreg(STM32_USB_CNTR);
  regval |= USB_CNTR_FSUSP;
  stm32_putreg(regval, STM32_USB_CNTR);

  /* If we are not a self-powered device, the got to low-power mode */

  if (!priv->selfpowered)
    {
      /* Setting LPMODE in the CNTR register removes static power
       * consumption in the USB analog transceivers but keeps them
       * able to detect resume activity
       */

      regval = stm32_getreg(STM32_USB_CNTR);
      regval |= USB_CNTR_LPMODE;
      stm32_putreg(regval, STM32_USB_CNTR);
    }

  /* Let the board-specific logic know that we have entered the suspend
   * state
   */ 

  stm32_usbsuspend((struct usbdev_s *)priv, false);
} 

/****************************************************************************
 * Name: stm32_initresume
 ****************************************************************************/

static void stm32_initresume(struct stm32_usbdev_s *priv) 
{
  uint16_t regval;

  /* This function is called when either (1) a WKUP interrupt is received from
   * the host PC, or (2) the class device implementation calls the wakeup()
   * method.
   */

  /* Clear the USB low power mode (lower power mode was not set if this is
   * a self-powered device.  Also, low power mode is automatically cleared by
   * hardware when a WKUP interrupt event occurs). 
   */

  regval = stm32_getreg(STM32_USB_CNTR);
  regval &= (~USB_CNTR_LPMODE);
  stm32_putreg(regval, STM32_USB_CNTR);
  
  /* Restore full power -- whatever that means for this particular board */ 

  stm32_usbsuspend((struct usbdev_s *)priv, true);
  
  /* Reset FSUSP bit and enable normal interrupt handling */

  stm32_putreg(STM32_CNTR_SETUP, STM32_USB_CNTR);

  /* Notify the class driver of the resume event */

  if (priv->driver)
    {
      CLASS_RESUME(priv->driver, &priv->usbdev);
    }
} 

/****************************************************************************
 * Name: stm32_esofpoll
 ****************************************************************************/

static void stm32_esofpoll(struct stm32_usbdev_s *priv) 
{
  uint16_t regval;

    /* Called periodically from ESOF interrupt after RSMSTATE_STARTED */

  switch (priv->rsmstate)
    {
    /* One ESOF after internal resume requested */

    case RSMSTATE_STARTED:
      regval         = stm32_getreg(STM32_USB_CNTR);
      regval        |= USB_CNTR_RESUME;
      stm32_putreg(regval, STM32_USB_CNTR);
      priv->rsmstate = RSMSTATE_WAITING;
      priv->nesofs   = 10;
      break;

    /* Countdown before completing the operation */

    case RSMSTATE_WAITING:
      priv->nesofs--;
      if (priv->nesofs == 0)
        {
          /* Okay.. we are ready to resume normal operation */

          regval         = stm32_getreg(STM32_USB_CNTR);
          regval        &= (~USB_CNTR_RESUME);
          stm32_putreg(regval, STM32_USB_CNTR);
          priv->rsmstate = RSMSTATE_IDLE;

          /* Disable ESOF polling, disable the SUSP interrupt, and enable
           * the WKUP interrupt.  Clear any pending WKUP interrupt.
           */

          stm32_setimask(priv, USB_CNTR_WKUPM, USB_CNTR_ESOFM|USB_CNTR_SUSPM);
          stm32_putreg(~USB_ISTR_WKUP, STM32_USB_ISTR);
        }
      break;

    case RSMSTATE_IDLE:
    default:
      priv->rsmstate = RSMSTATE_IDLE;
      break;
    }
}

/****************************************************************************
 * Endpoint Helpers
 ****************************************************************************/
/****************************************************************************
 * Name: stm32_epreserve
 ****************************************************************************/

static inline struct stm32_ep_s *
stm32_epreserve(struct stm32_usbdev_s *priv, uint8_t epset)
{
  struct stm32_ep_s *privep = NULL;
  irqstate_t flags;
  int epndx = 0;

  flags = irqsave();
  epset &= priv->epavail;
  if (epset)
    {
      /* Select the lowest bit in the set of matching, available endpoints
       * (skipping EP0)
       */

      for (epndx = 1; epndx < STM32_NENDPOINTS; epndx++)
        {
          uint8_t bit = STM32_ENDP_BIT(epndx);
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
 * Name: stm32_epunreserve
 ****************************************************************************/

static inline void
stm32_epunreserve(struct stm32_usbdev_s *priv, struct stm32_ep_s *privep)
{
  irqstate_t flags = irqsave();
  priv->epavail   |= STM32_ENDP_BIT(USB_EPNO(privep->ep.eplog));
  irqrestore(flags);
}

/****************************************************************************
 * Name: stm32_epreserved
 ****************************************************************************/

static inline bool
stm32_epreserved(struct stm32_usbdev_s *priv, int epno)
{
  return ((priv->epavail & STM32_ENDP_BIT(epno)) == 0);
}

/****************************************************************************
 * Name: stm32_epallocpma
 ****************************************************************************/

static int stm32_epallocpma(struct stm32_usbdev_s *priv)
{
  irqstate_t flags;
  int bufno = ERROR;
  int bufndx;

  flags = irqsave();
  for (bufndx = 2; bufndx < STM32_NBUFFERS; bufndx++)
    {
      /* Check if this buffer is available */

      uint8_t bit = STM32_BUFFER_BIT(bufndx);
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
 * Name: stm32_epfreepma
 ****************************************************************************/

static inline void
stm32_epfreepma(struct stm32_usbdev_s *priv, struct stm32_ep_s *privep)
{
  irqstate_t flags = irqsave();
  priv->epavail   |= STM32_ENDP_BIT(privep->bufno);
  irqrestore(flags);
}

/****************************************************************************
 * Endpoint operations
 ****************************************************************************/
/****************************************************************************
 * Name: stm32_epconfigure
 ****************************************************************************/

static int stm32_epconfigure(struct usbdev_ep_s *ep,
                             const struct usb_epdesc_s *desc,
                             bool last)
{
  struct stm32_ep_s *privep = (struct stm32_ep_s *)ep;
  uint16_t pma;
  uint16_t setting;
  uint16_t maxpacket;
  uint8_t  epno;

#ifdef CONFIG_DEBUG
  if (!ep || !desc)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), 0);
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
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADEPTYPE), (uint16_t)desc->type);
      return -EINVAL;
    }

  stm32_seteptype(epno, setting);

  /* Get the address of the PMA buffer allocated for this endpoint */

#warning "REVISIT: Should configure BULK EPs using double buffer feature"
  pma = STM32_BUFNO2BUF(privep->bufno);

  /* Get the maxpacket size of the endpoint. */

  maxpacket = GETUINT16(desc->mxpacketsize);
  DEBUGASSERT(maxpacket <= STM32_MAXPACKET_SIZE);
  ep->maxpacket = maxpacket;

  /* Get the subset matching the requested direction */

  if (USB_ISEPIN(desc->addr))
    {
      /* The full, logical EP number includes direction */
 
      ep->eplog = USB_EPIN(epno);

      /* Set up TX; disable RX */

      stm32_seteptxaddr(epno, pma);
      stm32_seteptxstatus(epno, USB_EPR_STATTX_NAK);
      stm32_seteprxstatus(epno, USB_EPR_STATRX_DIS);
    }
  else
    {
      /* The full, logical EP number includes direction */

      ep->eplog = USB_EPOUT(epno);

      /* Set up RX; disable TX */

      stm32_seteprxaddr(epno, pma);
      stm32_seteprxcount(epno, maxpacket);
      stm32_seteprxstatus(epno, USB_EPR_STATRX_VALID);
      stm32_seteptxstatus(epno, USB_EPR_STATTX_DIS);
    }

   stm32_dumpep(epno);
   return OK;
}

/****************************************************************************
 * Name: stm32_epdisable
 ****************************************************************************/

static int stm32_epdisable(struct usbdev_ep_s *ep)
{
  struct stm32_ep_s *privep = (struct stm32_ep_s *)ep;
  irqstate_t flags;
  uint8_t epno;

#ifdef CONFIG_DEBUG
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), 0);
      ulldbg("ERROR: ep=%p\n", ep);
      return -EINVAL;
    }
#endif

  epno = USB_EPNO(ep->eplog);
  usbtrace(TRACE_EPDISABLE, epno);

  /* Cancel any ongoing activity */

  flags = irqsave();
  stm32_cancelrequests(privep);

  /* Disable TX; disable RX */

  stm32_seteprxcount(epno, 0);
  stm32_seteprxstatus(epno, USB_EPR_STATRX_DIS);
  stm32_seteptxstatus(epno, USB_EPR_STATTX_DIS);

  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Name: stm32_epallocreq
 ****************************************************************************/

static struct usbdev_req_s *stm32_epallocreq(struct usbdev_ep_s *ep)
{
  struct stm32_req_s *privreq;

#ifdef CONFIG_DEBUG
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), 0);
      return NULL;
    }
#endif
  usbtrace(TRACE_EPALLOCREQ, USB_EPNO(ep->eplog));

  privreq = (struct stm32_req_s *)malloc(sizeof(struct stm32_req_s));
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_ALLOCFAIL), 0);
      return NULL;
    }

  memset(privreq, 0, sizeof(struct stm32_req_s));
  return &privreq->req;
}

/****************************************************************************
 * Name: stm32_epfreereq
 ****************************************************************************/

static void stm32_epfreereq(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  struct stm32_req_s *privreq = (struct stm32_req_s*)req;

#ifdef CONFIG_DEBUG
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), 0);
      return;
    }
#endif
  usbtrace(TRACE_EPFREEREQ, USB_EPNO(ep->eplog));

  free(privreq);
}

/****************************************************************************
 * Name: stm32_epsubmit
 ****************************************************************************/

static int stm32_epsubmit(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  struct stm32_req_s *privreq = (struct stm32_req_s *)req;
  struct stm32_ep_s *privep = (struct stm32_ep_s *)ep;
  struct stm32_usbdev_s *priv;
  irqstate_t flags;
  uint8_t epno;
  int ret = OK;

#ifdef CONFIG_DEBUG
  if (!req || !req->callback || !req->buf || !ep)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), 0);
      ulldbg("ERROR: req=%p callback=%p buf=%p ep=%p\n", req, req->callback, req->buf, ep);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPSUBMIT, USB_EPNO(ep->eplog));
  priv = privep->dev;

#ifdef CONFIG_DEBUG
  if (!priv->driver)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_NOTCONFIGURED), priv->usbdev.speed);
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
      stm32_abortrequest(privep, privreq, -EBUSY);
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

      stm32_rqenqueue(privep, privreq);
      usbtrace(TRACE_INREQQUEUED(epno), req->len);

      /* If the IN endpoint FIFO is available, then transfer the data now */

      if (!privep->txbusy)
        {
          priv->txstatus = USB_EPR_STATTX_NAK;
          ret = stm32_wrrequest(priv, privep);

          /* Set the new TX status */

          stm32_seteptxstatus(epno, priv->txstatus);
        }
    }

  /* Handle OUT (host-to-device) requests */

  else
    {
      /* Add the new request to the request queue for the OUT endpoint */

      privep->txnullpkt = 0;
      stm32_rqenqueue(privep, privreq);
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
          stm32_seteprxstatus(epno, priv->rxstatus);

          /* Data is no longer pending */

          priv->rxpending = false;
        }
    }

  irqrestore(flags);
  return ret;
}

/****************************************************************************
 * Name: stm32_epcancel
 ****************************************************************************/

static int stm32_epcancel(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
{
  struct stm32_ep_s *privep = (struct stm32_ep_s *)ep;
  struct stm32_usbdev_s *priv;
  irqstate_t flags;

#ifdef CONFIG_DEBUG
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif
  usbtrace(TRACE_EPCANCEL, USB_EPNO(ep->eplog));
  priv = privep->dev;

  flags = irqsave();
  stm32_cancelrequests(privep);
  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Name: stm32_epstall
 ****************************************************************************/

static int stm32_epstall(struct usbdev_ep_s *ep, bool resume)
{
  struct stm32_ep_s *privep;
  struct stm32_usbdev_s *priv;
  uint8_t epno = USB_EPNO(ep->eplog);
  uint16_t status;
  irqstate_t flags;

#ifdef CONFIG_DEBUG
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif
  privep = (struct stm32_ep_s *)ep;
  priv   = (struct stm32_usbdev_s *)privep->dev;
  epno   = USB_EPNO(ep->eplog);

  /* STALL or RESUME the endpoint */

  flags = irqsave();
  usbtrace(resume ? TRACE_EPRESUME : TRACE_EPSTALL, USB_EPNO(ep->eplog));

  /* Get status of the endpoint; stall the request if the endpoint is
   * disabled
   */
 
  if (USB_ISEPIN(ep->eplog))
    {
      status = stm32_geteptxstatus(epno);
    }
  else
    {
      status = stm32_geteprxstatus(epno);
    }

  if (status == 0)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_EPDISABLED), 0);

      if (epno == 0)
        {
          priv->ep0state = EP0STATE_STALLED;
        }

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

          if (stm32_eptxstalled(epno))
            {
              stm32_clrtxdtog(epno);

              /* Restart any queued write requests */

              priv->txstatus = USB_EPR_STATTX_NAK;
              (void)stm32_wrrequest(priv, privep);

              /* Set the new TX status */

              stm32_seteptxstatus(epno, priv->txstatus);
            }
        }
      else
        {
          /* OUT endpoint */ 

          if (stm32_eprxstalled(epno))
            {
              if (epno == EP0)
                {
                  /* After clear the STALL, enable the default endpoint receiver */

                  stm32_seteprxcount(epno, ep->maxpacket);
                }
              else
                {
                  stm32_clrrxdtog(epno);
                }

              priv->rxstatus = USB_EPR_STATRX_VALID;
              stm32_seteprxstatus(epno, USB_EPR_STATRX_VALID);
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
          stm32_seteptxstatus(epno, USB_EPR_STATTX_STALL);
        }
      else
        {
          /* OUT endpoint */ 

          priv->rxstatus = USB_EPR_STATRX_STALL;
          stm32_seteprxstatus(epno, USB_EPR_STATRX_STALL);
        }
    }

  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Device Controller Operations
 ****************************************************************************/
/****************************************************************************
 * Name: stm32_allocep
 ****************************************************************************/

static struct usbdev_ep_s *stm32_allocep(struct usbdev_s *dev, uint8_t epno,
                                         bool in, uint8_t eptype)
{
  struct stm32_usbdev_s *priv = (struct stm32_usbdev_s *)dev;
  struct stm32_ep_s *privep = NULL;
  uint8_t epset = STM32_ENDP_ALLSET;
  int bufno;

  usbtrace(TRACE_DEVALLOCEP, (uint16_t)epno);
#ifdef CONFIG_DEBUG
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), 0);
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

      if (epno >= STM32_NENDPOINTS)
        {
          usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADEPNO), (uint16_t)epno);
          return NULL;
        }

      /* Convert the logical address to a physical OUT endpoint address and
       * remove all of the candidate endpoints from the bitset except for the
       * the IN/OUT pair for this logical address.
       */

      epset = STM32_ENDP_BIT(epno);
    }

  /* Check if the selected endpoint number is available */

  privep = stm32_epreserve(priv, epset);
  if (!privep)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_EPRESERVE), (uint16_t)epset);
      goto errout;
    }
  epno = USB_EPNO(privep->ep.eplog);

  /* Allocate a PMA buffer for this endpoint */

#warning "REVISIT: Should configure BULK EPs using double buffer feature"
  bufno = stm32_epallocpma(priv);
  if (bufno < 0)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_EPBUFFER), 0);
      goto errout_with_ep;
    }
  privep->bufno = (uint8_t)bufno;
  return &privep->ep;

errout_with_ep:
  stm32_epunreserve(priv, privep);
errout:
  return NULL;
}

/****************************************************************************
 * Name: stm32_freeep
 ****************************************************************************/

static void stm32_freeep(struct usbdev_s *dev, struct usbdev_ep_s *ep)
{
  struct stm32_usbdev_s *priv;
  struct stm32_ep_s *privep;

#ifdef CONFIG_DEBUG
  if (!dev || !ep)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), 0);
      return;
    }
#endif
  priv   = (struct stm32_usbdev_s *)dev;
  privep = (struct stm32_ep_s *)ep;
  usbtrace(TRACE_DEVFREEEP, (uint16_t)USB_EPNO(ep->eplog));

  if (priv && privep)
    {
      /* Free the PMA buffer assigned to this endpoint */

      stm32_epfreepma(priv, privep);

      /* Mark the endpoint as available */

      stm32_epunreserve(priv, privep);
    }
}

/****************************************************************************
 * Name: stm32_getframe
 ****************************************************************************/

static int stm32_getframe(struct usbdev_s *dev)
{
  uint16_t fnr;

#ifdef CONFIG_DEBUG
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Return the last frame number detected by the hardware */

  fnr = stm32_getreg(STM32_USB_FNR);
  usbtrace(TRACE_DEVGETFRAME, fnr);
  return (fnr & USB_FNR_FN_MASK);
}

/****************************************************************************
 * Name: stm32_wakeup
 ****************************************************************************/

static int stm32_wakeup(struct usbdev_s *dev)
{
  struct stm32_usbdev_s *priv = (struct stm32_usbdev_s *)dev;
  irqstate_t flags;

  usbtrace(TRACE_DEVWAKEUP, 0);
#ifdef CONFIG_DEBUG
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Start the resume sequence.  The actual resume steps will be driven
   * by the ESOF interrupt.
   */

  flags = irqsave();
  stm32_initresume(priv);
  priv->rsmstate = RSMSTATE_STARTED;

  /* Disable the SUSP interrupt (until we are fully resumed), disable
   * the WKUP interrupt (we are already waking up), and enable the
   * ESOF interrupt that will drive the resume operations.  Clear any
   * pending ESOF interrupt.
   */

  stm32_setimask(priv, USB_CNTR_ESOFM, USB_CNTR_WKUPM|USB_CNTR_SUSPM);
  stm32_putreg(~USB_ISTR_ESOF, STM32_USB_ISTR);
  irqrestore(flags);
  return OK;
}

/****************************************************************************
 * Name: stm32_selfpowered
 ****************************************************************************/

static int stm32_selfpowered(struct usbdev_s *dev, bool selfpowered)
{
  struct stm32_usbdev_s *priv = (struct stm32_usbdev_s *)dev;

  usbtrace(TRACE_DEVSELFPOWERED, (uint16_t)selfpowered);

#ifdef CONFIG_DEBUG
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), 0);
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
 * Name: stm32_reset
 ****************************************************************************/

static void stm32_reset(struct stm32_usbdev_s *priv)
{
  int epno;

  /* Put the USB controller in reset, disable all interrupts */

  stm32_putreg(USB_CNTR_FRES, STM32_USB_CNTR);

  /* Tell the class driver that we are disconnected.  The class driver
   * should then accept any new configurations.
   */

  CLASS_DISCONNECT(priv->driver, &priv->usbdev);

  /* Reset the device state structure */

  priv->ep0state  = EP0STATE_IDLE;
  priv->rsmstate  = RSMSTATE_IDLE;
  priv->rxpending = false;

  /* Reset endpoints */

  for (epno = 0; epno < STM32_NENDPOINTS; epno++)
    {
      struct stm32_ep_s *privep = &priv->eplist[epno];

      /* Cancel any queued requests.  Since they are canceled
       * with status -ESHUTDOWN, then will not be requeued
       * until the configuration is reset.  NOTE:  This should
       * not be necessary... the CLASS_DISCONNECT above should
       * result in the class implementation calling stm32_epdisable
       * for each of its configured endpoints.
       */

      stm32_cancelrequests(privep);

      /* Reset endpoint status */

      privep->stalled   = false;
      privep->halted    = false;
      privep->txbusy    = false;
      privep->txnullpkt = false;
    }

  /* Re-configure the USB controller in its initial, unconnected state */

  stm32_hwreset(priv);
  priv->usbdev.speed = USB_SPEED_FULL;
} 

/****************************************************************************
 * Name: stm32_hwreset
 ****************************************************************************/

static void stm32_hwreset(struct stm32_usbdev_s *priv)
{
  /* Put the USB controller into reset, clear all interrupt enables */

  stm32_putreg(USB_CNTR_FRES, STM32_USB_CNTR);

  /* Disable interrupts (and perhaps take the USB controller out of reset) */

  priv->imask = 0;
  stm32_putreg(priv->imask, STM32_USB_CNTR);

  /* Set the STM32 BTABLE address */

  stm32_putreg(STM32_BTABLE_ADDRESS & 0xfff8, STM32_USB_BTABLE);

  /* Initialize EP0 */

  stm32_seteptype(EP0, USB_EPR_EPTYPE_CONTROL);
  stm32_seteptxstatus(EP0, USB_EPR_STATTX_NAK);
  stm32_seteprxaddr(EP0, STM32_EP0_RXADDR);
  stm32_seteprxcount(EP0, STM32_EP0MAXPACKET);
  stm32_seteptxaddr(EP0, STM32_EP0_TXADDR);
  stm32_clrstatusout(EP0);
  stm32_seteprxstatus(EP0, USB_EPR_STATRX_VALID);

  /* Set the device to respond on default address */

  stm32_setdevaddr(priv, 0);

  /* Clear any pending interrupts */

  stm32_putreg(0, STM32_USB_ISTR);

  /* Enable interrupts at the USB controller */

  stm32_setimask(priv, STM32_CNTR_SETUP, (USB_CNTR_ALLINTS & ~STM32_CNTR_SETUP));
  stm32_dumpep(EP0);
}

/****************************************************************************
 * Name: stm32_hwsetup
 ****************************************************************************/

static void stm32_hwsetup(struct stm32_usbdev_s *priv)
{
  int epno;

  /* Power the USB controller, put the USB controller into reset, disable
   * all USB interrupts
   */

  stm32_putreg(USB_CNTR_FRES|USB_CNTR_PDWN, STM32_USB_CNTR);

  /* Disconnect the device / disable the pull-up.  We don't want the
   * host to enumerate us until the class driver is registered.
   */ 

  stm32_usbpullup(&priv->usbdev, false);
  
  /* Initialize the device state structure.  NOTE: many fields
   * have the initial value of zero and, hence, are not explicitly
   * initialized here.
   */

  memset(priv, 0, sizeof(struct stm32_usbdev_s));
  priv->usbdev.ops   = &g_devops;
  priv->usbdev.ep0   = &priv->eplist[EP0].ep;
  priv->epavail      = STM32_ENDP_ALLSET & ~STM32_ENDP_BIT(EP0);
  priv->bufavail     = STM32_BUFFER_ALLSET & ~STM32_BUFFER_EP0;

  /* Initialize the endpoint list */

  for (epno = 0; epno < STM32_NENDPOINTS; epno++)
    {
      /* Set endpoint operations, reference to driver structure (not
       * really necessary because there is only one controller), and
       * the (physical) endpoint number which is just the index to the
       * endpoint.
       */

      priv->eplist[epno].ep.ops    = &g_epops;
      priv->eplist[epno].dev       = priv;
      priv->eplist[epno].ep.eplog  = epno;

      /* We will use a fixed maxpacket size for all endpoints (perhaps
       * ISOC endpoints could have larger maxpacket???).  A smaller
       * packet size can be selected when the endpoint is configured.
       */

      priv->eplist[epno].ep.maxpacket = STM32_MAXPACKET_SIZE;
    }

  /* Select a smaller endpoint size for EP0 */

#if STM32_EP0MAXPACKET < STM32_MAXPACKET_SIZE
  priv->eplist[EP0].ep.maxpacket = STM32_EP0MAXPACKET;
#endif

  /* Configure the USB controller.  USB uses the following GPIO pins:
   *
   *   PA9  - VBUS
   *   PA10 - ID
   *   PA11 - DM
   *   PA12 - DP
   *
   * "As soon as the USB is enabled, these pins [DM and DP] are connected to
   * the USB internal transceiver automatically."
   */

  /* Power up the USB controller, holding it in reset.  There is a delay of
   * about 1uS after applying power before the USB will behave predictably.
   * A 5MS delay is more than enough.  NOTE that we leave the USB controller
   * in the reset state; the hardware will not be initialized until the
   * class driver has been bound.
   */

  stm32_putreg(USB_CNTR_FRES, STM32_USB_CNTR);
  up_mdelay(5);
}

/****************************************************************************
 * Name: stm32_hwshutdown
 ****************************************************************************/

static void stm32_hwshutdown(struct stm32_usbdev_s *priv)
{
  priv->usbdev.speed = USB_SPEED_UNKNOWN;

  /* Disable all interrupts and force the USB controller into reset */ 

  stm32_putreg(USB_CNTR_FRES, STM32_USB_CNTR);

  /* Clear any pending interrupts */ 

  stm32_putreg(0, STM32_USB_ISTR);

  /* Disconnect the device / disable the pull-up */ 

  stm32_usbpullup(&priv->usbdev, false);
  
  /* Power down the USB controller */

  stm32_putreg(USB_CNTR_FRES|USB_CNTR_PDWN, STM32_USB_CNTR);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: up_usbinitialize
 * Description:
 *   Initialize the USB driver
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

  struct stm32_usbdev_s *priv = &g_usbdev;

  usbtrace(TRACE_DEVINIT, 0);
  stm32_checksetup();

  /* Configure USB GPIO alternate function pins */

#ifdef CONFIG_STM32_STM32F30XX
  (void)stm32_configgpio(GPIO_USB_DM);
  (void)stm32_configgpio(GPIO_USB_DP);
#endif

  /* Power up the USB controller, but leave it in the reset state */

  stm32_hwsetup(priv);

  /* Remap the USB interrupt as needed (Only supported by the STM32 F3 family) */

#ifdef CONFIG_STM32_STM32F30XX
#  ifdef CONFIG_STM32_USB_ITRMP
  /* Clear the ITRMP bit to use the legacy, shared USB/CAN interrupts */

  modifyreg32(STM32_RCC_APB1ENR, SYSCFG_CFGR1_USB_ITRMP, 0);
#  else
  /* Set the ITRMP bit to use the STM32 F3's dedicated USB interrupts */

  modifyreg32(STM32_RCC_APB1ENR, 0, SYSCFG_CFGR1_USB_ITRMP);
#  endif
#endif

  /* Attach USB controller interrupt handlers.  The hardware will not be
   * initialized and interrupts will not be enabled until the class device
   * driver is bound.  Getting the IRQs here only makes sure that we have
   * them when we need them later.
   */

  if (irq_attach(STM32_IRQ_USBHP, stm32_hpinterrupt) != 0)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_IRQREGISTRATION),
               (uint16_t)STM32_IRQ_USBHP);
      goto errout;
    }

  if (irq_attach(STM32_IRQ_USBLP, stm32_lpinterrupt) != 0)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_IRQREGISTRATION),
               (uint16_t)STM32_IRQ_USBLP);
      goto errout;
    }
  return;

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

  struct stm32_usbdev_s *priv = &g_usbdev;
  irqstate_t flags;

  flags = irqsave();
  usbtrace(TRACE_DEVUNINIT, 0);

  /* Disable and detach the USB IRQs */

  up_disable_irq(STM32_IRQ_USBHP);
  up_disable_irq(STM32_IRQ_USBLP);
  irq_detach(STM32_IRQ_USBHP);
  irq_detach(STM32_IRQ_USBLP);

  if (priv->driver)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_DRIVERREGISTERED), 0);
      usbdev_unregister(priv->driver);
    }

  /* Put the hardware in an inactive state */

  stm32_hwshutdown(priv);
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

  struct stm32_usbdev_s *priv = &g_usbdev;
  int ret;

  usbtrace(TRACE_DEVREGISTER, 0);

#ifdef CONFIG_DEBUG
  if (!driver || !driver->ops->bind || !driver->ops->unbind ||
      !driver->ops->disconnect || !driver->ops->setup)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }

  if (priv->driver)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_DRIVER), 0);
      return -EBUSY;
    }
#endif

  /* First hook up the driver */

  priv->driver = driver;

  /* Then bind the class driver */

  ret = CLASS_BIND(driver, &priv->usbdev);
  if (ret)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BINDFAILED), (uint16_t)-ret);
      priv->driver = NULL;
    }
  else
    {
      /* Setup the USB controller -- enabling interrupts at the USB controller */

      stm32_hwreset(priv);

      /* Enable USB controller interrupts at the NVIC */

      up_enable_irq(STM32_IRQ_USBHP);
      up_enable_irq(STM32_IRQ_USBLP);

      /* Set the interrrupt priority */

      up_prioritize_irq(STM32_IRQ_USBHP, CONFIG_USB_PRI);
      up_prioritize_irq(STM32_IRQ_USBLP, CONFIG_USB_PRI);

      /* Enable pull-up to connect the device.  The host should enumerate us
       * some time after this
       */

      stm32_usbpullup(&priv->usbdev, true);
      priv->usbdev.speed = USB_SPEED_FULL;
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

  struct stm32_usbdev_s *priv = &g_usbdev;
  irqstate_t flags;

  usbtrace(TRACE_DEVUNREGISTER, 0);

#ifdef CONFIG_DEBUG
  if (driver != priv->driver)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Reset the hardware and cancel all requests.  All requests must be
   * canceled while the class driver is still bound.
   */

  flags = irqsave();
  stm32_reset(priv);

  /* Unbind the class driver */

  CLASS_UNBIND(driver, &priv->usbdev);

  /* Disable USB controller interrupts (but keep them attached) */

  up_disable_irq(STM32_IRQ_USBHP);
  up_disable_irq(STM32_IRQ_USBLP);

  /* Put the hardware in an inactive state.  Then bring the hardware back up
   * in the reset state (this is probably not necessary, the stm32_reset()
   * call above was probably sufficient).
   */

  stm32_hwshutdown(priv);
  stm32_hwsetup(priv);

  /* Unhook the driver */

  priv->driver = NULL;
  irqrestore(flags);
  return OK;
}

#endif /* CONFIG_USBDEV && CONFIG_STM32_USB */
