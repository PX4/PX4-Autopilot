/*******************************************************************************
 * arch/arm/src/stm32/stm32_usbdev.c
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

#if defined(CONFIG_USBDEV) && defined(CONFIG_STM32_OTGFSDEV)

/*******************************************************************************
 * Definitions
 *******************************************************************************/
/* Configuration ***************************************************************/

#ifndef CONFIG_USBDEV_EP0_MAXSIZE
#  define CONFIG_USBDEV_EP0_MAXSIZE 64
#endif

#ifndef CONFIG_USBDEV_MAXPOWER
#  define CONFIG_USBDEV_MAXPOWER 100  /* mA */
#endif

#ifndef CONFIG_USBDEV_RXFIFO_SIZE
#  define CONFIG_USBDEV_RXFIFO_SIZE 128
#endif

#ifndef CONFIG_USBDEV_EP0_TXFIFO_SIZE
#  define CONFIG_USBDEV_EP0_TXFIFO_SIZE 128
#endif

#if CONFIG_USBDEV_EP0_TXFIFO_SIZE < 16 || CONFIG_USBDEV_EP0_TXFIFO_SIZE > 256
#  error "CONFIG_USBDEV_EP0_TXFIFO_SIZE is out of range"
#endif

#ifndef CONFIG_USBDEV_EP1_TXFIFO_SIZE
#  define CONFIG_USBDEV_EP1_TXFIFO_SIZE 128
#endif

#ifndef CONFIG_USBDEV_EP2_TXFIFO_SIZE
#  define CONFIG_USBDEV_EP2_TXFIFO_SIZE 128
#endif

#ifndef CONFIG_USBDEV_EP3_TXFIFO_SIZE
#  define CONFIG_USBDEV_EP3_TXFIFO_SIZE 128
#endif

/* Enable reading SOF from interrupt handler vs. simply reading on demand.  Probably
 * a bad idea... Unless there is some issue with sampling the SOF from hardware
 * asynchronously.
 */

#ifdef CONFIG_STM32_USBDEV_FRAME_INTERRUPT
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

#define STM32_TRACEERR_ALLOCFAIL            0x0001
#define STM32_TRACEERR_BADCLEARFEATURE      0x0002
#define STM32_TRACEERR_BADDEVGETSTATUS      0x0003
#define STM32_TRACEERR_BADEPNO              0x0004
#define STM32_TRACEERR_BADEPGETSTATUS       0x0005
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
#define STM32_TRACEERR_EP0SETUPSTALLED      0x0011
#define STM32_TRACEERR_EPINNULLPACKET       0x0012
#define STM32_TRACEERR_EPOUTNULLPACKET      0x0013
#define STM32_TRACEERR_INVALIDCTRLREQ       0x0014
#define STM32_TRACEERR_INVALIDPARMS         0x0015
#define STM32_TRACEERR_IRQREGISTRATION      0x0016
#define STM32_TRACEERR_NOEP                 0x0017
#define STM32_TRACEERR_NOTCONFIGURED        0x0018
#define STM32_TRACEERR_REQABORTED           0x0019

/* Trace interrupt codes */

#define STM32_TRACEINTID_USB                0x0001 /* USB Interrupt entry/exit */

#define STM32_TRACEINTID_EPOUT              0x0101 /* First level interrupt decode */
#define STM32_TRACEINTID_EPIN               0x0102
#define STM32_TRACEINTID_MISMATCH           0x0103
#define STM32_TRACEINTID_WAKEUP             0x0104
#define STM32_TRACEINTID_SUSPEND            0x0105
#define STM32_TRACEINTID_SOF                0x0106
#define STM32_TRACEINTID_RXFIFO             0x0107
#define STM32_TRACEINTID_DEVRESET           0x0108
#define STM32_TRACEINTID_ENUMDNE            0x0109
#define STM32_TRACEINTID_IISOIXFR           0x010a
#define STM32_TRACEINTID_IPXFR              0x010b
#define STM32_TRACEINTID_SRQ                0x010c
#define STM32_TRACEINTID_OTG                0x010d

#define STM32_TRACEINTID_EPOUT_XFRC         0x0200 /* EPOUT second level decode */
#define STM32_TRACEINTID_EPOUT_EPDISD       0x0201
#define STM32_TRACEINTID_EPOUT_SETUP        0x0202
#define STM32_TRACEINTID_EPIN_XFRC          0x0210 /* EPIN second level decode */
#define STM32_TRACEINTID_EPIN_TOC           0x0211
#define STM32_TRACEINTID_EPIN_ITTXFE        0x0212
#define STM32_TRACEINTID_EPIN_INEPNE        0x0213
#define STM32_TRACEINTID_EPIN_EPDISD        0x0214
#define STM32_TRACEINTID_EPIN_TXFE          0x0215

#define STM32_TRACEINTID_CLEARFEATURE       0x0220 /* Subsequence interrupt decode */
#define STM32_TRACEINTID_DEVGETSTATUS       0x0221
#define STM32_TRACEINTID_DISPATCH           0x0222
#define STM32_TRACEINTID_EP0COMPLETE        0x0223
#define STM32_TRACEINTID_EP0NAK             0x0224
#define STM32_TRACEINTID_EP0SETUP           0x0225
#define STM32_TRACEINTID_EPGETSTATUS        0x0226
#define STM32_TRACEINTID_EPIN               0x0227
#define STM32_TRACEINTID_EPINQEMPTY         0x0228
#define STM32_TRACEINTID_EP0INSETADDRESS    0x0229
#define STM32_TRACEINTID_EPOUT              0x022a
#define STM32_TRACEINTID_EPOUTQEMPTY        0x022b
#define STM32_TRACEINTID_EP0SETUPSETADDRESS 0x022c
#define STM32_TRACEINTID_GETCONFIG          0x022d
#define STM32_TRACEINTID_GETSETDESC         0x022e
#define STM32_TRACEINTID_GETSETIF           0x022f
#define STM32_TRACEINTID_GETSTATUS          0x0230
#define STM32_TRACEINTID_IFGETSTATUS        0x0231
#define STM32_TRACEINTID_SETCONFIG          0x0232
#define STM32_TRACEINTID_SETFEATURE         0x0233
#define STM32_TRACEINTID_SYNCHFRAME         0x0234

/* Hardware interface **********************************************************/
/* Endpoint Transfer Descriptor */
/* DTD nextdesc field */

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

/* Queue head (DQH) */
/* DQH capability field */

#define DQH_CAPABILITY_MULT_VARIABLE (0 << 30)    /* Bits 30-31 : Number of packets executed per transaction descriptor */
#define DQH_CAPABILITY_MULT_NUM(n)   ((n) << 30)
#define DQH_CAPABILITY_ZLT           (1 << 29)    /* Bit 29     : Zero Length Termination Select */
#define DQH_CAPABILITY_MAX_PACKET(n) ((n) << 16)  /* Bits 16-29 : Maximum packet size of associated endpoint (<1024) */
#define DQH_CAPABILITY_IOS           (1 << 15)    /* Bit 15     : Interrupt on Setup */

/* Endpoints ******************************************************************/

/* Number of endpoints */

#define STM32_NENDPOINTS             (4)          /* ep0-3 x2 for IN and OUT */

/* Odd physical endpoint numbers are IN; even are OUT */

#define STM32_EPPHYIN(epphy)         (((epphy)&1)!=0)
#define STM32_EPPHYOUT(epphy)        (((epphy)&1)==0)

#define STM32_EPPHYIN2LOG(epphy)     (((uint8_t)(epphy)>>1)|USB_DIR_IN)
#define STM32_EPPHYOUT2LOG(epphy)    (((uint8_t)(epphy)>>1)|USB_DIR_OUT)

/* Endpoint 0 is special... */

#define STM32_EP0_OUT                (0)
#define STM32_EP0_IN                 (1)

/* Each endpoint has somewhat different characteristics */

#define STM32_EPALLSET               (0xff)       /* All endpoints */
#define STM32_EPOUTSET               (0x55)       /* Even phy endpoint numbers are OUT EPs */
#define STM32_EPINSET                (0xaa)       /* Odd endpoint numbers are IN EPs */
#define STM32_EPCTRLSET              (0x03)       /* EP0 IN/OUT are control endpoints */
#define STM32_EPINTRSET              (0xa8)       /* Interrupt endpoints */
#define STM32_EPBULKSET              (0xfc)       /* Bulk endpoints */
#define STM32_EPISOCSET              (0xfc)       /* Isochronous endpoints */

/* Maximum packet sizes for full speed endpoints */

#define STM32_MAXPACKET              (64)         /* Max packet size (1-64) */

/* The address of the endpoint control register */

#define STM32_USBDEV_ENDPTCTRL(epphy) (STM32_USBDEV_ENDPTCTRL0 + ((epphy)>>1)*4)

/* Endpoint bit position in SETUPSTAT, PRIME, FLUSH, STAT, COMPLETE registers */

#define STM32_ENDPTSHIFT(epphy)      (STM32_EPPHYIN(epphy) ? (16 + ((epphy) >> 1)) : ((epphy) >> 1))
#define STM32_ENDPTMASK(epphy)       (1 << STM32_ENDPTSHIFT(epphy))
#define STM32_ENDPTMASK_ALL          0x000f000f

/* Delays **********************************************************************/

#define STM32_READY_DELAY            200000

/* Request queue operations ****************************************************/

#define stm32_rqempty(ep)            ((ep)->head == NULL)
#define stm32_rqpeek(ep)             ((ep)->head)

/*******************************************************************************
 * Private Types
 *******************************************************************************/
/* Parsed control request */

struct stm32_ctrlreq_s
{
  uint8_t  type;
  uint8_t  req;
  uint16_t value;
  uint16_t index;
  uint16_t len;
};

/* Device Status */

enum stm32_devstate_e
{
  DEVSTATE_DEFAULT = 0, 
  DEVSTATE_ADDRESSED
  DEVSTATE_CONFIGURED
  DEVSTATE_SUSPENDED
};

/* Endpoint 0 states */

enum stm32_ep0state_e
{
  EP0STATE_IDLE = 0,    /* Idle State, leave on receiving a setup packet or epsubmit */
  EP0STATE_SETUP_OUT,   /* Setup Packet received - SET/CLEAR */
  EP0STATE_SETUP_IN,    /* Setup Packet received - GET */
  EP0STATE_SHORTWRITE,  /* Short write without a usb_request */
  EP0STATE_NAK_OUT,     /* Waiting for Host to elicit status phase (GET) */
  EP0STATE_NAK_IN,      /* Waiting for Host to elicit status phase (SET/CLEAR) */
  EP0STATE_STATUS_OUT,  /* Wait for status phase to complete */
  EP0STATE_STATUS_IN,   /* Wait for status phase to complete */
  EP0STATE_DATA_IN,
  EP0STATE_DATA_OUT
};

/* This represents a Endpoint Transfer Descriptor - note these must be 32 byte
 * aligned.
 */

struct stm32_dtd_s
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

/* This represents a queue head  - note these must be aligned to a 2048 byte
 * boundary.
 */

struct stm32_dqh_s
{
  uint32_t                capability;  /* Endpoint capability */
  uint32_t                currdesc;    /* Current dTD pointer */
  struct stm32_dtd_s      overlay;     /* DTD overlay */
  volatile uint32_t       setup[2];    /* Set-up buffer */
  uint32_t                gap[4];      /* align to 64 bytes */
};

/* A container for a request so that the request may be retained in a list */

struct stm32_req_s
{
  struct usbdev_req_s  req;           /* Standard USB request */
  struct stm32_req_s  *flink;         /* Supports a singly linked list */
};

/* This is the internal representation of an endpoint */

struct stm32_ep_s
{
  /* Common endpoint fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_ep_s
   * to struct stm32_ep_s.
   */

  struct usbdev_ep_s      ep;          /* Standard endpoint structure */

  /* STM32-specific fields */

  struct stm32_usbdev_s *dev;          /* Reference to private driver data */
  struct stm32_req_s    *head;         /* Request list for this endpoint */
  struct stm32_req_s    *tail;
  uint8_t                epphy;        /* Physical EP address */
  uint8_t                stalled:1;    /* 1: Endpoint is stalled */
  uint8_t                isin:1;       /* 1: IN Endpoint */
  uint8_t                odd:1;        /* 1: Odd frame */
};

/* This structure retains the state of the USB device controller */

struct stm32_usbdev_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_s
   * to struct stm32_usbdev_s.
   */

  struct usbdev_s         usbdev;

  /* The bound device class driver */

  struct usbdevclass_driver_s *driver;

  /* STM32-specific fields */

  uint8_t                 stalled:1;     /* 1: Protocol stalled */
  uint8_t                 selfpowered:1; /* 1: Device is self powered */
  uint8_t                 paddrset:1;    /* 1: Peripheral addr has been set */
  uint8_t                 connected:1;   /* 1: Host connected */
  uint8_t                 wakeup:1;      /* 1: Device remote wake-up */

  uint8_t                 devstate;      /* See enum stm32_devstate_e */
  uint8_t                 ep0state;      /* See enum stm32_ep0state_e */
  uint8_t                 savestate;     /* Saved devstate */
  uint8_t                 ep0resp[2];    /* buffer for EP0 short transfers */
  uint8_t                 paddr;         /* Address assigned by SETADDRESS */
  uint8_t                 devcfg;        /* Selected configuration index */
  uint8_t                 testmode;      /* Selected test mode */

  uint32_t                softprio;      /* Bitset of high priority interrupts */
  uint32_t                epavail;       /* Bitset of available endpoints */
#ifdef CONFIG_STM32_USBDEV_FRAME_INTERRUPT
  uint32_t                sof;           /* Last start-of-frame */
#endif
  struct usb_ctrlreq_s    ctrlreq;       /* Received SETUP request */

  /* The endpoint list */

  struct stm32_ep_s       epin[STM32_NENDPOINTS];
  struct stm32_ep_s       epout[STM32_NENDPOINTS];
};

/*******************************************************************************
 * Private Function Prototypes
 *******************************************************************************/

/* Register operations ********************************************************/

#if defined(CONFIG_STM32_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
static uint32_t    stm32_getreg(uint32_t addr);
static void        stm32_putreg(uint32_t val, uint32_t addr);
#else
# define stm32_getreg(addr)     getreg32(addr)
# define stm32_putreg(val,addr) putreg32(val,addr)
#endif

static inline void stm32_clrbits(uint32_t mask, uint32_t addr);
static inline void stm32_setbits(uint32_t mask, uint32_t addr);
static inline void stm32_chgbits(uint32_t mask, uint32_t val, uint32_t addr);

/* Request queue operations ****************************************************/

static FAR struct stm32_req_s *stm32_rqdequeue(FAR struct stm32_ep_s *privep);
static bool       stm32_rqenqueue(FAR struct stm32_ep_s *privep,
                    FAR struct stm32_req_s *req);

/* Low level data transfers and request operations *****************************/

static inline void stm32_writedtd(struct stm32_dtd_s *dtd, const uint8_t *data,
                     uint32_t nbytes);
static inline void stm32_queuedtd(uint8_t epphy, struct stm32_dtd_s *dtd);
static inline void stm32_ep0xfer(uint8_t epphy, uint8_t *data, uint32_t nbytes);
static void        stm32_ep0read(FAR uint8_t *dest, uint16_t len)

static inline void stm32_set_address(struct stm32_usbdev_s *priv, uint16_t address);

static void        stm32_flushep(struct stm32_ep_s *privep);

static int         stm32_progressep(struct stm32_ep_s *privep);
static inline void stm32_abortrequest(struct stm32_ep_s *privep,
                     struct stm32_req_s *privreq, int16_t result);
static void        stm32_reqcomplete(struct stm32_ep_s *privep,
                     struct stm32_req_s *privreq, int16_t result);

static void        stm32_cancelrequests(struct stm32_ep_s *privep, int16_t status);

/* Interrupt handling **********************************************************/

static struct      stm32_ep_s *stm32_epfindbyaddr(struct stm32_usbdev_s *priv,
                     uint16_t eplog);
static void        stm32_dispatchrequest(struct stm32_usbdev_s *priv,
                     const struct usb_ctrlreq_s *ctrl);
static void        stm32_ep0configure(struct stm32_usbdev_s *priv);
static void        stm32_usbreset(struct stm32_usbdev_s *priv);

static inline void stm32_ep0state(struct stm32_usbdev_s *priv, uint16_t state);
static void        stm32_ep0setup(struct stm32_usbdev_s *priv);

static void        stm32_ep0complete(struct stm32_usbdev_s *priv, uint8_t epphy);
static void        stm32_ep0nak(struct stm32_usbdev_s *priv, uint8_t epphy);
static bool        stm32_epcomplete(struct stm32_usbdev_s *priv, uint8_t epphy);

/* Second level interrupt processing */

static inline void stm32_epininterrupt(FAR struct stm32_usbdev_s *priv);
static inline void stm32_epoutinterrupt(FAR struct stm32_usbdev_s *priv);

/* First level interrupt processing */

static int         stm32_usbinterrupt(int irq, FAR void *context);

/* Endpoint operations *********************************************************/

static int         stm32_epconfigure(FAR struct usbdev_ep_s *ep,
                     const struct usb_epdesc_s *desc, bool last);
static int         stm32_epdisable(FAR struct usbdev_ep_s *ep);
static FAR struct usbdev_req_s *stm32_epallocreq(FAR struct usbdev_ep_s *ep);
static void        stm32_epfreereq(FAR struct usbdev_ep_s *ep,
                     FAR struct usbdev_req_s *);
#ifdef CONFIG_ARCH_USBDEV_DMA
static void       *stm32_epallocbuffer(FAR struct usbdev_ep_s *ep, unsigned bytes);
static void        stm32_epfreebuffer(FAR struct usbdev_ep_s *ep, FAR void *buf);
#endif
static int         stm32_epsubmit(FAR struct usbdev_ep_s *ep,
                     struct usbdev_req_s *req);
static int         stm32_epcancel(FAR struct usbdev_ep_s *ep,
                     struct usbdev_req_s *req);
static int         stm32_epstall(FAR struct usbdev_ep_s *ep, bool resume);

static FAR struct usbdev_ep_s *stm32_allocep(FAR struct usbdev_s *dev,
                     uint8_t epno, bool in, uint8_t eptype);
static void        stm32_freeep(FAR struct usbdev_s *dev,
                     FAR struct usbdev_ep_s *ep);

/* USB device controller operations ********************************************/

static int         stm32_getframe(struct usbdev_s *dev);
static int         stm32_wakeup(struct usbdev_s *dev);
static int         stm32_selfpowered(struct usbdev_s *dev, bool selfpowered);
static int         stm32_pullup(struct usbdev_s *dev, bool enable);

/* Initialization **************************************************************/

static int         stm32_flushtxfifo(FAR struct stm32_usbdev_s *priv,
                     uint32_t txfnum);
static int         stm32_flushrxfifo(FAR struct stm32_usbdev_s *priv);
static void        stm32_swinitialize(FAR struct stm32_usbdev_s *priv);
static void        stm32_hwinitialize(FAR struct stm32_usbdev_s *priv);

/*******************************************************************************
 * Private Data
 *******************************************************************************/
/* Since there is only a single USB interface, all status information can be
 * be simply retained in a single global instance.
 */

static struct stm32_usbdev_s g_otgfsdev;

static struct stm32_dqh_s __attribute__((aligned(2048))) g_qh[STM32_NENDPOINTS];
static struct stm32_dtd_s __attribute__((aligned(32)))   g_td[STM32_NENDPOINTS];

static const struct usbdev_epops_s g_epops =
{
  .configure   = stm32_epconfigure,
  .disable     = stm32_epdisable,
  .allocreq    = stm32_epallocreq,
  .freereq     = stm32_epfreereq,
#ifdef CONFIG_ARCH_USBDEV_DMA
  .allocbuffer = stm32_epallocbuffer,
  .freebuffer  = stm32_epfreebuffer,
#endif
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
  .pullup      = stm32_pullup,
};

/*******************************************************************************
 * Public Data
 *******************************************************************************/

/*******************************************************************************
 * Private Functions
 *******************************************************************************/

/*******************************************************************************
 * Name: stm32_getreg
 *
 * Description:
 *   Get the contents of an STM32 register
 *
 *******************************************************************************/

#if defined(CONFIG_STM32_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
static uint32_t stm32_getreg(uint32_t addr)
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
 * Name: stm32_putreg
 *
 * Description:
 *   Set the contents of an STM32 register to a value
 *
 *******************************************************************************/

#if defined(CONFIG_STM32_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
static void stm32_putreg(uint32_t val, uint32_t addr)
{
  /* Show the register value being written */

  lldbg("%08x<-%08x\n", addr, val);

  /* Write the value */

  putreg32(val, addr);
}
#endif

/*******************************************************************************
 * Name: stm32_clrbits
 *
 * Description:
 *   Clear bits in a register
 *
 *******************************************************************************/

static inline void stm32_clrbits(uint32_t mask, uint32_t addr)
{
  uint32_t reg = stm32_getreg(addr);
  reg &= ~mask;
  stm32_putreg(reg, addr);
}

/*******************************************************************************
 * Name: stm32_setbits
 *
 * Description:
 *   Set bits in a register
 *
 *******************************************************************************/

static inline void stm32_setbits(uint32_t mask, uint32_t addr)
{
  uint32_t reg = stm32_getreg(addr);
  reg |= mask;
  stm32_putreg(reg, addr);
}

/*******************************************************************************
 * Name: stm32_chgbits
 *
 * Description:
 *   Change bits in a register
 *
 *******************************************************************************/

static inline void stm32_chgbits(uint32_t mask, uint32_t val, uint32_t addr)
{
  uint32_t reg = stm32_getreg(addr);
  reg &= ~mask;
  reg |= val;
  stm32_putreg(reg, addr);
}

/*******************************************************************************
 * Name: stm32_rqdequeue
 *
 * Description:
 *   Remove a request from an endpoint request queue
 *
 *******************************************************************************/

static FAR struct stm32_req_s *stm32_rqdequeue(FAR struct stm32_ep_s *privep)
{
  FAR struct stm32_req_s *ret = privep->head;

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
 * Name: stm32_rqenqueue
 *
 * Description:
 *   Add a request from an endpoint request queue
 *
 *******************************************************************************/

static bool stm32_rqenqueue(FAR struct stm32_ep_s *privep,
                              FAR struct stm32_req_s *req)
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
 * Name: stm32_writedtd
 *
 * Description:
 *   Initialise a DTD to transfer the data
 *
 *******************************************************************************/

static inline void stm32_writedtd(struct stm32_dtd_s *dtd, const uint8_t *data, uint32_t nbytes)
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
 * Name: stm32_queuedtd
 *
 * Description:
 *   Add the DTD to the device list
 *
 *******************************************************************************/

static void stm32_queuedtd(uint8_t epphy, struct stm32_dtd_s *dtd)
{
  /* Queue the DTD onto the Endpoint */
  /* NOTE - this only works when no DTD is currently queued */

  g_qh[epphy].overlay.nextdesc = (uint32_t) dtd;
  g_qh[epphy].overlay.config  &= ~(DTD_CONFIG_ACTIVE | DTD_CONFIG_HALTED);

  uint32_t bit = STM32_ENDPTMASK(epphy);

  stm32_setbits (bit, STM32_USBDEV_ENDPTPRIME);
    
  while (stm32_getreg (STM32_USBDEV_ENDPTPRIME) & bit)
    ;
}

/*******************************************************************************
 * Name: stm32_ep0xfer
 *
 * Description:
 *   Schedule a short transfer for Endpoint 0 (IN or OUT)
 *
 *******************************************************************************/

static inline void stm32_ep0xfer(uint8_t epphy, uint8_t *buf, uint32_t nbytes)
{
  struct stm32_dtd_s *dtd = &g_td[epphy];

  stm32_writedtd(dtd, buf, nbytes);
    
  stm32_queuedtd(epphy, dtd);
}

/*******************************************************************************
 * Name: stm32_ep0read
 *
 * Description:
 *   Read a Setup packet from the DTD.
 *
 *******************************************************************************/

static void stm32_ep0read(FAR uint8_t *dest, uint16_t len)
{
  uint32_t regaddr;
  int i;

  /* Get the address of the EP0 FIFO */

  regaddr = STM32_OTGFS_DFIFO_DEP(0);

  /* Read 32-bits and write 4 x 8-bits at time (to avoid unaligned accesses) */

  for (i = 0; i < len; i += 4)
    {
      union
      {
        uint32_t w;
        uint8_t  b[4];
      } data;

      /* Read 1 x 32-bits of EP0 packet data */

      data.w = stm32_getreg(fifo);

      /* Write 4 x 8-bits of EP0 packet data */

      *dest++ = data.b[0];
      *dest++ = data.b[1];
      *dest++ = data.b[2];
      *dest++ = data.b[3];
    }
}

/*******************************************************************************
 * Name: stm32_set_address
 *
 * Description:
 *   Set the devices USB address
 *
 *******************************************************************************/

static inline void stm32_set_address(struct stm32_usbdev_s *priv, uint16_t address)
{
  priv->paddr    = address;
  priv->paddrset = address != 0;

  stm32_chgbits(USBDEV_DEVICEADDR_MASK, priv->paddr << USBDEV_DEVICEADDR_SHIFT, 
                STM32_USBDEV_DEVICEADDR);
}

/*******************************************************************************
 * Name: stm32_flushep
 *
 * Description:
 *   Flush any primed descriptors from this ep
 *
 *******************************************************************************/

static void stm32_flushep(struct stm32_ep_s *privep)
{
  uint32_t mask = STM32_ENDPTMASK(privep->epphy);
  do
    {
      stm32_putreg (mask, STM32_USBDEV_ENDPTFLUSH);
      while ((stm32_getreg(STM32_USBDEV_ENDPTFLUSH) & mask) != 0)
      ;
    }
  while ((stm32_getreg(STM32_USBDEV_ENDPTSTATUS) & mask) != 0);
}

/*******************************************************************************
 * Name: stm32_progressep
 *
 * Description:
 *   Progress the Endpoint by priming the first request into the queue head
 *
 *******************************************************************************/

static int stm32_progressep(struct stm32_ep_s *privep)
{
  struct stm32_dtd_s *dtd = &g_td[privep->epphy];
  struct stm32_req_s *privreq;

  /* Check the request from the head of the endpoint request queue */

  privreq = stm32_rqpeek(privep);
  if (!privreq)
    {
      usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPINQEMPTY), 0);
      return OK;
    }

  /* Ignore any attempt to send a zero length packet */

  if (privreq->req.len == 0)
    {
    /* If the class driver is responding to a setup packet, then wait for the 
     * host to elicit the response */

    if (privep->epphy == STM32_EP0_IN && privep->dev->ep0state == EP0STATE_SETUP_OUT)
      {
        stm32_ep0state(privep->dev, EP0STATE_NAK_IN);
      }
    else
      {
        if (STM32_EPPHYIN(privep->epphy))
          {
            usbtrace(TRACE_DEVERROR(STM32_TRACEERR_EPINNULLPACKET), 0);
          }
        else
          {
            usbtrace(TRACE_DEVERROR(STM32_TRACEERR_EPOUTNULLPACKET), 0);
          }
      }
      
      stm32_reqcomplete(privep, stm32_rqdequeue(privep), OK);
      return OK;
    }

  if (privep->epphy == STM32_EP0_IN)
    {
      stm32_ep0state(privep->dev,  EP0STATE_DATA_IN);
    }
  else if (privep->epphy == STM32_EP0_OUT)
    {
      stm32_ep0state(privep->dev, EP0STATE_DATA_OUT);
    }

  int bytesleft = privreq->req.len - privreq->req.xfrd;

  if (STM32_EPPHYIN(privep->epphy))
    {
      usbtrace(TRACE_WRITE(privep->epphy), privreq->req.xfrd);
    }
  else
    {
      usbtrace(TRACE_READ(privep->epphy), privreq->req.xfrd);
    }

  /* Initialise the DTD to transfer the next chunk */

  stm32_writedtd (dtd, privreq->req.buf + privreq->req.xfrd, bytesleft);

  /* Then queue onto the DQH */

  stm32_queuedtd(privep->epphy, dtd);
  return OK;
}

/*******************************************************************************
 * Name: stm32_abortrequest
 *
 * Description:
 *   Discard a request
 *
 *******************************************************************************/

static inline void stm32_abortrequest(struct stm32_ep_s *privep,
                                      struct stm32_req_s *privreq,
                                      int16_t result)
{
  usbtrace(TRACE_DEVERROR(STM32_TRACEERR_REQABORTED), (uint16_t)privep->epphy);

  /* Save the result in the request structure */

  privreq->req.result = result;

  /* Callback to the request completion handler */

  privreq->req.callback(&privep->ep, &privreq->req);
}

/*******************************************************************************
 * Name: stm32_reqcomplete
 *
 * Description:
 *   Handle termination of the request at the head of the endpoint request queue.
 *
 *******************************************************************************/

static void stm32_reqcomplete(struct stm32_ep_s *privep,
                              struct stm32_req_s *privreq, int16_t result)
{
  /* If endpoint 0, temporarily reflect the state of protocol stalled
   * in the callback.
   */

  bool stalled = privep->stalled;
  if (privep->epphy == STM32_EP0_IN)
    {
      privep->stalled = privep->dev->stalled;
    }

  /* Save the result in the request structure */

  privreq->req.result = result;

  /* Callback to the request completion handler */

  privreq->req.callback(&privep->ep, &privreq->req);

  /* Restore the stalled indication */

  privep->stalled = stalled;
}

/*******************************************************************************
 * Name: stm32_cancelrequests
 *
 * Description:
 *   Cancel all pending requests for an endpoint
 *
 *******************************************************************************/

static void stm32_cancelrequests(struct stm32_ep_s *privep, int16_t status)
{
  if (!stm32_rqempty(privep))
    {
      stm32_flushep(privep);
    }

  while (!stm32_rqempty(privep))
    {
      // FIXME: the entry at the head should be sync'd with the DTD
      // FIXME: only report the error status if the transfer hasn't completed
      usbtrace(TRACE_COMPLETE(privep->epphy),
               (stm32_rqpeek(privep))->req.xfrd);
      stm32_reqcomplete(privep, stm32_rqdequeue(privep), status);
    }
}

/*******************************************************************************
 * Name: stm32_epfindbyaddr
 *
 * Description:
 *   Find the physical endpoint structure corresponding to a logic endpoint
 *   address
 *
 *******************************************************************************/

static struct stm32_ep_s *stm32_epfindbyaddr(struct stm32_usbdev_s *priv,
                         uint16_t eplog)
{
  struct stm32_ep_s *privep;
  int i;

  /* Endpoint zero is a special case */

  if (USB_EPNO(eplog) == 0)
    {
      return &priv->epin[0];
    }

  /* Handle the remaining */

  for (i = 1; i < STM32_NENDPOINTS; i++)
    {
      privep = &priv->epin[i];

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
 * Name: stm32_dispatchrequest
 *
 * Description:
 *   Provide unhandled setup actions to the class driver. This is logically part
 *   of the USB interrupt handler.
 *
 *******************************************************************************/

static void stm32_dispatchrequest(struct stm32_usbdev_s *priv,
                                    const struct usb_ctrlreq_s *ctrl)
{
  int ret = -EIO;

  usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_DISPATCH), 0);
  if (priv->driver)
    {
      /* Forward to the control request to the class driver implementation */

      ret = CLASS_SETUP(priv->driver, &priv->usbdev, ctrl);
    }

  if (ret < 0)
    {
      /* Stall on failure */

      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_DISPATCHSTALL), 0);
      priv->stalled = true;
    }
}

/*******************************************************************************
 * Name: stm32_ep0configure
 *
 * Description:
 *   Reset Usb engine
 *
 *******************************************************************************/

static void stm32_ep0configure(struct stm32_usbdev_s *priv)
{
  /* Enable ep0 IN and ep0 OUT */

  g_qh[STM32_EP0_OUT].capability = (DQH_CAPABILITY_MAX_PACKET(CONFIG_USBDEV_EP0_MAXSIZE) |
                      DQH_CAPABILITY_IOS |
                      DQH_CAPABILITY_ZLT);

  g_qh[STM32_EP0_IN ].capability = (DQH_CAPABILITY_MAX_PACKET(CONFIG_USBDEV_EP0_MAXSIZE) |
                      DQH_CAPABILITY_IOS |
                      DQH_CAPABILITY_ZLT);
  
  g_qh[STM32_EP0_OUT].currdesc = DTD_NEXTDESC_INVALID;
  g_qh[STM32_EP0_IN ].currdesc = DTD_NEXTDESC_INVALID;
  
  /* Enable EP0 */
  stm32_setbits (USBDEV_ENDPTCTRL0_RXE | USBDEV_ENDPTCTRL0_TXE, STM32_USBDEV_ENDPTCTRL0);
}

/*******************************************************************************
 * Name: stm32_usbreset
 *
 * Description:
 *   Reset Usb engine
 *
 *******************************************************************************/

static void stm32_usbreset(struct stm32_usbdev_s *priv)
{
  int epphy;

  /* Disable all endpoints */
#warning "Missing Logic"

  /* Clear all pending interrupts */
#warning "Missing Logic"

  /* Flush all FIFOs */
#warning "Missing Logic"

  /* Reset endpoints */

  for (epphy = 0; epphy < STM32_NENDPOINTS; epphy++)
    {
      struct stm32_ep_s *privep = &priv->epin[epphy];

      stm32_cancelrequests (privep, -ESHUTDOWN);

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

  /* Set USB address to 0 */

  stm32_set_address (priv, 0);

  /* EndPoint 0 initialization */

  stm32_ep0configure(priv);

  /* Re-enable device interrupts */
#warning "Missing Logic"
}

/*******************************************************************************
 * Name: stm32_setstate
 *
 * Description:
 *   Sets the EP0 state and manages the NAK interrupts
 *
 *******************************************************************************/

static inline void stm32_ep0state(struct stm32_usbdev_s *priv, uint16_t state)
{
  priv->ep0state = state;
  
  switch (state)
    {
    case EP0STATE_NAK_IN:
      stm32_putreg (STM32_ENDPTMASK(STM32_EP0_IN), STM32_USBDEV_ENDPTNAKEN);
      break;
    case EP0STATE_NAK_OUT:
      stm32_putreg (STM32_ENDPTMASK(STM32_EP0_OUT), STM32_USBDEV_ENDPTNAKEN);
      break;
    default:
      stm32_putreg(0, STM32_USBDEV_ENDPTNAKEN);
      break;
    }
}

/*******************************************************************************
 * Name: stm32_stdrequest
 *
 * Description:
 *   Handle a stanard request on EP0.  Pick off the things of interest to the
 *   USB device controller driver; pass what is left to the class driver.
 *
 *******************************************************************************/

static inline void stm32_stdrequest(struct stm32_usbdev_s *priv,
                                    FAR struct stm32_ctrlreq_s *ctrlreq)
{
  FAR struct stm32_ep_s *privep;

  /* Handle standard request */

  switch (ctrlreq->req)
    {
    case USB_REQ_GETSTATUS:
      {
        /* type:  device-to-host; recipient = device, interface, endpoint
         * value: 0
         * index: zero interface endpoint
         * len:   2; data = status
         */
  
        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_GETSTATUS), 0);
        if (!priv->paddrset || ctrlreq->len != 2 ||
            (ctrlreq->type & USB_REQ_DIR_IN) == 0 || ctrlreq->value != 0)
          {
            priv->stalled = true;
          }
        else
          {
            switch (ctrlreq->type & USB_REQ_RECIPIENT_MASK)
              {
              case USB_REQ_RECIPIENT_ENDPOINT:
                {
                  usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPGETSTATUS), 0);
                  privep = stm32_epfindbyaddr(priv, ctrlreq->index);
                  if (!privep)
                    {
                      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADEPGETSTATUS), 0);
                      priv->stalled = true;
                    }
                  else
                    {
                      if (privep->stalled)
                        {
                          priv->ep0resp[0] = 1; /* Stalled */
                        }
                      else
                        {
                          priv->ep0resp[0] = 0; /* Not stalled */
                        }

                      priv->ep0resp[1] = 0;
  
                      stm32_ep0xfer(STM32_EP0_IN, priv->ep0resp, 2);
                      stm32_ep0state(priv, EP0STATE_SHORTWRITE);
                    }
                }
                break;
  
              case USB_REQ_RECIPIENT_DEVICE:
                {
                  if (ctrlreq->index == 0)
                    {
                      usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_DEVGETSTATUS), 0);
  
                      /* Features:  Remote Wakeup=YES; selfpowered=? */
  
                      priv->ep0resp[0] = (priv->selfpowered << USB_FEATURE_SELFPOWERED) |
                                         (1 << USB_FEATURE_REMOTEWAKEUP);
                      priv->ep0resp[1] = 0;

                      stm32_ep0xfer(STM32_EP0_IN, priv->ep0resp, 2);
                      stm32_ep0state(priv, EP0STATE_SHORTWRITE);
                    }
                  else
                    {
                      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADDEVGETSTATUS), 0);
                      priv->stalled = true;
                    }
                }
                break;
  
              case USB_REQ_RECIPIENT_INTERFACE:
                {
                  usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_IFGETSTATUS), 0);
                  priv->ep0resp[0] = 0;
                  priv->ep0resp[1] = 0;

                  stm32_ep0xfer(STM32_EP0_IN, priv->ep0resp, 2);
                  stm32_ep0state(priv, EP0STATE_SHORTWRITE);
                }
                break;
  
              default:
                {
                  usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADGETSTATUS), 0);
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
  
        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_CLEARFEATURE), 0);
        if ((ctrlreq->type & USB_REQ_RECIPIENT_MASK) != USB_REQ_RECIPIENT_ENDPOINT)
          {
            stm32_dispatchrequest(priv, &priv->ctrlreq);
          }
        else if (priv->paddrset != 0 && ctrlreq->value == USB_FEATURE_ENDPOINTHALT && ctrlreq->len == 0 &&
                (privep = stm32_epfindbyaddr(priv, ctrlreq->index)) != NULL)
          {
            stm32_epstall(&privep->ep, true);
            stm32_ep0state(priv, EP0STATE_NAK_IN);
          }
        else
          {
            usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADCLEARFEATURE), 0);
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
  
        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_SETFEATURE), 0);
        if (((ctrlreq->type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE) &&
            ctrlreq->value == USB_FEATURE_TESTMODE)
          {
            ullvdbg("test mode: %d\n", ctrlreq->index);
          }
        else if ((ctrlreq->type & USB_REQ_RECIPIENT_MASK) != USB_REQ_RECIPIENT_ENDPOINT)
          {
            stm32_dispatchrequest(priv, &priv->ctrlreq);
          }
        else if (priv->paddrset != 0 && ctrlreq->value == USB_FEATURE_ENDPOINTHALT && ctrlreq->len == 0 &&
                (privep = stm32_epfindbyaddr(priv, ctrlreq->index)) != NULL)
          {
            stm32_epstall(&privep->ep, false);
            stm32_ep0state(priv, EP0STATE_NAK_IN);
          }
        else
          {
            usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADSETFEATURE), 0);
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

        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EP0SETUPSETADDRESS), ctrlreq->value);
        if ((ctrlreq->type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE &&
            ctrlreq->index  == 0 && ctrlreq->len == 0 && ctrlreq->value < 128)
          {
            /* Save the address.  We cannot actually change to the next address until
             * the completion of the status phase.
             */
  
            priv->paddr = priv->ctrlreq->value[0];
            priv->paddrset = false;
            stm32_ep0state(priv, EP0STATE_NAK_IN);
          }
        else
          {
            usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADSETADDRESS), 0);
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
        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_GETSETDESC), 0);
        if ((ctrlreq->type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE)
          {
            stm32_dispatchrequest(priv, &priv->ctrlreq);
          }
        else
          {
            usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADGETSETDESC), 0);
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
        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_GETCONFIG), 0);
        if (priv->paddrset && (ctrlreq->type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE &&
            ctrlreq->value == 0 && ctrlreq->index == 0 && ctrlreq->len == 1)
          {
            stm32_dispatchrequest(priv, &priv->ctrlreq);
          }
        else
          {
            usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADGETCONFIG), 0);
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
        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_SETCONFIG), 0);
        if ((ctrlreq->type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE &&
            ctrlreq->index == 0 && ctrlreq->len == 0)
          {
            stm32_dispatchrequest(priv, &priv->ctrlreq);
          }
        else
          {
            usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADSETCONFIG), 0);
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
        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_GETSETIF), 0);
        stm32_dispatchrequest(priv, &priv->ctrlreq);
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
        usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDCTRLREQ), 0);
        priv->stalled = true;
      }
      break;
    }
}

/*******************************************************************************
 * Name: stm32_ep0setup
 *
 * Description:
 *   USB Ctrl EP Setup Event. This is logically part of the USB interrupt
 *   handler.  This event occurs when a setup packet is receive on EP0 OUT.
 *
 *******************************************************************************/

static inline void stm32_ep0setup(struct stm32_usbdev_s *priv)
{
  FAR struct stm32_ep_s *privep;
  struct stm32_ctrlreq_s ctrlreq;

  /* Terminate any pending requests - since all DTDs will have been retired 
   * because of the setup packet.
   */

  stm32_cancelrequests(&priv->epin[STM32_EP0_OUT], -EPROTO);
  stm32_cancelrequests(&priv->epin[STM32_EP0_IN],  -EPROTO);

  /* Assume NOT stalled */

  priv->epin[STM32_EP0_OUT].stalled = false;
  priv->epin[STM32_EP0_IN].stalled = false;
  priv->stalled = false;

  /* Read EP0 setup data */

  stm32_ep0read((FAR uint8_t*)&ctrlreq, USB_SIZEOF_CTRLREQ);

  /* Starting a control request - update state */

  stm32_ep0state(priv, (priv->ctrlreq.type & USB_REQ_DIR_IN) ? EP0STATE_SETUP_IN : EP0STATE_SETUP_OUT);

  /* And extract the little-endian 16-bit values to host order */

  ctrlreq.type  = priv->ctrlreq.type;
  ctrlreq.req   = priv->ctrlreq.req;
  ctrlreq.value = GETUINT16(priv->ctrlreq.value);
  ctrlreq.index = GETUINT16(priv->ctrlreq.index);
  ctrlreq.len   = GETUINT16(priv->ctrlreq.len);

  ullvdbg("type=%02x req=%02x value=%04x index=%04x len=%04x\n",
          ctrlreq.type, ctrlreq.req, ctrlreq.value, ctrlreq.index, ctrlreq.len);

  /* Check for a standard request */

  if ((ctrlreq.type & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_STANDARD)
    {
      /* Dispatch any non-standard requests */

      stm32_dispatchrequest(priv, &priv->ctrlreq);
    }
  else
    {
      /* Handle standard requests. */

      stm32_stdrequest(priv, &ctrlreq);
    }

  if (priv->stalled)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_EP0SETUPSTALLED), priv->ep0state);
      stm32_epstall(&priv->epin[STM32_EP0_IN].ep, false);
      stm32_epstall(&priv->epin[STM32_EP0_OUT].ep, false);
    }
}

/*******************************************************************************
 * Name: stm32_ep0complete
 *
 * Description:
 *   Transfer complete handler for Endpoint 0
 *
 *******************************************************************************/

static void stm32_ep0complete(struct stm32_usbdev_s *priv, uint8_t epphy)
{
  FAR struct stm32_ep_s *privep = &priv->epin[epphy];

  usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EP0COMPLETE), (uint16_t)priv->ep0state);
  
  switch (priv->ep0state)
    {
    case EP0STATE_DATA_IN:
      if (stm32_rqempty(privep))
        {
          return;
        }

      if (stm32_epcomplete(priv, epphy))
        {
          stm32_ep0state(priv, EP0STATE_NAK_OUT);
        }
      break;

    case EP0STATE_DATA_OUT:
      if (stm32_rqempty(privep))
        {
          return;
        }
    
      if (stm32_epcomplete(priv, epphy))
        {
          stm32_ep0state(priv, EP0STATE_NAK_IN);
        }
      break;
    
    case EP0STATE_SHORTWRITE:
      stm32_ep0state(priv, EP0STATE_NAK_OUT);
      break;
    
    case EP0STATE_STATUS_IN:
      stm32_ep0state(priv, EP0STATE_IDLE);

      /* If we've received a SETADDRESS packet, then we set the address
       * now that the status phase has completed.
       */

      if (! priv->paddrset && priv->paddr != 0)
        {
          usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EP0INSETADDRESS), (uint16_t)priv->paddr);
          stm32_set_address (priv, priv->paddr);
        }
      break;

    case EP0STATE_STATUS_OUT:
      stm32_ep0state(priv, EP0STATE_IDLE);
      break;

    default:
#ifdef CONFIG_DEBUG
      DEBUGASSERT(priv->ep0state != EP0STATE_DATA_IN &&
          priv->ep0state != EP0STATE_DATA_OUT        &&
          priv->ep0state != EP0STATE_SHORTWRITE      &&
          priv->ep0state != EP0STATE_STATUS_IN  &&
          priv->ep0state != EP0STATE_STATUS_OUT);
#endif
      priv->stalled = true;
      break;
    }

  if (priv->stalled)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_EP0SETUPSTALLED), priv->ep0state);
      stm32_epstall(&priv->epin[STM32_EP0_IN].ep, false);
      stm32_epstall(&priv->epin[STM32_EP0_OUT].ep, false);
    }
}

/*******************************************************************************
 * Name: stm32_ep0nak
 *
 * Description:
 *   Handle a NAK interrupt on EP0
 *
 *******************************************************************************/

static void stm32_ep0nak(struct stm32_usbdev_s *priv, uint8_t epphy)
{
  usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EP0NAK), (uint16_t)priv->ep0state);

  switch (priv->ep0state)
    {
    case EP0STATE_NAK_IN:
      stm32_ep0xfer(STM32_EP0_IN, NULL, 0);
      stm32_ep0state(priv, EP0STATE_STATUS_IN);
      break;
    case EP0STATE_NAK_OUT:
      stm32_ep0xfer(STM32_EP0_OUT, NULL, 0);
      stm32_ep0state(priv, EP0STATE_STATUS_OUT);
      break;
    default:
#ifdef CONFIG_DEBUG
      DEBUGASSERT(priv->ep0state != EP0STATE_NAK_IN &&
          priv->ep0state != EP0STATE_NAK_OUT);
#endif
      priv->stalled = true;
      break;
    }

  if (priv->stalled)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_EP0SETUPSTALLED), priv->ep0state);
      stm32_epstall(&priv->epin[STM32_EP0_IN].ep, false);
      stm32_epstall(&priv->epin[STM32_EP0_OUT].ep, false);
    }
}

/*******************************************************************************
 * Name: stm32_epcomplete
 *
 * Description:
 *   Transfer complete handler for Endpoints other than 0
 *   returns whether the request at the head has completed
 *
 *******************************************************************************/

bool stm32_epcomplete(struct stm32_usbdev_s *priv, uint8_t epphy)
{
  struct stm32_ep_s  *privep  = &priv->epin[epphy];
  struct stm32_req_s *privreq = privep->head;
  struct stm32_dtd_s *dtd     = &g_td[epphy];

  if (privreq == NULL) /* This shouldn't really happen */
  {
    if (STM32_EPPHYOUT(privep->epphy))
      {
        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPINQEMPTY), 0);
      }
    else
      {
        usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPOUTQEMPTY), 0);
      |
    return true;
  }
    
  int xfrd = dtd->xfer_len - (dtd->config >> 16);
    
  privreq->req.xfrd += xfrd;

  bool complete = true;
  if (STM32_EPPHYOUT(privep->epphy))
    {
      /* read(OUT) completes when request filled, or a short transfer is received */

      usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPIN), complete);
    }
  else
    {
      /* write(IN) completes when request finished, unless we need to terminate with a ZLP */

      bool need_zlp = (xfrd == privep->ep.maxpacket) && ((privreq->req.flags & USBDEV_REQFLAGS_NULLPKT) != 0);

      complete = (privreq->req.xfrd >= privreq->req.len && !need_zlp);

      usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPOUT), complete);
    }

  /* If the transfer is complete, then dequeue and progress any further queued requests */

  if (complete)
    {
      privreq = stm32_rqdequeue (privep);
    }
    
  if (!stm32_rqempty(privep))
    {
      stm32_progressep(privep);
    }

  /* Now it's safe to call the completion callback as it may well submit a new request */

  if (complete)
    {
      usbtrace(TRACE_COMPLETE(privep->epphy), privreq->req.xfrd);
      stm32_reqcomplete(privep, privreq, OK);
    }

  return complete;
}

/*******************************************************************************
 * Name: stm32_epininterrupt
 *
 * Description:
 *   USB IN endpoint interrupt handler
 *
 *******************************************************************************/

static inline void stm32_epininterrupt(FAR struct stm32_usbdev_s *priv)
{
  uint32_t diepint;
  uint32_t daint;
  uint32_t mask;
  uint32_t empty;
  int epno;

  /* Get the pending, enabled interrupts for the IN endpoint from the endpoint
   * interrupt status register.
   */

  daint  = stm32_getreg(STM32_OTGFS_DAINT);
  daint &= stm32_getreg(STM32_OTGFS_DAINTMSK);
  daint &= OTGFS_DAINT_IEP_MASK;

  /* Process each pending IN endpoint interrupt */

  epno = 0;
  while (daint)
    {
      /* Is an IN interrupt pending for this endpoint? */
  
      if ((daint & 1) != 0)
        {
          /* Get IN interrupt mask register.  Bits 0-6 correspond to enabled
           * interrupts as will be found in the DIEPINT interrupt status
           * register.
           */

          mask = stm32_getreg(STM32_OTGFS_DIEPMSK);

          /* Check for FIFO not empty.  Bits n corresponds to endpoint n.
           * That condition corresponds to bit 7 of the DIEPINT interrupt
           * status register.
           */

          empty = stm32_getreg(STM32_OTGFS_DIEPEMPMSK);
          if ((empty & OTGFS_DIEPEMPMSK(epno)) != ))
            {
              mask |= OTGFS_DIEPINT_TXFE;
            }

          /* Now, read the interrupt status and mask out all disabled
           * interrupts.
           */

          diepint = stm32_getreg(STM32_OTGFS_DIEPINT(epno)) & mask;

          /* Decode and process the enabled, pending interrupts */
          /* Transfer completed interrupt */

          if ((diepint & OTGFS_DIEPINT_XFRC) != 0)
            {
              usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPIN_XFRC), (uint16_t)diepint);

              empty &= ~OTGFS_DIEPEMPMSK(epno);
              stm32_putreg(empty, STM32_OTGFS_DIEPEMPMSK);
              stm32_putreg(OTGFS_DIEPINT_XFRC, STM32_OTGFS_DIEPINT(epno));

              /* TX complete */

              stm32_txcomplete(priv, epno);
            }

          /* Timeout condition */

          if ((diepint & OTGFS_DIEPINT_TOC) != 0)
            {
              usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPIN_TOC), (uint16_t)diepint);
              stm32_putreg(OTGFS_DIEPINT_TOC, STM32_OTGFS_DIEPINT(epno));
            }

          /* IN token received when TxFIFO is empty */

          if ((diepint & OTGFS_DIEPINT_ITTXFE) != 0)
            {
              usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPIN_ITTXFE), (uint16_t)diepint);
              stm32_putreg(OTGFS_DIEPINT_ITTXFE, STM32_OTGFS_DIEPINT(epno));
            }

          /* IN endpoint NAK effective */

          if ((diepint & OTGFS_DIEPINT_INEPNE) != 0)
            {
              usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPIN_INEPNE), (uint16_t)diepint);
              stm32_putreg(OTGFS_DIEPINT_INEPNE, STM32_OTGFS_DIEPINT(epno));
            }

          /* Endpoint disabled interrupt */

          if ((diepint & OTGFS_DIEPINT_EPDISD) != 0)
            {
              usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPIN_EPDISD), (uint16_t)diepint);
              stm32_putreg(OTGFS_DIEPINT_EPDISD, STM32_OTGFS_DIEPINT(epno));
            }

          /* Transmit FIFO empty */

          if ((diepint & OTGFS_DIEPINT_TXFE) != 0)
            {
              usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPIN_TXFE), (uint16_t)diepint);
              stm32_txfifoempty(priv, epno);
              stm32_putreg(OTGFS_DIEPINT_TXFE, STM32_OTGFS_DIEPINT(epno));
            }
        }

      epno++;
      daint >>= 1;
    }

  return 1;
}

/*******************************************************************************
 * Name: stm32_epoutinterrupt
 *
 * Description:
 *   USB OUT endpoint interrupt handler
 *
 *******************************************************************************/

static inline void stm32_epoutinterrupt(FAR struct stm32_usbdev_s *priv)
{
  uint32_t daint;
  uint32_t regval;
  uint32_t doepint;
  int epno;

  /* Get the pending, enabled interrupts for the OUT endpoint from the endpoint
   * interrupt status register.
   */

  regval  = stm32_getreg(STM32_OTGFS_DAINT);
  regval &= stm32_getreg(STM32_OTGFS_DAINTMSK);
  daint   = (regval & OTGFS_DAINT_OEP_MASK) >> OTGFS_DAINT_OEP_SHIFT;

  /* Process each pending IN endpoint interrupt */

  epno = 0;
  while (daint)
    {
      /* Is an OUT interrupt pending for this endpoint? */
  
      if ((daint & 1) != 0)
        {
          /* Yes.. get the OUT endpoint interrupt status */

          doepint  = stm32_getreg(STM32_OTGFS_DOEPINT(epno));
          doepint &= stm32_getreg(STM32_OTGFS_DOEPMSK);

          /* Transfer completed interrupt */

          if ((doepint & OTGFS_DOEPINT_XFRC) != 0)
            {
              usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPOUT_XFRC), (uint16_t)diepint);

              /* Clear the bit in DOEPINTn for this interrupt */

              stm32_putreg(OTGFS_DOEPINT_XFRC, STM32_OTGFS_DOEPINT(epno));

              /* Handle the RX transer data ready event */

              stm32_epout(FAR struct stm32_usbdev_s *priv, uint8_t epno)
            }

          /* Endpoint disabled interrupt */

          if ((doepint & OTGFS_DOEPINT_EPDISD) != 0)
            {
              usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPOUT_EPDISD), (uint16_t)diepint);

              /* Clear the bit in DOEPINTn for this interrupt */

              stm32_putreg(OTGFS_DOEPINT_EPDISD, STM32_OTGFS_DOEPINT(epno));
            }

          /* Setup Phase Done (control EPs) */

          if ((doepint & OTGFS_DOEPINT_SETUP) != 0)
            {
              usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPOUT_SETUP), (uint16_t)diepint);

              /* Handle the receipt of the SETUP packet */

              stm32_ep0setup(priv);
              stm32_putreg(OTGFS_DOEPINT_SETUP, STM32_OTGFS_DOEPINT(epno));
            }
        }

      epno++;
      daint >>= 1;
    }

  return 1;
}

/*******************************************************************************
 * Name: stm32_usbinterrupt
 *
 * Description:
 *   USB interrupt handler
 *
 *******************************************************************************/

static int stm32_usbinterrupt(int irq, FAR void *context)
{
  /* At present, there is only a single OTG FS device support. Hence it is
   * pre-allocated as g_otgfsdev.  However, in most code, the private data
   * structure will be referenced using the 'priv' pointer (rather than the
   * global data) in order to simplify any future support for multiple devices.
   */

  FAR struct stm32_usbdev_s *priv = &g_otgfsdev;
  uint32_t regval;

  /* Assure that we are in device mode */

  DEBUGASSERT((stm32_getreg(STM32_OTGFS_GINTSTS) & OTGFS_GINTSTS_CMOD) == OTGFS_GINTSTS_DEVMODE);

  /* Get the state of all enabled interrupts */

  regval  = stm32_getreg(STM32_OTGFS_GINTSTS);
  regval &= stm32_getreg(STM32_OTGFS_GINTMSK);
  usbtrace(TRACE_INTENTRY(STM32_TRACEINTID_USB), (uint16_t)regval);

  /* OUT endpoint interrupt */

  if ((regval & OTGFS_GINT_OEP) != 0)
    {
      usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPOUT), (uint16_t)regval);
      (void)stm32_epoutinterrupt(priv);
    }

  /* IN endpoint interrupt */

  if ((regval & OTGFS_GINT_IEP) != 0)
    {
      usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_EPIN), (uint16_t)regval);
      stm32_epininterrupt(priv);
    }

  /* Mode mismatch interrupt */

#ifdef CONFIG_DEBUG_USB
  if ((regval & OTGFS_GINT_MMIS) != 0)
    {
      usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_MISMATCH), (uint16_t)regval);
      stm32_putreg(OTGFS_GINT_MMIS, STM32_OTGFS_GINTSTS);
    }
#endif

  /* Resume/remote wakeup detected interrupt */

  if ((regval & OTGFS_GINT_WKUP) != 0)
    {
      usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_WAKEUP), (uint16_t)regval);
      (void)stm32_resumeinterrupt(priv);
    }

  /* USB suspend interrupt */

  if ((regval & OTGFS_GINT_USBSUSP) != 0)
    {
      usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_SUSPEND), (uint16_t)regval);
      (void)stm32_suspendinterrupt(priv);
    }

  /* Start of frame interrupt */

#ifdef CONFIG_USBDEV_SOFINTERRUPT
  if ((regval & OTGFS_GINT_SOF) != 0)
    {
      usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_SOF), (uint16_t)regval);
      stm32_putreg(OTGFS_GINT_SOF, STM32_OTGFS_GINTSTS);
    }
#endif

  /* RxFIFO non-empty interrupt */

  if ((regval & OTGFS_GINT_RXFLVL) != 0)
    {
      usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_RXFIFO), (uint16_t)regval);
      (void)stm32_rxinterrupt(priv);
    }

  /* USB reset interrupt */

  if ((regval & OTGFS_GINT_USBRST) != 0)
    {
      usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_DEVRESET), (uint16_t)regval);
      stm32_usbreset(priv);
      usbtrace(TRACE_INTEXIT(STM32_TRACEINTID_USB), 0);
      return OK;
    }

  /* Enumeration done interrupt */

  if ((regval & OTGFS_GINT_ENUMDNE) != 0)
    {
      usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_ENUMDNE), (uint16_t)regval);
      (void)stm32_enuminterrupt(priv);
    }

  /* Incomplete isochronous IN transfer interrupt */

  if ((regval & OTGFS_GINT_IISOIXFR) != 0)
    {
      usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_IISOIXFR), (uint16_t)regval);
      (void)stm32_isocininterrupt(priv);
    }

  /* Incomplete periodic transfer interrupt*/

  if ((regval & OTGFS_GINT_IPXFR) != 0)
    {
      usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_IPXFR), (uint16_t)regval);
      (void)stm32_isocoutinterrupt(priv);
    }

  /* Session request/new session detected interrupt */

#ifdef CONFIG_USBDEV_VBUSSENSING
  if ((regval & OTGFS_GINT_SRQ) != 0)
    {
      usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_SRQ), (uint16_t)regval);
      (void)stm32_sessioninterrupt(priv);
    }

  /* OTG interrupt */

  if ((regval & OTGFS_GINT_OTG) != 0)
    {
      usbtrace(TRACE_INTDECODE(STM32_TRACEINTID_OTG), (uint16_t)regval);
      (void)stm32_otginterrupt(priv);
    }
#endif

  usbtrace(TRACE_INTEXIT(STM32_TRACEINTID_USB), 0);
  return OK;
}

/*******************************************************************************
 * Endpoint operations
 *******************************************************************************/

/*******************************************************************************
 * Name: stm32_epconfigure
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

static int stm32_epconfigure(FAR struct usbdev_ep_s *ep,
                               FAR const struct usb_epdesc_s *desc,
                               bool last)
{
  FAR struct stm32_ep_s *privep = (FAR struct stm32_ep_s *)ep;

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

  if (STM32_EPPHYIN(privep->epphy))
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
      stm32_chgbits (0xFFFF0000, cfg, STM32_USBDEV_ENDPTCTRL(privep->epphy));
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
      stm32_chgbits (0x0000FFFF, cfg, STM32_USBDEV_ENDPTCTRL(privep->epphy));
    }

  /* Reset endpoint status */

  privep->stalled = false;

  /* Enable the endpoint */

  if (STM32_EPPHYIN(privep->epphy))
    {
      stm32_setbits(USBDEV_ENDPTCTRL_TXE, STM32_USBDEV_ENDPTCTRL(privep->epphy));
    }
  else
    {
      stm32_setbits(USBDEV_ENDPTCTRL_RXE, STM32_USBDEV_ENDPTCTRL(privep->epphy));
    }
  
   return OK;
}

/*******************************************************************************
 * Name: stm32_epdisable
 *
 * Description:
 *   The endpoint will no longer be used
 *
 *******************************************************************************/

static int stm32_epdisable(FAR struct usbdev_ep_s *ep)
{
  FAR struct stm32_ep_s *privep = (FAR struct stm32_ep_s *)ep;
  irqstate_t flags;

#ifdef CONFIG_DEBUG
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif
  usbtrace(TRACE_EPDISABLE, privep->epphy);

  flags = irqsave();

  /* Disable Endpoint */

  if (STM32_EPPHYIN(privep->epphy))
    {
      stm32_clrbits (USBDEV_ENDPTCTRL_TXE, STM32_USBDEV_ENDPTCTRL(privep->epphy));
    }
  else
    {
      stm32_clrbits (USBDEV_ENDPTCTRL_RXE, STM32_USBDEV_ENDPTCTRL(privep->epphy));
    }

  privep->stalled = true;

  /* Cancel any ongoing activity */

  stm32_cancelrequests(privep, -ESHUTDOWN);

  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Name: stm32_epallocreq
 *
 * Description:
 *   Allocate an I/O request
 *
 *******************************************************************************/

static FAR struct usbdev_req_s *stm32_epallocreq(FAR struct usbdev_ep_s *ep)
{
  FAR struct stm32_req_s *privreq;

#ifdef CONFIG_DEBUG
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), 0);
      return NULL;
    }
#endif
  usbtrace(TRACE_EPALLOCREQ, ((FAR struct stm32_ep_s *)ep)->epphy);

  privreq = (FAR struct stm32_req_s *)malloc(sizeof(struct stm32_req_s));
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_ALLOCFAIL), 0);
      return NULL;
    }

  memset(privreq, 0, sizeof(struct stm32_req_s));
  return &privreq->req;
}

/*******************************************************************************
 * Name: stm32_epfreereq
 *
 * Description:
 *   Free an I/O request
 *
 *******************************************************************************/

static void stm32_epfreereq(FAR struct usbdev_ep_s *ep, FAR struct usbdev_req_s *req)
{
  FAR struct stm32_req_s *privreq = (FAR struct stm32_req_s *)req;

#ifdef CONFIG_DEBUG
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), 0);
      return;
    }
#endif

  usbtrace(TRACE_EPFREEREQ, ((FAR struct stm32_ep_s *)ep)->epphy);
  free(privreq);
}

/*******************************************************************************
 * Name: stm32_epallocbuffer
 *
 * Description:
 *   Allocate an I/O buffer
 *
 *******************************************************************************/

#ifdef CONFIG_ARCH_USBDEV_DMA
static void *stm32_epallocbuffer(FAR struct usbdev_ep_s *ep, unsigned bytes)
{
  usbtrace(TRACE_EPALLOCBUFFER, privep->epphy);
  return malloc(bytes)
}
#endif

/*******************************************************************************
 * Name: stm32_epfreebuffer
 *
 * Description:
 *   Free an I/O buffer
 *
 *******************************************************************************/

#ifdef CONFIG_LPC313x_USBDEV_DMA
static void stm32_epfreebuffer(FAR struct usbdev_ep_s *ep, FAR void *buf)
{
  usbtrace(TRACE_EPFREEBUFFER, privep->epphy);
  free(buf);
}
#endif

/*******************************************************************************
 * Name: stm32_epsubmit
 *
 * Description:
 *   Submit an I/O request to the endpoint
 *
 *******************************************************************************/

static int stm32_epsubmit(FAR struct usbdev_ep_s *ep, FAR struct usbdev_req_s *req)
{
  FAR struct stm32_req_s *privreq = (FAR struct stm32_req_s *)req;
  FAR struct stm32_ep_s *privep = (FAR struct stm32_ep_s *)ep;
  FAR struct stm32_usbdev_s *priv;
  irqstate_t flags;
  int ret = OK;

#ifdef CONFIG_DEBUG
  if (!req || !req->callback || !req->buf || !ep)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), 0);
      ullvdbg("req=%p callback=%p buf=%p ep=%p\n", req, req->callback, req->buf, ep);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPSUBMIT, privep->epphy);
  priv = privep->dev;

  if (!priv->driver || priv->usbdev.speed == USB_SPEED_UNKNOWN)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_NOTCONFIGURED), priv->usbdev.speed);
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

      if (STM32_EPPHYIN(privep->epphy))
        {
          usbtrace(TRACE_INREQQUEUED(privep->epphy), privreq->req.len);
        }
      else
        {
          usbtrace(TRACE_OUTREQQUEUED(privep->epphy), privreq->req.len);
        }

      if (stm32_rqenqueue(privep, privreq))
        {
          stm32_progressep(privep);
        }
    }

  irqrestore(flags);
  return ret;
}

/*******************************************************************************
 * Name: stm32_epcancel
 *
 * Description:
 *   Cancel an I/O request previously sent to an endpoint
 *
 *******************************************************************************/

static int stm32_epcancel(FAR struct usbdev_ep_s *ep, FAR struct usbdev_req_s *req)
{
  FAR struct stm32_ep_s *privep = (FAR struct stm32_ep_s *)ep;
  FAR struct stm32_usbdev_s *priv;
  irqstate_t flags;

#ifdef CONFIG_DEBUG
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), 0);
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

  stm32_cancelrequests(privep, -ESHUTDOWN);
  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Name: stm32_epstall
 *
 * Description:
 *   Stall or resume and endpoint
 *
 *******************************************************************************/

static int stm32_epstall(FAR struct usbdev_ep_s *ep, bool resume)
{
  FAR struct stm32_ep_s *privep = (FAR struct stm32_ep_s *)ep;
  irqstate_t flags;

  /* STALL or RESUME the endpoint */

  flags = irqsave();
  usbtrace(resume ? TRACE_EPRESUME : TRACE_EPSTALL, privep->epphy);

  uint32_t addr    = STM32_USBDEV_ENDPTCTRL(privep->epphy);
  uint32_t ctrl_xs = STM32_EPPHYIN(privep->epphy) ? USBDEV_ENDPTCTRL_TXS : USBDEV_ENDPTCTRL_RXS;
  uint32_t ctrl_xr = STM32_EPPHYIN(privep->epphy) ? USBDEV_ENDPTCTRL_TXR : USBDEV_ENDPTCTRL_RXR;

  if (resume)
    {
      privep->stalled = false;

      /* Clear stall and reset the data toggle */

      stm32_chgbits (ctrl_xs | ctrl_xr, ctrl_xr, addr);
    }
  else
    {
      privep->stalled = true;

      stm32_setbits (ctrl_xs, addr);
    }

  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Device operations
 *******************************************************************************/

/*******************************************************************************
 * Name: stm32_allocep
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

static FAR struct usbdev_ep_s *stm32_allocep(FAR struct usbdev_s *dev, uint8_t eplog,
                                               bool in, uint8_t eptype)
{
  FAR struct stm32_usbdev_s *priv = (FAR struct stm32_usbdev_s *)dev;
  uint32_t epset = STM32_EPALLSET & ~STM32_EPCTRLSET;
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

      if (eplog >= STM32_NENDPOINTS)
        {
          usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADEPNO), (uint16_t)eplog);
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
      epset &= STM32_EPINSET;
    }
  else
    {
      epset &= STM32_EPOUTSET;
    }

  /* Get the subset matching the requested type */

  switch (eptype)
    {
    case USB_EP_ATTR_XFER_INT: /* Interrupt endpoint */
      epset &= STM32_EPINTRSET;
      break;

    case USB_EP_ATTR_XFER_BULK: /* Bulk endpoint */
      epset &= STM32_EPBULKSET;
      break;

    case USB_EP_ATTR_XFER_ISOC: /* Isochronous endpoint */
      epset &= STM32_EPISOCSET;
      break;

    case USB_EP_ATTR_XFER_CONTROL: /* Control endpoint -- not a valid choice */
    default:
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BADEPTYPE), (uint16_t)eptype);
      return NULL;
    }

  /* Is the resulting endpoint supported by the STM32? */

  if (epset)
    {
      /* Yes.. now see if any of the request endpoints are available */

      flags = irqsave();
      epset &= priv->epavail;
      if (epset)
        {
          /* Select the lowest bit in the set of matching, available endpoints */

          for (epndx = 2; epndx < STM32_NENDPOINTS; epndx++)
            {
              uint32_t bit = 1 << epndx;
              if ((epset & bit) != 0)
                {
                  /* Mark the IN/OUT endpoint no longer available */

                  priv->epavail &= ~(3 << (bit & ~1));
                  irqrestore(flags);

                  /* And return the pointer to the standard endpoint structure */

                  return &priv->epin[epndx].ep;
                }
            }
          /* Shouldn't get here */
        }
      irqrestore(flags);
    }

  usbtrace(TRACE_DEVERROR(STM32_TRACEERR_NOEP), (uint16_t)eplog);
  return NULL;
}

/*******************************************************************************
 * Name: stm32_freeep
 *
 * Description:
 *   Free the previously allocated endpoint
 *
 *******************************************************************************/

static void stm32_freeep(FAR struct usbdev_s *dev, FAR struct usbdev_ep_s *ep)
{
  FAR struct stm32_usbdev_s *priv = (FAR struct stm32_usbdev_s *)dev;
  FAR struct stm32_ep_s *privep = (FAR struct stm32_ep_s *)ep;
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
 * Name: stm32_getframe
 *
 * Description:
 *   Returns the current frame number
 *
 *******************************************************************************/

static int stm32_getframe(struct usbdev_s *dev)
{
#ifdef CONFIG_STM32_USBDEV_FRAME_INTERRUPT
  FAR struct stm32_usbdev_s *priv = (FAR struct stm32_usbdev_s *)dev;

  /* Return last valid value of SOF read by the interrupt handler */

  usbtrace(TRACE_DEVGETFRAME, (uint16_t)priv->sof);
  return priv->sof;
#else
  /* Return the last frame number detected by the hardware */

  usbtrace(TRACE_DEVGETFRAME, 0);

  /* FIXME: this actually returns the micro frame number! */
  return (int)stm32_getreg(STM32_USBDEV_FRINDEX_OFFSET);
#endif
}

/*******************************************************************************
 * Name: stm32_wakeup
 *
 * Description:
 *   Tries to wake up the host connected to this device
 *
 *******************************************************************************/

static int stm32_wakeup(struct usbdev_s *dev)
{
  irqstate_t flags;

  usbtrace(TRACE_DEVWAKEUP, 0);

  flags = irqsave();
  stm32_setbits(USBDEV_PRTSC1_FPR, STM32_USBDEV_PORTSC1);
  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Name: stm32_selfpowered
 *
 * Description:
 *   Sets/clears the device selfpowered feature 
 *
 *******************************************************************************/

static int stm32_selfpowered(struct usbdev_s *dev, bool selfpowered)
{
  FAR struct stm32_usbdev_s *priv = (FAR struct stm32_usbdev_s *)dev;

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

/*******************************************************************************
 * Name: stm32_pullup
 *
 * Description:
 *   Software-controlled connect to/disconnect from USB host
 *
 *******************************************************************************/

static int stm32_pullup(struct usbdev_s *dev, bool enable)
{
  usbtrace(TRACE_DEVPULLUP, (uint16_t)enable);

  irqstate_t flags = irqsave();
  if (enable)
    stm32_setbits (USBDEV_USBCMD_RS, STM32_USBDEV_USBCMD);
  else
    stm32_clrbits (USBDEV_USBCMD_RS, STM32_USBDEV_USBCMD);
  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Name: stm32_swinitialize
 *
 * Description:
 *   Flush the specific TX fifo.
 *
 *******************************************************************************/

static int stm32_flushtxfifo(FAR struct stm32_usbdev_s *priv, uint32_t txfnum)
{
  uint32_t regval;
  uint32_t timeout;

  /* Initiate the TX FIFO flush operation */

  regval = OTGFS_GRSTCTL_TXFFLSH | txfnum;
  stm32_putreg(regval, STM32_OTGFS_GRSTCTL);

  /* Wait for the FLUSH to complete */

  for (timeout = 0; timeout < 200000; timeout++)
    {
      regval = stm32_getreg(STM32_OTGFS_GRSTCTL);
      if ((regval & OTGFS_GRSTCTL_TXFFLSH) == 0)
        {
          break;
        }
    }

  /* Wait for 3 PHY Clocks */

  up_udelay(3);
  return OK;
}

/*******************************************************************************
 * Name: stm32_swinitialize
 *
 * Description:
 *   Flush the RX fifo.
 *
 *******************************************************************************/

static int stm32_flushrxfifo(FAR struct stm32_usbdev_s *priv)
{
  uint32_t regval;
  uint32_t timeout;

  /* Initiate the RX FIFO flush operation */

  stm32_putreg(OTGFS_GRSTCTL_RXFFLSH, STM32_OTGFS_GRSTCTL);

  /* Wait for the FLUSH to complete */

  for (timeout = 0; timeout < 200000; timeout++)
    {
      regval = stm32_getreg(STM32_OTGFS_GRSTCTL);
      if ((regval & OTGFS_GRSTCTL_RXFFLSH) == 0)
        {
          break;
        }
    }

  /* Wait for 3 PHY Clocks */

  up_udelay(3);
  return OK;
}

/*******************************************************************************
 * Name: stm32_swinitialize
 *
 * Description:
 *   Initialize all driver data structures.
 *
 *******************************************************************************/

static void stm32_swinitialize(FAR struct stm32_usbdev_s *priv)
{
  FAR struct stm32_ep_s *privep;

  /* Initialize the device state structure */

  memset(priv, 0, sizeof(struct stm32_usbdev_s));
  priv->usbdev.ops = &g_devops;
  priv->usbdev.ep0 = &priv->epin[STM32_EP0_IN].ep;
  priv->epavail    = STM32_EPALLSET;

  /* Initialize the endpoint lists */

  for (i = 0; i < STM32_NENDPOINTS; i++)
    {
      uint32_t bit = 1 << i;

      /* Set endpoint operations, reference to driver structure (not
       * really necessary because there is only one controller), and
       * the physical endpoint number (which is just the index to the
       * endpoint).
       */

      privep           = &priv->epin[i];
      privep->ep.ops   = &g_epops;
      privep->dev      = priv;
      privep->isin     = 1;

      /* The index, i, is the physical endpoint address;  Map this
       * to a logical endpoint address usable by the class driver.
       */

      privep->epphy    = i;
      privep->ep.eplog = STM32_EPPHYIN2LOG(i);

      /* Control until endpoint is activated */

      privep->eptype       = USB_EP_ATTR_XFER_CONTROL;
      privep->ep.maxpacket = CONFIG_USBDEV_EP0_MAXSIZE;
    }

  /* Initialize the endpoint lists */

  for (i = 0; i < STM32_NENDPOINTS; i++)
    {
      uint32_t bit = 1 << i;

      /* Set endpoint operations, reference to driver structure (not
       * really necessary because there is only one controller), and
       * the physical endpoint number (which is just the index to the
       * endpoint).
       */

      privep           = &priv->epout[i];
      privep->ep.ops   = &g_epops;
      privep->dev      = priv;

      /* The index, i, is the physical endpoint address;  Map this
       * to a logical endpoint address usable by the class driver.
       */

      privep->epphy    = i;
      privep->ep.eplog = STM32_EPPHYOUT2LOG(i);

      /* Control until endpoint is activated */

      privep->eptype       = USB_EP_ATTR_XFER_CONTROL;
      privep->ep.maxpacket = CONFIG_USBDEV_EP0_MAXSIZE;
    }
}

/*******************************************************************************
 * Name: stm32_hwinitialize
 *
 * Description:
 *   Configure the OTG FS core for operation.
 *
 *******************************************************************************/

static void stm32_hwinitialize(FAR struct stm32_usbdev_s *priv)
{
  uint32_t regval;
  uint32_t timeout;
  uint32_t address;
  int i;

  /* At startup the core is in FS mode.  

  /* Disable the USB global interrupt by clearing GINTMSK in the global OTG
   * FS AHB configuration register.
   */

  stm32_putreg(0, STM32_OTGFS_GAHBCFG);

  /* Common USB OTG core initialization */
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

  stm32_putreg(OTGFS_GRSTCTL_CSRST, STM32_OTGFS_GRSTCTL);
  for (timeout = 0; timeout < STM32_READY_DELAY; timeout++)
    {
      regval = stm32_getreg(STM32_OTGFS_GRSTCTL);
      if ((reval & OTGFS_GRSTCTL_CSRST) == 0)
        {
          break;
        }
    }

  /* Wait for 3 PHY Clocks */

  up_udelay(3);

  /* Deactivate the power down */

  regval  = OTGFS_GCCFG_PWRDWN | OTGFS_GCCFG_VBUSASEN | OTGFS_GCCFG_VBUSBSEN
#ifndef CONFIG_USBDEV_VBUSSENSING
  regval |= OTGFS_GCCFG_NOVBUSSENS;
#endif
#ifdef CONFIG_USBDEV_SOFOUTPUT
  regval |= OTGFS_GCCFG_SOFOUTEN;
#endif
  stm32_putreg(regval, STM32_OTGFS_GCCFG);
  up_mdelay(20);

  /* Force Device Mode */

  regval  = stm32_getreg(STM32_OTGFS_GUSBCFG);
  regval &= ~OTGFS_GUSBCFG_FHMOD;
  regval |= OTGFS_GUSBCFG_FDMOD;
  stm32_putreg(regval, STM32_OTGFS_GUSBCFG);
  up_mdelay(50);

  /* Initialize device mode */
  /* Restart the Phy Clock */

  stm32_putreg(0, STM32_OTGFS_PCGCCTL);

  /* Device configuration register */

  regval = stm32_getreg(STM32_OTGFS_DCFG);
  regval &= ~OTGFS_DCFG_PFIVL_MASK
  regval |= OTGFS_DCFG_PFIVL_80PCT;
  stm32_putreg(regval, STM32_OTGFS_DCFG);

  /* Set full speed phy */

  regval = stm32_getreg(STM32_OTGFS_DCFG);
  regval &= ~OTGFS_DCFG_DSPD_MASK
  regval |= OTGFS_DCFG_DSPD_FS;
  stm32_putreg(regval, STM32_OTGFS_DCFG);

  /* set Rx FIFO size */

  stm32_putreg(CONFIG_USBDEV_RXFIFO_SIZE, STM32_OTGFS_GRXFSIZ);

  /* EP0 TX */

  address = CONFIG_USBDEV_RXFIFO_SIZE
  regval = (address << OTGFS_DIEPTXF0_TX0FD_SHIFT) ||
           (CONFIG_USBDEV_EP0_TXFIFO_SIZE << OTGFS_DIEPTXF0_TX0FSA_SHIFT);
  stm32_putreg(regval, STM32_OTGFS_DIEPTXF0);

  /* EP1 TX */

  address += CONFIG_USBDEV_EP0_TXFIFO_SIZE;
  regval = (address << OTGFS_DIEPTXF_INEPTXSA_SHIFT) ||
           (CONFIG_USBDEV_EP1_TXFIFO_SIZE << OTGFS_DIEPTXF_INEPTXFD_SHIFT);
  stm32_putreg(regval, STM32_OTGFS_DIEPTXF);

  /* EP2 TX */

  address += CONFIG_USBDEV_EP1_TXFIFO_SIZE;
  regval = (address << OTGFS_DIEPTXF_INEPTXSA_SHIFT) ||
           (CONFIG_USBDEV_EP2_TXFIFO_SIZE << OTGFS_DIEPTXF_INEPTXFD_SHIFT);
  stm32_putreg(regval, STM32_OTGFS_DIEPTXF2);

  /* EP3 TX */

  address += CONFIG_USBDEV_EP2_TXFIFO_SIZE;
  regval = (address << OTGFS_DIEPTXF_INEPTXSA_SHIFT) ||
           (CONFIG_USBDEV_EP3_TXFIFO_SIZE << OTGFS_DIEPTXF_INEPTXFD_SHIFT);
  stm32_putreg(regval, STM32_OTGFS_DIEPTXF3);

  /* Flush the FIFOs */

  stm32_flushtxfifo(priv, OTGFS_GRSTCTL_TXFNUM_DALL);
  stm32_flushrxfifo(priv);

  /* Clear all pending Device Interrupts */

  stm32_putreg(0, STM32_OTGFS_DIEPMSK);
  stm32_putreg(0, STM32_OTGFS_DOEPMSK);
  stm32_putreg(0xffffffff, STM32_OTGFS_DAINT);
  stm32_putreg(0, STM32_OTGFS_DAINTMSK);

  /* Configure all IN endpoints */

  for (i = 0; i < STM32_NENDPOINTS; i++)
    {
      regval = stm32_getreg(STM32_OTGFS_DIEPCTL(i));
      if ((regval & OTGFS_DIEPCTL_EPENA) != 0)
        {
          /* The endpoint is already enabled */

          regval = OTGFS_DIEPCTL_EPENA | OTGFS_DIEPCTL_SNAK;
        }
      else
        {
          regval = 0;
        }

      stm32_putreg(regval, STM32_OTGFS_DIEPCTL(i));
      stm32_putreg(0, STM32_OTGFS_DIEPTSIZ(i));
      stm32_putreg(0xff, STM32_OTGFS_DIEPINT(i));
    }

  /* Configure all OUT endpoints */

  for (i = 0; i < STM32_NENDPOINTS; i++)
    {
      regval = stm32_getreg(STM32_OTGFS_DOEPCTL(i));
      if ((regval & OTGFS_DOEPCTL_EPENA) != 0)
        {
          /* The endpoint is already enabled */

          regval = OTGFS_DOEPCTL_EPENA | OTGFS_DOEPCTL_SNAK;
        }
      else
        {
          regval = 0;
        }

      stm32_putreg(regval, STM32_OTGFS_DOEPCTL(i));
      stm32_putreg(0, STM32_OTGFS_DOEPTSIZ(i));
      stm32_putreg(0xff, STM32_OTGFS_DOEPINT(i));
    }

  /* Disable all interrupts. */

  stm32_putreg(0, STM32_OTGFS_GINTMSK);

  /* Clear any pending interrupts */

  stm32_putreg(0xbfffffff, STM32_OTGFS_GINTSTS);

  /* Enable the common interrupts */
  /* Clear any pending USB_OTG Interrupts */

  stm32_putreg(0xffffffff, STM32_OTGFS_GOTGINT);

  /* Clear any pending interrupts */

  stm32_putreg(0xbfffffff, STM32_OTGFS_GINTSTS);

  /* Enable the interrupts in the INTMSK */

  regval = (OTGFS_GINT_RXFLVL | OTGFS_GINT_USBSUSP | OTGFS_GINT_ENUMDNE |
            OTGFS_GINT_IEP | OTGFS_GINT_OEP | OTGFS_GINT_IISOIXFR |
            OTGFS_GINT_IPXFR | regval);

#ifdef CONFIG_USBDEV_SOFINTERRUPT
  regval |= OTGFS_GINT_SOF;
#endif

#ifdef CONFIG_USBDEV_VBUSSENSING
  regval |= (OTGFS_GINT_OTG | OTGFS_GINT_SRQ);
#endif

#ifdef CONFIG_DEBUG_USB
  regval |= OTGFS_GINT_MMIS;
#endif

  stm32_putreg(regval, STM32_OTGFS_GINTMSK);

  /* Ensable the USB global interrupt by setting GINTMSK in the global OTG
   * FS AHB configuration register.
   */

  stm32_putreg(OTGFS_GAHBCFG_GINTMSK, STM32_OTGFS_GAHBCFG);
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
  /* At present, there is only a single OTG FS device support. Hence it is
   * pre-allocated as g_otgfsdev.  However, in most code, the private data
   * structure will be referenced using the 'priv' pointer (rather than the
   * global data) in order to simplify any future support for multiple devices.
   */

  FAR struct stm32_usbdev_s *priv = &g_otgfsdev;
  int i;

  usbtrace(TRACE_DEVINIT, 0);

  /* Here we assume that:
   *
   * 1. GPIOA and OTG FS peripheral clocking has already been enabled as part
   *    of the boot sequence.
   * 2. Board-specific logic has already enabled other board specific GPIOs
   *    for things like soft pull-up, VBUS sensing, power controls, and over-
   *    current detection.
   */

  /* Configure OTG FS alternate function pins */

  stm32_configgpio(GPIO_OTGFS_DM);
  stm32_configgpio(GPIO_OTGFS_DP);
  stm32_configgpio(GPIO_OTGFS_ID);
  stm32_configgpio(GPIO_OTGFS_SOF);

  /* Uninitialize the hardware so that we know that we are starting from a
   * known state. */

  up_usbuninitialize(priv);

  /* Initialie the driver data structure */

  stm32_swinitialize(priv);

  /* Initialize the USB OTG core */

  ret = stm32_hwinitialize(priv);
  if (ret < 0)
    {
      udbg("stm32_hwinitialize failed\n", ret);
      goto errout;
    }

  /* Attach and enable interrupts */

  ret = irq_attach(STM32_IRQ_OTGFS, stm32_usbinterrupt);
  if (ret < 0)
    {
      udbg("irq_attach failed\n", ret);
      goto errout;
    }

  /* Disconnect device */

  stm32_pullup(&priv->usbdev, false);

  /* Reset/Re-initialize the USB hardware */

  stm32_usbreset(priv);

  /* Enable USB controller interrupts at the NVIC */

  up_enable_irq(STM32_IRQ_OTGFS);

  /* Set the interrrupt priority */

  up_prioritize_irq(STM32_IRQ_OTGFS, CONFIG_USB_PRI);
  return;

errout:
  up_usbuninitialize();
}

/*******************************************************************************
 * Name: up_usbuninitialize
 *******************************************************************************/

void up_usbuninitialize(void)
{
  /* At present, there is only a single OTG FS device support. Hence it is
   * pre-allocated as g_otgfsdev.  However, in most code, the private data
   * structure will be referenced using the 'priv' pointer (rather than the
   * global data) in order to simplify any future support for multiple devices.
   */

  struct stm32_usbdev_s *priv = &g_otgfsdev;
  irqstate_t flags;

  usbtrace(TRACE_DEVUNINIT, 0);

  if (priv->driver)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_DRIVERREGISTERED), 0);
      usbdev_unregister(priv->driver);
    }

  /* Disconnect device */

  flags = irqsave();
  stm32_pullup(&priv->usbdev, false);
  priv->usbdev.speed = USB_SPEED_UNKNOWN;

  /* Disable and detach IRQs */

  up_disable_irq(STM32_IRQ_USBOTG);
  irq_detach(STM32_IRQ_USBOTG);

  /* Reset the controller */

  stm32_putreg (USBDEV_USBCMD_RST, STM32_USBDEV_USBCMD);
  while (stm32_getreg (STM32_USBDEV_USBCMD) & USBDEV_USBCMD_RST)
      ;

  /* Turn off USB power and clocking */

  stm32_disableclock(CLKID_USBOTGAHBCLK);
  stm32_disableclock CLKID_EVENTROUTERPCLK);

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
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }

  if (g_otgfsdev.driver)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_DRIVER), 0);
      return -EBUSY;
    }
#endif

  /* First hook up the driver */

  g_otgfsdev.driver = driver;

  /* Then bind the class driver */

  ret = CLASS_BIND(driver, &g_otgfsdev.usbdev);
  if (ret)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_BINDFAILED), (uint16_t)-ret);
      g_otgfsdev.driver = NULL;
    }
  else
    {
      /* Enable USB controller interrupts */

      up_enable_irq(STM32_IRQ_USBOTG);

      /* FIXME: nothing seems to call DEV_CONNECT(), but we need to set
       *        the RS bit to enable the controller.  It kind of makes sense 
       *        to do this after the class has bound to us...
       * GEN:   This bug is really in the class driver.  It should make the
       *        soft connect when it is ready to be enumerated.  I have added
       *        that logic to the class drivers but left this logic here.
       */

      stm32_pullup(&g_otgfsdev.usbdev, true);
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
  if (driver != g_otgfsdev.driver)
    {
      usbtrace(TRACE_DEVERROR(STM32_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Unbind the class driver */

  CLASS_UNBIND(driver, &g_otgfsdev.usbdev);

  /* Disable USB controller interrupts */

  up_disable_irq(STM32_IRQ_USBOTG);

  /* Unhook the driver */

  g_otgfsdev.driver = NULL;
  return OK;
}

#endif /* CONFIG_USBDEV && CONFIG_STM32_OTGFSDEV */
