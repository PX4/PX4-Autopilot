/*******************************************************************************
 * arch/arm/src/at90usb/at90usb_usbdev.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

#include <avr/io.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

/*******************************************************************************
 * Definitions
 *******************************************************************************/

/* Configuration ***************************************************************/
/* PLL Settings are based on F_CPU frequency which is defined in the board.h file */

#if (BOARD_CPU_CLOCK == 8000000)
#  define USB_PLL_PSC                     ((1 << PLLP1) | (1 << PLLP0))
#elif (BOARD_CPU_CLOCK == 16000000)
#  if defined(__AVR_AT90USB647__)
#    define USB_PLL_PSC                   ((1 << PLLP2) | (1 << PLLP1))
#  else
#    define USB_PLL_PSC                   ((1 << PLLP2) | (1 << PLLP0))
#  endif
#else
#  error "Unsuppored CPU clock"
#endif

/* Debug ***********************************************************************/

/* Trace error codes */

#define AVR_TRACEERR_ALLOCFAIL            0x0001
#define AVR_TRACEERR_BADCLREPFEATURE      0x0002
#define AVR_TRACEERR_BADCLRDEVFEATURE     0x0003
#define AVR_TRACEERR_BADDEVGETSTATUS      0x0004
#define AVR_TRACEERR_BADEPNO              0x0005
#define AVR_TRACEERR_BADEPGETSTATUS       0x0006
#define AVR_TRACEERR_BADGETCONFIG         0x0007
#define AVR_TRACEERR_BADGETSETDESC        0x0008
#define AVR_TRACEERR_BADGETSTATUS         0x0009
#define AVR_TRACEERR_BADSETADDRESS        0x000a
#define AVR_TRACEERR_BADSETCONFIG         0x000b
#define AVR_TRACEERR_BADSETEPFEATURE      0x000c
#define AVR_TRACEERR_BADSETDEVFEATURE     0x000d
#define AVR_TRACEERR_BINDFAILED           0x000e
#define AVR_TRACEERR_DRIVER               0x000f
#define AVR_TRACEERR_DISPATCHSTALL        0x0010
#define AVR_TRACEERR_DRIVERREGISTERED     0x0011
#define AVR_TRACEERR_EPNULLPACKET         0x0012
#define AVR_TRACEERR_XFERTYPE             0x0013
#define AVR_TRACEERR_PKTSIZE              0x0014
#define AVR_TRACEERR_EPCFGBAD             0x0015
#define AVR_TRACEERR_EP0CFGBAD            0x0016
#define AVR_TRACEERR_EP0SETUPSTALLED      0x0017
#define AVR_TRACEERR_EP0RXOUTI            0x0018
#define AVR_TRACEERR_EP0FIFOFULL          0x0019
#define AVR_TRACEERR_EP0FIFONOTREADY      0x001a
#define AVR_TRACEERR_INFIFO               0x001b
#define AVR_TRACEERR_INVALIDCTRLREQ       0x001c
#define AVR_TRACEERR_INVALIDPARMS         0x001d
#define AVR_TRACEERR_IRQREGISTRATION      0x001e
#define AVR_TRACEERR_NOEP                 0x001f
#define AVR_TRACEERR_NOTCONFIGURED        0x0020

/* Trace interrupt codes */

#define AVR_TRACEINTID_GENINT             0x0001
#define AVR_TRACEINTID_EPINT              0x0002
#define AVR_TRACEINTID_VBUS               0x0003
#define AVR_TRACEINTID_SUSPEND            0x0004
#define AVR_TRACEINTID_WAKEUP             0x0005
#define AVR_TRACEINTID_EOR                0x0006
#define AVR_TRACEINTID_CLEARFEATURE       0x0007
#define AVR_TRACEINTID_DEVGETSTATUS       0x0008
#define AVR_TRACEINTID_DISPATCH           0x0009
#define AVR_TRACEINTID_EP0SETUP           0x000a
#define AVR_TRACEINTID_EPGETSTATUS        0x000b
#define AVR_TRACEINTID_EPIN               0x000c
#define AVR_TRACEINTID_EPOUT              0x000d
#define AVR_TRACEINTID_EP0SETUPSETADDRESS 0x000e
#define AVR_TRACEINTID_GETCONFIG          0x000f
#define AVR_TRACEINTID_GETSETDESC         0x0010
#define AVR_TRACEINTID_GETSETIF           0x0011
#define AVR_TRACEINTID_GETSTATUS          0x0012
#define AVR_TRACEINTID_IFGETSTATUS        0x0013
#define AVR_TRACEINTID_SETCONFIG          0x0014
#define AVR_TRACEINTID_SETFEATURE         0x0015
#define AVR_TRACEINTID_SYNCHFRAME         0x0016

/* Hardware interface **********************************************************/

/* Endpoints ******************************************************************/

/* Number of endpoints */

#define AVR_NENDPOINTS                    (7) /* ep0-6 */

/* Endpoint 0 is special... */

#define AVR_EP0		                      (0)
#define AVR_CTRLEP_SIZE                   (8)

/* Bit encoded ep0-6 */

#define AVR_ALL_EPS                       (0x7f)

/* Endpoint configuration definitions */

#define AVR_EPTYPE_CTRL                   (0 << EPTYPE0)
#define AVR_EPTYPE_ISOC                   (1 << EPTYPE0)
#define AVR_EPTYPE_BULK                   (2 << EPTYPE0)
#define AVR_EPTYPE_INTR                   (3 << EPTYPE0)

#define AVR_DIR_OUT                       (0 << EPDIR)
#define AVR_DIR_IN                        (1 << EPDIR)

#define AVR_SINGLE_BANK                   (0 << EPBK0)
#define AVR_DOUBLE_BANK                   (1 << EPBK0)

#define AVR_EPSIZE_8                      (0 << EPSIZE0)
#define AVR_EPSIZE_16                     (1 << EPSIZE0)
#define AVR_EPSIZE_32                     (2 << EPSIZE0)
#define AVR_EPSIZE_64                     (3 << EPSIZE0)
#define AVR_EPSIZE_128                    (4 << EPSIZE0)
#define AVR_EPSIZE_256                    (5 << EPSIZE0)

/* General endpoint defintions */

#define AVR_EP0                           (0)
#define AVR_NENDPOINTS                    (7)
#define AVR_EPNO_MASK                     (3)

#define AVR_TIMEOUT_LONG                  (100)
#define AVR_TIMEOUT_SHORT                 (32)
#define AVR_TIMEOUT_NONE                  (0)

/* Request queue operations ****************************************************/

#define avr_rqempty(ep)                   ((ep)->head == NULL)
#define avr_rqpeek(ep)                    ((ep)->head)

/*******************************************************************************
 * Private Types
 *******************************************************************************/

/* A container for a request so that the request may be retained in a list */

struct avr_req_s
{
  struct usbdev_req_s req;    /* Standard USB request */
  struct avr_req_s *flink;    /* Supports a singly linked list */
};

/* This is the internal representation of an endpoint */

struct avr_ep_s
{
  /* Common endpoint fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_ep_s
   * to struct avr_ep_s. */

  struct usbdev_ep_s ep;      /* Standard endpoint structure */

  /* AVR-specific fields */

  struct avr_req_s *head;     /* Request list for this endpoint */
  struct avr_req_s *tail;
  struct avr_req_s *pending;  /* Pending IN request */
  uint8_t stalled:1;          /* 1: Endpoint is stalled */
  uint8_t epin:1;             /* 1: IN endpoint */
};

/* This structure retains the state of the USB device controller */

struct avr_usbdev_s
{
  /* Common device fields.  This must be the first thing defined in the
   * structure so that it is possible to simply cast from struct usbdev_s to
   * struct avr_usbdev_s. */

  struct usbdev_s usbdev;

  /* The bound device class driver */

  struct usbdevclass_driver_s *driver;

  /* AVR-specific fields */

  uint8_t  ep0buf[64];        /* buffer for EP0 short transfers */
  uint8_t  paddr;             /* Address assigned by SETADDRESS */
  uint8_t  epavail;           /* Bitset of available (unconfigured) endpoints */
  uint8_t  epinset;           /* The set of all configured IN endpoints */
  uint8_t  epoutset;          /* The set of all configured OUT endpoints */
  uint8_t  stalled:1;         /* 1: Protocol stalled */
  uint8_t  selfpowered:1;     /* 1: Device is self powered */
  uint8_t  paddrset:1;        /* 1: Peripheral addr has been set */
  uint8_t  attached:1;        /* 1: Host attached */
#ifdef CONFIG_USBDEV_SELFPOWERED
  uint8_t  wkupen:1;          /* 1: Wake-up enabled */
#endif
  volatile bool connected;    /* Device is connected */

  /* The endpoint list */

  struct avr_ep_s eplist[AVR_NENDPOINTS];
};

/*******************************************************************************
 * Private Function Prototypes
 *******************************************************************************/

/* Request queue operations ****************************************************/

static FAR struct avr_req_s *avr_rqdequeue(FAR struct avr_ep_s *privep);
static inline void avr_rqenqueue(FAR struct avr_ep_s *privep,
                                 FAR struct avr_req_s *req);

/* Low level data transfers and request operations *****************************/

static void avr_txready(void);
static int avr_fifoready(int timeout);
static void avr_ep0send(FAR const uint8_t *buffer, uint16_t buflen);
static inline int avr_epNsend(FAR struct avr_ep_s *privep,
                              FAR struct avr_req_s *privreq);
static inline int avr_epNrecv(FAR struct avr_ep_s *privep,
                              FAR struct usbdev_req_s *req);
static int avr_epINqueue(FAR struct avr_ep_s *privep);
static int avr_epOUTqueue(FAR struct avr_ep_s *privep);
static void avr_reqcomplete(FAR struct avr_ep_s *privep, FAR struct avr_req_s *privreq,
                            int result);
static void avr_cancelrequests(FAR struct avr_ep_s *privep, int status);
static void avr_cancelall(int status);

/* Endpoint interrupt handling *************************************************/

static struct avr_ep_s *avr_epfindbyaddr(uint8_t epno);
static void avr_dispatchrequest(FAR const struct usb_ctrlreq_s *ctrl);
static int avr_ep0configure(void);
static void avr_setaddress(uint8_t address);
static void avr_ep0setup(void);
static int avr_epinterrupt(int irq, FAR void *context);

/* General interrupt handling **************************************************/

static void avr_epreset(FAR struct avr_ep_s *privep, int status);
static void avr_usbreset(void);
static void avr_genvbus(void);
static inline void avr_gensuspend(void);
static void avr_genwakeup(void);
static inline void avr_geneor(void);
static int avr_geninterrupt(int irq, FAR void *context);

/* USB device controller operations ********************************************/

static int avr_epconfigure(FAR struct usbdev_ep_s *ep,
                           const struct usb_epdesc_s *desc, bool last);
static int avr_epdisable(FAR struct usbdev_ep_s *ep);
static FAR struct usbdev_req_s *avr_epallocreq(FAR struct usbdev_ep_s *ep);
static void avr_epfreereq(FAR struct usbdev_ep_s *ep,
                          FAR struct usbdev_req_s *);
#ifdef CONFIG_USBDEV_DMA
static void *avr_epallocbuffer(FAR struct usbdev_ep_s *ep, unsigned bytes);
static void avr_epfreebuffer(FAR struct usbdev_ep_s *ep, FAR void *buf);
#endif
static int avr_epsubmit(FAR struct usbdev_ep_s *ep, struct usbdev_req_s *req);
static int avr_epcancel(FAR struct usbdev_ep_s *ep, struct usbdev_req_s *req);
static int avr_epstall(FAR struct usbdev_ep_s *ep, bool resume);

static FAR struct usbdev_ep_s *avr_allocep(FAR struct usbdev_s *dev,
                                           uint8_t epno, bool in,
                                           uint8_t eptype);
static void avr_freeep(FAR struct usbdev_s *dev, FAR struct usbdev_ep_s *ep);
static int avr_getframe(struct usbdev_s *dev);
static int avr_wakeup(struct usbdev_s *dev);
static int avr_selfpowered(struct usbdev_s *dev, bool selfpowered);
static int avr_pullup(struct usbdev_s *dev, bool enable);

/*******************************************************************************
 * Private Data
 *******************************************************************************/

/* Since there is only a single USB interface, all status information can be
 * be simply retained in a single global instance.
 */

static struct avr_usbdev_s g_usbdev;

static const struct usbdev_epops_s g_epops =
{
  .configure   = avr_epconfigure,
  .disable     = avr_epdisable,
  .allocreq    = avr_epallocreq,
  .freereq     = avr_epfreereq,
#ifdef CONFIG_USBDEV_DMA
  .allocbuffer = avr_epallocbuffer,
  .freebuffer  = avr_epfreebuffer,
#endif
  .submit      = avr_epsubmit,
  .cancel      = avr_epcancel,
  .stall       = avr_epstall,
};

static const struct usbdev_ops_s g_devops =
{
  .allocep     = avr_allocep,
  .freeep      = avr_freeep,
  .getframe    = avr_getframe,
  .wakeup      = avr_wakeup,
  .selfpowered = avr_selfpowered,
  .pullup      = avr_pullup,
};

/*******************************************************************************
 * Public Data
 *******************************************************************************/

/*******************************************************************************
 * Private Functions
 *******************************************************************************/

/*******************************************************************************
 * Name: avr_rqdequeue
 *
 * Description:
 *   Remove a request from an endpoint request queue
 *
 *******************************************************************************/

static FAR struct avr_req_s *avr_rqdequeue(FAR struct avr_ep_s *privep)
{
  FAR struct avr_req_s *ret = privep->head;

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
 * Name: avr_rqenqueue
 *
 * Description:
 *   Add a request from an endpoint request queue
 *
 *******************************************************************************/

static inline void avr_rqenqueue(FAR struct avr_ep_s *privep,
                                 FAR struct avr_req_s *req)
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
      privep->tail = req;
    }
}

/*******************************************************************************
 * Name: avr_txready
 *
 * Description:
 *   Wait for the selected endpoint to be ready for an IN (TX) transfer
 *
 *******************************************************************************/

static void avr_txready(void)
{
  int retries = 10000;
  while (((UEINTX & (1 << TXINI))  == 0) && retries-- > 0);
}

/*******************************************************************************
 * Name: avr_fifoready
 *
 * Description:
 *   Wait for the selected endpoint FIFO to be ready
 *
 *******************************************************************************/

static int avr_fifoready(int timeout)
{
  UDINT &= ~(1 << SOFI);

  for (;;)
    {
      /* Check if the FIFO is ready by testing RWAL (read/write allowed).  The
       * meaning of this bigtdepends on the direction of the endpoint: For an
       * OUT endpoint, the RWAL bit is set if the firmware can read data from
       * the bank, and cleared by hardware when the bank is empty; For an IN
       * endpoint, the RWAL bit is set if the firmware can write data to the
       * bank, and cleared by hardware when the bank is full.
       */

      if ((UEINTX & (1 << RWAL)) != 0)
        {
          return OK;
        }

      /* Check if we are still connected and not stalled */

      if (!(g_usbdev.connected))
        {
          return -ENODEV;
        }
      else if ((UECONX & (1 << STALLRQ)) != 0)
        {
          return -EAGAIN;
        }

      /* Timeing is driven by the start of frame (SOF) interrupt which we
       * assume here to be at a one millisecond rate. */

      if ((UDINT & (1 << SOFI)) != 0)
        {
          /* Clear the SOF interrupt decrement the count of elapsed
           * milliseconds */

          UDINT &= ~(1 << SOFI);

          if ((timeout--) > 0)
            {
              /* The timeout has elapsed... return a failure */

              return -ETIME;
            }
        }
    }
}

/*******************************************************************************
 * Name: avr_ep0send
 *
 * Description:
 *   Schedule a short TX transfer for Endpoint 0
 *
 * Assumptions:
 * - Endpoint 0 is already selected.
 *
 *******************************************************************************/

static void avr_ep0send(FAR const uint8_t *buffer, uint16_t buflen)
{
  FAR const uint8_t *ptr = buffer;
  uint8_t regval;

  /* Loop while there are more bytes to send and RXOUTI is clear.  RXOUTI is
   * set when a new OUT data is received
   */

  while (buflen)
    {
     /* Verify that RXOUTI is clear.  RXOUTI is set when a new OUT data is
      * received.  In this case, we have not option but to abort the transfer.
      */

      regval = UEINTX;
      if ((regval & (1 << RXOUTI)) != 0)
        {
           usbtrace(TRACE_DEVERROR(AVR_TRACEERR_EP0RXOUTI), regval);
           return;
        }

      /* Okay... wait for the selected endpoint to be ready for an TX transfer */

      avr_txready();

      /* Now send as many bytes as possible */

      while (buflen > 0)
        {
          /* Break out of the loop if the FIFO is full */

          if (UEBCX == AVR_CTRLEP_SIZE)
            {
              /* Clearing FIFOCON frees the current bank and switches to the
               * following bank.  TXINI must be cleared to acknowledge the
               * interrupt.
               */

              usbtrace(TRACE_DEVERROR(AVR_TRACEERR_EP0FIFOFULL), regval);

              /* TXINI must always be cleared BEFORE clearing FIFOCON */

              regval = UEINTX;
              regval &= ~(1 << TXINI);
              UEINTX = regval;
              regval &= ~(1 << FIFOCON);
              UEINTX = regval;
              break;
            }

          /* Not full, transfer another byte */

          UEDATX = *ptr++;
          buflen--;
        }

      /* Clearing FIFOCON frees the current bank and switches to the following
       * bank.  TXINI must be cleared to acknowledge the interrupt. TXINI must
       * always be cleared BEFORE clearing FIFOCON.
       */

      regval = UEINTX;
      regval &= ~(1 << TXINI);
      UEINTX = regval;
      regval &= ~(1 << FIFOCON);
      UEINTX = regval;
    }
}

/*******************************************************************************
 * Name: avr_epNsend
 *
 * Description:
 *   Perform a TX transfer for Endpoint N
 *
 *******************************************************************************/

static inline int avr_epNsend(FAR struct avr_ep_s *privep,
                              FAR struct avr_req_s *privreq)
{
  FAR struct usbdev_req_s *req;
  FAR const uint8_t *buffer;
  uint16_t buflen;
  uint16_t len;
  uint16_t pktmask;
  uint8_t  ret;
  uint8_t  more;
  uint8_t  regval;
  bool     zlp;

  /* Check if endpoint is ready for read/write operations */

  DEBUGASSERT((UEINTX & (1 << RWAL)) != 0);

  /* Setup pointers and counts for this transfer */

  req     = &privreq->req;
  buffer  = &req->buf[req->xfrd];
  buflen  = req->len - req->xfrd;
  zlp     = ((privreq->req.flags & USBDEV_REQFLAGS_NULLPKT) != 0);
  pktmask = privep->ep.maxpacket - 1;

  /* Select the endpoint */

  UENUM   = privep->ep.eplog;

  /* This function should not be called if we are not ready to write! */

  ret = avr_fifoready(AVR_TIMEOUT_LONG);
  if (ret != OK)
    {
      usbtrace(TRACE_DEVERROR(AVR_TRACEERR_EP0FIFOFULL), regval);
      return -EAGAIN;
    }

  /* Send the USB data.  The outer loop handles for each packet of data
   * (including zero-length packets) 
   */

  do
    {
      /* Then loop, putting each outgoing byte into the transmit FIFO until
       * either (1) all of the data has been sent (len == buflen) or
       * (2) the transmit FIFO is full
       */

      len = 0;
      while (len < buflen && (UEINTX & (1 << RWAL)) != 0)
        {
          /* Add another byte to the transmit FIFO */

          UEDATX = *buffer++;
          len++;
        }

      /* We now have one complete packet in the transmit FIFO(or maybe two
       * packets if dual buffering is enabled).
       *
       * Clear any pending TXINI interrupts
       */

      UEINT &= ~(1 <<  privep->ep.eplog);

      /* Clear TXINI and send what is in the transmit FIFO (could be a zero
       * length packet). TXINI must always be cleared BEFORE clearing FIFOCON.
       */

      regval = UEINTX;
      regval &= ~(1 << TXINI);
      UEINTX = regval;
      regval &= ~(1 << FIFOCON);
      UEINTX = regval;

      /* Adjust the remaining number of bytes to transfer. */

      req->xfrd += len;
      buffer    += len;
      buflen    -= len;

      usbtrace(TRACE_WRITE(privep->ep.eplog), privreq->req.xfrd);

      /* Check if we need to send a zero length packet (ZLP); We need to send
       * a ZLP if the last packet sent was exactly equal to the packet length
       * AND if the endpoint is configuration to send ZLPs. However, in dual
       * buffer mode, we may have actually just sent two packets so the actual
       * check is for a non-zero, transfer of a multiple of the packet
       */

      if (buflen > 0)
        {
          /* There is more data to be sent */

          more = true;
        }
      else if (zlp)
        {
          /* All of the data has been sent.  A ZLP might be needed if the last
           * transfer was an exact multiple of the packet size.
           */

          if (len && (len & pktmask) == 0)
            {
              /* The last packet was not a ZLP and was an example multiple of
               * the packet size.  A ZLP is needed.
               */

              more = true;
            }
          else
            {
              /* The last packet was a ZLP or was a partial packet.  We are
               * finished with this request.
               */

              more = false;
            }
        }
      else
        {
          /* No more data to be sent and a ZLP is not needed */

          more = false;
        }

      /* RWAL will be de-asserted when there is no more space in the transmit
       * FIFO.  We care only if we have more data (or a zero-length-packet) to
       * send. Try a short inline wait to see if the FIFO becomes write ready.
       * This saves handling an interrupt most of the time (really depends on
       * how fast the host takes the data from the transmit FIFO).
       */

      if (more && (ret = avr_fifoready(AVR_TIMEOUT_SHORT)))
        {
          /* If the endpoint simply did not become ready within the timeout,
           * then handle the remainder of the transfer asynchronously in the
           * TXINI interrupt handler. */

          if (ret == -ETIME)
            {
              /* Enable the endpoint IN interrupt ISR. */

              UEIENX |= (1 << TXINE);
            }

          /* A fatal error occurred while waiting for the IN FIFO to become
           * available.
           */

          usbtrace(TRACE_DEVERROR(AVR_TRACEERR_INFIFO), regval);
          return ret;
        }
    }
  while (more);
  return OK;
}

/*******************************************************************************
 * Name: avr_epNrecv
 *
 * Description:
 *   Perform an RX transfer for Endpoint N
 *
 *******************************************************************************/

static inline int avr_epNrecv(FAR struct avr_ep_s *privep,
                              FAR struct usbdev_req_s *req)
{
  FAR uint8_t *buffer;
  uint8_t regval;
  int ret;

  /* Setup pointers and counts for this transfer */

  buffer    = req->buf;
  req->xfrd = 0;

  /* This function should not be called if we are not ready to read! */

  ret = avr_fifoready(AVR_TIMEOUT_LONG);
  if (ret != OK)
    {
      usbtrace(TRACE_DEVERROR(AVR_TRACEERR_EP0FIFONOTREADY), ret);
      return ret;
    }

  /* Loop until the requested number of bytes have been read */

  while (req->xfrd < req->len)
    {
      /* RWAL will be de-asserted when everything has been read from the
       * receive FIFO */

      if (((UEINTX & (1 << RWAL)) == 0))
        {
          /* The FIFO is empty.. Acknowledge receipt of the packet. RXOUTI must
           * always be cleared BEFORE clearing FIFOCON.
           */

          regval = UEINTX;
          regval &= ~(1 << RXOUTI);
          UEINTX = regval;
          regval &= ~(1 << FIFOCON);
          UEINTX = regval;

          /* Return success */

          usbtrace(TRACE_READ(privep->ep.eplog), req->xfrd);
          return OK;
        }
      else
        {
          /* Receive the next byte */

          *buffer++ = UEDATX;

          /* Increment the number of bytes received and try again */

          req->xfrd++;
        }
    }

  /* We get here if the request buffer is full.  There could be more bytes
   * pending in the FIFO?
   *
   * Finalize the OUT stream transfer.  RXOUTI must always be cleared BEFORE
   * clearing FIFOCON.
   */

  regval = UEINTX;
  regval &= ~(1 << RXOUTI);
  UEINTX = regval;
  regval &= ~(1 << FIFOCON);
  UEINTX = regval;

  usbtrace(TRACE_READ(privep->ep.eplog), req->xfrd);
  return OK;
}

/*******************************************************************************
 * Name: avr_epINqueue
 *
 * Description:
 *   This is part of the IN endpoint interrupt handling logic.  It is called
 *   from interrupt handling logic for an endpoint when the TXIN endpoint
 *   interrupt occurs.  Thus function is also called from the requeust enqueuing
 *   logic BUT with interrupts disabled.
 *
 *******************************************************************************/

static int avr_epINqueue(FAR struct avr_ep_s *privep)
{
  FAR struct avr_req_s *privreq;
  int ret = OK;

  usbtrace(TRACE_INTDECODE(AVR_TRACEINTID_EPIN), 0);

  /* First, check if there is already pending IN transfer */

  if (privep->pending)
    {
      /* Yes.. use this request to continue the transfer */

      privreq         = privep->pending;
    }
  else
    {
       /* No.. remove the next request from the queue of IN requests */

       privreq         =  avr_rqdequeue(privep);
       privep->pending = privreq;
    }

  /* Is there an IN request */

  if (privreq)
    {
      /* Yes.. perform the IN transfer */

      ret = avr_epNsend(privep, privreq);

      /* The return value of -ETIME means that the transfer was not
       * finished within this interrupt.  We just need to exit with the
       * pending transfer in place.
       */

       if (ret == OK || ret != -ETIME)
        {
          /* The transfer has completed, perhaps with an error.  Return the request
           * to the class driver.
           */

          usbtrace(TRACE_COMPLETE(privep->ep.eplog), privreq->req.xfrd);
          privep->pending = NULL;
          avr_reqcomplete(privep, privreq, ret);
        }
    }
  return ret;
}

/*******************************************************************************
 * Name: avr_epOUTqueue
 *
 * Description:
 *   This is part of the OUT endpointeinterrupt handling logic.  It is called
 *   from interrupt handling logic for an endpoint when the RXOUT endpoint
 *   interrupt occurs.
 *
 *******************************************************************************/

static int avr_epOUTqueue(FAR struct avr_ep_s *privep)
{
  FAR struct avr_req_s *privreq;
  int ret = OK;

  usbtrace(TRACE_INTDECODE(AVR_TRACEINTID_EPOUT), 0);

  /* Remove the next request from the queue of OUT requests */

  privreq =  avr_rqdequeue(privep);

  /* Is there an OUT request */

  if (privreq)
    {
      /* Yes.. perform the OUT transfer */

      ret = avr_epNrecv(privep, &privreq->req);

      /* The transfer has completed, perhaps with an error.  Return the request
       * to the class driver.
       */

      usbtrace(TRACE_COMPLETE(privep->ep.eplog), privreq->req.xfrd);
      avr_reqcomplete(privep, privreq, ret);
    }
  return ret;
}

/*******************************************************************************
 * Name: avr_reqcomplete
 *
 * Description:
 *   Handle termination of the request at the head of the endpoint request queue.
 *
 *******************************************************************************/

static void avr_reqcomplete(FAR struct avr_ep_s *privep, FAR struct avr_req_s *privreq,
                            int result)
{
  /* If endpoint 0, temporarily reflect the state of protocol stalled in the
   * callback. */

  bool stalled = privep->stalled;
  if (privep->ep.eplog == AVR_EP0)
    {
      privep->stalled = g_usbdev.stalled;
    }

  /* Save the result in the request structure */

  privreq->req.result = result;

  /* Callback to the request completion handler */

  privreq->req.callback(&privep->ep, &privreq->req);

  /* Restore the stalled indication */

  privep->stalled = stalled;
}

/*******************************************************************************
 * Name: avr_cancelrequests
 *
 * Description:
 *   Cancel all pending requests for an endpoint
 *
 *******************************************************************************/

static void avr_cancelrequests(FAR struct avr_ep_s *privep, int status)
{
  /* Is there a pending, active IN transfer? */

  if (privep->pending)
    {
      /* Remove the pending request */

      FAR struct avr_req_s *privreq = privep->pending;
      privep->pending = NULL;

      /* Make sure that the endpoint IN interrupt is disabled.  */

      UENUM = privep->ep.eplog;
      UEIENX &= ~(1 << TXINE);

      /* Complete the request with the provided status */

      usbtrace(TRACE_COMPLETE(privep->ep.eplog), privreq->req.xfrd);
      avr_reqcomplete(privep, privreq, status);
    }

  /* Then complete any queue requests.  None of these should be active. */

  while (!avr_rqempty(privep))
    {
      usbtrace(TRACE_COMPLETE(privep->ep.eplog), (avr_rqpeek(privep))->req.xfrd);
      avr_reqcomplete(privep, avr_rqdequeue(privep), status);
    }
}

/*******************************************************************************
 * Name: avr_cancelall
 *
 * Description:
 *   Cancel all pending requests for an endpoint
 *
 *******************************************************************************/

static void avr_cancelall(int status)
{
  struct avr_ep_s *privep;
  int i;

  for (i = 1; i < AVR_NENDPOINTS; i++)
    {
      privep = &g_usbdev.eplist[i];
      if (privep)
        {
          avr_cancelrequests(privep, status);
        }
    }
}

/*******************************************************************************
 * Name: avr_epfindbyaddr
 *
 * Description:
 *   Find the physical endpoint structure corresponding to a logic endpoint
 *   address
 *
 *******************************************************************************/

static struct avr_ep_s *avr_epfindbyaddr(uint8_t epno)
{
  struct avr_ep_s *privep;
  int i;

  /* Endpoint zero is a special case */

  if (epno == AVR_EP0)
    {
      return &g_usbdev.eplist[0];
    }

  /* Handle the remaining */

  for (i = 1; i < AVR_NENDPOINTS; i++)
    {
      privep = &g_usbdev.eplist[i];

      /* Same logical endpoint number? (includes direction bit) */

      if (epno == privep->ep.eplog)
        {
          /* Return endpoint found */

          return privep;
        }
    }

  /* Return endpoint not found */

  return NULL;
}

/*******************************************************************************
 * Name: avr_dispatchrequest
 *
 * Description:
 *   Provide unhandled setup actions to the class driver. This is logically part
 *   of the USB interrupt handler.
 *
 *******************************************************************************/

static void avr_dispatchrequest(FAR const struct usb_ctrlreq_s *ctrl)
{
  int ret = -EIO;

  usbtrace(TRACE_INTDECODE(AVR_TRACEINTID_DISPATCH), 0);
  if (g_usbdev.driver)
    {
      /* Forward to the control request to the class driver implementation */

      ret = CLASS_SETUP(g_usbdev.driver, &g_usbdev.usbdev, ctrl, NULL, 0);
    }

  if (ret < 0)
    {
      /* Stall on failure */

      usbtrace(TRACE_DEVERROR(AVR_TRACEERR_DISPATCHSTALL), 0);
      g_usbdev.stalled = true;
    }
}

/*******************************************************************************
 * Name: avr_ep0configure
 *
 * Description:
 *   Reset Usb engine
 *
 *******************************************************************************/

static int avr_ep0configure(void)
{
  FAR struct avr_ep_s *privep = &g_usbdev.eplist[AVR_EP0];
  uint8_t regval;

  /* Configure endpoint 0 */

  UENUM   = AVR_EP0;
  UECONX |= (1 << EPEN);
  UECFG1X = 0;
  UECFG0X = AVR_EPTYPE_CTRL;
  UECFG1X =  (1 << ALLOC) | AVR_EPSIZE_8;

  /* Check for configuration failure */

  regval = UESTA0X;
  if ((regval &  (1 << CFGOK)) == 0)
    {
      usbtrace(TRACE_DEVERROR(AVR_TRACEERR_EP0CFGBAD), regval);
      return -EINVAL;
    }

  /* Initialize the endpoint data structure.  Mark EP0 as an IN endpoint so
   * that the submit() logic will know that any enqueue packets are to be
   * sent.
   */

  memset(privep, 0, sizeof(struct avr_ep_s));
  privep->ep.ops       = &g_epops;
  privep->ep.maxpacket = AVR_CTRLEP_SIZE;
  privep->epin         = 1;

  /* Enable OUT interrupts */

  UEIENX |= (1 << RXOUTE);
  return OK;
}

/*******************************************************************************
 * Name: avr_epreset
 *
 * Description:
 *   Reset the specified endpoint
 *
 * Input Parameters:
 *   epno - The endpoint to be reset
 *
 *******************************************************************************/

static void avr_epreset(FAR struct avr_ep_s *privep, int status)
{
  uint8_t epno = privep->ep.eplog;

  /* Reset the endpoint hardware */

  UEINT   &= ~(1 << epno);
  UENUM    = epno;
  UEIENX   = 0;
  UEINTX   = 0;
  UECFG1X &= ~(1 << ALLOC);
  UECONX  &= ~(1 << EPEN);

  /* Cancel all requests */

  avr_cancelrequests(privep, status);

  /* Reset endpoint status */

  privep->stalled = false;
}

/*******************************************************************************
 * Name: avr_usbreset
 *
 * Description:
 *   Reset Usb engine
 *
 *******************************************************************************/

static void avr_usbreset(void)
{
  uint8_t epno;
  uint8_t regval;

  /* Disable all interrupts */

  USBCON &= ~((1 << VBUSTE) | (1 << IDTE));
  UDIEN   = 0;

  /* Clear all pending interrupts */

  USBINT  = 0;
  UDINT   = 0;

  /* Reset selected state variables */

  g_usbdev.connected = false;
#ifdef CONFIG_USBDEV_SELFPOWERED
  g_usbdev.wkupen    = false;
#endif

  /* Reset endpoints */

  for (epno = 0; epno < AVR_NENDPOINTS; epno++)
    {
      struct avr_ep_s *privep = &g_usbdev.eplist[epno];
      avr_epreset(privep, -ESHUTDOWN);
    }

  /* Tell the class driver that we are disconnected. The class driver should
   * then accept any new configurations. */

  if (g_usbdev.driver)
    {
      CLASS_DISCONNECT(g_usbdev.driver, &g_usbdev.usbdev);
    }

  /* Enable the PLL */

  PLLCSR  =  USB_PLL_PSC;
  PLLCSR |= (1 << PLLE);
  while ((PLLCSR & (1 << PLOCK)) == 0);

  /* Reset the USB interface */

  regval = USBCON;
  USBCON = (regval & ~(1 << USBE));
  USBCON = (regval | (1 << USBE));

#ifndef CONFIG_USB_DISABLE_PADREGULATOR
  /* Enable the USB pad regulator */

  UHWCON |= (1 << UVREGE);
#endif

  /* Enable USB clock inputs */

  USBCON &= ~(1 << FRZCLK);

  /* Select low or high speed */

#ifdef CONFIG_USB_LOWSPEED
  UDCON |= (1 << LSM);
#else
  UDCON &= ~(1 << LSM);
#endif

  /* Set USB address to 0 */

  avr_setaddress(0);

  /* EndPoint 0 initialization */

  (void)avr_ep0configure();

  /* Enable VBUS interrupts */

  USBCON |= (1 << VBUSTE);

  /* Attach the device to the USB bus. This announces the device's presence to
   * any attached USB host, starting the enumeration process. If no host is
   * present, attaching the device will allow for enumeration once a host is
   * connected to the device.
   */

  UDCON &= ~(1 << DETACH);

  /* Enable Suspend and reset interrupts */

  UDIEN |=  ((1 << SUSPE) | (1 << EORSTE));
}

/*******************************************************************************
 * Name: avr_usbshutdown
 *
 * Description:
 *   Shutdown the USB interface and put the hardare in a known state
 *
 *******************************************************************************/

void avr_usbshutdown(void)
{
  /* Inform the class driver of the disconnection */

  if (g_usbdev.driver)
    {
      CLASS_DISCONNECT(g_usbdev.driver, &g_usbdev.usbdev);
    }

  /* Detach the device from the USB bus, terminating communications */

  UDCON |= (1 << DETACH);

  /* Disable all interrupts */

  USBCON &= ~((1 << VBUSTE) | (1 << IDTE));
  UDIEN = 0;

  /* Clear all pending interrupts */

  USBINT = 0;
  UDINT  = 0;

  g_usbdev.connected = false;
#ifdef CONFIG_USBDEV_REMOTEWAKEUP
  g_usbdev.wkupen    = false;
#endif

  /* Disable the USB interface */

  USBCON &= ~(1 << USBE);

  /* Shut down the USB PLL */

  PLLCSR = 0;

  /* Turn off the VBUS pad */

  USBCON &= ~(1 << OTGPADE);
}

/*******************************************************************************
 * Name: avr_setaddress
 *
 * Description:
 *   Set the devices USB address
 *
 *******************************************************************************/

static inline void avr_setaddress(uint8_t address)
{
  uint8_t regval;

  g_usbdev.paddr    = address;
  g_usbdev.paddrset = (address != 0);

  UEINTX &= ~(1 << RXSTPI);

  /* TXINI must always be cleared BEFORE clearing FIFOCON. */

  regval = UEINTX;
  regval &= ~(1 << TXINI);
  UEINTX = regval;
  regval &= ~(1 << FIFOCON);
  UEINTX = regval;
  regval  = UEINTX;

  avr_txready();
  UDADDR  = ((1 << ADDEN) | address);
}

/*******************************************************************************
 * Name: avr_ep0setup
 *
 * Description:
 *   USB Ctrl EP Setup Event. This is logically part of the USB interrupt
 *   handler.  This event occurs when a setup packet is receive on EP0 OUT.
 *
 *******************************************************************************/

static inline void avr_ep0setup(void)
{
  struct avr_ep_s *privep;
  struct usb_ctrlreq_s ctrl;
  uint8_t *ptr;
  uint16_t value;
  uint16_t index;
  uint16_t len;
  uint8_t  i;

  usbtrace(TRACE_INTDECODE(AVR_TRACEINTID_EP0SETUP), 0);

  /* Terminate any pending requests */

  avr_cancelrequests(&g_usbdev.eplist[AVR_EP0], -EPROTO);
  avr_cancelrequests(&g_usbdev.eplist[AVR_EP0], -EPROTO);

  /* Assume NOT stalled */

  g_usbdev.eplist[AVR_EP0].stalled = false;
  g_usbdev.eplist[AVR_EP0].stalled = false;
  g_usbdev.stalled = false;

  /* Read EP0 setup data -- Read the setup data from the hardware. */

  ptr = (uint8_t*)&ctrl;
  for (i = 0; i < USB_SIZEOF_CTRLREQ; i++)
    {
      *ptr++ = UEDATX;
    }

  /* Extract the little-endian 16-bit values to host order */

  value = GETUINT16(ctrl.value);
  index = GETUINT16(ctrl.index);
  len   = GETUINT16(ctrl.len);

  ullvdbg("type=%02x req=%02x value=%04x index=%04x len=%04x\n",
          ctrl.type, ctrl.req, value, index, len);

  /* Dispatch any non-standard requests */

  if ((ctrl.type & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_STANDARD)
    {
      avr_dispatchrequest(&ctrl);
    }
  else
    {
      /* Handle standard request.  Pick off the things of interest to the USB
       * device controller driver; pass what is left to the class driver.
       */

      switch (ctrl.req)
        {
        case USB_REQ_GETSTATUS:
          {
            /* type: device-to-host; recipient = device, interface, endpoint
             * value: 0 index: zero interface endpoint len: 2; data = status
             */

            usbtrace(TRACE_INTDECODE(AVR_TRACEINTID_GETSTATUS), 0);
            if (!g_usbdev.paddrset || len != 2 ||
                (ctrl.type & USB_REQ_DIR_IN) == 0 || value != 0)
              {
                g_usbdev.stalled = true;
              }
            else
              {
                switch (ctrl.type & USB_REQ_RECIPIENT_MASK)
                  {
                  case USB_REQ_RECIPIENT_ENDPOINT:
                    {
                      usbtrace(TRACE_INTDECODE(AVR_TRACEINTID_EPGETSTATUS), 0);
                      privep = avr_epfindbyaddr(index);
                      if (!privep)
                        {
                          usbtrace(TRACE_DEVERROR(AVR_TRACEERR_BADEPGETSTATUS), 0);
                          g_usbdev.stalled = true;
                        }
                      else
                        {
                          /* Return endpoint stalled status */

                          if (privep->stalled)
                            {
                              g_usbdev.ep0buf[0] = (1 << USB_FEATURE_ENDPOINTHALT); /* Stalled */
                            }
                          else
                            {
                              g_usbdev.ep0buf[0] = 0; /* Not stalled */
                            }
                          g_usbdev.ep0buf[1] = 0;

                          /* And send the response */

                          avr_ep0send(g_usbdev.ep0buf, 2);
                        }
                    }
                    break;

                  case USB_REQ_RECIPIENT_DEVICE:
                    {
                      if (index == 0)
                        {
                          usbtrace(TRACE_INTDECODE(AVR_TRACEINTID_DEVGETSTATUS), 0);

                          /* Features: Remote Wakeup=YES; selfpowered=? */
                          /* Return self-powered status */

#ifdef CONFIG_USBDEV_SELFPOWERED
                          g_usbdev.ep0buf[0] = (1 << USB_FEATURE_SELFPOWERED);
#else
                          g_usbdev.ep0buf[0] = 0;
#endif
                          /* Return remote wake-up enabled status */

#ifdef CONFIG_USBDEV_REMOTEWAKEUP
                          if (g_usbdev.wkupen)
                            {
                              status |= (1 << USB_FEATURE_REMOTEWAKEUP);
                            }
#endif
                          g_usbdev.ep0buf[1] = 0;

                          /* And send the response */

                          avr_ep0send(g_usbdev.ep0buf, 2);
                        }
                      else
                        {
                          usbtrace(TRACE_DEVERROR(AVR_TRACEERR_BADDEVGETSTATUS), 0);
                          g_usbdev.stalled = true;
                        }
                    }
                    break;

                  case USB_REQ_RECIPIENT_INTERFACE:
                    {
                      usbtrace(TRACE_INTDECODE(AVR_TRACEINTID_IFGETSTATUS), 0);
                      g_usbdev.ep0buf[0] = 0;
                      g_usbdev.ep0buf[1] = 0;

                      avr_ep0send(g_usbdev.ep0buf, 2);
                    }
                    break;

                  default:
                    {
                      usbtrace(TRACE_DEVERROR(AVR_TRACEERR_BADGETSTATUS), 0);
                      g_usbdev.stalled = true;
                    }
                    break;
                  }
              }
          }
          break;

        case USB_REQ_CLEARFEATURE:
          {
            /* type: host-to-device; recipient = device, interface or endpoint
             * value: feature selector index: zero interface endpoint; len:
             * zero, data = none
             */

            usbtrace(TRACE_INTDECODE(AVR_TRACEINTID_CLEARFEATURE), 0);
            switch (ctrl.type & USB_REQ_RECIPIENT_MASK)
              {
              case USB_REQ_RECIPIENT_ENDPOINT:
                if (g_usbdev.paddrset != 0 &&
                    value == USB_FEATURE_ENDPOINTHALT &&
                    len == 0 && 
                    (privep = avr_epfindbyaddr(index)) != NULL)
                  {
                    avr_epstall(&privep->ep, false);
                  }
                else
                  {
                    usbtrace(TRACE_DEVERROR(AVR_TRACEERR_BADCLREPFEATURE), value);
                    g_usbdev.stalled = true;
                  }
                break;

              case USB_REQ_RECIPIENT_DEVICE:
#ifdef CONFIG_USBDEV_SELFPOWERED
                if (g_usbdev.paddrset != 0 &&
                    value == USB_FEATURE_REMOTEWAKEUP &&
                    len == 0)
                  {
                    g_usbdev.wkupen = 0;
                  }
                else
                  {
                    usbtrace(TRACE_DEVERROR(AVR_TRACEERR_BADCLRDEVFEATURE), value);
                    g_usbdev.stalled = true;
                  }
                break;
#endif

              default:
                 avr_dispatchrequest(&ctrl);
                 break;
              }
          }
          break;

        case USB_REQ_SETFEATURE:
          {
            /* type: host-to-device; recipient = device, interface, endpoint
             * value: feature selector index: zero interface endpoint; len: 0;
             * data = none
             */

            usbtrace(TRACE_INTDECODE(AVR_TRACEINTID_SETFEATURE), 0);
            switch (ctrl.type & USB_REQ_RECIPIENT_MASK)
              {
              case USB_REQ_RECIPIENT_ENDPOINT:
                if (g_usbdev.paddrset != 0 &&
                    value == USB_FEATURE_ENDPOINTHALT &&
                    len == 0 && 
                    (privep = avr_epfindbyaddr(index)) != NULL)
                  {
                    avr_epstall(&privep->ep, true);
                  }
                else
                  {
                    usbtrace(TRACE_DEVERROR(AVR_TRACEERR_BADSETEPFEATURE), value);
                    g_usbdev.stalled = true;
                  }
                break;

              case USB_REQ_RECIPIENT_DEVICE:
#ifdef CONFIG_USBDEV_SELFPOWERED
                if (value == USB_FEATURE_TESTMODE)
                  {
                    ullvdbg("test mode: %d\n", index);
                  }
                else if (value == USB_FEATURE_REMOTEWAKEUP)
                  {
                     g_usbdev.wkupen = 1;
                  }
                else
                  {
                    usbtrace(TRACE_DEVERROR(AVR_TRACEERR_BADSETDEVFEATURE), value);
                    g_usbdev.stalled = true;
                  }
                break;
#endif

              default:
                 avr_dispatchrequest(&ctrl);
                 break;
              }
          }
          break;

        case USB_REQ_SETADDRESS:
          {
            /* type: host-to-device; recipient = device value: device address
             * index: 0 len: 0; data = none
             */

            usbtrace(TRACE_INTDECODE(AVR_TRACEINTID_EP0SETUPSETADDRESS), value);
            if ((ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE &&
                 index == 0 &&
                 len == 0 &&
                 value < 128)
              {
                /* Save the address */

                avr_setaddress(ctrl.value[0]);
              }
            else
              {
                usbtrace(TRACE_DEVERROR(AVR_TRACEERR_BADSETADDRESS), 0);
                g_usbdev.stalled = true;
              }
          }
          break;

        case USB_REQ_GETDESCRIPTOR:
          /* type: device-to-host; recipient = device value: descriptor type
           * and index index: 0 or language ID; len: descriptor len; data =
           * descriptor.
           */

        case USB_REQ_SETDESCRIPTOR:
          {
            /* type: host-to-device; recipient = device value: descriptor type
             * and index index: 0 or language ID; len: descriptor len; data =
             * descriptor
             */

            usbtrace(TRACE_INTDECODE(AVR_TRACEINTID_GETSETDESC), 0);
            if ((ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE)
              {
                avr_dispatchrequest(&ctrl);
              }
            else
              {
                usbtrace(TRACE_DEVERROR(AVR_TRACEERR_BADGETSETDESC), 0);
                g_usbdev.stalled = true;
              }
          }
          break;

        case USB_REQ_GETCONFIGURATION:
          {
            /* type: device-to-host; recipient = device value: 0; index: 0; len:
             * 1; data = configuration value
             */

            usbtrace(TRACE_INTDECODE(AVR_TRACEINTID_GETCONFIG), 0);
            if (g_usbdev.paddrset &&
                (ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE &&
                value == 0 &&
                index == 0 &&
                len == 1)
              {
                avr_dispatchrequest(&ctrl);
              }
            else
              {
                usbtrace(TRACE_DEVERROR(AVR_TRACEERR_BADGETCONFIG), 0);
                g_usbdev.stalled = true;
              }
          }
          break;

        case USB_REQ_SETCONFIGURATION:
          {
            /* type: host-to-device; recipient = device value: configuration
             * value index: 0; len: 0; data = none
             */

            usbtrace(TRACE_INTDECODE(AVR_TRACEINTID_SETCONFIG), 0);
            if ((ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE &&
                index == 0 &&
                len == 0)
              {
                avr_dispatchrequest(&ctrl);
              }
            else
              {
                usbtrace(TRACE_DEVERROR(AVR_TRACEERR_BADSETCONFIG), 0);
                g_usbdev.stalled = true;
              }
          }
          break;

        case USB_REQ_GETINTERFACE:
          /* type: device-to-host; recipient = interface value: 0 index:
           * interface; len: 1; data = alt interface
           */

        case USB_REQ_SETINTERFACE:
          {
            /* type: host-to-device; recipient = interface value: alternate
             * setting index: interface; len: 0; data = none
             */

            usbtrace(TRACE_INTDECODE(AVR_TRACEINTID_GETSETIF), 0);
            avr_dispatchrequest(&ctrl);
          }
          break;

        case USB_REQ_SYNCHFRAME:
          {
            /* type: device-to-host; recipient = endpoint value: 0 index:
             * endpoint; len: 2; data = frame number
             */

            usbtrace(TRACE_INTDECODE(AVR_TRACEINTID_SYNCHFRAME), 0);
          }
          break;

        default:
          {
            usbtrace(TRACE_DEVERROR(AVR_TRACEERR_INVALIDCTRLREQ), 0);
            g_usbdev.stalled = true;
          }
          break;
        }
    }

  if (g_usbdev.stalled)
    {
      usbtrace(TRACE_DEVERROR(AVR_TRACEERR_EP0SETUPSTALLED), 0);
      avr_epstall(&g_usbdev.eplist[AVR_EP0].ep, false);
      avr_epstall(&g_usbdev.eplist[AVR_EP0].ep, false);
    }

  if ((UEINTX & (1 << RXSTPI)) != 0)
    {
      UECONX |= (1 << STALLRQ);
      UEINTX &= ~(1 << RXSTPI);
    }
}

/*******************************************************************************
 * Name: avr_ep0interrupt
 *
 * Description:
 *   USB endpoint/pipe IN interrupt handler
 *
 *******************************************************************************/

static inline void avr_ep0interrupt(void)
{
  /* Check if the control endpoint endpoint is pending */

  if ((UEINT & (1 << AVR_EP0)) != 0)
    {
      /* Clear the endpoint interrupt */

      UEINT &= ~(1 << AVR_EP0);

      /* Select the control endpoint */

      UENUM = AVR_EP0;

      /* Check if the control endpoint has received a setup packet */

      if ((UEINTX & (1 << RXSTPI)) != 0)
        {
          /* It has... process the control packet */

          avr_ep0setup();
        }

      /* Handshake the endpoint setup interrupt */

      UEINTX &= ~(1 << RXSTPI);
    }
}

/*******************************************************************************
 * Name: avr_epNinterrupt
 *
 * Description:
 *   USB endpoint/pipe IN interrupt handler
 *
 *******************************************************************************/

static inline void avr_epNinterrupt(void)
{
  struct avr_ep_s *privep;
  uint8_t ueint = UEINT & (g_usbdev.epoutset | g_usbdev.epinset);
  uint8_t epno;
  uint8_t mask;

  /* Check if any endpoint interrupt is pending */

  for (epno = 1, mask = 2; epno < AVR_NENDPOINTS && ueint != 0; epno++, mask <<= 1)
    {
      /* Is there an interrupt pending on this endpoint? */

      if ((ueint & mask) != 0)
        {
          ueint &= ~mask;

          /* Select the endpoint */

          UENUM = epno;
          privep = &g_usbdev.eplist[epno];

          /* Is this an IN or an OUT interrupt? */

          if (privep->epin)
            {
              /* Clear the endpoint IN interrupt flag (TXINI) */

              UEINTX &= ~(1 << TXINI);

              /* Are IN endpoint interrupts enabled? */

              if ((UEIENX & (1 << TXINE)) != 0)
                {
                  /* Clear the endpoint interrupt */

                  UEINT &= ~(1 << epno);

                  /* Handle the IN request queue */

                  (void)avr_epINqueue(privep);
                }
            }
          else
            {
              /* Is is an OUT endpoint interrupt.  Are OUT endpoint
               * interrupts enabled?
               */

              if ((UEIENX & (1 << RXOUTE)) != 0)
                {
                  /* Clear the endpoint interrupt */

                  UEINT &= ~(1 << epno);

                  /* Handle the OUT request queue */

                  (void)avr_epOUTqueue(privep);
                }
            }
        }
    }
}

/*******************************************************************************
 * Name: avr_epinterrupt
 *
 * Description:
 *   USB endpoint/pipe interrupt handler
 *
 *******************************************************************************/

static int avr_epinterrupt(int irq, FAR void *context)
{
  usbtrace(TRACE_INTENTRY(AVR_TRACEINTID_EPINT), irq);

  /* Handle control endpoint interrupts */

  avr_ep0interrupt();

  /* Handle opther endpoint interrupts (N=1,2,..6 */

  avr_epNinterrupt();

  usbtrace(TRACE_INTEXIT(AVR_TRACEINTID_EPINT), irq);
  return OK;
}

/*******************************************************************************
 * Name: avr_genvbus
 *
 * Description:
 *   A change in VBUS has been detected.  Check if the device has been
 *   connected to or disconnected from a host.
 *
 *******************************************************************************/

static void avr_genvbus(void)
{
  bool vbus;

  usbtrace(TRACE_INTENTRY(AVR_TRACEINTID_VBUS), USBSTA);

  /* How has the VSUS signal changed? */

  vbus = ((USBSTA & (1 << VBUS)) != 0);
  if (!g_usbdev.connected && vbus)
    {
      /* We were not connected, but now we are */

      avr_usbreset();
      g_usbdev.connected = true;
    }
  else if (g_usbdev.connected && !vbus)
    {
      /* We were connected, but now we are not */
      /* Cancel all pending and queue requests */

      avr_cancelall(-ENODEV);

      /* Detach the device from the USB bus, terminating communications */

      UDCON |= (1 << DETACH);

      /* Disable the clock inputs (the ”Resume Detection” is still active).
       * This reduces the power consumption. Clear to enable the clock inputs. */

      USBCON |= (1 << FRZCLK);

      /* Shut down the USB PLL */

      PLLCSR = 0;

      /* Disable the USB pad regulator */

      UHWCON &= ~(1 << UVREGE);
      g_usbdev.connected = false;
    }
}

/*******************************************************************************
 * Name: avr_gensuspend
 *
 * Description:
 *   The USB controller has been put in suspend mode.
 *
 *******************************************************************************/

static inline void avr_gensuspend(void)
{
  usbtrace(TRACE_INTENTRY(AVR_TRACEINTID_SUSPEND), UDIEN);

  /* Notify the class driver of the suspend event */

  if (g_usbdev.driver)
    {
      CLASS_SUSPEND(g_usbdev.driver, &g_usbdev.usbdev);
    }

  /* Disable suspend event interrupts; enable wakeup event interrupt */

  UDIEN &= ~(1 << SUSPE);
  UDIEN |= (1 << WAKEUPE);

  /* Disable the clock inputs to reduce power consumption. (wakeup
   * detection is still active).
   */

  USBCON |= (1 << FRZCLK);

  /* And shut down the USB PLL */

  PLLCSR = 0;
}

/*******************************************************************************
 * Name: avr_genwakeup
 *
 * Description:
 *  Resume from suspend mode.
 *
 *******************************************************************************/

static void avr_genwakeup(void)
{
  usbtrace(TRACE_INTENTRY(AVR_TRACEINTID_WAKEUP), UDIEN);

  /* Re-enable the PLL */

  PLLCSR  =  USB_PLL_PSC;
  PLLCSR |= (1 << PLLE);
  while ((PLLCSR & (1 << PLOCK)) == 0);

  /* Re-enable USB clock inputs */

  USBCON &= ~(1 << FRZCLK);
  UDIEN  &= ~(1 << WAKEUPE);
  UDIEN  |=  (1 << SUSPE);

  /* Notify the class driver of the resume event */

  if (g_usbdev.driver)
    {
      CLASS_RESUME(g_usbdev.driver, &g_usbdev.usbdev);
    }
}

/*******************************************************************************
 * Name: avr_geneor
 *
 * Description:
 *  Handle an end-of-reset interrupt
 *
 *******************************************************************************/

static inline void avr_geneor(void)
{
  uint8_t epno;

  usbtrace(TRACE_INTENTRY(AVR_TRACEINTID_EOR), UDIEN);

  UDIEN &= ~(1 << SUSPE);
  UDIEN |= (1 << WAKEUPE);

  /* Reset all endpoints and reconfigure endpoint 0 */

  UEINT = 0;
  for (epno = 0; epno < AVR_NENDPOINTS; epno++)
    {
      struct avr_ep_s *privep = &g_usbdev.eplist[epno];
      avr_epreset(privep, -EAGAIN);
    }

  usbtrace(TRACE_EPCONFIGURE, AVR_EP0);

  /* Configure endpoint 0 */

  (void)avr_ep0configure();

  /* Reset endpoint status */

  g_usbdev.stalled = false;

  /* Enable the endpoint SETUP interrupt ISR for the control endpoint */

  UEIENX |= (1 << RXSTPE);
}

/*******************************************************************************
 * Name: avr_geninterrupt
 *
 * Description:
 *   USB general interrupt handler
 *
 *******************************************************************************/

static int avr_geninterrupt(int irq, FAR void *context)
{
  usbtrace(TRACE_INTENTRY(AVR_TRACEINTID_GENINT), irq);

  /* Check for a change in VBUS state detected */

  if ((USBINT & (1 << VBUSTI)) != 0 && (USBCON & (1 << VBUSTE)) != 0)
    {
      USBINT &= ~(1 << VBUSTI);
      avr_genvbus();
    }

  /* Check for a suspend event */

  if ((UDINT & (1 << SUSPI)) != 0 && (UDIEN & (1 << SUSPE)) != 0)
    {
      UDINT &= ~(1 << SUSPI);
      avr_gensuspend();
    }

  /* Check for a wake-up event */

  if ((UDINT & (1 << WAKEUPI)) != 0 && (UDIEN & (1 << WAKEUPE)) != 0)
    {
      UDINT &= ~(1 << WAKEUPI);
      avr_genwakeup();
    }

  /* Check for an end-of-reset, speed identification interrupt */

  if ((UDINT & (1 << EORSTI)) != 0 && (UDIEN & (1 << EORSTE)) != 0)
    {
      UDINT &= ~(1 << EORSTI);
      avr_geneor();
    }

  usbtrace(TRACE_INTEXIT(AVR_TRACEINTID_GENINT), irq);
  return OK;
}

/*******************************************************************************
 * Name: avr_epconfigure
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

static int avr_epconfigure(FAR struct usbdev_ep_s *ep,
                           FAR const struct usb_epdesc_s *desc, bool last)
{
  FAR struct avr_ep_s *privep = (FAR struct avr_ep_s *)ep;
  uint16_t maxpacket = GETUINT16(desc->mxpacketsize);
  uint8_t uecfg0x;
  uint8_t uecfg1x;
  uint8_t ueienx = 0;
  uint8_t regval;

  usbtrace(TRACE_EPCONFIGURE, ep->eplog);
  DEBUGASSERT(ep->eplog != 0 && desc->addr == ep->eplog);

  /* Configure the endpoint */

  uecfg0x = 0;
  uecfg1x = (1 << ALLOC);

  /* Handle the endpoint type */

  switch (desc->attr & USB_EP_ATTR_XFERTYPE_MASK)
    {
    case USB_EP_ATTR_XFER_CONTROL:
      uecfg0x |= AVR_EPTYPE_CTRL;
      break;

    case USB_EP_ATTR_XFER_ISOC:
      uecfg0x |= AVR_EPTYPE_ISOC;
      break;

    case USB_EP_ATTR_XFER_BULK:
      uecfg0x |= AVR_EPTYPE_BULK;
      break;

    case USB_EP_ATTR_XFER_INT:
      uecfg0x |= AVR_EPTYPE_INTR;
      break;

    default:
      usbtrace(TRACE_DEVERROR(AVR_TRACEERR_XFERTYPE), desc->attr);
      return -EINVAL;
    }

  /* Handle the endpoint direction */

  if (USB_ISEPIN(desc->addr))
    {
      DEBUGASSERT(privep->epin != 0);
      uecfg0x |= AVR_DIR_IN;
      ueienx   = (1 << RXOUTE);
    }
  else
    {
      DEBUGASSERT(privep->epin == 0);
    }

  /* Handle banking (this needs to be revisited... Always double bank?) */

  uecfg1x |= AVR_DOUBLE_BANK;

  /* Handle the maximum packet size */

  switch (maxpacket)
    {
      case 8:
        uecfg1x |= AVR_EPSIZE_8;
        break;
      case 16:
        uecfg1x |= AVR_EPSIZE_16;
        break;
      case 32:
        uecfg1x |= AVR_EPSIZE_32;
        break;
      case 64:
        uecfg1x |= AVR_EPSIZE_64;
        break;
      case 128:
        uecfg1x |= AVR_EPSIZE_128;
        break;
      case 256:
        if (ep->eplog == 1)
          {
            uecfg1x |= AVR_EPSIZE_8;
            break;
          }
      default:
        usbtrace(TRACE_DEVERROR(AVR_TRACEERR_PKTSIZE), maxpacket);
        return -EINVAL;
    }

  /* Instantiate the configuration */

  UENUM   = ep->eplog;
  UECONX |= (1 << EPEN);
  UECFG1X = 0;
  UECFG0X = uecfg0x;
  UECFG1X = uecfg1x;

  /* Check for configuration failure */

  regval = UESTA0X;
  if ((regval &  (1 << CFGOK)) == 0)
    {
      usbtrace(TRACE_DEVERROR(AVR_TRACEERR_EPCFGBAD), regval);
      return -EINVAL;
    }

  /* Save the new max packet size and reset endpoint status */

  privep->ep.maxpacket = maxpacket;
  privep->stalled      = 0;

  /* Enable interrupts as appropriate for this endpoint */

  UEIENX |= uecfg1x;
  return OK;
}

/*******************************************************************************
 * Name: avr_epdisable
 *
 * Description:
 *   The endpoint will no longer be used
 *
 *******************************************************************************/

static int avr_epdisable(FAR struct usbdev_ep_s *ep)
{
  FAR struct avr_ep_s *privep = (FAR struct avr_ep_s *)ep;
  irqstate_t flags;

#ifdef CONFIG_DEBUG
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(AVR_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif
  usbtrace(TRACE_EPDISABLE, privep->ep.eplog);

  flags = irqsave();

  /* Disable the endpoint */

  avr_epreset(privep, -ESHUTDOWN);
  g_usbdev.stalled = true;

  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Name: avr_epallocreq
 *
 * Description:
 *   Allocate an I/O request
 *
 *******************************************************************************/

static FAR struct usbdev_req_s *avr_epallocreq(FAR struct usbdev_ep_s *ep)
{
  FAR struct avr_req_s *privreq;

#ifdef CONFIG_DEBUG
  if (!ep)
    {
      usbtrace(TRACE_DEVERROR(AVR_TRACEERR_INVALIDPARMS), 0);
      return NULL;
    }
#endif
  usbtrace(TRACE_EPALLOCREQ, ((FAR struct avr_ep_s *)ep)->ep.eplog);

  privreq = (FAR struct avr_req_s *)malloc(sizeof(struct avr_req_s));
  if (!privreq)
    {
      usbtrace(TRACE_DEVERROR(AVR_TRACEERR_ALLOCFAIL), 0);
      return NULL;
    }

  memset(privreq, 0, sizeof(struct avr_req_s));
  return &privreq->req;
}

/*******************************************************************************
 * Name: avr_epfreereq
 *
 * Description:
 *   Free an I/O request
 *
 *******************************************************************************/

static void avr_epfreereq(FAR struct usbdev_ep_s *ep,
                          FAR struct usbdev_req_s *req)
{
  FAR struct avr_req_s *privreq = (FAR struct avr_req_s *)req;

#ifdef CONFIG_DEBUG
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(AVR_TRACEERR_INVALIDPARMS), 0);
      return;
    }
#endif

  usbtrace(TRACE_EPFREEREQ, ((FAR struct avr_ep_s *)ep)->ep.eplog);
  free(privreq);
}

/*******************************************************************************
 * Name: avr_epallocbuffer
 *
 * Description:
 *   Allocate an I/O buffer
 *
 *******************************************************************************/

#ifdef CONFIG_USBDEV_DMA
static void *avr_epallocbuffer(FAR struct usbdev_ep_s *ep, unsigned bytes)
{
  usbtrace(TRACE_EPALLOCBUFFER, privep->ep.eplog);

#ifdef CONFIG_USBDEV_DMAMEMORY
  return usbdev_dma_alloc(bytes);
#else
  return malloc(bytes);
#endif
}
#endif

/*******************************************************************************
 * Name: avr_epfreebuffer
 *
 * Description:
 *   Free an I/O buffer
 *
 *******************************************************************************/

#ifdef CONFIG_USBDEV_DMA
static void avr_epfreebuffer(FAR struct usbdev_ep_s *ep, FAR void *buf)
{
  usbtrace(TRACE_EPFREEBUFFER, privep->ep.eplog);

#ifdef CONFIG_USBDEV_DMAMEMORY
  usbdev_dma_free(buf);
#else
  free(buf);
#endif
}
#endif

/*******************************************************************************
 * Name: avr_epsubmit
 *
 * Description:
 *   Submit an I/O request to the endpoint
 *
 *******************************************************************************/

static int avr_epsubmit(FAR struct usbdev_ep_s *ep,
                        FAR struct usbdev_req_s *req)
{
  FAR struct avr_req_s *privreq = (FAR struct avr_req_s *)req;
  FAR struct avr_ep_s *privep = (FAR struct avr_ep_s *)ep;
  irqstate_t flags;
  int ret = OK;

#ifdef CONFIG_DEBUG
  if (!req || !req->callback || !req->buf || !ep)
    {
      usbtrace(TRACE_DEVERROR(AVR_TRACEERR_INVALIDPARMS), 0);
      ullvdbg("req=%p callback=%p buf=%p ep=%p\n",
              req, req->callback, req->buf, ep);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPSUBMIT, privep->ep.eplog);

  if (!g_usbdev.driver || g_usbdev.usbdev.speed == USB_SPEED_UNKNOWN)
    {
      usbtrace(TRACE_DEVERROR(AVR_TRACEERR_NOTCONFIGURED), g_usbdev.usbdev.speed);
      return -ESHUTDOWN;
    }

  /* Handle the request from the class driver */

  req->result = -EINPROGRESS;
  req->xfrd   = 0;

  /* Disable Interrupts */

  flags = irqsave();

  /* If we are stalled, then drop all requests on the floor */

  if (g_usbdev.stalled)
    {
      ret = -EBUSY;
    }

  /* Ignore any attempt to enqueue a zero length packet */

  else if (privreq->req.len == 0)
    {
      usbtrace(TRACE_DEVERROR(AVR_TRACEERR_EPNULLPACKET), 0);
      ret = -EINVAL;
    }
  else
    {
      /* Add the new request to the request queue for the endpoint */

      avr_rqenqueue(privep, privreq);

      /* Some special operations have to be performed for IN requests.  For
       * these, we may have to initiate the next transfer.
       */

      if (privep->epin)
        {
          /* It is an IN transfer */

          usbtrace(TRACE_INREQQUEUED(privep->ep.eplog), privreq->req.len);

          /* Is there an IN transfer in progress (waiting for the FIFO)? If
           * not and if the FIFO is available now, then start the next
           * IN transfer.
           */

          if (!privep->pending && avr_fifoready(AVR_TIMEOUT_NONE) == OK)
            {
              /* No, then start the next IN transfer */

              ret = avr_epINqueue(privep);
            }
        }
      else
        {
          /* It is an OUT transfer */

          usbtrace(TRACE_OUTREQQUEUED(privep->ep.eplog), privreq->req.len);

          /* If there is something avaible in the fifo now, then go get it */

          if (avr_fifoready(AVR_TIMEOUT_NONE) == OK);
            {
              ret = avr_epOUTqueue(privep);
            }
        }
    }

  irqrestore(flags);
  return ret;
}

/*******************************************************************************
 * Name: avr_epcancel
 *
 * Description:
 *   Cancel an I/O request previously sent to an endpoint
 *
 *******************************************************************************/

static int avr_epcancel(FAR struct usbdev_ep_s *ep,
                        FAR struct usbdev_req_s *req)
{
  FAR struct avr_ep_s *privep = (FAR struct avr_ep_s *)ep;
  irqstate_t flags;

#ifdef CONFIG_DEBUG
  if (!ep || !req)
    {
      usbtrace(TRACE_DEVERROR(AVR_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  usbtrace(TRACE_EPCANCEL, privep->ep.eplog);

  /* FIXME: if the request is the first, then we need to flush the EP otherwise 
   * just remove it from the list but ... all other implementations cancel all 
   * requests ... */

  flags = irqsave();
  avr_cancelrequests(privep, -ESHUTDOWN);
  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Name: avr_epstall
 *
 * Description:
 *   Stall or resume and endpoint
 *
 *******************************************************************************/

static int avr_epstall(FAR struct usbdev_ep_s *ep, bool resume)
{
  irqstate_t flags;

  /* STALL or RESUME the endpoint */

  flags = irqsave();
  if (resume)
    {
      /* Clear stall and reset the data toggle */

      UECONX       |= (1 << STALLRQC);
      UERST         = (1 << ep->eplog);
      UERST         = 0;
      UECONX       |= (1 << RSTDT);
      g_usbdev.stalled = false;
    }
  else
    {
      UECONX       |= (1 << STALLRQ);
      g_usbdev.stalled = true;
    }
  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Device operations
 *******************************************************************************/

/*******************************************************************************
 * Name: avr_allocep
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

static FAR struct usbdev_ep_s *avr_allocep(FAR struct usbdev_s *dev,
                                           uint8_t epno, bool in,
                                           uint8_t eptype)
{
  FAR struct avr_ep_s *privep;
  irqstate_t flags;
  uint8_t epset = g_usbdev.epavail;
  uint8_t epmask;
  uint8_t epndx = 0;

  usbtrace(TRACE_DEVALLOCEP, epno);

  /* Ignore any direction bits in the logical address */

  epno = USB_EPNO(epno);

  /* A logical address of 0 means that any endpoint will do */

  if (epno > 0)
    {
      /* Otherwise, we will return the endpoint structure only for the
       * requested 'logical' endpoint.
       */

#ifdef CONFIG_DEBUG
      if (epno >= AVR_NENDPOINTS)
        {
          usbtrace(TRACE_DEVERROR(AVR_TRACEERR_BADEPNO), (uint16_t)epno);
          return NULL;
        }
#endif

      /* Convert the logical address to a physical OUT endpoint address and
       * remove all of the candidate endpoints from the bitset except for the
       * the IN/OUT pair for this logical address. */

      epset &= (1 << epno);
    }

  /* Are any endpoints available? */

  if (epset)
    {
      /* Yes.. now see if any of the request endpoints are available */

      flags = irqsave();

      /* Select the lowest bit in the set of matching, available endpoints */

      for (epndx = 1; epndx < AVR_NENDPOINTS; epndx++)
        {
          epmask = 1 << epndx;
          if ((epset & epmask) != 0)
            {
               /* Initialize the endpoint structure */

               privep           = &g_usbdev.eplist[epndx];
               memset(privep, 0, sizeof(struct avr_ep_s));

               privep->ep.ops       = &g_epops;
               privep->ep.eplog     = epndx;
               privep->ep.maxpacket = (epndx == 1) ? 256 : 64;

               /* Mark the IN/OUT endpoint no longer available */

               g_usbdev.epavail &= ~epmask;
               if (in)
                 {
                   g_usbdev.epinset |= epmask;
                   privep->epin      = 1;
                 }
               else
                 {
                   g_usbdev.epoutset |= epmask;
                   privep->epin       = 0;
                 }

               /* And return the pointer to the standard endpoint structure */

               irqrestore(flags);
               return &privep->ep;
            }
        }

      /* Shouldn't get here */

      irqrestore(flags);
    }

  usbtrace(TRACE_DEVERROR(AVR_TRACEERR_NOEP), (uint16_t) epno);
  return NULL;
}

/*******************************************************************************
 * Name: avr_freeep
 *
 * Description:
 *   Free the previously allocated endpoint
 *
 *******************************************************************************/

static void avr_freeep(FAR struct usbdev_s *dev, FAR struct usbdev_ep_s *ep)
{
  FAR struct avr_ep_s *privep = (FAR struct avr_ep_s *)ep;
  irqstate_t flags;
  uint8_t epmask;

  usbtrace(TRACE_DEVFREEEP, (uint16_t) privep->ep.eplog);

  /* Mark the endpoint as available */

  flags = irqsave();
  epmask = (1 << privep->ep.eplog);
  g_usbdev.epavail  |= epmask;
  g_usbdev.epinset  &= ~epmask;
  g_usbdev.epoutset &= ~epmask;
  irqrestore(flags);
}

/*******************************************************************************
 * Name: avr_getframe
 *
 * Description:
 *   Returns the current frame number
 *
 *******************************************************************************/

static int avr_getframe(struct usbdev_s *dev)
{
  /* Return the last frame number detected by the hardware */

  usbtrace(TRACE_DEVGETFRAME, 0);
  return (int)UDFNUMH << 8 | (int)UDFNUML;
}

/*******************************************************************************
 * Name: avr_wakeup
 *
 * Description:
 *   Tries to wake up the host connected to this device
 *
 *******************************************************************************/

static int avr_wakeup(struct usbdev_s *dev)
{
  irqstate_t flags;

  usbtrace(TRACE_DEVWAKEUP, 0);

  flags = irqsave();
  avr_genwakeup();
  irqrestore(flags);
  return OK;
}

/*******************************************************************************
 * Name: avr_selfpowered
 *
 * Description:
 *   Sets/clears the device selfpowered feature 
 *
 *******************************************************************************/

static int avr_selfpowered(struct usbdev_s *dev, bool selfpowered)
{
  usbtrace(TRACE_DEVSELFPOWERED, (uint16_t) selfpowered);

#ifdef CONFIG_DEBUG
  if (!dev)
    {
      usbtrace(TRACE_DEVERROR(AVR_TRACEERR_INVALIDPARMS), 0);
      return -ENODEV;
    }
#endif

  g_usbdev.selfpowered = selfpowered;
  return OK;
}

/*******************************************************************************
 * Name: avr_pullup
 *
 * Description:
 *   Software-controlled connect to/disconnect from USB host
 *
 *******************************************************************************/

static int avr_pullup(struct usbdev_s *dev, bool enable)
{
  usbtrace(TRACE_DEVPULLUP, (uint16_t) enable);
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
  usbtrace(TRACE_DEVINIT, 0);

  /* Initialize the device state structure */

  memset(&g_usbdev, 0, sizeof(struct avr_usbdev_s));
  g_usbdev.usbdev.ops = &g_devops;
  g_usbdev.usbdev.ep0 = &g_usbdev.eplist[AVR_EP0].ep;
  g_usbdev.epavail    = AVR_ALL_EPS & ~(1 << AVR_EP0);

  /* Attach USB controller general interrupt handler */

  if (irq_attach(AT90USB_IRQ_USBGEN, avr_geninterrupt) != 0)
    {
      usbtrace(TRACE_DEVERROR(AVR_TRACEERR_IRQREGISTRATION), AT90USB_IRQ_USBGEN);
      goto errout;
    }

  /* Attach USB controller endpoint/pipe interrupt handler */

  if (irq_attach(AT90USB_IRQ_USBEP, avr_epinterrupt) != 0)
    {
      usbtrace(TRACE_DEVERROR(AVR_TRACEERR_IRQREGISTRATION), AT90USB_IRQ_USBEP);
      goto errout;
    }

  /* Shutdown the USB interface to put it in a known initial state */

  avr_usbshutdown();

  /* Select USB device mode */

  UHWCON |= (1 << UIMOD);

  /* Reset the interface to force re-enumeration (the reset operation
   * enables interrupts.
   */

  avr_usbreset();

  /* Set the VBUS pad */

  USBCON |= (1 << OTGPADE);

  /* Disconnect device */

  avr_pullup(&g_usbdev.usbdev, false);
  return;

errout:
  up_usbuninitialize();
}

/*******************************************************************************
 * Name: up_usbuninitialize
 *******************************************************************************/

void up_usbuninitialize(void)
{
  irqstate_t flags;

  usbtrace(TRACE_DEVUNINIT, 0);

  if (g_usbdev.driver)
    {
      usbtrace(TRACE_DEVERROR(AVR_TRACEERR_DRIVERREGISTERED), 0);
      usbdev_unregister(g_usbdev.driver);
    }

  /* Disconnect device */

  flags = irqsave();
  avr_pullup(&g_usbdev.usbdev, false);
  g_usbdev.usbdev.speed = USB_SPEED_UNKNOWN;

  /* Detach IRQs */

  irq_detach(AT90USB_IRQ_USBGEN);
  irq_detach(AT90USB_IRQ_USBEP);

  /* Shutdown the USB controller hardware */

  avr_usbshutdown();
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
      usbtrace(TRACE_DEVERROR(AVR_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }

  if (g_usbdev.driver)
    {
      usbtrace(TRACE_DEVERROR(AVR_TRACEERR_DRIVER), 0);
      return -EBUSY;
    }
#endif

  /* First hook up the driver */

  g_usbdev.driver = driver;

  /* Then bind the class driver */

  ret = CLASS_BIND(driver, &g_usbdev.usbdev);
  if (ret)
    {
      usbtrace(TRACE_DEVERROR(AVR_TRACEERR_BINDFAILED), (uint16_t) - ret);
      g_usbdev.driver = NULL;
    }
  else
    {
      /* FIXME: nothing seems to call DEV_CONNECT(), but we need to set the RS
       * bit to enable the controller.  It kind of makes sense to do this
       * after the class has bound to us... GEN: This bug is really in the
       * class driver.  It should make the soft connect when it is ready to be
       * enumerated.  I have added that logic to the class drivers but left
       * this logic here. */

      avr_pullup(&g_usbdev.usbdev, true);
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
      usbtrace(TRACE_DEVERROR(AVR_TRACEERR_INVALIDPARMS), 0);
      return -EINVAL;
    }
#endif

  /* Unbind the class driver */

  CLASS_UNBIND(driver, &g_usbdev.usbdev);

  /* Unhook the driver */

  g_usbdev.driver = NULL;
  return OK;
}

/*******************************************************************************
 * Name: avr_pollvbus
 *
 * Description:
 *   Sample VBUS to see if there are changes in our connection status.  There
 *   is actually an interrupt to signal this case so it should not be necessary 
 *   to poll our connection status.  However, on certain "noisy" systems, VBUS
 *   may bounce and provide inaccurate information in the interrupt handler
 *   (especially if a relay is used to switch VBUS!).  This poll is, then,
 *   simply a failsafe to assure that VBUS connection events are never missed.
 *
 *******************************************************************************/

#ifdef CONFIG_USB_NOISYVBUS
 void avr_pollvbus(void)
{
  irqstate_t flags;

  flags = irqsave();
  avr_genvbus();
  irqrestore(flags);
}
#endif