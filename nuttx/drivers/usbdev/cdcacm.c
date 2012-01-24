/****************************************************************************
 * drivers/usbdev/cdcacm.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <queue.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/serial.h>

#include <nuttx/usb/usb.h>
#include <nuttx/usb/cdc.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/cdc_serial.h>
#include <nuttx/usb/usbdev_trace.h>

#include "cdcacm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Container to support a list of requests */

struct cdcser_req_s
{
  FAR struct cdcser_req_s *flink;      /* Implements a singly linked list */
  FAR struct usbdev_req_s *req;        /* The contained request */
};

/* This structure describes the internal state of the driver */

struct cdcser_dev_s
{
  FAR struct uart_dev_s    serdev;     /* Serial device structure */
  FAR struct usbdev_s     *usbdev;     /* usbdev driver pointer */

  uint8_t config;                      /* Configuration number */
  uint8_t nwrq;                        /* Number of queue write requests (in reqlist)*/
  uint8_t nrdq;                        /* Number of queue read requests (in epbulkout) */
  bool    rxenabled;                   /* true: UART RX "interrupts" enabled */
  int16_t rxhead;                      /* Working head; used when rx int disabled */

  uint8_t                  ctrlline;   /* Buffered control line state */
  struct cdc_linecoding_s  linecoding; /* Buffered line status */
  cdcser_callback_t        callback;   /* Serial event callback function */

  FAR struct usbdev_ep_s  *epintin;    /* Interrupt IN endpoint structure */
  FAR struct usbdev_ep_s  *epbulkin;   /* Bulk IN endpoint structure */
  FAR struct usbdev_ep_s  *epbulkout;  /* Bulk OUT endpoint structure */
  FAR struct usbdev_req_s *ctrlreq;    /* Control request */
  struct sq_queue_s        reqlist;    /* List of write request containers */

  /* Pre-allocated write request containers.  The write requests will
   * be linked in a free list (reqlist), and used to send requests to
   * EPBULKIN; Read requests will be queued in the EBULKOUT.
   */

  struct cdcser_req_s wrreqs[CONFIG_CDCSER_NWRREQS];
  struct cdcser_req_s rdreqs[CONFIG_CDCSER_NWRREQS];

  /* Serial I/O buffers */

  char rxbuffer[CONFIG_CDCSER_RXBUFSIZE];
  char txbuffer[CONFIG_CDCSER_TXBUFSIZE];
};

/* The internal version of the class driver */

struct cdcser_driver_s
{
  struct usbdevclass_driver_s drvr;
  FAR struct cdcser_dev_s     *dev;
};

/* This is what is allocated */

struct cdcser_alloc_s
{
  struct cdcser_dev_s    dev;
  struct cdcser_driver_s drvr;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Transfer helpers *********************************************************/

static uint16_t cdcser_fillrequest(FAR struct cdcser_dev_s *priv,
                 uint8_t *reqbuf, uint16_t reqlen);
static int     cdcser_sndpacket(FAR struct cdcser_dev_s *priv);
static inline int cdcser_recvpacket(FAR struct cdcser_dev_s *priv,
                 uint8_t *reqbuf, uint16_t reqlen);

/* Request helpers *********************************************************/

static struct usbdev_req_s *cdcser_allocreq(FAR struct usbdev_ep_s *ep,
                 uint16_t len);
static void    cdcser_freereq(FAR struct usbdev_ep_s *ep,
                 FAR struct usbdev_req_s *req);

/* Configuration ***********************************************************/

static void    cdcser_resetconfig(FAR struct cdcser_dev_s *priv);
#ifdef CONFIG_USBDEV_DUALSPEED
static int     cdcser_epconfigure(FAR struct usbdev_ep_s *ep,
                 enum cdcser_epdesc_e epid, uint16_t mxpacket, bool last);
#endif
#ifndef CONFIG_CDCSER_COMPOSITE
static int     cdcser_setconfig(FAR struct cdcser_dev_s *priv,
                 uint8_t config);
#endif

/* Completion event handlers ***********************************************/

static void    cdcser_ep0incomplete(FAR struct usbdev_ep_s *ep,
                 FAR struct usbdev_req_s *req);
static void    cdcser_rdcomplete(FAR struct usbdev_ep_s *ep,
                 FAR struct usbdev_req_s *req);
static void    cdcser_wrcomplete(FAR struct usbdev_ep_s *ep,
                 FAR struct usbdev_req_s *req);

/* USB class device ********************************************************/

static int     cdcser_bind(FAR struct usbdev_s *dev,
                 FAR struct usbdevclass_driver_s *driver);
static void    cdcser_unbind(FAR struct usbdev_s *dev);
static int     cdcser_setup(FAR struct usbdev_s *dev,
                 const struct usb_ctrlreq_s *ctrl);
static void    cdcser_disconnect(FAR struct usbdev_s *dev);

/* UART Operationt **********************************************************/

static int     cdcuart_setup(FAR struct uart_dev_s *dev);
static void    cdcuart_shutdown(FAR struct uart_dev_s *dev);
static int     cdcuart_attach(FAR struct uart_dev_s *dev);
static void    cdcuart_detach(FAR struct uart_dev_s *dev);
static int     cdcuart_ioctl(FAR struct file *filep,int cmd,unsigned long arg);
static void    cdcuart_rxint(FAR struct uart_dev_s *dev, bool enable);
static void    cdcuart_txint(FAR struct uart_dev_s *dev, bool enable);
static bool    cdcuart_txempty(FAR struct uart_dev_s *dev);

/****************************************************************************
 * Private Variables
 ****************************************************************************/
/* USB class device *********************************************************/

static const struct usbdevclass_driverops_s g_driverops =
{
  cdcser_bind,          /* bind */
  cdcser_unbind,        /* unbind */
  cdcser_setup,         /* setup */
  cdcser_disconnect,    /* disconnect */
  NULL,                 /* suspend */
  NULL,                 /* resume */
};

/* Serial port **************************************************************/

static const struct uart_ops_s g_uartops =
{
  cdcuart_setup,        /* setup */
  cdcuart_shutdown,     /* shutdown */
  cdcuart_attach,       /* attach */
  cdcuart_detach,       /* detach */
  cdcuart_ioctl,        /* ioctl */
  NULL,                 /* receive */
  cdcuart_rxint,        /* rxinit */
  NULL,                 /* rxavailable */
  NULL,                 /* send */
  cdcuart_txint,        /* txinit */
  NULL,                 /* txready */
  cdcuart_txempty       /* txempty */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cdcser_fillrequest
 *
 * Description:
 *   If there is data to send it is copied to the given buffer.  Called
 *   either to initiate the first write operation, or from the completion
 *   interrupt handler service consecutive write operations.
 *
 * NOTE: The USB serial driver does not use the serial drivers
 *   uart_xmitchars() API.  That logic is essentially duplicated here because
 *   unlike UART hardware, we need to be able to handle writes not byte-by-byte,
 *   but packet-by-packet. Unfortunately, that decision also exposes some
 *   internals of the serial driver in the following.
 *
 ****************************************************************************/

static uint16_t cdcser_fillrequest(FAR struct cdcser_dev_s *priv, uint8_t *reqbuf,
                                   uint16_t reqlen)
{
  FAR uart_dev_t *serdev = &priv->serdev;
  FAR struct uart_buffer_s *xmit = &serdev->xmit;
  irqstate_t flags;
  uint16_t nbytes = 0;

  /* Disable interrupts */

  flags = irqsave();

  /* Transfer bytes while we have bytes available and there is room in the request */

  while (xmit->head != xmit->tail && nbytes < reqlen)
    {
      *reqbuf++ = xmit->buffer[xmit->tail];
      nbytes++;

      /* Increment the tail pointer */

      if (++(xmit->tail) >= xmit->size)
        {
          xmit->tail = 0;
        }
    }

  /* When all of the characters have been sent from the buffer
   * disable the "TX interrupt".
   */

  if (xmit->head == xmit->tail)
    {
      uart_disabletxint(serdev);
    }

  /* If any bytes were removed from the buffer, inform any waiters
   * there there is space available.
   */

  if (nbytes)
    {
      uart_datasent(serdev);
    }

  irqrestore(flags);
  return nbytes;
}

/****************************************************************************
 * Name: cdcser_sndpacket
 *
 * Description:
 *   This function obtains write requests, transfers the TX data into the
 *   request, and submits the requests to the USB controller.  This continues
 *   untils either (1) there are no further packets available, or (2) thre is
 *   no further data to send.
 *
 ****************************************************************************/

static int cdcser_sndpacket(FAR struct cdcser_dev_s *priv)
{
  FAR struct usbdev_ep_s *ep;
  FAR struct usbdev_req_s *req;
  FAR struct cdcser_req_s *reqcontainer;
  irqstate_t flags;
  int len;
  int ret = OK;

#ifdef CONFIG_DEBUG
  if (priv == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return -ENODEV;
    }
#endif

  flags = irqsave();

  /* Use our IN endpoint for the transfer */

  ep = priv->epbulkin;

  /* Loop until either (1) we run out or write requests, or (2) cdcser_fillrequest()
   * is unable to fill the request with data (i.e., untilthere is no more data
   * to be sent).
   */

  uvdbg("head=%d tail=%d nwrq=%d empty=%d\n",
        priv->serdev.xmit.head, priv->serdev.xmit.tail,
        priv->nwrq, sq_empty(&priv->reqlist));

  while (!sq_empty(&priv->reqlist))
    {
      /* Peek at the request in the container at the head of the list */

      reqcontainer = (struct cdcser_req_s *)sq_peek(&priv->reqlist);
      req          = reqcontainer->req;

      /* Fill the request with serial TX data */

      len = cdcser_fillrequest(priv, req->buf, req->len);
      if (len > 0)
        {
          /* Remove the empty container from the request list */

          (void)sq_remfirst(&priv->reqlist);
          priv->nwrq--;

          /* Then submit the request to the endpoint */

          req->len     = len;
          req->priv    = reqcontainer;
          req->flags   = USBDEV_REQFLAGS_NULLPKT;
          ret          = EP_SUBMIT(ep, req);
          if (ret != OK)
            {
              usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_SUBMITFAIL), (uint16_t)-ret);
              break;
            }
        }
      else
        {
          break;
        }
    }

  irqrestore(flags);
  return ret;
}

/****************************************************************************
 * Name: cdcser_recvpacket
 *
 * Description:
 *   A normal completion event was received by the read completion handler
 *   at the interrupt level (with interrupts disabled).  This function handles
 *   the USB packet and provides the received data to the uart RX buffer.
 *
 * Assumptions:
 *   Called from the USB interrupt handler with interrupts disabled.
 *
 ****************************************************************************/

static inline int cdcser_recvpacket(FAR struct cdcser_dev_s *priv,
                                    uint8_t *reqbuf, uint16_t reqlen)
{
  FAR uart_dev_t *serdev = &priv->serdev;
  FAR struct uart_buffer_s *recv = &serdev->recv;
  uint16_t currhead;
  uint16_t nexthead;
  uint16_t nbytes = 0;

  uvdbg("head=%d tail=%d nrdq=%d reqlen=%d\n",
        priv->serdev.recv.head, priv->serdev.recv.tail, priv->nrdq, reqlen);

  /* Get the next head index. During the time that RX interrupts are disabled, the
   * the serial driver will be extracting data from the circular buffer and modifying
   * recv.tail.  During this time, we should avoid modifying recv.head; Instead we will
   * use a shadow copy of the index.  When interrupts are restored, the real recv.head
   * will be updated with this indes.
   */

  if (priv->rxenabled)
    {
      currhead = recv->head;
    }
  else
    {
      currhead = priv->rxhead;
    }

  /* Pre-calculate the head index and check for wrap around.  We need to do this
   * so that we can determine if the circular buffer will overrun BEFORE we
   * overrun the buffer!
   */

  nexthead = currhead + 1;
  if (nexthead >= recv->size)
    {
      nexthead = 0;
    }

  /* Then copy data into the RX buffer until either: (1) all of the data has been
   * copied, or (2) the RX buffer is full.  NOTE:  If the RX buffer becomes full,
   * then we have overrun the serial driver and data will be lost.
   */

  while (nexthead != recv->tail && nbytes < reqlen)
    {
      /* Copy one byte to the head of the circular RX buffer */

      recv->buffer[currhead] = *reqbuf++;

      /* Update counts and indices */

      currhead = nexthead;
      nbytes++;

      /* Increment the head index and check for wrap around */

      nexthead = currhead + 1;
      if (nexthead >= recv->size)
        {
          nexthead = 0;
        }
    }

  /* Write back the head pointer using the shadow index if RX "interrupts"
   * are disabled.
   */

  if (priv->rxenabled)
    {
      recv->head = currhead;
    }
  else
    {
      priv->rxhead = currhead;
    }

  /* If data was added to the incoming serial buffer, then wake up any
   * threads is waiting for incoming data. If we are running in an interrupt
   * handler, then the serial driver will not run until the interrupt handler
   * returns.
   */

  if (priv->rxenabled && nbytes > 0)
    {
      uart_datareceived(serdev);
    }

  /* Return an error if the entire packet could not be transferred */

  if (nbytes < reqlen)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RXOVERRUN), 0);
      return -ENOSPC;
    }
  return OK;
}

/****************************************************************************
 * Name: cdcser_allocreq
 *
 * Description:
 *   Allocate a request instance along with its buffer
 *
 ****************************************************************************/

static struct usbdev_req_s *cdcser_allocreq(FAR struct usbdev_ep_s *ep,
                                            uint16_t len)
{
  FAR struct usbdev_req_s *req;

  req = EP_ALLOCREQ(ep);
  if (req != NULL)
    {
      req->len = len;
      req->buf = EP_ALLOCBUFFER(ep, len);
      if (!req->buf)
        {
          EP_FREEREQ(ep, req);
          req = NULL;
        }
    }
  return req;
}

/****************************************************************************
 * Name: cdcser_freereq
 *
 * Description:
 *   Free a request instance along with its buffer
 *
 ****************************************************************************/

static void cdcser_freereq(FAR struct usbdev_ep_s *ep,
                           FAR struct usbdev_req_s *req)
{
  if (ep != NULL && req != NULL)
    {
      if (req->buf != NULL)
        {
          EP_FREEBUFFER(ep, req->buf);
        }
      EP_FREEREQ(ep, req);
    }
}

/****************************************************************************
 * Name: cdcser_resetconfig
 *
 * Description:
 *   Mark the device as not configured and disable all endpoints.
 *
 ****************************************************************************/

static void cdcser_resetconfig(FAR struct cdcser_dev_s *priv)
{
  /* Are we configured? */

  if (priv->config != CDCSER_CONFIGIDNONE)
    {
      /* Yes.. but not anymore */

      priv->config = CDCSER_CONFIGIDNONE;

      /* Disable endpoints.  This should force completion of all pending
       * transfers.
       */

      EP_DISABLE(priv->epintin);
      EP_DISABLE(priv->epbulkin);
      EP_DISABLE(priv->epbulkout);
    }
}

/****************************************************************************
 * Name: cdcser_epconfigure
 *
 * Description:
 *   Configure one endpoint.
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DUALSPEED
static int cdcser_epconfigure(FAR struct usbdev_ep_s *ep,
                              enum cdcser_epdesc_e epid, uint16_t mxpacket,
                              bool last)
{
  struct usb_epdesc_s epdesc;
  cdcser_mkepdesc(epid, mxpacket, &epdesc);
  return EP_CONFIGURE(ep, &epdesc, last);
}
#endif

/****************************************************************************
 * Name: cdcser_setconfig
 *
 * Description:
 *   Set the device configuration by allocating and configuring endpoints and
 *   by allocating and queue read and write requests.
 *
 ****************************************************************************/

#ifndef CONFIG_CDCSER_COMPOSITE
static int cdcser_setconfig(FAR struct cdcser_dev_s *priv, uint8_t config)
{
  FAR struct usbdev_req_s *req;
  int i;
  int ret = 0;

#if CONFIG_DEBUG
  if (priv == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return -EIO;
    }
#endif

  if (config == priv->config)
    {
      /* Already configured -- Do nothing */

      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALREADYCONFIGURED), 0);
      return 0;
    }

  /* Discard the previous configuration data */

  cdcser_resetconfig(priv);

  /* Was this a request to simply discard the current configuration? */

  if (config == CDCSER_CONFIGIDNONE)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_CONFIGNONE), 0);
      return 0;
    }

  /* We only accept one configuration */

  if (config != CDCSER_CONFIGID)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_CONFIGIDBAD), 0);
      return -EINVAL;
    }

  /* Configure the IN interrupt endpoint */

#ifdef CONFIG_USBDEV_DUALSPEED
  if (priv->usbdev->speed == USB_SPEED_HIGH)
    {
      ret = cdcser_epconfigure(priv->epintin, CDCSER_EPINTIN,
                               CONFIG_CDCSER_EPINTIN_HSSIZE, false);
    }
  else
#endif
    {
      ret = EP_CONFIGURE(priv->epintin,
                         cdcser_getepdesc(CDCSER_EPINTIN), false);
    }

  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPINTINCONFIGFAIL), 0);
      goto errout;
    }
  priv->epintin->priv = priv;

  /* Configure the IN bulk endpoint */

#ifdef CONFIG_USBDEV_DUALSPEED
  if (priv->usbdev->speed == USB_SPEED_HIGH)
    {
      ret = cdcser_epconfigure(priv->epbulkin, CDCSER_EPBULKIN,
                               CONFIG_CDCSER_EPBULKIN_HSSIZE, false);
    }
  else
#endif
    {
      ret = EP_CONFIGURE(priv->epbulkin,
                         cdcser_getepdesc(CDCSER_EPBULKIN), false);
    }

  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKINCONFIGFAIL), 0);
      goto errout;
    }

  priv->epbulkin->priv = priv;

  /* Configure the OUT bulk endpoint */

#ifdef CONFIG_USBDEV_DUALSPEED
  if (priv->usbdev->speed == USB_SPEED_HIGH)
    {
      ret = cdcser_epconfigure(priv->epbulkout, CDCSER_EPBULKOUT,
                               CONFIG_CDCSER_EPBULKOUT_HSSIZE, true);
    }
  else
#endif
    {
      ret = EP_CONFIGURE(priv->epbulkout,
                         cdcser_getepdesc(CDCSER_EPBULKOUT), true);
    }

  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKOUTCONFIGFAIL), 0);
      goto errout;
    }

  priv->epbulkout->priv = priv;

  /* Queue read requests in the bulk OUT endpoint */

  DEBUGASSERT(priv->nrdq == 0);
  for (i = 0; i < CONFIG_CDCSER_NRDREQS; i++)
    {
      req           = priv->rdreqs[i].req;
      req->callback = cdcser_rdcomplete;
      ret           = EP_SUBMIT(priv->epbulkout, req);
      if (ret != OK)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSUBMIT), (uint16_t)-ret);
          goto errout;
        }
      priv->nrdq++;
    }

  priv->config = config;
  return OK;

errout:
  cdcser_resetconfig(priv);
  return ret;
}
#endif

/****************************************************************************
 * Name: cdcser_ep0incomplete
 *
 * Description:
 *   Handle completion of EP0 control operations
 *
 ****************************************************************************/

static void cdcser_ep0incomplete(FAR struct usbdev_ep_s *ep,
                                   FAR struct usbdev_req_s *req)
{
  if (req->result || req->xfrd != req->len)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_REQRESULT), (uint16_t)-req->result);
    }
}

/****************************************************************************
 * Name: cdcser_rdcomplete
 *
 * Description:
 *   Handle completion of read request on the bulk OUT endpoint.  This
 *   is handled like the receipt of serial data on the "UART"
 *
 ****************************************************************************/

static void cdcser_rdcomplete(FAR struct usbdev_ep_s *ep,
                              FAR struct usbdev_req_s *req)
{
  FAR struct cdcser_dev_s *priv;
  irqstate_t flags;
  int ret;

  /* Sanity check */

#ifdef CONFIG_DEBUG
  if (!ep || !ep->priv || !req)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
     }
#endif

  /* Extract references to private data */

  priv = (FAR struct cdcser_dev_s*)ep->priv;

  /* Process the received data unless this is some unusual condition */

  flags = irqsave();
  switch (req->result)
    {
    case 0: /* Normal completion */
      usbtrace(TRACE_CLASSRDCOMPLETE, priv->nrdq);
      cdcser_recvpacket(priv, req->buf, req->xfrd);
      break;

    case -ESHUTDOWN: /* Disconnection */
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSHUTDOWN), 0);
      priv->nrdq--;
      irqrestore(flags);
      return;

    default: /* Some other error occurred */
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDUNEXPECTED), (uint16_t)-req->result);
      break;
    };

  /* Requeue the read request */

#ifdef CONFIG_CDCSER_BULKREQLEN
  req->len = max(CONFIG_CDCSER_BULKREQLEN, ep->maxpacket);
#else
  req->len = ep->maxpacket;
#endif

  ret      = EP_SUBMIT(ep, req);
  if (ret != OK)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDSUBMIT), (uint16_t)-req->result);
    }
  irqrestore(flags);
}

/****************************************************************************
 * Name: cdcser_wrcomplete
 *
 * Description:
 *   Handle completion of write request.  This function probably executes
 *   in the context of an interrupt handler.
 *
 ****************************************************************************/

static void cdcser_wrcomplete(FAR struct usbdev_ep_s *ep,
                              FAR struct usbdev_req_s *req)
{
  FAR struct cdcser_dev_s *priv;
  FAR struct cdcser_req_s *reqcontainer;
  irqstate_t flags;

  /* Sanity check */

#ifdef CONFIG_DEBUG
  if (!ep || !ep->priv || !req || !req->priv)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
     }
#endif

  /* Extract references to our private data */

  priv         = (FAR struct cdcser_dev_s *)ep->priv;
  reqcontainer = (FAR struct cdcser_req_s *)req->priv;

  /* Return the write request to the free list */

  flags = irqsave();
  sq_addlast((sq_entry_t*)reqcontainer, &priv->reqlist);
  priv->nwrq++;
  irqrestore(flags);

  /* Send the next packet unless this was some unusual termination
   * condition
   */

  switch (req->result)
    {
    case OK: /* Normal completion */
      usbtrace(TRACE_CLASSWRCOMPLETE, priv->nwrq);
      cdcser_sndpacket(priv);
      break;

    case -ESHUTDOWN: /* Disconnection */
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_WRSHUTDOWN), priv->nwrq);
      break;

    default: /* Some other error occurred */
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_WRUNEXPECTED), (uint16_t)-req->result);
      break;
    }
}

/****************************************************************************
 * USB Class Driver Methods
 ****************************************************************************/

/****************************************************************************
 * Name: cdcser_bind
 *
 * Description:
 *   Invoked when the driver is bound to a USB device driver
 *
 ****************************************************************************/

static int cdcser_bind(FAR struct usbdev_s *dev, FAR struct usbdevclass_driver_s *driver)
{
  FAR struct cdcser_dev_s *priv = ((struct cdcser_driver_s*)driver)->dev;
  FAR struct cdcser_req_s *reqcontainer;
  irqstate_t flags;
  uint16_t reqlen;
  int ret;
  int i;

  usbtrace(TRACE_CLASSBIND, 0);

  /* Bind the structures */

  priv->usbdev   = dev;
  dev->ep0->priv = priv;

  /* Preallocate control request */

  priv->ctrlreq = cdcser_allocreq(dev->ep0, CDCSER_MXDESCLEN);
  if (priv->ctrlreq == NULL)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALLOCCTRLREQ), 0);
      ret = -ENOMEM;
      goto errout;
    }
  priv->ctrlreq->callback = cdcser_ep0incomplete;

  /* Pre-allocate all endpoints... the endpoints will not be functional
   * until the SET CONFIGURATION request is processed in cdcser_setconfig.
   * This is done here because there may be calls to kmalloc and the SET
   * CONFIGURATION processing probably occurrs within interrupt handling
   * logic where kmalloc calls will fail.
   */

  /* Pre-allocate the IN interrupt endpoint */

  priv->epintin = DEV_ALLOCEP(dev, CDCSER_EPINTIN_ADDR, true, USB_EP_ATTR_XFER_INT);
  if (!priv->epintin)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPINTINALLOCFAIL), 0);
      ret = -ENODEV;
      goto errout;
    }
  priv->epintin->priv = priv;

  /* Pre-allocate the IN bulk endpoint */

  priv->epbulkin = DEV_ALLOCEP(dev, CDCSER_EPINBULK_ADDR, true, USB_EP_ATTR_XFER_BULK);
  if (!priv->epbulkin)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKINALLOCFAIL), 0);
      ret = -ENODEV;
      goto errout;
    }
  priv->epbulkin->priv = priv;

  /* Pre-allocate the OUT bulk endpoint */

  priv->epbulkout = DEV_ALLOCEP(dev, CDCSER_EPOUTBULK_ADDR, false, USB_EP_ATTR_XFER_BULK);
  if (!priv->epbulkout)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPBULKOUTALLOCFAIL), 0);
      ret = -ENODEV;
      goto errout;
    }
  priv->epbulkout->priv = priv;

  /* Pre-allocate read requests */

#ifdef CONFIG_CDCSER_BULKREQLEN
  reqlen = max(CONFIG_CDCSER_BULKREQLEN, priv->epbulkout->maxpacket);
#else
  reqlen = priv->epbulkout->maxpacket;
#endif

  for (i = 0; i < CONFIG_CDCSER_NRDREQS; i++)
    {
      reqcontainer      = &priv->rdreqs[i];
      reqcontainer->req = cdcser_allocreq(priv->epbulkout, reqlen);
      if (reqcontainer->req == NULL)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_RDALLOCREQ), -ENOMEM);
          ret = -ENOMEM;
          goto errout;
        }
      reqcontainer->req->priv     = reqcontainer;
      reqcontainer->req->callback = cdcser_rdcomplete;
    }

  /* Pre-allocate write request containers and put in a free list */

#ifdef CONFIG_CDCSER_BULKREQLEN
  reqlen = max(CONFIG_CDCSER_BULKREQLEN, priv->epbulkin->maxpacket);
#else
  reqlen = priv->epbulkin->maxpacket;
#endif

  for (i = 0; i < CONFIG_CDCSER_NWRREQS; i++)
    {
      reqcontainer      = &priv->wrreqs[i];
      reqcontainer->req = cdcser_allocreq(priv->epbulkin, reqlen);
      if (reqcontainer->req == NULL)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_WRALLOCREQ), -ENOMEM);
          ret = -ENOMEM;
          goto errout;
        }
      reqcontainer->req->priv     = reqcontainer;
      reqcontainer->req->callback = cdcser_wrcomplete;

      flags = irqsave();
      sq_addlast((sq_entry_t*)reqcontainer, &priv->reqlist);
      priv->nwrq++;     /* Count of write requests available */
      irqrestore(flags);
    }

  /* Report if we are selfpowered */

#ifdef CONFIG_USBDEV_SELFPOWERED
  DEV_SETSELFPOWERED(dev);
#endif

  /* And pull-up the data line for the soft connect function */

  DEV_CONNECT(dev);
  return OK;

errout:
  cdcser_unbind(dev);
  return ret;
}

/****************************************************************************
 * Name: cdcser_unbind
 *
 * Description:
 *    Invoked when the driver is unbound from a USB device driver
 *
 ****************************************************************************/

static void cdcser_unbind(FAR struct usbdev_s *dev)
{
  FAR struct cdcser_dev_s *priv;
  FAR struct cdcser_req_s *reqcontainer;
  irqstate_t flags;
  int i;

  usbtrace(TRACE_CLASSUNBIND, 0);

#ifdef CONFIG_DEBUG
  if (!dev || !dev->ep0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
     }
#endif

  /* Extract reference to private data */

  priv = (FAR struct cdcser_dev_s *)dev->ep0->priv;

#ifdef CONFIG_DEBUG
  if (!priv)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EP0NOTBOUND), 0);
      return;
    }
#endif

  /* Make sure that we are not already unbound */

  if (priv != NULL)
    {
      /* Make sure that the endpoints have been unconfigured.  If
       * we were terminated gracefully, then the configuration should
       * already have been reset.  If not, then calling cdcser_resetconfig
       * should cause the endpoints to immediately terminate all
       * transfers and return the requests to us (with result == -ESHUTDOWN)
       */

      cdcser_resetconfig(priv);
      up_mdelay(50);

      /* Free the interrupt IN endpoint */

      if (priv->epintin)
        {
          DEV_FREEEP(dev, priv->epintin);
          priv->epintin = NULL;
        }

      /* Free the bulk IN endpoint */

      if (priv->epbulkin)
        {
          DEV_FREEEP(dev, priv->epbulkin);
          priv->epbulkin = NULL;
        }

      /* Free the pre-allocated control request */

      if (priv->ctrlreq != NULL)
        {
          cdcser_freereq(dev->ep0, priv->ctrlreq);
          priv->ctrlreq = NULL;
        }

      /* Free pre-allocated read requests (which should all have
       * been returned to the free list at this time -- we don't check)
       */

      DEBUGASSERT(priv->nrdq == 0);
      for (i = 0; i < CONFIG_CDCSER_NRDREQS; i++)
        {
          reqcontainer = &priv->rdreqs[i];
          if (reqcontainer->req)
            {
              cdcser_freereq(priv->epbulkout, reqcontainer->req);
              reqcontainer->req = NULL;
            }
        }

      /* Free the bulk OUT endpoint */

      if (priv->epbulkout)
        {
          DEV_FREEEP(dev, priv->epbulkout);
          priv->epbulkout = NULL;
        }

      /* Free write requests that are not in use (which should be all
       * of them
       */

      flags = irqsave();
      DEBUGASSERT(priv->nwrq == CONFIG_CDCSER_NWRREQS);
      while (!sq_empty(&priv->reqlist))
        {
          reqcontainer = (struct cdcser_req_s *)sq_remfirst(&priv->reqlist);
          if (reqcontainer->req != NULL)
            {
              cdcser_freereq(priv->epbulkin, reqcontainer->req);
              priv->nwrq--;     /* Number of write requests queued */
            }
        }
      DEBUGASSERT(priv->nwrq == 0);
      irqrestore(flags);
    }

  /* Clear out all data in the circular buffer */

  priv->serdev.xmit.head = 0;
  priv->serdev.xmit.tail = 0;
}

/****************************************************************************
 * Name: cdcser_setup
 *
 * Description:
 *   Invoked for ep0 control requests.  This function probably executes
 *   in the context of an interrupt handler.
 *
 ****************************************************************************/

static int cdcser_setup(FAR struct usbdev_s *dev, const struct usb_ctrlreq_s *ctrl)
{
  FAR struct cdcser_dev_s *priv;
  FAR struct usbdev_req_s *ctrlreq;
  uint16_t value;
  uint16_t index;
  uint16_t len;
  int ret = -EOPNOTSUPP;

#ifdef CONFIG_DEBUG
  if (!dev || !dev->ep0 || !ctrl)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return -EIO;
     }
#endif

  /* Extract reference to private data */

  usbtrace(TRACE_CLASSSETUP, ctrl->req);
  priv = (FAR struct cdcser_dev_s *)dev->ep0->priv;

#ifdef CONFIG_DEBUG
  if (!priv || !priv->ctrlreq)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EP0NOTBOUND), 0);
      return -ENODEV;
    }
#endif
  ctrlreq = priv->ctrlreq;

  /* Extract the little-endian 16-bit values to host order */

  value = GETUINT16(ctrl->value);
  index = GETUINT16(ctrl->index);
  len   = GETUINT16(ctrl->len);

  uvdbg("type=%02x req=%02x value=%04x index=%04x len=%04x\n",
        ctrl->type, ctrl->req, value, index, len);

  switch (ctrl->type & USB_REQ_TYPE_MASK)
    {
     /***********************************************************************
      * Standard Requests
      ***********************************************************************/

    case USB_REQ_TYPE_STANDARD:
      {
        /* If the serial device is used in as part of a composite device,
         * then standard descriptors are handled by logic in the composite
         * device logic.
         */

#ifdef CONFIG_CDCSER_COMPOSITE
        usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDSTDREQ), ctrl->req);
#else
        switch (ctrl->req)
          {
          case USB_REQ_GETDESCRIPTOR:
            {
              /* The value field specifies the descriptor type in the MS byte and the
               * descriptor index in the LS byte (order is little endian)
               */

              switch (ctrl->value[1])
                {
                case USB_DESC_TYPE_DEVICE:
                  {
                    ret = USB_SIZEOF_DEVDESC;
                    memcpy(ctrlreq->buf, cdcser_getdevdesc(), ret);
                  }
                  break;

#ifdef CONFIG_USBDEV_DUALSPEED
                case USB_DESC_TYPE_DEVICEQUALIFIER:
                  {
                    ret = USB_SIZEOF_QUALDESC;
                    memcpy(ctrlreq->buf, cdcser_getqualdesc(), ret);
                  }
                  break;

                case USB_DESC_TYPE_OTHERSPEEDCONFIG:
#endif /* CONFIG_USBDEV_DUALSPEED */

                case USB_DESC_TYPE_CONFIG:
                  {
#ifdef CONFIG_USBDEV_DUALSPEED
                    ret = cdcser_mkcfgdesc(ctrlreq->buf, dev->speed, ctrl->req);
#else
                    ret = cdcser_mkcfgdesc(ctrlreq->buf);
#endif
                  }
                  break;

                case USB_DESC_TYPE_STRING:
                  {
                    /* index == language code. */

                    ret = cdcser_mkstrdesc(ctrl->value[0], (struct usb_strdesc_s *)ctrlreq->buf);
                  }
                  break;

                default:
                  {
                    usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_GETUNKNOWNDESC), value);
                  }
                  break;
                }
            }
            break;

          case USB_REQ_SETCONFIGURATION:
            {
              if (ctrl->type == 0)
                {
                  ret = cdcser_setconfig(priv, value);
                }
            }
            break;

          case USB_REQ_GETCONFIGURATION:
            {
              if (ctrl->type == USB_DIR_IN)
                {
                  *(uint8_t*)ctrlreq->buf = priv->config;
                  ret = 1;
                }
            }
            break;

          case USB_REQ_SETINTERFACE:
            {
              if (ctrl->type == USB_REQ_RECIPIENT_INTERFACE)
                {
                  if (priv->config == CDCSER_CONFIGID &&
                      index == CDCSER_INTERFACEID &&
                      value == CDCSER_ALTINTERFACEID)
                    {
                      cdcser_resetconfig(priv);
                      cdcser_setconfig(priv, priv->config);
                      ret = 0;
                    }
                }
            }
            break;

          case USB_REQ_GETINTERFACE:
            {
              if (ctrl->type == (USB_DIR_IN|USB_REQ_RECIPIENT_INTERFACE) &&
                  priv->config == CDCSER_CONFIGIDNONE)
                {
                  if (index != CDCSER_INTERFACEID)
                    {
                      ret = -EDOM;
                    }
                  else
                     {
                      *(uint8_t*) ctrlreq->buf = CDCSER_ALTINTERFACEID;
                      ret = 1;
                    }
                }
             }
             break;

          default:
            usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDSTDREQ), ctrl->req);
            break;
          }
#endif
      }
      break;

    /************************************************************************
     * CDC ACM-Specific Requests
     ************************************************************************/

    /* ACM_GET_LINE_CODING requests current DTE rate, stop-bits, parity, and
     * number-of-character bits. (Optional)
     */

    case ACM_GET_LINE_CODING:
      {
        if (ctrl->type == (USB_DIR_IN|USB_REQ_TYPE_CLASS|USB_REQ_RECIPIENT_INTERFACE))
          {
            /* Return the current line status from the private data structure */

            memcpy(ctrlreq->buf, &priv->linecoding, SIZEOF_CDC_LINECODING);
            ret = SIZEOF_CDC_LINECODING;
          }
        else
          {
            usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDCLASSREQ), ctrl->type);
          }
      }
      break;

    /* ACM_SET_LINE_CODING configures DTE rate, stop-bits, parity, and
     * number-of-character bits. (Optional)
     */

    case ACM_SET_LINE_CODING:
      {
        if (ctrl->type == (USB_DIR_OUT|USB_REQ_TYPE_CLASS|USB_REQ_RECIPIENT_INTERFACE))
          {
            /* Save the new line coding in the private data structure */

            memcpy(&priv->linecoding, ctrlreq->buf, min(len, 7));
            ret = 0;

            /* If there is a registered callback to receive line status info, then
             * callout now.
             */

            if (priv->callback)
              {
                priv->callback(CDCSER_EVENT_LINECODING);
              }
          }
        else
          {
            usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDCLASSREQ), ctrl->type);
          }
      }
      break;

    /* ACM_SET_CTRL_LINE_STATE: RS-232 signal used to tell the DCE device the
     * DTE device is now present. (Optional)
     */

    case ACM_SET_CTRL_LINE_STATE:
      {
        if (ctrl->type == (USB_DIR_OUT|USB_REQ_TYPE_CLASS|USB_REQ_RECIPIENT_INTERFACE))
          {
            /* Save the control line state in the private data structure. Only bits
             * 0 and 1 have meaning.
             */

            priv->ctrlline = value & 3;
            ret = 0;

            /* If there is a registered callback to receive control line status info,
             * then callout now.
             */

            if (priv->callback)
              {
                priv->callback(CDCSER_EVENT_CTRLLINE);
              }
          }
        else
          {
            usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDCLASSREQ), ctrl->type);
          }
      }
      break;

    /*  Sends special carrier*/

    case ACM_SEND_BREAK:
      {
        if (ctrl->type == (USB_DIR_OUT|USB_REQ_TYPE_CLASS|USB_REQ_RECIPIENT_INTERFACE))
          {
            /* If there is a registered callback to handle the SendBreak request,
             * then callout now.
             */

            ret = 0;
            if (priv->callback)
              {
                priv->callback(CDCSER_EVENT_SENDBREAK);
              }
          }
        else
          {
            usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDCLASSREQ), ctrl->type);
          }
      }
      break;

    default:
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UNSUPPORTEDTYPE), ctrl->type);
      break;
    }

  /* Respond to the setup command if data was returned.  On an error return
   * value (ret < 0), the USB driver will stall.
   */

  if (ret >= 0)
    {
      ctrlreq->len   = min(len, ret);
      ctrlreq->flags = USBDEV_REQFLAGS_NULLPKT;
      ret            = EP_SUBMIT(dev->ep0, ctrlreq);
      if (ret < 0)
        {
          usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EPRESPQ), (uint16_t)-ret);
          ctrlreq->result = OK;
          cdcser_ep0incomplete(dev->ep0, ctrlreq);
        }
    }
  return ret;
}

/****************************************************************************
 * Name: cdcser_disconnect
 *
 * Description:
 *   Invoked after all transfers have been stopped, when the host is
 *   disconnected.  This function is probably called from the context of an
 *   interrupt handler.
 *
 ****************************************************************************/

static void cdcser_disconnect(FAR struct usbdev_s *dev)
{
  FAR struct cdcser_dev_s *priv;
  irqstate_t flags;

  usbtrace(TRACE_CLASSDISCONNECT, 0);

#ifdef CONFIG_DEBUG
  if (!dev || !dev->ep0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return;
     }
#endif

  /* Extract reference to private data */

  priv = (FAR struct cdcser_dev_s *)dev->ep0->priv;

#ifdef CONFIG_DEBUG
  if (!priv)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_EP0NOTBOUND), 0);
      return;
    }
#endif

  /* Reset the configuration */

  flags = irqsave();
  cdcser_resetconfig(priv);

  /* Clear out all data in the circular buffer */

  priv->serdev.xmit.head = 0;
  priv->serdev.xmit.tail = 0;
  irqrestore(flags);

  /* Perform the soft connect function so that we will we can be
   * re-enumerated.
   */

  DEV_CONNECT(dev); 
}

/****************************************************************************
 * Serial Device Methods
 ****************************************************************************/

/****************************************************************************
 * Name: cdcuart_setup
 *
 * Description:
 *   This method is called the first time that the serial port is opened.
 *
 ****************************************************************************/

static int cdcuart_setup(FAR struct uart_dev_s *dev)
{
  FAR struct cdcser_dev_s *priv;

  usbtrace(CDCSER_CLASSAPI_SETUP, 0);

  /* Sanity check */

#if CONFIG_DEBUG
  if (!dev || !dev->priv)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return -EIO;
    }
#endif

  /* Extract reference to private data */

  priv = (FAR struct cdcser_dev_s*)dev->priv;

  /* Check if we have been configured */

  if (priv->config == CDCSER_CONFIGIDNONE)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_SETUPNOTCONNECTED), 0);
      return -ENOTCONN;
    }

  return OK;
}

/****************************************************************************
 * Name: cdcuart_shutdown
 *
 * Description:
 *   This method is called when the serial port is closed.  This operation
 *   is very simple for the USB serial backend because the serial driver
 *   has already assured that the TX data has full drained -- it calls
 *   cdcuart_txempty() until that function returns true before calling this
 *   function.
 *
 ****************************************************************************/

static void cdcuart_shutdown(FAR struct uart_dev_s *dev)
{
  usbtrace(CDCSER_CLASSAPI_SHUTDOWN, 0);

  /* Sanity check */

#if CONFIG_DEBUG
  if (!dev || !dev->priv)
    {
       usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
    }
#endif
}

/****************************************************************************
 * Name: cdcuart_attach
 *
 * Description:
 *   Does not apply to the USB serial class device
 *
 ****************************************************************************/

static int cdcuart_attach(FAR struct uart_dev_s *dev)
{
  usbtrace(CDCSER_CLASSAPI_ATTACH, 0);
  return OK;
}

/****************************************************************************
 * Name: cdcuart_detach
 *
 * Description:
*   Does not apply to the USB serial class device
  *
 ****************************************************************************/

static void cdcuart_detach(FAR struct uart_dev_s *dev)
{
  usbtrace(CDCSER_CLASSAPI_DETACH, 0);
}

/****************************************************************************
 * Name: cdcuart_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int cdcuart_ioctl(FAR struct file *filep,int cmd,unsigned long arg)
{
  struct inode        *inode = filep->f_inode;
  struct cdcser_dev_s *priv  = inode->i_private;
  int                  ret   = OK;

  switch (cmd)
    {
    /* CAICO_REGISTERCB
     *   Register a callback for serial event notification. Argument:
     *   cdcser_callback_t.  See cdcser_callback_t type definition below.
     *   NOTE:  The callback will most likely invoked at the interrupt level.
     *   The called back function should, therefore, limit its operations to
     *   invoking some kind of IPC to handle the serial event in some normal
     *   task environment.
     */

    case CAIOC_REGISTERCB:
      {
        /* Save the new callback function */

        priv->callback = (cdcser_callback_t)((uintptr_t)arg);
      }
      break;

    /* CAIOC_GETLINECODING
     *   Get current line coding.  Argument: struct cdc_linecoding_s*.
     *   See include/nuttx/usb/cdc.h for structure definition.  This IOCTL
     *   should be called to get the data associated with the
     *   CDCSER_EVENT_LINECODING event).
     */

    case CAIOC_GETLINECODING:
      {
        FAR struct cdc_linecoding_s *ptr = (FAR struct cdc_linecoding_s *)((uintptr_t)arg);
        if (ptr)
          {
            memcpy(ptr, &priv->linecoding, sizeof(struct cdc_linecoding_s));
          }
        else
          {
            ret = -EINVAL;
          }
      }
      break;

    /* CAIOC_GETCTRLLINE
     *   Get control line status bits. Argument FAR int*.  See
     *   include/nuttx/usb/cdc.h for bit definitions.  This IOCTL should be
     *   called to get the data associated CDCSER_EVENT_CTRLLINE event.
     */

   case CAIOC_GETCTRLLINE:
      {
        FAR int *ptr = (FAR int *)((uintptr_t)arg);
        if (ptr)
          {
            *ptr = priv->ctrlline;
          }
        else
          {
            ret = -EINVAL;
          }
      }
      break;

    /* CAIOC_NOTIFY
     *   Send a serial state to the host via the Interrupt IN endpoint.
     *   Argument: int.  This includes the current state of the carrier detect,
     *   DSR, break, and ring signal.  See "Table 69: UART State Bitmap Values"
     *   and CDC_UART_definitions in include/nuttx/usb/cdc.h.
     */

    case CAIOC_NOTIFY:
      {
        /* Not yet implemented.  I probably won't bother to implement until
         * I comr up with a usage model that needs it.
         *
         * Here is what the needs to be done:
         *
         * 1. Format and send a request header with:
         *
         *   bmRequestType:
         *    USB_REQ_DIR_IN|USB_REQ_TYPE_CLASS|USB_REQ_RECIPIENT_INTERFACE
         *   bRequest: ACM_SERIAL_STATE
         *   wValue: 0
         *   wIndex: 0
         *   wLength: Length of data
         *
         * 2. Followed by the notification data (in a separate packet)
         */

        ret = -ENOSYS;
      }
      break;

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: cdcuart_rxint
 *
 * Description:
 *   Called by the serial driver to enable or disable RX interrupts.  We, of
 *   course, have no RX interrupts but must behave consistently.  This method
 *   is called under the conditions:
 *
 *   1. With enable==true when the port is opened (just after cdcuart_setup
 *      and cdcuart_attach are called called)
 *   2. With enable==false while transferring data from the RX buffer
 *   2. With enable==true while waiting for more incoming data
 *   3. With enable==false when the port is closed (just before cdcuart_detach
 *      and cdcuart_shutdown are called).
 *
 ****************************************************************************/

static void cdcuart_rxint(FAR struct uart_dev_s *dev, bool enable)
{
  FAR struct cdcser_dev_s *priv;
  FAR uart_dev_t *serdev;
  irqstate_t flags;

  usbtrace(CDCSER_CLASSAPI_RXINT, (uint16_t)enable);

  /* Sanity check */

#if CONFIG_DEBUG
  if (!dev || !dev->priv)
    {
       usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
       return;
    }
#endif

  /* Extract reference to private data */

  priv   = (FAR struct cdcser_dev_s*)dev->priv;
  serdev = &priv->serdev;

  /* We need exclusive access to the RX buffer and private structure
   * in the following.
   */

  flags = irqsave();
  if (enable)
    {
       /* RX "interrupts" are enabled.  Is this a transition from disabled
        * to enabled state?
        */

       if (!priv->rxenabled)
         {
           /* Yes.  During the time that RX interrupts are disabled, the
            * the serial driver will be extracting data from the circular
            * buffer and modifying recv.tail.  During this time, we
            * should avoid modifying recv.head; When interrupts are restored,
            * we can update the head pointer for all of the data that we
            * put into cicular buffer while "interrupts" were disabled.
            */

          if (priv->rxhead != serdev->recv.head)
            {
              serdev->recv.head = priv->rxhead;

              /* Yes... signal the availability of new data */

              uart_datareceived(serdev);
            }

          /* RX "interrupts are no longer disabled */

          priv->rxenabled = true;
        }
    }

  /* RX "interrupts" are disabled.  Is this a transition from enabled
   * to disabled state?
   */

  else if (priv->rxenabled)
    {
      /* Yes.  During the time that RX interrupts are disabled, the
       * the serial driver will be extracting data from the circular
       * buffer and modifying recv.tail.  During this time, we
       * should avoid modifying recv.head; When interrupts are disabled,
       * we use a shadow index and continue adding data to the circular
       * buffer.
       */

      priv->rxhead    = serdev->recv.head;
      priv->rxenabled = false;
    }
  irqrestore(flags);
}

/****************************************************************************
 * Name: cdcuart_txint
 *
 * Description:
 *   Called by the serial driver to enable or disable TX interrupts.  We, of
 *   course, have no TX interrupts but must behave consistently.  Initially,
 *   TX interrupts are disabled.  This method is called under the conditions:
 *
 *   1. With enable==false while transferring data into the TX buffer
 *   2. With enable==true when data may be taken from the buffer.
 *   3. With enable==false when the TX buffer is empty
 *
 ****************************************************************************/

static void cdcuart_txint(FAR struct uart_dev_s *dev, bool enable)
{
  FAR struct cdcser_dev_s *priv;

  usbtrace(CDCSER_CLASSAPI_TXINT, (uint16_t)enable);

  /* Sanity checks */

#if CONFIG_DEBUG
  if (!dev || !dev->priv)
    {
       usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
       return;
    }
#endif

 /* Extract references to private data */

  priv = (FAR struct cdcser_dev_s*)dev->priv;

  /* If the new state is enabled and if there is data in the XMIT buffer,
   * send the next packet now.
   */

  uvdbg("enable=%d head=%d tail=%d\n",
        enable, priv->serdev.xmit.head, priv->serdev.xmit.tail);

  if (enable && priv->serdev.xmit.head != priv->serdev.xmit.tail)
    {
      cdcser_sndpacket(priv);
    }
}

/****************************************************************************
 * Name: cdcuart_txempty
 *
 * Description:
 *   Return true when all data has been sent.  This is called from the
 *   serial driver when the driver is closed.  It will call this API
 *   periodically until it reports true.  NOTE that the serial driver takes all
 *   responsibility for flushing TX data through the hardware so we can be
 *   a bit sloppy about that.
 *
 ****************************************************************************/

static bool cdcuart_txempty(FAR struct uart_dev_s *dev)
{
  FAR struct cdcser_dev_s *priv = (FAR struct cdcser_dev_s*)dev->priv;

  usbtrace(CDCSER_CLASSAPI_TXEMPTY, 0);

#if CONFIG_DEBUG
  if (!priv)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_INVALIDARG), 0);
      return true;
    }
#endif

  /* When all of the allocated write requests have been returned to the
   * reqlist, then there is no longer any TX data in flight.
   */

  return priv->nwrq >= CONFIG_CDCSER_NWRREQS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cdcser_initialize
 *
 * Description:
 *   Register USB serial port (and USB serial console if so configured).
 *
 * Input Parameter:
 *   Device minor number.  E.g., minor 0 would correspond to /dev/ttyUSB0.
 *
 * Returned Value:
 *   Zero (OK) means that the driver was successfully registered.  On any
 *   failure, a negated errno value is retured.
 *
 ****************************************************************************/

int cdcser_initialize(int minor)
{
  FAR struct cdcser_alloc_s *alloc;
  FAR struct cdcser_dev_s *priv;
  FAR struct cdcser_driver_s *drvr;
  char devname[16];
  int ret;

  /* Allocate the structures needed */

  alloc = (FAR struct cdcser_alloc_s*)kmalloc(sizeof(struct cdcser_alloc_s));
  if (!alloc)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_ALLOCDEVSTRUCT), 0);
      return -ENOMEM;
    }

  /* Convenience pointers into the allocated blob */

  priv = &alloc->dev;
  drvr = &alloc->drvr;

  /* Initialize the USB serial driver structure */

  memset(priv, 0, sizeof(struct cdcser_dev_s));
  sq_init(&priv->reqlist);

  /* Fake line status */

  priv->linecoding.baud[0] = (115200) & 0xff;       /* Baud=115200 */
  priv->linecoding.baud[1] = (115200 >> 8) & 0xff;
  priv->linecoding.baud[2] = (115200 >> 16) & 0xff;
  priv->linecoding.baud[3] = (115200 >> 24) & 0xff;
  priv->linecoding.stop    = CDC_CHFMT_STOP1;       /* One stop bit */
  priv->linecoding.parity  = CDC_PARITY_NONE;       /* No parity */
  priv->linecoding.nbits   = 8;                     /* 8 data bits */

  /* Initialize the serial driver sub-structure */

  priv->serdev.recv.size   = CONFIG_CDCSER_RXBUFSIZE;
  priv->serdev.recv.buffer = priv->rxbuffer;
  priv->serdev.xmit.size   = CONFIG_CDCSER_TXBUFSIZE;
  priv->serdev.xmit.buffer = priv->txbuffer;
  priv->serdev.ops         = &g_uartops;
  priv->serdev.priv        = priv;

  /* Initialize the USB class driver structure */

#ifdef CONFIG_USBDEV_DUALSPEED
  drvr->drvr.speed         = USB_SPEED_HIGH;
#else
  drvr->drvr.speed         = USB_SPEED_FULL;
#endif
  drvr->drvr.ops           = &g_driverops;
  drvr->dev                = priv;

  /* Register the USB serial class driver */

  ret = usbdev_register(&drvr->drvr);
  if (ret)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_DEVREGISTER), (uint16_t)-ret);
      goto errout_with_alloc;
    }

  /* Register the USB serial console */

#ifdef CONFIG_CDCSER_CONSOLE
  g_usbserialport.isconsole = true;
  ret = uart_register("/dev/console", &pri->serdev);
  if (ret < 0)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_CONSOLEREGISTER), (uint16_t)-ret);
      goto errout_with_class;
    }
#endif

  /* Register the single port supported by this implementation */

  sprintf(devname, "/dev/ttyUSB%d", minor);
  ret = uart_register(devname, &priv->serdev);
  if (ret)
    {
      usbtrace(TRACE_CLSERROR(USBSER_TRACEERR_UARTREGISTER), (uint16_t)-ret);
      goto errout_with_class;
    }
  return OK;

errout_with_class:
  usbdev_unregister(&drvr->drvr);
errout_with_alloc:
  kfree(alloc);
  return ret;
}
