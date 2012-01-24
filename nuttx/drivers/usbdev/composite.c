/****************************************************************************
 * drivers/usbdev/composite.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/usb/usbdev_trace.h>

#include "composite.h"

#ifdef CONFIG_USBDEV_COMPOSITE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

 /****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: composite_setup
 *
 * Description:
 *   Invoked for ep0 control requests.  This function probably executes
 *   in the context of an interrupt handler.
 *
 ****************************************************************************/

 #if 0
static int composite_setup(FAR struct usbdev_s *dev,
                           FAR const struct usb_ctrlreq_s *ctrl)
{
  FAR struct composite_dev_s *priv;
  FAR struct usbdev_req_s *ctrlreq;
  uint16_t value;
  uint16_t index;
  uint16_t len;
  int ret = -EOPNOTSUPP;

#ifdef CONFIG_DEBUG
  if (!dev || !dev->ep0 || !ctrl)
    {
      usbtrace(TRACE_CLSERROR(USBSTRG_TRACEERR_SETUPINVALIDARGS), 0);
      return -EIO;
     }
#endif

  /* Extract reference to private data */

  usbtrace(TRACE_CLASSSETUP, ctrl->req);
  priv = (FAR struct composite_dev_s *)dev->ep0->priv;

#ifdef CONFIG_DEBUG
  if (!priv || !priv->ctrlreq)
    {
      usbtrace(TRACE_CLSERROR(USBSTRG_TRACEERR_EP0NOTBOUND2), 0);
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

  if ((ctrl->type & USB_REQ_TYPE_MASK) == USB_REQ_TYPE_STANDARD)
    {
      /**********************************************************************
       * Standard Requests
       **********************************************************************/

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
                  memcpy(ctrlreq->buf, composite_getdevdesc(), ret);
                }
                break;

#ifdef CONFIG_USBDEV_DUALSPEED
              case USB_DESC_TYPE_DEVICEQUALIFIER:
                {
                  ret = USB_SIZEOF_QUALDESC;
                  memcpy(ctrlreq->buf, composite_getqualdesc(), ret);
                }
                break;

              case USB_DESC_TYPE_OTHERSPEEDCONFIG:
#endif /* CONFIG_USBDEV_DUALSPEED */

              case USB_DESC_TYPE_CONFIG:
                {
#ifdef CONFIG_USBDEV_DUALSPEED
                  ret = composite_mkcfgdesc(ctrlreq->buf, dev->speed, ctrl->value[1]);
#else
                  ret = composite_mkcfgdesc(ctrlreq->buf);
#endif
                }
                break;

              case USB_DESC_TYPE_STRING:
                {
                  /* index == language code. */

                  ret = composite_mkstrdesc(ctrl->value[0], (struct usb_strdesc_s *)ctrlreq->buf);
                }
                break;

              default:
                {
                  usbtrace(TRACE_CLSERROR(USBSTRG_TRACEERR_GETUNKNOWNDESC), value);
                }
                break;
              }
          }
          break;

        case USB_REQ_SETCONFIGURATION:
          {
            if (ctrl->type == 0)
              {
                /* Signal the worker thread to instantiate the new configuration */

                priv->theventset |= USBSTRG_EVENT_CFGCHANGE;
                priv->thvalue     = value;
                pthread_cond_signal(&priv->cond);

                /* Return here... the response will be provided later by the
                 * worker thread.
                 */

                return OK;
              }
          }
          break;

        case USB_REQ_GETCONFIGURATION:
          {
            if (ctrl->type == USB_DIR_IN)
              {
                ctrlreq->buf[0] = priv->config;
                ret = 1;
              }
          }
          break;

        case USB_REQ_SETINTERFACE:
          {
            if (ctrl->type == USB_REQ_RECIPIENT_INTERFACE)
              {
                if (priv->config == USBSTRG_CONFIGID &&
                    index == USBSTRG_INTERFACEID &&
                    value == USBSTRG_ALTINTERFACEID)
                  {
                    /* Signal to instantiate the interface change */

                    priv->theventset |= USBSTRG_EVENT_IFCHANGE;
                    pthread_cond_signal(&priv->cond);

                    /* Return here... the response will be provided later by the
                     * worker thread.
                     */

                    return OK;
                  }
              }
          }
          break;

        case USB_REQ_GETINTERFACE:
          {
            if (ctrl->type == (USB_DIR_IN|USB_REQ_RECIPIENT_INTERFACE) &&
                priv->config == USBSTRG_CONFIGIDNONE)
              {
                if (index != USBSTRG_INTERFACEID)
                  {
                    ret = -EDOM;
                  }
                else
                  {
                    ctrlreq->buf[0] = USBSTRG_ALTINTERFACEID;
                    ret = 1;
                  }
              }
           }
           break;

        default:
          usbtrace(TRACE_CLSERROR(USBSTRG_TRACEERR_UNSUPPORTEDSTDREQ), ctrl->req);
          break;
        }

      /* Respond to the setup command if data was returned.  On an error return
       * value (ret < 0), the USB driver will stall EP0.
       */

      if (ret >= 0)
        {
          ctrlreq->len   = min(len, ret);
          ctrlreq->flags = USBDEV_REQFLAGS_NULLPKT;
          ret            = EP_SUBMIT(dev->ep0, ctrlreq);
          if (ret < 0)
            {
              usbtrace(TRACE_CLSERROR(USBSTRG_TRACEERR_EPRESPQ), (uint16_t)-ret);
              ctrlreq->result = OK;
              composite_ep0incomplete(dev->ep0, ctrlreq);
        }
    }
  else
    {
      /**********************************************************************
       * Non-Standard Class Requests
       **********************************************************************/
#warning "Missing Logic"
    }

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* CONFIG_USBDEV_COMPOSITE */
