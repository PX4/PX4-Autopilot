/****************************************************************************
 * drivers/usbdev/cdc_serial.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __DRIVERS_USBDEV_USBDEV_CDC_SERIAL_H
#define __DRIVERS_USBDEV_USBDEV_CDC_SERIAL_H 1

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* Descriptors **************************************************************/
/* These settings are not modifiable via the NuttX configuration */

#define CDCSER_CONFIGIDNONE        (0)      /* Config ID means to return to address mode */
#define CDCSER_INTERFACEID         (0)
#define CDCSER_ALTINTERFACEID      (0)

/* Configuration descriptor values */

#define CDCSER_CONFIGID            (1)      /* The only supported configuration ID */

/* Endpoint configuration */

#define CDCSER_EPINTIN_ADDR        (USB_DIR_IN|CONFIG_CDCSER_EPINTIN)
#define CDCSER_EPINTIN_ATTR        (USB_EP_ATTR_XFER_INT)

#define CDCSER_EPOUTBULK_ADDR      (CONFIG_CDCSER_EPBULKOUT)
#define CDCSER_EPOUTBULK_ATTR      (USB_EP_ATTR_XFER_BULK)

#define CDCSER_EPINBULK_ADDR       (USB_DIR_IN|CONFIG_CDCSER_EPBULKIN)
#define CDCSER_EPINBULK_ATTR       (USB_EP_ATTR_XFER_BULK)

/* Buffer big enough for any of our descriptors (the config descriptor is the
 * biggest).
 */

#define CDCSER_MXDESCLEN           (64)

/* Misc Macros **************************************************************/
/* min/max macros */

#ifndef min
#  define min(a,b) ((a)<(b)?(a):(b))
#endif

#ifndef max
#  define max(a,b) ((a)>(b)?(a):(b))
#endif

/* Trace values *************************************************************/

#define CDCSER_CLASSAPI_SETUP       TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_SETUP)
#define CDCSER_CLASSAPI_SHUTDOWN    TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_SHUTDOWN)
#define CDCSER_CLASSAPI_ATTACH      TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_ATTACH)
#define CDCSER_CLASSAPI_DETACH      TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_DETACH)
#define CDCSER_CLASSAPI_IOCTL       TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_IOCTL)
#define CDCSER_CLASSAPI_RECEIVE     TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_RECEIVE)
#define CDCSER_CLASSAPI_RXINT       TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_RXINT)
#define CDCSER_CLASSAPI_RXAVAILABLE TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_RXAVAILABLE)
#define CDCSER_CLASSAPI_SEND        TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_SEND)
#define CDCSER_CLASSAPI_TXINT       TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_TXINT)
#define CDCSER_CLASSAPI_TXREADY     TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_TXREADY)
#define CDCSER_CLASSAPI_TXEMPTY     TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_TXEMPTY)

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum cdcser_epdesc_e
{
  CDCSER_EPINTIN = 0,  /* Interrupt IN endpoint descriptor */
  CDCSER_EPBULKOUT,    /* Bulk OUT endpoint descriptor */
  CDCSER_EPBULKIN      /* Bulk IN endpoint descriptor */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: cdcser_mkstrdesc
 *
 * Description:
 *   Construct a string descriptor
 *
 ****************************************************************************/

int cdcser_mkstrdesc(uint8_t id, struct usb_strdesc_s *strdesc);

/****************************************************************************
 * Name: cdcser_getepdesc
 *
 * Description:
 *   Return a pointer to the raw device descriptor
 *
 ****************************************************************************/

FAR const struct usb_devdesc_s *cdcser_getdevdesc(void);

/****************************************************************************
 * Name: cdcser_getepdesc
 *
 * Description:
 *   Return a pointer to the raw endpoint descriptor (used for configuring
 *   endpoints)
 *
 ****************************************************************************/

FAR const struct usb_epdesc_s *cdcser_getepdesc(enum cdcser_epdesc_e epid);

/****************************************************************************
 * Name: cdcser_mkepdesc
 *
 * Description:
 *   Construct the endpoint descriptor using the correct max packet size.
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DUALSPEED
void cdcser_mkepdesc(enum cdcser_epdesc_e epid,
                     uint16_t mxpacket, FAR struct usb_epdesc_s *outdesc);
#endif

/****************************************************************************
 * Name: cdcser_mkcfgdesc
 *
 * Description:
 *   Construct the configuration descriptor
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DUALSPEED
int16_t cdcser_mkcfgdesc(FAR uint8_t *buf, uint8_t speed, uint8_t type);
#else
int16_t cdcser_mkcfgdesc(FAR uint8_t *buf);
#endif

/****************************************************************************
 * Name: cdcser_getqualdesc
 *
 * Description:
 *   Return a pointer to the raw qual descriptor
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DUALSPEED
FAR const struct usb_qualdesc_s *cdcser_getqualdesc(void);
#endif

#endif /* __DRIVERS_USBDEV_USBDEV_CDC_SERIAL_H */
