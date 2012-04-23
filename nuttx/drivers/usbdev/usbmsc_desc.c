/****************************************************************************
 * drivers/usbdev/usbmsc_desc.c
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
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev_trace.h>

#include "usbmsc.h"

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
/* Descriptors **************************************************************/
/* Device descriptor.  If the USB mass storage device is configured as part
 * of a composite device, then the device descriptor will be provided by the
 * composite device logic.
 */

#ifndef CONFIG_USBMSC_COMPOSITE
static const struct usb_devdesc_s g_devdesc =
{
  USB_SIZEOF_DEVDESC,                           /* len */
  USB_DESC_TYPE_DEVICE,                         /* type */
  {LSBYTE(0x0200), MSBYTE(0x0200)},             /* usb */
  USB_CLASS_PER_INTERFACE,                      /* classid */
  0,                                            /* subclass */
  0,                                            /* protocol */
  CONFIG_USBMSC_EP0MAXPACKET,                   /* maxpacketsize */
  {                                             /* vendor */
    LSBYTE(CONFIG_USBMSC_VENDORID),
    MSBYTE(CONFIG_USBMSC_VENDORID)
  },
  {                                             /* product */
    LSBYTE(CONFIG_USBMSC_PRODUCTID),
    MSBYTE(CONFIG_USBMSC_PRODUCTID) },
  {                                             /* device */
    LSBYTE(CONFIG_USBMSC_VERSIONNO),
    MSBYTE(CONFIG_USBMSC_VERSIONNO)
  },
  USBMSC_MANUFACTURERSTRID,                    /* imfgr */
  USBMSC_PRODUCTSTRID,                         /* iproduct */
  USBMSC_SERIALSTRID,                          /* serno */
  USBMSC_NCONFIGS                              /* nconfigs */
};
#endif

/* Configuration descriptor  If the USB mass storage device is configured as part
 * of a composite device, then the configuration descriptor will be provided by the
 * composite device logic.
 */

#ifndef CONFIG_USBMSC_COMPOSITE
static const struct usb_cfgdesc_s g_cfgdesc =
{
  USB_SIZEOF_CFGDESC,                           /* len */
  USB_DESC_TYPE_CONFIG,                         /* type */
  {                                             /* totallen */
    LSBYTE(SIZEOF_USBMSC_CFGDESC),
    MSBYTE(SIZEOF_USBMSC_CFGDESC)
  },
  USBMSC_NINTERFACES,                           /* ninterfaces */
  USBMSC_CONFIGID,                              /* cfgvalue */
  USBMSC_CONFIGSTRID,                           /* icfg */
  USB_CONFIG_ATTR_ONE|SELFPOWERED|REMOTEWAKEUP, /* attr */
  (CONFIG_USBDEV_MAXPOWER + 1) / 2              /* mxpower */
};
#endif

/* Single interface descriptor */

static const struct usb_ifdesc_s g_ifdesc =
{
  USB_SIZEOF_IFDESC,                            /* len */
  USB_DESC_TYPE_INTERFACE,                      /* type */
  USBMSC_INTERFACEID,                           /* ifno */
  USBMSC_ALTINTERFACEID,                        /* alt */
  USBMSC_NENDPOINTS,                            /* neps */
  USB_CLASS_MASS_STORAGE,                       /* classid */
  USBMSC_SUBCLASS_SCSI,                         /* subclass */
  USBMSC_PROTO_BULKONLY,                        /* protocol */
  USBMSC_INTERFACESTRID                         /* iif */
};

/* Endpoint descriptors */

static const struct usb_epdesc_s g_fsepbulkoutdesc =
{
  USB_SIZEOF_EPDESC,                            /* len */
  USB_DESC_TYPE_ENDPOINT,                       /* type */
  USBMSC_EPOUTBULK_ADDR,                        /* addr */
  USBMSC_EPOUTBULK_ATTR,                        /* attr */
  {                                             /* maxpacket */
    LSBYTE(USBMSC_FSBULKMAXPACKET),
    MSBYTE(USBMSC_FSBULKMAXPACKET)
  },
  0                                             /* interval */
};

static const struct usb_epdesc_s g_fsepbulkindesc =
{
  USB_SIZEOF_EPDESC,                            /* len */
  USB_DESC_TYPE_ENDPOINT,                       /* type */
  USBMSC_EPINBULK_ADDR,                         /* addr */
  USBMSC_EPINBULK_ATTR,                         /* attr */
  {                                             /* maxpacket */
    LSBYTE(USBMSC_FSBULKMAXPACKET),
    MSBYTE(USBMSC_FSBULKMAXPACKET)
  },
  0                                             /* interval */
};

#ifdef CONFIG_USBDEV_DUALSPEED
#ifndef CONFIG_USBMSC_COMPOSITE
static const struct usb_qualdesc_s g_qualdesc =
{
  USB_SIZEOF_QUALDESC,                          /* len */
  USB_DESC_TYPE_DEVICEQUALIFIER,                /* type */
  {                                             /* usb */
    LSBYTE(0x0200),
    MSBYTE(0x0200)
  },
  USB_CLASS_PER_INTERFACE,                      /* classid */
  0,                                            /* subclass */
  0,                                            /* protocol */
  CONFIG_USBMSC_EP0MAXPACKET,                   /* mxpacketsize */
  USBMSC_NCONFIGS,                              /* nconfigs */
  0,                                            /* reserved */
};
#endif

static const struct usb_epdesc_s g_hsepbulkoutdesc =
{
  USB_SIZEOF_EPDESC,                            /* len */
  USB_DESC_TYPE_ENDPOINT,                       /* type */
  USBMSC_EPOUTBULK_ADDR,                        /* addr */
  USBMSC_EPOUTBULK_ATTR,                        /* attr */
  {                                             /* maxpacket */
    LSBYTE(USBMSC_HSBULKMAXPACKET),
    MSBYTE(USBMSC_HSBULKMAXPACKET)
  },
  0                                             /* interval */
};

static const struct usb_epdesc_s g_hsepbulkindesc =
{
  USB_SIZEOF_EPDESC,                            /* len */
  USB_DESC_TYPE_ENDPOINT,                       /* type */
  USBMSC_EPINBULK_ADDR,                         /* addr */
  USBMSC_EPINBULK_ATTR,                         /* attr */
  {                                             /* maxpacket */
    LSBYTE(USBMSC_HSBULKMAXPACKET),
    MSBYTE(USBMSC_HSBULKMAXPACKET)
  },
  0                                             /* interval */
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/
/* Strings ******************************************************************/

#ifndef CONFIG_USBMSC_COMPOSITE
const char g_mscvendorstr[]  = CONFIG_USBMSC_VENDORSTR;
const char g_mscproductstr[] = CONFIG_USBMSC_PRODUCTSTR;
const char g_mscserialstr[]  = CONFIG_USBMSC_SERIALSTR;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbmsc_mkstrdesc
 *
 * Description:
 *   Construct a string descriptor
 *
 ****************************************************************************/

int usbmsc_mkstrdesc(uint8_t id, struct usb_strdesc_s *strdesc)
{
  const char *str;
  int len;
  int ndata;
  int i;

  switch (id)
    {
#ifndef CONFIG_USBMSC_COMPOSITE
    case 0:
      {
        /* Descriptor 0 is the language id */

        strdesc->len     = 4;
        strdesc->type    = USB_DESC_TYPE_STRING;
        strdesc->data[0] = LSBYTE(USBMSC_STR_LANGUAGE);
        strdesc->data[1] = MSBYTE(USBMSC_STR_LANGUAGE);
        return 4;
      }

      case USBMSC_MANUFACTURERSTRID:
      str = g_mscvendorstr;
      break;

    case USBMSC_PRODUCTSTRID:
      str = g_mscproductstr;
      break;

    case USBMSC_SERIALSTRID:
      str = g_mscserialstr;
      break;
#endif

 /* case USBMSC_CONFIGSTRID: */
    case USBMSC_INTERFACESTRID:
      str = CONFIG_USBMSC_CONFIGSTR;
      break;

    default:
      return -EINVAL;
    }

   /* The string is utf16-le.  The poor man's utf-8 to utf16-le
    * conversion below will only handle 7-bit en-us ascii
    */

   len = strlen(str);
   for (i = 0, ndata = 0; i < len; i++, ndata += 2)
     {
       strdesc->data[ndata]   = str[i];
       strdesc->data[ndata+1] = 0;
     }

   strdesc->len  = ndata+2;
   strdesc->type = USB_DESC_TYPE_STRING;
   return strdesc->len;
}

/****************************************************************************
 * Name: usbmsc_getepdesc
 *
 * Description:
 *   Return a pointer to the raw device descriptor
 *
 ****************************************************************************/

#ifndef CONFIG_USBMSC_COMPOSITE
FAR const struct usb_devdesc_s *usbmsc_getdevdesc(void)
{
  return &g_devdesc;
}
#endif

/****************************************************************************
 * Name: usbmsc_getepdesc
 *
 * Description:
 *   Return a pointer to the raw endpoint descriptor (used for configuring
 *   endpoints)
 *
 ****************************************************************************/

FAR const struct usb_epdesc_s *usbmsc_getepdesc(enum usbmsc_epdesc_e epid)
{
  switch (epid)
    {
      case USBMSC_EPFSBULKOUT:     /* Full speed bulk OUT endpoint descriptor */
        return &g_fsepbulkoutdesc;

      case USBMSC_EPFSBULKIN:      /* Full speed bulk IN endpoint descriptor */
        return &g_fsepbulkindesc;

#ifdef  CONFIG_USBDEV_DUALSPEED
      case USBMSC_EPHSBULKOUT:     /* High speed bulk OUT endpoint descriptor */
        return &g_hsepbulkoutdesc;

      case USBMSC_EPHSBULKIN:      /* High speed bulk IN endpoint descriptor */
        return &g_hsepbulkindesc;
#endif
      default:
        return NULL;
    }
};

/****************************************************************************
 * Name: usbmsc_mkcfgdesc
 *
 * Description:
 *   Construct the configuration descriptor
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DUALSPEED
int16_t usbmsc_mkcfgdesc(uint8_t *buf, uint8_t speed, uint8_t type)
#else
int16_t usbmsc_mkcfgdesc(uint8_t *buf)
#endif
{
#ifdef CONFIG_USBDEV_DUALSPEED
  FAR const struct usb_epdesc_s *epdesc;
  bool hispeed = (speed == USB_SPEED_HIGH);
  uint16_t bulkmxpacket;
#endif

  /* Configuration descriptor.  If the USB mass storage device is
   * configured as part of a composite device, then the configuration
   * descriptor will be provided by the composite device logic.
   */

#ifndef CONFIG_USBMSC_COMPOSITE
  memcpy(buf, &g_cfgdesc, USB_SIZEOF_CFGDESC);
  buf += USB_SIZEOF_CFGDESC;
#endif

  /* Copy the canned interface descriptor */

  memcpy(buf, &g_ifdesc, USB_SIZEOF_IFDESC);
  buf += USB_SIZEOF_IFDESC;

  /* Make the two endpoint configurations */

#ifdef CONFIG_USBDEV_DUALSPEED
  /* Check for switches between high and full speed */

  hispeed = (speed == USB_SPEED_HIGH);
  if (type == USB_DESC_TYPE_OTHERSPEEDCONFIG)
    {
      hispeed = !hispeed;
    }

  bulkmxpacket = USBMSC_BULKMAXPACKET(hispeed);
  epdesc       = USBMSC_EPBULKINDESC(hispeed);
  memcpy(buf, epdesc, USB_SIZEOF_EPDESC);
  buf += USB_SIZEOF_EPDESC;

  epdesc       = USBMSC_EPBULKOUTDESC(hispeed);
  memcpy(buf, epdesc, USB_SIZEOF_EPDESC);
#else
  memcpy(buf, &g_fsepbulkoutdesc, USB_SIZEOF_EPDESC);
  buf += USB_SIZEOF_EPDESC;
  memcpy(buf, &g_fsepbulkindesc, USB_SIZEOF_EPDESC);
#endif

  return SIZEOF_USBMSC_CFGDESC;
}

/****************************************************************************
 * Name: usbmsc_getqualdesc
 *
 * Description:
 *   Return a pointer to the raw qual descriptor
 *
 ****************************************************************************/

#if !defined(CONFIG_USBMSC_COMPOSITE) && defined(CONFIG_USBDEV_DUALSPEED)
FAR const struct usb_qualdesc_s *usbmsc_getqualdesc(void)
{
  return &g_qualdesc;
}
#endif

