/****************************************************************************
 * drivers/usbdev/msc_descriptors.c
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

#include "msc.h"

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

#ifndef CONFIG_USBSTRG_COMPOSITE
static const struct usb_devdesc_s g_devdesc =
{
  USB_SIZEOF_DEVDESC,                           /* len */
  USB_DESC_TYPE_DEVICE,                         /* type */
  {LSBYTE(0x0200), MSBYTE(0x0200)},             /* usb */
  USB_CLASS_PER_INTERFACE,                      /* class */
  0,                                            /* subclass */
  0,                                            /* protocol */
  CONFIG_USBSTRG_EP0MAXPACKET,                  /* maxpacketsize */
  {                                             /* vendor */
    LSBYTE(CONFIG_USBSTRG_VENDORID),
    MSBYTE(CONFIG_USBSTRG_VENDORID)
  },
  {                                             /* product */
    LSBYTE(CONFIG_USBSTRG_PRODUCTID),
    MSBYTE(CONFIG_USBSTRG_PRODUCTID) },
  {                                             /* device */
    LSBYTE(CONFIG_USBSTRG_VERSIONNO),
    MSBYTE(CONFIG_USBSTRG_VERSIONNO)
  },
  USBSTRG_MANUFACTURERSTRID,                    /* imfgr */
  USBSTRG_PRODUCTSTRID,                         /* iproduct */
  USBSTRG_SERIALSTRID,                          /* serno */
  USBSTRG_NCONFIGS                              /* nconfigs */
};
#endif

/* Configuration descriptor  If the USB mass storage device is configured as part
 * of a composite device, then the configuration descriptor will be provided by the
 * composite device logic.
 */

#ifndef CONFIG_USBSTRG_COMPOSITE
static const struct usb_cfgdesc_s g_cfgdesc =
{
  USB_SIZEOF_CFGDESC,                           /* len */
  USB_DESC_TYPE_CONFIG,                         /* type */
  {0, 0},                                       /* totallen -- to be provided */
  USBSTRG_NINTERFACES,                          /* ninterfaces */
  USBSTRG_CONFIGID,                             /* cfgvalue */
  USBSTRG_CONFIGSTRID,                          /* icfg */
  USB_CONFIG_ATTR_ONE|SELFPOWERED|REMOTEWAKEUP, /* attr */
  (CONFIG_USBDEV_MAXPOWER + 1) / 2              /* mxpower */
};
#endif

/* Single interface descriptor */

static const struct usb_ifdesc_s g_ifdesc =
{
  USB_SIZEOF_IFDESC,                            /* len */
  USB_DESC_TYPE_INTERFACE,                      /* type */
  USBSTRG_INTERFACEID,                          /* ifno */
  USBSTRG_ALTINTERFACEID,                       /* alt */
  USBSTRG_NENDPOINTS,                           /* neps */
  USB_CLASS_MASS_STORAGE,                       /* class */
  USBSTRG_SUBCLASS_SCSI,                        /* subclass */
  USBSTRG_PROTO_BULKONLY,                       /* protocol */
  USBSTRG_INTERFACESTRID                        /* iif */
};

/* Endpoint descriptors */

static const struct usb_epdesc_s g_fsepbulkoutdesc =
{
  USB_SIZEOF_EPDESC,                            /* len */
  USB_DESC_TYPE_ENDPOINT,                       /* type */
  USBSTRG_EPOUTBULK_ADDR,                       /* addr */
  USBSTRG_EPOUTBULK_ATTR,                       /* attr */
  {                                             /* maxpacket */
    LSBYTE(USBSTRG_FSBULKMAXPACKET),
    MSBYTE(USBSTRG_FSBULKMAXPACKET)
  },
  0                                             /* interval */
};

static const struct usb_epdesc_s g_fsepbulkindesc =
{
  USB_SIZEOF_EPDESC,                            /* len */
  USB_DESC_TYPE_ENDPOINT,                       /* type */
  USBSTRG_EPINBULK_ADDR,                        /* addr */
  USBSTRG_EPINBULK_ATTR,                        /* attr */
  {                                             /* maxpacket */
    LSBYTE(USBSTRG_FSBULKMAXPACKET),
    MSBYTE(USBSTRG_FSBULKMAXPACKET)
  },
  0                                             /* interval */
};

#ifdef CONFIG_USBDEV_DUALSPEED
#ifndef CONFIG_USBSTRG_COMPOSITE
static const struct usb_qualdesc_s g_qualdesc =
{
  USB_SIZEOF_QUALDESC,                          /* len */
  USB_DESC_TYPE_DEVICEQUALIFIER,                /* type */
  {                                             /* usb */
    LSBYTE(0x0200),
    MSBYTE(0x0200)
  },
  USB_CLASS_PER_INTERFACE,                      /* class */
  0,                                            /* subclass */
  0,                                            /* protocol */
  CONFIG_USBSTRG_EP0MAXPACKET,                  /* mxpacketsize */
  USBSTRG_NCONFIGS,                             /* nconfigs */
  0,                                            /* reserved */
};
#endif

static const struct usb_epdesc_s g_hsepbulkoutdesc =
{
  USB_SIZEOF_EPDESC,                            /* len */
  USB_DESC_TYPE_ENDPOINT,                       /* type */
  USBSTRG_EPOUTBULK_ADDR,                       /* addr */
  USBSTRG_EPOUTBULK_ATTR,                       /* attr */
  {                                             /* maxpacket */
    LSBYTE(USBSTRG_HSBULKMAXPACKET),
    MSBYTE(USBSTRG_HSBULKMAXPACKET)
  },
  0                                             /* interval */
};

static const struct usb_epdesc_s g_hsepbulkindesc =
{
  USB_SIZEOF_EPDESC,                            /* len */
  USB_DESC_TYPE_ENDPOINT,                       /* type */
  USBSTRG_EPINBULK_ADDR,                        /* addr */
  USBSTRG_EPINBULK_ATTR,                        /* attr */
  {                                             /* maxpacket */
    LSBYTE(USBSTRG_HSBULKMAXPACKET),
    MSBYTE(USBSTRG_HSBULKMAXPACKET)
  },
  0                                             /* interval */
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/
/* Strings ******************************************************************/

#ifndef CONFIG_USBSTRG_COMPOSITE
const char g_vendorstr[]  = CONFIG_USBSTRG_VENDORSTR;
const char g_productstr[] = CONFIG_USBSTRG_PRODUCTSTR;
const char g_serialstr[]  = CONFIG_USBSTRG_SERIALSTR;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbstrg_mkstrdesc
 *
 * Description:
 *   Construct a string descriptor
 *
 ****************************************************************************/

int usbstrg_mkstrdesc(uint8_t id, struct usb_strdesc_s *strdesc)
{
  const char *str;
  int len;
  int ndata;
  int i;

  switch (id)
    {
#ifndef CONFIG_USBSTRG_COMPOSITE
    case 0:
      {
        /* Descriptor 0 is the language id */

        strdesc->len     = 4;
        strdesc->type    = USB_DESC_TYPE_STRING;
        strdesc->data[0] = LSBYTE(USBSTRG_STR_LANGUAGE);
        strdesc->data[1] = MSBYTE(USBSTRG_STR_LANGUAGE);
        return 4;
      }

      case USBSTRG_MANUFACTURERSTRID:
      str = g_vendorstr;
      break;

    case USBSTRG_PRODUCTSTRID:
      str = g_productstr;
      break;

    case USBSTRG_SERIALSTRID:
      str = g_serialstr;
      break;
#endif

 /* case USBSTRG_CONFIGSTRID: */
    case USBSTRG_INTERFACESTRID:
      str = CONFIG_USBSTRG_CONFIGSTR;
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
 * Name: usbstrg_getepdesc
 *
 * Description:
 *   Return a pointer to the raw device descriptor
 *
 ****************************************************************************/

#ifndef CONFIG_USBSTRG_COMPOSITE
FAR const struct usb_devdesc_s *usbstrg_getdevdesc(void)
{
  return &g_devdesc;
}
#endif

/****************************************************************************
 * Name: usbstrg_getepdesc
 *
 * Description:
 *   Return a pointer to the raw endpoint descriptor (used for configuring
 *   endpoints)
 *
 ****************************************************************************/

FAR const struct usb_epdesc_s *usbstrg_getepdesc(enum usbstrg_epdesc_e epid)
{
  switch (epid)
    {
      case USBSTRG_EPFSBULKOUT:     /* Full speed bulk OUT endpoint descriptor */
        return &g_fsepbulkoutdesc;

      case USBSTRG_EPFSBULKIN:      /* Full speed bulk IN endpoint descriptor */
        return &g_fsepbulkindesc;

#ifdef  CONFIG_USBDEV_DUALSPEED
      case USBSTRG_EPHSBULKOUT:     /* High speed bulk OUT endpoint descriptor */
        return &g_hsepbulkoutdesc;

      case USBSTRG_EPHSBULKIN:      /* High speed bulk IN endpoint descriptor */
        return &g_hsepbulkindesc;
#endif
      default:
        return NULL;
    }
};

/****************************************************************************
 * Name: usbstrg_mkcfgdesc
 *
 * Description:
 *   Construct the configuration descriptor
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DUALSPEED
int16_t usbstrg_mkcfgdesc(uint8_t *buf, uint8_t speed, uint8_t type)
#else
int16_t usbstrg_mkcfgdesc(uint8_t *buf)
#endif
{
#ifndef CONFIG_USBSTRG_COMPOSITE
  FAR struct usb_cfgdesc_s *cfgdesc = (struct usb_cfgdesc_s*)buf;
#endif
#ifdef CONFIG_USBDEV_DUALSPEED
  FAR const struct usb_epdesc_s *epdesc;
  bool hispeed = (speed == USB_SPEED_HIGH);
  uint16_t bulkmxpacket;
#endif
  uint16_t totallen;

  /* This is the total length of the configuration (not necessarily the
   * size that we will be sending now.
   */

  totallen = USB_SIZEOF_CFGDESC + USB_SIZEOF_IFDESC + USBSTRG_NENDPOINTS * USB_SIZEOF_EPDESC;

  /* Configuration descriptor -- Copy the canned descriptor and fill in the
   * type (we'll also need to update the size below).  If the USB mass storage
   * device is configured as part of a composite device, then the configuration
   * descriptor will be provided by the composite device logic.
   */

#ifndef CONFIG_USBSTRG_COMPOSITE
  memcpy(cfgdesc, &g_cfgdesc, USB_SIZEOF_CFGDESC);
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

  bulkmxpacket = USBSTRG_BULKMAXPACKET(hispeed);
  epdesc       = USBSTRG_EPBULKINDESC(hispeed);
  memcpy(buf, epdesc, USB_SIZEOF_EPDESC);
  buf += USB_SIZEOF_EPDESC;

  epdesc       = USBSTRG_EPBULKOUTDESC(hispeed);
  memcpy(buf, epdesc, USB_SIZEOF_EPDESC);
#else
  memcpy(buf, &g_fsepbulkoutdesc, USB_SIZEOF_EPDESC);
  buf += USB_SIZEOF_EPDESC;
  memcpy(buf, &g_fsepbulkindesc, USB_SIZEOF_EPDESC);
#endif

  /* Finally, fill in the total size of the configuration descriptor */

#ifndef CONFIG_USBSTRG_COMPOSITE
  cfgdesc->totallen[0] = LSBYTE(totallen);
  cfgdesc->totallen[1] = MSBYTE(totallen);
#endif
  return totallen;
}

/****************************************************************************
 * Name: usbstrg_getqualdesc
 *
 * Description:
 *   Return a pointer to the raw qual descriptor
 *
 ****************************************************************************/

#if !defined(CONFIG_USBSTRG_COMPOSITE) && defined(CONFIG_USBDEV_DUALSPEED)
FAR const struct usb_qualdesc_s *usbstrg_getqualdesc(void)
{
  return &g_qualdesc;
}
#endif

