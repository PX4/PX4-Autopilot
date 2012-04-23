/****************************************************************************
 * drivers/usbdev/cdcacm_desc.c
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
#include <nuttx/usb/cdc.h>
#include <nuttx/usb/cdcacm.h>
#include <nuttx/usb/usbdev_trace.h>

#include "cdcacm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Describes one description in the group of descriptors forming the
 * total configuration descriptor.
 */

struct cfgdecsc_group_s
{
  uint16_t  descsize; /* Size of the descriptor in bytes */
  uint16_t  hsepsize; /* High speed max packet size */
  FAR void *desc;     /* A pointer to the descriptor */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* USB descriptor templates these will be copied and modified **************/
/* Device Descriptor.  If the USB serial device is configured as part of
 * composite device, then the device descriptor will be provided by the
 * composite device logic.
 */

#ifndef CONFIG_CDCACM_COMPOSITE
static const struct usb_devdesc_s g_devdesc =
{
  USB_SIZEOF_DEVDESC,                           /* len */
  USB_DESC_TYPE_DEVICE,                         /* type */
  {                                             /* usb */
    LSBYTE(0x0200),
    MSBYTE(0x0200)
  },
  USB_CLASS_CDC,                                /* class */
  CDC_SUBCLASS_NONE,                            /* subclass */
  CDC_PROTO_NONE,                               /* protocol */
  CONFIG_CDCACM_EP0MAXPACKET,                   /* maxpacketsize */
  {
    LSBYTE(CONFIG_CDCACM_VENDORID),             /* vendor */
    MSBYTE(CONFIG_CDCACM_VENDORID)
  },
  {
    LSBYTE(CONFIG_CDCACM_PRODUCTID),            /* product */
    MSBYTE(CONFIG_CDCACM_PRODUCTID)
  },
  {
    LSBYTE(CDCACM_VERSIONNO),                   /* device */
    MSBYTE(CDCACM_VERSIONNO)
  },
  CDCACM_MANUFACTURERSTRID,                     /* imfgr */
  CDCACM_PRODUCTSTRID,                          /* iproduct */
  CDCACM_SERIALSTRID,                           /* serno */
  CDCACM_NCONFIGS                               /* nconfigs */
};
#endif

/* Configuration descriptor.  If the USB serial device is configured as part of
 * composite device, then the configuration descriptor will be provided by the
 * composite device logic.
 */

#ifndef CONFIG_CDCACM_COMPOSITE
static const struct usb_cfgdesc_s g_cfgdesc =
{
  USB_SIZEOF_CFGDESC,                           /* len */
  USB_DESC_TYPE_CONFIG,                         /* type */
  {
    LSBYTE(SIZEOF_CDCACM_CFGDESC),              /* LS totallen */
    MSBYTE(SIZEOF_CDCACM_CFGDESC)               /* MS totallen */
  },
  CDCACM_NINTERFACES,                           /* ninterfaces */
  CDCACM_CONFIGID,                              /* cfgvalue */
  CDCACM_CONFIGSTRID,                           /* icfg */
  USB_CONFIG_ATTR_ONE|SELFPOWERED|REMOTEWAKEUP, /* attr */
  (CONFIG_USBDEV_MAXPOWER + 1) / 2              /* mxpower */
};
#endif

/* Interface association descriptor */

#if defined(CONFIG_CDCACM_COMPOSITE) && defined(CONFIG_COMPOSITE_IAD)
static const struct usb_iaddesc_s g_iaddesc =
{
  USB_SIZEOF_IADDESC,                           /* len */
  USB_DESC_TYPE_INTERFACEASSOCIATION,           /* type */
  CONFIG_CDCACM_IFNOBASE,                       /* firstif */
  CDCACM_NINTERFACES,                           /* nifs */
  USB_CLASS_CDC,                                /* class */
  CDC_SUBCLASS_ACM,                             /* subclass */
  CDC_PROTO_NONE,                               /* protocol */
  0                                             /* ifunction */
};
#endif

/* Notification interface */

static const struct usb_ifdesc_s g_notifdesc =
{
  USB_SIZEOF_IFDESC,                            /* len */
  USB_DESC_TYPE_INTERFACE,                      /* type */
  CDCACM_NOTIFID,                               /* ifno */
  CDCACM_NOTALTIFID,                            /* alt */
  1,                                            /* neps */
  USB_CLASS_CDC,                                /* class */
  CDC_SUBCLASS_ACM,                             /* subclass */
  CDC_PROTO_ATM,                                /* proto */
#ifdef CONFIG_CDCACM_NOTIFSTR
  CDCACM_NOTIFSTRID                             /* iif */
#else
  0                                             /* iif */
#endif
};

/* Header functional descriptor */

static const struct cdc_hdr_funcdesc_s g_funchdr =
{
  SIZEOF_HDR_FUNCDESC,                          /* size */
  USB_DESC_TYPE_CSINTERFACE,                    /* type */
  CDC_DSUBTYPE_HDR,                             /* subtype */
  {
    LSBYTE(CDC_VERSIONNO),                      /* LS cdc */
    MSBYTE(CDC_VERSIONNO)                       /* MS cdc */
  }
};
 
/* ACM functional descriptor */

static const struct cdc_acm_funcdesc_s g_acmfunc =
{
  SIZEOF_ACM_FUNCDESC,                          /* size */
  USB_DESC_TYPE_CSINTERFACE,                    /* type */
  CDC_DSUBTYPE_ACM,                             /* subtype */
  0x06                                          /* caps */
};

/* Union functional descriptor */

static const struct cdc_union_funcdesc_s g_unionfunc =
{
  SIZEOF_UNION_FUNCDESC(1),                     /* size */
  USB_DESC_TYPE_CSINTERFACE,                    /* type */
  CDC_DSUBTYPE_UNION,                           /* subtype */
  0,                                            /* master */
  {1}                                           /* slave[0] */
};

/* Interrupt IN endpoint descriptor */

static const struct usb_epdesc_s g_epintindesc =
{
  USB_SIZEOF_EPDESC,                            /* len */
  USB_DESC_TYPE_ENDPOINT,                       /* type */
  CDCACM_EPINTIN_ADDR,                          /* addr */
  CDCACM_EPINTIN_ATTR,                          /* attr */
  {
    LSBYTE(CONFIG_CDCACM_EPINTIN_FSSIZE),       /* maxpacket (full speed) */
    MSBYTE(CONFIG_CDCACM_EPINTIN_FSSIZE)
  },
  0xff                                          /* interval */
};

/* Data interface descriptor */

static const struct usb_ifdesc_s g_dataifdesc =
{
  USB_SIZEOF_IFDESC,                            /* len */
  USB_DESC_TYPE_INTERFACE,                      /* type */
  CDCACM_DATAIFID,                              /* ifno */
  CDCACM_DATAALTIFID,                           /* alt */
  2,                                            /* neps */
  USB_CLASS_CDC_DATA,                           /* class */
  CDC_DATA_SUBCLASS_NONE,                       /* subclass */
  CDC_DATA_PROTO_NONE,                          /* proto */
#ifdef CONFIG_CDCACM_DATAIFSTR
  CDCACM_DATAIFSTRID                            /* iif */
#else
  0                                             /* iif */
#endif
};

/* Bulk OUT endpoint descriptor */

static const struct usb_epdesc_s g_epbulkoutdesc =
{
  USB_SIZEOF_EPDESC,                            /* len */
  USB_DESC_TYPE_ENDPOINT,                       /* type */
  CDCACM_EPOUTBULK_ADDR,                        /* addr */
  CDCACM_EPOUTBULK_ATTR,                        /* attr */
  {
    LSBYTE(CONFIG_CDCACM_EPBULKOUT_FSSIZE),     /* maxpacket (full speed) */
    MSBYTE(CONFIG_CDCACM_EPBULKOUT_FSSIZE)
  },
  1                                             /* interval */
};

/* Bulk IN endpoint descriptor */

static const struct usb_epdesc_s g_epbulkindesc =
{
  USB_SIZEOF_EPDESC,                            /* len */
  USB_DESC_TYPE_ENDPOINT,                       /* type */
  CDCACM_EPINBULK_ADDR,                         /* addr */
  CDCACM_EPINBULK_ATTR,                         /* attr */
  {
    LSBYTE(CONFIG_CDCACM_EPBULKIN_FSSIZE),      /* maxpacket (full speed) */
    MSBYTE(CONFIG_CDCACM_EPBULKIN_FSSIZE)
  },
  1                                             /* interval */
};

/* The components of the the configuration descriptor are maintained as
 * a collection of separate descriptor structure coordinated by the
 * following array.  These descriptors could have been combined into
 * one larger "super" configuration descriptor structure.  However, I
 * have concerns about compiler-dependent alignment and packing.  Since
 * the individual structures consist only of byte types, alignment and
 * packing is not an issue.  And since the are concatentated at run time
 * instead of compile time, there should no issues there either.
 */

static const struct cfgdecsc_group_s g_cfggroup[CDCACM_CFGGROUP_SIZE] =
{
  /* Configuration Descriptor.  If the serial device is used in as part
   * or a composite device, then the configuration descriptor is
   * provided by the composite device logic.
   */

#if !defined(CONFIG_CDCACM_COMPOSITE)
  {
    USB_SIZEOF_CFGDESC,            /* 1. Configuration descriptor */
    0,
    (FAR void *)&g_cfgdesc
  },

  /* If the serial device is part of a composite device, then it should
   * begin with an interface association descriptor (IAD) because the
   * CDC/ACM device consists of more than one interface. The IAD associates
   * the two CDC/ACM interfaces with the same CDC/ACM device.
   */

#elif defined(CONFIG_COMPOSITE_IAD)
  {
    USB_SIZEOF_IADDESC,            /* 1. Interface association descriptor */
    0,
    (FAR void *)&g_iaddesc
  },
#endif
  {
    USB_SIZEOF_IFDESC,             /* 2. Notification interface */
    0,
    (FAR void *)&g_notifdesc
  },
  {
    SIZEOF_HDR_FUNCDESC,           /* 3. Header functional descriptor */
    0,
    (FAR void *)&g_funchdr
  },
  {
    SIZEOF_ACM_FUNCDESC,           /* 4. ACM functional descriptor */
    0,
    (FAR void *)&g_acmfunc
  },
  {
    SIZEOF_UNION_FUNCDESC(1),      /* 5. Union functional descriptor */
    0,
    (FAR void *)&g_unionfunc
  },
  {
    USB_SIZEOF_EPDESC,             /* 6. Interrupt IN endpoint descriptor */
    CONFIG_CDCACM_EPINTIN_HSSIZE,
    (FAR void *)&g_epintindesc
  },
  {
    USB_SIZEOF_IFDESC,             /* 7. Data interface descriptor */
    0,
    (FAR void *)&g_dataifdesc
  },
  {
    USB_SIZEOF_EPDESC,             /* 8. Bulk OUT endpoint descriptor */
    CONFIG_CDCACM_EPBULKOUT_HSSIZE,
    (FAR void *)&g_epbulkoutdesc
  },
  {
    USB_SIZEOF_EPDESC,             /* 9. Bulk OUT endpoint descriptor */
    CONFIG_CDCACM_EPBULKIN_HSSIZE,
    (FAR void *)&g_epbulkindesc
  }
};

#if !defined(CONFIG_CDCACM_COMPOSITE) && defined(CONFIG_USBDEV_DUALSPEED)
static const struct usb_qualdesc_s g_qualdesc =
{
  USB_SIZEOF_QUALDESC,                          /* len */
  USB_DESC_TYPE_DEVICEQUALIFIER,                /* type */
  {                                             /* usb */
     LSBYTE(0x0200),
     MSBYTE(0x0200)
  },
  USB_CLASS_VENDOR_SPEC,                        /* class */
  0,                                            /* subclass */
  0,                                            /* protocol */
  CONFIG_CDCACM_EP0MAXPACKET,                   /* mxpacketsize */
  CDCACM_NCONFIGS,                              /* nconfigs */
  0,                                            /* reserved */
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cdcacm_mkstrdesc
 *
 * Description:
 *   Construct a string descriptor
 *
 ****************************************************************************/

int cdcacm_mkstrdesc(uint8_t id, struct usb_strdesc_s *strdesc)
{
#if !defined(CONFIG_CDCACM_COMPOSITE) || defined(CONFIG_CDCACM_NOTIFSTR) || \
     defined(CONFIG_CDCACM_DATAIFSTR)

  const char *str;
  int len;
  int ndata;
  int i;

  switch (id)
    {
#ifndef CONFIG_CDCACM_COMPOSITE
    case 0:
      {
        /* Descriptor 0 is the language id */

        strdesc->len     = 4;
        strdesc->type    = USB_DESC_TYPE_STRING;
        strdesc->data[0] = LSBYTE(CDCACM_STR_LANGUAGE);
        strdesc->data[1] = MSBYTE(CDCACM_STR_LANGUAGE);
        return 4;
      }

    case CDCACM_MANUFACTURERSTRID:
      str = CONFIG_CDCACM_VENDORSTR;
      break;

    case CDCACM_PRODUCTSTRID:
      str = CONFIG_CDCACM_PRODUCTSTR;
      break;

    case CDCACM_SERIALSTRID:
      str = CONFIG_CDCACM_SERIALSTR;
      break;

    case CDCACM_CONFIGSTRID:
      str = CONFIG_CDCACM_CONFIGSTR;
      break;
#endif

#ifdef CONFIG_CDCACM_NOTIFSTR
    case CDCACM_NOTIFSTRID:
      str = CONFIG_CDCACM_NOTIFSTR;
      break;
#endif

#ifdef CONFIG_CDCACM_DATAIFSTR
    case CDCACM_DATAIFSTRID:
      str = CONFIG_CDCACM_DATAIFSTR;
      break;
#endif

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
#else
   return -EINVAL;
#endif
}

/****************************************************************************
 * Name: cdcacm_getepdesc
 *
 * Description:
 *   Return a pointer to the raw device descriptor
 *
 ****************************************************************************/

#ifndef CONFIG_CDCACM_COMPOSITE
FAR const struct usb_devdesc_s *cdcacm_getdevdesc(void)
{
  return &g_devdesc;
}
#endif

/****************************************************************************
 * Name: cdcacm_getepdesc
 *
 * Description:
 *   Return a pointer to the raw endpoint struct (used for configuring
 *   endpoints)
 *
 ****************************************************************************/

FAR const struct usb_epdesc_s *cdcacm_getepdesc(enum cdcacm_epdesc_e epid)
{
  switch (epid)
    {
      case CDCACM_EPINTIN:       /* Interrupt IN endpoint */
        return &g_epintindesc;

      case CDCACM_EPBULKOUT:     /* Bulk OUT endpoint */
        return &g_epbulkoutdesc;

      case CDCACM_EPBULKIN:      /* Bulk IN endpoint */
        return &g_epbulkindesc;

      default:
        return NULL;
    }
}

/****************************************************************************
 * Name: cdcacm_mkepdesc
 *
 * Description:
 *   Construct the endpoint descriptor using the correct max packet size.
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DUALSPEED
void cdcacm_mkepdesc(num cdcacm_epdesc_e epid, uint16_t mxpacket,
                     FAR struct usb_epdesc_s *outdesc)
{
  FAR const struct usb_epdesc_s *indesc;

  /* Copy the "canned" descriptor */

  indesc = cdcacm_getepdesc(epid)
  memcpy(outdesc, indesc, USB_SIZEOF_EPDESC);

  /* Then add the correct max packet size */

  outdesc->mxpacketsize[0] = LSBYTE(mxpacket);
  outdesc->mxpacketsize[1] = MSBYTE(mxpacket);
}
#endif

/****************************************************************************
 * Name: cdcacm_mkcfgdesc
 *
 * Description:
 *   Construct the configuration descriptor
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DUALSPEED
int16_t cdcacm_mkcfgdesc(FAR uint8_t *buf, uint8_t speed, uint8_t type)
#else
int16_t cdcacm_mkcfgdesc(FAR uint8_t *buf)
#endif
{
  FAR const struct cfgdecsc_group_s *group;
  FAR uint8_t *dest = buf;
  int i;

#ifdef CONFIG_USBDEV_DUALSPEED
  bool hispeed = (speed == USB_SPEED_HIGH);

  /* Check for switches between high and full speed */

  if (type == USB_DESC_TYPE_OTHERSPEEDCONFIG)
    {
      hispeed = !hispeed;
    }
#endif

  /* Copy all of the descriptors in the group */

  for (i = 0, dest = buf; i < CDCACM_CFGGROUP_SIZE; i++)
    {
      group = &g_cfggroup[i];

      /* The "canned" descriptors all have full speed endpoint maxpacket
       * sizes. If high speed is selected, we will have to change the
       * endpoint maxpacket size.
       *
       * Is there a alternative high speed maxpacket size in the table?
       * If so, that is sufficient proof that the descriptor that we
       * just copied is an endpoint descriptor and needs the fixup
       */

#ifdef CONFIG_USBDEV_DUALSPEED
      if (highspeed && group->hsepsize != 0)
        {
          cdcacm_mkepdesc(group->desc, group->hsepsize,
                            (FAR struct usb_epdesc_s*)dest);
        }
      else
#endif
      /* Copy the "canned" descriptor with the full speed max packet
       * size
       */

        {
          memcpy(dest, group->desc, group->descsize);
        }

      /* Advance to the destination location for the next descriptor */

      dest += group->descsize;
    }

  return SIZEOF_CDCACM_CFGDESC;
}

/****************************************************************************
 * Name: cdcacm_getqualdesc
 *
 * Description:
 *   Return a pointer to the raw qual descriptor
 *
 ****************************************************************************/

#if !defined(CONFIG_CDCACM_COMPOSITE) && defined(CONFIG_USBDEV_DUALSPEED)
FAR const struct usb_qualdesc_s *cdcacm_getqualdesc(void)
{
  return &g_qualdesc;
}
#endif
