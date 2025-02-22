/****************************************************************************
 * boards/arm/rp23xx/common/src/rp23xx_composite.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <debug.h>
#include <assert.h>

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/cdcacm.h>
#include <nuttx/usb/usbmsc.h>
#include <nuttx/usb/composite.h>

#if defined(CONFIG_BOARDCTL_USBDEVCTRL) && defined(CONFIG_USBDEV_COMPOSITE)

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_USBMSC_COMPOSITE
static void *g_mschandle;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_mscclassobject
 *
 * Description:
 *   If the mass storage class driver is part of composite device, then
 *   its instantiation and configuration is a multi-step, board-specific,
 *   process (See comments for usbmsc_configure below).  In this case,
 *   board-specific logic must provide board_mscclassobject().
 *
 *   board_mscclassobject() is called from the composite driver.  It must
 *   encapsulate the instantiation and configuration of the mass storage
 *   class and the return the mass storage device's class driver instance
 *   to the composite driver.
 *
 * Input Parameters:
 *   classdev - The location to return the mass storage class' device
 *     instance.
 *
 * Returned Value:
 *   0 on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_USBMSC_COMPOSITE
static int board_mscclassobject(int minor,
                                struct usbdev_devinfo_s *devinfo,
                                struct usbdevclass_driver_s **classdev)
{
  int ret;

  DEBUGASSERT(g_mschandle == NULL);

  /* Configure the mass storage device */

  uinfo("Configuring with NLUNS=1\n");
  ret = usbmsc_configure(1, &g_mschandle);
  if (ret < 0)
    {
      uerr("ERROR: usbmsc_configure failed: %d\n", -ret);
      return ret;
    }

  uinfo("MSC handle=%p\n", g_mschandle);

  /* Bind the LUN(s) */

  uinfo("Bind LUN=0 to /dev/mmcsd0\n");
  ret = usbmsc_bindlun(g_mschandle, "/dev/mmcsd0", 0, 0, 0, false);
  if (ret < 0)
    {
      uerr("ERROR: usbmsc_bindlun failed for LUN 1 at /dev/mmcsd0: %d\n",
           ret);
      usbmsc_uninitialize(g_mschandle);
      g_mschandle = NULL;
      return ret;
    }

  /* Get the mass storage device's class object */

  ret = usbmsc_classobject(g_mschandle, devinfo, classdev);
  if (ret < 0)
    {
      uerr("ERROR: usbmsc_classobject failed: %d\n", -ret);
      usbmsc_uninitialize(g_mschandle);
      g_mschandle = NULL;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: board_mscuninitialize
 *
 * Description:
 *   Un-initialize the USB storage class driver.
 *   This is just an application specific wrapper for usbmsc_unitialize()
 *   that is called form the composite device logic.
 *
 * Input Parameters:
 *   classdev - The class driver instrance previously give to the composite
 *     driver by board_mscclassobject().
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_USBMSC_COMPOSITE
static void board_mscuninitialize(struct usbdevclass_driver_s *classdev)
{
  if (g_mschandle)
    {
      usbmsc_uninitialize(g_mschandle);
    }

  g_mschandle = NULL;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_composite_initialize
 *
 * Description:
 *   Perform architecture specific initialization of a composite USB device.
 *
 ****************************************************************************/

int board_composite_initialize(int port)
{
  return OK;
}

/****************************************************************************
 * Name:  board_composite_connect
 *
 * Description:
 *   Connect the USB composite device on the specified USB device port using
 *   the specified configuration.  The interpretation of the configid is
 *   board specific.
 *
 * Input Parameters:
 *   port     - The USB device port.
 *   configid - The USB composite configuration
 *
 * Returned Value:
 *   A non-NULL handle value is returned on success.  NULL is returned on
 *   any failure.
 *
 ****************************************************************************/

void *board_composite_connect(int port, int configid)
{
  /* Here we are composing the configuration of the usb composite device.
   *
   * The standard is to use one CDC/ACM and one USB mass storage device.
   */

  if (configid == 0)
    {
      struct composite_devdesc_s dev[2];
      int ifnobase = 0;
      int strbase  = COMPOSITE_NSTRIDS;
      int n = 0;

#ifdef CONFIG_USBMSC_COMPOSITE
      /* Configure the mass storage device device */

      /* Ask the usbmsc driver to fill in the constants we didn't
       * know here.
       */

      usbmsc_get_composite_devdesc(&dev[n]);

      /* Overwrite and correct some values... */

      /* The callback functions for the USBMSC class */

      dev[n].classobject  = board_mscclassobject;
      dev[n].uninitialize = board_mscuninitialize;

      /* Interfaces */

      dev[n].devinfo.ifnobase = ifnobase;               /* Offset to Interface-IDs */
      dev[n].minor = 0;                                 /* The minor interface number */

      /* Strings */

      dev[n].devinfo.strbase = strbase;                 /* Offset to String Numbers */

      /* Endpoints */

      dev[n].devinfo.epno[USBMSC_EP_BULKIN_IDX]  = 1;
      dev[n].devinfo.epno[USBMSC_EP_BULKOUT_IDX] = 2;

      /* Count up the base numbers */

      ifnobase += dev[n].devinfo.ninterfaces;
      strbase  += dev[n].devinfo.nstrings;
      n++;
#endif

#ifdef CONFIG_CDCACM_COMPOSITE
      /* Configure the CDC/ACM device */

      /* Ask the cdcacm driver to fill in the constants we didn't
       * know here.
       */

      cdcacm_get_composite_devdesc(&dev[n]);

      /* Overwrite and correct some values... */

      /* The callback functions for the CDC/ACM class */

      dev[n].classobject  = cdcacm_classobject;
      dev[n].uninitialize = cdcacm_uninitialize;

      /* Interfaces */

      dev[n].devinfo.ifnobase = ifnobase;             /* Offset to Interface-IDs */
      dev[n].minor = 0;                               /* The minor interface number */

      /* Strings */

      dev[n].devinfo.strbase = strbase;               /* Offset to String Numbers */

      /* Endpoints */

      dev[n].devinfo.epno[CDCACM_EP_INTIN_IDX]   = 3;
      dev[n].devinfo.epno[CDCACM_EP_BULKIN_IDX]  = 4;
      dev[n].devinfo.epno[CDCACM_EP_BULKOUT_IDX] = 5;
      n++;
#endif

      return composite_initialize(composite_getdevdescs(), dev, n);
    }
  else
    {
      return NULL;
    }
}

#endif /* CONFIG_BOARDCTL_USBDEVCTRL && CONFIG_USBDEV_COMPOSITE */
