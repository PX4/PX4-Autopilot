/************************************************************************************
 * configs/cloudctrl/src/up_usbdev.c
 * arch/arm/src/board/up_boot.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Darcy Gong <darcy.gong@gmail.com>
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbdev_trace.h>

#include "up_arch.h"
#include "stm32_internal.h"
#include "cloudctrl-internal.h"

#ifdef CONFIG_STM32_OTGFS

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#if defined(CONFIG_USBDEV) || defined(CONFIG_USBHOST)
#  define HAVE_USB 1
#else
#  warning "CONFIG_STM32_OTGFS is enabled but neither CONFIG_USBDEV nor CONFIG_USBHOST"
#  undef HAVE_USB
#endif

#ifndef CONFIG_USBHOST_DEFPRIO
#  define CONFIG_USBHOST_DEFPRIO 50
#endif

#ifndef CONFIG_USBHOST_STACKSIZE
#  define CONFIG_USBHOST_STACKSIZE 1024
#endif

/************************************************************************************
 * Private Data
 ************************************************************************************/

#ifdef CONFIG_USBHOST
static struct usbhost_driver_s *g_drvr;
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: usbhost_waiter
 *
 * Description:
 *   Wait for USB devices to be connected.
 *
 ************************************************************************************/

#ifdef CONFIG_USBHOST
static int usbhost_waiter(int argc, char *argv[])
{
  bool connected = false;
  int ret;

  uvdbg("Running\n");
  for (;;)
    {
      /* Wait for the device to change state */

      ret = DRVR_WAIT(g_drvr, connected);
      DEBUGASSERT(ret == OK);

      connected = !connected;
      uvdbg("%s\n", connected ? "connected" : "disconnected");

      /* Did we just become connected? */

      if (connected)
        {
          /* Yes.. enumerate the newly connected device */

          (void)DRVR_ENUMERATE(g_drvr);
        }
    }

  /* Keep the compiler from complaining */

  return 0;
}
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called from stm32_usbinitialize very early in inialization to setup USB-related
 *   GPIO pins for the STM3240G-EVAL board.
 *
 ************************************************************************************/

void stm32_usbinitialize(void)
{
  /* The OTG FS has an internal soft pull-up.  No GPIO configuration is required */

  /* Configure the OTG FS VBUS sensing GPIO, Power On, and Overcurrent GPIOs */

#ifdef CONFIG_STM32_OTGFS
  stm32_configgpio(GPIO_OTGFS_VBUS);
  stm32_configgpio(GPIO_OTGFS_PWRON);
  stm32_configgpio(GPIO_OTGFS_OVER);
#endif
}

/***********************************************************************************
 * Name: stm32_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host functionality.
 *   This function will start a thread that will monitor for device
 *   connection/disconnection events.
 *
 ***********************************************************************************/

#ifdef CONFIG_USBHOST
int stm32_usbhost_initialize(void)
{
  int pid;
  int ret;

  /* First, register all of the class drivers needed to support the drivers
   * that we care about:
   */

  uvdbg("Register class drivers\n");
  ret = usbhost_storageinit();
  if (ret != OK)
    {
      udbg("Failed to register the mass storage class\n");
    }

  /* Then get an instance of the USB host interface */

  uvdbg("Initialize USB host\n");
  g_drvr = usbhost_initialize(0);
  if (g_drvr)
    {
      /* Start a thread to handle device connection. */

      uvdbg("Start usbhost_waiter\n");

      pid = TASK_CREATE("usbhost", CONFIG_USBHOST_DEFPRIO,
                        CONFIG_USBHOST_STACKSIZE,
                        (main_t)usbhost_waiter, (const char **)NULL);
      return pid < 0 ? -ENOEXEC : OK;
    }

  return -ENODEV;
}
#endif

/***********************************************************************************
 * Name: stm32_usbhost_vbusdrive
 *
 * Description:
 *   Enable/disable driving of VBUS 5V output.  This function must be provided be
 *   each platform that implements the STM32 OTG FS host interface
 *
 *   "On-chip 5 V VBUS generation is not supported. For this reason, a charge pump 
 *    or, if 5 V are available on the application board, a basic power switch, must 
 *    be added externally to drive the 5 V VBUS line. The external charge pump can 
 *    be driven by any GPIO output. When the application decides to power on VBUS 
 *    using the chosen GPIO, it must also set the port power bit in the host port 
 *    control and status register (PPWR bit in OTG_FS_HPRT).
 *
 *   "The application uses this field to control power to this port, and the core 
 *    clears this bit on an overcurrent condition."
 *
 * Input Parameters:
 *   iface - For future growth to handle multiple USB host interface.  Should be zero.
 *   enable - true: enable VBUS power; false: disable VBUS power
 *
 * Returned Value:
 *   None
 *
 ***********************************************************************************/

#ifdef CONFIG_USBHOST
void stm32_usbhost_vbusdrive(int iface, bool enable)
{
  DEBUGASSERT(iface == 0);
  
  if (enable)
    {
      /* Enable the Power Switch by driving the enable pin low */

      stm32_gpiowrite(GPIO_OTGFS_PWRON, false);
    }
  else
    { 
      /* Disable the Power Switch by driving the enable pin high */
 
      stm32_gpiowrite(GPIO_OTGFS_PWRON, true);
    }
}
#endif

/************************************************************************************
 * Name: stm32_setup_overcurrent
 *
 * Description:
 *   Setup to receive an interrupt-level callback if an overcurrent condition is
 *   detected.
 *
 * Input paramter:
 *   handler - New overcurrent interrupt handler
 *
 * Returned value:
 *   Old overcurrent interrupt handler
 *
 ************************************************************************************/

#ifdef CONFIG_USBHOST
xcpt_t stm32_setup_overcurrent(xcpt_t handler)
{
  return NULL;
}
#endif

/************************************************************************************
 * Name:  stm32_usbsuspend
 *
 * Description:
 *   Board logic must provide the stm32_usbsuspend logic if the USBDEV driver is
 *   used.  This function is called whenever the USB enters or leaves suspend mode.
 *   This is an opportunity for the board logic to shutdown clocks, power, etc.
 *   while the USB is suspended.
 *
 ************************************************************************************/

#ifdef CONFIG_USBDEV
void stm32_usbsuspend(FAR struct usbdev_s *dev, bool resume)
{
  ulldbg("resume: %d\n", resume);
}
#endif

#endif /* CONFIG_STM32_OTGFS */



