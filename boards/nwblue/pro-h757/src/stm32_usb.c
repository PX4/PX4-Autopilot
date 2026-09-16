/****************************************************************************
 * boards/arm/stm32h7/stm32h747i-disco/src/stm32_usb.c
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
#include <stdint.h>
#include <stdbool.h>
#include <sched.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kthread.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbdev_trace.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_otg.h"
//#include "stm32h747i-disco.h"

#if defined(CONFIG_STM32H7_OTGHS) || defined(CONFIG_STM32H7_OTGFS)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_USBDEV) || defined(CONFIG_USBHOST)
#  define HAVE_USB 1
#else
#  warning "CONFIG_STM32H7_OTGHS is enabled but neither CONFIG_USBDEV nor CONFIG_USBHOST"
#  undef HAVE_USB
#endif

#ifndef CONFIG_STM32H747XI_DISCO_USBHOST_PRIO
#  define CONFIG_STM32H747XI_DISCO_USBHOST_PRIO 100
#endif

#ifndef CONFIG_STM32H747XI_DISCO_USBHOST_STACKSIZE
#  define CONFIG_STM32H747XI_DISCO_USBHOST_STACKSIZE 1024
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_USBHOST
static struct usbhost_connection_s *g_usbconn;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_waiter
 *
 * Description:
 *   Wait for USB devices to be connected.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST
static int usbhost_waiter(int argc, char *argv[])
{
	struct usbhost_hubport_s *hport;

	uinfo("Running\n");

	for (; ;) {
		/* Wait for the device to change state */

		DEBUGVERIFY(CONN_WAIT(g_usbconn, &hport));
		uinfo("%s\n", hport->connected ? "connected" : "disconnected");

		/* Did we just become connected? */

		if (hport->connected) {
			/* Yes.. enumerate the newly connected device */

			(void)CONN_ENUMERATE(g_usbconn, hport);
		}
	}

	/* Keep the compiler from complaining */

	return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called from stm32_usbinitialize very early in inialization to setup
 *   USB-related GPIO pins for the STM32H747I DISCO board.
 *
 ****************************************************************************/

void stm32_usbinitialize(void)
{
	/* Configure the Overcurrent GPIO */

#if defined(CONFIG_STM32H7_OTGHS) || defined(CONFIG_STM32H7_OTGFS)
#ifdef GPIO_OTGHS_OVER
	stm32_configgpio(GPIO_OTGHS_OVER);
#endif
#endif
}

/****************************************************************************
 * Name: stm32_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality.  This function will start a thread that will monitor for
 *   device connection/disconnection events.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST
int stm32_usbhost_initialize(void)
{
	int ret;

	/* First, register all of the class drivers needed to support the drivers
	 * that we care about:
	 */

	uinfo("Register class drivers\n");

#ifdef CONFIG_USBHOST_HUB
	/* Initialize USB hub class support */

	ret = usbhost_hub_initialize();

	if (ret < 0) {
		uerr("ERROR: usbhost_hub_initialize failed: %d\n", ret);
	}

#endif

#ifdef CONFIG_USBHOST_MSC
	/* Register the USB mass storage class class */

	ret = usbhost_msc_initialize();

	if (ret != OK) {
		uerr("ERROR: Failed to register the mass storage class: %d\n", ret);
	}

#endif

#ifdef CONFIG_USBHOST_PTP
	/* Register the PTP class */

	ret = usbhost_ptp_initialize();

	if (ret != OK) {
		uerr("ERROR: Failed to register the PTP class: %d\n", ret);
	}

#endif


#ifdef CONFIG_USBHOST_CDCACM
	/* Register the CDC/ACM serial class */

	ret = usbhost_cdcacm_initialize();

	if (ret != OK) {
		uerr("ERROR: Failed to register the CDC/ACM serial class: %d\n", ret);
	}

#endif

#ifdef CONFIG_USBHOST_HIDKBD
	/* Initialize the HID keyboard class */

	ret = usbhost_kbdinit();

	if (ret != OK) {
		uerr("ERROR: Failed to register the HID keyboard class\n");
	}

#endif

#ifdef CONFIG_USBHOST_HIDMOUSE
	/* Initialize the HID mouse class */

	ret = usbhost_mouse_init();

	if (ret != OK) {
		uerr("ERROR: Failed to register the HID mouse class\n");
	}

#endif

	/* Then get an instance of the USB host interface */

	uinfo("Initialize USB host\n");
	g_usbconn = stm32_otgfshost_initialize(0);

	if (g_usbconn) {
		/* Start a thread to handle device connection. */

		uinfo("Start usbhost_waiter\n");

		ret = kthread_create("usbhost", CONFIG_STM32H747XI_DISCO_USBHOST_PRIO,
				     CONFIG_STM32H747XI_DISCO_USBHOST_STACKSIZE,
				     (main_t)usbhost_waiter, (char *const *)NULL);
		return ret < 0 ? -ENOEXEC : OK;
	}

	return -ENODEV;
}
#endif

/****************************************************************************
 * Name: stm32_usbhost_vbusdrive
 *
 * Description:
 *   Enable/disable driving of VBUS 5V output.  This function must be
 *   provided be each platform that implements the STM32 OTG HS host
 *   interface.
 *
 *   "On-chip 5 V VBUS generation is not supported. For this reason, a
 *    charge pump or, if 5 V are available on the application board, a basic
 *    power switch, must be added externally to drive the 5 V VBUS line.
 *    The external charge pump can be driven by any GPIO output.  When the
 *    application decides to power on VBUS using the chosen GPIO, it must
 *    also set the port power bit in the host port control and status
 *    register (PPWR bit in OTG_FS_HPRT).
 *
 *   "The application uses this field to control power to this port, and the
 *    core clears this bit on an overcurrent condition."
 *
 * Input Parameters:
 *   iface  - For future growth to handle multiple USB host interface.
 *            Should be zero.
 *   enable - true: enable VBUS power; false: disable VBUS power
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST
void stm32_usbhost_vbusdrive(int iface, bool enable)
{
	DEBUGASSERT(iface == 0);
}
#endif

/****************************************************************************
 * Name: stm32_setup_overcurrent
 *
 * Description:
 *   Setup to receive an interrupt-level callback if an over-current
 *   condition is detected.
 *
 * Input Parameters:
 *   handler - New over-current interrupt handler
 *   arg     - The argument provided for the interrupt handler
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST
int stm32_setup_overcurrent(xcpt_t handler, void *arg)
{
#ifdef GPIO_OTGHS_OVER
	return stm32_gpiosetevent(GPIO_OTGHS_OVER, true, true, true, handler, arg);
#else
	return 0;
#endif
}
#endif

/****************************************************************************
 * Name:  stm32_usbsuspend
 *
 * Description:
 *   Board logic must provide the stm32_usbsuspend logic if the USBDEV
 *   driver is used.  This function is called whenever the USB enters or
 *   leaves suspend mode.  This is an opportunity for the board logic to
 *   shutdown clocks, power, etc.  while the USB is suspended.
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV
void stm32_usbsuspend(struct usbdev_s *dev, bool resume)
{
	uinfo("resume: %d\n", resume);
}
#endif

#endif /* CONFIG_STM32_OTGHS */
