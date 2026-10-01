/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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

/**
 * @file usb.c
 *
 * Board-specific USB functions.
 */

#include "board_config.h"
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>
#include <stm32_otg.h>
#include <debug.h>
#include <syslog.h>

#ifdef CONFIG_USBDEV_COMPOSITE
#  include <nuttx/usb/cdcacm.h>
#  include <nuttx/usb/composite.h>
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
__EXPORT void stm32_usbsuspend(FAR struct usbdev_s *dev, bool resume)
{
	uinfo("resume: %d\n", resume);
}

int board_read_VBUS_state(void)
{
	return BOARD_ADC_USB_VALID ? 0 : -1;
}

#ifdef CONFIG_USBDEV_COMPOSITE
/****************************************************************************
 * Composite USB device: two CDC/ACM interfaces.
 *
 * The CubeRed primary bootloader exposes two USB serial ports:
 *   /dev/ttyACM0 - the PX4 bootloader protocol (this MCU)
 *   /dev/ttyACM1 - a transparent passthrough to the secondary MCU's
 *                  bootloader over UART7, so the host can flash the secondary
 *                  through the primary's USB.
 *
 * The STM32H7 OTG-FS has 7 endpoints (EP0-6); two CDC/ACM functions use
 * exactly six (interrupt-in, bulk-in, bulk-out each), so the endpoint numbers
 * are assigned 1-3 for the first and 4-6 for the second function.
 ****************************************************************************/

static void cubered_cdcacm_devdesc(struct composite_devdesc_s *dev, int minor,
				   int ifnobase, int strbase, int epbase)
{
	cdcacm_get_composite_devdesc(dev);
	dev->classobject  = cdcacm_classobject;
	dev->uninitialize = cdcacm_uninitialize;
	dev->minor = minor;
	dev->devinfo.ifnobase = ifnobase;
	dev->devinfo.strbase  = strbase;
	dev->devinfo.epno[CDCACM_EP_INTIN_IDX]   = epbase;
	dev->devinfo.epno[CDCACM_EP_BULKIN_IDX]  = epbase + 1;
	dev->devinfo.epno[CDCACM_EP_BULKOUT_IDX] = epbase + 2;
}

int board_composite_initialize(int port)
{
	return OK;
}

FAR void *board_composite_connect(int port, int configid)
{
	if (configid != 0) {
		return NULL;
	}

	struct composite_devdesc_s dev[2];

	int ifnobase = 0;

	int strbase  = COMPOSITE_NSTRIDS;

	/* First CDC/ACM -> /dev/ttyACM0 (bootloader protocol), endpoints 1-3. */
	cubered_cdcacm_devdesc(&dev[0], 0, ifnobase, strbase, 1);

	ifnobase += dev[0].devinfo.ninterfaces;

	strbase  += dev[0].devinfo.nstrings;

	/* Second CDC/ACM -> /dev/ttyACM1 (passthrough to secondary), endpoints 4-6. */
	cubered_cdcacm_devdesc(&dev[1], 1, ifnobase, strbase, 4);

	ifnobase += dev[1].devinfo.ninterfaces;

	strbase  += dev[1].devinfo.nstrings;

	return composite_initialize(2, dev);
}
#endif /* CONFIG_USBDEV_COMPOSITE */
