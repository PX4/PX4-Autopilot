/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * Dual CDC/ACM composite: /dev/ttyACM0 (MAVLink) and /dev/ttyACM1 (SLCAN).
 *
 * STM32H7 OTG FS has 7 endpoint numbers (EP0-6). Each CDC/ACM uses 3
 * (interrupt IN, bulk IN, bulk OUT).
 */

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/usb/cdcacm.h>
#include <nuttx/usb/composite.h>
#include <nuttx/usb/usbdev.h>

#if defined(CONFIG_BOARDCTL_USBDEVCTRL) && defined(CONFIG_USBDEV_COMPOSITE) && \
	defined(CONFIG_CDCACM_COMPOSITE)

int board_composite_initialize(int port)
{
	(void)port;
	return OK;
}

void *board_composite_connect(int port, int configid)
{
	struct composite_devdesc_s dev[2];
	int ifnobase = 0;
	int strbase = COMPOSITE_NSTRIDS;

	(void)port;
	(void)configid;

	/* ttyACM0 — MAVLink / QGC */

	cdcacm_get_composite_devdesc(&dev[0]);
	dev[0].classobject  = cdcacm_classobject;
	dev[0].uninitialize = cdcacm_uninitialize;
	dev[0].minor = 0;
	dev[0].devinfo.ifnobase = ifnobase;
	dev[0].devinfo.strbase  = strbase;
	dev[0].devinfo.epno[CDCACM_EP_INTIN_IDX]   = 1;
	dev[0].devinfo.epno[CDCACM_EP_BULKIN_IDX]  = 2;
	dev[0].devinfo.epno[CDCACM_EP_BULKOUT_IDX] = 3;
	ifnobase += dev[0].devinfo.ninterfaces;
	strbase  += dev[0].devinfo.nstrings;

	/* ttyACM1 — SLCAN / DroneCAN GUI Tool */

	cdcacm_get_composite_devdesc(&dev[1]);
	dev[1].classobject  = cdcacm_classobject;
	dev[1].uninitialize = cdcacm_uninitialize;
	dev[1].minor = 1;
	dev[1].devinfo.ifnobase = ifnobase;
	dev[1].devinfo.strbase  = strbase;
	dev[1].devinfo.epno[CDCACM_EP_INTIN_IDX]   = 4;
	dev[1].devinfo.epno[CDCACM_EP_BULKIN_IDX]  = 5;
	dev[1].devinfo.epno[CDCACM_EP_BULKOUT_IDX] = 6;

	return composite_initialize(2, dev);
}

#endif
