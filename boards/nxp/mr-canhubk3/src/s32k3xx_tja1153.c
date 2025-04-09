/****************************************************************************
 * boards/arm/s32k3xx/mr-canhubk3/src/s32k3xx_tja1153.c
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

/* Copyright 2022 NXP
 *
 * This TJA1153 initialization routine is intended for ENGINEERING
 * DEVELOPMENT OR EVALUATION PURPOSES ONLY.  It is provided as an example to
 * use the TJA1153.  Please refer to the datasheets and application hints
 * provided on NXP.com to implement full functionality.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <nuttx/can.h>
#include <netpacket/can.h>
#include <nuttx/signal.h>

#include "s32k3xx_pin.h"
#include <arch/board/board.h>
#include "board_config.h"

#ifdef CONFIG_S32K3XX_TJA1153

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Bitrate must be set to 125, 250 or 500 kbit/s for CAN 2.0 and CAN FD
 * arbitration phase
 */

#ifdef CONFIG_S32K3XX_FLEXCAN4
#  if CONFIG_NET_CAN_CANFD
#    if CONFIG_FLEXCAN4_ARBI_BITRATE > 500000
#      error "FLEXCAN4_ARBI_BITRATE > 500000"
#    endif
#  else
#    if CONFIG_FLEXCAN4_BITRATE > 500000
#      error "FLEXCAN4_BITRATE > 500000"
#    endif
#  endif
#endif

#ifdef CONFIG_S32K3XX_FLEXCAN5
#  if CONFIG_NET_CAN_CANFD
#    if CONFIG_FLEXCAN5_ARBI_BITRATE > 500000
#      error "FLEXCAN5_ARBI_BITRATE > 500000"
#    endif
#  else
#    if CONFIG_FLEXCAN5_BITRATE > 500000
#      error "FLEXCAN5_BITRATE > 500000"
#    endif
#  endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k3xx_tja1153_initialize
 *
 * Description:
 *   Initialize a TJA1153 CAN PHY connected to a FlexCAN peripheral (0-5)
 *
 ****************************************************************************/

int s32k3xx_tja1153_initialize(int bus)
{
	int sock;
	struct sockaddr_can addr;
	struct can_frame frame_config1;
	struct can_frame frame_config2;
	struct can_frame frame_config3;
	struct can_frame frame_config4;
	struct can_frame frame_config5;
	struct ifreq ifr;
	uint32_t pin_can_txd;
	uint32_t pin_can_rxd;
	uint32_t pin_can_enable;
	uint32_t pin_can_stb_n;
	int ret = 0;

	/* Select interface and pins */

	switch (bus) {
#ifdef CONFIG_S32K3XX_FLEXCAN4

	case 4: {
			strlcpy(ifr.ifr_name, "can4", IFNAMSIZ);

			pin_can_txd    = PIN_CAN4_TX;
			pin_can_rxd    = PIN_CAN4_RX;
			pin_can_enable = PIN_CAN4_ENABLE;
			pin_can_stb_n  = PIN_CAN4_STB;
		}
		break;
#endif
#ifdef CONFIG_S32K3XX_FLEXCAN5

	case 5: {
			strlcpy(ifr.ifr_name, "can5", IFNAMSIZ);

			pin_can_txd    = PIN_CAN5_TX;
			pin_can_rxd    = PIN_CAN5_RX;
			pin_can_enable = PIN_CAN5_ENABLE;
			pin_can_stb_n  = PIN_CAN5_STB;
		}
		break;
#endif

	default: {
			/* This FlexCAN is not supported (yet) */

			return -1;
		}
	}

	/* First check if configuration is actually needed */

	s32k3xx_pinconfig((pin_can_txd & (_PIN_PORT_MASK | _PIN_MASK)) |
			  GPIO_OUTPUT | GPIO_OUTPUT_ZERO);

	if (s32k3xx_gpioread(pin_can_rxd)) {
		_info("CAN%d TJA1153 already configured\n", bus);

		s32k3xx_pinconfig(pin_can_txd); /* Restore CAN_TXD pinconfig */
		return 0;
	}

	s32k3xx_pinconfig(pin_can_txd); /* Restore CAN_TXD pinconfig */

	/* Find network interface */

	ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);

	if (!ifr.ifr_ifindex) {
		_err("CAN%d TJA1153: if_nametoindex failed\n", bus);
		return -1;
	}

	/* Configure pins */

	s32k3xx_pinconfig(pin_can_enable);
	s32k3xx_pinconfig(pin_can_stb_n);

	s32k3xx_gpiowrite(pin_can_enable, true); /* Enable TJA1153 */
	s32k3xx_gpiowrite(pin_can_stb_n, false); /* Inverted, so TJA1153 is put in STANDBY */

	/* Init CAN frames, e.g. LEN = 0 */

	memset(&frame_config1, 0, sizeof(frame_config1));
	memset(&frame_config2, 0, sizeof(frame_config2));
	memset(&frame_config3, 0, sizeof(frame_config3));
	memset(&frame_config4, 0, sizeof(frame_config4));
	memset(&frame_config5, 0, sizeof(frame_config5));

	/* Prepare CAN frames. Refer to the TJA1153 datasheets and application
	 * hints available on NXP.com for details.
	 */

	frame_config1.can_id  = 0x555;
	frame_config1.can_dlc = 0;

	frame_config2.can_id  = 0x18da00f1 | CAN_EFF_FLAG;
	frame_config2.can_dlc = 6;
	frame_config2.data[0] = 0x10;
	frame_config2.data[1] = 0x00;
	frame_config2.data[2] = 0x50;
	frame_config2.data[3] = 0x00;
	frame_config2.data[4] = 0x07;
	frame_config2.data[5] = 0xff;

	frame_config3.can_id  = 0x18da00f1 | CAN_EFF_FLAG;
	frame_config3.can_dlc = 6;
	frame_config3.data[0] = 0x10;
	frame_config3.data[1] = 0x01;
	frame_config3.data[2] = 0x9f;
	frame_config3.data[3] = 0xff;
	frame_config3.data[4] = 0xff;
	frame_config3.data[5] = 0xff;

	frame_config4.can_id  = 0x18da00f1 | CAN_EFF_FLAG;
	frame_config4.can_dlc = 6;
	frame_config4.data[0] = 0x10;
	frame_config4.data[1] = 0x02;
	frame_config4.data[2] = 0xc0;
	frame_config4.data[3] = 0x00;
	frame_config4.data[4] = 0x00;
	frame_config4.data[5] = 0x00;

	frame_config5.can_id  = 0x18da00f1 | CAN_EFF_FLAG;
	frame_config5.can_dlc = 8;
	frame_config5.data[0] = 0x71;
	frame_config5.data[1] = 0x02;
	frame_config5.data[2] = 0x03;
	frame_config5.data[3] = 0x04;
	frame_config5.data[4] = 0x05;
	frame_config5.data[5] = 0x06;
	frame_config5.data[6] = 0x07;
	frame_config5.data[7] = 0x08;

	/* Open socket */

	if ((sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		_err("CAN%d TJA1153: Failed to open socket\n", bus);
		return -1;
	}

	/* Bring up the interface */

	ifr.ifr_flags = IFF_UP;
	ret = ioctl(sock, SIOCSIFFLAGS, (unsigned long)&ifr);

	if (ret < 0) {
		_err("CAN%d TJA1153: ioctl failed (can't set interface flags)\n", bus);
		close(sock);
		return -1;
	}

	/* Initialize sockaddr struct */

	memset(&addr, 0, sizeof(addr));
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	/* Disable default receive filter on this RAW socket
	 *
	 * This is obsolete as we do not read from the socket at all, but for this
	 * reason we can remove the receive list in the kernel to save a little
	 * (really very little!) CPU usage.
	 */

	setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

	/* Bind socket and send the CAN frames */

	if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		_err("CAN%d TJA1153: Failed to bind socket\n", bus);
		close(sock);
		return -1;
	}

	if (write(sock, &frame_config1, CAN_MTU) != CAN_MTU) {
		_err("CAN%d TJA1153: Failed to write frame_config1\n", bus);
		close(sock);
		return -1;
	}

	if (write(sock, &frame_config2, CAN_MTU) != CAN_MTU) {
		_err("CAN%d TJA1153: Failed to write frame_config2\n", bus);
		close(sock);
		return -1;
	}

	if (write(sock, &frame_config3, CAN_MTU) != CAN_MTU) {
		_err("CAN%d TJA1153: Failed to write frame_config3\n", bus);
		close(sock);
		return -1;
	}

	if (write(sock, &frame_config4, CAN_MTU) != CAN_MTU) {
		_err("CAN%d TJA1153: Failed to write frame_config4\n", bus);
		close(sock);
		return -1;
	}

	if (write(sock, &frame_config5, CAN_MTU) != CAN_MTU) {
		_err("CAN%d TJA1153: Failed to write frame_config5\n", bus);
		close(sock);
		return -1;
	}

	/* Sleep for 100 ms to ensure that CAN frames have been transmitted */

	nxsig_usleep(100 * 1000);

	/* TJA1153 must be taken out of STB mode */

	s32k3xx_gpiowrite(pin_can_stb_n, true); /* Inverted, so TJA1153 comes out of STANDBY */

	/* Bring down the interface */

	ifr.ifr_flags = IFF_DOWN;
	ret = ioctl(sock, SIOCSIFFLAGS, (unsigned long)&ifr);

	if (ret < 0) {
		_err("CAN%d TJA1153: ioctl failed (can't set interface flags)\n", bus);
		close(sock);
		return -1;
	}

	close(sock);
	_info("CAN%d TJA1153 configuration successful\n", bus);
	return 0;
}

#endif /* CONFIG_S32K3XX_TJA1153 */
