/****************************************************************************
 *
 *   Copyright (c) 2023 Technology Innovation Institute. All rights reserved.
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
 * @file netconfig.cpp
 * Simple network configuration
 *
 * @author Jukka Laitinen
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <netinet/in.h>
#include <netutils/netlib.h>
#include <lib/parameters/param.h>

__BEGIN_DECLS
__EXPORT int  netconfig_main(int argc, char *argv[]);
__END_DECLS

int netconfig_main(int argc, char *argv[])
{
	struct in_addr addr;
	int mav_id;
	const char *ifname = argv[1];

	if (argc != 2) {
		return PX4_ERROR;
	}

	param_get(param_find("MAV_SYS_ID"), &mav_id);

	if (mav_id < 1) {
		return PX4_ERROR;
	}

	/* IP: 192.168.201.100 + mav_id */

	addr.s_addr = 0x00c9a8c0;

	mav_id += 100;

	if (mav_id > 253) {
		return PX4_ERROR;
	}

	addr.s_addr |= ((uint32_t)mav_id << 24);
	netlib_set_ipv4addr(ifname, &addr);

	/* GW: 192.168.201.1 */

	addr.s_addr = 0x01c9a8c0;
	netlib_set_dripv4addr(ifname, &addr);

	/* netmask: 255.255.255.0 */

	addr.s_addr = 0x00ffffff;
	netlib_set_ipv4netmask(ifname, &addr);

	netlib_ifup(ifname);

	return PX4_OK;
}
