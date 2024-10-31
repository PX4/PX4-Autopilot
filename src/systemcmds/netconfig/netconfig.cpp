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
#include <px4_platform_common/module.h>
#include <px4_platform_common/log.h>
#include <netinet/in.h>
#include <netutils/netlib.h>
#include <lib/parameters/param.h>

__BEGIN_DECLS
__EXPORT int  netconfig_main(int argc, char *argv[]);
__END_DECLS

/* string constants for commands */
static const char sz_nc_help_str[] 	 = "-h";
static const char sz_nc_init_str[] 	 = "init";
static const char sz_nc_get_ipv4_str[]  = "get_ipv4";
static const char sz_nc_set_ipv4_str[]  = "set_ipv4";
static const char sz_nc_get_drip_str[]  = "get_drip";
static const char sz_nc_set_drip_str[]  = "set_drip";
static const char sz_nc_get_mask_str[]  = "get_mask";
static const char sz_nc_set_mask_str[]  = "set_mask";

static void usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_INFO_RAW("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION("Control network configuration");

	PRINT_MODULE_USAGE_NAME("netconfig", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("-h", "Usage info");
	PRINT_MODULE_USAGE_COMMAND_DESCR("init", "Initialize network");
	PRINT_MODULE_USAGE_COMMAND_DESCR("get_ipv4", "Get current ipv4 address");
	PRINT_MODULE_USAGE_COMMAND_DESCR("set_ipv4", "Set current ipv4 address");
	PRINT_MODULE_USAGE_ARG("<addr>", "address as string (e.g. '192.168.0.1')", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("get_drip", "Get drip (gateway) address");
	PRINT_MODULE_USAGE_COMMAND_DESCR("set_drip", "Set drip (gateway) address");
	PRINT_MODULE_USAGE_ARG("<addr>", "address as string (e.g. '192.168.0.1')", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("get_mask", "Get netmask");
	PRINT_MODULE_USAGE_COMMAND_DESCR("set_mask", "Set netmask");
	PRINT_MODULE_USAGE_ARG("<addr>", "address as string (e.g. '255.255.255.0')", false);
}

void netconfig_print_addr(struct in_addr *addr)
{
	char addr_str[INET_ADDRSTRLEN];
	inet_ntop(AF_INET, addr, addr_str, INET_ADDRSTRLEN);
	printf("%s", addr_str);
}

int netconfig_get_ipv4()
{
	struct in_addr addr;
	const char ifname[] = CONFIG_NETCONFIG_IFNAME;
	int ret = netlib_get_ipv4addr(ifname, &addr);

	if (ret == 0) {
		netconfig_print_addr(&addr);
		return PX4_OK;
	}

	PX4_ERR("No ipv4 address found");
	return PX4_ERROR;
}

int netconfig_get_drip()
{
	struct in_addr addr;
	const char ifname[] = CONFIG_NETCONFIG_IFNAME;
	int ret = netlib_get_dripv4addr(ifname, &addr);

	if (ret == 0) {
		netconfig_print_addr(&addr);
		return PX4_OK;
	}

	PX4_ERR("No drip address found");
	return PX4_ERROR;
}

int netconfig_get_mask()
{
	struct in_addr addr;
	const char ifname[] = CONFIG_NETCONFIG_IFNAME;
	int ret = netlib_get_ipv4netmask(ifname, &addr);

	if (ret == 0) {
		netconfig_print_addr(&addr);
		return PX4_OK;
	}

	PX4_ERR("No netmask found");
	return PX4_ERROR;
}

int netconfig_set_ipv4(char *addr_str)
{
	struct in_addr addr;
	const char ifname[] = CONFIG_NETCONFIG_IFNAME;
	inet_pton(AF_INET, addr_str, &addr);
	int ret = netlib_set_ipv4addr(ifname, &addr);

	if (ret == 0) {
		return PX4_OK;
	}

	PX4_ERR("Set drip address failed");
	return PX4_ERROR;
}

int netconfig_set_drip(char *addr_str)
{
	struct in_addr addr;
	const char ifname[] = CONFIG_NETCONFIG_IFNAME;
	inet_pton(AF_INET, addr_str, &addr);
	int ret = netlib_set_dripv4addr(ifname, &addr);

	if (ret == 0) {
		return PX4_OK;
	}

	PX4_ERR("Set drip address failed");
	return PX4_ERROR;
}

int netconfig_set_mask(char *addr_str)
{
	struct in_addr addr;
	const char ifname[] = CONFIG_NETCONFIG_IFNAME;
	inet_pton(AF_INET, addr_str, &addr);
	int ret = netlib_set_ipv4netmask(ifname, &addr);

	if (ret == 0) {
		return PX4_OK;
	}

	PX4_ERR("Set netmask failed");
	return PX4_ERROR;
}


int netconfig_init()
{
	struct in_addr addr;
	int32_t mav_id;
	int32_t mav_comp_id;
	int32_t ip;
	const char ifname[] = CONFIG_NETCONFIG_IFNAME;

	param_get(param_find("MAV_SYS_ID"), &mav_id);
	param_get(param_find("MAV_COMP_ID"), &mav_comp_id);

	if (mav_id < 1 || mav_id > 63 || mav_comp_id < 1 || mav_comp_id > 4) {
		return PX4_ERROR;
	}

	/* IP: CONFIG_NETCONFIG_IPSUBNET 3 bytes
	 * last byte:
	 *   2 high bits: mav_comp_id - 1
	 *   6 low bits: mav_id
	 */

	addr.s_addr = CONFIG_NETCONFIG_IPSUBNET & 0xffffff;

	/* Autopilot IP examples:
	   MAV_ID 1:
	   192.168.202.1 : primary FC1 (comp_id 1)
	   192.168.202.65 : redundant FC2 (comp_id 2)
	   192.168.202.129 : redundant FC3 (comp_id 3)
	   192.168.202.193 : redundant FC4 (comp_id 4)
	   MAV_ID 2:
	   192.168.202.2 : primary FC1 (comp_id 1)
	   192.168.202.66 : redundant FC2 (comp_id 2)
	   192.168.202.130 : redundant FC3 (comp_id 3)
	   192.168.202.194 : redundant FC4 (comp_id 4)
	*/

	mav_comp_id--;
	ip = ((mav_comp_id & 0x3) << 6) + mav_id;

	addr.s_addr |= ip << 24;
	netlib_set_ipv4addr(ifname, &addr);

	/* GW */

	addr.s_addr = CONFIG_NETCONFIG_DRIPADDR;
	netlib_set_dripv4addr(ifname, &addr);

	/* netmask */

	addr.s_addr = CONFIG_NETCONFIG_NETMASK;
	netlib_set_ipv4netmask(ifname, &addr);

	netlib_ifup(ifname);

	return PX4_OK;
}


int netconfig_main(int argc, char *argv[])
{
	if (argc >= 2) {
		if (!strncmp(argv[1], sz_nc_help_str, sizeof(sz_nc_help_str))) {
			usage("");
			return PX4_OK;

		} else if (!strncmp(argv[1], sz_nc_init_str, sizeof(sz_nc_init_str))) {
			return netconfig_init();

		} else if (!strncmp(argv[1], sz_nc_get_ipv4_str, sizeof(sz_nc_get_ipv4_str))) {
			return netconfig_get_ipv4();

		} else if (!strncmp(argv[1], sz_nc_get_drip_str, sizeof(sz_nc_set_drip_str))) {
			return netconfig_get_drip();

		} else if (!strncmp(argv[1], sz_nc_get_mask_str, sizeof(sz_nc_get_mask_str))) {
			return netconfig_get_mask();

		} else if (!strncmp(argv[1], sz_nc_set_ipv4_str, sizeof(sz_nc_set_ipv4_str))) {
			if (argc >= 3) {
				return netconfig_set_ipv4(argv[2]);

			} else {
				usage("Not enough arguments.");
				return PX4_ERROR;
			}

		} else if (!strncmp(argv[1], sz_nc_set_drip_str, sizeof(sz_nc_set_drip_str))) {
			if (argc >= 3) {
				return netconfig_set_drip(argv[2]);

			} else {
				usage("Not enough arguments.");
				return PX4_ERROR;
			}

		} else if (!strncmp(argv[1], sz_nc_set_mask_str, sizeof(sz_nc_set_mask_str))) {
			if (argc >= 3) {
				return netconfig_set_mask(argv[2]);

			} else {
				usage("Not enough arguments.");
				return PX4_ERROR;
			}

		} else {
			usage("Invalid arguments.");
			return PX4_ERROR;
		}

	} else {
		// Backward compatibility, perform init in case of no arguments
		return netconfig_init();
	}

	return PX4_OK;
}
