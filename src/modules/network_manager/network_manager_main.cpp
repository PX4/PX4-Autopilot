/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "network_manager_main.h"

#define NET_DEVNAME "eth0"

extern "C" __EXPORT int network_manager_main(int argc, char *argv[]);

//Network_manager class single instance
NetworkManager *_network_manager_instance = nullptr;


void NetworkManager::start()
{
	_network_manager_instance = new NetworkManager();

	if (_network_manager_instance == nullptr) {
		warnx("Out of memory");

	} else {
		struct network_manager_param_handles network_manager_param_handles = {0};
		struct network_manager_params network_manager_params = {0};

		_network_manager_instance->param_init(&network_manager_param_handles);
		_network_manager_instance->param_update(&network_manager_param_handles, &network_manager_params);

		_network_manager_instance->set_gateway_addr(&network_manager_params);
		_network_manager_instance->set_ip_addr(&network_manager_params);
		_network_manager_instance->set_netmask_addr(&network_manager_params);

		//set mac address will only take effect after ifup
		netlib_ifdown(NET_DEVNAME);
		_network_manager_instance->set_mac_addr(&network_manager_params);
		netlib_ifup(NET_DEVNAME);
	}

	if (_network_manager_instance != nullptr) {
		delete _network_manager_instance;
		_network_manager_instance = nullptr;
	}
}

//get handles to network parameters
void NetworkManager::param_init(struct network_manager_param_handles *h)
{
	h->ip_addr = param_find("ETH_IP_ADDR");

	h->sub_pref_s = param_find("ETH_SUB_PREF_S");

	h->gateway = param_find("ETH_GATEWAY");

	h->mac_addr_0 = param_find("ETH_MAC_ADDR_0");
	h->mac_addr_1 = param_find("ETH_MAC_ADDR_1");
	h->mac_addr_2 = param_find("ETH_MAC_ADDR_2");
	h->mac_addr_3 = param_find("ETH_MAC_ADDR_3");
	h->mac_addr_4 = param_find("ETH_MAC_ADDR_4");
	h->mac_addr_5 = param_find("ETH_MAC_ADDR_5");
}

//update attributes according to network parameters
void NetworkManager::param_update(const struct network_manager_param_handles *h, struct network_manager_params *p)
{
	param_get(h->ip_addr, &(p->ip_addr));

	param_get(h->sub_pref_s, &(p->sub_pref_s));

	param_get(h->gateway, &(p->gateway));

	param_get(h->mac_addr_0, &(p->mac_addr[0]));
	param_get(h->mac_addr_1, &(p->mac_addr[1]));
	param_get(h->mac_addr_2, &(p->mac_addr[2]));
	param_get(h->mac_addr_3, &(p->mac_addr[3]));
	param_get(h->mac_addr_4, &(p->mac_addr[4]));
	param_get(h->mac_addr_5, &(p->mac_addr[5]));
}

void NetworkManager::set_mac_addr(const struct network_manager_params *p)
{
	uint8_t u8mac_addr[6];

	u8mac_addr[0] = p->mac_addr[0] & 0xFF;
	u8mac_addr[1] = p->mac_addr[1] & 0xFF;
	u8mac_addr[2] = p->mac_addr[2] & 0xFF;
	u8mac_addr[3] = p->mac_addr[3] & 0xFF;
	u8mac_addr[4] = p->mac_addr[4] & 0xFF;
	u8mac_addr[5] = p->mac_addr[5] & 0xFF;

	netlib_setmacaddr(NET_DEVNAME, u8mac_addr);
}

void NetworkManager::set_ip_addr(const struct network_manager_params *p)
{
	struct in_addr addr;

	addr.s_addr = HTONL(p->ip_addr);
	netlib_set_ipv4addr(NET_DEVNAME, &addr);
}

void NetworkManager::set_netmask_addr(const struct network_manager_params *p)
{
	struct in_addr addr;

	uint32_t unetmask = ~(0xFFFFFFFF & ((1 << (32 - p->sub_pref_s)) - 1));

	addr.s_addr = HTONL(unetmask);
	netlib_set_ipv4netmask(NET_DEVNAME, &addr);
}

void NetworkManager::set_gateway_addr(const struct network_manager_params *p)
{
	struct in_addr addr;

	addr.s_addr = HTONL(p->gateway);
	netlib_set_dripv4addr(NET_DEVNAME, &addr);
}

int network_manager_main(int argc, char *argv[])
{
	//No command
	if (argc < 2) {
		PX4_INFO("usage : Network manager {start|status}");
		return 1;
	}

	if (!strcmp(argv[1], "status")) {
		if (!_network_manager_instance) {
			PX4_INFO("Network manager not running");
			return 1;
		}

		PX4_INFO("Network manager is running");
		return 0;

	} else if (!strcmp(argv[1], "start")) {
		if (_network_manager_instance) {
			PX4_INFO("Network manager already running");
			return 1;
		}

		NetworkManager::start();

		return 0;
	}

	//Unrecognized command
	PX4_INFO("usage : Network manager {start|status}");
	return 1;
}
