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

#pragma once

#include <px4_config.h>					//Configuration flags used in code
#include <px4_defines.h>				//Generally used magic defines
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <uORB/uORB.h>					//API for the uORB lightweight object broker
#include <systemlib/err.h>				//Simple error/warning functions
#include <arpa/inet.h>
#include <netinet/in.h>
#include "netutils/netlib.h"

struct network_manager_param_handles {
	param_t ip_addr;
	param_t sub_pref_s;
	param_t gateway;
	param_t mac_addr_0, mac_addr_1, mac_addr_2, mac_addr_3, mac_addr_4, mac_addr_5;
};	//network parameters handles

struct network_manager_params {
	int32_t ip_addr;
	int32_t sub_pref_s;
	int32_t gateway;
	int32_t mac_addr[6];
};	//network attributes


class NetworkManager
{
private:
	NetworkManager() {;}

	NetworkManager(const NetworkManager &);
	NetworkManager operator=(const NetworkManager &);

	void param_init(struct network_manager_param_handles *h);
	void param_update(const struct network_manager_param_handles *h, struct network_manager_params *p);
	void set_mac_addr(const struct network_manager_params *p);
	void set_ip_addr(const struct network_manager_params *p);
	void set_netmask_addr(const struct network_manager_params *p);
	void set_gateway_addr(const struct network_manager_params *p);

public:
	~NetworkManager() {;}

	static void start();
};
