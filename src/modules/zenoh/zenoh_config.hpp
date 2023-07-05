/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file zenoh_config.hpp
 *
 * Defines Zenoh configuration backend
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>

#include <lib/parameters/param.h>
#include <containers/List.hpp>
#include <zenoh-pico.h>

#define ZENOH_MAX_PATH_LENGTH (128 + 40)
#define ZENOH_SD_ROOT_PATH    CONFIG_ZENOH_CONFIG_PATH
#define ZENOH_PUB_CONFIG_PATH ZENOH_SD_ROOT_PATH"/pub.csv"
#define ZENOH_SUB_CONFIG_PATH ZENOH_SD_ROOT_PATH"/sub.csv"
#define ZENOH_NET_CONFIG_PATH ZENOH_SD_ROOT_PATH"/net.txt"

#define NET_MODE_SIZE sizeof("client")
#define NET_LOCATOR_SIZE 64
#define NET_CONFIG_LINE_SIZE NET_MODE_SIZE + NET_LOCATOR_SIZE
#define TOPIC_INFO_SIZE 64
#define MAX_LINE_SIZE 2*TOPIC_INFO_SIZE

class Zenoh_Config
{
public:
	Zenoh_Config();
	~Zenoh_Config();

	int cli(int argc, char *argv[]);

	void getNetworkConfig(char *mode, char *locator);
	int getPubCount()
	{
		return getLineCount(ZENOH_PUB_CONFIG_PATH);
	}
	int getSubCount()
	{
		return getLineCount(ZENOH_SUB_CONFIG_PATH);
	}
	int getPublisherMapping(char *topic, char *type)
	{
		return getPubSubMapping(topic, type, ZENOH_PUB_CONFIG_PATH);
	}
	int getSubscriberMapping(char *topic, char *type)
	{
		return getPubSubMapping(topic, type, ZENOH_SUB_CONFIG_PATH);
	}


private:
	int getPubSubMapping(char *topic, char *type, const char *filename);
	int AddPubSub(char *topic, char *datatype, const char *filename);
	int SetNetworkConfig(char *mode, char *locator);
	int getLineCount(const char *filename);

	const char *get_csv_field(char *line, int num);
	void generate_clean_config();
	void dump_config();

	FILE *fp_mapping;


};
