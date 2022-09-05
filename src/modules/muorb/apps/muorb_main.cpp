/****************************************************************************
 *
 *   Copyright (c) 2022 ModalAI, Inc. All rights reserved.
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

#include <string.h>
#include "uORBAppsProtobufChannel.hpp"

extern "C" { __EXPORT int muorb_main(int argc, char *argv[]); }

static void usage()
{
	PX4_INFO("Usage: muorb 'start', 'test', 'stop', 'status'");
}

static bool enable_debug = false;

int
muorb_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
		return -EINVAL;
	}

	// TODO: Add an optional  start parameter to control debug messages
	if (!strcmp(argv[1], "start")) {
		// Register the protobuf channel with UORB.
		uORB::AppsProtobufChannel *channel = uORB::AppsProtobufChannel::GetInstance();

		if (channel && channel->Initialize(enable_debug)) { return OK; }

	} else if (!strcmp(argv[1], "test")) {
		uORB::AppsProtobufChannel *channel = uORB::AppsProtobufChannel::GetInstance();

		if (channel && channel->Initialize(enable_debug) && channel->Test()) { return OK; }

	} else if (!strcmp(argv[1], "stop")) {
		if (uORB::AppsProtobufChannel::isInstance() == false) {
			PX4_WARN("muorb not running");
		}

		return OK;

	} else if (!strcmp(argv[1], "status")) {
		if (uORB::AppsProtobufChannel::isInstance()) {
			PX4_INFO("muorb initialized");

		} else {
			PX4_INFO("muorb not running");
		}

		return OK;
	}

	usage();
	return -EINVAL;
}
