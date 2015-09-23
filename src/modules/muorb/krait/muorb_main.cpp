/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
#include "uORBManager.hpp"
#include "uORBKraitFastRpcChannel.hpp"

extern "C" { __EXPORT int muorb_main(int argc, char *argv[]); }

static void usage()
{
	warnx("Usage: muorb 'start', 'stop', 'status'");
}


int
muorb_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
		return -EINVAL;
	}

	/*
	 * Start/load the driver.
	 *
	 * XXX it would be nice to have a wrapper for this...
	 */
	if (!strcmp(argv[1], "start")) {
		// register the fast rpc channel with UORB.
		uORB::Manager::get_instance()->set_uorb_communicator(uORB::KraitFastRpcChannel::GetInstance());

		// start the KaitFastRPC channel thread.
		uORB::KraitFastRpcChannel::GetInstance()->Start();
		return OK;

	}

	if (!strcmp(argv[1], "stop")) {

		uORB::KraitFastRpcChannel::GetInstance()->Stop();
		return OK;
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "status")) {
		return OK;
	}

	usage();
	return -EINVAL;
}
