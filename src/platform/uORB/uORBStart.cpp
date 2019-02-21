/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "uORB.h"

#include "uORBManager.hpp"
#include "uORBCommon.hpp"

#include <px4_log.h>
#include <px4_module.h>

static uORB::DeviceMaster *g_dev = nullptr;

int uorb_start(void)
{
	if (g_dev != nullptr) {
		PX4_WARN("already loaded");
		/* user wanted to start uorb, its already running, no error */
		return 0;
	}

	if (!uORB::Manager::initialize()) {
		PX4_ERR("uorb manager alloc failed");
		return -ENOMEM;
	}

	/* create the driver */
	g_dev = uORB::Manager::get_instance()->get_device_master();

	if (g_dev == nullptr) {
		return -errno;
	}

#if !defined(__PX4_QURT) && !defined(__PX4_POSIX_EAGLE) && !defined(__PX4_POSIX_EXCELSIOR)
	/* FIXME: this fails on Snapdragon (see https://github.com/PX4/Firmware/issues/5406),
	 * so we disable logging messages to the ulog for now. This needs further investigations.
	 */
	px4_log_initialize();
#endif

	return OK;
}

int uorb_status(void)
{
	if (g_dev != nullptr) {
		g_dev->printStatistics(true);

	} else {
		PX4_INFO("uorb is not running");
	}

	return PX4_OK;
}

int uorb_top(char **topic_filter, int num_filters)
{
	if (g_dev != nullptr) {
		g_dev->showTop(topic_filter, num_filters);

	} else {
		PX4_INFO("uorb is not running");
	}

	return OK;
}
