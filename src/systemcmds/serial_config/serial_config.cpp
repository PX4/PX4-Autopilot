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

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>

#include <nuttx/serial/serial.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

static int usage(const char *reason = nullptr)
{
	if (reason) {
		PX4_ERR("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(R"DESCR_STR(
Configure UART pin swapping before starting a serial service.
Called by the generated serial startup script for SER_*_SWAP parameters.
No background task is created. Reboot to restore normal pin assignments.
Only STM32H7 UARTs with NuttX TIOCSSWAP support are currently supported.
The UART driver must preserve the setting across close and reopen.
Do not run on a port with an active service. This command refuses to run while armed.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("serial_config", "system");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<file:dev>", "UART device", false);
	PRINT_MODULE_USAGE_PARAM_FLAG('s', "Swap RX and TX pins", false);
	return reason ? PX4_ERROR : PX4_OK;
}

extern "C" __EXPORT int serial_config_main(int argc, char *argv[])
{
	const char *device = nullptr;
	bool swap = false;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	int ch;

	while ((ch = px4_getopt(argc, argv, "d:s", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd': device = myoptarg; break;
		case 's': swap = true; break;
		default: return usage("invalid arguments");
		}
	}

	if (!device || !swap || strncmp(device, "/dev/ttyS", 9) != 0) {
		return usage("specify a UART device and -s");
	}

	uORB::Subscription armed_sub{ORB_ID(actuator_armed)};
	actuator_armed_s armed{};

	if (armed_sub.copy(&armed) && armed.armed) {
		PX4_ERR("refusing UART configuration while armed");
		return PX4_ERROR;
	}

	const int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (fd < 0) {
		PX4_ERR("open %s failed (%d)", device, errno);
		return PX4_ERROR;
	}

	const int result = ioctl(fd, TIOCSSWAP, SER_SWAP_ENABLED);
	const int saved_errno = errno;
	close(fd);

	if (result != 0) {
		PX4_ERR("RX/TX swap failed on %s (%d)", device, saved_errno);
		return PX4_ERROR;
	}

	PX4_INFO("RX/TX swapped on %s", device);
	return PX4_OK;
}
