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

/**
 * @file mklittlefs.cpp
 *
 * Format a device with littlefs filesystem.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

#include <sys/mount.h>
#include <errno.h>
#include <string.h>

static void print_usage()
{
	PRINT_MODULE_DESCRIPTION("Format a device with the littlefs filesystem.");

	PRINT_MODULE_USAGE_NAME_SIMPLE("mklittlefs", "command");
	PRINT_MODULE_USAGE_ARG("<device> <mountpoint>", "Device and mount point (e.g. /dev/mtd0 /fs/flash)", false);
}

extern "C" __EXPORT int mklittlefs_main(int argc, char *argv[])
{
	if (argc < 3) {
		print_usage();
		return 1;
	}

	const char *device = argv[1];
	const char *mountpoint = argv[2];

	// Try to unmount first (ignore error if not mounted)
	umount(mountpoint);

	int ret = mount(device, mountpoint, "littlefs", 0, "forceformat");

	if (ret < 0) {
		PX4_ERR("format failed: %s", strerror(errno));
		return 1;
	}

	PX4_INFO("formatted %s as littlefs, mounted at %s", device, mountpoint);
	return 0;
}
