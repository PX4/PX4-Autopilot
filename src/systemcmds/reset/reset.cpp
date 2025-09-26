/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

// PX4 Modules
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_config.h>

// C library
#include <stdlib.h>
#include <string.h>

#include <unistd.h>
#include <limits.h>
#include <stdio.h>
#include <libgen.h>

static void usage();

extern "C" {
	__EXPORT int reset_main(int argc, char *argv[]);
}

// Helper function to get the PX4 source directory from the binary path
static const char *get_px4_source_dir(const char *bin_path)
{
	// We need to find the path to the PX4 root source directory.
	// The binary path is something like: /path/to/PX4-Autopilot/build/px4_sitl_default/bin/px4
	// We need to get to: /path/to/PX4-Autopilot

	// Make a copy of the path since dirname may modify it
	char path_buffer[PATH_MAX];
	strncpy(path_buffer, bin_path, sizeof(path_buffer));
	path_buffer[sizeof(path_buffer) - 1] = '\0';

	// Step 1: Get the directory of the binary: /path/to/PX4-Autopilot/build/px4_sitl_default/bin/
	char *bin_dir = dirname(path_buffer);

	// Step 2: Go up 3 levels to get to the root source directory
	// bin/ -> build/px4_sitl_default/ -> build/ -> PX4-Autopilot/
	snprintf(path_buffer, sizeof(path_buffer), "%s/../../..", bin_dir);

	// Step 3: Get the absolute path
	char *abs_path = realpath(path_buffer, NULL);

	if (abs_path == nullptr) {
		PX4_ERR("Failed to get absolute path for %s", path_buffer);
		return nullptr;
	}

	// Check if the Tools/simulation directory exists in this path
	char check_path[PATH_MAX];
	snprintf(check_path, sizeof(check_path), "%s/Tools/simulation", abs_path);

	if (access(check_path, F_OK) == 0) {
		// Directory exists, this is likely the correct path
		return abs_path; // Note: caller must free this memory

	} else {
		free(abs_path);
		PX4_ERR("Expected Tools/simulation directory not found at %s", check_path);
		return nullptr;
	}
}

int
reset_main(int argc, char *argv[])
{
	PX4_INFO("Requesting simulation reset...");

	// Get the PX4 source directory from the binary path
	const char *px4_source_dir = get_px4_source_dir(argv[0]);

	if (px4_source_dir == nullptr) {
		PX4_ERR("Failed to determine PX4 source directory from binary path: %s", argv[0]);
		usage();
		return 1;
	}

	// Build the command to run the script
	char command[512];
	snprintf(command, sizeof(command), "%s/Tools/simulation/reset.sh %s &",
		 px4_source_dir, px4_source_dir);

	PX4_DEBUG("Executing: %s", command);

	// Execute the command
	int ret = system(command);

	// Free the memory allocated by realpath
	free((void *)px4_source_dir);

	if (ret == 0) {
		PX4_INFO("Reset command spawned successfully.");
		return 0;

	} else {
		PX4_ERR("Failed to spawn reset command (%d)", ret);
		usage();
		return 1;
	}
}

static void
usage()
{

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Command-line tool to reset the current SITL world.

The command kills the current simulator instance (Gazebo or JMavSim) and restarts it.
It is only available in SITL

### Examples
$ reset
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("reset", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("set", "Reset the simulation world");

}
