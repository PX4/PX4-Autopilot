/****************************************************************************
 *
 *   Copyright (C) 2018 PX4 Development Team. All rights reserved.
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
 * @file dyn.cpp
 *
 * @author Mara Bos <m-ou.se@m-ou.se>
 */

#include <dlfcn.h>

#include <px4_platform_common/module.h>
#include <px4_platform_common/log.h>

static void usage();

extern "C" {
	__EXPORT int dyn_main(int argc, char *argv[]);
}

static void usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"(
### Description
Load and run a dynamic PX4 module, which was not compiled into the PX4 binary.

### Example
$ dyn ./hello.px4mod start

)");
	PRINT_MODULE_USAGE_NAME_SIMPLE("dyn", "command");
	PRINT_MODULE_USAGE_ARG("<file>", "File containing the module", false);
	PRINT_MODULE_USAGE_ARG("arguments...", "Arguments to the module", true);
}

int dyn_main(int argc, char *argv[]) {
	if (argc < 2) {
		usage();
		return 1;
	}

	void *handle = dlopen(argv[1], RTLD_NOW);

	if (!handle) {
		PX4_ERR("%s", dlerror());
		return 1;
	}

	void *main_address = dlsym(handle, "px4_module_main");

	if (!main_address) {
		PX4_ERR("%s", dlerror());
		dlclose(handle);
		return 1;
	}

	auto main_function = (int (*)(int, char **))main_address;

	return main_function(argc - 1, argv + 1);
}
