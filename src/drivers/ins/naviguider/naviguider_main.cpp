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

#include "naviguider.hpp"
#include <parameters/param.h>

extern "C" __EXPORT int naviguider_main(int argc, char *argv[]) {
	//return Naviguider::main(argc, argv);
	return ModuleBase::main(Naviguider::module_desc, argc, argv);
}


int Naviguider::task_spawn(int argc, char *argv[]) {
	int32_t bus = 2;
	int32_t addr = 0x28;
	int32_t enabled = 1;

	param_t p;

	p = param_find("SENS_NG_BUS");

	if (p != PARAM_INVALID) {
		param_get(p, &bus);
	}

	p = param_find("SENS_NG_ADDR");

	if (p != PARAM_INVALID) {
		param_get(p, &addr);
	}

	p = param_find("SENS_EN_NG");

	if (p != PARAM_INVALID) {
		param_get(p, &enabled);
	}

	if (enabled == 0) {
		PX4_WARN("Naviguider disabled by parameter");
		return PX4_ERROR;
	}

	// Spawn instance
	Naviguider *instance = new Naviguider(bus, static_cast<uint8_t>(addr));

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	module_desc.object.store(instance);
	module_desc.task_id = task_id_is_work_queue;

	if (instance->init() != PX4_OK) {
		PX4_ERR("init failed");
		delete instance;
		module_desc.object.store(nullptr);
		module_desc.task_id = -1;
		return PX4_ERROR;
	}

	return PX4_OK;
}

int Naviguider::custom_command(int argc, char *argv[]) {
	return print_usage("unknown command");
}

int Naviguider::print_usage(const char *reason) {
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
Driver for the PNI NaviGuider over I2C.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("naviguider", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");

	return 0;
}
