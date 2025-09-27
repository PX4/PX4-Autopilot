/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file main.cpp
 *
 * This file serves as the wrapper layer for HiwonderEMM driver, working with parameters
 * and scheduling stuffs on PX4 side.
 *
 * Product: https://www.hiwonder.com/products/4-channel-encoder-motor-driver
 *
 */

#include <px4_log.h>
#include <drivers/device/device.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <px4_platform_common/module.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/sem.hpp>

#include "HiwonderEMM.h"

using namespace time_literals;

class HiwonderEMMWrapper : public ModuleBase<HiwonderEMMWrapper>, public OutputModuleInterface
{
public:
	HiwonderEMMWrapper();
	~HiwonderEMMWrapper() override;
	HiwonderEMMWrapper(const HiwonderEMMWrapper &) = delete;
	HiwonderEMMWrapper operator=(const HiwonderEMMWrapper &) = delete;

	int init();

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool updateOutputs(uint16_t *outputs, unsigned num_outputs,
			   unsigned num_control_groups_updated) override;

	int print_status() override;

protected:
	void updateParams() override;

private:
	HiwonderEMM *hiwonderemm = nullptr;
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	MixingOutput _mixing_output {
		"EMM",
		CHANNEL_COUNT,
		*this,
		MixingOutput::SchedulingPolicy::Auto,
		false
	};

	void Run() override;
};

HiwonderEMMWrapper::HiwonderEMMWrapper() :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

HiwonderEMMWrapper::~HiwonderEMMWrapper()
{
	if (hiwonderemm != nullptr) {
		delete hiwonderemm;
	}
}

int HiwonderEMMWrapper::init()
{
	int ret = hiwonderemm->init();

	if (ret != PX4_OK) { return ret; }

	this->ChangeWorkQueue(px4::device_bus_to_wq(hiwonderemm->get_device_id()));

	PX4_INFO("running on I2C bus %d address 0x%.2x", hiwonderemm->get_device_bus(), hiwonderemm->get_device_address());

	ScheduleNow();

	return PX4_OK;
}

bool HiwonderEMMWrapper::updateOutputs(uint16_t *outputs, unsigned num_outputs,
				       unsigned num_control_groups_updated)
{
	uint8_t speed_values[4];

	for (unsigned i = 0; i < num_outputs && i < CHANNEL_COUNT; i++) {
		speed_values[i] = (uint8_t)(outputs[i] - 128);
	}

	hiwonderemm->set_motor_speed(speed_values);

	return true;
}

void HiwonderEMMWrapper::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();
		delete hiwonderemm;
		hiwonderemm = nullptr;
		exit_and_cleanup();
		return;
	}

	_mixing_output.update();

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}

	_mixing_output.updateSubscriptions(false);
}

int HiwonderEMMWrapper::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Hiwonder encoder motor module driver for PX4.
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("hiwonder_emm", "driver");
    PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int HiwonderEMMWrapper::print_status() {
    int ret =  ModuleBase::print_status();
    PX4_INFO("HiwonderEMM @I2C Bus %d, address 0x%.2x",
            hiwonderemm->get_device_bus(),
            hiwonderemm->get_device_address());

    return ret;
}

int HiwonderEMMWrapper::custom_command(int argc, char **argv) {
    return PX4_OK;
}

int HiwonderEMMWrapper::task_spawn(int argc, char **argv) {
	int address = I2C_ADDR;
	int iicbus = I2CBUS;

	auto *instance = new HiwonderEMMWrapper();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		instance->hiwonderemm = new HiwonderEMM(iicbus, address);
		if (instance->hiwonderemm == nullptr) {
			PX4_ERR("alloc failed");
			goto driverInstanceAllocFailed;
		}

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		} else {
			PX4_ERR("driver init failed");
			delete instance->hiwonderemm;
			instance->hiwonderemm = nullptr;
		}
	} else {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	driverInstanceAllocFailed:
	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

void HiwonderEMMWrapper::updateParams() {
    ModuleParams::updateParams();
}

extern "C" __EXPORT int hiwonder_emm_main(int argc, char *argv[]){
	return HiwonderEMMWrapper::main(argc, argv);
}
