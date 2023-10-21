/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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
 * @file pca9685/main.cpp
 *
 * This file serves as the wrapper layer for PCA9685 driver, working with parameters
 * and scheduling stuffs on PX4 side.
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

#include "PCA9685.h"

#define PCA9685_DEFAULT_IICBUS  1
#define PCA9685_DEFAULT_ADDRESS (0x40)

using namespace drv_pca9685_pwm;
using namespace time_literals;

class PCA9685Wrapper : public ModuleBase<PCA9685Wrapper>, public OutputModuleInterface
{
public:
	PCA9685Wrapper();
	~PCA9685Wrapper() override;
	PCA9685Wrapper(const PCA9685Wrapper &) = delete;
	PCA9685Wrapper operator=(const PCA9685Wrapper &) = delete;

	int init();

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool updateOutputs(bool stop_motors, uint16_t *outputs, unsigned num_outputs,
			   unsigned num_control_groups_updated) override;

	int print_status() override;

protected:
	void updateParams() override;

private:
	perf_counter_t	_cycle_perf;

	enum class STATE : uint8_t {
		INIT,
		WAIT_FOR_OSC,
		RUNNING
	} state{STATE::INIT};

	PCA9685 *pca9685 = nullptr;
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	MixingOutput _mixing_output {
		"PCA9685",
		PCA9685_PWM_CHANNEL_COUNT,
		*this,
		MixingOutput::SchedulingPolicy::Disabled,
		true
	};

	float param_pwm_freq, previous_pwm_freq;
	float param_schd_rate, previous_schd_rate;
	uint32_t param_duty_mode;

	void Run() override;
};

PCA9685Wrapper::PCA9685Wrapper() :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
}

PCA9685Wrapper::~PCA9685Wrapper()
{
	if (pca9685 != nullptr) {
		pca9685->setAllPWM(0);
		pca9685->sleep();
		delete pca9685;
	}

	perf_free(_cycle_perf);
}

int PCA9685Wrapper::init()
{
	int ret = pca9685->init();

	if (ret != PX4_OK) { return ret; }

	this->ChangeWorkQueue(px4::device_bus_to_wq(pca9685->get_device_id()));

	PX4_INFO("running on I2C bus %d address 0x%.2x", pca9685->get_device_bus(), pca9685->get_device_address());

	ScheduleNow();

	return PX4_OK;
}

bool PCA9685Wrapper::updateOutputs(bool stop_motors, uint16_t *outputs, unsigned num_outputs,
				   unsigned num_control_groups_updated)
{
	if (state != STATE::RUNNING) { return false; }

	uint16_t low_level_outputs[PCA9685_PWM_CHANNEL_COUNT] = {};
	num_outputs = num_outputs > PCA9685_PWM_CHANNEL_COUNT ? PCA9685_PWM_CHANNEL_COUNT : num_outputs;

	for (uint8_t i = 0; i < num_outputs; ++i) {
		if (param_duty_mode & (1 << i)) {
			low_level_outputs[i] = outputs[i];

		} else {
			low_level_outputs[i] = pca9685->calcRawFromPulse(outputs[i]);
		}
	}

	if (pca9685->updateRAW(low_level_outputs, num_outputs) != PX4_OK) {
		PX4_ERR("Failed to write PWM to PCA9685");
		return false;
	}

	return true;
}

void PCA9685Wrapper::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		pca9685->setAllPWM(0);
		pca9685->sleep();
		delete pca9685;
		pca9685 = nullptr;

		exit_and_cleanup();
		return;
	}

	switch (state) {
	case STATE::INIT:
		updateParams();
		pca9685->updateFreq(param_pwm_freq);
		previous_pwm_freq = param_pwm_freq;
		previous_schd_rate = param_schd_rate;

		pca9685->wake();
		state = STATE::WAIT_FOR_OSC;
		ScheduleDelayed(500);
		break;

	case STATE::WAIT_FOR_OSC: {
			state = STATE::RUNNING;
			ScheduleOnInterval(1000000 / param_schd_rate, 0);
		}
		break;

	case STATE::RUNNING:
		perf_begin(_cycle_perf);

		_mixing_output.update();

		// check for parameter updates
		if (_parameter_update_sub.updated()) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			updateParams();

			// apply param updates
			if ((float)fabs(previous_pwm_freq - param_pwm_freq) > 0.01f) {
				previous_pwm_freq = param_pwm_freq;

				ScheduleClear();

				pca9685->sleep();
				pca9685->updateFreq(param_pwm_freq);
				pca9685->wake();

				// update of PWM freq will always trigger scheduling change
				previous_schd_rate = param_schd_rate;

				state = STATE::WAIT_FOR_OSC;
				ScheduleDelayed(500);

			} else if ((float)fabs(previous_schd_rate - param_schd_rate) > 0.01f) {
				// case when PWM freq not changed but scheduling rate does
				previous_schd_rate = param_schd_rate;
				ScheduleClear();
				ScheduleOnInterval(1000000 / param_schd_rate, 1000000 / param_schd_rate);
			}
		}

		_mixing_output.updateSubscriptions(false);

		perf_end(_cycle_perf);
		break;
	}
}

int PCA9685Wrapper::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This is a PCA9685 PWM output driver.

It runs on I2C workqueue which is asynchronous with FC control loop,
fetching the latest mixing result and write them to PCA9685 at its scheduling ticks.

It can do full 12bits output as duty-cycle mode, while also able to output precious pulse width
that can be accepted by most ESCs and servos.

### Examples
It is typically started with:
$ pca9685_pwm_out start -a 0x40 -b 1

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("pca9685_pwm_out", "driver");
    PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task");
    PRINT_MODULE_USAGE_PARAM_STRING('a',"0x40","<addr>","7-bits I2C address of PCA9685",true);
	PRINT_MODULE_USAGE_PARAM_INT('b',1,0,255,"bus that pca9685 is connected to",true);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int PCA9685Wrapper::print_status() {
    int ret =  ModuleBase::print_status();
    PX4_INFO("PCA9685 @I2C Bus %d, address 0x%.2x, real frequency %.2f",
            pca9685->get_device_bus(),
            pca9685->get_device_address(),
             (double)(pca9685->getFreq()));

    return ret;
}

int PCA9685Wrapper::custom_command(int argc, char **argv) {
    return PX4_OK;
}

int PCA9685Wrapper::task_spawn(int argc, char **argv) {
	int ch;
	int address=PCA9685_DEFAULT_ADDRESS;
	int iicbus=PCA9685_DEFAULT_IICBUS;

	int myoptind = 1;
	const char *myoptarg = nullptr;
	while ((ch = px4_getopt(argc, argv, "a:b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
			case 'a':
                errno = 0;
				address = strtol(myoptarg, nullptr, 16);
                if (errno != 0) {
                    PX4_WARN("Invalid address");
                    return PX4_ERROR;
                }
				break;

			case 'b':
				iicbus = strtol(myoptarg, nullptr, 10);
                if (errno != 0) {
                    PX4_WARN("Invalid bus");
                    return PX4_ERROR;
                }
				break;

			case '?':
				PX4_WARN("Unsupported args");
				return PX4_ERROR;

			default:
				break;
		}
	}

    auto *instance = new PCA9685Wrapper();

    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;

        instance->pca9685 = new PCA9685(iicbus, address);
        if(instance->pca9685==nullptr){
            PX4_ERR("alloc failed");
            goto driverInstanceAllocFailed;
        }

        if (instance->init() == PX4_OK) {
            return PX4_OK;
        } else {
            PX4_ERR("driver init failed");
            delete instance->pca9685;
            instance->pca9685=nullptr;
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

void PCA9685Wrapper::updateParams() {
    ModuleParams::updateParams();

    param_t param = param_find("PCA9685_SCHD_HZ");
    if (param != PARAM_INVALID) {
        param_get(param, &param_schd_rate);
    } else {
        PX4_ERR("param PCA9685_SCHD_HZ not found");
    }

    param = param_find("PCA9685_PWM_FREQ");
    if (param != PARAM_INVALID) {
        param_get(param, &param_pwm_freq);
    } else {
        PX4_ERR("param PCA9685_PWM_FREQ not found");
    }

    param = param_find("PCA9685_DUTY_EN");
    if (param != PARAM_INVALID) {
        param_get(param, (int32_t*)&param_duty_mode);
    } else {
        PX4_ERR("param PCA9685_DUTY_EN not found");
    }
}

extern "C" __EXPORT int pca9685_pwm_out_main(int argc, char *argv[]){
	return PCA9685Wrapper::main(argc, argv);
}
