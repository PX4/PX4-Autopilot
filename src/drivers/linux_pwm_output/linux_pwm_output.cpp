/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file linux_pwm_output.cpp
 * Base class to manage all PWM devices which run on Linux platform such as RPi
 * Implemented CDev framework only makes mixer files loaded in a different way
 * I don't think this can be the final solution.
 *
 * @author SalimTerryLi <lhf2613@gmail.com>
 */

#include <px4_log.h>
#include <drivers/device/device.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <px4_platform_common/module.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_mixer.h>

#include "PWMDeviceBase.h"

#include "device_debug.h"
#include "PCA9685.h"
#include "navio_sysfs.h"
#include "ocpoc_mmap.h"
#include "bbblue_pwm_rc.h"

#define LINUX_PWM_OUTPUT_CHANNELS   16
#define LINUXPWM_DEVICE_PATH	"/dev/linux_pwm_out_wrapper"

using namespace linux_pwm_output;

class LinuxPWMOutWrapper : public cdev::CDev, public ModuleBase<LinuxPWMOutWrapper>, public OutputModuleInterface
{
public:

	LinuxPWMOutWrapper();
	~LinuxPWMOutWrapper() override ;

	int init() override;

	int ioctl(cdev::file_t *filep, int cmd, unsigned long arg) override;

	void mixerChanged() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);
	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool updateOutputs(bool stop_motors, uint16_t *outputs, unsigned num_outputs,
			   unsigned num_control_groups_updated) override;

	LinuxPWMOutWrapper(const LinuxPWMOutWrapper &) = delete;
	LinuxPWMOutWrapper operator=(const LinuxPWMOutWrapper &) = delete;

private:
	perf_counter_t	_cycle_perf;

	void Run() override;

protected:
	void updateParams() override;

	void updatePWMParams();

	PWMDeviceBase *pwmDevice = nullptr; // driver handle.

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)}; // param handle

	MixingOutput _mixing_output{LINUX_PWM_OUTPUT_CHANNELS, *this, MixingOutput::SchedulingPolicy::Auto, true};

private:

};

LinuxPWMOutWrapper::LinuxPWMOutWrapper() :
	CDev(LINUXPWM_DEVICE_PATH),
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_mixing_output.setAllMinValues(PWM_DEFAULT_MIN);
	_mixing_output.setAllMaxValues(PWM_DEFAULT_MAX);
}

LinuxPWMOutWrapper::~LinuxPWMOutWrapper()
{
	if (pwmDevice != nullptr) { // normally this should not be called.
		PX4_DEBUG("Destruction of LinuxPWMOutWrapper without pwmDevice unloaded!");
		pwmDevice->deviceDeinit(); // force stop
		delete pwmDevice;
		pwmDevice = nullptr;
	}

	perf_free(_cycle_perf);
}

int LinuxPWMOutWrapper::init()
{
	int ret = CDev::init();

	if (ret != PX4_OK) {
		return ret;
	}

	ret = pwmDevice->deviceInit();

	if (ret != PX4_OK) {
		return ret;
	}

	updatePWMParams();

	ScheduleNow();

	return PX4_OK;
}

void LinuxPWMOutWrapper::updateParams()
{
	updatePWMParams();
	ModuleParams::updateParams();
}

// TODO add support for trim value.
void LinuxPWMOutWrapper::updatePWMParams()
{
	// update pwm params
	const char *pname_format_main_max = "PWM_MAIN_MAX%d";
	const char *pname_format_main_min = "PWM_MAIN_MIN%d";
	const char *pname_format_main_fail = "PWM_MAIN_FAIL%d";
	const char *pname_format_main_dis = "PWM_MAIN_DIS%d";
	const char *pname_format_main_rev = "PWM_MAIN_REV%d";
	//const char *pname_format_main_trim="PWM_MAIN_TRIM%d";
	const char *pname_format_aux_max = "PWM_AUX_MAX%d";
	const char *pname_format_aux_min = "PWM_AUX_MIN%d";
	const char *pname_format_aux_fail = "PWM_AUX_FAIL%d";
	const char *pname_format_aux_dis = "PWM_AUX_DIS%d";
	const char *pname_format_aux_rev = "PWM_AUX_REV%d";
	//const char *pname_format_aux_trim="PWM_AUX_TRIM%d";

	//int16_t trim_values[LINUX_PWM_OUTPUT_CHANNELS] = {};

	param_t param_h = param_find("PWM_MAX");

	if (param_h != PARAM_INVALID) {
		int32_t pval = 0;
		param_get(param_h, &pval);
		_mixing_output.setAllMaxValues(pval);

	} else {
		PX4_ERR("PARAM_INVALID: %s", "PWM_MAX");
	}

	param_h = param_find("PWM_MIN");

	if (param_h != PARAM_INVALID) {
		int32_t pval = 0;
		param_get(param_h, &pval);
		_mixing_output.setAllMinValues(pval);

	} else {
		PX4_ERR("PARAM_INVALID: %s", "PWM_MIN");
	}

	param_h = param_find("PWM_RATE");

	if (param_h != PARAM_INVALID) {
		int32_t pval = 0;
		param_get(param_h, &pval);

		if (pwmDevice->setFreq(pval) != PX4_OK) {
			PX4_ERR("failed to set pwm frequency");
		}

	} else {
		PX4_ERR("PARAM_INVALID: %s", "PWM_RATE");
	}

	for (int i = 0; i < 8; i++) {
		char pname[16];

		sprintf(pname, pname_format_main_max, i + 1);
		param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);

			if (pval != -1) {
				_mixing_output.maxValue(i) = pval;
			}

		} else {
			PX4_ERR("PARAM_INVALID: %s", pname);
		}

		sprintf(pname, pname_format_main_min, i + 1);
		param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);

			if (pval != -1) {
				_mixing_output.minValue(i) = pval;
			}

		} else {
			PX4_ERR("PARAM_INVALID: %s", pname);
		}

		sprintf(pname, pname_format_main_fail, i + 1);
		param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);

			if (pval != -1) {
				_mixing_output.failsafeValue(i) = pval;
			}

		} else {
			PX4_ERR("PARAM_INVALID: %s", pname);
		}

		sprintf(pname, pname_format_main_dis, i + 1);
		param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);

			if (pval != -1) {
				_mixing_output.disarmedValue(i) = pval;
			}

		} else {
			PX4_ERR("PARAM_INVALID: %s", pname);
		}

		sprintf(pname, pname_format_main_rev, i + 1);
		param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			uint16_t &reverse_pwm_mask = _mixing_output.reverseOutputMask();
			int32_t pval = 0;
			param_get(param_h, &pval);
			reverse_pwm_mask &= (0xfffe << i);  // clear this bit
			reverse_pwm_mask |= (((uint16_t)(pval != 0)) << i); // set to new val

		} else {
			PX4_ERR("PARAM_INVALID: %s", pname);
		}

		sprintf(pname, pname_format_aux_max, i + 1);
		param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);

			if (pval != -1) {
				_mixing_output.maxValue(i + 8) = pval;
			}

		} else {
			PX4_ERR("PARAM_INVALID: %s", pname);
		}

		sprintf(pname, pname_format_aux_min, i + 1);
		param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);

			if (pval != -1) {
				_mixing_output.minValue(i + 8) = pval;
			}

		} else {
			PX4_ERR("PARAM_INVALID: %s", pname);
		}

		sprintf(pname, pname_format_aux_fail, i + 1);
		param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);

			if (pval != -1) {
				_mixing_output.failsafeValue(i + 8) = pval;
			}

		} else {
			PX4_ERR("PARAM_INVALID: %s", pname);
		}

		sprintf(pname, pname_format_aux_dis, i + 1);
		param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			int32_t pval = 0;
			param_get(param_h, &pval);

			if (pval != -1) {
				_mixing_output.disarmedValue(i + 8) = pval;
			}

		} else {
			PX4_ERR("PARAM_INVALID: %s", pname);
		}

		sprintf(pname, pname_format_aux_rev, i + 1);
		param_h = param_find(pname);

		if (param_h != PARAM_INVALID) {
			uint16_t &reverse_pwm_mask = _mixing_output.reverseOutputMask();
			int32_t pval = 0;
			param_get(param_h, &pval);
			reverse_pwm_mask &= (0xfffe << (i + 8)); // clear this bit
			reverse_pwm_mask |= (((uint16_t)(pval != 0)) << (i + 8)); // set to new val

		} else {
			PX4_ERR("PARAM_INVALID: %s", pname);
		}

		/*sprintf(pname, pname_format_main_trim, i + 1);
		param_h = param_find(pname);
		if (param_h != PARAM_INVALID) {
		    float pval = 0.0f;
		    param_get(param_h, &pval);
		    trim_values[i] = (int16_t)(10000 * pval);
		}else {
		    PX4_ERR("PARAM_INVALID: %s",pname);
		}

		sprintf(pname, pname_format_aux_trim, i + 1);
		param_h = param_find(pname);
		if (param_h != PARAM_INVALID) {
		    float pval = 0.0f;
		    param_get(param_h, &pval);
		    trim_values[i+8] = (int16_t)(10000 * pval);
		}else {
		    PX4_ERR("PARAM_INVALID: %s",pname);
		}*/
	}

	/*unsigned n_out = _mixing_output.mixers()->set_trims(trim_values, LINUX_PWM_OUTPUT_CHANNELS);
	PX4_DEBUG("set %d trims", n_out);*/

}

bool LinuxPWMOutWrapper::updateOutputs(bool stop_motors, uint16_t *outputs, unsigned num_outputs,
				       unsigned num_control_groups_updated)
{
	return pwmDevice->updatePWM(outputs, num_outputs);;
}

// TODO
void LinuxPWMOutWrapper::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		pwmDevice->deviceDeinit();
		delete pwmDevice;
		pwmDevice = nullptr;

		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	_mixing_output.update();

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}

	_mixing_output.updateSubscriptions(true);

	perf_end(_cycle_perf);
}

// TODO
int LinuxPWMOutWrapper::ioctl(cdev::file_t *filep, int cmd, unsigned long arg)
{
	int ret = OK;
	PX4_INFO("linux_pwm ioctl cmd: %d, arg: %ld", cmd, arg);

	lock();

	switch (cmd) {
	case MIXERIOCRESET:
		_mixing_output.resetMixerThreadSafe();

		break;

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			unsigned buflen = strlen(buf);

			ret = _mixing_output.loadMixerThreadSafe(buf, buflen);

			break;
		}

	default:
		ret = -ENOTTY;
		break;
	}

	unlock();

	if (ret == -ENOTTY) {
		ret = CDev::ioctl(filep, cmd, arg);
	}

	return ret;
}

int LinuxPWMOutWrapper::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module is responsible for generate pwm pulse on Linux based device such as RPi.

It listens on the actuator_controls topics, does the mixing and writes the PWM outputs.

### Implementation
By default the module runs on a work queue with a callback on the uORB actuator_controls topic.

### Examples
It is typically started with:
$ linux_pwm start debug -a -b 2

Use the `mixer` command to load mixer files.
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("linux_pwm_output", "driver");
    PRINT_MODULE_USAGE_COMMAND_DESCR("start debug", "Use debug_driver as pwm destination");
    PRINT_MODULE_USAGE_COMMAND_DESCR("start navio2", "Use NAVIO2 as pwm destination");
    PRINT_MODULE_USAGE_COMMAND_DESCR("start pca9685", "Use pca9685 as pwm destination");
    PRINT_MODULE_USAGE_COMMAND_DESCR("start ocpoc_mmap", "Use ocpoc_mmap as pwm destination");
    PRINT_MODULE_USAGE_COMMAND_DESCR("start bbblue_rc", "Use bbblue_rc as pwm destination");
    PRINT_MODULE_USAGE_PARAM_COMMENT("");
    PWMDeviceBase::deviceUsage();
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int LinuxPWMOutWrapper::custom_command(int argc, char **argv) {
    return PX4_OK;
}

int LinuxPWMOutWrapper::task_spawn(int argc, char **argv) {

    auto *instance = new LinuxPWMOutWrapper();

    if (instance!=nullptr) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;

        // Detect device type
        // TODO load each device
        if(argc>=2) {
            const char *verb = argv[1];
            if (!strcmp(verb, "debug")) {
                instance->pwmDevice=new device_debug();
            }else if (!strcmp(verb, "navio2")) {
                instance->pwmDevice=new NavioSysfsPWMOut();
            }else if (!strcmp(verb, "pca9685")) {
                instance->pwmDevice=new PCA9685();
            }else if (!strcmp(verb, "ocpoc_mmap")) {
                instance->pwmDevice=new OcpocMmapPWMOut();
            }else if (!strcmp(verb, "bbblue_rc")) {
                //instance->pwmDevice=new BBBlueRcPWMOut(); // cannot find a previous implement
            }else{
                PX4_ERR("unknown device");
                goto deviceNotLoaded;
            }
        } else{
            PX4_ERR("device not declared");
            goto deviceNotLoaded;
        }

        if(instance->pwmDevice == nullptr) {
            PX4_ERR("pwmDevice alloc failed");
            goto deviceNotLoaded;
        }
        if(instance->pwmDevice->deviceConfigure(argc-1, argv+1) != PX4_OK){
            PX4_ERR("failed to configure driver from cmd line arguments");
            goto deviceCfgFailed;
        }

        if (instance->init() == PX4_OK) {
            return PX4_OK;
        } else {
            PX4_ERR("driver init failed");
        }

        deviceCfgFailed:
        delete (instance->pwmDevice); // also delete the true driver
        instance->pwmDevice=nullptr;
        deviceNotLoaded:
        delete instance;
        _object.store(nullptr);
        _task_id = -1;
    } else {
        PX4_ERR("alloc failed");
    }

    return PX4_ERROR;
}

// TODO add support for trim value.
void LinuxPWMOutWrapper::mixerChanged() {
    OutputModuleInterface::mixerChanged();
}

extern "C" __EXPORT int linux_pwm_output_main(int argc, char *argv[]);

int linux_pwm_output_main(int argc, char *argv[]){
	return LinuxPWMOutWrapper::main(argc, argv);
}