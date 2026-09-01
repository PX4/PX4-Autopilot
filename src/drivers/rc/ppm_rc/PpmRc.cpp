/****************************************************************************
 *
 *   Copyright (c) 2012-2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "PpmRc.hpp"

#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

ModuleBase::Descriptor PpmRc::desc{task_spawn, custom_command, print_usage};

PpmRc::PpmRc() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")),
	_publish_interval_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": publish interval"))
{
}

PpmRc::~PpmRc()
{
#if defined(HRT_PPM_CHANNEL)
	px4_arch_unconfiggpio(GPIO_PPM_IN);
#endif
	perf_free(_cycle_perf);
	perf_free(_publish_interval_perf);
}

int PpmRc::task_spawn(int argc, char *argv[])
{
#if !defined(HRT_PPM_CHANNEL)
	PX4_ERR("PPM not supported on this board");
	return PX4_ERROR;
#else

#ifdef RC_SERIAL_PORT_SHARED_PPM_PIN_GPIO_RX
	int32_t rc_prot = 0;

	if (param_get(param_find("SER_RC_PROTO"), &rc_prot) == PX4_OK && rc_prot != 0) {
		PX4_ERR("PPM shares the RC UART pin; set SER_RC_PROTO to Disabled");
		return PX4_ERROR;
	}

	px4_arch_unconfiggpio(RC_SERIAL_PORT_SHARED_PPM_PIN_GPIO_RX);
#endif

	px4_arch_configgpio(GPIO_PPM_IN);

	PpmRc *instance = new PpmRc();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;
	instance->ScheduleOnInterval(_current_update_interval);
	return PX4_OK;
#endif
}

void PpmRc::Run()
{
	if (should_exit()) {
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_cycle_perf);

#if defined(HRT_PPM_CHANNEL)

	if ((ppm_last_valid_decode != _timestamp_last_signal) && ppm_decoded_channels > 3) {
		input_rc_s input_rc{};
		input_rc.timestamp_last_signal = ppm_last_valid_decode;
		input_rc.channel_count = math::min((unsigned)ppm_decoded_channels,
						   (unsigned)input_rc_s::RC_INPUT_MAX_CHANNELS);
		input_rc.input_source = input_rc_s::RC_INPUT_SOURCE_PX4FMU_PPM;
		input_rc.rssi = -1;
		input_rc.link_quality = -1;
		input_rc.rssi_dbm = NAN;
		input_rc.rc_ppm_frame_length = ppm_frame_length;

		unsigned valid_chans = 0;

		for (unsigned i = 0; i < input_rc.channel_count; i++) {
			input_rc.values[i] = ppm_buffer[i];

			if (ppm_buffer[i] != 0) {
				valid_chans++;
			}
		}

		input_rc.rc_lost = (valid_chans == 0);
		input_rc.timestamp = hrt_absolute_time();
		_input_rc_pub.publish(input_rc);
		perf_count(_publish_interval_perf);
		_timestamp_last_signal = ppm_last_valid_decode;

		if (valid_chans > 0 && !_locked) {
			_locked = true;
			PX4_INFO("RC input locked");
		}
	}

#endif

	perf_end(_cycle_perf);
}

int PpmRc::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int PpmRc::print_status()
{
	PX4_INFO("RC state: %s", _locked ? "found" : "searching for signal");
	perf_print_counter(_cycle_perf);
	perf_print_counter(_publish_interval_perf);
	return 0;
}

int PpmRc::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
PPM RC input from the FMU capture pin.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ppm_rc", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("radio_control");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int ppm_rc_main(int argc, char *argv[])
{
	return ModuleBase::main(PpmRc::desc, argc, argv);
}
