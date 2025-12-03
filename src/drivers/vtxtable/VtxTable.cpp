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

#include "VtxTable.hpp"

#include <fcntl.h>
#include <termios.h>
#include <drivers/drv_hrt.h>

using namespace time_literals;

vtx::Config VtxTable::data;

#ifdef CONFIG_VTXTABLE_UORB_CONFIG
VtxTable::VtxTable()
	: ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_perf_cycle(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle time")),
	_perf_error(perf_alloc(PC_COUNT, MODULE_NAME": errors"))
{
	ScheduleNow();
}
#endif

VtxTable::~VtxTable()
{
#ifdef CONFIG_VTXTABLE_UORB_CONFIG
	perf_free(_perf_cycle);
#endif
}

#ifdef CONFIG_VTXTABLE_UORB_CONFIG
void VtxTable::Run()
{
	perf_begin(_perf_cycle);

	vtx_aux_map_s vtx_aux_map{};
	vtx_table_s vtx_table{};

	if (_vtx_table_sub.update(&vtx_table)) {
		data.copy_from(&vtx_table);

		if (data.store(_config_file) != PX4_OK) {
			perf_count(_perf_error);
		}
	}

	if (_vtx_aux_map_sub.update(&vtx_aux_map)) {
		data.copy_from(&vtx_aux_map);

		if (data.store(_config_file) != PX4_OK) {
			perf_count(_perf_error);
		}
	}

	ScheduleDelayed(100_ms);
	perf_end(_perf_cycle);
}
#endif

#ifdef CONFIG_VTXTABLE_USE_STORAGE
int VtxTable::store(const char *filename)
{
	if (filename == nullptr) { filename = _config_file; }

	const int rv = data.store(filename);

	if (rv < 0) {
		PX4_ERR("%s is not accessible!", filename);
		return PX4_ERROR;
	}

	PX4_INFO("saved to %s", filename);
	return PX4_OK;
}

int VtxTable::load(const char *filename)
{
	if (filename == nullptr) { filename = _config_file; }

	const int rv = data.load(filename);

	if (rv < 0) {
		switch (rv) {
		case -ENOENT:
			PX4_ERR("%s not found!", filename);
			break;

		case -EPROTO:
			PX4_ERR("VTX config serialization format is unsupported in %s!", filename);
			break;

		case -EMSGSIZE:
			PX4_ERR("VTX config in %s is incomplete!", filename);
			break;

		default:
			PX4_ERR("Loading VTX config from %s failed!", filename);
			break;
		}

		return PX4_ERROR;
	}

	PX4_INFO("loaded from %s", filename);
	return PX4_OK;
}
#endif

int VtxTable::custom_command(int argc, char *argv[])
{
	if (!strcmp(argv[0], "start")) {
		if (is_running()) {
			return print_usage("already running");
		}

		int ret = VtxTable::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	if (!strcmp(argv[0], "name")) {
		if (argc < 2) {
			return print_usage("not enough arguments");
		}

		return VtxTable::data.set_name(argv[1]) ? PX4_OK : PX4_ERROR;
	}

	if (!strcmp(argv[0], "band")) {
		if (argc < 5) {
			return print_usage("not enough arguments");
		}

		const size_t band = atoi(argv[1]) - 1;
		VtxTable::data.set_band_name(band, argv[2]);
		VtxTable::data.set_band_letter(band, argv[3][0]);
		VtxTable::data.set_band_attribute(band, argv[4][0] == 'F' ? vtx::Config::BandAttribute::FACTORY : vtx::Config::BandAttribute::CUSTOM);

		for (int i = 5; i < argc; i++) {
			VtxTable::data.set_frequency(band, i - 5, atoi(argv[i]));
		}
		return PX4_OK;
	}

	if (!strcmp(argv[0], "bands")) {
		if (argc < 2) {
			return print_usage("not enough arguments");
		}

		return VtxTable::data.set_bands(atoi(argv[1])) ? PX4_OK : PX4_ERROR;
	}

	if (!strcmp(argv[0], "channels")) {
		if (argc < 2) {
			return print_usage("not enough arguments");
		}

		return VtxTable::data.set_channels(atoi(argv[1])) ? PX4_OK : PX4_ERROR;
	}

	if (!strcmp(argv[0], "powervalues")) {
		uint8_t levels{1};

		for (; levels < argc; levels++) {
			VtxTable::data.set_power_value(levels - 1, atoi(argv[levels]));
		}

		return VtxTable::data.set_power_levels(levels - 1) ? PX4_OK : PX4_ERROR;
	}

	if (!strcmp(argv[0], "powerlabels")) {
		uint8_t levels{1};

		for (; levels < argc; levels++) {
			VtxTable::data.set_power_label(levels - 1, argv[levels]);
		}

		return VtxTable::data.set_power_levels(levels - 1) ? PX4_OK : PX4_ERROR;
	}

	if (!strcmp(argv[0], "powerlevels")) {
		if (argc < 2) {
			return print_usage("not enough arguments");
		}

		return VtxTable::data.set_power_levels(atoi(argv[1])) ? PX4_OK : PX4_ERROR;
	}

#ifdef CONFIG_VTXTABLE_AUX_MAP
	char *endptr{};
	strtol(argv[0], &endptr, 10);
	if (endptr != argv[0]) {
		if (argc < 7) {
			return print_usage("not enough arguments");
		}

		return VtxTable::data.set_map_entry(
			atoi(argv[0]), atoi(argv[1]) + 4 - 1, atoi(argv[2]),
			atoi(argv[3]), atoi(argv[4]), atoi(argv[5]), atoi(argv[6])) ? PX4_OK : PX4_ERROR;
	}
#endif

#ifdef CONFIG_VTXTABLE_USE_STORAGE
	const char *const filename = (argc == 2) ? argv[1] : nullptr;

	if (!strcmp(argv[0], "save")) {
		return VtxTable::store(filename);
	}

	if (!strcmp(argv[0], "load")) {
		return VtxTable::load(filename);
	}

#endif

	return print_usage("unknown command");
}

int VtxTable::task_spawn(int argc, char *argv[])
{
	auto *const instance = new VtxTable();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	return PX4_OK;
}

void VtxTable::print_frequencies()
{
	for (uint8_t b = 0; b < data.bands(); b++) {
		PX4_INFO_RAW("INFO  [vtxtable]  %c: %-*s=", data.band_letter(b), vtx::Config::NAME_LENGTH, data.band_name(b));
		for (uint8_t c = 0; c < data.channels(); c++) {
			PX4_INFO_RAW(" %4hu", data.frequency(b, c));
		}
		if (data.band_attribute(b) == vtx::Config::BandAttribute::CUSTOM) {
			PX4_INFO_RAW("  CUSTOM\n");
		} else {
			PX4_INFO_RAW("\n");
		}
	}
}

void VtxTable::print_power_levels()
{
	for (uint8_t p = 0; p < data.power_levels(); p++) {
		PX4_INFO("  %u: %2hi = %s", p + 1, data.power_value(p), data.power_label(p));
	}
}

#ifdef CONFIG_VTXTABLE_AUX_MAP
void VtxTable::print_aux_map()
{
	for (uint8_t i = 0; i < vtx::Config::MAP_LENGTH; i++) {
		uint8_t rc_channel{}, band{}, channel{};
		int8_t power_level{};
		uint16_t start_range{}, end_range{};
		data.map_entry(i, &rc_channel, &band, &channel, &power_level, &start_range, &end_range);

		if (!start_range && !end_range) { break; }

		if (!band && !channel) {
			if (power_level == -1) {
				PX4_INFO("  %2u: Ch%-2u, %4hu - %4hu = pit mode",
					 i, rc_channel, start_range, end_range);

			} else {
				PX4_INFO("  %2u: Ch%-2u, %4hu - %4hu = %s",
					 i, rc_channel, start_range, end_range,
					 data.power_label(power_level - 1));
			}

		} else {
			PX4_INFO("  %2u: Ch%-2u, %4hu - %4hu =  %u + %u",
				 i, rc_channel, start_range, end_range,
				 band, channel);
		}
	}
}
#endif

void VtxTable::print_info()
{
	if (data.bands() == 0) {
		PX4_INFO("VTX table: <not configured>");
	} else {
		PX4_INFO("VTX table %ux%u: %s", data.bands(), data.channels(), data.name());
		print_frequencies();
	}

	if (data.power_levels() == 0) {
		PX4_INFO("Power levels: <not configured>");
	} else {
		PX4_INFO("Power levels:");
		print_power_levels();
	}

#ifdef CONFIG_VTXTABLE_AUX_MAP
	if (data.map_size() == 0) {
		PX4_INFO("RC mapping: <not configured>");
	} else {
		PX4_INFO("RC mapping:");
		print_aux_map();
	}
#endif
}

int VtxTable::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Manages the VTX frequency, power level and RC mapping table for VTX configuration.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("vtxtable", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");

	PRINT_MODULE_USAGE_COMMAND_DESCR("name", "Sets the VTX table name: <name>");

	PRINT_MODULE_USAGE_COMMAND_DESCR("bands", "Sets the number of bands: int");
	PRINT_MODULE_USAGE_COMMAND_DESCR("band", "Sets the band frequencies: <1-index> <name> <letter> <attribute> <8x frequency>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("channels", "Sets the number of channels: int");

	PRINT_MODULE_USAGE_COMMAND_DESCR("powerlevels", "Sets number of power levels: <number of power levels>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("powervalues", "Sets the ≤8 power levels values: <level-0> <level-1>...");
	PRINT_MODULE_USAGE_COMMAND_DESCR("powerlabels", "Sets the ≤8 power levels in mW: <mW level-0> <mW level-1>...");

#ifdef CONFIG_VTXTABLE_AUX_MAP
	PRINT_MODULE_USAGE_COMMAND_DESCR("<int>", "Sets an entry in the mapping table: <index> <aux channel> <band> <channel> <power level> <start range> <end range>");
#endif

#ifdef CONFIG_VTXTABLE_USE_STORAGE
	PRINT_MODULE_USAGE_COMMAND_DESCR("save", "Saves the current VTX config to a file: <file>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("load", "Loads the VTX config from a file: <file>");
#endif

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int vtxtable_main(int argc, char *argv[])
{
	return VtxTable::main(argc, argv);
}
