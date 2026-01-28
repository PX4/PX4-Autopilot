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

#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#ifdef CONFIG_VTXTABLE_USE_STORAGE
#include <px4_platform_common/workqueue.h>
#endif

using namespace time_literals;

static vtx::Config vtxtable_data;

vtx::Config &vtxtable()
{
	return vtxtable_data;
}

#ifdef CONFIG_VTXTABLE_USE_STORAGE
int vtxtable_store(const char *filename)
{
	if (filename == nullptr) { filename = CONFIG_VTXTABLE_CONFIG_FILE; }

	const int rv = vtxtable().store(filename);

	if (rv < 0) {
		PX4_ERR("%s is not accessible!", filename);
		return PX4_ERROR;
	}

	PX4_INFO("saved to %s", filename);
	return PX4_OK;
}

int vtxtable_load(const char *filename)
{
	if (filename == nullptr) { filename = CONFIG_VTXTABLE_CONFIG_FILE; }

	const int rv = vtxtable().load(filename);

	if (rv < 0) {
		switch (rv) {
		case -ENOENT:
			PX4_ERR("%s not found!", filename);
			break;

		case -EPROTO:
			PX4_ERR("VTX config serialization format is unsupported in %s!", filename);
			break;

		case -EBADMSG:
			PX4_ERR("VTX config has a corrupt CRC in %s!", filename);
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

static struct work_s storing_work;
#endif

static void vtxtable_autosave()
{
#ifdef CONFIG_VTXTABLE_USE_STORAGE
	work_queue(LPWORK, &storing_work, [](void *) { vtxtable_store(nullptr); }, nullptr, USEC2TICK(1_s));
#endif
}

static int vtxtable_print_usage(const char *reason)
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

	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Shows the current VTX table configuration.");

	PRINT_MODULE_USAGE_COMMAND_DESCR("name", "Sets the VTX table name: <string>");

	PRINT_MODULE_USAGE_COMMAND_DESCR("bands", "Sets the number of bands: <int>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("band", "Sets the band frequencies: <1-index> <name> <letter> <attribute> <frequencies...>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("channels", "Sets the number of channels: <int>");

	PRINT_MODULE_USAGE_COMMAND_DESCR("powerlevels", "Sets number of power levels: <int>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("powervalues", "Sets the power level values: <int...>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("powerlabels", "Sets the power level labels: <3 chars...>");

#ifdef CONFIG_VTXTABLE_AUX_MAP
	PRINT_MODULE_USAGE_COMMAND_DESCR("<int>", "Sets an entry in the mapping table: <0-index> <aux channel> <band> <channel> <power level> <start range> <end range>");
#endif
	PRINT_MODULE_USAGE_COMMAND_DESCR("clear", "Clears the VTX table configuration.");

#ifdef CONFIG_VTXTABLE_USE_STORAGE
	PRINT_MODULE_USAGE_COMMAND_DESCR("save", "Saves the VTX config to a file: <file>");
	PRINT_MODULE_USAGE_COMMAND_DESCR("load", "Loads the VTX config from a file: <file>");
#endif

	return 0;
}

void vtxtable_print_status()
{
	if (vtxtable().bands() == 0) {
		PX4_INFO("VTX table: <not configured>");

	} else {
		PX4_INFO("VTX table %ux%u: %s", vtxtable().bands(), vtxtable().channels(), vtxtable().name());
		vtxtable_print_frequencies();
	}

	if (vtxtable().power_levels() == 0) {
		PX4_INFO("Power levels: <not configured>");

	} else {
		PX4_INFO("Power levels:");
		vtxtable_print_power_levels();
	}

#ifdef CONFIG_VTXTABLE_AUX_MAP

	if (vtxtable().map_size() == 0) {
		PX4_INFO("RC mapping: <not configured>");

	} else {
		PX4_INFO("RC mapping:");
		vtxtable_print_aux_map();
	}

#endif
}

int vtxtable_custom_command(int argc, char *argv[])
{
	if (!strcmp(argv[0], "status")) {
		vtxtable_print_status();
		return PX4_OK;
	}

	if (!strcmp(argv[0], "name")) {
		if (argc < 2) {
			return vtxtable_print_usage("not enough arguments");
		}

		vtxtable_autosave();
		return vtxtable().set_name(argv[1]) ? PX4_OK : PX4_ERROR;
	}

	if (!strcmp(argv[0], "band")) {
		if (argc < 5) {
			return vtxtable_print_usage("not enough arguments");
		}

		const size_t band = atoi(argv[1]) - 1;
		vtxtable().set_band_name(band, argv[2]);
		vtxtable().set_band_letter(band, argv[3][0]);
		vtxtable().set_band_attribute(band,
						  argv[4][0] == 'F' ? vtx::Config::BandAttribute::FACTORY : vtx::Config::BandAttribute::CUSTOM);

		for (int i = 5; i < argc; i++) {
			vtxtable().set_frequency(band, i - 5, atoi(argv[i]));
		}

		vtxtable_autosave();
		return PX4_OK;
	}

	if (!strcmp(argv[0], "bands")) {
		if (argc < 2) {
			return vtxtable_print_usage("not enough arguments");
		}

		vtxtable_autosave();
		return vtxtable().set_bands(atoi(argv[1])) ? PX4_OK : PX4_ERROR;
	}

	if (!strcmp(argv[0], "channels")) {
		if (argc < 2) {
			return vtxtable_print_usage("not enough arguments");
		}

		vtxtable_autosave();
		return vtxtable().set_channels(atoi(argv[1])) ? PX4_OK : PX4_ERROR;
	}

	if (!strcmp(argv[0], "powervalues")) {
		uint8_t levels{1};

		for (; levels < argc; levels++) {
			vtxtable().set_power_value(levels - 1, atoi(argv[levels]));
		}

		vtxtable_autosave();
		return PX4_OK;
	}

	if (!strcmp(argv[0], "powerlabels")) {
		uint8_t levels{1};

		for (; levels < argc; levels++) {
			vtxtable().set_power_label(levels - 1, argv[levels]);
		}

		vtxtable_autosave();
		return PX4_OK;
	}

	if (!strcmp(argv[0], "powerlevels")) {
		if (argc < 2) {
			return vtxtable_print_usage("not enough arguments");
		}

		vtxtable_autosave();
		return vtxtable().set_power_levels(atoi(argv[1])) ? PX4_OK : PX4_ERROR;
	}

#ifdef CONFIG_VTXTABLE_AUX_MAP
	char *endptr {};
	strtol(argv[0], &endptr, 10);

	if (endptr != argv[0]) {
		if (argc < 7) {
			return vtxtable_print_usage("not enough arguments");
		}

		vtxtable_autosave();
		return vtxtable().set_map_entry(
			       atoi(argv[0]), atoi(argv[1]) + 4 - 1, atoi(argv[2]),
			       atoi(argv[3]), atoi(argv[4]), atoi(argv[5]), atoi(argv[6])) ? PX4_OK : PX4_ERROR;
	}

#endif

	if (!strcmp(argv[0], "clear")) {
		vtxtable().clear();
		return PX4_OK;
	}

#ifdef CONFIG_VTXTABLE_USE_STORAGE
	const char *const filename = (argc == 2) ? argv[1] : nullptr;

	if (!strcmp(argv[0], "save")) {
		return vtxtable_store(filename);
	}

	if (!strcmp(argv[0], "load")) {
		return vtxtable_load(filename);
	}

#endif

	return vtxtable_print_usage("unknown command");
}

void vtxtable_print_frequencies()
{
	for (uint8_t b = 0; b < vtxtable().bands(); b++) {
		PX4_INFO_RAW("INFO  [vtxtable]  %c: %-*s=", vtxtable().band_letter(b), vtx::Config::NAME_LENGTH, vtxtable().band_name(b));

		for (uint8_t c = 0; c < vtxtable().channels(); c++) {
			PX4_INFO_RAW(" %4hu", vtxtable().frequency(b, c));
		}

		if (vtxtable().band_attribute(b) == vtx::Config::BandAttribute::CUSTOM) {
			PX4_INFO_RAW("  CUSTOM\n");

		} else {
			PX4_INFO_RAW("\n");
		}
	}
}

void vtxtable_print_power_levels()
{
	for (uint8_t p = 0; p < vtxtable().power_levels(); p++) {
		PX4_INFO("  %u: %2hi = %s", p + 1, vtxtable().power_value(p), vtxtable().power_label(p));
	}
}

#ifdef CONFIG_VTXTABLE_AUX_MAP
void vtxtable_print_aux_map()
{
	for (uint8_t i = 0; i < vtx::Config::MAP_LENGTH; i++) {
		uint8_t rc_channel{}, band{}, channel{};
		int8_t power_level{};
		uint16_t start_range{}, end_range{};
		vtxtable().map_entry(i, &rc_channel, &band, &channel, &power_level, &start_range, &end_range);

		if (!start_range && !end_range) { break; }

		if (!band && !channel) {
			if (power_level == -1) {
				PX4_INFO("  %2u: Ch%-2u, %4hu - %4hu = pit mode",
					 i, rc_channel, start_range, end_range);

			} else {
				PX4_INFO("  %2u: Ch%-2u, %4hu - %4hu = %s",
					 i, rc_channel, start_range, end_range,
					 vtxtable().power_label(power_level - 1));
			}

		} else {
			PX4_INFO("  %2u: Ch%-2u, %4hu - %4hu =  %u + %u",
				 i, rc_channel, start_range, end_range,
				 band, channel);
		}
	}
}
#endif

extern "C" __EXPORT int vtxtable_main(int argc, char *argv[])
{
	if (argc <= 1) {
		return vtxtable_print_usage("missing command");
	}
	argc--; argv++;
	return vtxtable_custom_command(argc, argv);
}
