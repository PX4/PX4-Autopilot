/****************************************************************************
*
* Copyright (c) 2025 PX4 Development Team. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in
* the documentation and/or other materials provided with the
* distribution.
* 3. Neither the name PX4 nor the names of its contributors may be
* used to endorse or promote products derived from this software
* without specific prior written permission.
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
* @file mft_cfg.c
*/

#include <board_config.h>
#include <px4_platform_common/board_common.h>
#include <px4_platform/board_hw_eeprom_rev_ver.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/px4_manifest.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>


static void usage(const char *reason)
{
	if (reason != nullptr) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION("Tool to set and get manifest configuration");

	PRINT_MODULE_USAGE_NAME("mft_cfg", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("get", "get manifest configuration");
	PRINT_MODULE_USAGE_COMMAND_DESCR("set", "set manifest configuration");
	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "reset manifest configuration");
	PRINT_MODULE_USAGE_ARG("hwver|hwrev", "Select type: MTD_MTF_VER|MTD_MTF_REV", false);
	PRINT_MODULE_USAGE_PARAM_INT('i', 0x10, 0x10, 0xFFFF,
				     "argument to set extended hardware id (id == version for <hwver>, id == revision for <hwrev> )", false);

}


static int print_extended_id(const char *type)
{
	mtd_mft_v0_t mtd_mft;
	mtd_mft.version.id = MTD_MFT_v0;
	mtd_mft.hw_extended_id = -1;

	const char *path = nullptr;
	int ret_val = px4_mtd_query(type, NULL, &path);

	if (ret_val != PX4_OK) {
		PX4_ERR("Can't get mtd query (%s, %i)", type, ret_val);

	} else {

		ret_val = board_get_eeprom_hw_info(path, (mtd_mft_t *)&mtd_mft);

		if (ret_val == PX4_OK) {
			PX4_INFO("%s, hw_extended_id = %#x", type, mtd_mft.hw_extended_id);

		} else  {

			if (ret_val == -EPROTO) {
				PX4_ERR("Manifest data may not exist for %s", path);

			} else {
				PX4_ERR("Can't read hw_extended_id from EEPROM (%s, %i)", type, ret_val);
			}
		}
	}

	return ret_val;
}

extern "C" __EXPORT int mft_cfg_main(int argc, char *argv[])
{
	if ((argc == 2) && (!strcmp(argv[1], "get"))) {

		char type_ver[] = "MTD_MFT_VER";
		char type_rev[] = "MTD_MFT_REV";

		print_extended_id(type_ver);
		print_extended_id(type_rev);

		return 0;

	} else if (argc >= 3) {

		int ret_val = -1;
		const char *path = nullptr;
		bool arg_exist = false;

		for (int i = 2; i < argc; ++i) {
			if (strcmp("hwver", argv[i]) == 0) {
				ret_val = px4_mtd_query("MTD_MFT_VER", NULL, &path);
				arg_exist = true;
				break;
			}

			if (strcmp("hwrev", argv[i]) == 0) {
				ret_val = px4_mtd_query("MTD_MFT_REV", NULL, &path);
				arg_exist = true;
				break;
			}
		}

		if (!arg_exist) {
			PX4_ERR("Missing <hwver> or <hwrev> arguments'");
			return 1;
		}

		if (ret_val != PX4_OK) {
			PX4_ERR("Can't get mtd query (%i)", ret_val);
			return 1;
		}

		mtd_mft_v0_t mtd_mft;
		mtd_mft.version.id = MTD_MFT_v0;
		mtd_mft.hw_extended_id = -1;

		if (!strcmp(argv[1], "set")) {
			if (argc == 5) {

				const char *myoptarg = NULL;
				int ch = 0;
				int myoptind = 1;
				int hw_extended_id = -1;

				while ((ch = px4_getopt(argc, argv, "i:", &myoptind, &myoptarg)) != EOF) {
					switch (ch) {
					case 'i':
						hw_extended_id = strtol(myoptarg, NULL, 0);
						break;

					default:
						PX4_ERR("To set id use '-i x'");
						break;
					}
				}

				if (hw_extended_id != -1) {

					mtd_mft.hw_extended_id = (uint16_t)hw_extended_id;

					ret_val = board_set_eeprom_hw_info(path, (mtd_mft_t *)&mtd_mft);

					if (ret_val != PX4_OK) {
						PX4_ERR("Can't write to EEPROM (%i)", ret_val);

					} else {
						board_get_eeprom_hw_info(path, (mtd_mft_t *)&mtd_mft);
						PX4_INFO("New hw_extended_id = %#x", mtd_mft.hw_extended_id);
					}
				}

			} else {
				PX4_ERR("Not enough arguments, try 'mft_cfg set hwver -i x'");
				return 1;
			}

			return 0;
		}

		if (!strcmp(argv[1], "get")) {

			ret_val = board_get_eeprom_hw_info(path, (mtd_mft_t *)&mtd_mft);

			if (ret_val == PX4_OK) {
				PX4_INFO("hw_extended_id = %#x", mtd_mft.hw_extended_id);

			} else {

				if (ret_val == -EPROTO) {
					PX4_ERR("Manifest data may not exist for %s", path);

				} else {
					PX4_ERR("Can't read from EEPROM (%s, %i)", path, ret_val);
				}

				return 1;
			}

			return 0;
		}

		if (!strcmp(argv[1], "reset")) {

			uint8_t buffer[64];
			memset(buffer, 0xFF, sizeof(buffer));

			int fd = open(path, O_WRONLY);

			if (fd == -1) {
				PX4_ERR("Failed to open partition %s", path);
				return 1;
			}

			while (write(fd, buffer, sizeof(buffer)) == sizeof(buffer)) {}

			PX4_INFO("Reset for %s completed. To remove manifest data from RAM, a reboot is required.", path);
			close(fd);

			return 0;
		}

	} else {
		usage("Error, not enough parameters.");
		return 1;
	}

	return 0;
}
