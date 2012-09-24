/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
 * @file boardinfo.c
 * autopilot and carrier board information app
 */


#include <nuttx/config.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "systemlib/systemlib.h"

__EXPORT int boardinfo_main(int argc, char *argv[]);

/**
 * Reads out the board information
 *
 * @param argc the number of string arguments (including the executable name)
 * @param argv the argument strings
 *
 * @return 0 on success, 1 on error
 */
int boardinfo_main(int argc, char *argv[])
{
	const char *commandline_usage = "\tusage: boardinfo [-c|-f] [-t id] [-w \"<info>\"]\n";

	bool carrier_mode = false;
	bool fmu_mode = false;
	bool write_mode = false;
	char *write_string = 0;
	bool silent = false;
	bool test_enabled = false;
	int test_boardid = -1;
	int ch;

	while ((ch = getopt(argc, argv, "cft:w:v")) != -1) {
		switch (ch) {
		case 'c':
			carrier_mode = true;
			break;

		case 'f':
			fmu_mode = true;
			break;

		case 't':
			test_enabled = true;
			test_boardid = strtol(optarg, NULL, 10);
			silent = true;
			break;

		case 'w':
			write_mode = true;
			write_string = optarg;
			break;

		default:
			printf(commandline_usage);
			exit(0);
		}
	}

	/* Check if write is required - only one mode is allowed then */
	if (write_mode && fmu_mode && carrier_mode) {
		fprintf(stderr, "[boardinfo] Please choose only one mode for writing: --carrier or --fmu\n");
		printf(commandline_usage);
		return ERROR;
	}

	/* Write FMU information
	if (fmu_mode && write_mode) {
		struct fmu_board_info_s info;
		int ret = fmu_store_board_info(&info);


		if (ret == OK) {
			printf("[boardinfo] Successfully wrote FMU board info\n");
		} else {
			fprintf(stderr, "[boardinfo] ERROR writing board info to FMU EEPROM, aborting\n");
			return ERROR;
		}
	}*/

	/* write carrier board info */
	if (carrier_mode && write_mode) {

		struct carrier_board_info_s info;
		bool parse_ok = true;
		unsigned parse_idx = 0;
		//int maxlen = strlen(write_string);
		char *curr_char;

		/* Parse board info string */
		if (write_string[0] != 'P' || write_string[1] != 'X' || write_string[2] != '4') {
			fprintf(stderr, "[boardinfo] header must start with 'PX4'\n");
			parse_ok = false;
		}

		info.header[0] = 'P'; info.header[1] = 'X'; info.header[2] = '4';
		parse_idx = 3;
		/* Copy board name */

		int i = 0;

		while (write_string[parse_idx] != 0x20 && (parse_idx < sizeof(info.board_name) + sizeof(info.header))) {
			info.board_name[i] = write_string[parse_idx];
			i++; parse_idx++;
		}

		/* Enforce null termination */
		info.board_name[sizeof(info.board_name) - 1] = '\0';

		curr_char = write_string + parse_idx;

		/* Index is now on next field */
		info.board_id = strtol(curr_char, &curr_char, 10);//atoi(write_string + parse_index);
		info.board_version = strtol(curr_char, &curr_char, 10);

		/* Read in multi ports */
		for (i = 0; i < MULT_COUNT; i++) {
			info.multi_port_config[i] = strtol(curr_char, &curr_char, 10);
		}

		/* Read in production data */
		info.production_year = strtol(curr_char, &curr_char, 10);

		if (info.production_year < 2012 || info.production_year > 3000) {
			fprintf(stderr, "[boardinfo] production year is invalid: %d\n", info.production_year);
			parse_ok = false;
		}

		info.production_month = strtol(curr_char, &curr_char, 10);

		if (info.production_month < 1 || info.production_month > 12) {
			fprintf(stderr, "[boardinfo] production month is invalid: %d\n", info.production_month);
			parse_ok = false;
		}

		info.production_day = strtol(curr_char, &curr_char, 10);

		if (info.production_day < 1 || info.production_day > 31) {
			fprintf(stderr, "[boardinfo] production day is invalid: %d\n", info.production_day);
			parse_ok = false;
		}

		info.production_fab = strtol(curr_char, &curr_char, 10);
		info.production_tester = strtol(curr_char, &curr_char, 10);

		if (!parse_ok) {
			/* Parsing failed */
			fprintf(stderr, "[boardinfo] failed parsing info string:\n\t%s\naborting\n", write_string);
			return ERROR;

		} else {
			int ret = carrier_store_board_info(&info);

			/* Display result */
			if (ret == sizeof(info)) {
				printf("[boardinfo] Successfully wrote carrier board info\n");

			} else {
				fprintf(stderr, "[boardinfo] ERROR writing board info to carrier EEPROM (forgot to pull the WRITE_ENABLE line high?), aborting\n");
				return ERROR;
			}
		}
	}

	/* Print FMU information */
	if (fmu_mode && !silent) {
		struct fmu_board_info_s info;
		int ret = fmu_get_board_info(&info);

		/* Print human readable name */
		if (ret == sizeof(info)) {
			printf("[boardinfo] Autopilot:\n\t%s\n", info.header);

		} else {
			fprintf(stderr, "[boardinfo] ERROR loading board info from FMU, aborting\n");
			return ERROR;
		}
	}

	/* print carrier information */
	if (carrier_mode && !silent) {

		struct carrier_board_info_s info;
		int ret = carrier_get_board_info(&info);

		/* Print human readable name */
		if (ret == sizeof(info)) {
			printf("[boardinfo] Carrier board:\n\t%s\n", info.header);
			printf("\tboard id:\t\t%d\n", info.board_id);
			printf("\tversion:\t\t%d\n", info.board_version);

			for (unsigned i = 0; i < MULT_COUNT; i++) {
				printf("\tmulti port #%d:\t\t%s: function #%d\n", i, multiport_info.port_names[i], info.multi_port_config[i]);
			}

			printf("\tproduction date:\t%d-%d-%d (fab #%d / tester #%d)\n", info.production_year, info.production_month, info.production_day, info.production_fab, info.production_tester);

		} else {
			fprintf(stderr, "[boardinfo] ERROR loading board info from carrier EEPROM (errno #%d), aborting\n", -ret);
			return ERROR;
		}
	}

	/* test for a specific carrier */
	if (test_enabled) {

		struct carrier_board_info_s info;
		int ret = carrier_get_board_info(&info);

		if (ret != sizeof(info)) {
			fprintf(stderr, "[boardinfo] no EEPROM for board %d\n", test_boardid);
			exit(1);
		}

		if (info.board_id == test_boardid) {
			printf("[boardinfo] Found carrier board with ID %d, test succeeded\n", info.board_id);
			exit(0);

		} else {
			/* exit silently with an error so we can test for multiple boards quietly */
			exit(1);
		}
	}

	return 0;
}


