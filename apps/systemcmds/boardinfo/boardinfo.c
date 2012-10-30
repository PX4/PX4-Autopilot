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

#include <nuttx/i2c.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/bson/tinybson.h>

__EXPORT int boardinfo_main(int argc, char *argv[]);

#if 1

struct eeprom_info_s
{
	unsigned	bus;
	unsigned	address;
	unsigned	page_size;
	unsigned	page_count;
	unsigned	page_write_delay;
};

/* XXX currently code below only supports 8-bit addressing */
const struct eeprom_info_s eeprom_info[] = {
	{3, 0x57, 8, 16, 3300},
	{0, 0, 0, 0, 0}
};

struct board_parameter_s {
	const char	*name;
	bson_type_t	type;
};

const struct board_parameter_s board_parameters[] = {
	{"name",	BSON_STRING},	/* ascii board name */
	{"vers",	BSON_INT32},	/* board version (major << 8) | minor */
	{"date",	BSON_INT32},	/* manufacture date */
	{"build",	BSON_INT32}	/* build code (fab << 8) | tester */
};

const unsigned num_parameters = sizeof(board_parameters) / sizeof(board_parameters[0]);

static int
eeprom_write(const struct eeprom_info_s *eeprom, uint8_t *buf, unsigned size)
{
	int result = -1;

	struct i2c_dev_s *dev = up_i2cinitialize(eeprom->bus);
	if (dev == NULL) {
		warnx("failed to init bus %d for EEPROM", eeprom->bus);
		goto out;
	}
	I2C_SETFREQUENCY(dev, 400000);

	/* loop until all data has been transferred */
	for (unsigned address = 0; address < size; ) {

		uint8_t pagebuf[eeprom->page_size + 1];

		/* how many bytes available to transfer? */
		unsigned count = size - address;

		/* constrain writes to the page size */
		if (count > eeprom->page_size)
			count = eeprom->page_size;

		pagebuf[0] = address & 0xff;
		memcpy(pagebuf + 1, buf + address, count);

		struct i2c_msg_s msgv[1] = {
			{
				.addr = eeprom->address,
				.flags = 0,
				.buffer = pagebuf,
				.length = count + 1
			}
		};

		result = I2C_TRANSFER(dev, msgv, 1);
		if (result != OK) {
			warnx("EEPROM write failed: %d", result);
			goto out;
		}
		usleep(eeprom->page_write_delay);
		address += count;
	}

out:
	if (dev != NULL)
		up_i2cuninitialize(dev);
	return result;
}

static int
eeprom_read(const struct eeprom_info_s *eeprom, uint8_t *buf, unsigned size)
{
	int result = -1;

	struct i2c_dev_s *dev = up_i2cinitialize(eeprom->bus);
	if (dev == NULL) {
		warnx("failed to init bus %d for EEPROM", eeprom->bus);
		goto out;
	}
	I2C_SETFREQUENCY(dev, 400000);

	/* loop until all data has been transferred */
	for (unsigned address = 0; address < size; ) {

		/* how many bytes available to transfer? */
		unsigned count = size - address;

		/* constrain transfers to the page size (bus anti-hog) */
		if (count > eeprom->page_size)
			count = eeprom->page_size;

		uint8_t addr = address;
		struct i2c_msg_s msgv[2] = {
			{
				.addr = eeprom->address,
				.flags = 0,
				.buffer = &addr,
				.length = 1
			},
			{
				.addr = eeprom->address,
				.flags = I2C_M_READ,
				.buffer = buf + address,
				.length = count
			}
		};

		result = I2C_TRANSFER(dev, msgv, 2);
		if (result != OK) {
			warnx("EEPROM read failed: %d", result);
			goto out;
		}
		address += count;
	}

out:
	if (dev != NULL)
		up_i2cuninitialize(dev);
	return result;
}

static void *
idrom_read(const struct eeprom_info_s *eeprom)
{
	uint32_t size = 0xffffffff;
	int result;
	void *buf = NULL;

	result = eeprom_read(eeprom, (uint8_t *)&size, sizeof(size));
	if (result != 0) {
		warnx("failed reading ID ROM length");
		goto fail;
	}
	if (size > (eeprom->page_size * eeprom->page_count)) {
		warnx("ID ROM not programmed");
		goto fail;
	}

	buf = malloc(size);
	if (buf == NULL) {
		warnx("could not allocate %d bytes for ID ROM", size);
		goto fail;
	}
	result = eeprom_read(eeprom, buf, size);
	if (result != 0) {
		warnx("failed reading ID ROM");
		goto fail;
	}
	return buf;

fail:
	if (buf != NULL)
		free(buf);
	return NULL;
}

static void
boardinfo_set(const struct eeprom_info_s *eeprom, char *spec)
{
	struct bson_encoder_s encoder;
	int result = 1;
	char *state, *token;
	unsigned i;

	/* create the encoder and make a writable copy of the spec */
	bson_encoder_init_buf(&encoder, NULL, 0);

	for (i = 0, token = strtok_r(spec, ",", &state);
	     token && (i < num_parameters);
	     i++, token = strtok_r(NULL, ",", &state)) {

		switch (board_parameters[i].type) {
		case BSON_STRING:
			result = bson_encoder_append_string(&encoder, board_parameters[i].name, token);
			break;
		case BSON_INT32:
			result = bson_encoder_append_int(&encoder, board_parameters[i].name, strtoul(token, NULL, 0));
			break;
		default:
			result = 1;
		}
		if (result) {
			warnx("bson append failed for %s<%s>", board_parameters[i].name, token);
			goto out;
		}
	}
	bson_encoder_fini(&encoder);
	if (i != num_parameters) {
		warnx("incorrect parameter list, expected: \"<name>,<version><date>,<buildcode>\"");
		result = 1;
		goto out;
	}
	if (bson_encoder_buf_size(&encoder) > (int)(eeprom->page_size * eeprom->page_count)) {
		warnx("data too large for EEPROM");
		result = 1;
		goto out;
	}
	if ((int)*(uint32_t *)bson_encoder_buf_data(&encoder) != bson_encoder_buf_size(&encoder)) {
		warnx("buffer length mismatch");
		result = 1;
		goto out;
	}
	warnx("writing %p/%u", bson_encoder_buf_data(&encoder), bson_encoder_buf_size(&encoder));

	result = eeprom_write(eeprom, (uint8_t *)bson_encoder_buf_data(&encoder), bson_encoder_buf_size(&encoder));
	if (result < 0) {
		warnc(-result, "error writing EEPROM");
		result = 1;
	} else {
		result = 0;
	}

out:
	free(bson_encoder_buf_data(&encoder));

	exit(result);
}

static int
boardinfo_print(bson_decoder_t decoder, void *private, bson_node_t node)
{
	switch (node->type) {
	case BSON_INT32:
		printf("%s: %d / 0x%08x\n", node->name, (int)node->i, (unsigned)node->i);
		break;
	case BSON_STRING: {
		char buf[bson_decoder_data_pending(decoder)];
		bson_decoder_copy_data(decoder, buf);
		printf("%s: %s\n", node->name, buf);
		break;
	}
	case BSON_EOO:
		break;
	default:
		warnx("unexpected node type %d", node->type);
		break;
	}
	return 1;
}

static void
boardinfo_show(const struct eeprom_info_s *eeprom)
{
	struct bson_decoder_s decoder;
	void *buf;

	buf = idrom_read(eeprom);
	if (buf == NULL)
		errx(1, "ID ROM read failed");

	if (bson_decoder_init_buf(&decoder, buf, 0, boardinfo_print, NULL) == 0) {
		while (bson_decoder_next(&decoder) > 0)
			;
	} else {
		warnx("failed to init decoder");
	}
	free(buf);
	exit(0);
}

struct {
	const char *property;
	const char *value;
} test_args;

static int
boardinfo_test_callback(bson_decoder_t decoder, void *private, bson_node_t node)
{
	/* reject nodes with non-matching names */
	if (strcmp(node->name, test_args.property))
		return 1;

	/* compare node values to check for a match */
	switch (node->type) {
	case BSON_STRING: {
		char buf[bson_decoder_data_pending(decoder)];
		bson_decoder_copy_data(decoder, buf);

		/* check for a match */
		if (!strcmp(test_args.value, buf)) {
			return 2;
		}
		break;
	}

	case BSON_INT32: {
		int32_t val = strtol(test_args.value, NULL, 0);

		/* check for a match */
		if (node->i == val) {
			return 2;
		}
		break;
	}

	default:
		break;
	}

	return 1;
}

static void
boardinfo_test(const struct eeprom_info_s *eeprom, const char *property, const char *value)
{
	struct bson_decoder_s decoder;
	void *buf;
	int result = -1;

	if ((property == NULL) || (strlen(property) == 0) ||
	    (value == NULL) || (strlen(value) == 0))
		errx(1, "missing property name or value");

	test_args.property = property;
	test_args.value = value;

	buf = idrom_read(eeprom);
	if (buf == NULL)
		errx(1, "ID ROM read failed");

	if (bson_decoder_init_buf(&decoder, buf, 0, boardinfo_test_callback, NULL) == 0) {
		do {
			result = bson_decoder_next(&decoder);
		} while (result == 1);
	} else {
		warnx("failed to init decoder");
	}
	free(buf);

	/* if we matched, we exit with zero success */	
	exit((result == 2) ? 0 : 1);
}

int
boardinfo_main(int argc, char *argv[])
{
	if (!strcmp(argv[1], "set"))
		boardinfo_set(&eeprom_info[0], argv[2]);

	if (!strcmp(argv[1], "show"))
		boardinfo_show(&eeprom_info[0]);

	if (!strcmp(argv[1], "test"))
		boardinfo_test(&eeprom_info[0], argv[2], argv[3]);

	errx(1, "missing/unrecognised command, try one of 'set', 'show', 'test'");
}

#else
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


#endif