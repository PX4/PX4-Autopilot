/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file gpio_test.cpp
 * @author Julian Kent <julian@auterion.com>
 *
 * GPIO read and write tool
 */

#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_config.h>


__BEGIN_DECLS
__EXPORT int gpio_main(int argc, char *argv[]);
__END_DECLS


static void
usage(const char *reason)
{
	printf("FAIL\n");

	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This command is used to read and write GPIOs

gpio read <PORT> <PIN> [PULLDOWN|PULLUP]
gpio write <PORT> <PIN> <VALUE> [OPENDRAIN]

### Examples
Read the value on port H pin 4 configured as pullup, and it is high
$ gpio_test read H 4 PU
1 OK

Set the output value on Port E pin 7 to high
$ gpio_test write E 7 1
OK

)DESCR_STR");

}

int
gpio_main(int argc, char *argv[])
{


	if (argc < 4) {
		usage("not enough arguments");
		return -1;
	}

	const char* command = argv[1];
	bool is_read = false;
	if (strcmp(command, "read") == 0) {
		is_read = true;
	} else if (strcmp(command, "write") != 0) {
		usage("command not read or write");
		return -1;
	}

	int mask = 0;
	const char* port_string = argv[2];
	if (strlen(port_string) != 1) {
		usage("port not a single character");
	}
	char port = port_string[0];
	if ('A' <= port && port <= 'K') {
		mask |= ((port - 'A') << GPIO_PORT_SHIFT) & GPIO_PORT_MASK;
	} else if ('a' <= port && port <= 'k') {
		mask |= ((port - 'a') << GPIO_PORT_SHIFT) & GPIO_PORT_MASK;
	}else {
		usage("invalid port");
		return -1;
	}

	char *end;
	int pin = strtol(argv[3], &end, 10);
	if (errno == 0 && *end == '\0' && 0 <= pin && pin <= 15) {
		mask |= (pin << GPIO_PIN_SHIFT) & GPIO_PIN_MASK;;
	} else {
		usage("invalid pin");
		return -1;
	}

	if (is_read) {
		if (argc == 5) {
			const char* extra = argv[4];
			if (strcasecmp(extra, "PULLUP") == 0) {
				mask |= GPIO_PULLUP;
			} else if (strcasecmp(extra, "PULLDOWN") == 0) {
				mask |= GPIO_PULLDOWN;
			} else {
				usage ("extra read argument not PULLUP or PULLDOWN");
				return -1;
			}
		}

		int value = px4_arch_gpioread(mask);
		printf("%d OK\n", value);
	} else {

		if (argc < 5) {
			usage("not enough arguments");
			return -1;
		}

		int value = strtol(argv[4], &end, 10);
		if (errno != 0 || *end != '\0' || (value != 0 && value != 1)) {
			usage("value not 0 or 1");
			return -1;
		}

		if (argc == 6) {
			const char* extra = argv[5];
			if (strcasecmp(extra, "OPENDRAIN") == 0) {
				mask |= GPIO_OPENDRAIN;
			} else {
				usage("extra write argument not OPENDRAIN");
				return -1;
			}
		}

		px4_arch_gpiowrite(mask, value);
		printf("OK\n");
	}
	return 0;
}
