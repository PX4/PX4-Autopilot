/****************************************************************************
*
* Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
* @file ver.c
*
* Version command, unifies way of showing versions of HW, SW, Build, GCC
* In case you want to add new version just extend version_main function
*
* @author Vladimir Kulla <ufon@kullaonline.net>
*/

#include <stdio.h>
#include <string.h>
#include <version/version.h>
#include <systemlib/err.h>

// string constants for version commands
static const char sz_ver_hw_str[] 	= "hw";
static const char sz_ver_hwcmp_str[] = "hwcmp";
static const char sz_ver_git_str[] 	= "git";
static const char sz_ver_bdate_str[] = "bdate";
static const char sz_ver_gcc_str[] 	= "gcc";
static const char sz_ver_all_str[] 	= "all";

#ifdef CONFIG_ARCH_CHIP_STM32
#include <up_arch.h>

static const char mcu_ver_str[]		= "mcu";

#define DBGMCU_IDCODE	0xE0042000

#define STM32F40x_41x	0x413
#define STM32F42x_43x	0x419

#define REVID_MASK	0xFFFF0000
#define DEVID_MASK	0xFFF

/* magic numbers from reference manual */
enum STM32F4_REV {
	STM32F4_REV_A = 0x1000,
	STM32F4_REV_Z = 0x1001,
	STM32F4_REV_Y = 0x1003,
	STM32F4_REV_1 = 0x1007,
	STM32F4_REV_3 = 0x2001
};
#else
#error stm32
#endif

static void usage(const char *reason)
{
	if (reason != NULL) {
		printf("%s\n", reason);
	}

	printf("usage: ver {hw|hwcmp|git|bdate|gcc|all|mcu}\n\n");
}

__EXPORT int ver_main(int argc, char *argv[]);

int ver_main(int argc, char *argv[])
{
	int ret = 1;	//defaults to an error

	// first check if there are at least 2 params
	if (argc >= 2) {
		if (argv[1] != NULL) {
			if (!strncmp(argv[1], sz_ver_hw_str, sizeof(sz_ver_hw_str))) {
				printf("%s\n", HW_ARCH);
				ret = 0;

			} else if (!strncmp(argv[1], sz_ver_hwcmp_str, sizeof(sz_ver_hwcmp_str))) {
				if (argc >= 3 && argv[2] != NULL) {
					// compare 3rd parameter with HW_ARCH string, in case of match, return 0
					ret = strncmp(HW_ARCH, argv[2], strlen(HW_ARCH));

					if (ret == 0) {
						printf("ver hwcmp match: %s\n", HW_ARCH);
					}

				} else {
					errx(1, "Not enough arguments, try 'ver hwcmp PX4FMU_V2'");
				}

			} else if (!strncmp(argv[1], sz_ver_git_str, sizeof(sz_ver_git_str))) {
				printf("%s\n", FW_GIT);
				ret = 0;

			} else if (!strncmp(argv[1], sz_ver_bdate_str, sizeof(sz_ver_bdate_str))) {
				printf("%s %s\n", __DATE__, __TIME__);
				ret = 0;

			} else if (!strncmp(argv[1], sz_ver_gcc_str, sizeof(sz_ver_gcc_str))) {
				printf("%s\n", __VERSION__);
				ret = 0;

			} else if (!strncmp(argv[1], sz_ver_all_str, sizeof(sz_ver_all_str))) {
				printf("HW arch: %s\n", HW_ARCH);
				printf("Build datetime: %s %s\n", __DATE__, __TIME__);
				printf("FW git-hash: %s\n", FW_GIT);
				printf("GCC toolchain: %s\n", __VERSION__);
				ret = 0;

#ifdef CONFIG_ARCH_CHIP_STM32
			} else if (!strncmp(argv[1], mcu_ver_str, sizeof(mcu_ver_str))) {
				uint32_t abc = getreg32(DBGMCU_IDCODE);

				uint32_t chip_version = abc & DEVID_MASK;
				enum STM32F4_REV revid = (abc & REVID_MASK) >> 16;

				printf("CHIP TYPE: ");

				switch (revid) {
				case STM32F40x_41x:
					printf("STM32F40x");
					break;
				case STM32F42x_43x:
					printf("STM32F42x");
					break;
				default:
					printf("STM32F???");
					break;
				}

				char rev;

				switch (chip_version) {

					case STM32F4_REV_A:
						rev = 'A';
						break;
					case STM32F4_REV_Z:
						rev = 'Z';
						break;
					case STM32F4_REV_Y:
						rev = 'Y';
						break;
					case STM32F4_REV_1:
						rev = '1';
						break;
					case STM32F4_REV_3:
						rev = '3';
						break;
					default:
						rev = '?';
						break;
				}

				printf("\nHW REV: %c\n", rev);

				if (rev < STM32F4_REV_3) {
					printf("\n\nWARNING   WARNING   WARNING!\n"
						"Revision %c has a silicon errata\n"
						"on USB connectivity combined with\n"
						"flash bank #2. This device can only\n"
						"utilize a maximum of 1MB flash safely!\n"
						"http://px4.io/help/errata\n", rev);
				}
				ret = 0;
#endif

			} else {
				errx(1, "unknown command.\n");
			}

		} else {
			usage("Error, input parameter NULL.\n");
		}

	} else {
		usage("Error, not enough parameters.");
	}

	return ret;
}
