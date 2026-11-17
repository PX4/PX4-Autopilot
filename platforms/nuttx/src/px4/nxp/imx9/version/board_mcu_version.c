/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
 *   Author: @author Peter van der Perk <peter.vanderperk@nxp.com>
 *                   David Sidrane <david_s5@nscdg.com>
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
 * @file board_mcu_version.c
 * Implementation of the i.MX95 SoC version API
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>

#include <string.h>

#include <imx9_scmi.h>

#define CHIP_TAG     "IMX95"

/* The silicon revision is not available in the OCOTP fuse shadow words, and
 * the ELE mailbox is not reachable from the M7 in the NavQ95 System Manager
 * configuration. The System Manager does report it over SCMI though, so ask
 * it through the misc protocol MISC_SI_INFO message.
 *
 * The name returned by MISC_SI_INFO is already the fully decoded revision
 * string, "i.MX95 B0" style: the System Manager starts from "i.MX95 A0" and
 * patches the base letter and the metal digit in from DIGPROG and the fuses
 * (see DEV_SM_SiInfoGet() in devices/MIMX95/sm/dev_sm_common.c). Use it as
 * the chip name as is, and report the base letter, the first character of the
 * revision field, through *rev, which holds a single character.
 *
 * If the query fails the name stays at the compile time chip tag and the
 * revision reads back as '?', rather than claiming a revision we do not know.
 */

int board_mcu_version(char *rev, const char **revstr, const char **errata)
{
	static char chip[SCMI_MISC_MAX_SINAME] = CHIP_TAG;
	static char revision = '?';
	static bool queried = false;

	if (!queried) {
		scmi_si_info_t info;

		if (imx9_scmi_siinfoget(SM_PLATFORM_A2P, &info) == 0) {
			strncpy(chip, (const char *)info.siname, sizeof(chip) - 1);
			chip[sizeof(chip) - 1] = '\0';

			/* The revision follows the last space, as in "i.MX95 B0" */

			const char *sep = strrchr(chip, ' ');

			if (sep != NULL && sep[1] != '\0') {
				revision = sep[1];
			}
		}

		queried = true;
	}

	*revstr = chip;
	*rev = revision;

	if (errata) {
		*errata = NULL;
	}

	return 0;
}
