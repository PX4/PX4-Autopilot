/****************************************************************************
 *
 *   Copyright (c) 2021 Technology Innovation Institute. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <errno.h>
#include <stddef.h>
#include <debug.h>
#include <string.h>
#include <limits.h>

#include <nuttx/timers/pwm.h>
#include <arch/board/board.h>
#include "mpfs_corepwm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpfs_pwm_setup
 *
 * Description:
 *
 *   Initialize PWM and register PWM devices
 *
 ****************************************************************************/

#define PWM_DEV_NAME "/dev/pwm%d"

int mpfs_pwm_setup(void)
{
	int npwm = 0;
	char devname[sizeof(PWM_DEV_NAME)];        /* Buffer for device name    */
	struct pwm_lowerhalf_s *lower_half = NULL; /* lower-half handle         */
	int config_npwm = 0;                       /* Number of channels in use */

	/* The underlying CorePWM driver "knows" there are up to 16 channels
	 * available for each timer device, so we don't have to do anything
	 * special here.
	 */

#ifdef CONFIG_MPFS_COREPWM0
	config_npwm++;
#endif
#ifdef CONFIG_MPFS_COREPWM1
	config_npwm++;
#endif

	for (npwm = 0; npwm < config_npwm; npwm++) {
		lower_half = mpfs_corepwm_init(npwm);

		/* If we can't get the lower-half handle, skip and keep going. */

		if (lower_half) {
			/* Translate the peripheral number to a device name. */
			snprintf(devname, sizeof(devname), PWM_DEV_NAME, npwm);
			pwm_register(devname, lower_half);
		}
	}

	return 0;
}
