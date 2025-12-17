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

#include "board_config.h"
#include <nuttx/wqueue.h>
#include <px4_platform_common/log.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/system_power.h>
#include <stdint.h>

#define MODULE_NAME "system"
static constexpr uint16_t SYSTEM_CHECK_INTERVAL_MS {500};
static constexpr uint16_t OVERCURRENT_SLEEP_MS{10};

static struct work_s system_work;
uORB::Subscription _system_power_sub{ORB_ID(system_power)};

static void system_check(void *arg)
{
	system_power_s system_power;

	if (_system_power_sub.update(&system_power)) {
		if (system_power.periph_5v_oc || !system_power.can1_gps1_5v_valid) {
			if (system_power.periph_5v_oc) { PX4_ERR("5V Periph overcurrent"); }

			if (!system_power.can1_gps1_5v_valid) { PX4_ERR("5V CAN1 GPS1 overcurrent"); }

			// can1_gps1 and periph share the same efuse enable line
			VDD_5V_PERIPH_EN(false);
			system_usleep(OVERCURRENT_SLEEP_MS * 1000ul);
			VDD_5V_PERIPH_EN(true);
		}

		if (system_power.hipower_5v_oc) {
			PX4_ERR("5V HiPower overcurrent");
			VDD_5V_HIPOWER_EN(false);
			system_usleep(OVERCURRENT_SLEEP_MS * 1000ul);
			VDD_5V_HIPOWER_EN(true);
		}
	}

	work_queue(LPWORK, &system_work, system_check, NULL, USEC2TICK(SYSTEM_CHECK_INTERVAL_MS * 1000ul));
}

int skynode_system_initialize(void)
{
	system_check(nullptr);
	return 0;
}
