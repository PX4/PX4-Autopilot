/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#pragma once

#include <drivers/drv_hrt.h>
#include <stdint.h>
#include <drivers/device/device.h>

struct UpdateIntervalData {
	unsigned  interval{0}; /**< if nonzero minimum interval between updates */

	struct hrt_call update_call {}; /**< deferred wakeup call if update_period is nonzero */

#ifndef __PX4_NUTTX
	uint64_t last_update {0}; /**< time at which the last update was provided, used when update_interval is nonzero */
#endif

};

struct SubscriberData {
	~SubscriberData()
	{
		if (update_interval) {
			hrt_cancel(&update_interval->update_call);
			delete (update_interval);
		}
	}

	unsigned generation{0}; /**< last generation the subscriber has seen */

	UpdateIntervalData *update_interval{nullptr}; /**< if null, no update interval */

	bool _update_reported{false};

	bool update_reported() const { return _update_reported; }
	void set_update_reported(bool update_reported_flag) { _update_reported = update_reported_flag; }

	int set_interval(unsigned interval);
	unsigned get_interval();
};

inline SubscriberData *filp_to_sd(device::file_t *filp)
{
#ifndef __PX4_NUTTX

	if (!filp) {
		return nullptr;
	}

#endif
	return (SubscriberData *)(filp->f_priv);
}
