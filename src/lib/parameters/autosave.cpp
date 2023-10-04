/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "autosave.h"

#include <px4_platform_common/log.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/Subscription.hpp>

#include "param.h"
#include "atomic_transaction.h"

using namespace time_literals;

ParamAutosave::ParamAutosave()
	: ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

void ParamAutosave::request()
{
	if (_scheduled.load() || _disabled) {
		return;
	}

	// wait at least 300ms before saving, because:
	// - tasks often call param_set() for multiple params, so this avoids unnecessary save calls
	// - the logger stores changed params. He gets notified on a param change via uORB and then
	//   looks at all unsaved params.
	hrt_abstime delay = 300_ms;

	static constexpr const hrt_abstime rate_limit = 2_s; // rate-limit saving to 2 seconds
	const hrt_abstime last_save_elapsed = hrt_elapsed_time(&_last_timestamp);

	if (last_save_elapsed < rate_limit && rate_limit > last_save_elapsed + delay) {
		delay = rate_limit - last_save_elapsed;
	}

	_scheduled.store(true);
	ScheduleDelayed(delay);
}

void ParamAutosave::enable(bool enable)
{
	AtomicTransaction transaction;
	_disabled = !enable;

	if (!enable && _scheduled.load()) {
		_scheduled.store(false);
		px4::ScheduledWorkItem::ScheduleClear();
	}
}

void ParamAutosave::Run()
{
	bool disabled = false;

	if (!param_get_default_file()) {
		// In case we save to FLASH, defer param writes until disarmed,
		// as writing to FLASH can stall the entire CPU (in rare cases around 300ms on STM32F7)
		uORB::SubscriptionData<actuator_armed_s> armed_sub{ORB_ID(actuator_armed)};

		if (armed_sub.get().armed) {
			ScheduleDelayed(1_s);
			return;
		}
	}

	{
		const AtomicTransaction transaction;
		_last_timestamp = hrt_absolute_time();
		// Note that the order is important here: we first clear _scheduled, then save the parameters, as during export,
		// more parameter changes could happen.
		_scheduled.store(false);
		disabled = _disabled;
	}

	if (disabled) {
		return;
	}

	PX4_DEBUG("Autosaving params");
	int ret = param_save_default(false);

	if (ret != PX4_OK) {
		// re-request to be saved in the future, try 3 times at most
		if (_retry_count < 3) {
			_retry_count++;
			PX4_INFO("param auto save unavailable (%i), retrying..", ret);
			request();

		} else {
			PX4_ERR("param auto save failed (%i)", ret);
			_retry_count = 0;
		}

	} else {
		_retry_count = 0;
	}
}

