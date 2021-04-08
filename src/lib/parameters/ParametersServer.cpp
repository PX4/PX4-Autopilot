/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "ParametersServer.hpp"

#include <px4_platform_common/atomic.h>

#include "param.h"

static ParametersServer *_param_autosave{nullptr};

float ParametersServer::last_autosave_elapsed() const
{
	return hrt_elapsed_time_atomic(&_last_autosave_timestamp) * 1e-6f;
}

void ParametersServer::AutoSave()
{
	if (_param_autosave == nullptr) {
		return;
	}

	// wait at least 300ms before saving, because:
	// - tasks often call param_set() for multiple params, so this avoids unnecessary save calls
	// - the logger stores changed params. He gets notified on a param change via uORB and then
	//   looks at all unsaved params.
	float delay = 0.3f;

	static constexpr float rate_limit = 2.0f; // rate-limit saving to 2 seconds
	const float last_save_elapsed = _param_autosave->last_autosave_elapsed();

	if (last_save_elapsed < rate_limit && rate_limit > last_save_elapsed + delay) {
		delay = rate_limit - last_save_elapsed;
	}

	uint64_t delay_us = delay * 1e6f;
	_param_autosave->ScheduleDelayed(delay_us);
}

bool ParametersServer::EnableAutoSave(bool enable)
{
	if (enable) {
		if (_param_autosave == nullptr) {
			_param_autosave = new ParametersServer();

			if (_param_autosave == nullptr) {
				PX4_ERR("ParametersServer alloc failed");
				return false;
			}
		}

	} else {
		// TODO: how to prevent delete if currently running?
		delete _param_autosave;
		_param_autosave = nullptr;
	}

	return true;
}

void ParametersServer::print_status()
{
	if (_param_autosave) {
		PX4_INFO("last auto save: %.3f seconds ago", (double)_param_autosave->last_autosave_elapsed());

	} else {
		PX4_INFO("auto save: off");
	}
}

void ParametersServer::Run()
{
	_last_autosave_timestamp = hrt_absolute_time();
	int ret = param_save_default();

	if (ret != 0) {
		PX4_ERR("param auto save failed (%i)", ret);
	}
}
