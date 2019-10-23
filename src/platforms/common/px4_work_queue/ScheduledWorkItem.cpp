/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "ScheduledWorkItem.hpp"

namespace px4
{

ScheduledWorkItem::~ScheduledWorkItem()
{
	ScheduleClear();
}

void ScheduledWorkItem::schedule_trampoline(void *arg)
{
	ScheduledWorkItem *dev = reinterpret_cast<ScheduledWorkItem *>(arg);
	dev->ScheduleNow();
}

void ScheduledWorkItem::ScheduleDelayed(uint32_t delay_us)
{
	hrt_call_after(&_call, delay_us, (hrt_callout)&ScheduledWorkItem::schedule_trampoline, this);
}

void ScheduledWorkItem::ScheduleOnInterval(uint32_t interval_us, uint32_t delay_us)
{
	hrt_call_every(&_call, delay_us, interval_us, (hrt_callout)&ScheduledWorkItem::schedule_trampoline, this);
}

void ScheduledWorkItem::ScheduleClear()
{
	hrt_cancel(&_call);
}

} // namespace px4
