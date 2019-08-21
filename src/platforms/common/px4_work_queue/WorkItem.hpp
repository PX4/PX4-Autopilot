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

#pragma once


#include "WorkQueueManager.hpp"
#include "WorkQueue.hpp"

#include <containers/IntrusiveQueue.hpp>
#include <px4_defines.h>
#include <drivers/drv_hrt.h>

namespace px4
{

class WorkItem : public IntrusiveQueueNode<WorkItem *>
{
public:

	explicit WorkItem(const wq_config_t &config);
	WorkItem() = delete;

	virtual ~WorkItem();

	inline void ScheduleNow() { if (_wq != nullptr) _wq->Add(this); }

	virtual void Run() = 0;

protected:

	/**
	 * Initialize WorkItem given a WorkQueue config. This call
	 * can also be used to switch to a different WorkQueue.
	 * NOTE: Caller is responsible for synchronization.
	 *
	 * @param config The WorkQueue configuration (see WorkQueueManager.hpp).
	 * @return true if initialization was successful
	 */
	bool Init(const wq_config_t &config);
	void Deinit();

private:

	WorkQueue *_wq{nullptr};

};

} // namespace px4
