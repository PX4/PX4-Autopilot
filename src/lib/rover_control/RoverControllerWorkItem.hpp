/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file RoverControllerWorkItem.hpp
 *
 * Work item for a rover controller that runs on the topic it consumes, only while it is enabled.
 */

#pragma once

#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/SubscriptionCallback.hpp>

/**
 * @brief Base class for a rover controller that runs on its own trigger topic.
 *
 * Only an enabled controller is woken by its trigger topic. All rover controllers and their module run on the same
 * work queue, so enabling, disabling and resetting a controller from the module never overlaps with a controller run.
 *
 * The trigger subscription only wakes the controller, it is never read. Derived classes read their inputs through their
 * own subscriptions. Destroy a controller only from its work queue thread or while it is disabled, because the trigger
 * callback is unregistered after the derived class is already destroyed.
 */
class RoverControllerWorkItem : public px4::WorkItem
{
public:
	/**
	 * @brief Check that the trigger callback can be registered.
	 *
	 * Registering once also creates the trigger topic, so setEnabled() can register it later without failing.
	 *
	 * @return true if the trigger callback could be registered
	 */
	bool init();

	/**
	 * @brief Enable or disable the controller.
	 *
	 * On enabling it also runs once right away, so it does not miss the trigger that was published before it registered.
	 * It only counts as enabled once the trigger callback is registered, otherwise the next call retries.
	 *
	 * @param enabled true to run the controller on its trigger topic
	 * @return true if the operation succeeded
	 */
	bool setEnabled(bool enabled);

	bool enabled() const { return _enabled; }

	/**
	 * @brief Move the trigger callback to the end of the callback list.
	 *
	 * Callbacks sharing a topic run in registration order, so this keeps a lower controller of the cascade behind an
	 * upper controller that was enabled after it. On failure the controller is disabled and the next setEnabled() retries.
	 *
	 * @return true if the trigger callback could be registered again
	 */
	bool reregisterTrigger();

protected:
	/**
	 * @param name Work item name, shown by work_queue status
	 * @param trigger Topic that triggers the controller
	 */
	RoverControllerWorkItem(const char *name, const orb_metadata *trigger);

	/**
	 * @brief Run one controller step, called on every trigger while enabled.
	 */
	virtual void runController() = 0;

	/**
	 * @brief Called when the controller gets enabled, e.g. to restart the time step so it does not span the time spent disabled.
	 */
	virtual void onEnable() {}

private:
	void Run() final;

	uORB::SubscriptionCallbackWorkItem _trigger_sub;
	bool _enabled{false};
};
