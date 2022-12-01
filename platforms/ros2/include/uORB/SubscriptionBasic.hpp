/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file Subscription.hpp
 *
 */

#pragma once

#include "rclcpp/rclcpp.hpp"

#include "rclcpp/subscription_base.hpp"

#include "uORB.h"

namespace uORB
{

// Base subscription wrapper class
class SubscriptionBasic
{
public:

	/**
	 * Constructor
	 *
	 * @param id The uORB ORB_ID enum for the topic.
	 * @param instance The instance for multi sub.
	 */
	Subscription(rclcpp::Node *node, ORB_ID id, uint8_t instance = 0) :
		_node(*node),
		_id(id)
	{
		subscribe();

		(void)instance; // TODO:
	}

	~Subscription()
	{
		unsubscribe();
	}

	bool subscribe()
	{
		if (_subscription == nullptr) {
			//_subscription = _node.create_subscription<px4::msg::VehicleStatus>("vehicle_status", 1, std::bind(&Subscription::topic_callback, this, _1));
		}

		return (_subscription != nullptr);
	}

	void unsubscribe() {}

	bool valid() const { return _subscription != nullptr; }
	bool advertised()
	{
		if (valid()) {
			return true;
		}

		// try to initialize
		if (subscribe()) {
			// check again if valid
			if (valid()) {
				return true;
			}
		}

		return false;
	}

	/**
	 * Check if there is a new update.
	 */
	//bool updated() { return advertised() && _node->updates_available(_last_generation); }
	bool updated() { return true; }

	/**
	 * Update the struct
	 * @param dst The uORB message struct we are updating.
	 */
	bool update(void *dst) { return updated() && copy(dst); }

	/**
	 * Copy the struct
	 * @param dst The uORB message struct we are updating.
	 */
	bool copy(void *dst)
	{
		if (advertised()) {
			//_subscription->return_message(&msg);
			//memcpy(dst, &msg, sizeof(px4::msg::VehicleStatus));
		}

		return false;
	}

	/**
	 * Change subscription instance
	 * @param instance The new multi-Subscription instance
	 */
	bool ChangeInstance(uint8_t instance);

	uint8_t  get_instance() const { return _instance; }
	unsigned get_last_generation() const { return _last_generation; }
	orb_id_t get_topic() const { return get_orb_meta(_id); }

protected:

	rclcpp::SubscriptionBase::SharedPtr _subscription{nullptr};
	rclcpp::Node &_node;

	unsigned _last_generation{0}; /**< last generation the subscriber has seen */

	ORB_ID _id{ORB_ID::INVALID};
	uint8_t _instance{0};

private:

	uint32_t _topic_generation{0};
};

} // namespace uORB
