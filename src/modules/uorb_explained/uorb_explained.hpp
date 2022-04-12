/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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
 *
 * Proof of concept module publishing custom uORB topics
 *
 * This is a material for the 'uORB Explained' espisode of the PX4 Explained Series
 * on the px4.io blog.
 *
 * @author Junwoo Hwang <junwoo091400@gmail.com>
 */

#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>

#include <px4_platform_common/module.h>

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

#include <uORB/topics/pasta_information.h>

using namespace time_literals; // To use time literals like "100_ms"

extern "C" __EXPORT int uorb_explained_main(int argc, char *argv[]);

class uORBExplained : public ModuleBase<uORBExplained>
{
public:
	uORBExplained();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase */
	static uORBExplained *instantiate(int argc, char *argv[]);

	/**
	 * @brief Main run function that runs on a thread
	 * @see ModuleBase
	 */
	void run() override;

private:
	/**
	 * If a certain time has passed from the last order, fill in the pasta_order with the
	 * required pasta specification. Returns true if the order has been filled in.
	 */
	bool customer_order_pasta(hrt_abstime now, pasta_information_s &pasta_order);

	/**
	 * Handles Waiters responsibilities: Checking new order from the customer and relaying it
	 * to the Chef efficiently
	 */
	void waiter_handle_orders(uORB::Subscription &pasta_order_sub, uORB::Publication<pasta_information_s> &pasta_cook_pub);

	/**
	 * Handle Chef's responsibilities: Checking for a new order from the waiter and cooking the pasta!
	 */
	void chef_handle_orders(uORB::Subscription &pasta_cook_sub);

	/**
	 * Publication for the 'pasta_order'
	 *
	 * CUSTOMER -> [pasta_cook] -> WAITER
	 *
	 * Imaginary topic where a 'customer' orders the 'waiter' to order a pasta.
	 */
	uORB::PublicationData<pasta_information_s> _pasta_order_pub{ORB_ID(pasta_order)};

	/**
	 * Subscription for 'pasta_order', which is done by waiter to receive order from the customer
	 */
	uORB::Subscription _pasta_order_sub{ORB_ID(pasta_order)};

	/**
	 * Publication for the 'pasta_cook'
	 *
	 * WAITER -> [pasta_cook] -> CHEF
	 *
	 * Imaginary topic where a 'waiter' orders the 'chef' to cook a certain pasta.
	 */
	uORB::Publication<pasta_information_s> _pasta_cook_pub{ORB_ID(pasta_cook)};

	/**
	 * Subscription for 'pasta_cook', which is done by the chef to receive order from the waiter
	 */
	uORB::Subscription _pasta_cook_sub{ORB_ID(pasta_cook)};
};
