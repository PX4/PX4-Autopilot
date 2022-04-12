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
 * This module is an example module as part of the PX4 uORB Explained Series Part 4.
 *
 * It simulates publishing and subscribing the pasta_order and pasta_cook uORB topics!
 */

#include "uorb_explained.hpp"

void uORBExplained::run()
{
	while (!should_exit()) {
		hrt_abstime now = hrt_absolute_time();

		// If a customer ordered a new pasta, publish (update) the new order
		if (customer_order_pasta(now, _pasta_order_pub.get())) {
			_pasta_order_pub.update();
		}

		// Handle Waiter relaying the order from the customer to the chef
		waiter_handle_orders(_pasta_order_sub, _pasta_cook_pub);

		// Handle Chef
		chef_handle_orders(_pasta_cook_sub);

		// Run the loop at 10 Hz, to not overload the processing power
		px4_usleep(100_ms);
	}
}

bool uORBExplained::customer_order_pasta(hrt_abstime now, pasta_information_s &pasta_order)
{
	// [us] Period of customers ordering the pasta
	static constexpr hrt_abstime PASTA_ORDER_TIME_PERIOD = 1_s;

	static uint8_t CUSTOMER_PASTA_TYPE_PREFERENCE = pasta_information_s::PASTA_TYPE_RIGATONI;
	static uint8_t CUSTOMER_PASTA_COOKED_TEXTURE_PREFERENCE = pasta_information_s::PASTA_COOKED_AL_DENTE;

	// [deg C] Prefered pasta temperature of the customer
	static constexpr float CUSTOMER_PASTA_TEMPERATURE_PREFERENCE = 36.5;
	// [us] Period of the sine curve that will determine the pasta temperature
	static hrt_abstime PASTA_TEMPERATURE_CURVE_PERIOD = 16_s;
	// [deg C] Magnitude that will be multiplied to the sine value to calculate temperature
	static constexpr float PASTA_TEMPERATURE_CURVE_MAGNITUDE = 10.0f;

	static hrt_abstime last_pasta_order_time{0};
	static uint8_t lasta_pasta_ordered_menu{pasta_information_s::PASTA_MENU_UNDEFINED};

	if ((now - last_pasta_order_time) > PASTA_ORDER_TIME_PERIOD) {
		pasta_order.timestamp = now;

		// Select MENU: Cycle through the next pasta menu
		switch (lasta_pasta_ordered_menu) {
		case pasta_information_s::PASTA_MENU_AGLIO_E_OLIO:
			pasta_order.menu_name = pasta_information_s::PASTA_MENU_AMATRICIANA;
			break;

		case pasta_information_s::PASTA_MENU_AMATRICIANA:
			pasta_order.menu_name = pasta_information_s::PASTA_MENU_CARBONARA;
			break;

		case pasta_information_s::PASTA_MENU_CARBONARA:
			pasta_order.menu_name = pasta_information_s::PASTA_MENU_BOLOGNESE;
			break;

		case pasta_information_s::PASTA_MENU_BOLOGNESE:
		default:
			// FALLTHROUGH: If the pasta ordered last time is invalid, order
			// Aglio e Olio as a default behavior on this loop
			pasta_order.menu_name = pasta_information_s::PASTA_MENU_AGLIO_E_OLIO;
		}

		// Set COOKED TEXTURE
		pasta_order.cooked_texture = CUSTOMER_PASTA_COOKED_TEXTURE_PREFERENCE;
		// Set TYPE of pasta
		pasta_order.pasta_type = CUSTOMER_PASTA_TYPE_PREFERENCE;

		// Set TEMPERATURE
		// It will be a sine curve with a preference value centered, manitude value varying,
		// period- repeating temperature
		const float current_phase_rad = ((float)now / PASTA_TEMPERATURE_CURVE_PERIOD) * M_PI_F * 2.0f;
		const float temperature_adjustment = PASTA_TEMPERATURE_CURVE_MAGNITUDE * sinf(current_phase_rad);
		pasta_order.pasta_temperature = CUSTOMER_PASTA_TEMPERATURE_PREFERENCE + temperature_adjustment;

		// Update the states
		last_pasta_order_time = now;
		lasta_pasta_ordered_menu = pasta_order.menu_name;

		return true;
	}

	// No order is created
	return false;
}

void uORBExplained::waiter_handle_orders(uORB::Subscription &pasta_order_sub,
		uORB::Publication<pasta_information_s> &pasta_cook_pub)
{
	static pasta_information_s customer_order{};
	// Internal table ID waiter tracks to know to which customer (table) each order belongs to
	static uint16_t customer_table_id{0};

	// New order from the customer is received
	if (pasta_order_sub.update(&customer_order)) {
		// Assign table ID to the customer
		customer_order.customer_table_id = ++customer_table_id;

		// If the customer isn't sure about the menu, recommend Carbonara!
		if (customer_order.menu_name == pasta_information_s::PASTA_MENU_UNDEFINED) {
			customer_order.menu_name = pasta_information_s::PASTA_MENU_CARBONARA;
		}

		// If customer isn't sure about the cooked texture, recommend the Al Dente!
		if (customer_order.cooked_texture == pasta_information_s::PASTA_COOKED_UNDEFINED) {
			customer_order.cooked_texture = pasta_information_s::PASTA_COOKED_AL_DENTE;
		}

		// If customer isn't sure about the pasta type, recommend Rigatoni!
		if (customer_order.pasta_type == pasta_information_s::PASTA_TYPE_UNDEFINED) {
			customer_order.pasta_type = pasta_information_s::PASTA_TYPE_RIGATONI;
		}

		// Publish so that the chef can receive the order!
		pasta_cook_pub.publish(customer_order);
	}
}

void uORBExplained::chef_handle_orders(uORB::Subscription &pasta_cook_sub)
{
	static pasta_information_s waiter_order{};

	// New order from the waiter is received
	if (pasta_cook_sub.update(&waiter_order)) {
		PX4_INFO("Chef Cooking new pasta menu %d with temperature %f[deg C]!", waiter_order.menu_name,
			 (double)waiter_order.pasta_temperature);
	}
}

/**
 * "uorb_explained" module start / stop handling function
 */
int uorb_explained_main(int argc, char *argv[])
{
	return uORBExplained::main(argc, argv);
}

uORBExplained::uORBExplained()
{
	// Do nothing in the constructor
}


int uORBExplained::task_spawn(int argc, char *argv[])
{
	px4_main_t entry_point = (px4_main_t)&run_trampoline;

	_task_id = px4_task_spawn_cmd("uorb_explained",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1500,
				      entry_point,
				      (char *const *)argv);

	if (_task_id < 0) {
		PX4_INFO("uORB Explained module instantiation Failed!");
		_task_id = -1;
		return -errno;

	} else {
		return PX4_OK;

	}
}

uORBExplained *uORBExplained::instantiate(int argc, char *argv[])
{
	return new uORBExplained();
}

int uORBExplained::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int uORBExplained::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Publishes and subscribes to pasta_cook and pasta_order uORB messages

### Examples
CLI usage example:
$ uorb_explained start

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("uuv_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start")
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}
