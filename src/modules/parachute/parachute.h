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

#pragma once

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/parachute.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>

extern "C" __EXPORT int parachute_main(int argc, char *argv[]);

/**
 * @brief Parachute Module
 *
 * Releases the parachute output function when the DO_PARACHUTE vehicle command
 * is received, without requiring flight termination. The release is latching:
 * once released, the parachute stays released until reboot.
 *
 * The module runs based on having a subscription callback mechanism to the
 * 'vehicle_command' topic as it only needs to run when we receive the relevant
 * vehicle command.
 */
class Parachute : public ModuleBase, public px4::WorkItem
{
public:
	static Descriptor desc;

	Parachute();

	/** @see ModuleBase **/
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase. Override "status" output when invoked via Commandline, to give detailed status **/
	int print_status() override;

	// Initializes the module
	bool init();

	// Sends a DO_PARACHUTE release command, provoked by custom_command() via CLI interface
	void release();

private:
	/**
	 * @brief Main Run function that runs when subscription callback is triggered
	 */
	void Run() override;

	/**
	 * @brief Releases the parachute output function based on the received DO_PARACHUTE vehicle command
	 */
	void handle_parachute_command(const vehicle_command_s &vehicle_command);

	/**
	 * @brief Send ack response to the DO_PARACHUTE vehicle command
	 */
	bool send_vehicle_command_ack(const uint8_t command_result, const uint8_t target_system,
				      const uint8_t target_component);

	/**
	 * @brief Whether the command is addressed to this autopilot (or broadcast), as opposed to
	 * another component such as an external MAVLink parachute system (MAV_COMP_ID_PARACHUTE)
	 */
	bool command_addresses_autopilot(const vehicle_command_s &vehicle_command);

	bool _released{false};

	// Subscription
	uORB::SubscriptionCallbackWorkItem _vehicle_command_sub{this, ORB_ID(vehicle_command)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	// Publications
	uORB::Publication<parachute_s> _parachute_pub{ORB_ID(parachute)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Publication<vehicle_command_ack_s> _vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};
};
