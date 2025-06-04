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

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <sys/socket.h>
#include <net/if.h>
#include <netpacket/can.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <sys/socket.h>
#include <nuttx/can.h>
#include <sys/time.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

#include <uORB/topics/raw_canfd.h>

using namespace time_literals;

extern "C" __EXPORT int rs_canfd_socket_main(int argc, char *argv[]);
class RoboSubCANFDSocket : public ModuleBase<RoboSubCANFDSocket>, public ModuleParams
{
public:
	__attribute__((optimize(0))) RoboSubCANFDSocket();

	virtual ~RoboSubCANFDSocket() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase */
	static RoboSubCANFDSocket *instantiate(int argc, char *argv[]);

	/** @see ModuleBase::run() */
	void __attribute__((optimize(0))) run();

	/** @see ModuleBase::print_status() */
	int print_status() override;

	struct sockaddr_can addr;
	struct ifreq ifr;

	uint32_t index = 0;

	struct iovec       _send_iov {};
	struct canfd_frame _send_frame {};
	struct msghdr      _send_msg {};
	struct cmsghdr     *_send_cmsg {};
	struct timeval     *_send_tv {};  /* TX deadline timestamp */
	uint8_t            _send_control[sizeof(struct cmsghdr) + sizeof(struct timeval)] {};

	// Receive msg structure
	struct iovec       _recv_iov {};
	struct canfd_frame _recv_frame {};
	struct msghdr      _recv_msg {};
	struct cmsghdr     *_recv_cmsg {};
	uint8_t            _recv_control[sizeof(struct cmsghdr) + sizeof(struct timeval)] {};

	char ctrlmsg[CMSG_SPACE(sizeof(struct timeval) + 3*sizeof(struct timespec) + sizeof(uint32_t))];
	int nbytes;
private:
	// perf_counter_t	_loop_perf;

	int s = 0;
	const int on = 1;
	const bool can_fd = 1;

	raw_canfd_s _raw_canfd_msg{};

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);

	/**
	 * Setup the can socket
	 * @return False if failed, True if succeded
	 */
	bool setup_can_socket();

	bool send_raw_canfd_msg();

	bool send_init();

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
	)

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	union can_id_u {
		uint32_t can_id;
		struct  {
			uint8_t emergency : 2; // Emergency status
			uint8_t module_id_src : 5;
			uint8_t client_id_src : 5;
			uint8_t module_id_des : 5;
			uint8_t client_id_des : 5;
			uint8_t session : 3;
			uint8_t command_type : 4;
			uint8_t rest : 3;
		} can_id_seg;
	};
	can_id_u received_id;

	uORB::Publication<px4::msg::RawCanfd> raw_canfd_pub{ORB_ID(raw_canfd)};
	uORB::Subscription send_raw_canfd_sub{ORB_ID(send_raw_canfd)};

};
