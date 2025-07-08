/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include <inttypes.h>
#include <stdint.h>

#include <drivers/drv_hrt.h>
#include <lib/cdev/CDev.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_config.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/actuator_motors.h>

#include <sys/time.h>
#include <sys/socket.h>

#include <nuttx/can.h>
#include <netpacket/can.h>

using namespace time_literals;

class Rudder : public ModuleBase<Rudder>
{
public:
	Rudder();
	~Rudder() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static Rudder *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	int init(uint8_t index);

private:
	perf_counter_t _sample_perf;
	uint8_t _can_index{0};

	int open();
	int send(struct can_frame &frame, hrt_abstime usec);
	int receive(struct can_frame *frame, hrt_abstime *usec);

	int                _fd{-1};

	//// Send msg structure
	struct iovec       _send_iov {};
	struct can_frame   _send_frame {};
	struct msghdr      _send_msg {};
	struct cmsghdr    *_send_cmsg {};
	struct timeval    *_send_tv {}; /* TX deadline timestamp */
	uint8_t            _send_control[sizeof(struct cmsghdr) + sizeof(struct timeval)] {};

	//// Receive msg structure
	struct iovec       _recv_iov {};
	struct can_frame   _recv_frame {};
	struct msghdr      _recv_msg {};
	struct cmsghdr    *_recv_cmsg {};
	uint8_t            _recv_control[sizeof(struct cmsghdr) + sizeof(struct timeval)] {};
};
