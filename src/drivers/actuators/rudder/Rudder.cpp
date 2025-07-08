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

#include <uORB/Subscription.hpp>

#include <net/if.h>
#include <sys/ioctl.h>
#include <string.h>
#include <nuttx/can.h>
#include <netpacket/can.h>

#include "Rudder.hpp"

Rudder::Rudder() :
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": sample"))
{
}

Rudder::~Rudder()
{
	close(_fd);
	perf_free(_sample_perf);
}

int Rudder::init(uint8_t index)
{
	_can_index = index <= 1 ? index : 1;
	return open();
}

void Rudder::run()
{
	while (!should_exit()) {
		perf_begin(_sample_perf);

		static uint64_t msg_payload = 0;

		// send a message
		// FIXME: Adapt for actual content
		struct can_frame tx_frame {
			.can_id = 0x12345678 | CAN_EFF_FLAG,
			.can_dlc = 8
		};
		memcpy(tx_frame.data, &msg_payload, sizeof(msg_payload));

		const int send_res = send(tx_frame, 1_s);

		if (send_res < 0) {
			PX4_ERR("rudder send_res=%i", send_res);

		} else {
			msg_payload++;
		}

		struct can_frame rx_frame;

		uint64_t usec;

		const int recv_res = receive(&rx_frame, &usec);

		if (recv_res < 0) {
			// PX4_ERR("rudder recv_res=%i", recv_res);

		} else {
			PX4_INFO("rudder recv can_id=%#08lx, can_dlc=%u, data=%u, usec=%llu",
				 rx_frame.can_id, rx_frame.can_dlc, rx_frame.data[0], usec);
		}

		perf_end(_sample_perf);
		px4_usleep(50_ms);
	}
}

int Rudder::open()
{
	if (_fd >= 0) { return 0; }

	if ((_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		PX4_ERR("socket");
		return PX4_ERROR;
	}

	const char name[5] {'c', 'a', 'n', (char)((uint8_t)'0' + _can_index), 0};
	struct sockaddr_can addr {};
	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = if_nametoindex(name);

	/* disable default receive filter on this RAW socket */
	setsockopt(_fd, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

	/* RX Timestamping */
	const int on = 1;

	if (setsockopt(_fd, SOL_SOCKET, SO_TIMESTAMP, &on, sizeof(on)) < 0) {
		PX4_ERR("SO_TIMESTAMP is disabled");
		return PX4_ERROR;
	}

	if (setsockopt(_fd, SOL_CAN_RAW, CAN_RAW_TX_DEADLINE, &on, sizeof(on)) < 0) {
		PX4_ERR("CAN_RAW_TX_DEADLINE is disabled");
		return PX4_ERROR;
	}


	if (bind(_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		PX4_ERR("bind");
		return PX4_ERROR;
	}

	// Setup TX msg
	_send_iov.iov_base = &_send_frame;
	_send_iov.iov_len = sizeof(struct can_frame);

	memset(&_send_control, 0x00, sizeof(_send_control));

	_send_msg.msg_iov        = &_send_iov;
	_send_msg.msg_iovlen     = 1;
	_send_msg.msg_control    = &_send_control;
	_send_msg.msg_controllen = sizeof(_send_control);

	_send_cmsg             = CMSG_FIRSTHDR(&_send_msg);
	_send_cmsg->cmsg_level = SOL_CAN_RAW;
	_send_cmsg->cmsg_type  = CAN_RAW_TX_DEADLINE;
	_send_cmsg->cmsg_len   = sizeof(struct timeval);

	_send_tv = (struct timeval *)CMSG_DATA(_send_cmsg);

	// Setup RX msg
	_recv_iov.iov_base = &_recv_frame;
	_recv_iov.iov_len = sizeof(struct can_frame);

	memset(_recv_control, 0x00, sizeof(_recv_control));

	_recv_msg.msg_iov = &_recv_iov;
	_recv_msg.msg_iovlen = 1;
	_recv_msg.msg_control = &_recv_control;
	_recv_msg.msg_controllen = sizeof(_recv_control);

	_recv_cmsg = CMSG_FIRSTHDR(&_recv_msg);

	return PX4_OK;
}

int Rudder::send(struct can_frame &frame, hrt_abstime usec)
{
	auto *const net_frame = (struct can_frame *)&_send_frame;
	net_frame->can_id = frame.can_id;
	net_frame->can_dlc = frame.can_dlc;
	memcpy(&net_frame->data, frame.data, frame.can_dlc);

	/* Set CAN_RAW_TX_DEADLINE timestamp */
	const hrt_abstime deadline = hrt_absolute_time() + usec;
	_send_tv->tv_usec = deadline % 1000000ULL;
	_send_tv->tv_sec = (deadline - _send_tv->tv_usec) / 1000000ULL;

	const int res = sendmsg(_fd, &_send_msg, MSG_DONTWAIT);

	return res > 0 ? 1 : res;
}

int Rudder::receive(struct can_frame *frame, hrt_abstime *usec)
{
	int32_t result = recvmsg(_fd, &_recv_msg, MSG_DONTWAIT);

	if (result < 0) {
		return result;
	}

	/* Copy SocketCAN frame to CanardFrame */
	auto *const recv_frame = (struct can_frame *)&_recv_frame;

	if (recv_frame->can_dlc > CAN_MAX_DLEN) {
		return -EFAULT;
	}

	frame->can_id = recv_frame->can_id;
	frame->can_dlc = recv_frame->can_dlc;
	memcpy(&frame->data, &recv_frame->data, recv_frame->can_dlc);

	/* Read SO_TIMESTAMP value */
	if (_recv_cmsg->cmsg_level == SOL_SOCKET && _recv_cmsg->cmsg_type == SO_TIMESTAMP) {
		struct timeval *tv = (struct timeval *)CMSG_DATA(_recv_cmsg);
		*usec = (hrt_abstime)(tv->tv_sec * 1000000ULL + tv->tv_usec);
	}

	return result;
}

int Rudder::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("rudder",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1200,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

Rudder *Rudder::instantiate(int argc, char *argv[])
{
	Rudder *instance = new Rudder();

	if (instance) {
		// initializing on CAN1=0, since CAN2=1 is used for UAVCAN
		int status = instance->init(1);

		if (status != PX4_OK) {
			delete instance;
			return nullptr;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	return instance;
}

int Rudder::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Rudder::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
SPEED 3: Rudder Control

starring

Chris HELMsworth
David BOWie
Howard STERN
Jeff BRIDGEs
Paul RUDD(er)

with

Natalie PORTman
Tom CRUISE
Laurence FISHburne
Adam SANDler
David SCHWIMMER
Daniel RadCLIFFe
Kiefer SutherLAND

Produced by

Chris MAST
Henry HULL

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rudder", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int rudder_main(int argc, char *argv[])
{
	return Rudder::main(argc, argv);
}
