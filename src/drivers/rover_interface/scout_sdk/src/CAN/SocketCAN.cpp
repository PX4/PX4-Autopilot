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

#include "scout_sdk/CAN/SocketCAN.hpp"

#include <net/if.h>
#include <sys/ioctl.h>

#define MODULE_NAME "SCOUT_SDK"

#include <px4_platform_common/log.h>

uint64_t getMonotonicTimestampUSec(void)
{
	struct timespec ts {};

	clock_gettime(CLOCK_MONOTONIC, &ts);

	return ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;
}


namespace scoutsdk
{
int SocketCAN::Init(const char *const can_iface_name, const uint32_t can_bitrate)
{
	struct sockaddr_can addr;
	struct ifreq ifr;

	// Disable CAN FD mode
	bool can_fd = 0;

	_can_fd = can_fd;

	/* Open socket */
	if ((_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		PX4_ERR("Opening socket failed");
		return -1;
	}

	strncpy(ifr.ifr_name, can_iface_name, IFNAMSIZ - 1);
	ifr.ifr_name[IFNAMSIZ - 1] = '\0';
	ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);

	if (!ifr.ifr_ifindex) {
		PX4_ERR("if_nametoindex");
		return -1;
	}

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	const int on = 1;
	/* RX Timestamping */

	if (setsockopt(_fd, SOL_SOCKET, SO_TIMESTAMP, &on, sizeof(on)) < 0) {
		PX4_ERR("SO_TIMESTAMP is disabled");
		return -1;
	}

	/* NuttX Feature: Enable TX deadline when sending CAN frames
	 * When a deadline occurs the driver will remove the CAN frame
	 */

	if (setsockopt(_fd, SOL_CAN_RAW, CAN_RAW_TX_DEADLINE, &on, sizeof(on)) < 0) {
		PX4_ERR("CAN_RAW_TX_DEADLINE is disabled");
		return -1;
	}

	if (can_fd) {
		if (setsockopt(_fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &on, sizeof(on)) < 0) {
			PX4_ERR("no CAN FD support");
			return -1;
		}
	}

	if (bind(_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		PX4_ERR("bind");
		return -1;
	}

	// Setup TX msg
	_send_iov.iov_base = &_send_frame;

	if (_can_fd) { _send_iov.iov_len = sizeof(struct canfd_frame); }

	else { _send_iov.iov_len = sizeof(struct can_frame); }

	memset(&_send_control, 0x00, sizeof(_send_control));

	_send_msg.msg_iov    = &_send_iov;
	_send_msg.msg_iovlen = 1;
	_send_msg.msg_control = &_send_control;
	_send_msg.msg_controllen = sizeof(_send_control);

	_send_cmsg = CMSG_FIRSTHDR(&_send_msg);
	_send_cmsg->cmsg_level = SOL_CAN_RAW;
	_send_cmsg->cmsg_type = CAN_RAW_TX_DEADLINE;
	_send_cmsg->cmsg_len = sizeof(struct timeval);
	_send_tv = (struct timeval *)CMSG_DATA(_send_cmsg);

	// Setup RX msg
	_recv_iov.iov_base = &_recv_frame;

	if (can_fd) { _recv_iov.iov_len = sizeof(struct canfd_frame); }

	else { _recv_iov.iov_len = sizeof(struct can_frame); }

	memset(_recv_control, 0x00, sizeof(_recv_control));

	_recv_msg.msg_iov = &_recv_iov;
	_recv_msg.msg_iovlen = 1;
	_recv_msg.msg_control = &_recv_control;
	_recv_msg.msg_controllen = sizeof(_recv_control);
	_recv_cmsg = CMSG_FIRSTHDR(&_recv_msg);

	// Setup bitrate
	ifr.ifr_ifru.ifru_can_data.arbi_bitrate = can_bitrate / 1000;
	ifr.ifr_ifru.ifru_can_data.arbi_samplep = 88;
	ifr.ifr_ifru.ifru_can_data.data_bitrate = 4 * can_bitrate / 1000;
	ifr.ifr_ifru.ifru_can_data.data_samplep = 75;

	if (ioctl(_fd, SIOCSCANBITRATE, &ifr) < 0) {
		PX4_ERR("Setting CAN bitrate to %d bit/s failed", can_bitrate);
		return -1;
	}

	// Setup RX range filter [ RANGE FILTER NEEDS TO BE SET BEFORE ANY BIT FILTER]
	ifr.ifr_ifru.ifru_can_filter.fid1 = 0x211;	// lower end
	ifr.ifr_ifru.ifru_can_filter.fid2 = 0x231;	// higher end
	ifr.ifr_ifru.ifru_can_filter.ftype = CAN_FILTER_RANGE;
	ifr.ifr_ifru.ifru_can_filter.fprio = CAN_MSGPRIO_LOW;

	if (ioctl(_fd, SIOCACANSTDFILTER, &ifr) < 0) {
		PX4_ERR("Setting RX range filter failed");
		return -1;
	}

	// Setup RX bit filter
	ifr.ifr_ifru.ifru_can_filter.fid1 = 0x41A;	// value
	ifr.ifr_ifru.ifru_can_filter.fid2 = 0x7FF;	// mask
	ifr.ifr_ifru.ifru_can_filter.ftype = CAN_FILTER_MASK;
	ifr.ifr_ifru.ifru_can_filter.fprio = CAN_MSGPRIO_LOW;

	if (ioctl(_fd, SIOCACANSTDFILTER, &ifr) < 0) {
		PX4_ERR("Setting RX bit filter A failed");
		return -1;
	}

	return 0;
}

int16_t SocketCAN::SendFrame(const TxFrame &txf, int timeout_ms)
{
	/* Copy Frame to can_frame/canfd_frame */
	if (_can_fd) {
		_send_frame.can_id = txf.frame.can_id | CAN_EFF_FLAG;
		_send_frame.len = txf.frame.payload_size;
		memcpy(&_send_frame.data, txf.frame.payload, txf.frame.payload_size);

	} else {
		struct can_frame *frame = (struct can_frame *)&_send_frame;
		//frame->can_id = txf.frame.can_id | CAN_EFF_FLAG;
		frame->can_id = txf.frame.can_id;
		frame->can_dlc = txf.frame.payload_size;
		memcpy(&frame->data, txf.frame.payload, txf.frame.payload_size);
		PX4_DEBUG("cansend len %d; can_id: %03x", frame->can_dlc, frame->can_id);
	}

	uint64_t deadline_systick = getMonotonicTimestampUSec() + (txf.tx_deadline_usec - hrt_absolute_time()) +
				    CONFIG_USEC_PER_TICK; // Compensate for precision loss when converting hrt to systick

	/* Set CAN_RAW_TX_DEADLINE timestamp  */
	_send_tv->tv_usec = deadline_systick % 1000000ULL;
	_send_tv->tv_sec = (deadline_systick - _send_tv->tv_usec) / 1000000ULL;

	auto ret =  sendmsg(_fd, &_send_msg, 0);

	return ret;
}

int16_t SocketCAN::ReceiveFrame(RxFrame *rxf)
{
	int32_t result = recvmsg(_fd, &_recv_msg, MSG_DONTWAIT);

	if (result <= 0) { return result; }

	/* Copy CAN frame to Frame */

	if (_can_fd) {
		struct canfd_frame *recv_frame = (struct canfd_frame *)&_recv_frame;
		rxf->frame.can_id = recv_frame->can_id & CAN_EFF_MASK;
		rxf->frame.payload_size = recv_frame->len;

		if (recv_frame->len > 0) {
			memcpy(rxf->frame.payload, &recv_frame->data, recv_frame->len);
		}

	} else {
		struct can_frame *recv_frame = (struct can_frame *)&_recv_frame;
		rxf->frame.can_id = recv_frame->can_id & CAN_SFF_MASK;
		rxf->frame.payload_size = recv_frame->can_dlc;

		if (recv_frame->can_dlc > 0 && recv_frame->can_dlc <= CAN_MAX_DLEN) {
			rxf->frame.payload_size = recv_frame->can_dlc;
			memcpy(rxf->frame.payload, &recv_frame->data, recv_frame->can_dlc);
			PX4_DEBUG("can len %d; can_id: %03x", recv_frame->can_dlc, recv_frame->can_id);
		}
	}

	/* Read SO_TIMESTAMP value */

	if (_recv_cmsg->cmsg_level == SOL_SOCKET && _recv_cmsg->cmsg_type == SO_TIMESTAMP) {
		struct timeval *tv = (struct timeval *)CMSG_DATA(_recv_cmsg);
		rxf->timestamp_usec = tv->tv_sec * 1000000ULL + tv->tv_usec;
	}

	return result;
}
}	// namespace scoutsdk
