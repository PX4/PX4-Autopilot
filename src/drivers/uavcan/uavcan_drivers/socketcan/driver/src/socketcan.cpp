/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 * NuttX SocketCAN port Copyright (C) 2022 NXP Semiconductors
 *
 *
 *
 */

/****************************************************************************
 *
 *   Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 *   NuttX SocketCAN port Copyright (C) 2022 NXP Semiconductors
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

#include <uavcan_nuttx/socketcan.hpp>
#include <uavcan_nuttx/clock.hpp>
#include <uavcan/util/templates.hpp>

#include <net/if.h>
#include <sys/ioctl.h>
#include <string.h>

#include <nuttx/can.h>
#include <netpacket/can.h>

#define MODULE_NAME "UAVCAN_SOCKETCAN"

#include <px4_platform_common/log.h>

namespace uavcan_socketcan
{
namespace
{

struct BitTimingSettings {
	std::uint32_t canclkdiv;
	std::uint32_t canbtr;

	bool isValid() const { return canbtr != 0; }
};

} // namespace

uavcan::uint32_t CanIface::socketInit(uint32_t index)
{

	struct sockaddr_can addr;
	struct ifreq ifr;

	//FIXME Change this when we update to DroneCAN with CAN FD support
	bool can_fd = 0;

	_can_fd = can_fd;

	/* open socket */
	if ((_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		PX4_ERR("socket");
		return -1;
	}

	snprintf(ifr.ifr_name, IFNAMSIZ, "can%li", index);
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

	if (_can_fd) {
		_send_iov.iov_len = sizeof(struct canfd_frame);

	} else {
		_send_iov.iov_len = sizeof(struct can_frame);
	}

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

	if (can_fd) {
		_recv_iov.iov_len = sizeof(struct canfd_frame);

	} else {
		_recv_iov.iov_len = sizeof(struct can_frame);
	}

	memset(_recv_control, 0x00, sizeof(_recv_control));

	_recv_msg.msg_iov = &_recv_iov;
	_recv_msg.msg_iovlen = 1;
	_recv_msg.msg_control = &_recv_control;
	_recv_msg.msg_controllen = sizeof(_recv_control);
	_recv_cmsg = CMSG_FIRSTHDR(&_recv_msg);

	return 0;
}

uavcan::int16_t CanIface::send(const uavcan::CanFrame &frame, uavcan::MonotonicTime tx_deadline,
			       uavcan::CanIOFlags flags)
{
	int res = -1;

	/* Copy CanardFrame to can_frame/canfd_frame */
	if (_can_fd) {
		_send_frame.can_id = frame.id | CAN_EFF_FLAG;
		_send_frame.len = frame.dlc;
		memcpy(&_send_frame.data, frame.data, frame.dlc);

	} else {
		struct can_frame *net_frame = (struct can_frame *)&_send_frame;
		net_frame->can_id = frame.id | CAN_EFF_FLAG;
		net_frame->can_dlc = frame.dlc;
		memcpy(&net_frame->data, frame.data, frame.dlc);
	}

	/* Set CAN_RAW_TX_DEADLINE timestamp  */
	_send_tv->tv_usec = tx_deadline.toUSec() % 1000000ULL;
	_send_tv->tv_sec = (tx_deadline.toUSec() - _send_tv->tv_usec) / 1000000ULL;

	res = sendmsg(_fd, &_send_msg, MSG_DONTWAIT);

	if (res > 0) {
		return 1;

	} else {
		return res;
	}
}

uavcan::int16_t CanIface::receive(uavcan::CanFrame &out_frame, uavcan::MonotonicTime &out_ts_monotonic,
				  uavcan::UtcTime &out_ts_utc, uavcan::CanIOFlags &out_flags)
{
	int32_t result = recvmsg(_fd, &_recv_msg, MSG_DONTWAIT);

	if (result < 0) {
		return result;
	}

	/* Copy SocketCAN frame to CanardFrame */

	if (_can_fd) {
		struct canfd_frame *recv_frame = (struct canfd_frame *)&_recv_frame;
		out_frame.id = recv_frame->can_id;
		out_frame.dlc = recv_frame->len;
		memcpy(out_frame.data, &recv_frame->data, recv_frame->len);

	} else {
		struct can_frame *recv_frame = (struct can_frame *)&_recv_frame;
		out_frame.id = recv_frame->can_id;
		out_frame.dlc = recv_frame->can_dlc;
		memcpy(out_frame.data, &recv_frame->data, recv_frame->can_dlc);
	}

	/* Read SO_TIMESTAMP value */

	if (_recv_cmsg->cmsg_level == SOL_SOCKET && _recv_cmsg->cmsg_type == SO_TIMESTAMP) {
		struct timeval *tv = (struct timeval *)CMSG_DATA(_recv_cmsg);
		out_ts_monotonic = uavcan::MonotonicTime::fromUSec(tv->tv_sec * 1000000ULL + tv->tv_usec);
	}

	return result;
}


uavcan::int16_t CanIface::configureFilters(const uavcan::CanFilterConfig *filter_configs,
		uavcan::uint16_t num_configs)
{
	//FIXME
	return 0;
}

uavcan::uint64_t CanIface::getErrorCount() const
{
	//FIXME query SocketCAN network stack
	return 0;
}

uavcan::uint16_t CanIface::getNumFilters() const
{
	//FIXME
	return 0;
}

int CanIface::getFD()
{
	return _fd;
}

uavcan::uint32_t CanDriver::detectBitRate(void (*idle_callback)())
{
	//FIXME
	return 1;
}

int CanDriver::init(uavcan::uint32_t bitrate)
{
	for (int i = 0; i < UAVCAN_SOCKETCAN_NUM_IFACES; i++) {
		pfds[i].fd     = if_[i].getFD();
		pfds[i].events = POLLIN | POLLOUT;
	}

	/*
	 * TODO add filter configuration ioctl
	 */

	return 0;
}

uavcan::uint32_t CanDriver::getRxQueueOverflowCount() const
{
	//FIXME query SocketCAN network stack
	return 0;
}

bool CanDriver::isInBusOffState() const
{
	//FIXME no interface available yet, maybe make a NuttX ioctl
	return false;
}

uavcan::int16_t CanDriver::select(uavcan::CanSelectMasks &inout_masks,
				  const uavcan::CanFrame * (&)[uavcan::MaxCanIfaces],
				  uavcan::MonotonicTime blocking_deadline)
{
	std::int64_t timeout_usec = (blocking_deadline - clock.getMonotonic()).toUSec();

	if (timeout_usec < 0) {
		timeout_usec = 0;
	}

	inout_masks.read = 0;
	inout_masks.write = 0;

	if (poll(pfds, UAVCAN_SOCKETCAN_NUM_IFACES, timeout_usec / 1000) > 0) {
		for (int i = 0; i < UAVCAN_SOCKETCAN_NUM_IFACES; i++) {
			if (pfds[i].revents & POLLIN) {
				inout_masks.read |= 1U << i;
			}

			if (pfds[i].revents & POLLOUT) {
				inout_masks.write |= 1U << i;
			}
		}
	}

	return 0;           // Return value doesn't matter as long as it is non-negative
}


uavcan::ICanIface *CanDriver::getIface(uavcan::uint8_t iface_index)
{
	if (iface_index > (UAVCAN_SOCKETCAN_NUM_IFACES - 1)) {
		return nullptr;
	}

	return &if_[iface_index];
}

uavcan::uint8_t CanDriver::getNumIfaces() const
{
	return UAVCAN_SOCKETCAN_NUM_IFACES;
}

}
