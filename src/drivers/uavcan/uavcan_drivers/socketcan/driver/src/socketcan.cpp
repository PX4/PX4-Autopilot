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
#include <inttypes.h>
#include <string.h>
#include <errno.h>

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
	_index = index;

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
		if (flags & uavcan::CanIOFlagLoopback) {
			pushLoopback(frame);
		}

		return 1;
	}

	/* The frame never reached a hardware mailbox. NuttX does not buffer it, so
	 * report "not sent" rather than an error and let libuavcan re-queue it --
	 * returning negative here loses the frame, which breaks any multi-frame
	 * transfer in progress. A non-blocking send that the driver could not take
	 * immediately comes back as ETIMEDOUT from net_timedwait(), not ENOBUFS.
	 */
	if (errno == ETIMEDOUT || errno == EAGAIN || errno == EWOULDBLOCK ||
	    errno == ENOBUFS || errno == EINTR || errno == EBUSY) {
		return 0;
	}

	return -1;
}

void CanIface::pushLoopback(const uavcan::CanFrame &frame)
{
	const SystemClock &clock = SystemClock::instance();
	LoopbackItem &item = _loopback[(_loopback_head + _loopback_count) % LoopbackDepth];
	item.frame = frame;
	item.ts_mono = clock.getMonotonic();
	item.ts_utc = clock.utcFromMonotonic(item.ts_mono);

	if (_loopback_count < LoopbackDepth) {
		_loopback_count++;

	} else {
		// Ring full: the oldest echo is overwritten and lost
		_loopback_head = (_loopback_head + 1) % LoopbackDepth;
	}
}

uavcan::int16_t CanIface::receive(uavcan::CanFrame &out_frame, uavcan::MonotonicTime &out_ts_monotonic,
				  uavcan::UtcTime &out_ts_utc, uavcan::CanIOFlags &out_flags)
{
	if (_loopback_count > 0) {
		const LoopbackItem &item = _loopback[_loopback_head];
		out_frame = item.frame;
		out_ts_monotonic = item.ts_mono;
		out_ts_utc = item.ts_utc;
		out_flags = uavcan::CanIOFlagLoopback;
		_loopback_head = (_loopback_head + 1) % LoopbackDepth;
		_loopback_count--;
		return 1;
	}

	out_flags = 0;

	int32_t result = recvmsg(_fd, &_recv_msg, MSG_DONTWAIT);

	if (result < 0) {
		/* Nothing to read is not a driver failure; uc_can_io maps any negative
		 * return onto -ErrDriver.
		 */
		if (errno == EAGAIN || errno == EWOULDBLOCK || errno == ETIMEDOUT) {
			return 0;
		}

		return -1;
	}

	/* Copy SocketCAN frame to CanardFrame */

	if (_can_fd) {
		struct canfd_frame *recv_frame = (struct canfd_frame *)&_recv_frame;
		out_frame.id = recv_frame->can_id;

		if (recv_frame->len > CANFD_MAX_DLEN) {
			return -EFAULT;
		}

		out_frame.dlc = recv_frame->len;
		memcpy(out_frame.data, &recv_frame->data, recv_frame->len);

	} else {
		struct can_frame *recv_frame = (struct can_frame *)&_recv_frame;
		out_frame.id = recv_frame->can_id;

		if (recv_frame->can_dlc > CAN_MAX_DLEN) {
			return -EFAULT;
		}

		out_frame.dlc = recv_frame->can_dlc;
		memcpy(out_frame.data, &recv_frame->data, recv_frame->can_dlc);
	}

	/* Read SO_TIMESTAMP value */

	if (_recv_cmsg->cmsg_level == SOL_SOCKET && _recv_cmsg->cmsg_type == SO_TIMESTAMP) {
		struct timeval *tv = (struct timeval *)CMSG_DATA(_recv_cmsg);
		out_ts_monotonic = uavcan::MonotonicTime::fromUSec(tv->tv_sec * 1000000ULL + tv->tv_usec);

	} else {
		out_ts_monotonic = SystemClock::instance().getMonotonic();
	}

	out_ts_utc = SystemClock::instance().utcFromMonotonic(out_ts_monotonic);

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
	pollErrors();
	return _bus_errors_valid ? _bus_errors.errors : 0;
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

int CanIface::pollErrors() const
{
#ifdef SIOCGCANERRORS

	if (_fd < 0) {
		return -1;
	}

	struct ifreq ifr {};

	snprintf(ifr.ifr_name, IFNAMSIZ, "can%" PRIu32, _index);

	if (ioctl(_fd, SIOCGCANERRORS, &ifr) < 0) {
		return -1;
	}

	const struct can_ioctl_errors_s &e = ifr.ifr_ifru.ifru_can_errors;

	_bus_errors.state = e.state;
	_bus_errors.tec = e.txerr;
	_bus_errors.rec = e.rxerr;
	_bus_errors.errors = e.errors;
	_bus_errors.rx_overruns = e.rx_overruns;
	_bus_errors_valid = true;
	return 0;
#else
	return -1;
#endif
}

int CanIface::setBitRate(uint32_t bitrate)
{
	if (_fd < 0 || bitrate == 0) {
		return -1;
	}

	struct ifreq ifr {};

	snprintf(ifr.ifr_name, IFNAMSIZ, "can%" PRIu32, _index);

	const uint16_t kbps = bitrate / 1000;

	if (ioctl(_fd, SIOCGCANBITRATE, &ifr) < 0) {
		PX4_WARN("can%" PRIu32 ": cannot read bit rate (%d), leaving it as configured", _index, errno);
		return 0;
	}

	if (ifr.ifr_ifru.ifru_can_data.arbi_bitrate == kbps) {
		return 0;
	}

#ifndef SIOCGCANERRORS
	/* Before the PX4/NuttX change that added SIOCGCANERRORS, SIOCSCANBITRATE
	 * restarted a running controller from inside the driver, which on FlexCAN
	 * with ECC RAM initialisation is a bus fault. Keep the configured rate.
	 */
	PX4_WARN("can%" PRIu32 ": UAVCAN_BITRATE %u kbit/s needs a newer NuttX, staying at %u kbit/s",
		 _index, kbps, ifr.ifr_ifru.ifru_can_data.arbi_bitrate);
	return 0;
#else
	/* Only the nominal rate changes; the data phase keeps the driver's
	 * settings so an FD-capable controller is never told data_bitrate 0.
	 * The driver applies the timing at the next ifup, so the interface is
	 * taken down around the request and brought back up whatever happens.
	 */
	ifr.ifr_ifru.ifru_can_data.arbi_bitrate = kbps;

	struct ifreq flags {};

	snprintf(flags.ifr_name, IFNAMSIZ, "can%" PRIu32, _index);

	if (ioctl(_fd, SIOCGIFFLAGS, &flags) < 0) {
		PX4_ERR("can%" PRIu32 ": cannot read interface flags (%d)", _index, errno);
		return -1;
	}

	const bool was_up = flags.ifr_flags & IFF_UP;

	/* NuttX takes IFF_DOWN / IFF_UP as requests, not as a state mask */
	if (was_up) {
		flags.ifr_flags = IFF_DOWN;

		if (ioctl(_fd, SIOCSIFFLAGS, &flags) < 0) {
			PX4_ERR("can%" PRIu32 ": cannot take the interface down (%d)", _index, errno);
			return -1;
		}
	}

	const int res = ioctl(_fd, SIOCSCANBITRATE, &ifr);
	const int set_errno = errno;

	if (was_up) {
		flags.ifr_flags = IFF_UP;

		if (ioctl(_fd, SIOCSIFFLAGS, &flags) < 0) {
			PX4_ERR("can%" PRIu32 ": cannot bring the interface back up (%d)", _index, errno);
			return -1;
		}
	}

	if (res < 0) {
		PX4_ERR("can%" PRIu32 ": %u kbit/s rejected (%d)", _index, kbps, set_errno);
		return -1;
	}

	return 0;
#endif
}

const char *CanIface::busStateName(uint8_t state)
{
	switch (state) {
	case BusErrors::Active:  return "error-active";

	case BusErrors::Warning: return "error-warning";

	case BusErrors::Passive: return "error-passive";

	case BusErrors::BusOff:  return "bus-off";
	}

	return "unknown";
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

		if (if_[i].getFD() >= 0 && if_[i].setBitRate(bitrate) < 0) {
			return -1;
		}
	}

	/*
	 * TODO add filter configuration ioctl
	 */

	return 0;
}

uavcan::uint32_t CanDriver::getRxQueueOverflowCount() const
{
	uint32_t total = 0;

	for (int i = 0; i < UAVCAN_SOCKETCAN_NUM_IFACES; i++) {
		CanIface::BusErrors e;

		if (if_[i].lastErrors(e)) {
			total += e.rx_overruns;
		}
	}

	return total;
}

bool CanDriver::isInBusOffState() const
{
	for (int i = 0; i < UAVCAN_SOCKETCAN_NUM_IFACES; i++) {
		CanIface::BusErrors e;

		if (if_[i].lastErrors(e) && e.state == CanIface::BusErrors::BusOff) {
			return true;
		}
	}

	return false;
}

uavcan::int16_t CanDriver::select(uavcan::CanSelectMasks &inout_masks,
				  const uavcan::CanFrame * (&)[uavcan::MaxCanIfaces],
				  uavcan::MonotonicTime blocking_deadline)
{
	std::int64_t timeout_usec = (blocking_deadline - SystemClock::instance().getMonotonic()).toUSec();

	if (timeout_usec < 0) {
		timeout_usec = 0;
	}

	inout_masks.read = 0;
	inout_masks.write = 0;

	for (int i = 0; i < UAVCAN_SOCKETCAN_NUM_IFACES; i++) {
		if (if_[i].hasLoopbackPending()) {
			inout_masks.read |= 1U << i;
			timeout_usec = 0;
		}
	}

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
