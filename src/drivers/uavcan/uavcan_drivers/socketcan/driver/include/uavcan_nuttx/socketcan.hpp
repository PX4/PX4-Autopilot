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

#pragma once

#include <uavcan_nuttx/thread.hpp>
#include <uavcan_nuttx/clock.hpp>
#include <uavcan/driver/can.hpp>

#include <sys/time.h>
#include <sys/socket.h>

#include <nuttx/can.h>
#include <netpacket/can.h>

namespace uavcan_socketcan
{

class CanIface : public uavcan::ICanIface
	, uavcan::Noncopyable
{
	int               _fd{-1};
	bool              _can_fd{false};

	//// Send msg structure
	struct iovec       _send_iov {};
	struct canfd_frame _send_frame {};
	struct msghdr      _send_msg {};
	struct cmsghdr     *_send_cmsg {};
	struct timeval     *_send_tv {};  /* TX deadline timestamp */
	uint8_t            _send_control[sizeof(struct cmsghdr) + sizeof(struct timeval)] {};

	//// Receive msg structure
	struct iovec       _recv_iov {};
	struct canfd_frame _recv_frame {};
	struct msghdr      _recv_msg {};
	struct cmsghdr     *_recv_cmsg {};
	uint8_t            _recv_control[sizeof(struct cmsghdr) + sizeof(struct timeval)] {};

	SystemClock clock;

public:
	uavcan::uint32_t socketInit(uint32_t index);

	uavcan::int16_t send(const uavcan::CanFrame &frame,
			     uavcan::MonotonicTime tx_deadline,
			     uavcan::CanIOFlags flags) override;

	uavcan::int16_t receive(uavcan::CanFrame &out_frame,
				uavcan::MonotonicTime &out_ts_monotonic,
				uavcan::UtcTime &out_ts_utc,
				uavcan::CanIOFlags &out_flags) override;

	uavcan::int16_t configureFilters(const uavcan::CanFilterConfig *filter_configs,
					 uavcan::uint16_t num_configs) override;

	uavcan::uint64_t getErrorCount() const override;

	uavcan::uint16_t getNumFilters() const override;

	int getFD();
};

/**
 * This class implements CAN driver interface for libuavcan.
 * No configuration needed other than CAN baudrate.
 */
class CanDriver
	: public uavcan::ICanDriver
	, uavcan::Noncopyable
{
	BusEvent update_event_;
	CanIface if_[UAVCAN_SOCKETCAN_NUM_IFACES];
	SystemClock clock;
	struct pollfd pfds[UAVCAN_SOCKETCAN_NUM_IFACES];

public:
	CanDriver() : update_event_(*this)
	{}

	uavcan::int32_t initIface(uint32_t index)
	{
		if (index > (UAVCAN_SOCKETCAN_NUM_IFACES - 1)) {
			return -1;
		}

		return if_[index].socketInit(index);
	}

	/**
	 * Attempts to detect bit rate of the CAN bus.
	 * This function may block for up to X seconds, where X is the number of bit rates to try.
	 * This function is NOT guaranteed to reset the CAN controller upon return.
	 * @return On success: detected bit rate, in bits per second.
	 *         On failure: zero.
	 */
	static uavcan::uint32_t detectBitRate(void (*idle_callback)() = nullptr);

	/**
	 * Returns negative value if the requested baudrate can't be used.
	 * Returns zero if OK.
	 */
	int init(uavcan::uint32_t bitrate);

	/**
	 * Returns the number of times the RX queue was overrun.
	 */
	uavcan::uint32_t getRxQueueOverflowCount() const;

	/**
	 * Whether the controller is currently in bus off state.
	 * Note that the driver recovers the CAN controller from the bus off state automatically!
	 * Therefore, this method serves only monitoring purposes and is not necessary to use.
	 */
	bool isInBusOffState() const;

	uavcan::int16_t select(uavcan::CanSelectMasks &inout_masks,
			       const uavcan::CanFrame * (&)[uavcan::MaxCanIfaces],
			       uavcan::MonotonicTime blocking_deadline) override;

	uavcan::ICanIface *getIface(uavcan::uint8_t iface_index) override;

	uavcan::uint8_t getNumIfaces() const override;

	BusEvent &updateEvent() { return update_event_; }
};


template <unsigned RxQueueCapacity = 128>
class CanInitHelper
{
	//CanRxItem queue_storage_[UAVCAN_KINETIS_NUM_IFACES][RxQueueCapacity];

public:
	enum { BitRateAutoDetect = 0 };

	CanDriver driver;

	CanInitHelper(uint32_t unused = 0x7) :
		driver()
	{
	}

	/**
	 * This overload simply configures the provided bitrate.
	 * Auto bit rate detection will not be performed.
	 * Bitrate value must be positive.
	 * @return  Negative value on error; non-negative on success. Refer to constants Err*.
	 */
	int init(uavcan::uint32_t bitrate)
	{
		for (int i = 0; i < UAVCAN_SOCKETCAN_NUM_IFACES; i++) {
			driver.initIface(i);
		}

		return driver.init(bitrate);
	}

	/**
	 * This function can either initialize the driver at a fixed bit rate, or it can perform
	 * automatic bit rate detection. For theory please refer to the CiA application note #801.
	 *
	  * @param bitrate     Fixed bit rate or zero. Zero invokes the bit rate detection process.
	 *                          If auto detection was used, the function will update the argument
	 *                          with established bit rate. In case of an error the value will be undefined.
	 *
	 * @return                  Negative value on error; non-negative on success. Refer to constants Err*.
	 */

	int init(uavcan::uint32_t &bitrate = 1000000)
	{
		if (bitrate > 0) {
			return driver.init(bitrate);
		}

		return -1;
	}

	/**
	 * Use this value for listening delay during automatic bit rate detection.
	 */
	static uavcan::MonotonicDuration getRecommendedListeningDelay()
	{
		return uavcan::MonotonicDuration::fromMSec(1050);
	}
};

}
