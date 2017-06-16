/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *   Author: Pavel Kirienko <pavel.kirienko@gmail.com>
 *           David Sidrane <david_s5@nscdg.com>
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

#include <px4_config.h>
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <drivers/device/device.h>
#include <uavcan/protocol/file/BeginFirmwareUpdate.hpp>
#include <uavcan/node/timer.hpp>
/**
 *
 * Defines functionality of UAVCAN ESC node.
 */

#define NUM_ACTUATOR_CONTROL_GROUPS_UAVCAN	1
#define UAVCAN_DEVICE_PATH	"/dev/uavcan/esc"

// we add two to allow for actuator_direct and busevent
#define UAVCAN_NUM_POLL_FDS (NUM_ACTUATOR_CONTROL_GROUPS_UAVCAN+2)

/**
 * A UAVCAN node.
 */
class UavcanEsc : public device::CDev
{
	/*
	 * This memory is reserved for uavcan to use as over flow for message
	 * Coming from multiple sources that my not be considered at development
	 * time.
	 *
	 * The call to getNumFreeBlocks will tell how many blocks there are
	 * free -and multiply it times getBlockSize to get the number of bytes
	 *
	 */
	static constexpr unsigned MemPoolSize        = 2048;

	/*
	 * This memory is reserved for uavcan to use for queuing CAN frames.
	 * At 1Mbit there is approximately one CAN frame every 200 uS.
	 * The number of buffers sets how long you can go without calling
	 * node_spin_xxxx. Since our task is the only one running and the
	 * driver will light the fd when there is a CAN frame we can nun with
	 * a minimum number of buffers to conserver memory. Each buffer is
	 * 32 bytes. So 5 buffers costs 160 bytes and gives us a maximum required
	 * poll rate of ~1 mS
	 *
	 */
	static constexpr unsigned RxQueueLenPerIface = 5;

	/*
	 * This memory is uses for the tasks stack size
	 */

	static constexpr unsigned StackSize          = 4096;

public:
	typedef uavcan::Node<MemPoolSize> Node;
	typedef uavcan_stm32::CanInitHelper<RxQueueLenPerIface> CanInitHelper;
	typedef uavcan::protocol::file::BeginFirmwareUpdate BeginFirmwareUpdate;

	UavcanEsc(uavcan::ICanDriver &can_driver, uavcan::ISystemClock &system_clock);

	virtual		~UavcanEsc();

	virtual int	ioctl(file *filp, int cmd, unsigned long arg);

	static int	start(uavcan::NodeID node_id, uint32_t bitrate);

	Node		&get_node() { return _node; }

	int		teardown();

	void		print_info();

	static UavcanEsc *instance() { return _instance; }


	/* The bit rate that can be passed back to the bootloader */

	int32_t active_bitrate;


private:
	void		fill_node_info();
	int		init(uavcan::NodeID node_id);
	void		node_spin_once();
	int		run();
	int		add_poll_fd(int fd);			///< add a fd to poll list, returning index into _poll_fds[]


	int			_task = -1;			///< handle to the OS task
	bool			_task_should_exit = false;	///< flag to indicate to tear down the CAN driver

	static UavcanEsc	*_instance;			///< singleton pointer
	Node			_node;				///< library instance
	pthread_mutex_t		_node_mutex;

	pollfd			_poll_fds[UAVCAN_NUM_POLL_FDS] = {};
	unsigned		_poll_fds_num = 0;

	typedef uavcan::MethodBinder<UavcanEsc *,
		void (UavcanEsc::*)(const uavcan::ReceivedDataStructure<UavcanEsc::BeginFirmwareUpdate::Request> &,
				    uavcan::ServiceResponseDataStructure<UavcanEsc::BeginFirmwareUpdate::Response> &)>
		BeginFirmwareUpdateCallBack;

	uavcan::ServiceServer<BeginFirmwareUpdate, BeginFirmwareUpdateCallBack> _fw_update_listner;
	void cb_beginfirmware_update(const uavcan::ReceivedDataStructure<UavcanEsc::BeginFirmwareUpdate::Request> &req,
				     uavcan::ServiceResponseDataStructure<UavcanEsc::BeginFirmwareUpdate::Response> &rsp);

public:

	/* A timer used to reboot after the response is sent */

	uavcan::TimerEventForwarder<void (*)(const uavcan::TimerEvent &)> _reset_timer;

};
