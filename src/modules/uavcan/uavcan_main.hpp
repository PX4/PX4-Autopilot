/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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

#include <nuttx/config.h>

#include <uavcan_stm32/uavcan_stm32.hpp>
#include <drivers/device/device.h>
#include <systemlib/perf_counter.h>

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/test_motor.h>
#include <uORB/topics/actuator_direct.h>

#include "actuators/esc.hpp"
#include "sensors/sensor_bridge.hpp"

#if defined(USE_FW_NODE_SERVER)
# include <uavcan/protocol/dynamic_node_id_server/centralized.hpp>
# include <uavcan/protocol/node_info_retriever.hpp>
# include <uavcan_posix/basic_file_server_backend.hpp>
# include <uavcan/protocol/firmware_update_trigger.hpp>
# include <uavcan/protocol/file_server.hpp>
#endif

/**
 * @file uavcan_main.hpp
 *
 * Defines basic functinality of UAVCAN node.
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#define NUM_ACTUATOR_CONTROL_GROUPS_UAVCAN	4
#define UAVCAN_DEVICE_PATH	"/dev/uavcan/esc"
#define UAVCAN_NODE_DB_PATH     "/fs/microsd/uavcan.db"
#define UAVCAN_FIRMWARE_PATH    "/fs/microsd/fw"
#define UAVCAN_LOG_FILE          UAVCAN_NODE_DB_PATH"/trace.log"

// we add two to allow for actuator_direct and busevent
#define UAVCAN_NUM_POLL_FDS (NUM_ACTUATOR_CONTROL_GROUPS_UAVCAN+2)
/**
 * A UAVCAN node.
 */
class UavcanNode : public device::CDev
{
	static constexpr unsigned MaxBitRatePerSec   = 1000000;
	static constexpr unsigned bitPerFrame        = 148;
	static constexpr unsigned FramePerSecond     = MaxBitRatePerSec / bitPerFrame;
	static constexpr unsigned FramePerMSecond    = ((FramePerSecond / 1000) + 1);

	static constexpr unsigned PollTimeoutMs      = 10;

	static constexpr unsigned MemPoolSize        = 10752; ///< Refer to the libuavcan manual to learn why

	/*
	 * This memory is reserved for uavcan to use for queuing CAN frames.
	 * At 1Mbit there is approximately one CAN frame every 145 uS.
	 * The number of buffers sets how long you can go without calling
	 * node_spin_xxxx. Since our task is the only one running and the
	 * driver will light the fd when there is a CAN frame we can nun with
	 * a minimum number of buffers to conserver memory. Each buffer is
	 * 32 bytes. So 5 buffers costs 160 bytes and gives us a poll rate
	 * of ~1 mS
	 *  1000000/200
	 */

	static constexpr unsigned RxQueueLenPerIface = FramePerMSecond * PollTimeoutMs; // At
	static constexpr unsigned StackSize          = 3400;

public:
	typedef uavcan::Node<MemPoolSize> Node;
	typedef uavcan_stm32::CanInitHelper<RxQueueLenPerIface> CanInitHelper;

	UavcanNode(uavcan::ICanDriver &can_driver, uavcan::ISystemClock &system_clock);

	virtual		~UavcanNode();

	virtual int	ioctl(file *filp, int cmd, unsigned long arg);

	static int	start(uavcan::NodeID node_id, uint32_t bitrate);

	Node		&get_node() { return _node; }

	// TODO: move the actuator mixing stuff into the ESC controller class
	static int	control_callback(uintptr_t handle, uint8_t control_group, uint8_t control_index, float &input);

	void		subscribe();

	int		teardown();
	int		arm_actuators(bool arm);

	void		print_info();

	static UavcanNode *instance() { return _instance; }

private:
	void		fill_node_info();
	int		init(uavcan::NodeID node_id);
	void		node_spin_once();
	int		run();
	int		add_poll_fd(int fd);			///< add a fd to poll list, returning index into _poll_fds[]


	int			_task = -1;			///< handle to the OS task
	bool			_task_should_exit = false;	///< flag to indicate to tear down the CAN driver
	int			_armed_sub = -1;		///< uORB subscription of the arming status
	actuator_armed_s	_armed = {};			///< the arming request of the system
	bool			_is_armed = false;		///< the arming status of the actuators on the bus

	int			_test_motor_sub = -1;   ///< uORB subscription of the test_motor status
	test_motor_s		_test_motor = {};
	bool			_test_in_progress = false;

	unsigned		_output_count = 0;		///< number of actuators currently available

	static UavcanNode	*_instance;			///< singleton pointer

	Node			_node;				///< library instance
	pthread_mutex_t		_node_mutex;

	UavcanEscController	_esc_controller;


#if defined(USE_FW_NODE_SERVER)
	static uavcan::dynamic_node_id_server::CentralizedServer *_server_instance;              ///< server singleton pointer

	uavcan_posix::BasicFileSeverBackend _fileserver_backend;
	uavcan::NodeInfoRetriever  _node_info_retriever;
	uavcan::FirmwareUpdateTrigger  _fw_upgrade_trigger;
	uavcan::BasicFileServer        _fw_server;
#endif
	List<IUavcanSensorBridge *> _sensor_bridges;		///< List of active sensor bridges

	MixerGroup		*_mixers = nullptr;

	uint32_t		_groups_required = 0;
	uint32_t		_groups_subscribed = 0;
	int			_control_subs[NUM_ACTUATOR_CONTROL_GROUPS_UAVCAN] = {};
	actuator_controls_s 	_controls[NUM_ACTUATOR_CONTROL_GROUPS_UAVCAN] = {};
	orb_id_t		_control_topics[NUM_ACTUATOR_CONTROL_GROUPS_UAVCAN] = {};
	pollfd			_poll_fds[UAVCAN_NUM_POLL_FDS] = {};
	unsigned		_poll_fds_num = 0;

	int			_actuator_direct_sub = -1;   ///< uORB subscription of the actuator_direct topic
	uint8_t			_actuator_direct_poll_fd_num = 0;
	actuator_direct_s	_actuator_direct = {};

	actuator_outputs_s	_outputs = {};

	// index into _poll_fds for each _control_subs handle
	uint8_t			_poll_ids[NUM_ACTUATOR_CONTROL_GROUPS_UAVCAN];

	perf_counter_t _perfcnt_node_spin_elapsed        = perf_alloc(PC_ELAPSED, "uavcan_node_spin_elapsed");
	perf_counter_t _perfcnt_esc_mixer_output_elapsed = perf_alloc(PC_ELAPSED, "uavcan_esc_mixer_output_elapsed");
	perf_counter_t _perfcnt_esc_mixer_total_elapsed  = perf_alloc(PC_ELAPSED, "uavcan_esc_mixer_total_elapsed");
};
