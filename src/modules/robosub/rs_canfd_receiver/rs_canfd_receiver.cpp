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

#include "rs_canfd_receiver.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/parameter_update.h>
#include <sys/ioctl.h>




int RoboSubCANFDReceiver::print_status()
{
        PX4_INFO("Running");
        // TODO: print additional runtime information about the state of the module
        return 0;
}

int RoboSubCANFDReceiver::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}

int RoboSubCANFDReceiver::task_spawn(int argc, char *argv[])
{
	RoboSubCANFDReceiver *instance = new RoboSubCANFDReceiver();

	if(instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;
		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

bool RoboSubCANFDReceiver::init()
{


	ScheduleOnInterval(1000000_us); // 2000 us interval, 200 Hz rate

        return true;
}

RoboSubCANFDReceiver::RoboSubCANFDReceiver()
        : ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)	/* performance counters */
	// _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_last_sent = hrt_absolute_time();

	// iov.iov_base = &frame;
	// msg.msg_name = &addr;
	// msg.msg_iov = &iov;
	// msg.msg_iovlen = 1;
	// msg.msg_control = &ctrlmsg;

	s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    	if (s < 0) {
        	PX4_ERR("Failed to open CAN socket");
        	return;
    	}

	snprintf(ifr.ifr_name, IFNAMSIZ, "can%li", index);
	ifr.ifr_name[IFNAMSIZ - 1] = '\0';
	ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);

	if (!ifr.ifr_ifindex) {
		PX4_ERR("if_nametoindex");
		return;
	}

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	const int on = 1;

	if (setsockopt(s, SOL_SOCKET, SO_TIMESTAMP, &on, sizeof(on)) < 0) {
		PX4_ERR("SO_TIMESTAMP is disabled");
		return;
	}

	if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_TX_DEADLINE, &on, sizeof(on)) < 0) {
		PX4_ERR("CAN_RAW_TX_DEADLINE is disabled");
		return;
	}

	if (can_fd) {
		if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &on, sizeof(on)) < 0) {
			PX4_ERR("no CAN FD support");
			return;
		}
	}

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		PX4_ERR("bind");
		return;
	}

	_send_iov.iov_base = &_send_frame;

	if (can_fd) {
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

	_recv_msg.msg_name = &addr;
	_recv_msg.msg_namelen = sizeof(addr);

	// strcpy(ifr.ifr_name, "can0");

	// int ret = ioctl(s, SIOCGIFINDEX, &ifr);
	// if(ret == 0) {
	// 	PX4_INFO("test");
	// }

	// // struct sockaddr_can addr;
	// ifr.ifr_flags = IFF_UP;

	// addr.can_family = AF_CAN;
	// addr.can_ifindex = ifr.ifr_ifindex;
	// ret = bind(s, (struct sockaddr *)&addr, sizeof(addr));
	// if(ret == 0) {
	// 	PX4_INFO("test");
	// }
	// // Enable CAN FD frames
	// ret = setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd));
	// if(ret == 0) {
	// 	PX4_INFO("test");
	// }
	// PX4_INFO("Enabled can socker");

}

void RoboSubCANFDReceiver::Run()
{
	PX4_INFO("RoboSubCANFDReceiver::Run()");

	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	// perf_begin(_loop_perf);

	// initialize parameters
	parameters_update();
	// iov.iov_len = sizeof(frame);
	// msg.msg_namelen = sizeof(addr);
	// msg.msg_controllen = sizeof(ctrlmsg);
	// msg.msg_flags = 0;

	if (fcntl(s, F_GETFD) == -1 && errno == EBADF) {
		PX4_ERR("Socket FD is bad");
	}
	int test_fd = fcntl(s, F_GETFD);
	PX4_INFO("test_fd: %i", test_fd);
	nbytes = recvmsg(s, &_recv_msg, MSG_DONTWAIT);
	// nbytes = read(s, &frame, sizeof(struct canfd_frame));
	// ssize_t nbytes = recvmsg(s, &msg, 0);
	PX4_INFO("Received CAN frame: ID=0x%lu, Length=%d", _recv_frame.can_id, _recv_frame.len);
	if (nbytes < 0) {
		PX4_ERR("Failed to read CAN frame");
	}
		// if (nbytes >= (int)sizeof(struct can_frame)) {
		// can_msg.timestamp = hrt_absolute_time();
		// can_msg.can_id = frame.can_id;
		// can_msg.len = frame.len;
		// memcpy(can_msg.data, frame.data, frame.len > 64 ? 64 : frame.len);
		// can_msg.bus_id = 0; // Adjust if you have multiple CAN interfaces

		// raw_can_fd_pub.publish(can_msg);
		// }


}

void RoboSubCANFDReceiver::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}

	// perf_end(_loop_perf);
}

int RoboSubCANFDReceiver::print_usage(const char *reason)
{
        if (reason) {
                PX4_WARN("%s\n", reason);
        }
        PRINT_MODULE_DESCRIPTION(
                R"DESCRSTR(
### Description
Section that describes the provided module functionality.
This is a template for a module running as a task in the background with start/stop/status functionality.
### Implementation
Section describing the high-level implementation of this module.
### Examples
CLI usage example:
$ module start -f -p 42
)DESCRSTR");
        PRINT_MODULE_USAGE_NAME("module", "rs canfd sender");
        PRINT_MODULE_USAGE_COMMAND("start");
        PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
        PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
        PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
        return 0;
}

int rs_canfd_receiver_main(int argc, char *argv[])
{
        return RoboSubCANFDReceiver::main(argc, argv);
}
