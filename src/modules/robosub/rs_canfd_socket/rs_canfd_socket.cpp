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

#include "rs_canfd_socket.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/parameter_update.h>
#include <sys/ioctl.h>

int RoboSubCANFDSocket::print_status()
{
        PX4_INFO("Running");
        // TODO: print additional runtime information about the state of the module
        return 0;
}

int RoboSubCANFDSocket::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	if (!strcmp(argv[0], "stop")) {
		get_instance()->exit_and_cleanup();
		return 0;
	}


	if (!strcmp(argv[0], "init")) {
		get_instance()->send_init();
		return 0;
	}

	return print_usage("unknown command");
}

int RoboSubCANFDSocket::task_spawn(int argc, char *argv[])
{
	RoboSubCANFDSocket *instance = new RoboSubCANFDSocket();

	if (!instance) {
		PX4_ERR("alloc failed");
		return -1;
	}

	_object.store(instance);
	_task_id = px4_task_spawn_cmd("rs_canfd_socket",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      2000,
				      (px4_main_t)&run_trampoline,
				      argv);

	if (_task_id < 0) {
		delete instance;
		_object.store(nullptr);
		return -1;
	}

	return 0;
}

RoboSubCANFDSocket *RoboSubCANFDSocket::instantiate(int argc, char *argv[])
{

	RoboSubCANFDSocket *instance = new RoboSubCANFDSocket();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

RoboSubCANFDSocket::RoboSubCANFDSocket()
        : ModuleParams(nullptr)
	/* performance counters */
	// _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
}

bool RoboSubCANFDSocket::setup_can_socket()
{
	PX4_INFO("RoboSubCANFDSocket::setup_can_socket()");

	s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    	if (s < 0) {
        	PX4_ERR("Failed to open CAN socket");
        	return 0;
    	}

	snprintf(ifr.ifr_name, IFNAMSIZ, "can%li", index);
	ifr.ifr_name[IFNAMSIZ - 1] = '\0';
	ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);

	if (!ifr.ifr_ifindex) {
		PX4_ERR("if_nametoindex");
		return 0;
	}

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (setsockopt(s, SOL_SOCKET, SO_TIMESTAMP, &on, sizeof(on)) < 0) {
		PX4_ERR("SO_TIMESTAMP is disabled");
		return 0;
	}

	if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_TX_DEADLINE, &on, sizeof(on)) < 0) {
		PX4_ERR("CAN_RAW_TX_DEADLINE is disabled");
		return 0;
	}

	if (can_fd) {
		if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &on, sizeof(on)) < 0) {
			PX4_ERR("no CAN FD support");
			return 0;
		}
	}

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		PX4_ERR("bind");
		return 0;
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

	return 1;
}

bool RoboSubCANFDSocket::send_raw_canfd_msg()
{
	// Check if the message is valid
	if (_raw_canfd_msg.len > sizeof(_send_frame.data)) {
		PX4_ERR("Invalid CAN frame length: %d", _raw_canfd_msg.len);
		return false;
	}

	_send_frame.can_id = _raw_canfd_msg.id; // set the can id
	_send_frame.len = _raw_canfd_msg.len; // set the length of the can frame
	memcpy(_send_frame.data, _raw_canfd_msg.data, sizeof(_raw_canfd_msg.data)); // copy the data from the message struct to the can frame

	gettimeofday(_send_tv, NULL);
	_send_tv->tv_usec += 500000; // add 500ms to the timestamp
	if (_send_tv->tv_usec >= 1000000) {
		_send_tv->tv_sec++;
		_send_tv->tv_usec -= 1000000;
	}

	if (sendmsg(s, &_send_msg, 0) < 0) {
		PX4_ERR("Failed to send CAN frame: %d", errno);
		return false;
	}

	return true;
}

bool RoboSubCANFDSocket::send_init() {
	_raw_canfd_msg.id = (0x8001083 | CAN_EFF_FLAG);
	_raw_canfd_msg.data[0] = 0x01;
	_raw_canfd_msg.len = 1;

	if(!send_raw_canfd_msg()) {
		PX4_ERR("Failed to send initial CAN frame");
		return false;
	}
	_raw_canfd_msg.data[0] = 0x02;
	if(!send_raw_canfd_msg()) {
		PX4_ERR("Failed to send second CAN frame");
		return false;
	}
	return true;
}

void RoboSubCANFDSocket::run()
{
	PX4_INFO("RoboSubCANFDSocket::Run()");
	parameters_update();

	if (!setup_can_socket()) {
		PX4_ERR("Failed to init CAN socket");
		return;
	}

	while (!should_exit()) {
		nbytes = recvmsg(s, &_recv_msg, MSG_DONTWAIT);

		if (nbytes < 0) {
			if (errno != EAGAIN && errno != EWOULDBLOCK) {
				PX4_ERR("recvmsg failed: %d", errno);
			}
			if (send_raw_canfd_sub.update(&_raw_canfd_msg)) {
				if(!send_raw_canfd_msg()) {
					PX4_ERR("Failed to send raw CAN FD message");
					continue;
				}
			}
			usleep(10000); // avoid spinning
			continue;
		}
		if (nbytes < (int)sizeof(struct canfd_frame)) { // check if theres enough bytes received to be a valid canfd message
			PX4_ERR("Received short CAN frame: %d bytes", nbytes);
			continue;
		}
		if (_recv_frame.can_id & CAN_ERR_FLAG) { // check for error flags
			PX4_ERR("Received error frame: ID=0x%lx, Length=%d", _recv_frame.can_id, _recv_frame.len);
			continue;
		}


		raw_canfd_s raw_canfd_msg{}; // create the temp message struct
		raw_canfd_msg.timestamp = hrt_absolute_time(); // set the timestamp
		raw_canfd_msg.id = _recv_frame.can_id; // set the can id
		raw_canfd_msg.len = _recv_frame.len; // set the length of the can frame
		memcpy(raw_canfd_msg.data, _recv_frame.data, sizeof(_recv_frame.data)); // copy the data from the can frame to the message struct
		raw_canfd_pub.publish(raw_canfd_msg); // publish the raw canfd message

		received_id.can_id = _recv_frame.can_id; // Put the received can id in the union to parse the id.

		if (received_id.can_id_seg.module_id_src == 0x02) { // Check if the src module is 0x02 (mainbrain)
			if (received_id.can_id_seg.client_id_src == 0x04) { // check if the src client id is 0x04 (LP4 GPIO non contact water level)
				water_presence_s water_presence_msg{}; // create the temp message struct
				water_presence_msg.timestamp = hrt_absolute_time(); // set the timestamp
				water_presence_msg.water_detected = _recv_frame.data[0]; // setthe water detected bit, this should be the first byte

				water_presence_pub.publish(water_presence_msg); // publish the data
			}
		}
	}
	// cleanup
	close(s);
	PX4_INFO("RoboSubCANFDSocket thread exiting");
	return;
}

void RoboSubCANFDSocket::parameters_update(bool force)
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

int RoboSubCANFDSocket::print_usage(const char *reason)
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

int rs_canfd_socket_main(int argc, char *argv[])
{
        return RoboSubCANFDSocket::main(argc, argv);
}
