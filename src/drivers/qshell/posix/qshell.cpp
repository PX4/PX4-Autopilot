/****************************************************************************
 *
 * Copyright (C) 2022 ModalAI, Inc. All rights reserved.
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

#include <px4_platform_common/log.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <stdlib.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/qshell_retval.h>

#include "qshell.h"

#define QSHELL_RESPONSE_WAIT_TIME_US (20 * 1000000) // 20 sec, for temporary ESC calibration

// Static variables
px4::AppState QShell::appState;
uint32_t QShell::_current_sequence{0};
uORB::Subscription *QShell::_qshell_retval_sub{nullptr};


int QShell::main(std::vector<std::string> argList)
{
	if (_qshell_retval_sub == nullptr) {
		_qshell_retval_sub = new uORB::Subscription(ORB_ID(qshell_retval));

		if (_qshell_retval_sub == nullptr) {
			PX4_ERR("Couldn't initialilze Qshell response subscription");
		}
	}

	int ret = _send_cmd(argList);

	if (ret != 0) {
		PX4_ERR("Could not send command");
		return -1;
	}

	ret = _wait_for_retval();

	if (ret != 0) {
		PX4_ERR("Command failed");
		return -1;
	}

	return 0;
}

int QShell::_send_cmd(std::vector<std::string> &argList)
{
	// Let's use a sequence number to check if a return value belongs to the
	// command that we just sent and not a previous one that we assumed that
	// it had timed out.
	++_current_sequence;

	qshell_req_s qshell_req{};
	std::string cmd;

	for (size_t i = 0; i < argList.size(); i++) {
		cmd += argList[i];

		if (i < argList.size() - 1) {
			cmd += " ";
		}
	}

	if (cmd.size() >= qshell_req.MAX_STRLEN) {
		PX4_ERR("Command too long: %d >= %d", (int) cmd.size(), (int) qshell_req.MAX_STRLEN);
		return -1;
	}

	PX4_INFO("Send cmd: '%s'", cmd.c_str());

	qshell_req.strlen = cmd.size();
	strcpy((char *)qshell_req.cmd, cmd.c_str());
	qshell_req.request_sequence = _current_sequence;
	qshell_req.timestamp = hrt_absolute_time();

	_qshell_req_pub.publish(qshell_req);

	return 0;
}

int QShell::_wait_for_retval()
{
	const hrt_abstime time_started_us = hrt_absolute_time();
	qshell_retval_s retval;
	memset(&retval, 0, sizeof(qshell_retval_s));

	while (hrt_elapsed_time(&time_started_us) < QSHELL_RESPONSE_WAIT_TIME_US) {
		if (_qshell_retval_sub->update(&retval)) {
			if (retval.return_sequence != _current_sequence) {
				PX4_WARN("Ignoring return value with wrong sequence");

			} else {
				if (retval.return_value) {
					PX4_INFO("cmd returned with: %d", retval.return_value);
				}

				PX4_INFO("qshell return value timestamp: %lu, local time: %lu", retval.timestamp, hrt_absolute_time());

				return retval.return_value;
			}
		}

		px4_usleep(1000);
	}

	PX4_ERR("command timed out");
	return -1;
}
