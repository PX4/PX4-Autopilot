/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

/**
 * @file qshell.cpp
 * Listener for shell commands from posix
 *
 * @author Nicolas de Palezieux <ndepal@gmail.com>
 */

#include "qshell.h"

#include <px4_log.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <px4_posix.h>
#include <px4_middleware.h>
#include <dspal_platform.h>

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include "modules/uORB/uORB.h"
#include <drivers/drv_hrt.h>
#include "DriverFramework.hpp"

extern void init_app_map(std::map<std::string, px4_main_t> &apps);

using std::map;
using std::string;

px4::AppState QShell::appState;

QShell::QShell()
{
	init_app_map(apps);
}

int QShell::main()
{
	int rc;
	appState.setRunning(true);
	int sub_qshell_req = orb_subscribe(ORB_ID(qshell_req));

	if (sub_qshell_req == PX4_ERROR) {
		PX4_ERR("Error subscribing to qshell_req topic");
		return -1;
	}

	int i = 0;

	while (!appState.exitRequested()) {
		bool updated = false;

		if (orb_check(sub_qshell_req, &updated) == 0) {
			if (updated) {
				PX4_DEBUG("[%d]qshell_req status is updated... reading new value", i);

				if (orb_copy(ORB_ID(qshell_req), sub_qshell_req, &m_qshell_req) != 0) {
					PX4_ERR("[%d]Error calling orb copy for qshell_req... ", i);
					break;
				}

				char current_char;
				std::string arg;
				std::vector<std::string> appargs;

				for (int str_idx = 0; str_idx < m_qshell_req.strlen; str_idx++) {
					current_char = m_qshell_req.string[str_idx];

					if (isspace(current_char)) { // split at spaces
						if (arg.length()) {
							appargs.push_back(arg);
							arg = "";
						}

					} else {
						arg += current_char;
					}
				}

				appargs.push_back(arg);  // push last argument

				int ret = run_cmd(appargs);

				if (ret) {
					PX4_ERR("Failed to execute command");
				}
			}

		} else {
			PX4_ERR("[%d]Error checking the updated status for qshell_req ", i);
			break;
		}

		// sleep for 1/2 sec.
		usleep(500000);

		++i;
	}

	return 0;
	appState.setRunning(false);
	return rc;
}

int QShell::run_cmd(const std::vector<std::string> &appargs)
{
	// command is appargs[0]
	std::string command = appargs[0];

	//replaces app.find with iterator code to avoid null pointer exception
	for (map<string, px4_main_t>::iterator it = apps.begin(); it != apps.end(); ++it) {
		if (it->first == command) {
			const char *arg[2 + 1];

			unsigned int i = 0;

			while (i < appargs.size() && appargs[i].c_str()[0] != '\0') {
				arg[i] = (char *)appargs[i].c_str();
				PX4_DEBUG("  arg%d = '%s'\n", i, arg[i]);
				++i;
			}

			arg[i] = (char *)0;

			//PX4_DEBUG_PRINTF(i);
			if (apps[command] == NULL) {
				PX4_ERR("Null function !!\n");

			} else {
				return apps[command](i, (char **)arg);
			}

		}
	}

	PX4_ERR("Command %s not found", command.c_str());
	return 1;
}
