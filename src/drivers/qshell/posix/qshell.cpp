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
 * Send shell commands to qurt
 *
 * @author Nicolas de Palezieux <ndepal@gmail.com>
 */

#include "qshell.h"
#include <px4_log.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <stdlib.h>

px4::AppState QShell::appState;

int QShell::main(std::vector<std::string> argList)
{
	appState.setRunning(true);

	std::string cmd;

	for (int i = 0; i < argList.size(); i++) {
		cmd += argList[i];

		if (i < argList.size() - 1) {
			cmd += " ";
		}
	}

	if (cmd.size() > m_qshell_req.MAX_STRLEN) {
		PX4_ERR("The provided command exceeds the maximum length of characters: %d > %d", (int) cmd.size(),
			(int) m_qshell_req.MAX_STRLEN);
		return -1;
	}

	PX4_DEBUG("Requesting %s", cmd.c_str());

	orb_advert_t pub_id_qshell_req = orb_advertise(ORB_ID(qshell_req), & m_qshell_req);

	m_qshell_req.strlen = cmd.size();

	for (int i = 0; i < cmd.size(); i++) {
		m_qshell_req.string[i] = (int) cmd[i];
	}

	if (orb_publish(ORB_ID(qshell_req), pub_id_qshell_req, &m_qshell_req) == PX4_ERROR) {
		PX4_ERR("Error publishing the qshell_req message");
		return -1;
	}

	appState.setRunning(false);
	return 0;
}
