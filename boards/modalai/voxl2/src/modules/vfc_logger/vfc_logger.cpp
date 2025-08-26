/****************************************************************************
 *
 *   Copyright (c) 2025 ModalAI, inc. All rights reserved.
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

#include "mpa.hpp"
#include <string.h>
#include <px4_log.h>
#include <px4_platform_common/tasks.h>

#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/vfc_status.h>

class VFCLogger
{
public:

	static int Initialize();

private:
	static void receiveData(char* data, int bytes);
	static bool _initialized;
	static int _fd;
	static uORB::PublicationMulti<vfc_status_s> _vfc_status_pub;
};

bool VFCLogger::_initialized{false};
int VFCLogger::_fd{-1};
uORB::PublicationMulti<vfc_status_s> VFCLogger::_vfc_status_pub{ORB_ID(vfc_status)};

void VFCLogger::receiveData(char* data, int bytes) {
	// PX4_INFO("VFCLogger::receiveData got %d bytes", bytes);

	vfc_status_s status{0};
	vfc_data_t *dptr = (vfc_data_t*) data;

	status.timestamp = hrt_absolute_time();
	status.backtrack_seconds = dptr->backtrack_seconds;
	status.altitude_ok = dptr->altitude_ok;
	status.flow_ok = dptr->flow_ok;
	status.position_ok = dptr->position_ok;
	status.backtrack_active = dptr->backtrack_active;
	status.desired_submode = dptr->desired_submode;
	status.actual_submode = dptr->actual_submode;

	_vfc_status_pub.publish(status);
}

int VFCLogger::Initialize()
{
	if (_initialized) {
		// Already successfully initialized
		return 0;
	}

	if (MPA::Initialize() == -1) {
		return -1;
	}

	_fd = MPA::PipeConnect("vfc", sizeof(vfc_data_t), receiveData);

	if (_fd == -1) {
		return -1;
	}

	_initialized = true;

	return 0;
}

extern "C" __EXPORT int vfc_logger_main(int argc, char *argv[])
{
	return VFCLogger::Initialize();
}
