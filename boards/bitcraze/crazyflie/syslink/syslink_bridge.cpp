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
 * @file syslink_bridge.cpp
 *
 * Character device for talking to the radio as a plain serial port
 */

#include "syslink_main.h"
#include <cstring>



SyslinkBridge::SyslinkBridge(Syslink *link) :
	CDev("/dev/bridge0"),
	_link(link),
	_readbuffer(16, sizeof(crtp_message_t))
{
	_msg_to_send.header = 0;
	_msg_to_send.size = sizeof(_msg_to_send.header);
	_msg_to_send.port = CRTP_PORT_MAVLINK;
	_msg_to_send_size_remaining = CRTP_MAX_DATA_SIZE - 1; // to send 30 bytes of data
	//ideally _msg_to_send.data size should be CRTP_MAX_DATA_SIZE but cfbridge does not receive 31 bytes of data due to a bug somewhere
}

int
SyslinkBridge::init()
{
	int ret = CDev::init();

	/* if init failed, bail now */
	if (ret != OK) {
		PX4_DEBUG("CDev init failed");
		return ret;
	}


	return ret;
}

pollevent_t
SyslinkBridge::poll_state(struct file *filp)
{
	pollevent_t state = 0;

	if (!_readbuffer.empty()) {
		state |= POLLIN;
	}

	if (_link->_writebuffer.space() > 0) {
		state |= POLLOUT;
	}

	return state;
}

ssize_t
SyslinkBridge::read(struct file *filp, char *buffer, size_t buflen)
{
	int nread = 0;
	crtp_message_t msg;

	while (!_readbuffer.empty() && buflen >= sizeof(CRTP_MAX_DATA_SIZE)) {
		_readbuffer.get(&msg, sizeof(msg));
		int size = msg.size - sizeof(msg.header);
		memcpy(buffer, &msg.data, size);

		nread += size;
		buffer += size;
		buflen -= size;
	}

	return nread;
}

ssize_t
SyslinkBridge::write(struct file *filp, const char *buffer, size_t buflen)
{
	int buflen_rem = buflen;

	while (buflen_rem > 0) {

		int datasize = MIN(_msg_to_send_size_remaining, buflen_rem);
		_msg_to_send.size += datasize;
		memcpy(&_msg_to_send.data[CRTP_MAX_DATA_SIZE - 1 - _msg_to_send_size_remaining], buffer, datasize);

		buffer += datasize;
		_msg_to_send_size_remaining -= datasize;
		buflen_rem -= datasize;


		if (_msg_to_send_size_remaining == 0) {

			if (_link->_writebuffer.force(&_msg_to_send, sizeof(crtp_message_t))) {
				PX4_WARN("write buffer overflow");
			}

			_msg_to_send.size = sizeof(_msg_to_send.header);
			_msg_to_send_size_remaining = CRTP_MAX_DATA_SIZE - 1;
		}
	}

	return buflen;
}

int
SyslinkBridge::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	// All termios commands should be silently ignored as they are handled

	switch (cmd) {

	case FIONSPACE:
		*((int *) arg) = _link->_writebuffer.space() * CRTP_MAX_DATA_SIZE;
		return 0;

	default:
		/* give it to the superclass */
		CDev::ioctl(filp, cmd, arg);
		return 0;
	}
}


void
SyslinkBridge::pipe_message(crtp_message_t *msg)
{
	_readbuffer.force(msg, sizeof(msg->size) + msg->size);
	poll_notify(POLLIN);
}
