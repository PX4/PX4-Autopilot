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
 * @file protocol_splitter.cpp
 * NuttX Driver to split mavlink 2 and another protocol on a serial port.
 * Makes sure the two protocols can be read & written simultanously by 2 processes.
 * It will create two devices:
 * /dev/protocol
 * /dev/mavlink
 */

#include <drivers/device/device.h>
#include <px4_sem.hpp>
#include <px4_log.h>

#include <sys/ioctl.h>
#include <unistd.h>
#include <cstdint>


class ProtocolDev;
class Mavlink2Dev;

extern "C" __EXPORT int protocol_splitter_main(int argc, char *argv[]);

struct StaticData {
	ProtocolDev *protocol;
	Mavlink2Dev *mavlink2;
	sem_t lock;
	char device_name[16];
};

namespace
{
static StaticData *objects = nullptr;
}



class DevCommon : public device::CDev
{
public:
	DevCommon(const char *device_name, const char *device_path);
	virtual ~DevCommon();

	virtual int	ioctl(struct file *filp, int cmd, unsigned long arg);

	virtual int	open(file *filp);
	virtual int	close(file *filp);

protected:

	virtual pollevent_t poll_state(struct file *filp);


	void lock()
	{
		while (sem_wait(&objects->lock) != 0) {
			/* The only case that an error should occur here is if
			 * the wait was awakened by a signal.
			 */
			ASSERT(get_errno() == EINTR);
		}
	}

	void unlock()
	{
		sem_post(&objects->lock);
	}

	int _fd = -1;

	uint16_t _packet_len;
	enum class ParserState : uint8_t {
		Idle = 0,
		GotLength
	};
	ParserState _parser_state = ParserState::Idle;

	bool _had_data = false; ///< whether poll() returned available data

private:
};

DevCommon::DevCommon(const char *device_name, const char *device_path)
	: CDev(device_name, device_path)
{
}

DevCommon::~DevCommon()
{
	if (_fd >= 0) {
		::close(_fd);
	}
}

int DevCommon::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	//pretend we have enough space left to write, so mavlink will not drop data and throw off
	//our parsing state
	if (cmd == FIONSPACE) {
		*(int *)arg = 1024;
		return 0;
	}

	return ::ioctl(_fd, cmd, arg);
}

int DevCommon::open(file *filp)
{
	_fd = ::open(objects->device_name, O_RDWR | O_NOCTTY);
	CDev::open(filp);
	return _fd >= 0 ? 0 : -1;
}

int DevCommon::close(file *filp)
{
	//int ret = ::close(_fd); // FIXME: calling this results in a dead-lock, because DevCommon::close()
	// is called from within another close(), and NuttX seems to hold a semaphore at this point
	_fd = -1;
	CDev::close(filp);
	return 0;
}

pollevent_t DevCommon::poll_state(struct file *filp)
{
	pollfd fds[1];
	fds[0].fd = _fd;
	fds[0].events = POLLIN;

	/* Here we should just check the poll state (which is called before an actual poll waiting).
	 * Instead we poll on the fd with some timeout, and then pretend that there is data.
	 * This will let the calling poll return immediately (there's still no busy loop since
	 * we do actually poll here).
	 * We do this because there is no simple way with the given interface to poll on
	 * the _fd in here or by overriding some other method.
	 */

	int ret = ::poll(fds, sizeof(fds) / sizeof(fds[0]), 100);
	_had_data = ret > 0 && (fds[0].revents & POLLIN);

	return POLLIN;
}


class ProtocolDev : public DevCommon
{
public:
	ProtocolDev();
	virtual ~ProtocolDev() {}

	virtual ssize_t	read(struct file *filp, char *buffer, size_t buflen);
	virtual ssize_t	write(struct file *filp, const char *buffer, size_t buflen);

};

ProtocolDev::ProtocolDev()
	: DevCommon("Protocol", "/dev/protocol")
{
}

ssize_t ProtocolDev::read(struct file *filp, char *buffer, size_t buflen)
{
	PX4_ERR("read from protocol unsupported");
	return 0;
}

ssize_t ProtocolDev::write(struct file *filp, const char *buffer, size_t buflen)
{
	// TODO: something similar as Mavlink2Dev::write()

	return 0;
}


class Mavlink2Dev : public DevCommon
{
public:
	Mavlink2Dev();
	virtual ~Mavlink2Dev() {}

	virtual ssize_t	read(struct file *filp, char *buffer, size_t buflen);
	virtual ssize_t	write(struct file *filp, const char *buffer, size_t buflen);

};

Mavlink2Dev::Mavlink2Dev()
	: DevCommon("Mavlink2", "/dev/mavlink")
{
}

ssize_t Mavlink2Dev::read(struct file *filp, char *buffer, size_t buflen)
{
	if (!_had_data) {
		return 0;
	}

	//no need for locking here
	return ::read(_fd, buffer, buflen);
}

ssize_t Mavlink2Dev::write(struct file *filp, const char *buffer, size_t buflen)
{
	/*
	 * we need to look into the data to make sure the output is locked for the duration
	 * of a whole packet.
	 * assumptions:
	 * - packet header is written all at once (or at least it contains the payload length)
	 * - a single write call does not contain multiple (or parts of multiple) packets
	 */
	ssize_t ret = 0;

	switch (_parser_state) {
	case ParserState::Idle:
		ASSERT(buflen >= 3);

		if ((unsigned char)buffer[0] == 253) {
			uint8_t payload_len = buffer[1];
			uint8_t incompat_flags = buffer[2];
			_packet_len = payload_len + 12;

			if (incompat_flags & 0x1) { //signing
				_packet_len += 13;
			}

			_parser_state = ParserState::GotLength;
			lock();

		} else if ((unsigned char)buffer[0] == 254) { // mavlink 1
			uint8_t payload_len = buffer[1];
			_packet_len = payload_len + 8;

			_parser_state = ParserState::GotLength;
			lock();

		} else {
			PX4_ERR("parser error");
			return 0;
		}

	//no break
	case ParserState::GotLength: {
			_packet_len -= buflen;
			int buf_free;
			::ioctl(_fd, FIONSPACE, (unsigned long)&buf_free);

			if (buf_free < buflen) {
				//let write fail, to let mavlink know the buffer would overflow
				//(this is because in the ioctl we pretend there is always enough space)
				ret = -1;

			} else {
				ret = ::write(_fd, buffer, buflen);
			}

			if (_packet_len == 0) {
				unlock();
				_parser_state = ParserState::Idle;
			}
		}

		break;
	}

	return ret;
}


int protocol_splitter_main(int argc, char *argv[])
{
	if (argc < 2) {
		goto out;
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		if (objects) {
			PX4_ERR("already running");
			return 1;
		}

		if (argc != 3) {
			goto out;
		}

		objects = new StaticData();

		if (!objects) {
			PX4_ERR("alloc failed");
			return -1;
		}

		strncpy(objects->device_name, argv[2], sizeof(objects->device_name));
		sem_init(&objects->lock, 1, 1);
		objects->mavlink2 = new Mavlink2Dev();
		objects->protocol = new ProtocolDev();

		if (!objects->mavlink2 || !objects->protocol) {
			delete objects->protocol;
			delete objects->mavlink2;
			sem_destroy(&objects->lock);
			delete objects;
			objects = nullptr;
			PX4_ERR("alloc failed");
			return -1;

		} else {
			objects->mavlink2->init();
			objects->protocol->init();
		}
	}

	if (!strcmp(argv[1], "stop")) {
		if (objects) {
			delete objects->protocol;
			delete objects->mavlink2;
			sem_destroy(&objects->lock);
			delete objects;
			objects = nullptr;
		}
	}

	/*
	 * Print driver status.
	 */
	if (!strcmp(argv[1], "status")) {
		if (objects) {
			PX4_INFO("running");

		} else {
			PX4_INFO("not running");
		}
	}

	return 0;

out:
	PX4_ERR("unrecognized command, try 'start <device>', 'stop', 'status'");
	return 1;
}

