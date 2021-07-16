/****************************************************************************
 *
 *   Copyright (c) 2016-2021 PX4 Development Team. All rights reserved.
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
 *
 * NuttX Driver to multiplex MAVLink and RTPS on a single serial port.
 * Makes sure the two protocols can be read & written simultaneously by two
 * processes.
 *
 * It will create two devices:
 *    /dev/mavlink
 *    /dev/rtps
 */

#include <lib/cdev/CDev.hpp>
#include <px4_platform_common/sem.hpp>
#include <px4_platform_common/log.h>
#include <lib/perf/perf_counter.h>
#include <mathlib/mathlib.h>

#include <sys/ioctl.h>
#include <assert.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <cstdint>
#include <cstring>


static constexpr uint64_t reader_timeout_us = 1000000;

class Mavlink2Dev;
class RtpsDev;
class ReadBuffer;

extern "C" __EXPORT int protocol_splitter_main(int argc, char *argv[]);

/*
 MessageType is in MSB of header[1]
                |
                v
        Mavlink 0000 0000b
        Rtps    1000 0000b
*/
enum class MessageType : uint8_t {Mavlink = 0x00, Rtps = 0x01};

constexpr char  Sp2HeaderMagic = 'S';
constexpr int   Sp2HeaderSize  = 4;

/*
 Header Structure:

      bits:   1 2 3 4 5 6 7 8
 header[0] - |     Magic     |
 header[1] - |T|   LenH      |
 header[2] - |     LenL      |
 header[3] - |   Checksum    |
*/
union __attribute__((packed)) Sp2Header {
	uint8_t bytes[4];
	struct {
		char magic;                // 'S'
		uint8_t len_h:	7,         // Length MSB
			 type:	1;         // 0=MAVLINK, 1=RTPS
		uint8_t len_l;             // Length LSB
		uint8_t checksum;          // XOR of the three bytes above
	} fields;
};

struct StaticData {
	Mavlink2Dev *mavlink2;
	RtpsDev *rtps;
	sem_t r_lock;
	sem_t w_lock;
	char device_name[16];
	ReadBuffer *read_buffer;
};

namespace
{
static StaticData *objects = nullptr;
}

// Perf counters
perf_counter_t bytes_received_count;
perf_counter_t header_bytes_received_count;
perf_counter_t bytes_lost_count;
perf_counter_t mavlink_messages_parsed_count;
perf_counter_t mavlink_bytes_parsed_count;
perf_counter_t rtps_messages_parsed_count;
perf_counter_t rtps_bytes_parsed_count;

class ReadBuffer
{
public:
	int read(int fd);
	void copy(void *dest, size_t pos, size_t n);
	void remove(size_t pos, size_t n);

	void print_stats();
	void update_lost_stats();

	uint8_t buffer[1024] = {};
	size_t buf_size = 0;

	// We keep track of the first Mavlink and Rtps packet in the buffer.
	// If start and end are equal there is no packet.
	size_t start_mavlink = 0;
	size_t end_mavlink = 0;
	size_t start_rtps = 0;
	size_t end_rtps = 0;
	// Just for stats.
	size_t mavlink_parsed = 0;
	size_t rtps_parsed = 0;
	size_t bytes_received = 0;
	size_t bytes_lost = 0;
	size_t header_bytes_received = 0;

	// To keep track of readers.
	hrt_abstime last_mavlink_read = 0;
	hrt_abstime last_rtps_read = 0;
};

int ReadBuffer::read(int fd)
{
	if (sizeof(buffer) == buf_size) {
		// This happens if one consumer does not read the data, or not fast enough.
		// TODO: add a mechanism to thrown away data if a user is no longer reading.
		PX4_DEBUG("Buffer full: %zu %zu %zu %zu", start_mavlink, end_mavlink, start_rtps, end_rtps);
	}

	int r = ::read(fd, buffer + buf_size, sizeof(buffer) - buf_size);

	if (r < 0) {
		return r;
	}

	buf_size += r;
	bytes_received += r;

	// Update the lost/unused bytes count
	update_lost_stats();

	perf_set_count(bytes_received_count, bytes_received);

	return r;
}

void ReadBuffer::copy(void *dest, size_t pos, size_t n)
{
	ASSERT(pos < buf_size);
	ASSERT(pos + n <= buf_size);

	if (dest) {
		memcpy(dest, buffer + pos, n);
	}
}

void ReadBuffer::remove(size_t pos, size_t n)
{
	ASSERT(pos < buf_size);
	ASSERT(pos + n <= buf_size);

	memmove(buffer + pos, buffer + (pos + n), buf_size - pos - n);
	buf_size -= n;
}

void ReadBuffer::update_lost_stats()
{
	bytes_lost = bytes_received - mavlink_parsed - rtps_parsed - header_bytes_received;

	if (end_mavlink > start_mavlink) {
		bytes_lost -= end_mavlink - start_mavlink;
	}

	if (end_rtps > start_rtps) {
		bytes_lost -= end_rtps - start_rtps;
	}

	perf_set_count(bytes_lost_count, bytes_lost);
}

void ReadBuffer::print_stats()
{
	PX4_INFO_RAW("\tReceived:\n");
	PX4_INFO_RAW("\tTotal:   %9zu bytes\n",
		     bytes_received);
	PX4_INFO_RAW("\tHeaders: %9zu bytes (%5.1f %%)\n",
		     header_bytes_received,
		     static_cast<double>(static_cast<float>(header_bytes_received)
					 / static_cast<float>(bytes_received)
					 * 100.f));
	PX4_INFO_RAW("\tMAVLink: %9zu bytes (%5.1f %%)\n",
		     mavlink_parsed,
		     static_cast<double>(static_cast<float>(mavlink_parsed)
					 / static_cast<float>(bytes_received - header_bytes_received)
					 * 100.f));
	PX4_INFO_RAW("\tRTPS:    %9zu bytes (%5.1f %%)\n",
		     rtps_parsed,
		     static_cast<double>(static_cast<float>(rtps_parsed)
					 / static_cast<float>(bytes_received - header_bytes_received)
					 * 100.f));

	PX4_INFO_RAW("\tUnused:  %9zu bytes (%5.1f %%)\n", bytes_lost,
		     static_cast<double>(static_cast<float>(bytes_lost)
					 / static_cast<float>(bytes_received)
					 * 100.f));
}


class DevCommon : public cdev::CDev
{
public:
	DevCommon(const char *device_path, ReadBuffer *read_buffer);
	virtual ~DevCommon();

	virtual int	ioctl(struct file *filp, int cmd, unsigned long arg);

	virtual int	open(file *filp);
	virtual int	close(file *filp);

	enum Operation {Read, Write};

protected:

	virtual pollevent_t poll_state(struct file *filp);

	int try_to_copy_data(char *buffer, size_t buflen, MessageType message_type);
	void scan_for_packets();
	void check_for_timeouts();
	void cleanup();

	void lock(enum Operation op)
	{
		sem_t *this_lock = op == Read ? &objects->r_lock : &objects->w_lock;

		while (sem_wait(this_lock) != 0) {
			/* The only case that an error should occur here is if
			 * the wait was awakened by a signal.
			 */
			ASSERT(get_errno() == EINTR);
		}
	}

	void unlock(enum Operation op)
	{
		sem_t *this_lock = op == Read ? &objects->r_lock : &objects->w_lock;
		sem_post(this_lock);
	}

	int _fd = -1;

	Sp2Header _header;

	uint16_t _packet_len;
	enum class ParserState : uint8_t {
		Idle = 0,
		GotLength
	};
	ParserState _parser_state = ParserState::Idle;

	ReadBuffer *_read_buffer;
};

DevCommon::DevCommon(const char *device_path, ReadBuffer *read_buffer)
	: CDev(device_path)
	, _read_buffer(read_buffer)
{
}

DevCommon::~DevCommon()
{
	if (_fd >= 0) {
		// discard all pending data, as close() might block otherwise on NuttX with flow control enabled
		tcflush(_fd, TCIOFLUSH);
		::close(_fd);
	}
}

int DevCommon::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	// pretend we have enough space left to write, so mavlink will not drop data and throw off
	// our parsing state
	if (cmd == FIONSPACE) {
		*(int *)arg = 1024;
		return 0;
	}

	return ::ioctl(_fd, cmd, arg);
}

int DevCommon::open(file *filp)
{
	_fd = ::open(objects->device_name, O_RDWR | O_NOCTTY);

	if (_fd < 0) {
		PX4_ERR("open failed: %s", strerror(errno));
		return -1;
	}

	CDev::open(filp);


	return 0;
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

	::poll(fds, sizeof(fds) / sizeof(fds[0]), 100);

	return POLLIN;
}

int DevCommon::try_to_copy_data(char *buffer, size_t buflen, MessageType message_type)
{
	if (buflen == 0) {
		return -1;
	}

	switch (message_type) {
	case MessageType::Mavlink:
		if (_read_buffer->start_mavlink < _read_buffer->end_mavlink) {
			// We have Mavlink data ready to send.
			const size_t len_available = _read_buffer->end_mavlink - _read_buffer->start_mavlink;
			// We can only send what fits in the callers buffer.
			const size_t len_to_copy = math::min(len_available, buflen);

			// Copy it to the callers buffer and remove it from our buffer.
			_read_buffer->copy(buffer, _read_buffer->start_mavlink, len_to_copy);

			// Shift the markers accordingly.
			_read_buffer->start_mavlink += len_to_copy;

			// Keep track for stats.
			_read_buffer->mavlink_parsed += len_to_copy;

			// Update the lost/unused bytes count
			_read_buffer->update_lost_stats();

			// Update the number of MAVLink bytes parsed
			perf_set_count(mavlink_bytes_parsed_count, _read_buffer->mavlink_parsed);

			// Update the number of MAVLink messages parsed
			perf_count(mavlink_messages_parsed_count);

			return len_to_copy;

		} else {
			return -1;
		}

	case MessageType::Rtps:
		if (_read_buffer->start_rtps < _read_buffer->end_rtps) {
			// We have Rtps data ready to send
			const size_t len_available = _read_buffer->end_rtps - _read_buffer->start_rtps;
			// We can only send what fits in the callers buffer.
			const size_t len_to_copy = math::min(len_available, buflen);

			// Copy it to the callers buffer and remove it from our buffer.
			_read_buffer->copy(buffer, _read_buffer->start_rtps, len_to_copy);

			// Shift the markers accordingly.
			_read_buffer->start_rtps += len_to_copy;

			// Keep track for stats.
			_read_buffer->rtps_parsed += len_to_copy;

			// Update the lost/unused bytes count
			_read_buffer->update_lost_stats();

			// Update the number of RTPS bytes parsed
			perf_set_count(rtps_bytes_parsed_count, _read_buffer->rtps_parsed);

			// Update the number of RTPS messages parsed
			perf_count(rtps_messages_parsed_count);

			return len_to_copy;

		} else {
			return -1;
		}

		break;


	default:
		return -1;
	}
}

void DevCommon::scan_for_packets()
{
	if (_read_buffer->buf_size < Sp2HeaderSize) {
		// We have not even one header in the buffer, no need to scan yet.
		return;
	}

	bool mavlink_available = (_read_buffer->start_mavlink < _read_buffer->end_mavlink);
	bool rtps_available = (_read_buffer->start_rtps < _read_buffer->end_rtps);

	if (mavlink_available && rtps_available) {
		// We still have data for both, no need to scan yet.
		return;
	}

	const size_t begin = math::min(_read_buffer->end_mavlink, _read_buffer->end_rtps);

	for (size_t i = begin; i < _read_buffer->buf_size - Sp2HeaderSize; /* ++i */) {

		const Sp2Header *header = reinterpret_cast<Sp2Header *>(&_read_buffer->buffer[i]);

		if (header->fields.magic != Sp2HeaderMagic) {
			// Not the magic byte that we're looking for.
			++i;
			continue;
		}

		const uint8_t checksum = (_read_buffer->buffer[i] ^ _read_buffer->buffer[i + 1] ^ _read_buffer->buffer[i + 2]);

		if (header->fields.checksum != checksum) {
			// Checksum failed.
			++i;
			continue;
		}

		if (header->fields.type != static_cast<uint8_t>(MessageType::Mavlink) &&
		    header->fields.type != static_cast<uint8_t>(MessageType::Rtps)) {
			// Ignore unknown protocols
			++i;
			continue;
		}

		const size_t payload_len = ((uint16_t)header->fields.len_h << 8) | header->fields.len_l;

		if (payload_len > sizeof(_read_buffer->buffer)) {
			// This can happen if by accident data matches the header including checksum.
			// Given we skip most data using the last payload_len, we should not see this too often,
			// unless the link is very lossy and we often have to re-sync.
			PX4_DEBUG("payload size %zu > buffer size %zu: %d, protocol: %s",
				  payload_len, sizeof(_read_buffer->buffer),
				  (header->fields.type == static_cast<uint8_t>(MessageType::Mavlink)) ? "Mavlink" : "Rtps");
			++i;
			continue;
		}

		if (i + Sp2HeaderSize + payload_len > _read_buffer->buf_size) {
			// We don't have a enough data in the buffer yet, try again later.
			break;
		}

		if (header->fields.type == static_cast<uint8_t>(MessageType::Mavlink) && !mavlink_available) {
			_read_buffer->start_mavlink = i + Sp2HeaderSize;
			_read_buffer->end_mavlink = _read_buffer->start_mavlink + payload_len;
			mavlink_available = true;

			// Overwrite header magic byte, so we don't parse them again.
			_read_buffer->buffer[i] = 0;
			_read_buffer->header_bytes_received += Sp2HeaderSize;

		} else if (header->fields.type == static_cast<uint8_t>(MessageType::Rtps) && !rtps_available) {
			_read_buffer->start_rtps = i + Sp2HeaderSize;
			_read_buffer->end_rtps = _read_buffer->start_rtps + payload_len;
			rtps_available = true;

			// Overwrite header magic byte, so we don't parse them again.
			_read_buffer->buffer[i] = 0;
			_read_buffer->header_bytes_received += Sp2HeaderSize;
		}

		if (mavlink_available && rtps_available) {
			// Both have at least one message ready, we can stop now.
			break;
		}

		// Update the lost/unused bytes count
		_read_buffer->update_lost_stats();

		perf_set_count(header_bytes_received_count, _read_buffer->header_bytes_received);

		i += payload_len;

	}
}


void DevCommon::check_for_timeouts()
{
	// If a reader has timed out, mark its data as read.

	if (hrt_elapsed_time(&_read_buffer->last_mavlink_read) > reader_timeout_us) {
		if (_read_buffer->start_mavlink < _read_buffer->end_mavlink) {
			_read_buffer->start_mavlink = _read_buffer->end_mavlink;
		}
	}

	if (hrt_elapsed_time(&_read_buffer->last_rtps_read) > reader_timeout_us) {
		if (_read_buffer->start_rtps < _read_buffer->end_rtps) {
			_read_buffer->start_rtps = _read_buffer->end_rtps;
		}
	}
}

void DevCommon::cleanup()
{
	const bool mavlink_available = (_read_buffer->start_mavlink < _read_buffer->end_mavlink);
	const bool rtps_available = (_read_buffer->start_rtps < _read_buffer->end_rtps);

	// Clean up garbage bytes and accumulated headers

	size_t garbage_end = 0;

	if (!mavlink_available && !rtps_available) {
		garbage_end = math::max(_read_buffer->start_mavlink, _read_buffer->start_rtps);

	} else {
		garbage_end = math::min(_read_buffer->start_mavlink, _read_buffer->start_rtps);
	}

	if (garbage_end > 0) {
		_read_buffer->remove(0, garbage_end);

		_read_buffer->start_mavlink -= math::min(garbage_end, _read_buffer->start_mavlink);
		_read_buffer->end_mavlink -= math::min(garbage_end, _read_buffer->end_mavlink);
		_read_buffer->start_rtps -= math::min(garbage_end, _read_buffer->start_rtps);
		_read_buffer->end_rtps -= math::min(garbage_end, _read_buffer->end_rtps);
	}
}


class Mavlink2Dev : public DevCommon
{
public:
	Mavlink2Dev(ReadBuffer *read_buffer);
	virtual ~Mavlink2Dev() {}

	virtual ssize_t	read(struct file *filp, char *buffer, size_t buflen);
	virtual ssize_t	write(struct file *filp, const char *buffer, size_t buflen);
};

Mavlink2Dev::Mavlink2Dev(ReadBuffer *read_buffer)
	: DevCommon("/dev/mavlink", read_buffer)
{
	_header.fields.magic 		= Sp2HeaderMagic;
	_header.fields.len_h 		= 0;
	_header.fields.len_l 		= 0;
	_header.fields.checksum		= 0;
	_header.fields.type		= static_cast<uint8_t>(MessageType::Mavlink);
}

ssize_t Mavlink2Dev::read(struct file *filp, char *buffer, size_t buflen)
{
	_read_buffer->last_mavlink_read = hrt_absolute_time();

	lock(Read);

	// The cleanup needs to be right after a scan, so we don't clean up
	// something that we haven't found yet.
	scan_for_packets();
	check_for_timeouts();
	cleanup();

	// If we have already a packet ready in the current buffer, we don't have
	// to read and can grab data straightaway.
	int ret = try_to_copy_data(buffer, buflen, MessageType::Mavlink);

	if (ret > 0) {
		unlock(Read);
		return ret;
	}

	// Otherwise, we have to do a read.
	ret = _read_buffer->read(_fd);

	if (ret < 0) {
		unlock(Read);
		return ret;
	}

	// Now we need to check again if there is data available.
	scan_for_packets();

	// And try to copy it out.
	ret = try_to_copy_data(buffer, buflen, MessageType::Mavlink);

	unlock(Read);
	return ret;
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
			lock(Write);

		} else if ((unsigned char)buffer[0] == 254) { // mavlink 1
			uint8_t payload_len = buffer[1];
			_packet_len = payload_len + 8;

			_parser_state = ParserState::GotLength;
			lock(Write);

		} else {
			PX4_ERR("parser error");
			return 0;
		}

	/* FALLTHROUGH */

	case ParserState::GotLength: {
			_packet_len -= buflen;
			int buf_free;
			::ioctl(_fd, FIONSPACE, (unsigned long)&buf_free);

			if (buf_free < (int)buflen) {
				//let write fail, to let mavlink know the buffer would overflow
				//(this is because in the ioctl we pretend there is always enough space)
				ret = -1;

			} else {
				_header.fields.len_h = (buflen >> 8) & 0x7f;
				_header.fields.len_l = buflen & 0xff;
				_header.fields.checksum = _header.bytes[0] ^ _header.bytes[1] ^ _header.bytes[2];
				::write(_fd, _header.bytes, 4);
				ret = ::write(_fd, buffer, buflen);
			}

			if (_packet_len == 0) {
				unlock(Write);
				_parser_state = ParserState::Idle;
			}
		}

		break;
	}

	return ret;
}

class RtpsDev : public DevCommon
{
public:
	RtpsDev(ReadBuffer *read_buffer);
	virtual ~RtpsDev() {}

	virtual ssize_t	read(struct file *filp, char *buffer, size_t buflen);
	virtual ssize_t	write(struct file *filp, const char *buffer, size_t buflen);

protected:
	static const uint8_t HEADER_SIZE = 10;
};

RtpsDev::RtpsDev(ReadBuffer *read_buffer)
	: DevCommon("/dev/rtps", read_buffer)
{
	_header.fields.magic		= Sp2HeaderMagic;
	_header.fields.len_h		= 0;
	_header.fields.len_l		= 0;
	_header.fields.checksum		= 0;
	_header.fields.type		= static_cast<uint8_t>(MessageType::Rtps);
}

ssize_t RtpsDev::read(struct file *filp, char *buffer, size_t buflen)
{
	_read_buffer->last_rtps_read = hrt_absolute_time();

	lock(Read);

	scan_for_packets();
	check_for_timeouts();
	cleanup();

	// If we have already a packet ready in the current buffer, we don't have to read.
	int ret = try_to_copy_data(buffer, buflen, MessageType::Rtps);

	if (ret > 0) {
		unlock(Read);
		return ret;
	}

	// Otherwise, we have to do a read.
	ret = _read_buffer->read(_fd);

	if (ret < 0) {
		unlock(Read);
		return ret;
	}

	scan_for_packets();

	// And check again.
	ret = try_to_copy_data(buffer, buflen, MessageType::Rtps);

	unlock(Read);
	return ret;
}

ssize_t RtpsDev::write(struct file *filp, const char *buffer, size_t buflen)
{
	/*
	 * we need to look into the data to make sure the output is locked for the duration
	 * of a whole packet.
	 * assumptions:
	 * - packet header is written all at once (or at least it contains the payload length)
	 * - a single write call does not contain multiple (or parts of multiple) packets
	 */
	ssize_t ret = 0;
	uint16_t payload_len;

	switch (_parser_state) {
	case ParserState::Idle:
		ASSERT(buflen >= HEADER_SIZE);

		if (memcmp(buffer, ">>>", 3) != 0) {
			PX4_ERR("parser error");
			return 0;
		}

		payload_len = ((uint16_t)buffer[6] << 8) | buffer[7];
		_packet_len = payload_len + HEADER_SIZE;
		_parser_state = ParserState::GotLength;
		lock(Write);

	/* FALLTHROUGH */

	case ParserState::GotLength: {
			_packet_len -= buflen;
			int buf_free;
			::ioctl(_fd, FIONSPACE, (unsigned long)&buf_free);

			// TODO should I care about this for rtps?
			if ((unsigned)buf_free < buflen) {
				//let write fail, to let rtps know the buffer would overflow
				//(this is because in the ioctl we pretend there is always enough space)
				ret = -1;

			} else {
				_header.fields.len_h = (buflen >> 8) & 0x7f;
				_header.fields.len_l = buflen & 0xff;
				_header.fields.checksum = _header.bytes[0] ^ _header.bytes[1] ^ _header.bytes[2];
				::write(_fd, _header.bytes, 4);
				ret = ::write(_fd, buffer, buflen);
			}

			if (_packet_len == 0) {
				unlock(Write);
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
			PX4_WARN("already running");
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

		bytes_received_count = perf_alloc(PC_COUNT, "protocol_splitter: bytes received");
		bytes_lost_count = perf_alloc(PC_COUNT, "protocol_splitter: bytes unused/lost");
		header_bytes_received_count = perf_alloc(PC_COUNT, "protocol_splitter: header bytes received");
		mavlink_messages_parsed_count = perf_alloc(PC_COUNT, "protocol_splitter: MAVLink messages parsed");
		mavlink_bytes_parsed_count = perf_alloc(PC_COUNT, "protocol_splitter: MAVLink messages bytes parsed");
		rtps_messages_parsed_count = perf_alloc(PC_COUNT, "protocol_splitter: RTPS messages parsed");
		rtps_bytes_parsed_count = perf_alloc(PC_COUNT, "protocol_splitter: RTPS messages bytes parsed");

		strncpy(objects->device_name, argv[2], sizeof(objects->device_name));
		sem_init(&objects->r_lock, 1, 1);
		sem_init(&objects->w_lock, 1, 1);
		objects->read_buffer = new ReadBuffer();
		objects->mavlink2 = new Mavlink2Dev(objects->read_buffer);
		objects->rtps = new RtpsDev(objects->read_buffer);

		if (!objects->mavlink2 || !objects->rtps) {
			delete objects->mavlink2;
			delete objects->rtps;
			delete objects->read_buffer;
			sem_destroy(&objects->r_lock);
			sem_destroy(&objects->w_lock);
			delete objects;
			objects = nullptr;
			PX4_ERR("alloc failed");
			return -1;

		} else {
			objects->mavlink2->init();
			objects->rtps->init();
		}
	}

	if (!strcmp(argv[1], "stop")) {
		if (objects) {
			delete objects->mavlink2;
			delete objects->rtps;
			delete objects->read_buffer;
			sem_destroy(&objects->r_lock);
			sem_destroy(&objects->w_lock);
			delete objects;
			objects = nullptr;

			perf_free(bytes_received_count);
			perf_free(header_bytes_received_count);
			perf_free(bytes_lost_count);
			perf_free(mavlink_messages_parsed_count);
			perf_free(mavlink_bytes_parsed_count);
			perf_free(rtps_messages_parsed_count);
			perf_free(rtps_bytes_parsed_count);
		}
	}

	/*
	 * Print driver status.
	 */
	if (!strcmp(argv[1], "status")) {
		if (objects) {
			PX4_INFO("running");

			if (sem_wait(&objects->r_lock) != 0) {
				return -1;
			}

			objects->read_buffer->print_stats();
			sem_post(&objects->r_lock);

		} else {
			PX4_INFO("not running");
		}
	}

	return 0;

out:
	PX4_ERR("unrecognized command, try 'start <device>', 'stop', 'status'");
	return 1;
}
