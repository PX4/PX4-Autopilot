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

/**
 * @file mavlink_ftp.h
 *
 * MAVLink remote file server.
 *
 * Messages are wrapped in ENCAPSULATED_DATA messages. Every message includes
 * a session ID and sequence number.
 *
 * A limited number of requests (currently 2) may be outstanding at a time.
 * Additional messages will be discarded.
 *
 * Messages consist of a fixed header, followed by a data area.
 *
 */
 
#include <dirent.h>
#include <queue.h>

#include <nuttx/wqueue.h>
#include <systemlib/err.h>

#include "mavlink_messages.h"
#include "mavlink_main.h"

class MavlinkFTP
{
public:
	MavlinkFTP();

	static MavlinkFTP	*getServer();

	// static interface
	void			handle_message(Mavlink* mavlink,
					mavlink_message_t *msg);

private:

	static const unsigned	kRequestQueueSize = 2;

	static MavlinkFTP	*_server;

	struct RequestHeader
	{
		uint8_t		magic;
		uint8_t		session;
		uint8_t		opcode;
		uint8_t		size;
		uint32_t	crc32;
		uint32_t	offset;
		uint8_t		data[];
	};

	enum Opcode : uint8_t
	{
		kCmdNone,	// ignored, always acked
		kCmdTerminate,	// releases sessionID, closes file
		kCmdReset,	// terminates all sessions
		kCmdList,	// list files in <path> from <offset>
		kCmdOpen,	// opens <path> for reading, returns <session>
		kCmdRead,	// reads <size> bytes from <offset> in <session>
		kCmdCreate,	// creates <path> for writing, returns <session>
		kCmdWrite,	// appends <size> bytes at <offset> in <session>
		kCmdRemove,	// remove file (only if created by server?)

		kRspAck,
		kRspNak
	};

	enum ErrorCode : uint8_t
	{
		kErrNone,
		kErrNoRequest,
		kErrNoSession,
		kErrSequence,
		kErrNotDir,
		kErrNotFile,
		kErrEOF,
		kErrNotAppend,
		kErrTooBig,
		kErrIO,
		kErrPerm
	};

    int _findUnusedSession(void);
    bool _validSession(unsigned index);

    static const unsigned kMaxSession = 2;
    int	_session_fds[kMaxSession];

	class Request
	{
	public:
		union {
			dq_entry_t	entry;
			work_s		work;
		};

		bool		decode(Mavlink *mavlink, mavlink_message_t *fromMessage) {
			if (fromMessage->msgid == MAVLINK_MSG_ID_ENCAPSULATED_DATA) {
				_mavlink = mavlink;
				mavlink_msg_encapsulated_data_decode(fromMessage, &_message);
				return true;
			}
			return false;
		}

		void		reply() {

			// XXX the proper way would be an IOCTL / uORB call, rather than exploiting the
			// flat memory architecture, as we're operating between threads here.
			mavlink_message_t msg;
			msg.checksum = 0;
			unsigned len = mavlink_msg_encapsulated_data_pack_chan(_mavlink->get_system_id(), _mavlink->get_component_id(),
				_mavlink->get_channel(), &msg, sequence(), rawData());

			_mavlink->lockMessageBufferMutex();
			bool success = _mavlink->message_buffer_write(&msg, len);
			_mavlink->unlockMessageBufferMutex();

			if (!success) {
				warnx("FTP TX ERR");
			} 
			// else {
				// warnx("wrote: sys: %d, comp: %d, chan: %d, len: %d, checksum: %d",
				// 	_mavlink->get_system_id(),
				// 	_mavlink->get_component_id(),
				// 	_mavlink->get_channel(),
				// 	len,
				// 	msg.checksum);
			// }
		}

		uint8_t		*rawData() { return &_message.data[0]; }
		RequestHeader 	*header()  { return reinterpret_cast<RequestHeader *>(&_message.data[0]); }
		uint8_t         *requestData() { return &(header()->data[0]); }
		unsigned	dataSize() { return header()->size + sizeof(RequestHeader); }
		uint16_t	sequence() const { return _message.seqnr; }
		mavlink_channel_t channel() { return _mavlink->get_channel(); }

		char		*dataAsCString();

	private:
		Mavlink			*_mavlink;
		mavlink_encapsulated_data_t _message;

	};

	static const uint8_t	kProtocolMagic = 'f';
	static const char	kDirentFile = 'F';
	static const char	kDirentDir = 'D';
	static const char	kDirentUnknown = 'U';
	static const uint8_t	kMaxDataLength = MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN - sizeof(RequestHeader);

	/// Request worker; runs on the low-priority work queue to service
	/// remote requests.
	///
	static void		_workerTrampoline(void *arg);
	void			_worker(Request *req);

	/// Reply to a request (XXX should be a Request method)
	///
	void			_reply(Request *req);

	ErrorCode		_workList(Request *req);
	ErrorCode		_workOpen(Request *req, bool create);
	ErrorCode		_workRead(Request *req);
	ErrorCode		_workWrite(Request *req);
	ErrorCode		_workRemove(Request *req);
	ErrorCode		_workTerminate(Request *req);
	ErrorCode		_workReset();

	// work freelist
	Request			_workBufs[kRequestQueueSize];
	dq_queue_t		_workFree;
	sem_t			_lock;

	void			_qLock() { do {} while (sem_wait(&_lock) != 0); }
	void			_qUnlock() { sem_post(&_lock); }

	void			_qFree(Request *req) {
		_qLock();
		dq_addlast(&req->entry, &_workFree);
		_qUnlock();
	}

	Request			*_dqFree() {
		_qLock();
		auto req = reinterpret_cast<Request *>(dq_remfirst(&_workFree));
		_qUnlock();
		return req;
	}
    
};
