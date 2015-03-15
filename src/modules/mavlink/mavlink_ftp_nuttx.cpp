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

/// @file mavlink_ftp.cpp
///	@author px4dev, Don Gagne <don@thegagnes.com>

#include <crc32.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <errno.h>

#include "mavlink_ftp.h"
#include "mavlink_tests/mavlink_ftp_test.h"

// Uncomment the line below to get better debug output. Never commit with this left on.
//#define MAVLINK_FTP_DEBUG

MavlinkFTP *
MavlinkFTP::get_server(void)
{
	static MavlinkFTP server;
        return &server;
}

MavlinkFTP::MavlinkFTP() :
	_request_bufs{},
	_request_queue{},
	_request_queue_sem{},
	_utRcvMsgFunc{},
	_ftp_test{}
{
	// initialise the request freelist
	dq_init(&_request_queue);
	sem_init(&_request_queue_sem, 0, 1);

	// initialize session list
	for (size_t i=0; i<kMaxSession; i++) {
		_session_fds[i] = -1;
	}

	// drop work entries onto the free list
	for (unsigned i = 0; i < kRequestQueueSize; i++) {
		_return_request(&_request_bufs[i]);
	}

}

#ifdef MAVLINK_FTP_UNIT_TEST
void
MavlinkFTP::set_unittest_worker(ReceiveMessageFunc_t rcvMsgFunc, MavlinkFtpTest *ftp_test)
{
	_utRcvMsgFunc = rcvMsgFunc;
	_ftp_test = ftp_test;
}
#endif

void
MavlinkFTP::handle_message(Mavlink* mavlink, mavlink_message_t *msg)
{
	// get a free request
	struct Request* req = _get_request();

	// if we couldn't get a request slot, just drop it
	if (req == nullptr) {
		warnx("Dropping FTP request: queue full\n");
		return;
	}

	if (msg->msgid == MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL) {
		mavlink_msg_file_transfer_protocol_decode(msg, &req->message);
		
#ifdef MAVLINK_FTP_UNIT_TEST
		if (!_utRcvMsgFunc) {
			warnx("Incorrectly written unit test\n");
			return;
		}
		// We use fake ids when unit testing
		req->serverSystemId = MavlinkFtpTest::serverSystemId;
		req->serverComponentId = MavlinkFtpTest::serverComponentId;
		req->serverChannel = MavlinkFtpTest::serverChannel;
#else
		// Not unit testing, use the real thing
		req->serverSystemId = mavlink->get_system_id();
		req->serverComponentId = mavlink->get_component_id();
		req->serverChannel = mavlink->get_channel();
#endif
		
		// This is the system id we want to target when sending
		req->targetSystemId = msg->sysid;
			
		if (req->message.target_system == req->serverSystemId) {
			req->mavlink = mavlink;
#ifdef MAVLINK_FTP_UNIT_TEST
			// We are running in Unit Test mode. Don't queue, just call _worket directly.
			_process_request(req);
#else
			// We are running in normal mode. Queue the request to the worker
			work_queue(LPWORK, &req->work, &MavlinkFTP::_worker_trampoline, req, 0);
#endif
            return;
		}
	}

	_return_request(req);
}

/// @brief Queued static work queue routine to handle mavlink messages
void
MavlinkFTP::_worker_trampoline(void *arg)
{
	Request* req = reinterpret_cast<Request *>(arg);
	MavlinkFTP* server = MavlinkFTP::get_server();

	// call the server worker with the work item
	server->_process_request(req);
}

/// @brief Processes an FTP message
void
MavlinkFTP::_process_request(Request *req)
{
	PayloadHeader *payload = reinterpret_cast<PayloadHeader *>(&req->message.payload[0]);

	ErrorCode errorCode = kErrNone;

	// basic sanity checks; must validate length before use
	if (payload->size > kMaxDataLength) {
		errorCode = kErrInvalidDataSize;
		goto out;
	}

#ifdef MAVLINK_FTP_DEBUG
	printf("ftp: channel %u opc %u size %u offset %u\n", req->serverChannel, payload->opcode, payload->size, payload->offset);
#endif

	switch (payload->opcode) {
	case kCmdNone:
		break;

	case kCmdTerminateSession:
		errorCode = _workTerminate(payload);
		break;

	case kCmdResetSessions:
		errorCode = _workReset(payload);
		break;

	case kCmdListDirectory:
		errorCode = _workList(payload);
		break;

	case kCmdOpenFileRO:
		errorCode = _workOpen(payload, O_RDONLY);
		break;

	case kCmdCreateFile:
		errorCode = _workOpen(payload, O_CREAT | O_EXCL | O_WRONLY);
		break;

	case kCmdOpenFileWO:
		errorCode = _workOpen(payload, O_CREAT | O_WRONLY);
		break;

	case kCmdReadFile:
		errorCode = _workRead(payload);
		break;

	case kCmdWriteFile:
		errorCode = _workWrite(payload);
		break;

	case kCmdRemoveFile:
		errorCode = _workRemoveFile(payload);
		break;

	case kCmdRename:
		errorCode = _workRename(payload);
		break;

	case kCmdTruncateFile:
		errorCode = _workTruncateFile(payload);
		break;

	case kCmdCreateDirectory:
		errorCode = _workCreateDirectory(payload);
		break;
			
	case kCmdRemoveDirectory:
		errorCode = _workRemoveDirectory(payload);
		break;
			

	case kCmdCalcFileCRC32:
		errorCode = _workCalcFileCRC32(payload);
		break;

	default:
		errorCode = kErrUnknownCommand;
		break;	
	}

out:
	// handle success vs. error
	if (errorCode == kErrNone) {
		payload->req_opcode = payload->opcode;
		payload->opcode = kRspAck;
#ifdef MAVLINK_FTP_DEBUG
		warnx("FTP: ack\n");
#endif
	} else {
		int r_errno = errno;
		warnx("FTP: nak %u", errorCode);
		payload->req_opcode = payload->opcode;
		payload->opcode = kRspNak;
		payload->size = 1;
		payload->data[0] = errorCode;
		if (errorCode == kErrFailErrno) {
			payload->size = 2;
			payload->data[1] = r_errno;
		}
	}

	
	// respond to the request
	_reply(req);

	_return_request(req);
}

/// @brief Sends the specified FTP reponse message out through mavlink
void
MavlinkFTP::_reply(Request *req)
{
	PayloadHeader *payload = reinterpret_cast<PayloadHeader *>(&req->message.payload[0]);
	
	payload->seqNumber = payload->seqNumber + 1;

	mavlink_message_t msg;
	msg.checksum = 0;
#ifndef MAVLINK_FTP_UNIT_TEST
	uint16_t len =
#endif
	mavlink_msg_file_transfer_protocol_pack_chan(req->serverSystemId,	// Sender system id
								    req->serverComponentId,	// Sender component id
								    req->serverChannel,		// Channel to send on
								    &msg,			// Message to pack payload into
								    0,				// Target network
								    req->targetSystemId,	// Target system id
								    0,				// Target component id
								    (const uint8_t*)payload);	// Payload to pack into message
	
	bool success = true;
#ifdef MAVLINK_FTP_UNIT_TEST
	// Unit test hook is set, call that instead
	_utRcvMsgFunc(&msg, _ftp_test);
#else
	Mavlink	*mavlink = req->mavlink;
	
	mavlink->lockMessageBufferMutex();
	success = mavlink->message_buffer_write(&msg, len);
	mavlink->unlockMessageBufferMutex();
		
#endif

	if (!success) {
		warnx("FTP TX ERR");
	}
#ifdef MAVLINK_FTP_DEBUG
	else {
		warnx("wrote: sys: %d, comp: %d, chan: %d, checksum: %d",
		      req->serverSystemId,
		      req->serverComponentId,
		      req->serverChannel,
		      msg.checksum);
	}
#endif
}

/// @brief Responds to a List command
MavlinkFTP::ErrorCode
MavlinkFTP::_workList(PayloadHeader* payload)
{
    char dirPath[kMaxDataLength];
    strncpy(dirPath, _data_as_cstring(payload), kMaxDataLength);
    
	DIR *dp = opendir(dirPath);

	if (dp == nullptr) {
		warnx("FTP: can't open path '%s'", dirPath);
		return kErrFailErrno;
	}
    
#ifdef MAVLINK_FTP_DEBUG
	warnx("FTP: list %s offset %d", dirPath, payload->offset);
#endif

	ErrorCode errorCode = kErrNone;
	struct dirent entry, *result = nullptr;
	unsigned offset = 0;

	// move to the requested offset
	seekdir(dp, payload->offset);

	for (;;) {
		// read the directory entry
		if (readdir_r(dp, &entry, &result)) {
			warnx("FTP: list %s readdir_r failure\n", dirPath);
			errorCode = kErrFailErrno;
			break;
		}

		// no more entries?
		if (result == nullptr) {
			if (payload->offset != 0 && offset == 0) {
				// User is requesting subsequent dir entries but there were none. This means the user asked
				// to seek past EOF.
				errorCode = kErrEOF;
			}
			// Otherwise we are just at the last directory entry, so we leave the errorCode at kErrorNone to signal that
			break;
		}

		uint32_t fileSize = 0;
		char buf[256];
		char direntType;

		// Determine the directory entry type
		switch (entry.d_type) {
		case DTYPE_FILE:
			// For files we get the file size as well
			direntType = kDirentFile;
			snprintf(buf, sizeof(buf), "%s/%s", dirPath, entry.d_name);
			struct stat st;
			if (stat(buf, &st) == 0) {
				fileSize = st.st_size;
			}
			break;
		case DTYPE_DIRECTORY:
			if (strcmp(entry.d_name, ".") == 0 || strcmp(entry.d_name, "..") == 0) {
				// Don't bother sending these back
				direntType = kDirentSkip;
			} else {
				direntType = kDirentDir;
			}
			break;
		default:
			// We only send back file and diretory entries, skip everything else
			direntType = kDirentSkip;
		}
		
		if (direntType == kDirentSkip) {
			// Skip send only dirent identifier
			buf[0] = '\0';
		} else if (direntType == kDirentFile) {
			// Files send filename and file length
			snprintf(buf, sizeof(buf), "%s\t%d", entry.d_name, fileSize);
		} else {
			// Everything else just sends name
			strncpy(buf, entry.d_name, sizeof(buf));
			buf[sizeof(buf)-1] = 0;
		}
		size_t nameLen = strlen(buf);

		// Do we have room for the name, the one char directory identifier and the null terminator?
		if ((offset + nameLen + 2) > kMaxDataLength) {
			break;
		}
		
		// Move the data into the buffer
		payload->data[offset++] = direntType;
		strcpy((char *)&payload->data[offset], buf);
#ifdef MAVLINK_FTP_DEBUG
		printf("FTP: list %s %s\n", dirPath, (char *)&payload->data[offset-1]);
#endif
		offset += nameLen + 1;
	}

	closedir(dp);
	payload->size = offset;

	return errorCode;
}

/// @brief Responds to an Open command
MavlinkFTP::ErrorCode
MavlinkFTP::_workOpen(PayloadHeader* payload, int oflag)
{
	int session_index = _find_unused_session();
	if (session_index < 0) {
		warnx("FTP: Open failed - out of sessions\n");
		return kErrNoSessionsAvailable;
	}

	char *filename = _data_as_cstring(payload);

	uint32_t fileSize = 0;
	struct stat st;
	if (stat(filename, &st) != 0) {
		// fail only if requested open for read
		if (oflag & O_RDONLY)
			return kErrFailErrno;
		else
			st.st_size = 0;
	}
	fileSize = st.st_size;

	int fd = ::open(filename, oflag);
	if (fd < 0) {
		return kErrFailErrno;
	}
	_session_fds[session_index] = fd;

	payload->session = session_index;
	payload->size = sizeof(uint32_t);
	*((uint32_t*)payload->data) = fileSize;

	return kErrNone;
}

/// @brief Responds to a Read command
MavlinkFTP::ErrorCode
MavlinkFTP::_workRead(PayloadHeader* payload)
{
	int session_index = payload->session;

	if (!_valid_session(session_index)) {
		return kErrInvalidSession;
	}

	// Seek to the specified position
#ifdef MAVLINK_FTP_DEBUG
	warnx("seek %d", payload->offset);
#endif
	if (lseek(_session_fds[session_index], payload->offset, SEEK_SET) < 0) {
		// Unable to see to the specified location
		warnx("seek fail");
		return kErrEOF;
	}

	int bytes_read = ::read(_session_fds[session_index], &payload->data[0], kMaxDataLength);
	if (bytes_read < 0) {
		// Negative return indicates error other than eof
		warnx("read fail %d", bytes_read);
		return kErrFailErrno;
	}

	payload->size = bytes_read;

	return kErrNone;
}

/// @brief Responds to a Write command
MavlinkFTP::ErrorCode
MavlinkFTP::_workWrite(PayloadHeader* payload)
{
	int session_index = payload->session;

	if (!_valid_session(session_index)) {
		return kErrInvalidSession;
	}

	// Seek to the specified position
#ifdef MAVLINK_FTP_DEBUG
	warnx("seek %d", payload->offset);
#endif
	if (lseek(_session_fds[session_index], payload->offset, SEEK_SET) < 0) {
		// Unable to see to the specified location
		warnx("seek fail");
		return kErrFailErrno;
	}

	int bytes_written = ::write(_session_fds[session_index], &payload->data[0], payload->size);
	if (bytes_written < 0) {
		// Negative return indicates error other than eof
		warnx("write fail %d", bytes_written);
		return kErrFailErrno;
	}

	payload->size = sizeof(uint32_t);
	*((uint32_t*)payload->data) = bytes_written;

	return kErrNone;
}

/// @brief Responds to a RemoveFile command
MavlinkFTP::ErrorCode
MavlinkFTP::_workRemoveFile(PayloadHeader* payload)
{
	char file[kMaxDataLength];
	strncpy(file, _data_as_cstring(payload), kMaxDataLength);
	
	if (unlink(file) == 0) {
		payload->size = 0;
		return kErrNone;
	} else {
		return kErrFailErrno;
	}
}

/// @brief Responds to a TruncateFile command
MavlinkFTP::ErrorCode
MavlinkFTP::_workTruncateFile(PayloadHeader* payload)
{
	char file[kMaxDataLength];
	const char temp_file[] = "/fs/microsd/.trunc.tmp";
	strncpy(file, _data_as_cstring(payload), kMaxDataLength);
	payload->size = 0;

	// emulate truncate(file, payload->offset) by
	// copying to temp and overwrite with O_TRUNC flag.

	struct stat st;
	if (stat(file, &st) != 0) {
		return kErrFailErrno;
	}

	if (!S_ISREG(st.st_mode)) {
		errno = EISDIR;
		return kErrFailErrno;
	}

	// check perms allow us to write (not romfs)
	if (!(st.st_mode & (S_IWUSR | S_IWGRP | S_IWOTH))) {
		errno = EROFS;
		return kErrFailErrno;
	}

	if (payload->offset == (unsigned)st.st_size) {
		// nothing to do
		return kErrNone;
	}
	else if (payload->offset == 0) {
		// 1: truncate all data
		int fd = ::open(file, O_TRUNC | O_WRONLY);
		if (fd < 0) {
			return kErrFailErrno;
		}

		::close(fd);
		return kErrNone;
	}
	else if (payload->offset > (unsigned)st.st_size) {
		// 2: extend file
		int fd = ::open(file, O_WRONLY);
		if (fd < 0) {
			return kErrFailErrno;
		}

		if (lseek(fd, payload->offset - 1, SEEK_SET) < 0) {
			::close(fd);
			return kErrFailErrno;
		}

		bool ok = 1 == ::write(fd, "", 1);
		::close(fd);

		return (ok)? kErrNone : kErrFailErrno;
	}
	else {
		// 3: truncate
		if (_copy_file(file, temp_file, payload->offset) != 0) {
			return kErrFailErrno;
		}
		if (_copy_file(temp_file, file, payload->offset) != 0) {
			return kErrFailErrno;
		}
		if (::unlink(temp_file) != 0) {
			return kErrFailErrno;
		}

		return kErrNone;
	}
}

/// @brief Responds to a Terminate command
MavlinkFTP::ErrorCode
MavlinkFTP::_workTerminate(PayloadHeader* payload)
{
	if (!_valid_session(payload->session)) {
		return kErrInvalidSession;
	}
    
	::close(_session_fds[payload->session]);
	_session_fds[payload->session] = -1;
	
	payload->size = 0;

	return kErrNone;
}

/// @brief Responds to a Reset command
MavlinkFTP::ErrorCode
MavlinkFTP::_workReset(PayloadHeader* payload)
{
	for (size_t i=0; i<kMaxSession; i++) {
		if (_session_fds[i] != -1) {
			::close(_session_fds[i]);
			_session_fds[i] = -1;
		}
	}

	payload->size = 0;
	
	return kErrNone;
}

/// @brief Responds to a Rename command
MavlinkFTP::ErrorCode
MavlinkFTP::_workRename(PayloadHeader* payload)
{
	char oldpath[kMaxDataLength];
	char newpath[kMaxDataLength];

	char *ptr = _data_as_cstring(payload);
	size_t oldpath_sz = strlen(ptr);

	if (oldpath_sz == payload->size) {
		// no newpath
		errno = EINVAL;
		return kErrFailErrno;
	}

	strncpy(oldpath, ptr, kMaxDataLength);
	strncpy(newpath, ptr + oldpath_sz + 1, kMaxDataLength);

	if (rename(oldpath, newpath) == 0) {
		payload->size = 0;
		return kErrNone;
	} else {
		return kErrFailErrno;
	}
}

/// @brief Responds to a RemoveDirectory command
MavlinkFTP::ErrorCode
MavlinkFTP::_workRemoveDirectory(PayloadHeader* payload)
{
	char dir[kMaxDataLength];
	strncpy(dir, _data_as_cstring(payload), kMaxDataLength);
	
	if (rmdir(dir) == 0) {
		payload->size = 0;
		return kErrNone;
	} else {
		return kErrFailErrno;
	}
}

/// @brief Responds to a CreateDirectory command
MavlinkFTP::ErrorCode
MavlinkFTP::_workCreateDirectory(PayloadHeader* payload)
{
	char dir[kMaxDataLength];
	strncpy(dir, _data_as_cstring(payload), kMaxDataLength);
	
	if (mkdir(dir, S_IRWXU | S_IRWXG | S_IRWXO) == 0) {
		payload->size = 0;
		return kErrNone;
	} else {
		return kErrFailErrno;
	}
}

/// @brief Responds to a CalcFileCRC32 command
MavlinkFTP::ErrorCode
MavlinkFTP::_workCalcFileCRC32(PayloadHeader* payload)
{
	char file_buf[256];
	uint32_t checksum = 0;
	ssize_t bytes_read;
	strncpy(file_buf, _data_as_cstring(payload), kMaxDataLength);

	int fd = ::open(file_buf, O_RDONLY);
	if (fd < 0) {
		return kErrFailErrno;
	}

	do {
		bytes_read = ::read(fd, file_buf, sizeof(file_buf));
		if (bytes_read < 0) {
			int r_errno = errno;
			::close(fd);
			errno = r_errno;
			return kErrFailErrno;
		}

		checksum = crc32part((uint8_t*)file_buf, bytes_read, checksum);
	} while (bytes_read == sizeof(file_buf));

	::close(fd);

	payload->size = sizeof(uint32_t);
	*((uint32_t*)payload->data) = checksum;
	return kErrNone;
}

/// @brief Returns true if the specified session is a valid open session
bool
MavlinkFTP::_valid_session(unsigned index)
{
	if ((index >= kMaxSession) || (_session_fds[index] < 0)) {
		return false;
	}
	return true;
}

/// @brief Returns an unused session index
int
MavlinkFTP::_find_unused_session(void)
{
	for (size_t i=0; i<kMaxSession; i++) {
		if (_session_fds[i] == -1) {
			return i;
		}
	}

	return -1;
}

/// @brief Guarantees that the payload data is null terminated.
///     @return Returns a pointer to the payload data as a char *
char *
MavlinkFTP::_data_as_cstring(PayloadHeader* payload)
{
	// guarantee nul termination
	if (payload->size < kMaxDataLength) {
		payload->data[payload->size] = '\0';
	} else {
		payload->data[kMaxDataLength - 1] = '\0';
	}

	// and return data
	return (char *)&(payload->data[0]);
}

/// @brief Returns a unused Request entry. NULL if none available.
MavlinkFTP::Request *
MavlinkFTP::_get_request(void)
{
	_lock_request_queue();
	Request* req = reinterpret_cast<Request *>(dq_remfirst(&_request_queue));
	_unlock_request_queue();
	return req;
}

/// @brief Locks a semaphore to provide exclusive access to the request queue
void
MavlinkFTP::_lock_request_queue(void)
{
	do {}
	while (sem_wait(&_request_queue_sem) != 0);
}

/// @brief Unlocks the semaphore providing exclusive access to the request queue
void
MavlinkFTP::_unlock_request_queue(void)
{
	sem_post(&_request_queue_sem);
}

/// @brief Returns a no longer needed request to the queue
void
MavlinkFTP::_return_request(Request *req)
{
	_lock_request_queue();
	dq_addlast(&req->work.dq, &_request_queue);
	_unlock_request_queue();
}

/// @brief Copy file (with limited space)
int
MavlinkFTP::_copy_file(const char *src_path, const char *dst_path, size_t length)
{
	char buff[512];
	int src_fd = -1, dst_fd = -1;
	int op_errno = 0;

	src_fd = ::open(src_path, O_RDONLY);
	if (src_fd < 0) {
		return -1;
	}

	dst_fd = ::open(dst_path, O_CREAT | O_TRUNC | O_WRONLY);
	if (dst_fd < 0) {
		op_errno = errno;
		::close(src_fd);
		errno = op_errno;
		return -1;
	}

	while (length > 0) {
		ssize_t bytes_read, bytes_written;
		size_t blen = (length > sizeof(buff))? sizeof(buff) : length;

		bytes_read = ::read(src_fd, buff, blen);
		if (bytes_read == 0) {
			// EOF
			break;
		}
		else if (bytes_read < 0) {
			warnx("cp: read");
			op_errno = errno;
			break;
		}

		bytes_written = ::write(dst_fd, buff, bytes_read);
		if (bytes_written != bytes_read) {
			warnx("cp: short write");
			op_errno = errno;
			break;
		}

		length -= bytes_written;
	}

	::close(src_fd);
	::close(dst_fd);

	errno = op_errno;
	return (length > 0)? -1 : 0;
}
