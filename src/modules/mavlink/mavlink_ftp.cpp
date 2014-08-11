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

#include <crc32.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>

#include "mavlink_ftp.h"

MavlinkFTP *MavlinkFTP::_server;

MavlinkFTP *
MavlinkFTP::getServer()
{
	// XXX this really cries out for some locking...
	if (_server == nullptr) {
		_server = new MavlinkFTP;
	}
	return _server;
}

MavlinkFTP::MavlinkFTP() :
	_session_fds{},
	_workBufs{},
	_workFree{},
	_lock{}
{
	// initialise the request freelist
	dq_init(&_workFree);
	sem_init(&_lock, 0, 1);

	// initialize session list
	for (size_t i=0; i<kMaxSession; i++) {
		_session_fds[i] = -1;
	}

	// drop work entries onto the free list
	for (unsigned i = 0; i < kRequestQueueSize; i++) {
		_qFree(&_workBufs[i]);
	}

}

void
MavlinkFTP::handle_message(Mavlink* mavlink, mavlink_message_t *msg)
{
	// get a free request
	auto req = _dqFree();

	// if we couldn't get a request slot, just drop it
	if (req != nullptr) {

		// decode the request
		if (req->decode(mavlink, msg)) {

			// and queue it for the worker
			work_queue(LPWORK, &req->work, &MavlinkFTP::_workerTrampoline, req, 0);
		} else {
			_qFree(req);
		}
	}
}

void
MavlinkFTP::_workerTrampoline(void *arg)
{
	auto req = reinterpret_cast<Request *>(arg);
	auto server = MavlinkFTP::getServer();

	// call the server worker with the work item
	server->_worker(req);
}

void
MavlinkFTP::_worker(Request *req)
{
	auto hdr = req->header();
	ErrorCode errorCode = kErrNone;
	uint32_t messageCRC;

	// basic sanity checks; must validate length before use
	if ((hdr->magic != kProtocolMagic) || (hdr->size > kMaxDataLength)) {
		errorCode = kErrNoRequest;
		goto out;
	}

	// check request CRC to make sure this is one of ours
	messageCRC = hdr->crc32;
	hdr->crc32 = 0;
	if (crc32(req->rawData(), req->dataSize()) != messageCRC) {
		errorCode = kErrNoRequest;
		goto out;
		warnx("ftp: bad crc");
	}

	//printf("ftp: channel %u opc %u size %u offset %u\n", req->channel(), hdr->opcode, hdr->size, hdr->offset);

	switch (hdr->opcode) {
	case kCmdNone:
		break;

	case kCmdTerminate:
		errorCode = _workTerminate(req);
		break;

	case kCmdReset:
		errorCode = _workReset();
		break;

	case kCmdList:
		errorCode = _workList(req);
		break;

	case kCmdOpen:
		errorCode = _workOpen(req, false);
		break;

	case kCmdCreate:
		errorCode = _workOpen(req, true);
		break;

	case kCmdRead:
		errorCode = _workRead(req);
		break;

	case kCmdWrite:
		errorCode = _workWrite(req);
		break;

	case kCmdRemove:
		errorCode = _workRemove(req);
		break;

	default:
		errorCode = kErrNoRequest;
		break;	
	}

out:
	// handle success vs. error
	if (errorCode == kErrNone) {
		hdr->opcode = kRspAck;
		//warnx("FTP: ack\n");
	} else {
		warnx("FTP: nak %u", errorCode);
		hdr->opcode = kRspNak;
		hdr->size = 1;
		hdr->data[0] = errorCode;
	}

	// respond to the request
	_reply(req);

	// free the request buffer back to the freelist
	_qFree(req);
}

void
MavlinkFTP::_reply(Request *req)
{
	auto hdr = req->header();
	
	hdr->magic = kProtocolMagic;

	// message is assumed to be already constructed in the request buffer, so generate the CRC
	hdr->crc32 = 0;
	hdr->crc32 = crc32(req->rawData(), req->dataSize());

	// then pack and send the reply back to the request source
	req->reply();
}

MavlinkFTP::ErrorCode
MavlinkFTP::_workList(Request *req)
{
	auto hdr = req->header();
    
    char dirPath[kMaxDataLength];
    strncpy(dirPath, req->dataAsCString(), kMaxDataLength);
    
	DIR *dp = opendir(dirPath);

	if (dp == nullptr) {
		warnx("FTP: can't open path '%s'", dirPath);
		return kErrNotDir;
	}
    
	//warnx("FTP: list %s offset %d", dirPath, hdr->offset);

	ErrorCode errorCode = kErrNone;
	struct dirent entry, *result = nullptr;
	unsigned offset = 0;

	// move to the requested offset
	seekdir(dp, hdr->offset);

	for (;;) {
		// read the directory entry
		if (readdir_r(dp, &entry, &result)) {
			warnx("FTP: list %s readdir_r failure\n", dirPath);
			errorCode = kErrIO;
			break;
		}

		// no more entries?
		if (result == nullptr) {
			if (hdr->offset != 0 && offset == 0) {
				// User is requesting subsequent dir entries but there were none. This means the user asked
				// to seek past EOF.
				errorCode = kErrEOF;
			}
			// Otherwise we are just at the last directory entry, so we leave the errorCode at kErrorNone to signal that
			break;
		}

		uint32_t fileSize = 0;
		char buf[256];

		// store the type marker
		switch (entry.d_type) {
		case DTYPE_FILE:
			hdr->data[offset++] = kDirentFile;
			snprintf(buf, sizeof(buf), "%s/%s", dirPath, entry.d_name);
			struct stat st;
			if (stat(buf, &st) == 0) {
				fileSize = st.st_size;
			}
			break;
		case DTYPE_DIRECTORY:
			hdr->data[offset++] = kDirentDir;
			break;
		default:
			hdr->data[offset++] = kDirentUnknown;
			break;
		}
		
		if (entry.d_type == DTYPE_FILE) {
			snprintf(buf, sizeof(buf), "%s\t%d", entry.d_name, fileSize);
		} else {
			strncpy(buf, entry.d_name, sizeof(buf));
			buf[sizeof(buf)-1] = 0;
		}
		size_t nameLen = strlen(buf);

		// name too big to fit?
		if ((nameLen + offset + 2) > kMaxDataLength) {
			break;
		}
		
		// copy the name, which we know will fit
		strcpy((char *)&hdr->data[offset], buf);
		//printf("FTP: list %s %s\n", dirPath, (char *)&hdr->data[offset-1]);
		offset += nameLen + 1;
	}

	closedir(dp);
	hdr->size = offset;

	return errorCode;
}

MavlinkFTP::ErrorCode
MavlinkFTP::_workOpen(Request *req, bool create)
{
	auto hdr = req->header();
    
	int session_index = _findUnusedSession();
	if (session_index < 0) {
		return kErrNoSession;
	}

	
	uint32_t fileSize = 0;
	if (!create) {
		struct stat st;
		if (stat(req->dataAsCString(), &st) != 0) {
			return kErrNotFile;
		}
		fileSize = st.st_size;
	}

	int oflag = create ? (O_CREAT | O_EXCL | O_APPEND) : O_RDONLY;
    
	int fd = ::open(req->dataAsCString(), oflag);
	if (fd < 0) {
		return create ? kErrPerm : kErrNotFile;
	}
	_session_fds[session_index] = fd;

	hdr->session = session_index;
	if (create) {
		hdr->size = 0;
	} else {
		hdr->size = sizeof(uint32_t);
		*((uint32_t*)hdr->data) = fileSize;
	}

	return kErrNone;
}

MavlinkFTP::ErrorCode
MavlinkFTP::_workRead(Request *req)
{
	auto hdr = req->header();

	int session_index = hdr->session;

	if (!_validSession(session_index)) {
		return kErrNoSession;
	}

	// Seek to the specified position
	//warnx("seek %d", hdr->offset);
	if (lseek(_session_fds[session_index], hdr->offset, SEEK_SET) < 0) {
		// Unable to see to the specified location
		warnx("seek fail");
		return kErrEOF;
	}

	int bytes_read = ::read(_session_fds[session_index], &hdr->data[0], kMaxDataLength);
	if (bytes_read < 0) {
		// Negative return indicates error other than eof
		warnx("read fail %d", bytes_read);
		return kErrIO;
	}

	hdr->size = bytes_read;

	return kErrNone;
}

MavlinkFTP::ErrorCode
MavlinkFTP::_workWrite(Request *req)
{
#if 0
    // NYI: Coming soon
	auto hdr = req->header();

	// look up session
	auto session = getSession(hdr->session);
	if (session == nullptr) {
		return kErrNoSession;
	}

	// append to file
	int result = session->append(hdr->offset, &hdr->data[0], hdr->size);

	if (result < 0) {
		// XXX might also be no space, I/O, etc.
		return kErrNotAppend;
	}

	hdr->size = result;
	return kErrNone;
#else
	return kErrPerm;
#endif
}

MavlinkFTP::ErrorCode
MavlinkFTP::_workRemove(Request *req)
{
	//auto hdr = req->header();

	// for now, send error reply
	return kErrPerm;
}

MavlinkFTP::ErrorCode
MavlinkFTP::_workTerminate(Request *req)
{
	auto hdr = req->header();
    
	if (!_validSession(hdr->session)) {
		return kErrNoSession;
	}
    
	::close(_session_fds[hdr->session]);

	return kErrNone;
}

MavlinkFTP::ErrorCode
MavlinkFTP::_workReset(void)
{
	for (size_t i=0; i<kMaxSession; i++) {
		if (_session_fds[i] != -1) {
			::close(_session_fds[i]);
			_session_fds[i] = -1;
		}
	}

	return kErrNone;
}

bool
MavlinkFTP::_validSession(unsigned index)
{
	if ((index >= kMaxSession) || (_session_fds[index] < 0)) {
		return false;
	}
	return true;
}

int
MavlinkFTP::_findUnusedSession(void)
{
	for (size_t i=0; i<kMaxSession; i++) {
		if (_session_fds[i] == -1) {
			return i;
		}
	}

	return -1;
}

char *
MavlinkFTP::Request::dataAsCString()
{
	// guarantee nul termination
	if (header()->size < kMaxDataLength) {
		requestData()[header()->size] = '\0';
	} else {
		requestData()[kMaxDataLength - 1] = '\0';
	}

	// and return data
	return (char *)&(header()->data[0]);
}
