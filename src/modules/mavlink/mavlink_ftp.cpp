/****************************************************************************
 *
 *   Copyright (c) 2014-2021 PX4 Development Team. All rights reserved.
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

#if defined(__PX4_NUTTX)
#include <nuttx/crc32.h>
#else
#include <crc32.h>
#endif
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <errno.h>
#include <cstring>

#include "mavlink_ftp.h"
#include "mavlink_main.h"

using namespace time_literals;

constexpr const char MavlinkFTP::_root_dir[];
constexpr const char MavlinkFTP::_mav_log_prefix[];
constexpr const char MavlinkFTP::_mav_log_dir[];

MavlinkFTP::MavlinkFTP(Mavlink &mavlink) :
	_mavlink(mavlink)
{
}

MavlinkFTP::~MavlinkFTP()
{
	_close_all_sessions();
}

bool
MavlinkFTP::burst_active() const
{
	for (const SessionInfo &session : _sessions) {
		if (session.stream_download) {
			return true;
		}
	}

	return false;
}

MavlinkFTP::SessionInfo *
MavlinkFTP::_find_session(uint8_t id, uint32_t system_id, uint8_t component_id)
{
	for (SessionInfo &session : _sessions) {
		if (session.is_open() && (session.id == id)
		    && (session.owner_system_id == system_id) && (session.owner_component_id == component_id)) {
			return &session;
		}
	}

	return nullptr;
}

MavlinkFTP::SessionInfo *
MavlinkFTP::_free_session()
{
	for (SessionInfo &session : _sessions) {
		if (!session.is_open()) {
			return &session;
		}
	}

	return nullptr;
}

uint8_t
MavlinkFTP::_allocate_session_id()
{
	// Only called with a slot free, so at most kMaxSessions - 1 ids are taken.
	bool in_use;

	do {
		in_use = false;

		for (const SessionInfo &session : _sessions) {
			if (session.is_open() && (session.id == _next_session_id)) {
				in_use = true;
				_next_session_id++;
				break;
			}
		}
	} while (in_use);

	return _next_session_id++;
}

void
MavlinkFTP::_close_session(SessionInfo &session)
{
	if (session.fd >= 0) {
		::close(session.fd);
	}

	session.fd = -1;
	session.param.close();
	session.stream_download = false;
	session.file_size = 0;
	_last_reply_valid = false;
}

void
MavlinkFTP::_close_all_sessions()
{
	for (SessionInfo &session : _sessions) {
		if (session.is_open()) {
			_close_session(session);
		}
	}
}

int
MavlinkFTP::_read_session(SessionInfo &session, uint32_t offset, uint8_t *buf, uint16_t count)
{
	if (session.param.is_open()) {
		const int bytes_read = session.param.read(offset, buf, count);

		if (bytes_read < 0) {
			_our_errno = EIO;
		}

		return bytes_read;
	}

	// lseek allows seeking past EOF, so test it ourselves
	if (offset >= session.file_size) {
		return 0;
	}

	if (lseek(session.fd, offset, SEEK_SET) < 0) {
		_our_errno = errno;
		return -1;
	}

	const int bytes_read = ::read(session.fd, buf, count);

	if (bytes_read < 0) {
		_our_errno = errno;
	}

	return bytes_read;
}

unsigned
MavlinkFTP::packet_overhead() const
{
	// PayloadHeader, the three target fields and the v2 framing, plus the
	// signature block when signing is active - without accounting for that the
	// packet silently grows past what the radio carries in one piece again.
	unsigned overhead = sizeof(PayloadHeader) + 3 + MAVLINK_NUM_NON_PAYLOAD_BYTES;

	const mavlink_status_t *status = mavlink_get_channel_status(_mavlink.get_channel());

	if ((status != nullptr) && (status->signing != nullptr)) {
		overhead += MAVLINK_SIGNATURE_BLOCK_LEN;
	}

	return overhead;
}

unsigned
MavlinkFTP::radio_clamped_length(unsigned requested) const
{
	// Only shrink behind a radio; a direct link has no such constraint.
	if (!_mavlink.radio_status_available()) {
		return requested;
	}

	// Only MAVLink 2 drops the trailing zeros of a payload, so on a MAVLink 1
	// link a shorter reply still goes out as a full 251 byte payload. Shrinking
	// it there would cost the client bytes without making the packet smaller.
	if (_mavlink.getProtocolVersion() == 1) {
		return requested;
	}

	const unsigned overhead = packet_overhead();

	if (kRadioMaxPacketLength <= overhead) {
		return requested;
	}

	return math::min(requested, kRadioMaxPacketLength - overhead);
}

unsigned
MavlinkFTP::_burst_data_length(const SessionInfo &session) const
{
	// What the client asked for. Zero means as much as fits, which is what
	// ArduPilot does and what the spec describes.
	const unsigned requested = (session.stream_size == 0) ?
				   (unsigned)kMaxDataLength :
				   math::min((unsigned)session.stream_size, (unsigned)kMaxDataLength);

	// The clamp comes after the request, never instead of it: every ground
	// station out there asks for a full payload, so honouring the request on
	// its own would hand a radio packets it has to split.
	return radio_clamped_length(requested);
}

unsigned
MavlinkFTP::_burst_wire_size(const SessionInfo &session) const
{
	// A MAVLink 1 packet carries the whole payload whatever we put in it, so
	// that is what it costs the link, and pacing has to budget for it.
	if (_mavlink.getProtocolVersion() == 1) {
		return kMaxDataLength + packet_overhead();
	}

	return _burst_data_length(session) + packet_overhead();
}

unsigned
MavlinkFTP::get_size()
{
	if (burst_active()) {
		return MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;

	} else {
		return 0;
	}
}

uint32_t
MavlinkFTP::_getServerSystemId()
{
	return _mavlink.get_system_id();
}

uint8_t
MavlinkFTP::_getServerComponentId()
{
	return _mavlink.get_component_id();
}

uint8_t
MavlinkFTP::_getServerChannel()
{
	return _mavlink.get_channel();
}

void
MavlinkFTP::handle_message(const mavlink_message_t *msg)
{
	//warnx("MavlinkFTP::handle_message %d %d", buf_size_1, buf_size_2);

	if (msg->msgid == MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL) {
		mavlink_file_transfer_protocol_t ftp_request;
		mavlink_msg_file_transfer_protocol_decode(msg, &ftp_request);

		const uint32_t target_system = mavlink_target_system(msg, ftp_request.target_system);

		PX4_DEBUG("FTP: received ftp protocol message target_system: %" PRIu32 " target_component: %d, seq: %d",
			  target_system, ftp_request.target_component, msg->seq);

		if ((target_system == _getServerSystemId() || target_system == 0) &&
		    (ftp_request.target_component == _getServerComponentId() || ftp_request.target_component == 0)) {
			_process_request(&ftp_request, msg->sysid, msg->compid);
		}
	}
}

/// @brief Processes an FTP message
void
MavlinkFTP::_process_request(
	mavlink_file_transfer_protocol_t *ftp_req,
	uint32_t target_system_id,
	uint8_t target_comp_id)
{
	bool stream_send = false;
	PayloadHeader *payload = reinterpret_cast<PayloadHeader *>(&ftp_req->payload[0]);
	SessionInfo *session = nullptr;

	ErrorCode errorCode = kErrNone;

	_ensure_buffers_exist();

	// basic sanity checks; must validate length before use
	if (payload->size > kMaxDataLength) {
		errorCode = kErrInvalidDataSize;
		PX4_WARN("invalid data size: %d", payload->size);
		goto out;
	}

	// check the sequence number: if this is a resent request, resend the last response
	if (_last_reply_valid) {
		const mavlink_file_transfer_protocol_t *last_reply = reinterpret_cast<const mavlink_file_transfer_protocol_t *>
				(_last_reply);
		const PayloadHeader *last_payload = reinterpret_cast<const PayloadHeader *>(&last_reply->payload[0]);

		if (payload->seq_number + 1 == last_payload->seq_number
		    && _last_reply_target_system == target_system_id
		    && last_reply->target_component == target_comp_id) {
			// this is the same request as the one we replied to last. It means the (n)ack got lost, and the GCS
			// resent the request. The request buffer is no longer needed, so expand the compact cache into it;
			// the serializer reads the whole message.
			memset(ftp_req, 0, sizeof(*ftp_req));
			memcpy(ftp_req, _last_reply, sizeof(_last_reply));
			_mavlink.lock_send();
			mavlink_msg_send_struct_target(_mavlink.get_channel(), FILE_TRANSFER_PROTOCOL, ftp_req, target_system_id);
			_mavlink.unlock_send();
			return;
		}
	}


	PX4_DEBUG("ftp: channel %" PRIu8 " opc %" PRIu8 " size %" PRIu8 " offset %" PRIu32,
		  _getServerChannel(), payload->opcode, payload->size, payload->offset);

	session = _find_session(payload->session, target_system_id, target_comp_id);

	switch (payload->opcode) {
	case kCmdNone:
		break;

	case kCmdTerminateSession:
		errorCode = _workTerminate(payload, session);
		break;

	case kCmdResetSessions:
		errorCode = _workReset(payload);
		break;

	case kCmdListDirectory:
		errorCode = _workList(payload);
		break;

	case kCmdListDirectoryWithTime:
		errorCode = _workList(payload, true);
		break;

	case kCmdOpenFileRO:
		errorCode = _workOpen(payload, O_RDONLY, target_system_id, target_comp_id);
		break;

	case kCmdCreateFile:
		errorCode = _workOpen(payload, O_CREAT | O_TRUNC | O_WRONLY, target_system_id, target_comp_id);
		break;

	case kCmdOpenFileWO:
		errorCode = _workOpen(payload, O_CREAT | O_WRONLY, target_system_id, target_comp_id);
		break;

	case kCmdReadFile:
		errorCode = _workRead(payload, session);
		break;

	case kCmdBurstReadFile:
		errorCode = _workBurst(payload, session);
		stream_send = true;
		break;

	case kCmdWriteFile:
		errorCode = _workWrite(payload, session);
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

	if (errorCode == kErrFailErrno && (payload->opcode == kCmdOpenFileRO
					   || payload->opcode == kCmdCreateFile || payload->opcode == kCmdOpenFileWO)) {
		PX4_ERR("FTP: open '%s' for %s failed: %s", _data_as_cstring(payload),
			payload->opcode == kCmdOpenFileRO ? "read" : "write", strerror(_our_errno));
	}

out:
	payload->seq_number++;

	// handle success vs. error
	if (errorCode == kErrNone) {
		payload->req_opcode = payload->opcode;
		payload->opcode = kRspAck;

	} else {
		PX4_DEBUG("errorCode: %d, errno: %d / %s", errorCode, _our_errno, strerror(_our_errno));
		payload->req_opcode = payload->opcode;
		payload->opcode = kRspNak;
		payload->size = 1;

		if (_our_errno == EEXIST) {
			errorCode = kErrFailFileExists;

		} else if (_our_errno == ENOENT && errorCode == kErrFailErrno) {
			errorCode = kErrFileNotFound;
		}

		payload->data[0] = errorCode;

		if (errorCode == kErrFailErrno) {
			payload->size = 2;
			payload->data[1] = _our_errno;
		}
	}

	_last_reply_valid = false;

	// Stream download replies are sent through mavlink stream mechanism. Unless we need to Nack.
	if (!stream_send || errorCode != kErrNone) {
		// respond to the request
		ftp_req->target_network = 0;
		ftp_req->target_component = target_comp_id;
		_reply(ftp_req, target_system_id);
	}
}

void MavlinkFTP::_ensure_buffers_exist()
{
	_last_work_buffer_access = hrt_absolute_time();
}

/// @brief Sends the specified FTP response message out through mavlink
void
MavlinkFTP::_reply(mavlink_file_transfer_protocol_t *ftp_req, uint32_t target_system_id)
{
	PayloadHeader *payload = reinterpret_cast<PayloadHeader *>(&ftp_req->payload[0]);

	// clear any not used payload data to correctly trim mavlink ftp message reply
	if (payload->size > kMaxDataLength) {
		// Should not happen: every producer bounds itself. Clamp rather than let the
		// subtraction below wrap into a memset of the whole address space.
		payload->size = kMaxDataLength;
	}

	memset(&payload->data[payload->size], 0, kMaxDataLength - payload->size);

	// keep a copy of the last sent response ((n)ack), so that if it gets lost and the GCS resends the request,
	// we can simply resend the response.
	// we only keep small responses to reduce RAM usage and avoid large memcpy's. The larger responses are all data
	// retrievals without side-effects, meaning it's ok to reexecute them if a response gets lost.
	// Skip only a no-sessions NAK so a retry can Open again. data[0] on an Open ACK is the file-size LSB, not an error code.
	if (payload->size <= sizeof(uint32_t)
	    && !(payload->opcode == kRspNak && payload->data[0] == kErrNoSessionsAvailable)) {
		_last_reply_valid = true;
		memcpy(_last_reply, ftp_req, sizeof(_last_reply));
		_last_reply_target_system = target_system_id;
	}

	PX4_DEBUG("FTP: %s seq_number: %" PRIu16, payload->opcode == kRspAck ? "Ack" : "Nak", payload->seq_number);

	// Called from the receiver thread; the per-channel mavlink_status global is
	// also written by the sending task_main thread, so serialize via lock_send().
	_mavlink.lock_send();
	mavlink_msg_send_struct_target(_mavlink.get_channel(), FILE_TRANSFER_PROTOCOL, ftp_req, target_system_id);
	_mavlink.unlock_send();
}
void MavlinkFTP::_constructPath(char *dst, int dst_len, const char *path) const
{
	// MAVLink FTP virtual directory: paths starting with "@MAV_LOG"
	// are remapped to the flight-stack log root directory.
	const char *p = path;

	if (strncmp(p, _mav_log_prefix, _mav_log_prefix_len) == 0
	    && (p[_mav_log_prefix_len] == '\0' || p[_mav_log_prefix_len] == '/')) {
		strncpy(dst, _mav_log_dir, dst_len);
		dst[dst_len - 1] = '\0';
		int used = strlen(dst);
		strncpy(dst + used, p + _mav_log_prefix_len, dst_len - used);
		dst[dst_len - 1] = '\0';
		return;
	}

	strncpy(dst, _root_dir, dst_len);
	int root_dir_len = _root_dir_len;

	// If neither the root ends nor the given path starts with a '/', add a separating '/' in between,
	// which also makes the path absolute with an empty root, for file systems without a current directory
	if ((dst[0] == '\0' || dst[strlen(dst) - 1] != '/') && path[0] != '/') {
		strncat(dst, "/", dst_len);
		++root_dir_len;
	}

	strncpy(dst + root_dir_len, path, dst_len - root_dir_len);
	// Ensure termination
	dst[dst_len - 1] = '\0';
}

/// @brief stat()s an entry of the directory in _work_buffer1, leaving size and mtime untouched on failure
void
MavlinkFTP::_statDirent(const char *name, uint32_t &size, uint32_t &mtime)
{
	// No double slash in the root directory, which not all file systems resolve
	const size_t dir_len = strlen(_work_buffer1);
	const char *separator = (dir_len > 0 && _work_buffer1[dir_len - 1] == '/') ? "" : "/";
	int ret = snprintf(_work_buffer2, _work_buffer2_len, "%s%s%s", _work_buffer1, separator, name);

	if ((ret > 0) && (ret < _work_buffer2_len)) {
		struct stat st;

		if (stat(_work_buffer2, &st) == 0) {
			size = st.st_size;
			mtime = st.st_mtime;
		}
	}
}

/// @brief Responds to a List command
MavlinkFTP::ErrorCode
MavlinkFTP::_workList(PayloadHeader *payload, bool include_time)
{
	_constructPath(_work_buffer1, _work_buffer1_len, _data_as_cstring(payload));

	if (!_validatePath(_work_buffer1)) {
		return kErrFailFileProtected;
	}

	ErrorCode errorCode = kErrNone;
	unsigned offset = 0;

	PX4_DEBUG("opendir: %s", _work_buffer1);

	DIR *dp = opendir(_work_buffer1);

	if (dp == nullptr) {
		_our_errno = errno;
		PX4_DEBUG("Dir open failed %s: %s", _work_buffer1, strerror(_our_errno));
		return kErrFileNotFound;
	}

	PX4_DEBUG("FTP: list %s offset %" PRIu32, _work_buffer1, payload->offset);

	struct dirent *result = nullptr;

	// move to the requested offset
	int requested_offset = payload->offset;

	PX4_DEBUG("readdir with offset: %d", requested_offset);

	while (requested_offset-- > 0 && readdir(dp)) {}

	for (;;) {
		errno = 0;
		result = readdir(dp);

		// read the directory entry
		if (result == nullptr) {
			_our_errno = errno;

			if (_our_errno) {
				PX4_WARN("readdir failed: %s", strerror(_our_errno));

				// Room for the identifier and the null terminator, as in the entry loop below
				if ((offset + 2) <= kMaxDataLength) {
					payload->data[offset++] = kDirentSkip;
					*((char *)&payload->data[offset]) = '\0';
					offset++;
				}

				errorCode = kErrFailErrno;

			} else if (offset == 0) {
				// User is requesting subsequent dir entries but there were none. This means the user asked
				// to seek past EOF. This can happen with `payload->offset == 0` if the directory is empty.
				errorCode = kErrEOF;
			}

			// Otherwise we are just at the last directory entry, so we leave the errorCode at kErrorNone to signal that
			break;
		}

		uint32_t fileSize = 0;
		uint32_t fileTime = 0;	// seconds since the UNIX epoch, 0 if unknown
		char direntType;

		// Determine the directory entry type
		switch (result->d_type) {
#ifdef __PX4_NUTTX

		case DTYPE_FILE: {
#else

		case DT_REG: {
#endif
				// For files we get the file size as well
				direntType = kDirentFile;
				_statDirent(result->d_name, fileSize, fileTime);
				break;
			}

#ifdef __PX4_NUTTX

		case DTYPE_DIRECTORY:
#else
		case DT_DIR:
#endif
			if (strcmp(result->d_name, ".") == 0 || strcmp(result->d_name, "..") == 0) {
				// Don't bother sending these back
				direntType = kDirentSkip;

			} else {
				direntType = kDirentDir;

				if (include_time) {
					// Directories have no meaningful size, but do have a modification time
					_statDirent(result->d_name, fileSize, fileTime);
					fileSize = 0;
				}
			}

			break;

		default:
			// We only send back file and diretory entries, skip everything else
			direntType = kDirentSkip;
		}

		if (direntType == kDirentSkip) {
			// Skip send only dirent identifier
			_work_buffer2[0] = '\0';

		} else if (include_time) {
			// ListDirectoryWithTime: every entry is <name>\t<size>\t<mtime>, directories with size 0
			int ret = snprintf(_work_buffer2, _work_buffer2_len, "%s\t%" PRIu32 "\t%" PRIu32, result->d_name, fileSize,
					   fileTime);

			if (!((ret > 0) && (ret < _work_buffer2_len))) {
				_work_buffer2[_work_buffer2_len - 1] = '\0';
			}

		} else if (direntType == kDirentFile) {
			// ListDirectory: files send <name>\t<size>
			int ret = snprintf(_work_buffer2, _work_buffer2_len, "%s\t%" PRIu32, result->d_name, fileSize);

			if (!((ret > 0) && (ret < _work_buffer2_len))) {
				_work_buffer2[_work_buffer2_len - 1] = '\0';
			}

		} else {
			// ListDirectory: directories send only the name, existing clients take the whole string as the name
			strncpy(_work_buffer2, result->d_name, _work_buffer2_len);
			_work_buffer2[_work_buffer2_len - 1] = '\0';
		}

		size_t nameLen = strlen(_work_buffer2);

		// Do we have room for the name, the one char directory identifier and the null terminator?
		if ((offset + nameLen + 2) > kMaxDataLength) {
			break;
		}

		// Move the data into the buffer
		payload->data[offset++] = direntType;
		strcpy((char *)&payload->data[offset], _work_buffer2);
		PX4_DEBUG("FTP: list %s %s", _work_buffer1, (char *)&payload->data[offset - 1]);
		offset += nameLen + 1;
	}

	closedir(dp);
	payload->size = offset;

	return errorCode;
}

/// @brief Responds to an Open command
MavlinkFTP::ErrorCode
MavlinkFTP::_workOpen(PayloadHeader *payload, int oflag, uint32_t owner_system_id, uint8_t owner_component_id)
{
	// Opening never closes another session: a client that lost track of one
	// gets it back from the idle close, everyone else keeps theirs.
	SessionInfo *session = _free_session();

	if (session == nullptr) {
		return kErrNoSessionsAvailable;
	}

	const char *path = _data_as_cstring(payload);
	const bool for_write = (oflag & O_ACCMODE) != O_RDONLY;
	uint32_t file_size = 0;

	if (ParamPckFile::is_param_path(path)) {
		if (for_write) {
			if (!session->param.open_write()) {
				_our_errno = EINVAL;
				return kErrFailErrno;
			}

		} else if (!session->param.open(path, kMaxDataLength)) {
			_our_errno = EINVAL;
			return kErrFailErrno;

		} else {
			file_size = session->param.size();
		}

	} else {
		_constructPath(_work_buffer1, _work_buffer1_len, path);

		if (!_validatePath(_work_buffer1)) {
			return kErrFailFileProtected;
		}

		// CreateFile and OpenFileWO create or truncate the file as part of the open, so the
		// effect lands before any write arrives and has to be authorized here. Test the
		// access mode rather than the individual flags: on NuttX O_RDONLY is a bit and
		// O_RDWR is O_RDONLY | O_WRONLY, so a read-only open would match O_RDWR.
		if (for_write && !_validatePathIsWritable(_work_buffer1)) {
			return kErrFailFileProtected;
		}

		PX4_DEBUG("FTP: open '%s'", _work_buffer1);

		struct stat st;

		if (stat(_work_buffer1, &st) == 0) {
			file_size = st.st_size;

		} else if (!for_write) {
			_our_errno = errno;
			return kErrFailErrno;
		}

		// Set mode to 666 incase oflag has O_CREAT
		int fd = ::open(_work_buffer1, oflag, PX4_O_MODE_666);

		if (fd < 0) {
			_our_errno = errno;
			return kErrFailErrno;
		}

		session->fd = fd;
	}

	session->id = _allocate_session_id();
	session->owner_system_id = owner_system_id;
	session->owner_component_id = owner_component_id;
	session->file_size = file_size;
	session->stream_download = false;

	payload->session = session->id;
	payload->size = sizeof(uint32_t);
	std::memcpy(payload->data, &file_size, payload->size);

	return kErrNone;
}

/// @brief Responds to a Read command
MavlinkFTP::ErrorCode
MavlinkFTP::_workRead(PayloadHeader *payload, SessionInfo *session)
{
	if (session == nullptr) {
		return kErrInvalidSession;
	}

	PX4_DEBUG("FTP: read offset:%" PRIu32, payload->offset);

	// Same clamp as a burst: a recovery read that fragments defeats the point
	// of keeping the burst inside one radio packet. Only behind a radio, and
	// the size field says what was actually read.
	const int bytes_read = _read_session(*session, payload->offset, payload->data,
					     radio_clamped_length(payload->size));

	if (bytes_read < 0) {
		PX4_ERR("read fail: %s", strerror(_our_errno));
		return kErrFailErrno;
	}

	if (bytes_read == 0) {
		PX4_WARN("request past EOF");
		return kErrEOF;
	}

	payload->size = bytes_read;

	return kErrNone;
}

/// @brief Responds to a Stream command
MavlinkFTP::ErrorCode
MavlinkFTP::_workBurst(PayloadHeader *payload, SessionInfo *session)
{
	if (session == nullptr) {
		PX4_DEBUG("_workBurst: no session or no fd");
		return kErrInvalidSession;
	}

	PX4_DEBUG("FTP: burst offset:%" PRIu32, payload->offset);

	// Setup for streaming sends. Only a burst on an idle link starts with a
	// packet straight away; joining the others must not reset their pacing.
	if (!burst_active()) {
		_last_burst_send = 0;
	}

	session->stream_download = true;
	session->stream_offset = payload->offset;
	session->stream_chunk_transmitted = 0;
	session->stream_size = payload->size;
	session->stream_seq_number = payload->seq_number + 1;

	return kErrNone;
}

/// @brief Responds to a Write command
MavlinkFTP::ErrorCode
MavlinkFTP::_workWrite(PayloadHeader *payload, SessionInfo *session)
{
	if (session == nullptr) {
		PX4_DEBUG("_workWrite: no session or no fd");
		return kErrInvalidSession;
	}

	if (session->param.is_open()) {
		if (!session->param.is_writing()) {
			return kErrFailFileProtected;
		}

		const int bytes_written = session->param.write(payload->offset, payload->data, payload->size);

		if (bytes_written < 0) {
			_our_errno = EINVAL;
			return kErrFailErrno;
		}

		payload->size = sizeof(uint32_t);
		std::memcpy(payload->data, &bytes_written, payload->size);
		return kErrNone;
	}

	// The path is authorized in _workOpen(), which is where the descriptor this writes to
	// was bound. Re-checking here would validate _work_buffer1, a scratch buffer any
	// intervening request overwrites, rather than the path behind session->fd.

	if (lseek(session->fd, payload->offset, SEEK_SET) < 0) {
		_our_errno = errno;
		PX4_ERR("seek fail: %s", strerror(_our_errno));
		return kErrFailErrno;
	}

	PX4_DEBUG("write %d bytes", payload->size);
	int bytes_written = ::write(session->fd, &payload->data[0], payload->size);

	if (bytes_written < 0) {
		// Negative return indicates error other than eof
		_our_errno = errno;
		PX4_ERR("write fail %d, %s", bytes_written, strerror(_our_errno));
		return kErrFailErrno;
	}

	payload->size = sizeof(uint32_t);
	std::memcpy(payload->data, &bytes_written, payload->size);

	return kErrNone;
}

/// @brief Responds to a RemoveFile command
MavlinkFTP::ErrorCode
MavlinkFTP::_workRemoveFile(PayloadHeader *payload)
{
	_constructPath(_work_buffer1, _work_buffer1_len, _data_as_cstring(payload));

	if (!_validatePath(_work_buffer1) || !_validatePathIsWritable(_work_buffer1)) {
		return kErrFailFileProtected;
	}

	PX4_DEBUG("unlink %s", _work_buffer1);

	if (unlink(_work_buffer1) == 0) {
		payload->size = 0;
		return kErrNone;

	} else {
		_our_errno = errno;
		PX4_ERR("unlink failed: %s", strerror(_our_errno));
		return kErrFailErrno;
	}
}

/// @brief Responds to a TruncateFile command
MavlinkFTP::ErrorCode
MavlinkFTP::_workTruncateFile(PayloadHeader *payload)
{
	_constructPath(_work_buffer1, _work_buffer1_len, _data_as_cstring(payload));
	payload->size = 0;

	if (!_validatePath(_work_buffer1) || !_validatePathIsWritable(_work_buffer1)) {
		return kErrFailFileProtected;
	}

#ifdef __PX4_NUTTX

	// emulate truncate(_work_buffer1, payload->offset) by
	// copying to temp and overwrite with O_TRUNC flag (NuttX does not support truncate()).
	const char temp_file[] = PX4_STORAGEDIR"/.trunc.tmp";

	struct stat st;

	PX4_DEBUG("stat: %s", _work_buffer1);

	if (stat(_work_buffer1, &st) != 0) {
		_our_errno = errno;
		PX4_ERR("stat failed: %s", strerror(_our_errno));
		return kErrFailErrno;
	}

	if (!S_ISREG(st.st_mode)) {
		_our_errno = EISDIR;
		return kErrFailErrno;
	}

	// check perms allow us to write (not romfs)
	if (!(st.st_mode & (S_IWUSR | S_IWGRP | S_IWOTH))) {
		_our_errno = EROFS;
		return kErrFailErrno;
	}

	if (payload->offset == (unsigned)st.st_size) {
		// nothing to do
		return kErrNone;

	} else if (payload->offset == 0) {
		// 1: truncate all data
		int fd = ::open(_work_buffer1, O_TRUNC | O_WRONLY);

		if (fd < 0) {
			return kErrFailErrno;
		}

		::close(fd);
		return kErrNone;

	} else if (payload->offset > (unsigned)st.st_size) {
		// 2: extend file

		PX4_DEBUG("extend file: %s", _work_buffer1);

		int fd = ::open(_work_buffer1, O_WRONLY);

		if (fd < 0) {
			_our_errno = errno;
			PX4_ERR("open failed: %s", strerror(_our_errno));
			return kErrFailErrno;
		}

		if (lseek(fd, payload->offset - 1, SEEK_SET) < 0) {
			_our_errno = errno;
			PX4_ERR("seek failed: %s", strerror(_our_errno));
			::close(fd);
			return kErrFailErrno;
		}

		PX4_DEBUG("write 1");
		bool ok = 1 == ::write(fd, "", 1);

		if (!ok) {
			_our_errno = errno;
			PX4_ERR("write 1 failed: %s", strerror(_our_errno));
		}

		::close(fd);

		return (ok) ? kErrNone : kErrFailErrno;

	} else {
		// 3: truncate
		PX4_DEBUG("truncate file %s", _work_buffer1);

		if (_copy_file(_work_buffer1, temp_file, payload->offset) != 0) {
			return kErrFailErrno;
		}

		if (_copy_file(temp_file, _work_buffer1, payload->offset) != 0) {
			return kErrFailErrno;
		}

		if (::unlink(temp_file) != 0) {
			_our_errno = errno;
			PX4_ERR("unlink failed: %s", strerror(_our_errno));
			return kErrFailErrno;
		}

		return kErrNone;
	}

#else
	int ret = truncate(_work_buffer1, payload->offset);

	if (ret == 0) {
		return kErrNone;
	}

	return kErrFailErrno;
#endif /* __PX4_NUTTX */
}

/// @brief Responds to a Terminate command
MavlinkFTP::ErrorCode
MavlinkFTP::_workTerminate(PayloadHeader *payload, SessionInfo *session)
{
	if (session == nullptr) {
		return kErrInvalidSession;
	}

	PX4_DEBUG("work terminate: close");

	const bool write_ok = !session->param.is_writing() || session->param.finish_write();
	_close_session(*session);

	if (!write_ok) {
		_our_errno = EINVAL;
		return kErrFailErrno;
	}

	payload->size = 0;

	return kErrNone;
}

/// @brief Responds to a Reset command
MavlinkFTP::ErrorCode
MavlinkFTP::_workReset(PayloadHeader *payload)
{
	// Every session of every client, as the protocol specifies: having a
	// session of its own does not protect a client from another one resetting.
	PX4_DEBUG("work reset: close");
	_close_all_sessions();

	payload->size = 0;

	return kErrNone;
}

/// @brief Responds to a Rename command
MavlinkFTP::ErrorCode
MavlinkFTP::_workRename(PayloadHeader *payload)
{
	char *ptr = _data_as_cstring(payload);
	size_t oldpath_sz = strlen(ptr);

	if (oldpath_sz + 2 >= payload->size) {
		// no newpath
		errno = EINVAL;
		return kErrFailErrno;
	}

	_constructPath(_work_buffer1, _work_buffer1_len, ptr);
	_constructPath(_work_buffer2, _work_buffer2_len, ptr + oldpath_sz + 1);

	if (!_validatePath(_work_buffer1) || !_validatePath(_work_buffer2) || !_validatePathIsWritable(_work_buffer2)) {
		return kErrFailFileProtected;
	}

	PX4_DEBUG("rename from %s to %s", _work_buffer1, _work_buffer2);

	if (rename(_work_buffer1, _work_buffer2) == 0) {
		payload->size = 0;
		return kErrNone;

	} else {
		_our_errno = errno;
		PX4_ERR("rename failed: %d %s", _our_errno, strerror(_our_errno));
		return kErrFailErrno;
	}
}

/// @brief Responds to a RemoveDirectory command
MavlinkFTP::ErrorCode
MavlinkFTP::_workRemoveDirectory(PayloadHeader *payload)
{
	_constructPath(_work_buffer1, _work_buffer1_len, _data_as_cstring(payload));

	if (!_validatePath(_work_buffer1) || !_validatePathIsWritable(_work_buffer1)) {
		return kErrFailFileProtected;
	}

	PX4_DEBUG("remove dir %s", _work_buffer1);

	if (rmdir(_work_buffer1) == 0) {
		payload->size = 0;
		return kErrNone;

	} else {
		_our_errno = errno;
		PX4_DEBUG("remove dir failed: %d %s", _our_errno, strerror(_our_errno));
		return kErrFailErrno;
	}
}

/// @brief Responds to a CreateDirectory command
MavlinkFTP::ErrorCode
MavlinkFTP::_workCreateDirectory(PayloadHeader *payload)
{
	_constructPath(_work_buffer1, _work_buffer1_len, _data_as_cstring(payload));

	if (!_validatePath(_work_buffer1) || !_validatePathIsWritable(_work_buffer1)) {
		return kErrFailFileProtected;
	}

	PX4_DEBUG("create dir %s", _work_buffer1);

	if (mkdir(_work_buffer1, S_IRWXU | S_IRWXG | S_IRWXO) == 0) {
		payload->size = 0;
		return kErrNone;

	} else {
		_our_errno = errno;
		PX4_ERR("create dir failed: %s", strerror(_our_errno));
		return kErrFailErrno;
	}
}

/// @brief Responds to a CalcFileCRC32 command
MavlinkFTP::ErrorCode
MavlinkFTP::_workCalcFileCRC32(PayloadHeader *payload)
{
	uint32_t checksum = 0;
	ssize_t bytes_read;
	_constructPath(_work_buffer2, _work_buffer2_len, _data_as_cstring(payload));

	if (!_validatePath(_work_buffer2)) {
		return kErrFailFileProtected;
	}

	int fd = ::open(_work_buffer2, O_RDONLY);

	if (fd < 0) {
		return kErrFailErrno;
	}

	do {
		bytes_read = ::read(fd, _work_buffer2, _work_buffer2_len);

		if (bytes_read < 0) {
			_our_errno = errno;
			::close(fd);
			return kErrFailErrno;
		}

		checksum = crc32part((uint8_t *)_work_buffer2, bytes_read, checksum);
	} while (bytes_read == _work_buffer2_len);

	::close(fd);

	payload->size = sizeof(uint32_t);
	std::memcpy(payload->data, &checksum, payload->size);
	return kErrNone;
}

/// @brief Guarantees that the payload data is null terminated.
///     @return Returns a pointer to the payload data as a char *
char *
MavlinkFTP::_data_as_cstring(PayloadHeader *payload)
{
	// guarantee nul termination
	if (payload->size < kMaxDataLength) {
		payload->data[payload->size] = '\0';

	} else {
		payload->data[kMaxDataLength - 1] = '\0';
	}

	// and return data
	return (char *) & (payload->data[0]);
}

/// @brief Copy file (with limited space)
int
MavlinkFTP::_copy_file(const char *src_path, const char *dst_path, size_t length)
{
	PX4_DEBUG("copy file from %s to %s", src_path, dst_path);

	int src_fd = -1, dst_fd = -1;

	src_fd = ::open(src_path, O_RDONLY);

	if (src_fd < 0) {
		return -1;
	}

	dst_fd = ::open(dst_path, O_CREAT | O_TRUNC | O_WRONLY
// POSIX requires the permissions to be supplied if O_CREAT passed
#ifdef __PX4_POSIX
			, 0666
#endif
		       );

	if (dst_fd < 0) {
		_our_errno = errno;
		::close(src_fd);
		return -1;
	}

	while (length > 0) {
		ssize_t bytes_read, bytes_written;
		size_t blen = (length > _work_buffer2_len) ? _work_buffer2_len : length;

		bytes_read = ::read(src_fd, _work_buffer2, blen);

		if (bytes_read == 0) {
			// EOF
			break;

		} else if (bytes_read < 0) {
			_our_errno = errno;
			PX4_ERR("cp: read");
			break;
		}

		bytes_written = ::write(dst_fd, _work_buffer2, bytes_read);

		if (bytes_written != bytes_read) {
			_our_errno = errno;
			PX4_ERR("cp: short write");
			break;
		}

		length -= bytes_written;
	}

	::close(src_fd);
	::close(dst_fd);

	return (length > 0) ? -1 : 0;
}

MavlinkFTP::SessionInfo *
MavlinkFTP::_next_burst_session()
{
	// Round robin, one packet each: concurrent bursts split the one budget of
	// the link between them rather than each getting its own.
	for (unsigned i = 0; i < kMaxSessions; i++) {
		SessionInfo &session = _sessions[(_burst_cursor + i) % kMaxSessions];

		if (session.stream_download) {
			return &session;
		}
	}

	return nullptr;
}

void MavlinkFTP::send()
{
	if (hrt_elapsed_time(&_last_work_buffer_access) > 30_s) {
		for (SessionInfo &session : _sessions) {
			if (session.is_open()) {
				_close_session(session);
				PX4_WARN("Session was closed without activity");
			}
		}
	}

	// Anything to stream?
	SessionInfo *stream = _next_burst_session();

	if (stream == nullptr) {
		return;
	}

	// Skip send if not enough room
	unsigned max_bytes_to_send = _mavlink.get_free_tx_buf();
	PX4_DEBUG("MavlinkFTP::send max_bytes_to_send(%u) get_free_tx_buf(%u)", max_bytes_to_send, _mavlink.get_free_tx_buf());

	if (max_bytes_to_send < get_size()) {
		return;
	}

	// Pace the burst against the link budget. Otherwise the only bound is the
	// local UART buffer, which knows nothing about what the radio behind it can
	// carry, and we hand it several times its capacity.
	const int datarate = _mavlink.radio_status_available() ? _mavlink.get_data_rate() : 0;

	if (datarate > 0) {
		const hrt_abstime now = hrt_absolute_time();

		// The first packet of a burst goes out immediately, after that wait
		// until the link has earned one.
		if (_last_burst_send != 0) {
			// Scaled by the radio's own txbuf feedback, so the burst finds
			// whatever the link can actually carry: that differs by more than
			// two to one depending on whether the radio has error correction
			// enabled, which we have no way of knowing here.
			//
			// Deliberately not get_rate_mult(), which also folds in the budget
			// split this share has already been subtracted from - applying that
			// would count the reduction twice and starve the burst into the
			// client's inter-packet timeout.
			const float rate = (float)datarate * _mavlink.bulk_bandwidth_share() * _mavlink.radio_status_mult();
			const float credit = (float)(now - _last_burst_send) * rate / 1e6f;

			if (credit < (float)_burst_wire_size(*stream)) {
				return;
			}
		}

		// Exactly one packet per call, never a batch. Otherwise the loop below
		// fills the whole UART buffer, a dozen packets back to back. A SiK
		// radio only fits about three per transmit window and drops the rest.
		// Credit is not banked either, so a pause cannot be spent at once.
		_last_burst_send = now;
		max_bytes_to_send = _burst_wire_size(*stream);
	}

	// Send stream packets until buffer is full

	bool more_data;

	do {
		more_data = false;
		stream = _next_burst_session();

		if (stream == nullptr) {
			break;
		}

		ErrorCode error_code = kErrNone;

		mavlink_file_transfer_protocol_t ftp_msg;
		PayloadHeader *payload = reinterpret_cast<PayloadHeader *>(&ftp_msg.payload[0]);

		payload->seq_number = stream->stream_seq_number;
		payload->session = stream->id;
		payload->opcode = kRspAck;
		payload->req_opcode = kCmdBurstReadFile;
		payload->offset = stream->stream_offset;
		payload->burst_complete = false;
		stream->stream_seq_number++;

		PX4_DEBUG("stream send: offset %" PRIu32, stream->stream_offset);

		const int bytes_read = _read_session(*stream, payload->offset, &payload->data[0], _burst_data_length(*stream));

		if (bytes_read < 0) {
			error_code = kErrFailErrno;
			PX4_WARN("stream download: read fail: %s", strerror(_our_errno));

		} else if (bytes_read == 0) {
			error_code = kErrEOF;
			PX4_DEBUG("stream download: sending Nak EOF");

		} else {
			payload->size = bytes_read;
			stream->stream_offset += bytes_read;
			stream->stream_chunk_transmitted += bytes_read;
		}

		if (error_code != kErrNone) {
			payload->opcode = kRspNak;
			// The burst ends here, whether the file did or something went wrong.
			// ArduPilot marks this packet the same way, and a client that waits
			// for the end of the burst would otherwise keep waiting.
			payload->burst_complete = true;
			payload->size = 1;
			uint8_t *pData = &payload->data[0];
			*pData = error_code; // Straight reference to data[0] is causing bogus gcc array subscript error

			if (error_code == kErrFailErrno) {
				payload->size = 2;
				payload->data[1] = _our_errno;
			}

			stream->stream_download = false;

		} else {
			if (max_bytes_to_send < (get_size() * 2)) {
				more_data = false;

				/* perform transfers in 35K chunks - this is determined empirical */
				if (stream->stream_chunk_transmitted > 35000) {
					payload->burst_complete = true;
					stream->stream_download = false;
					stream->stream_chunk_transmitted = 0;
				}

			} else {
				more_data = true;
				max_bytes_to_send -= get_size();
			}
		}

		ftp_msg.target_network = 0;
		ftp_msg.target_component = stream->owner_component_id;
		_reply(&ftp_msg, stream->owner_system_id);
		_burst_cursor = (stream - _sessions + 1) % kMaxSessions;

		// Streaming a burst is activity: without this the session is torn down
		// from under a burst that runs longer than the inactivity timeout, which
		// on a slow link a single chunk easily does.
		_last_work_buffer_access = hrt_absolute_time();
	} while (more_data);
}

/**
 * Reject paths containing ".." components to prevent directory traversal
 * outside of _root_dir. Walks the path checking each component between
 * '/' separators. A component of exactly ".." (followed by '/' or end
 * of string) is rejected.
 *
 * Examples:
 *   "/fs/microsd/logs"         -> allowed
 *   "/fs/microsd/../etc"       -> rejected
 *   "../../tmp/pwned"          -> rejected
 *   "/fs/microsd/logs/.."      -> rejected
 *   "/fs/microsd/..hidden"     -> allowed (not a ".." component)
 */
bool MavlinkFTP::_validatePath(const char *path)
{
	const char *p = path;

	while (*p != '\0') {
		if ((p == path || *(p - 1) == '/') && p[0] == '.' && p[1] == '.'
		    && (p[2] == '/' || p[2] == '\0')) {
			PX4_ERR("FTP: rejecting path traversal in %s", path);
			return false;
		}

		p++;
	}

	return true;
}

bool MavlinkFTP::_validatePathIsWritable(const char *path)
{
	// The root can expose read-only data next to the storage directory, so confine writes
	// to storage. On NuttX that keeps them off the in-RAM system paths; on POSIX it keeps
	// them out of the ROMFS.
	// Ideally we'd canonicalize the path (with 'realpath'), but it might not exist, so realpath() would fail.
	// The next simpler thing is to check there's no reference to a parent dir.
	static constexpr const char storage_prefix[] = PX4_STORAGEDIR "/";

	if (strncmp(path, storage_prefix, sizeof(storage_prefix) - 1) != 0 || strstr(path, "/../") != nullptr) {
		PX4_ERR("Disallowing write to %s", path);
		return false;
	}

	return true;
}
