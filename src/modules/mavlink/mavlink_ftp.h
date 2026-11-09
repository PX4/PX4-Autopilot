/****************************************************************************
 *
 *   Copyright (c) 2014, 2015 PX4 Development Team. All rights reserved.
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

/// @file mavlink_ftp.h
///     @author px4dev, Don Gagne <don@thegagnes.com>

#include <dirent.h>
#include <sys/queue.h>

#include <px4_platform_common/defines.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include "mavlink_bridge_header.h"
#include "mavlink_ftp_param.h"

class Mavlink;

/// MAVLink remote file server. Support FTP like commands using MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL message.
class MavlinkFTP
{
public:
	MavlinkFTP(Mavlink &mavlink);
	~MavlinkFTP();

	/**
	 * Handle sending of messages. Call this regularly at a fixed frequency.
	 * @param t current time
	 */
	void send();

	/// Handle possible FTP message
	void handle_message(const mavlink_message_t *msg);

	/// True while a burst download is streaming out.
	bool burst_active() const;

	/// Framing overhead of one FTP reply, signature block included.
	unsigned packet_overhead() const;

	/// Reduce a requested payload to what the link carries in one piece.
	unsigned radio_clamped_length(unsigned requested) const;

	/// @brief This is the payload which is in mavlink_file_transfer_protocol_t.payload.
	/// This needs to be packed, because it's typecasted from mavlink_file_transfer_protocol_t.payload, which starts
	/// at a 3 byte offset, causing an unaligned access to seq_number and offset
	struct __attribute__((__packed__)) PayloadHeader {
		uint16_t	seq_number;	///< sequence number for message
		uint8_t		session;	///< Session id for read and write commands
		uint8_t		opcode;		///< Command opcode
		uint8_t		size;		///< Size of data
		uint8_t		req_opcode;	///< Request opcode returned in kRspAck, kRspNak message
		uint8_t		burst_complete; ///< Only used if req_opcode=kCmdBurstReadFile - 1: set of burst packets complete, 0: More burst packets coming.
		uint8_t		padding;        ///< 32 bit aligment padding
		uint32_t	offset;		///< Offsets for List and Read commands
		uint8_t		data[];		///< command data, varies by Opcode
	};

	/// @brief Command opcodes
	enum Opcode : uint8_t {
		kCmdNone,		///< ignored, always acked
		kCmdTerminateSession,	///< Terminates open Read session
		kCmdResetSessions,	///< Terminates all open Read sessions
		kCmdListDirectory,	///< List files in <path> from <offset>
		kCmdOpenFileRO,		///< Opens file at <path> for reading, returns <session>
		kCmdReadFile,		///< Reads <size> bytes from <offset> in <session>
		kCmdCreateFile,		///< Creates file at <path> for writing, returns <session>
		kCmdWriteFile,		///< Writes <size> bytes to <offset> in <session>
		kCmdRemoveFile,		///< Remove file at <path>
		kCmdCreateDirectory,	///< Creates directory at <path>
		kCmdRemoveDirectory,	///< Removes Directory at <path>, must be empty
		kCmdOpenFileWO,		///< Opens file at <path> for writing, returns <session>
		kCmdTruncateFile,	///< Truncate file at <path> to <offset> length
		kCmdRename,		///< Rename <path1> to <path2>
		kCmdCalcFileCRC32,	///< Calculate CRC32 for file at <path>
		kCmdBurstReadFile,	///< Burst download session file
		kCmdListDirectoryWithTime,	///< List files in <path> from <offset>, including last-modification time

		kRspAck = 128,		///< Ack response
		kRspNak			///< Nak response
	};

	/// @brief Error codes returned in Nak response PayloadHeader.data[0].
	enum ErrorCode : uint8_t {
		kErrNone,
		kErrFail,			///< Unknown failure
		kErrFailErrno,			///< Command failed, errno sent back in PayloadHeader.data[1]
		kErrInvalidDataSize,		///< PayloadHeader.size is invalid
		kErrInvalidSession,		///< Session is not currently open
		kErrNoSessionsAvailable,	///< All available Sessions in use
		kErrEOF,			///< Offset past end of file for List and Read commands
		kErrUnknownCommand,		///< Unknown command opcode
		kErrFailFileExists,		///< File/directory exists already
		kErrFailFileProtected,		///< File/directory is write protected
		kErrFileNotFound                ///< File/directory not found
	};

	unsigned get_size();

	/**
	 * Sessions open at the same time on one MAVLink instance.
	 *
	 * Each one embeds a ParamPckFile, which holds more than 512 bytes for
	 * @PARAM/param.pck whether or not it is in use, and the server exists
	 * once per MAVLink instance. Three cover a GCS and a companion each
	 * transferring a file next to a parameter download or upload.
	 */
	static constexpr unsigned kMaxSessions = 3;

private:
	struct SessionInfo {
		int		fd{-1};
		uint8_t		id{0};			///< session field on the wire
		uint32_t	owner_system_id{0};	///< only the client that opened it may use it
		uint8_t		owner_component_id{0};
		uint32_t	file_size{0};
		bool		stream_download{false};
		uint32_t	stream_offset{0};
		uint16_t	stream_seq_number{0};
		unsigned	stream_chunk_transmitted{0};
		uint8_t		stream_size{0};		///< payload size the client asked for, 0 = as much as fits
		ParamPckFile	param;			///< virtual parameter file, open instead of fd

		bool is_open() const { return (fd >= 0) || param.is_open(); }
	};

	char		*_data_as_cstring(PayloadHeader *payload);

	void		_process_request(mavlink_file_transfer_protocol_t *ftp_req, uint32_t target_system_id, uint8_t target_comp_id);
	void		_reply(mavlink_file_transfer_protocol_t *ftp_req, uint32_t target_system_id);
	int		_copy_file(const char *src_path, const char *dst_path, size_t length);

	ErrorCode	_workList(PayloadHeader *payload, bool include_time = false);
	void		_statDirent(const char *name, uint32_t &size, uint32_t &mtime);
	ErrorCode	_workOpen(PayloadHeader *payload, int oflag, uint32_t owner_system_id, uint8_t owner_component_id);
	ErrorCode	_workRead(PayloadHeader *payload, SessionInfo *session);
	ErrorCode	_workBurst(PayloadHeader *payload, SessionInfo *session);
	ErrorCode	_workWrite(PayloadHeader *payload, SessionInfo *session);
	ErrorCode	_workTerminate(PayloadHeader *payload, SessionInfo *session);
	ErrorCode	_workReset(PayloadHeader *payload);
	ErrorCode	_workRemoveDirectory(PayloadHeader *payload);
	ErrorCode	_workCreateDirectory(PayloadHeader *payload);
	ErrorCode	_workRemoveFile(PayloadHeader *payload);
	ErrorCode	_workTruncateFile(PayloadHeader *payload);
	ErrorCode	_workRename(PayloadHeader *payload);
	ErrorCode	_workCalcFileCRC32(PayloadHeader *payload);

	uint32_t _getServerSystemId(void);
	uint8_t _getServerComponentId(void);
	uint8_t _getServerChannel(void);

	/// The open session with this id, if it belongs to the requesting client.
	SessionInfo *_find_session(uint8_t id, uint32_t system_id, uint8_t component_id);

	/// A closed slot to open a session in, nullptr if all are in use.
	SessionInfo *_free_session();

	/// The next id no open session uses, so a stale request for a closed session does not reach a new one.
	uint8_t _allocate_session_id();

	void _close_session(SessionInfo &session);
	void _close_all_sessions();

	/// Read from an open session. Returns bytes read, 0 at EOF, -1 on error with _our_errno set.
	int _read_session(SessionInfo &session, uint32_t offset, uint8_t *buf, uint16_t count);

	/// Burst payload to use for this session on this link.
	unsigned _burst_data_length(const SessionInfo &session) const;

	/// On-wire size of one burst packet of this session on this link.
	unsigned _burst_wire_size(const SessionInfo &session) const;

	/// The session the next burst packet comes from, nullptr if none is streaming.
	SessionInfo *_next_burst_session();

	/**
	 * Construct local path by appending `path` to `_root_dir` and storing the result in `dst`.
	 */
	void _constructPath(char *dst, int dst_len, const char *path) const;

	bool _validatePath(const char *path);
	bool _validatePathIsWritable(const char *path);

	/// Stamp last request time; drives the 30 s idle session close.
	void _ensure_buffers_exist();

	static const char	kDirentFile = 'F';	///< Identifies File returned from List command
	static const char	kDirentDir = 'D';	///< Identifies Directory returned from List command
	static const char	kDirentSkip = 'S';	///< Identifies Skipped entry from List command

	/// @brief Maximum data size in RequestHeader::data
	static const uint8_t	kMaxDataLength = MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN - sizeof(PayloadHeader);

	/**
	 * Largest MAVLink packet we will put on a link behind a telemetry radio.
	 *
	 * A SiK radio splits any packet larger than its max_data_packet_length
	 * across several radio packets. It reassembles them correctly, but the
	 * split costs air time and lowers the rate the link can carry, which is
	 * what pushes a burst over the edge. That limit is 250 bytes without error
	 * correction and 118 with it, and we cannot tell which from here, so take
	 * the smaller. The payload that fits is derived from this, since the
	 * framing overhead depends on whether signing is active.
	 */
	static const unsigned	kRadioMaxPacketLength = 118;

	SessionInfo _sessions[kMaxSessions] {};
	uint8_t _next_session_id{0};

	Mavlink &_mavlink;

	/* do not allow copying this class */
	MavlinkFTP(const MavlinkFTP &);
	MavlinkFTP operator=(const MavlinkFTP &);

	static constexpr int _work_buffer1_len = kMaxDataLength;
	static constexpr int _work_buffer2_len = 256;
	char _work_buffer1[_work_buffer1_len];
	char _work_buffer2[_work_buffer2_len];
	hrt_abstime _last_work_buffer_access{0};

	/// Paces all bursts together against the one link budget.
	hrt_abstime _last_burst_send{0};

	/// Slot the next burst packet is taken from first.
	unsigned _burst_cursor{0};

	// prepend a root directory to each file/dir access to avoid enumerating the full FS tree (e.g. on Linux).
	// Path traversal via ".." is rejected by _validatePath().
	static constexpr const char _root_dir[] = PX4_ROOTFSDIR;
	static constexpr const int _root_dir_len = sizeof(_root_dir) - 1;

	// Virtual directory prefix for log files as defined by the MAVLink FTP spec.
	// Paths that start with this prefix are mapped to the flight-stack log root.
	static constexpr const char _mav_log_prefix[] = "@MAV_LOG";
	static constexpr const int _mav_log_prefix_len = sizeof(_mav_log_prefix) - 1;
	static constexpr const char _mav_log_dir[] = PX4_STORAGEDIR "/log";
	static constexpr const int _mav_log_dir_len = sizeof(_mav_log_dir) - 1;

	bool _last_reply_valid = false;
	uint32_t _last_reply_target_system{0}; ///< _last_reply only holds the 8-bit payload field
	uint8_t _last_reply[MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_LEN - MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN
								      + sizeof(PayloadHeader) + sizeof(uint32_t)];

	int _our_errno {0};
};
