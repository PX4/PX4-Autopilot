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
#include <queue.h>

#include <systemlib/err.h>

#include "mavlink_stream.h"
#include "mavlink_bridge_header.h"

class MavlinkFtpTest;

/// MAVLink remote file server. Support FTP like commands using MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL message.
class MavlinkFTP : public MavlinkStream
{
public:
	/// @brief Contructor is only public so unit test code can new objects.
	MavlinkFTP(Mavlink *mavlink);
	~MavlinkFTP();

	static MavlinkStream *new_instance(Mavlink *mavlink);
	
	/// Handle possible FTP message
	void handle_message(const mavlink_message_t *msg);

	typedef void (*ReceiveMessageFunc_t)(const mavlink_file_transfer_protocol_t* ftp_req, void *worker_data);
	
	/// @brief Sets up the server to run in unit test mode.
	///	@param rcvmsgFunc Function which will be called to handle outgoing mavlink messages.
	///	@param worker_data Data to pass to worker
	void set_unittest_worker(ReceiveMessageFunc_t rcvMsgFunc, void *worker_data);

	/// @brief This is the payload which is in mavlink_file_transfer_protocol_t.payload. We pad the structure ourselves to
	/// 32 bit alignment to avoid usage of any pack pragmas.
	struct PayloadHeader
        {
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
	enum Opcode : uint8_t
	{
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
		
		kRspAck = 128,		///< Ack response
		kRspNak			///< Nak response
	};
	
	/// @brief Error codes returned in Nak response PayloadHeader.data[0].
	enum ErrorCode : uint8_t
        {
		kErrNone,
		kErrFail,			///< Unknown failure
		kErrFailErrno,			///< Command failed, errno sent back in PayloadHeader.data[1]
		kErrInvalidDataSize,		///< PayloadHeader.size is invalid
		kErrInvalidSession,		///< Session is not currently open
		kErrNoSessionsAvailable,	///< All available Sessions in use
		kErrEOF,			///< Offset past end of file for List and Read commands
		kErrUnknownCommand		///< Unknown command opcode
        };
	
	// MavlinkStream overrides
	virtual const char *get_name(void) const;
	virtual uint8_t get_id(void);
	virtual unsigned get_size(void);
	
private:
	char		*_data_as_cstring(PayloadHeader* payload);
	
	void		_process_request(mavlink_file_transfer_protocol_t* ftp_req, uint8_t target_system_id);
	void		_reply(mavlink_file_transfer_protocol_t* ftp_req);
	int		_copy_file(const char *src_path, const char *dst_path, size_t length);

	ErrorCode	_workList(PayloadHeader *payload, bool list_hidden = false);
	ErrorCode	_workOpen(PayloadHeader *payload, int oflag);
	ErrorCode	_workRead(PayloadHeader *payload);
	ErrorCode	_workBurst(PayloadHeader* payload, uint8_t target_system_id);
	ErrorCode	_workWrite(PayloadHeader *payload);
	ErrorCode	_workTerminate(PayloadHeader *payload);
	ErrorCode	_workReset(PayloadHeader* payload);
	ErrorCode	_workRemoveDirectory(PayloadHeader *payload);
	ErrorCode	_workCreateDirectory(PayloadHeader *payload);
	ErrorCode	_workRemoveFile(PayloadHeader *payload);
	ErrorCode	_workTruncateFile(PayloadHeader *payload);
	ErrorCode	_workRename(PayloadHeader *payload);
	ErrorCode	_workCalcFileCRC32(PayloadHeader *payload);
	
	uint8_t _getServerSystemId(void);
	uint8_t _getServerComponentId(void);
	uint8_t _getServerChannel(void);

	// Overrides from MavlinkStream
	virtual void send(const hrt_abstime t);
	
	static const char	kDirentFile = 'F';	///< Identifies File returned from List command
	static const char	kDirentDir = 'D';	///< Identifies Directory returned from List command
	static const char	kDirentSkip = 'S';	///< Identifies Skipped entry from List command
	
	/// @brief Maximum data size in RequestHeader::data
	static const uint8_t	kMaxDataLength = MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN - sizeof(PayloadHeader);
	
	struct SessionInfo {
		int		fd;
		uint32_t	file_size;
		bool		stream_download;
		uint32_t	stream_offset;
		uint16_t	stream_seq_number;
		uint8_t		stream_target_system_id;
		unsigned	stream_chunk_transmitted;
	};
	struct SessionInfo _session_info;	///< Session info, fd=-1 for no active session
	
	ReceiveMessageFunc_t	_utRcvMsgFunc;	///< Unit test override for mavlink message sending
	void			*_worker_data;	///< Additional parameter to _utRcvMsgFunc;
	
	/* do not allow copying this class */
	MavlinkFTP(const MavlinkFTP&);
	MavlinkFTP operator=(const MavlinkFTP&);
	
	
	// Mavlink test needs to be able to call send
	friend class MavlinkFtpTest;
};
