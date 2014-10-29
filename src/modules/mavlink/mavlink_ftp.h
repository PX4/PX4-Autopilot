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

/// @file mavlink_ftp.h
///     @author px4dev, Don Gagne <don@thegagnes.com>
 
#include <dirent.h>
#include <queue.h>

#include <nuttx/wqueue.h>
#include <systemlib/err.h>

#include "mavlink_messages.h"
#include "mavlink_main.h"

class MavlinkFtpTest;

/// @brief MAVLink remote file server. Support FTP like commands using MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL message.
/// A limited number of requests (kRequestQueueSize) may be outstanding at a time. Additional messages will be discarded.
class MavlinkFTP
{
public:
	/// @brief Returns the one Mavlink FTP server in the system.
	static MavlinkFTP* get_server(void);
    
	/// @brief Contructor is only public so unit test code can new objects.
	MavlinkFTP();
	
	/// @brief Adds the specified message to the work queue.
	void handle_message(Mavlink* mavlink, mavlink_message_t *msg);
    
	typedef void (*ReceiveMessageFunc_t)(const mavlink_message_t *msg, MavlinkFtpTest* ftpTest);
	
	/// @brief Sets up the server to run in unit test mode.
	///	@param rcvmsgFunc Function which will be called to handle outgoing mavlink messages.
	///	@param ftp_test MavlinkFtpTest object which the function is associated with
	void set_unittest_worker(ReceiveMessageFunc_t rcvMsgFunc, MavlinkFtpTest *ftp_test);

	/// @brief This is the payload which is in mavlink_file_transfer_protocol_t.payload. We pad the structure ourselves to
	/// 32 bit alignment to avoid usage of any pack pragmas.
	struct PayloadHeader
        {
		uint16_t	seqNumber;	///< sequence number for message
		uint8_t		session;	///< Session id for read and write commands
		uint8_t		opcode;		///< Command opcode
		uint8_t		size;		///< Size of data
		uint8_t		req_opcode;	///< Request opcode returned in kRspAck, kRspNak message
		uint8_t		padding[2];	///< 32 bit aligment padding
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
	
private:
	/// @brief Unit of work which is queued to work_queue
	struct Request
	{
		work_s	work;			///< work queue entry
		Mavlink	*mavlink;		///< Mavlink to reply to
		uint8_t serverSystemId;		///< System ID to send from
		uint8_t serverComponentId;	///< Component ID to send from
		uint8_t serverChannel;		///< Channel to send to
		uint8_t targetSystemId;		///< System ID to target reply to

		mavlink_file_transfer_protocol_t message;	///< Protocol message
	};
	
	Request		*_get_request(void);
	void		_return_request(Request *req);
	void		_lock_request_queue(void);
	void		_unlock_request_queue(void);
	
	char		*_data_as_cstring(PayloadHeader* payload);
	
	static void	_worker_trampoline(void *arg);
	void		_process_request(Request *req);
	void		_reply(Request *req);
	int		_copy_file(const char *src_path, const char *dst_path, ssize_t length);

	ErrorCode	_workList(PayloadHeader *payload);
	ErrorCode	_workOpen(PayloadHeader *payload, int oflag);
	ErrorCode	_workRead(PayloadHeader *payload);
	ErrorCode	_workWrite(PayloadHeader *payload);
	ErrorCode	_workTerminate(PayloadHeader *payload);
	ErrorCode	_workReset(PayloadHeader* payload);
	ErrorCode	_workRemoveDirectory(PayloadHeader *payload);
	ErrorCode	_workCreateDirectory(PayloadHeader *payload);
	ErrorCode	_workRemoveFile(PayloadHeader *payload);
	ErrorCode	_workTruncateFile(PayloadHeader *payload);
	ErrorCode	_workRename(PayloadHeader *payload);
	ErrorCode	_workCalcFileCRC32(PayloadHeader *payload);

	static const unsigned	kRequestQueueSize = 2;			///< Max number of queued requests
	Request			_request_bufs[kRequestQueueSize];	///< Request buffers which hold work
	dq_queue_t		_request_queue;				///< Queue of available Request buffers
	sem_t			_request_queue_sem;			///< Semaphore for locking access to _request_queue
	
	int _find_unused_session(void);
	bool _valid_session(unsigned index);
	
	static const char	kDirentFile = 'F';	///< Identifies File returned from List command
	static const char	kDirentDir = 'D';	///< Identifies Directory returned from List command
	static const char	kDirentSkip = 'S';	///< Identifies Skipped entry from List command
	
	/// @brief Maximum data size in RequestHeader::data
	static const uint8_t	kMaxDataLength = MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN - sizeof(PayloadHeader);
	
	static const unsigned kMaxSession = 2;	///< Max number of active sessions
	int	_session_fds[kMaxSession];	///< Session file descriptors, 0 for empty slot
	
	ReceiveMessageFunc_t _utRcvMsgFunc;	///< Unit test override for mavlink message sending
	MavlinkFtpTest *_ftp_test;		///< Additional parameter to _utRcvMsgFunc;
};
