/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
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

/// @file mavlink_ftp_test.h
///	@author Don Gagne <don@thegagnes.com>

#pragma once

#include <unit_test/unit_test.h>
#include "../mavlink_bridge_header.h"
#include "../mavlink_ftp.h"

class MavlinkFtpTest : public UnitTest
{
public:
	MavlinkFtpTest();
	virtual ~MavlinkFtpTest();
	
	virtual bool run_tests(void);
	
	static void receive_message(const mavlink_message_t *msg, MavlinkFtpTest* ftpTest);
	
	static const uint8_t serverSystemId = 50;	///< System ID for server
	static const uint8_t serverComponentId = 1;	///< Component ID for server
	static const uint8_t serverChannel = 0;		///< Channel to send to
	
	static const uint8_t clientSystemId = 1;	///< System ID for client
	static const uint8_t clientComponentId = 0;	///< Component ID for client
	
	// We don't want any of these
	MavlinkFtpTest(const MavlinkFtpTest&);
	MavlinkFtpTest& operator=(const MavlinkFtpTest&);
	
private:
	virtual void _init(void);
	virtual void _cleanup(void);
	
	bool _ack_test(void);
	bool _bad_opcode_test(void);
	bool _bad_datasize_test(void);
	bool _list_test(void);
	bool _list_eof_test(void);
	bool _open_badfile_test(void);
	bool _open_terminate_test(void);
	bool _terminate_badsession_test(void);
	bool _read_test(void);
	bool _read_badsession_test(void);
	bool _removedirectory_test(void);
	bool _createdirectory_test(void);
	bool _removefile_test(void);
	
	void _receive_message(const mavlink_message_t *msg);
	void _setup_ftp_msg(MavlinkFTP::PayloadHeader *payload_header, uint8_t size, const uint8_t *data, mavlink_message_t *msg);
	bool _decode_message(const mavlink_message_t *msg, mavlink_file_transfer_protocol_t *ftp_msg, MavlinkFTP::PayloadHeader **payload);
	bool _send_receive_msg(MavlinkFTP::PayloadHeader	*payload_header,
                           uint8_t				size,
                           const uint8_t			*data,
                           mavlink_file_transfer_protocol_t	*ftp_msg_reply,
                           MavlinkFTP::PayloadHeader		**payload_reply);
	void _cleanup_microsd(void);
	
	MavlinkFTP *_ftp_server;
    
	mavlink_message_t _reply_msg;
    
	uint16_t _lastOutgoingSeqNumber;
	
	struct ReadTestCase {
		const char	*file;
		const uint16_t	length;
	};
	static const ReadTestCase _rgReadTestCases[];
	
	static const char _unittest_microsd_dir[];
	static const char _unittest_microsd_file[];
};

bool mavlink_ftp_test(void);
