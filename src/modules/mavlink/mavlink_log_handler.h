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

/// @file mavlink_log_handler.h
/// @author px4dev, Gus Grubba <mavlink@grubba.com>

#include <dirent.h>
#include <queue.h>
#include <time.h>
#include <stdio.h>
#include <v1.0/mavlink_types.h>
#include "mavlink_stream.h"

class Mavlink;

// Log Listing Helper
class LogListHelper
{
public:
	LogListHelper();
	~LogListHelper();

public:
	static void delete_all(const char* dir);
	
public:

	bool 	get_entry		(int idx, uint32_t& size, uint32_t& date, char* filename = 0);
	size_t 	get_log_data		(uint8_t len, uint8_t* buffer);

	enum {
		LOG_HANDLER_IDLE,
		LOG_HANDLER_LISTING,
		LOG_HANDLER_SENDING_DATA
	};

	int 		next_entry;
	int 		last_entry;
	int 		log_count;

	int		current_status;
	uint16_t	current_log_index;
	uint32_t 	current_log_size;
	uint32_t	current_log_data_offset;
	uint32_t	current_log_data_remaining;
	char		current_log_filename[128];

private:
	void 	_init			();
	bool 	_get_session_date	(const char* path, const char* dir, time_t& date);
	void	_scan_logs		(FILE* f, const char* dir, time_t& date);
	bool 	_get_log_time_size	(const char* path, const char* file, time_t& date, uint32_t& size);
};

// MAVLink LOG_* Message Handler
class MavlinkLogHandler : public MavlinkStream
{
public:
	MavlinkLogHandler(Mavlink *mavlink);

	static MavlinkLogHandler *new_instance(Mavlink *mavlink);

	// Handle possible LOG message
	void handle_message		(const mavlink_message_t *msg);

	// Overrides from MavlinkStream
	const char*	get_name	(void) const;
	uint8_t 	get_id		(void);
	unsigned	get_size	(void);
	void 		send		(const hrt_abstime t);

private:
	void _log_message	(const mavlink_message_t *msg);
	void _log_request_list	(const mavlink_message_t *msg);
	void _log_request_data	(const mavlink_message_t *msg);
	void _log_request_erase	(const mavlink_message_t *msg);
	void _log_request_end	(const mavlink_message_t *msg);
	void _log_send_listing	();
	void _log_send_data	();

private:
	LogListHelper	*_pLogHandlerHelper;

};
