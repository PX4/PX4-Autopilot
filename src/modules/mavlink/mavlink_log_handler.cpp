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

/// @file mavlink_log_handler.h
/// @author px4dev, Gus Grubba <mavlink@grubba.com>

#include "mavlink_log_handler.h"
#include "mavlink_main.h"
#include <sys/stat.h>
#include <time.h>

#define MOUNTPOINT PX4_ROOTFSDIR "/fs/microsd"
 
static const char *kLogRoot    = MOUNTPOINT "/log";
static const char *kLogData    = MOUNTPOINT "/logdata.txt";
static const char *kTmpData    = MOUNTPOINT "/$log$.txt";

#ifdef __PX4_NUTTX
    #define PX4LOG_REGULAR_FILE DTYPE_FILE
    #define PX4LOG_DIRECTORY 	DTYPE_DIRECTORY
#else
    #define PX4LOG_REGULAR_FILE DT_REG
    #define PX4LOG_DIRECTORY 	DT_DIR
#endif

//#define MAVLINK_LOG_HANDLER_VERBOSE

#ifdef MAVLINK_LOG_HANDLER_VERBOSE
#define PX4LOG_WARN(fmt, ...) warnx(fmt, ##__VA_ARGS__)
#else
#define PX4LOG_WARN(fmt, ...)
#endif

//-------------------------------------------------------------------
static bool
stat_file(const char* file, time_t* date = 0, uint32_t* size = 0) {
	struct stat st;
	if (stat(file, &st) == 0) {
		if (date) *date = st.st_mtime;
		if (size) *size = st.st_size;
		return true;
	}
	return false;
}

//-------------------------------------------------------------------
MavlinkLogHandler *
MavlinkLogHandler::new_instance(Mavlink *mavlink)
{
    return new MavlinkLogHandler(mavlink);
}

//-------------------------------------------------------------------
MavlinkLogHandler::MavlinkLogHandler(Mavlink *mavlink)
    : MavlinkStream(mavlink)
    , _pLogHandlerHelper(0)
{

}

//-------------------------------------------------------------------
void
MavlinkLogHandler::handle_message(const mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
		_log_request_list(msg);
		break;
	case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
		_log_request_data(msg);
		break;
	case MAVLINK_MSG_ID_LOG_ERASE:
		_log_request_erase(msg);
		break;
	case MAVLINK_MSG_ID_LOG_REQUEST_END:
		_log_request_end(msg);
		break;
	}
}

//-------------------------------------------------------------------
const char*
MavlinkLogHandler::get_name(void) const
{
    return "MAVLINK_LOG_HANDLER";
}

//-------------------------------------------------------------------
uint8_t
MavlinkLogHandler::get_id(void)
{
    return MAVLINK_MSG_ID_LOG_ENTRY;
}

//-------------------------------------------------------------------
unsigned
MavlinkLogHandler::get_size(void)
{
	//-- Sending Log Entries
	if (_pLogHandlerHelper && _pLogHandlerHelper->current_status == LogListHelper::LOG_HANDLER_LISTING) {
		return MAVLINK_MSG_ID_LOG_ENTRY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	//-- Sending Log Data (contents of one of the log files)
	} else if (_pLogHandlerHelper && _pLogHandlerHelper->current_status == LogListHelper::LOG_HANDLER_SENDING_DATA) {
		return MAVLINK_MSG_ID_LOG_DATA_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	//-- Idle
	} else {
		return 0;
	}
}

//-------------------------------------------------------------------
void
MavlinkLogHandler::send(const hrt_abstime /*t*/)
{
	//-- Send log entry stream packets until buffer is full
	while (_pLogHandlerHelper && _pLogHandlerHelper->current_status == LogListHelper::LOG_HANDLER_LISTING && _mavlink->get_free_tx_buf() > get_size()) {
		_log_send_listing();
	};
	//-- An arbitrary count of max log data packets in one go
	int count = 100;
	while (_pLogHandlerHelper && _pLogHandlerHelper->current_status == LogListHelper::LOG_HANDLER_SENDING_DATA && _mavlink->get_free_tx_buf() > get_size() && --count) {
		_log_send_data();
	};
}

//-------------------------------------------------------------------
void
MavlinkLogHandler::_log_request_list(const mavlink_message_t *msg)
{
	mavlink_log_request_list_t request;
	mavlink_msg_log_request_list_decode(msg, &request);
	//-- Check for re-requests (data loss) or new request
	if(_pLogHandlerHelper) {
		_pLogHandlerHelper->current_status = LogListHelper::LOG_HANDLER_IDLE;
		//-- Is this a new request?
		if((request.end - request.start) > _pLogHandlerHelper->log_count) {
			delete _pLogHandlerHelper;
			_pLogHandlerHelper = NULL;
		}
	}
	if(!_pLogHandlerHelper) {
		//-- Prepare new request
		_pLogHandlerHelper = new LogListHelper;
	}
        if (_pLogHandlerHelper->log_count)
        {
        	//-- Define (and clamp) range
		_pLogHandlerHelper->next_entry = request.start < _pLogHandlerHelper->log_count ? request.start : _pLogHandlerHelper->log_count - 1;
		_pLogHandlerHelper->last_entry = request.end   < _pLogHandlerHelper->log_count ? request.end   : _pLogHandlerHelper->log_count - 1;
        }
	PX4LOG_WARN("\nMavlinkLogHandler::_log_request_list: start: %u last: %u count: %u\n",
		_pLogHandlerHelper->next_entry,
		_pLogHandlerHelper->last_entry,
		_pLogHandlerHelper->log_count);
	//-- Enable streaming
	_pLogHandlerHelper->current_status = LogListHelper::LOG_HANDLER_LISTING;
}

//-------------------------------------------------------------------
void
MavlinkLogHandler::_log_request_data(const mavlink_message_t *msg)
{
	//-- If we haven't listed, we can't do much
	if(!_pLogHandlerHelper) {
		PX4LOG_WARN("MavlinkLogHandler::_log_request_data Log request with no list requested.\n");
		return;
	}
	mavlink_log_request_data_t request;
	mavlink_msg_log_request_data_decode(msg, &request);
	//-- Does the requested log exist?
	if (request.id >= _pLogHandlerHelper->log_count) {
		PX4LOG_WARN("MavlinkLogHandler::_log_request_data Requested log %u but we only have %u.\n", request.id, _pLogHandlerHelper->log_count);
		return;
	}
	//-- If we were sending log entries, stop it
        _pLogHandlerHelper->current_status = LogListHelper::LOG_HANDLER_IDLE;
        //-- Init send log dataset
        _pLogHandlerHelper->current_log_filename[0] = 0;
        _pLogHandlerHelper->current_log_index = request.id;
	uint32_t time_utc = 0;
	_pLogHandlerHelper->get_entry(_pLogHandlerHelper->current_log_index, _pLogHandlerHelper->current_log_size, time_utc, _pLogHandlerHelper->current_log_filename);
        _pLogHandlerHelper->current_log_data_offset = request.ofs;
        if (_pLogHandlerHelper->current_log_data_offset >= _pLogHandlerHelper->current_log_size) {
		_pLogHandlerHelper->current_log_data_remaining = 0;
        } else {
		_pLogHandlerHelper->current_log_data_remaining = _pLogHandlerHelper->current_log_size - request.ofs;
        }
        if (_pLogHandlerHelper->current_log_data_remaining > request.count) {
		_pLogHandlerHelper->current_log_data_remaining = request.count;
        }
	//-- Enable streaming
        _pLogHandlerHelper->current_status = LogListHelper::LOG_HANDLER_SENDING_DATA;
}

//-------------------------------------------------------------------
void
MavlinkLogHandler::_log_request_erase(const mavlink_message_t* /*msg*/)
{
	/*
	mavlink_log_erase_t request;
	mavlink_msg_log_erase_decode(msg, &request);
	*/
	if (_pLogHandlerHelper) {
		delete _pLogHandlerHelper;
		_pLogHandlerHelper = 0;
	}
	LogListHelper::delete_all(kLogRoot);
}

//-------------------------------------------------------------------
void
MavlinkLogHandler::_log_request_end(const mavlink_message_t* /*msg*/)
{
	PX4LOG_WARN("MavlinkLogHandler::_log_request_end\n");
	if (_pLogHandlerHelper) {
		delete _pLogHandlerHelper;
		_pLogHandlerHelper = 0;
	}
}

//-------------------------------------------------------------------
void
MavlinkLogHandler::_log_send_listing()
{
	mavlink_log_entry_t response;
	_pLogHandlerHelper->get_entry(_pLogHandlerHelper->next_entry, response.size, response.time_utc);
	response.id 		= _pLogHandlerHelper->next_entry;
	response.num_logs	= _pLogHandlerHelper->log_count;
	response.last_log_num	= _pLogHandlerHelper->last_entry;
	_mavlink->send_message(MAVLINK_MSG_ID_LOG_ENTRY, &response);
        //-- If we're done listing, flag it.
        if (_pLogHandlerHelper->next_entry == _pLogHandlerHelper->last_entry) {
		_pLogHandlerHelper->current_status = LogListHelper::LOG_HANDLER_IDLE;
        } else {
		_pLogHandlerHelper->next_entry++;
        }
	PX4LOG_WARN("MavlinkLogHandler::_log_send_listing id: %u count: %u last: %u size: %u date: %u status: %d\n",
		response.id,
		response.num_logs,
		response.last_log_num,
		response.size,
		response.time_utc,
		_pLogHandlerHelper->current_status);
}

//-------------------------------------------------------------------
void
MavlinkLogHandler::_log_send_data()
{
	mavlink_log_data_t response;
	uint32_t len = _pLogHandlerHelper->current_log_data_remaining;
	if (len > sizeof(response.data)) {
		len = sizeof(response.data);
	}
	size_t read_size = _pLogHandlerHelper->get_log_data(len, response.data);
	if (read_size < sizeof(response.data)) {
		memset(&response.data[read_size], 0, sizeof(response.data) - read_size);
	}
	response.ofs   = _pLogHandlerHelper->current_log_data_offset;
	response.id    = _pLogHandlerHelper->current_log_index;
	response.count = read_size;
	_mavlink->send_message(MAVLINK_MSG_ID_LOG_DATA, &response);
	_pLogHandlerHelper->current_log_data_offset    += read_size;
	_pLogHandlerHelper->current_log_data_remaining -= read_size;
	if (read_size < sizeof(response.data) || _pLogHandlerHelper->current_log_data_remaining == 0) {
		_pLogHandlerHelper->current_status = LogListHelper::LOG_HANDLER_IDLE;
	}
}

//-------------------------------------------------------------------
LogListHelper::LogListHelper()
	: next_entry(0)
	, last_entry(0)
	, log_count(0)
	, current_status(LOG_HANDLER_IDLE)
	, current_log_index(0)
	, current_log_size(0)
	, current_log_data_offset(0)
	, current_log_data_remaining(0)
{
	_init();
}

//-------------------------------------------------------------------
LogListHelper::~LogListHelper()
{
	// Remove log data files (if any)
	unlink(kLogData);
	unlink(kTmpData);
}

//-------------------------------------------------------------------
bool
LogListHelper::get_entry(int idx, uint32_t& size, uint32_t& date, char* filename)
{
	//-- Find log file in log list file created during init()
	size = 0;
	date = 0;
	bool result = false;
	//-- Open list of log files
	FILE* f = ::fopen(kLogData, "r");
	if (f) {
		//--- Find requested entry
		char line[160];
		int count = 0;
		while (fgets(line, sizeof(line), f)) {
			//-- Found our "index"
			if(count++ == idx) {
				char file[128];
				if(sscanf(line, "%u %u %s", &date, &size, file) == 3) {
					if(filename) {
						strcpy(filename, file);
					}
					result = true;
					break;
				}
			}
		}
		fclose(f);
	}
	return result;
}

//-------------------------------------------------------------------
size_t
LogListHelper::get_log_data(uint8_t len, uint8_t* buffer)
{
	if(!current_log_filename[0]) 
		return 0;
	FILE* f = fopen(current_log_filename, "r");
	if (!f) {
		PX4LOG_WARN("MavlinkLogHandler::get_log_data Could not open %s\n", current_log_filename);
		return 0;
	}
	if(fseek(f, current_log_data_offset, SEEK_SET)) {
		fclose(f);
		PX4LOG_WARN("MavlinkLogHandler::get_log_data Seek error in %s\n", current_log_filename);
		return 0;
	}
	size_t result = fread(buffer, 1, len, f);
	fclose(f);
	return result;
}

//-------------------------------------------------------------------
void
LogListHelper::_init()
{
	/*
	
		When this helper is created, it scans the log directory
		and collects all log files found into one file for easy,
		subsequent access.
	*/

	current_log_filename[0] = 0;
	// Remove old log data file (if any)
	unlink(kLogData);
    	// Open log directory
	DIR *dp = opendir(kLogRoot);
	if (dp == nullptr) {
		// No log directory. Nothing to do.
		return;
	}
    	// Create work file
    	FILE* f = ::fopen(kTmpData, "w");
	if (!f) {
		PX4LOG_WARN("MavlinkLogHandler::init Error creating %s\n", kTmpData);
		closedir(dp);
		return;
	}
	// Scan directory and collect log files
	struct dirent entry, *result = nullptr;
	while (readdir_r(dp, &entry, &result) == 0) {
		// no more entries?
		if (result == nullptr) {
			break;
		}
		if (entry.d_type == PX4LOG_DIRECTORY)
		{
			time_t tt;
			char log_path[128];
			snprintf(log_path, sizeof(log_path), "%s/%s", kLogRoot, entry.d_name);
		        if (_get_session_date(log_path, entry.d_name, tt)) {
				_scan_logs(f, log_path, tt);
			}
		}
	}
	closedir(dp);
	fclose(f);
	// Rename temp file to data file
	if (rename(kTmpData, kLogData)) {
		PX4LOG_WARN("MavlinkLogHandler::init Error renaming %s\n", kTmpData);
		log_count = 0;
	}
}

//-------------------------------------------------------------------
bool
LogListHelper::_get_session_date(const char* path, const char* dir, time_t& date)
{
	if(strlen(dir) > 4) {
		// Convert "sess000" to 00:00 Jan 1 1970 (day per session)
		if (strncmp(dir, "sess", 4) == 0) {
			unsigned u;
			if(sscanf(&dir[4], "%u", &u) == 1) {
				date = u * 60 * 60 * 24;
				return true;
			}
		} else {
			if (stat_file(path, &date)) {
				return true;
			}
			/* strptime not available for some reason
			// Get date from directory name
			struct tm tt;
			if(strptime(dir, "%Y-%m-%d", &tt)) {
				date = mktime(&tt);
				return true;
			}
			*/
		}
	}
	return false;
}

//-------------------------------------------------------------------
void
LogListHelper::_scan_logs(FILE* f, const char* dir, time_t& date)
{
	DIR *dp = opendir(dir);
	if (dp) {
		struct dirent entry, *result = nullptr;
		while (readdir_r(dp, &entry, &result) == 0) {
			// no more entries?
			if (result == nullptr) {
				break;
			}
			if (entry.d_type == PX4LOG_REGULAR_FILE) {
				time_t  ldate = date;
				uint32_t size = 0;
				char log_file_path[128];
				snprintf(log_file_path, sizeof(log_file_path), "%s/%s", dir, entry.d_name);
				if(_get_log_time_size(log_file_path, entry.d_name, ldate, size)) {
					//-- Write entry out to list file
					fprintf(f, "%u %u %s\n", (unsigned)ldate, (unsigned)size, log_file_path);
					log_count++;
				}
			}
		}
	}
}

//-------------------------------------------------------------------
bool
LogListHelper::_get_log_time_size(const char* path, const char* file, time_t& date, uint32_t& size)
{
	if(file && file[0] && strstr(file, ".px4log")) {
		// Convert "log000" to 00:00 (minute per flight in session)
		if (strncmp(file, "log", 3) == 0) {
			unsigned u;
			if(sscanf(&file[3], "%u", &u) == 1) {
				date += (u * 60);
				if (stat_file(path, 0, &size)) {
					return true;
				}
			}
		} else {
			if (stat_file(path, &date, &size)) {
				return true;
			}
			/* strptime not available for some reason
			// Get time from file name
			struct tm tt;
			if(strptime(file, "%H_%M_%S", &tt)) {
				date += mktime(&tt);
				return true;
			}
			*/
		}
	}
	return false;
}

//-------------------------------------------------------------------
void
LogListHelper::delete_all(const char* dir)
{
    	//-- Open log directory
	DIR *dp = opendir(dir);
	if (dp == nullptr) {
		return;
	}
	struct dirent entry, *result = nullptr;
	while (readdir_r(dp, &entry, &result) == 0) {
		// no more entries?
		if (result == nullptr) {
			break;
		}
		if (entry.d_type == PX4LOG_DIRECTORY && entry.d_name[0] != '.') {
			char log_path[128];
			snprintf(log_path, sizeof(log_path), "%s/%s", dir, entry.d_name);
			LogListHelper::delete_all(log_path);
			if(rmdir(log_path)) {
				PX4LOG_WARN("MavlinkLogHandler::delete_all Error removing %s\n", log_path);
			}
		}
		if (entry.d_type == PX4LOG_REGULAR_FILE) {
			char log_path[128];
			snprintf(log_path, sizeof(log_path), "%s/%s", dir, entry.d_name);
			if(unlink(log_path)) {
				PX4LOG_WARN("MavlinkLogHandler::delete_all Error deleting %s\n", log_path);
			}
		}
	}
	closedir(dp);
}
