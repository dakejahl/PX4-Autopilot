/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "mavlink_log_handler.h"
#include "mavlink_main.h"
#include <dirent.h>
#include <sys/stat.h>

static constexpr int MAX_BYTES_BURST = 256 * 1024;
static const char *kLogListFilePath = PX4_STORAGEDIR "/logdata.txt";
static const char *kLogListFilePathTemp = PX4_STORAGEDIR "/$log$.txt";
static const char *kLogDir = PX4_STORAGEDIR "/log";

#ifdef __PX4_NUTTX
#define PX4LOG_REGULAR_FILE DTYPE_FILE
#define PX4LOG_DIRECTORY    DTYPE_DIRECTORY
#define PX4_MAX_FILEPATH 	CONFIG_PATH_MAX
#else
#define PX4LOG_REGULAR_FILE DT_REG
#define PX4LOG_DIRECTORY    DT_DIR
#define PX4_MAX_FILEPATH 	PATH_MAX
#endif

MavlinkLogHandler::MavlinkLogHandler(Mavlink &mavlink)
	: _mavlink(mavlink)
{}

MavlinkLogHandler::~MavlinkLogHandler()
{
	perf_free(_create_file_elapsed);

	if (_current_entry.fp) {
		::fclose(_current_entry.fp);
	}

	unlink(kLogListFilePath);
	unlink(kLogListFilePathTemp);
}

void MavlinkLogHandler::send()
{
	switch (_state) {
	case LogHandlerState::Inactive:
	case LogHandlerState::Idle: {
			handle_state_idle();
			break;
		}

	case LogHandlerState::Listing: {
			handle_state_listing();
			break;
		}

	case LogHandlerState::SendingData: {
			handle_state_sending_data();
			break;
		}
	}
}

void MavlinkLogHandler::handle_state_idle()
{
	if (_current_entry.fp) {
		PX4_INFO("you forgot to close the file");
		::fclose(_current_entry.fp);
		_current_entry.fp = nullptr;
	}

	_current_entry.id = 0xffff;
	_current_entry.time_utc = 0;
	_current_entry.size_bytes = 0;
	_current_entry.fp = nullptr;
	_current_entry.filepath[0] = 0;
	_current_entry.offset = 0;

	_current_entry_request.id = 0xffff;
	_current_entry_request.start_offset = 0;
	_current_entry_request.byte_count = 0;
}

void MavlinkLogHandler::handle_state_listing()
{
	if (_mavlink.get_free_tx_buf() <= sizeof(mavlink_log_entry_t)) {
		return;
	}

	DIR *dp = ::opendir(kLogDir);

	if (!dp) {
		// No log directory. Nothing to do.
		PX4_WARN("No logs available");
		return;
	}

	FILE *fp = ::fopen(kLogListFilePath, "r");

	if (!fp) {
		PX4_WARN("Failed to open log list file");
		::closedir(dp);
		return;
	}

	fseek(fp, _list_request.file_index, SEEK_SET);

	size_t bytes_sent = 0;

	char line[60];

	while (fgets(line, sizeof(line), fp)) {

		if (_list_request.current_id < _list_request.first_id) {
			_list_request.current_id++;
			continue;
		}

		// We can send!
		uint32_t size_bytes = 0;
		uint32_t time_utc = 0;
		char filepath[PX4_MAX_FILEPATH];

		// If parsed lined successfully, send the entry
		if (sscanf(line, "%" PRIu32 " %" PRIu32 " %s", &time_utc, &size_bytes, filepath) != 3) {
			PX4_INFO("sscanf failed");
			continue;
		}

		send_log_entry(time_utc, size_bytes);
		bytes_sent += sizeof(mavlink_log_entry_t);
		_list_request.current_id++;

		// Yield if we've exceed mavlink burst or buffer limit
		if (_mavlink.get_free_tx_buf() <= sizeof(mavlink_log_entry_t) || bytes_sent >= MAX_BYTES_BURST) {
			_list_request.file_index = ftell(fp);
			::fclose(fp);
			::closedir(dp);
			return;
		}
	}

	::fclose(fp);
	::closedir(dp);

	_list_request.current_id = 0;
	_list_request.file_index = 0;
	_state = LogHandlerState::Idle;
}

void MavlinkLogHandler::handle_state_sending_data()
{
	if (_current_entry_request.id != _current_entry.id) {

		// Close the old file
		if (_current_entry.fp) {
			::fclose(_current_entry.fp);
			_current_entry.fp = nullptr;
		}

		LogEntry entry = {};

		if (!log_entry_from_id(_current_entry_request.id, &entry)) {
			PX4_WARN("Log file ID %u does not exist", _current_entry_request.id);
			_state = LogHandlerState::Idle;
			return;
		}

		entry.fp = ::fopen(entry.filepath, "rb");

		if (!entry.fp) {
			PX4_WARN("Failed to open file %s", entry.filepath);
			return;
		}

		_current_entry = entry;

		PX4_INFO("sending entry id %" PRIu16 " size %" PRIu32, entry.id, entry.size_bytes);
	}

	if (_current_entry_request.start_offset >= _current_entry.size_bytes) {
		PX4_WARN("Request offset %" PRIu32 "greater than file size %" PRIu32, _current_entry_request.start_offset,
			 _current_entry.size_bytes);
		_state = LogHandlerState::Idle;
		return;
	}

	size_t bytes_sent = 0;

	while (_mavlink.get_free_tx_buf() > sizeof(mavlink_log_data_t) && bytes_sent < MAX_BYTES_BURST) {

		if (_starting_new_chunk) {
			_current_entry.offset = _current_entry_request.start_offset;
			_starting_new_chunk = false;
			_chunk_finished = false;
		}

		if (_chunk_finished) {
			return;
		}

		::fseek(_current_entry.fp, _current_entry.offset, SEEK_SET);

		mavlink_log_data_t msg;
		size_t bytes_read = fread(msg.data, 1, sizeof(msg.data), _current_entry.fp);

		// send mavlink message
		msg.id = _current_entry.id;
		msg.ofs = _current_entry.offset;
		msg.count = bytes_read;

		mavlink_msg_log_data_send_struct(_mavlink.get_channel(), &msg);

		bytes_sent += sizeof(mavlink_log_data_t);
		_current_entry.offset += bytes_read;

		bool file_finished = _current_entry.offset >= _current_entry.size_bytes;

		if (file_finished) {
			PX4_INFO("file finished");
			::fclose(_current_entry.fp);
			_current_entry.fp = nullptr;
			_state = LogHandlerState::Idle;
			return;
		}

		_chunk_finished = _current_entry.offset >= (_current_entry_request.byte_count + _current_entry_request.start_offset);

		if (_chunk_finished) {
			PX4_INFO("chunk finished");
		}
	}
}

void MavlinkLogHandler::handle_message(const mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
		handle_log_request_list(msg);
		break;

	case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
		handle_log_request_data(msg);
		break;

	case MAVLINK_MSG_ID_LOG_REQUEST_END:
		handle_log_request_end(msg);
		break;

	case MAVLINK_MSG_ID_LOG_ERASE:
		handle_log_erase(msg);
		break;
	}
}

void MavlinkLogHandler::handle_log_request_list(const mavlink_message_t *msg)
{
	PX4_INFO("handle_log_request_list");

	mavlink_log_request_list_t request;
	mavlink_msg_log_request_list_decode(msg, &request);

	if (!create_log_list_file()) {
		return;
	}

	// TODO: mutex?
	_list_request.first_id = request.start;
	_list_request.last_id = request.end == 0xffff ? _num_logs : request.end;
	_list_request.current_id = 0;
	_list_request.file_index = 0;
	_logs_listed = true;
	_state = LogHandlerState::Listing;
}

void MavlinkLogHandler::handle_log_request_data(const mavlink_message_t *msg)
{
	PX4_WARN("handle_log_request_data");

	if (!_logs_listed) {
		PX4_WARN("Logs not yet listed");
		return;
	}

	mavlink_log_request_data_t request;
	mavlink_msg_log_request_data_decode(msg, &request);

	if (request.id >= _num_logs) {
		PX4_WARN("Requested log %" PRIu16 " but we only have %u", request.id, _num_logs);
		return;
	}

	_current_entry_request.id = request.id;
	_current_entry_request.start_offset = request.ofs;
	_current_entry_request.byte_count = request.count;
	_starting_new_chunk = true;
	_state = LogHandlerState::SendingData;
}

void MavlinkLogHandler::handle_log_request_end(const mavlink_message_t *msg)
{
	_state = LogHandlerState::Idle;
}

void MavlinkLogHandler::handle_log_erase(const mavlink_message_t *msg)
{
	if (_current_entry.fp) {
		::fclose(_current_entry.fp);
		_current_entry.fp = nullptr;
	}

	_state = LogHandlerState::Inactive;
	unlink(kLogListFilePath);
	unlink(kLogListFilePathTemp);

	delete_all_logs(kLogDir);
}

bool MavlinkLogHandler::create_log_list_file()
{
	perf_begin(_create_file_elapsed);

	auto start_time = hrt_absolute_time();

	// clean up old file
	::unlink(kLogListFilePath);
	_num_logs = 0;

	// open logging dir
	DIR *dp = ::opendir(kLogDir);

	if (!dp) {
		// No log directory. Nothing to do.
		PX4_WARN("No logs available");
		return false;
	}

	FILE *temp_fp = ::fopen(kLogListFilePathTemp, "w");

	if (!temp_fp) {
		PX4_WARN("Failed to create temp file");
		::closedir(dp);
		return false;
	}

	struct dirent *result = nullptr;

	// Iterate over the log/ directory which contains subdirectories for each yyyy-mm-dd
	while (1) {
		result = readdir(dp);

		if (!result) {
			// Reached end of directory
			break;
		}

		if (result->d_type != PX4LOG_DIRECTORY) {
			// Skip stray files
			continue;
		}

		// Skip the '.' and '..' entries
		if (strcmp(result->d_name, ".") == 0 || strcmp(result->d_name, "..") == 0) {
			continue;
		}

		// Open up the sub directory
		char dirpath[60];
		int ret = snprintf(dirpath, sizeof(dirpath), "%s/%s", kLogDir, result->d_name);

		bool path_is_ok = (ret > 0) && (ret < (int)sizeof(dirpath));

		if (!path_is_ok) {
			PX4_WARN("Log subdir path error: %s", dirpath);
			continue;
		}

		// Iterate over files inside the subdir and write them to the file
		write_entries_to_file(temp_fp, dirpath);
	}

	::fclose(temp_fp);
	::closedir(dp);

	// Rename temp file to data file
	if (::rename(kLogListFilePathTemp, kLogListFilePath)) {
		PX4_WARN("Failed to rename temp file");
		return false;
	}

	perf_end(_create_file_elapsed);

	perf_print_counter(_create_file_elapsed);

	PX4_INFO("time: %" PRIu64, hrt_elapsed_time(&start_time));

	return true;
}

void MavlinkLogHandler::write_entries_to_file(FILE *fp, const char *dir)
{
	DIR *dp = opendir(dir);
	struct dirent *result = nullptr;

	while (1) {
		result = readdir(dp);

		if (!result) {
			// Reached end of directory
			break;
		}

		if (result->d_type != PX4LOG_REGULAR_FILE) {
			// Skip non files
			continue;
		}

		char filepath[PX4_MAX_FILEPATH];
		int ret = snprintf(filepath, sizeof(filepath), "%s/%s", dir, result->d_name);
		bool path_is_ok = (ret > 0) && (ret < (int)sizeof(filepath));

		if (!path_is_ok) {
			PX4_WARN("Log subdir path error: %s", filepath);
			continue;
		}

		struct stat filestat;

		if (::stat(filepath, &filestat) != 0) {
			PX4_WARN("stat() failed: %s", filepath);
			continue;
		}

		// [ time ] [ size_bytes ] [ filepath ]
		fprintf(fp, "%u %u %s\n", unsigned(filestat.st_mtim.tv_sec), unsigned(filestat.st_size), filepath);
		_num_logs++;
	}

	::closedir(dp);
}

void MavlinkLogHandler::send_log_entry(uint32_t time_utc, uint32_t size_bytes)
{
	mavlink_log_entry_t msg;
	msg.time_utc     = time_utc;
	msg.size         = size_bytes;
	msg.id           = _list_request.current_id;
	msg.num_logs     = _num_logs;
	msg.last_log_num = _list_request.last_id;
	mavlink_msg_log_entry_send_struct(_mavlink.get_channel(), &msg);
}

bool MavlinkLogHandler::log_entry_from_id(uint16_t log_id, LogEntry *entry)
{
	DIR *dp = ::opendir(kLogDir);

	if (!dp) {
		PX4_WARN("No logs available");
		return false;
	}

	FILE *fp = ::fopen(kLogListFilePath, "r");

	if (!fp) {
		PX4_WARN("Failed to open %s", kLogListFilePath);
		::closedir(dp);
		return false;
	}

	bool found_entry = false;
	uint16_t current_id = 0;
	char line[60];

	while (fgets(line, sizeof(line), fp)) {

		if (current_id != log_id) {
			current_id++;
			continue;
		}

		if (sscanf(line, "%" PRIu32 " %" PRIu32 " %s", &(entry->time_utc), &(entry->size_bytes), entry->filepath) != 3) {
			PX4_INFO("sscanf failed");
			break;
		}

		entry->id = log_id;
		found_entry = true;
		break;
	}

	::fclose(fp);
	::closedir(dp);

	return found_entry;
}

void MavlinkLogHandler::delete_all_logs(const char *dir)
{
	//-- Open log directory
	DIR *dp = opendir(dir);

	if (dp == nullptr) {
		return;
	}

	struct dirent *result = nullptr;

	while ((result = readdir(dp))) {
		// no more entries?
		if (result == nullptr) {
			break;
		}

		if (result->d_type == PX4LOG_DIRECTORY && result->d_name[0] != '.') {
			char filepath[PX4_MAX_FILEPATH];
			int ret = snprintf(filepath, sizeof(filepath), "%s/%s", dir, result->d_name);
			bool path_is_ok = (ret > 0) && (ret < (int)sizeof(filepath));

			if (path_is_ok) {
				delete_all_logs(filepath);

				if (rmdir(filepath)) {
					PX4_WARN("Error removing %s", filepath);
				}
			}
		}

		if (result->d_type == PX4LOG_REGULAR_FILE) {
			char filepath[PX4_MAX_FILEPATH];
			int ret = snprintf(filepath, sizeof(filepath), "%s/%s", dir, result->d_name);
			bool path_is_ok = (ret > 0) && (ret < (int)sizeof(filepath));

			if (path_is_ok) {
				if (unlink(filepath)) {
					PX4_WARN("Error unlinking %s", filepath);
				}
			}
		}
	}

	closedir(dp);
}
