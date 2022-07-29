/* Implementation of MessageDisplay class.
 */

#include "message_display.hpp"

namespace msp_osd {

void MessageDisplay::set(const MessageDisplayType mode, const char* string) {
	switch(mode) {
	case MessageDisplayType::WARNING:
		if (strcmp(warning_msg, string) != 0)
		{
			strncpy(warning_msg, string, MAX_MSG_LENGTH);
			updated_ = true;
		}
		break;
	case MessageDisplayType::FLIGHT_MODE:
		if (strcmp(flight_mode_msg, string) != 0)
		{
			strncpy(flight_mode_msg, string, MAX_MSG_LENGTH);
			updated_ = true;
		}
		break;
	case MessageDisplayType::ARMING:
		if (strcmp(arming_msg, string) != 0)
		{
			strncpy(arming_msg, string, MAX_MSG_LENGTH);
			updated_ = true;
		}
		break;
	case MessageDisplayType::STATUS:
		if (strcmp(status_msg, string) != 0)
		{
			strncpy(status_msg, string, MAX_MSG_LENGTH);
			updated_ = true;
		}
		break;
	case MessageDisplayType::HEADING:
		if (strcmp(heading_msg, string) != 0)
		{
			strncpy(heading_msg, string, MAX_MSG_LENGTH);
			updated_ = true;
		}
		break;
	default:
		PX4_ERR("Received unsupported message display mode.");
		break;
	}
}

// get the latest subsection of the message
void MessageDisplay::get(char* string) {
	// check if we should update the full message (and reset display)
	if (updated_) {
		// // full_message = "Flight Mode: " + flight_mode_msg + " - ARMED: " + arming_msg + " - STATUS: " + status_msg + " - WARNING: " + warning_msg + "            ";

		// // construct full message
		// full_message = flight_mode_msg.substring(0,3) + "|" + arming_msg + "|" + heading_msg;
		// if (warning_msg.length() != 0) {
		// 	full_message += " | WARNING: " + warning_msg;
		// 	// pad with whitespace
		// 	for (unsigned i = 0; i != msg_length_; ++i)
		// 		full_message += " ";
		// }

		// last_update_ = 0;
		// updated_ = true;
		// index = 0;
	}

	// handle edge case where full message is short
	if (strlen(full_message) < msg_length_) {
		strncpy(string, full_message, msg_length_);
		return;
	}

	// check if we should update the sub-message (giving extra time to the beginning)
	uint32_t current_time = hrt_absolute_time();
	uint32_t dt = current_time - last_update_;
	if ( (index == 0 && dt > (4 * period_)) || (index != 0 && dt > period_) ) {
		// scroll through message
		// update index
		if (++index > strlen(full_message) - msg_length_)
			index = 0;

		// save timestamp
		last_update_ = current_time;
	}

	// reset update flag and return latest message
	updated_ = false;
	strncpy(string, full_message + index, msg_length_);
	return;
}

} // namespace msp_osd
