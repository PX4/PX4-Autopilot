/* Implementation of MessageDisplay class.
 */

#include <math.h>
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

void MessageDisplay::get(char* string) {
	// check if we should update the full message (and reset display)
	if (updated_) {
		// full_message = "Flight Mode: " + flight_mode_msg + " - ARMED: " + arming_msg + " - STATUS: " + status_msg + " - WARNING: " + warning_msg + "            ";

		// reset and construct full message
		strcpy(full_message, "");
		strncat(full_message, flight_mode_msg, 3);	// first three characters of Flight Mode
		strcat(full_message, "|");
		strncat(full_message, arming_msg, 3);		// first three characters of Arming Message
		strcat(full_message, "|");
		strncat(full_message, heading_msg, 2);		// first two characters of Heading

		// add a warning message, if it's not empty
		if (strlen(warning_msg) != 0) {
			// copy as much of warning message as we can fit
			strcat(full_message, " | WARNING: ");
			const size_t chars_used {22};
			strncat(full_message, warning_msg, MAX_MSG_LENGTH - chars_used - FULL_MSG_LENGTH - 1);

			// pad with one full length of terminal whitespace
			for (unsigned i = 0; i != FULL_MSG_LENGTH; ++i)
				strcat(full_message, " ");
		}

		// reset display variables
		last_update_ = 0;
		index = 0;
		updated_ = false;
	}

	// handle edge case where full message is short
	if (strlen(full_message) < FULL_MSG_LENGTH) {
		strncpy(string, full_message, FULL_MSG_LENGTH);
		return;
	}

	// check if we should update the sub-message (giving extra time to the beginning)
	uint32_t current_time = hrt_absolute_time();
	uint32_t dt = current_time - last_update_;
	if ( (index == 0 && dt > (4 * period_)) || (index != 0 && dt > period_) ) {
		// scroll through message by updating index
		if (++index > strlen(full_message) - FULL_MSG_LENGTH)
			index = 0;

		// save timestamp
		last_update_ = current_time;
	}

	// reset update flag and return latest message
	strncpy(string, full_message + index, FULL_MSG_LENGTH);
	return;
}

} // namespace msp_osd
