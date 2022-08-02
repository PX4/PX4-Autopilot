/* Class for displaying messages to a limited screen.
 */

#pragma once

#include <string.h>

#include <px4_platform_common/defines.h>

namespace msp_osd {

// character size limitations
#define MAX_MSG_LENGTH 250
#define FULL_MSG_LENGTH 12

// supported message types
enum MessageDisplayType
{
	WARNING,
	FLIGHT_MODE,
	ARMING,
	STATUS,
	HEADING
};

// display information
class MessageDisplay
{
	// working information
	char warning_msg[MAX_MSG_LENGTH] {""};
	char flight_mode_msg[MAX_MSG_LENGTH] {"???"};
	char arming_msg[MAX_MSG_LENGTH] {"???"};
	char heading_msg[MAX_MSG_LENGTH] {"??"};
	// currently unused:
	char status_msg[MAX_MSG_LENGTH] {""};

	// the full message and the part we're currently displaying
	char full_message[MAX_MSG_LENGTH] {"INITIALIZING"};

	// current index we're displaying
	uint16_t index {0};

	// last update timestamp
	uint64_t last_update_ {0};
	bool updated_ {false};

	// available message length for display and update period (us)
	const uint64_t period_;

	public:
	MessageDisplay()=delete;
	MessageDisplay(uint64_t period) : period_(period) {}

	// set the given string
	void set(const MessageDisplayType mode, const char* string);

	// get the latest subsection of the message
	void get(char* string);
};


} // namespace msp_osd
