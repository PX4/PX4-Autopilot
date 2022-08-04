/* Class for displaying messages to a limited screen.
 */

#pragma once

#include <string.h>

#include <px4_platform_common/defines.h>

namespace msp_osd {

// character size limitations
#define MSG_BUFFER_SIZE 250
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
	char warning_msg[MSG_BUFFER_SIZE] {""};
	char flight_mode_msg[MSG_BUFFER_SIZE] {"???"};
	char arming_msg[MSG_BUFFER_SIZE] {"???"};
	char heading_msg[MSG_BUFFER_SIZE] {"??"};
	// currently unused:
	char status_msg[MSG_BUFFER_SIZE] {""};

	// the full message and the part we're currently displaying
	char full_message[MSG_BUFFER_SIZE] {"INITIALIZING"};

	// current index we're displaying
	uint16_t index {0};

	// last update timestamp
	uint64_t last_update_ {0};
	bool updated_ {false};

	// available message length for display and update period (us)
	uint64_t period_;

	public:
	MessageDisplay()=delete;
	MessageDisplay(uint64_t period) : period_(period) {}

	// set the given string
	void set(const MessageDisplayType mode, const char* string);

	// get the latest subsection of the message
	void get(char* string);

	// update local parameters
	void set_period(const uint64_t period) {period_ = period;};
};


} // namespace msp_osd
