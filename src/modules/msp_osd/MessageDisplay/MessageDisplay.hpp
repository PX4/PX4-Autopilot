/* Class for displaying messages to a limited screen.
 */

#pragma once

#include <string.h>
#include <stdint.h>

namespace msp_osd {

// character size limitations
#define MSG_BUFFER_SIZE 250

// size of available characters, accounting for null terminator
//  note: the craft_name seems to think it has 15 chars. From testing
//        that seems incorrect
#define FULL_MSG_LENGTH 12
#define FULL_MSG_BUFFER 13

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
	char arming_msg[MSG_BUFFER_SIZE] {"????"};
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

	// dwell duration update period (us)
	uint64_t period_;
	uint64_t dwell_;

	public:
	MessageDisplay()=delete;
	MessageDisplay(const uint64_t period, const uint64_t dwell)
		: period_(period), dwell_(dwell) {}

	// set the given string
	void set(const MessageDisplayType mode, const char* string);

	// get the latest subsection of the message
	void get(char* string, const uint32_t current_time);

	// update local parameters
	void set_period(const uint64_t period) {period_ = period;};
	void set_dwell(const uint64_t dwell) {dwell_ = dwell;};
};


} // namespace msp_osd
