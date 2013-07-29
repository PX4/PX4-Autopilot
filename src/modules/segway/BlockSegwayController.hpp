#pragma once

#include <controllib/uorb/blocks.hpp>

using namespace control;

class BlockSegwayController : public control::BlockUorbEnabledAutopilot {
public:
	BlockSegwayController() :
		BlockUorbEnabledAutopilot(NULL,"SEG"),
		theta2spd(this, "THETA2SPD"),
		q2spd(this, "Q2SPD"),
		_attPoll(),
		_timeStamp(0)
	{
		_attPoll.fd = _att.getHandle();
		_attPoll.events = POLLIN;
	}
	void update();
private:
	enum {CH_LEFT, CH_RIGHT};
	BlockPI theta2spd;
	BlockP q2spd;
	struct pollfd _attPoll;
	uint64_t _timeStamp;
};

