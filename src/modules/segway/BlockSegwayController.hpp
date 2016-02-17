#pragma once

#include <controllib/uorb/blocks.hpp>

using namespace control;

class BlockSegwayController : public control::BlockUorbEnabledAutopilot
{
public:
	BlockSegwayController() :
		BlockUorbEnabledAutopilot(NULL, "SEG"),
		th2v(this, "TH2V"),
		q2v(this, "Q2V"),
		_attPoll(),
		_timeStamp(0)
	{
		_attPoll.fd = _att.getHandle();
		_attPoll.events = POLLIN;
	}
	void update();
private:
	enum {CH_LEFT, CH_RIGHT};
	BlockPI th2v;
	BlockP q2v;
	struct pollfd _attPoll;
	uint64_t _timeStamp;
};

