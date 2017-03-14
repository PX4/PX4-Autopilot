#pragma once

#include <px4_posix.h>
#include <controllib/blocks.hpp>
#include <controllib/uorb/blocks.hpp>

using control::BlockPI;
using control::BlockP;

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

	px4_pollfd_struct_t _attPoll;
	uint64_t _timeStamp;
};
