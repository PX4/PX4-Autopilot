#pragma once

#include <controllib/uorb/blocks.hpp>

using namespace control;

class BlockSegwayController : public control::BlockUorbEnabledAutopilot {
public:
	BlockSegwayController() :
		BlockUorbEnabledAutopilot(NULL,"SEG"),
		phi2spd(this, "PHI2SPD")
	{
	}
	void update();
private:
	BlockP phi2spd;
};

