/**
 * @file rcrecover.h
 * RC recovery navigation mode
 */

#ifndef NAVIGATOR_RTL_ADVANCED_H
#define NAVIGATOR_RTL_ADVANCED_H

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#include "navigator_mode.h"
#include "mission_block.h"

class Navigator;

class RTLAdvanced : public MissionBlock
{
public:
	RTLAdvanced(Navigator *navigator, const char *name);

	~RTLAdvanced();

	void on_inactive();
	void on_activation();
	void on_active();

private:
	// Set the mission item
	void update_mission_item();

	enum {
		STATE_NONE = 0,
		STATE_RETURN,
		STATE_LOITER
	} _state;

	bool _start_lock;
	
	double loiter_lat;
	double loiter_lon;
	float loiter_alt;
	
	control::BlockParamFloat _param_rtlb_delay;
};

#endif
