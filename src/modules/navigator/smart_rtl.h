/**
 * @file rcrecover.h
 * RC recovery navigation mode
 */

#ifndef NAVIGATOR_SMART_RTL_H
#define NAVIGATOR_SMART_RTL_H

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#include "navigator_mode.h"
#include "mission_block.h"
#include "tracker.h"

class Navigator;

class SmartRTL : public MissionBlock
{
public:
	SmartRTL(Navigator *navigator, const char *name);

	~SmartRTL();

	void on_inactive();
	void on_activation();
	void on_active();

private:
	// Inits a setpoint
	void init_setpoint(position_setpoint_s &sp);

	// Updates the deadline from the current time and parameters.
	void update_deadline();

	// Returns the current distance/bearing to a setpoint
	float distance_to_setpoint(position_setpoint_s &sp);
	float bearing_to_setpoint(position_setpoint_s &sp);

	// Prints information about the specified setpoint
	void dump_setpoint(const char *name, position_setpoint_s &sp);

	// Advances the setpoint triplet by loading the next position from the return path.
	// Returns true if progress was made.
	bool advance_setpoint_triplet(position_setpoint_triplet_s *pos_sp_triplet);


	Tracker *_tracker;

	Tracker::path_finding_context_t current_return_context;
	Tracker::path_finding_context_t next_return_context;

	hrt_abstime deadline = HRT_ABSTIME_MAX; // This deadline makes sure that progress is made.
	bool land_after_deadline; // If true and the deadline is reached, land, otherwise, fall back to basic RTL.

	hrt_abstime next_log = 0;

	control::BlockParamFloat _param_fallback_delay;
	control::BlockParamFloat _param_land_delay;
};

#endif
