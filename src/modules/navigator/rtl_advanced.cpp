/**
 * @file rcrecover.cpp
 * RC recovery navigation mode
 */

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>
#include <algorithm>

#include <systemlib/mavlink_log.h>
#include <systemlib/err.h>
#include <geo/geo.h>

#include <uORB/uORB.h>
#include <uORB/topics/home_position.h>

#include "navigator.h"
#include "rtl_advanced.h"

#define DELAY_SIGMA	0.01f

RTLAdvanced::RTLAdvanced(Navigator *navigator, const char *name) :
	MissionBlock(navigator, name),
	_state(STATE_NONE),
	_start_lock(false),
	_param_rtlb_delay(this, "RTLA_RTLB_DELAY", false)
{
	// load initial params
	updateParams();
	
	// initial reset
	on_inactive();
}

RTLAdvanced::~RTLAdvanced()
{
}

void RTLAdvanced::on_inactive()
{
	// reset RTLAdvanced state only if setpoint moved (why?)
	if (!_navigator->get_can_loiter_at_sp())
		_state = STATE_NONE;
}

void RTLAdvanced::on_activation()
{
	/* reset starting point so we override what the triplet contained from the previous navigation state */
	_start_lock = false;
	set_current_position_item(&_mission_item);
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);

	if (_state == STATE_NONE) {
		// for safety reasons don't go into RTL if landed
		if (_navigator->get_land_detected()->landed) {
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Return To Land: not available when landed");

		// otherwise, return along shortest path
		} else {
			_state = STATE_RETURN;
			
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Return To Land: return along shortest path");
			
			// in case there is no path, loiter at the current position
			loiter_lat = _navigator->get_global_position()->lat;
			loiter_lon = _navigator->get_global_position()->lon;
			loiter_alt = _navigator->get_global_position()->alt;
		}

	}

	update_mission_item();
}

void RTLAdvanced::on_active()
{
	if (is_mission_item_reached())
		update_mission_item();
}

void RTLAdvanced::update_mission_item()
{
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// make sure we have the latest params
	updateParams();

	if (!_start_lock)
		set_previous_pos_setpoint();

	_navigator->set_can_loiter_at_sp(false);
	
	//if (_state == STATE_LOITER)
		// todo: switch to RTL Basic (fallback) mode

	switch (_state) {
	case STATE_RETURN: {

		// This is temporary until we have a nicer way to follow a fine-grained path.
		// It ensures that we look ahead far enough to find the next point that is not aleady in the acceptance radius.
		const int PREFETCH = 6;
		double lat[PREFETCH];
		double lon[PREFETCH];
		float alt[PREFETCH];
		int prefetch = _navigator->get_tracker().get_path_to_home(lat, lon, alt, PREFETCH);
		for (int i = 0; i < prefetch - 1; i++) {
			if (fabs(lat[i] - _mission_item.lat) < DBL_EPSILON && fabs(lon[i] - _mission_item.lon) < DBL_EPSILON && fabs(alt[i] - _mission_item.altitude) < FLT_EPSILON) {
				lon[0] = lon[i + 1];
				lat[0] = lat[i + 1];
				alt[0] = alt[i + 1];
				break;
			}
		}

		
		if (prefetch) {
			loiter_lat = _mission_item.lat = lat[0];
			loiter_lon = _mission_item.lon = lon[0];
			loiter_alt = _mission_item.altitude = alt[0];
			TRACKER_DBG("tracker proposed %lf %lf %f", _mission_item.lat, _mission_item.lon, _mission_item.altitude);
			
			_mission_item.altitude_is_relative = false;
			_mission_item.yaw = NAN;
			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.loiter_direction = 1;
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.pitch_min = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;
			
			_start_lock = true;
			break;
		}
		
		// Recent path is empty: fall through to loiter
		_state = STATE_LOITER;
	}

	case STATE_LOITER: {
		bool fallbackRTL = _param_rtlb_delay.get() > -DELAY_SIGMA;

		_mission_item.lat = loiter_lat;
		_mission_item.lon = loiter_lon;
		_mission_item.altitude = loiter_alt;
		_mission_item.altitude_is_relative = false;
		_mission_item.yaw = _navigator->get_global_position()->yaw;
		_mission_item.loiter_radius = _navigator->get_loiter_radius();
		_mission_item.loiter_direction = 1;
		_mission_item.nav_cmd = fallbackRTL ? NAV_CMD_LOITER_TIME_LIMIT : NAV_CMD_LOITER_UNLIMITED;
		_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
		_mission_item.time_inside = std::max(_param_rtlb_delay.get(), .0f);
		_mission_item.pitch_min = 0.0f;
		_mission_item.autocontinue = fallbackRTL;
		_mission_item.origin = ORIGIN_ONBOARD;

		_navigator->set_can_loiter_at_sp(true);

		if (fallbackRTL) {
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Return To Land: loiter %.1fs", (double)_mission_item.time_inside);
		} else {
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Return To Land: completed, loiter");
		}
			
		break;
	}

	default:
		break;
	}

	reset_mission_item_reached();

	/* convert mission item to current position setpoint and make it valid */
	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();
}
