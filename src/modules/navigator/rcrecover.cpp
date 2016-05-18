/**
 * @file rcrecover.cpp
 * RC recovery navigation mode
 */

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>
#include <lib/mathlib/math/Limits.hpp>

#include <systemlib/mavlink_log.h>
#include <systemlib/err.h>
#include <geo/geo.h>

#include <uORB/uORB.h>
#include <uORB/topics/home_position.h>

#include "navigator.h"
#include "rcrecover.h"

#define DELAY_SIGMA	0.01f

RCRecover::RCRecover(Navigator *navigator, const char *name) :
	MissionBlock(navigator, name),
	_state(STATE_NONE),
	_start_lock(false),
	_param_rtl_delay(this, "RCRCVR_RTL_DELAY", false)
{
	// load initial params
	updateParams();

	// initial reset
	on_inactive();
}

RCRecover::~RCRecover()
{
}

void RCRecover::on_inactive()
{
	_navigator->get_tracker()->set_recent_path_tracking_enabled(true);

	// reset RCRecover state only if setpoint moved (why?)
	if (!_navigator->get_can_loiter_at_sp()) {
		_state = STATE_NONE;
	}
}

void RCRecover::on_activation()
{
	_navigator->get_tracker()->set_recent_path_tracking_enabled(false);

	/* reset starting point so we override what the triplet contained from the previous navigation state */
	_start_lock = false;
	set_current_position_item(&_mission_item);
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);

	if (_state == STATE_NONE) {
		// for safety reasons don't go into RCRecover if landed
		if (_navigator->get_land_detected()->landed) {
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "RC Recover: not available when landed");

			// otherwise, return along recent path

		} else {
			_state = STATE_RETURN;

			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "RC Recover: return along recent path");

			// in case there is no recent path, loiter at the current position
			loiter_lat = _navigator->get_global_position()->lat;
			loiter_lon = _navigator->get_global_position()->lon;
			loiter_alt = _navigator->get_global_position()->alt;
		}

	}

	update_mission_item();
}

void RCRecover::on_active()
{
	if (is_mission_item_reached()) {
		update_mission_item();
	}
}

void RCRecover::update_mission_item()
{
	// todo: directly write to setpoint triplet, as it is done in smart RTL

	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// make sure we have the latest params
	updateParams();

	if (!_start_lock) {
		set_previous_pos_setpoint();
	}

	_navigator->set_can_loiter_at_sp(false);

	//if (_state == STATE_LOITER)
	// todo: switch to RTL mode

	switch (_state) {
	case STATE_RETURN: {

			if (_navigator->get_tracker()->pop_recent_path(_mission_item.lat, _mission_item.lon, _mission_item.altitude)) {
				loiter_lat = _mission_item.lat;
				loiter_lon = _mission_item.lon;
				loiter_alt = _mission_item.altitude;
				TRACKER_DBG("tracker proposed %f %f %f", _mission_item.lat, _mission_item.lon, (double)_mission_item.altitude);

				_mission_item.altitude_is_relative = false;
				_mission_item.yaw = NAN;
				_mission_item.loiter_radius = _navigator->get_loiter_radius();
				_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
				_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
				_mission_item.time_inside = 0.0f;
				_mission_item.autocontinue = true;
				_mission_item.origin = ORIGIN_ONBOARD;

				_start_lock = true;
				break;
			}

			// Recent path is empty: fall through to loiter
			_state = STATE_LOITER;
		}

	case STATE_LOITER: {
			bool autortl = _param_rtl_delay.get() > -DELAY_SIGMA;

			_mission_item.lat = loiter_lat;
			_mission_item.lon = loiter_lon;
			_mission_item.altitude = loiter_alt;
			_mission_item.altitude_is_relative = false;
			_mission_item.yaw = _navigator->get_global_position()->yaw;
			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.nav_cmd = autortl ? NAV_CMD_LOITER_TIME_LIMIT : NAV_CMD_LOITER_UNLIMITED;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = math::max(_param_rtl_delay.get(), .0f);
			_mission_item.autocontinue = autortl;
			_mission_item.origin = ORIGIN_ONBOARD;

			_navigator->set_can_loiter_at_sp(true);

			if (autortl) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "RC Recover: loiter %.1fs", (double)_mission_item.time_inside);

			} else {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "RC Recover: completed, loiter");
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
