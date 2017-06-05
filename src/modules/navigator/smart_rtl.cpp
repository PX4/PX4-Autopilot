/**
 * @file rcrecover.cpp
 * RC recovery navigation mode
 */

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>

#include <systemlib/mavlink_log.h>
#include <systemlib/err.h>
#include <geo/geo.h>

#include <uORB/uORB.h>
#include <uORB/topics/home_position.h>

#include "navigator.h"
#include "smart_rtl.h"

#define DELAY_SIGMA	0.01f

#define DEBUG_RTL

SmartRTL::SmartRTL(Navigator *navigator, const char *name) :
	MissionBlock(navigator, name),
	_param_fallback_delay(this, "RTL_FALLBCK_DLY", false),
	_param_land_delay(this, "RTL_LAND_DELAY", false)
{
	// load initial params
	updateParams();

	// initial reset
	on_inactive();
}

SmartRTL::~SmartRTL()
{
}

void SmartRTL::on_inactive()
{
	_tracker = NULL;
	deadline = HRT_ABSTIME_MAX;
}

void SmartRTL::on_activation()
{
	// For safety reasons don't go into RTL if landed
	if (_navigator->get_land_detected()->landed) {
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Return To Land: not available when landed");
		return;
	}

	mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Return To Land: return along recorded path");

	_tracker = _navigator->get_tracker();
	_tracker->dump_graph();
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->previous.valid = false;
	pos_sp_triplet->next.valid = false;

	// Init return path and setpoint triplet
	init_setpoint(pos_sp_triplet->current);
	pos_sp_triplet->current.valid = _tracker->init_return_path_global(
						current_return_context,
						pos_sp_triplet->current.lat,
						pos_sp_triplet->current.lon,
						pos_sp_triplet->current.alt);

	if (pos_sp_triplet->current.valid) {
		TRACKER_DBG("got return context");
	}

	_tracker->dump_graph();

	advance_setpoint_triplet(pos_sp_triplet);
	update_deadline();
}

void SmartRTL::on_active()
{
	// If the tracker fails, do the same as if the deadline was reached
	if (_tracker && _tracker->get_graph_fault()) {
		PX4_ERR("the flight graph became inconsistent and return path was lost");
		deadline = 0;
	}

	if (deadline <= hrt_absolute_time()) {

		_tracker = NULL;
		deadline = HRT_ABSTIME_MAX;

		if (land_after_deadline) {
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Return To Land: landing");
			TRACKER_DBG("proceed to land");

			// Perform landing
			set_land_item(&_mission_item, true);
			position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
			mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
			pos_sp_triplet->next.valid = false;
			_navigator->set_position_setpoint_triplet_updated();

		} else {
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Return To Land: fallback to legacy mode");
			TRACKER_DBG("fall back to legacy RTL");
			_navigator->set_rtl_variant(false);
		}

	} else if (_tracker && _tracker->is_context_close_to_head(current_return_context)) {
		if (advance_setpoint_triplet(_navigator->get_position_setpoint_triplet())) {
			// We made progress, update the deadline.
			update_deadline();

		} else {
			// If the return path is empty, discard the tracker so we don't make further unneccessary checks.
			_tracker = nullptr;
		}
	}

#ifdef DEBUG_RTL
	hrt_abstime now = hrt_absolute_time();

	if (now >= next_log) {
		next_log = now + (hrt_abstime)(1 * 1e6f);
		position_setpoint_s &sp = _navigator->get_position_setpoint_triplet()->next;
		PX4_WARN("next waypoint: %.0f°, %.2fm", (double)bearing_to_setpoint(sp), (double)distance_to_setpoint(sp));
	}

#endif
}

void SmartRTL::update_deadline()
{
	updateParams();

	land_after_deadline = _tracker->is_context_close_to_home(current_return_context);
	float delay = land_after_deadline ? _param_land_delay.get() : _param_fallback_delay.get();

	if (delay < 0) {
		deadline = HRT_ABSTIME_MAX;

	} else {
		deadline = hrt_absolute_time() + (hrt_abstime)(delay * 1e6f);
	}
}

void SmartRTL::init_setpoint(position_setpoint_s &sp)
{
	sp.valid = false;
	sp.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	sp.position_valid = false;
	sp.x = 0;
	sp.y = 0;
	sp.z = 0;
	sp.lat = 0;
	sp.lon = 0;
	sp.alt = 0;
	sp.acceptance_radius = 1;
	sp.disable_mc_yaw_control = true;
	sp.cruising_speed = _navigator->get_cruising_speed();
	sp.cruising_throttle = _navigator->get_cruising_throttle();
	sp.loiter_radius = _navigator->get_loiter_radius();
	sp.loiter_direction = 1;
}


float SmartRTL::distance_to_setpoint(position_setpoint_s &sp)
{
	float distance_xy, distance_z;
	get_distance_to_point_global_wgs84(
		_navigator->get_global_position()->lat,
		_navigator->get_global_position()->lon,
		_navigator->get_global_position()->alt,
		sp.lat, sp.lon, sp.alt,
		&distance_xy, &distance_z);
	return sqrt(distance_xy * distance_xy + distance_z * distance_z);
}

float SmartRTL::bearing_to_setpoint(position_setpoint_s &sp)
{
	float bearing = get_bearing_to_next_waypoint(
				_navigator->get_global_position()->lat,
				_navigator->get_global_position()->lon,
				sp.lat, sp.lon) * 180.f / (float)M_PI;

	if (bearing <= 0) {
		bearing += 360;
	}

	return bearing;
}


void SmartRTL::dump_setpoint(const char *name, position_setpoint_s &sp)
{
	TRACKER_DBG("%s setpoint is lat %f lon %f alt %f, distance %f, %s", name,
		    sp.lat, sp.lon, (double)sp.alt,
		    (double)distance_to_setpoint(sp), sp.valid ? "valid" : "invalid");
}


bool SmartRTL::advance_setpoint_triplet(position_setpoint_triplet_s *pos_sp_triplet)
{
	// Shift setpoints if possible
	if (pos_sp_triplet->next.valid) {
		if (pos_sp_triplet->current.valid) {
			pos_sp_triplet->previous = pos_sp_triplet->current;
		}

		pos_sp_triplet->current = pos_sp_triplet->next;
		current_return_context = next_return_context;

	} else {
		next_return_context = current_return_context;
	}

	// Load next setpoint
	init_setpoint(pos_sp_triplet->next);
	pos_sp_triplet->next.valid = _tracker->advance_return_path_global(next_return_context,
				     pos_sp_triplet->next.lat,
				     pos_sp_triplet->next.lon,
				     pos_sp_triplet->next.alt);

	// Apply updated setpoint triplet
	_navigator->set_position_setpoint_triplet_updated();

	//dump_setpoint("previous", pos_sp_triplet->previous);
	//dump_setpoint("current", pos_sp_triplet->current);
	//dump_setpoint("next", pos_sp_triplet->next);

	return pos_sp_triplet->next.valid && !_tracker->is_same_pos(current_return_context, next_return_context);
}
